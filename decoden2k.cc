/*
  mini prg to write a command to write and read the vynmea chips
 */
#include <stdio.h>      // standard input / output functions
#include <stdint.h>
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions

#include "gps.h"
#include "gpsd.h"
#include "driver_vyspi.h"
#include "bits.h"

#define MAX_PACKET_LENGTH	516	/* 7 + 506 + 3 */

typedef enum {
    complete = 0, 
    error = 1,  
    incomplete = 2
} nmea2000_state;

struct nmea2000_packet_t {
  uint16_t idx;
  uint32_t ptr;
  uint32_t fast_packet_len;
  uint8_t outbuffer[MAX_PACKET_LENGTH*2 + 1];
  uint32_t outbuflen;
  uint32_t pgn;
  uint32_t state;
  uint32_t frame_count;
  uint32_t packet_count;
  uint32_t error_count;
  uint32_t cancel_count;
  uint32_t packet_transfer_count;
};

struct nmea2000_packet_t nmea2000_packet;

uint32_t packetCounter;
uint32_t packetFastCounter;
uint32_t frameCounter;

uint32_t n2k_fixed_fast_list[] = {
    126464,
    126996,
    127489,
    128275,  // distance log as provided by DST800
    129540,
    129542,
    129793,
    129794,
    129798,
    129802,
    129809,
    129810,
    130935, // observed in AIS
    130842, // observed in AIS
    262161,
    262384,
    262386,  // observed when testing ROT with actisense

    0
};

void gpsd_throttled_report(const int errlevel UNUSED, const char * buf UNUSED) {}
void gpsd_report(const int debuglevel, const int errlevel, const char *fmt, ...)
/* our version of the logger */
{
    if(debuglevel < errlevel)
      return;

    va_list ap;
    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, LOG_ERROR - 1, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}

void gpsd_external_report(const int debuglevel UNUSED, const int errlevel UNUSED,
			  const char *fmt UNUSED, ...) {
}

ssize_t gpsd_write(struct gps_device_t *session,
		   const char *buf,
		   const size_t len)
/* pass low-level data to devices, echoing it to the log window */
{
    return gpsd_serial_write(session, buf, len);
}


void nmea2000_parsemsg(struct gps_device_t *session,
                       uint32_t pgn, uint8_t * bin, uint16_t binlen,
                       struct nmea2000_packet_t * packet) {

    uint8_t l2;
    uint8_t fast = 0;
    uint16_t cnt = 0;

    frameCounter++;

    cnt = 0;
    while(n2k_fixed_fast_list[cnt] > 0) {

        if(pgn == n2k_fixed_fast_list[cnt]) {
            fast = 1;
            break;
        }

        cnt++;
    }

    gpsd_report(session->context->debug, LOG_RAW,
                "Parse transmission pgn= %u with len= %u\n", pgn, binlen);
    
    // is this a fast transmission (list of pgn from gpsd)
    if(fast && (binlen == 9)) {

      if((bin[0] & 0x1f) == 0) {
          
          // start of fast transmission
          
        // count cancled other fast transfers
        if((packet->pgn == 0) && (packet->state == incomplete))
            packet->cancel_count++;
        packet->fast_packet_len = bin[1];
        packet->idx = bin[0];
        packet->ptr = 0;
        packet->idx += 1;
        packet->pgn = 0; // use this sign for whole fast trans being done
        packet->state = incomplete;

        for (l2=2;l2<8;l2++) {
          packet->outbuffer[packet->ptr++]= bin[l2];
        }
        gpsd_report(session->context->debug, LOG_SPIN,
                    "Start of fast transmission pgn= %u with len %u\n", pgn, packet->fast_packet_len);
        
      } else if(bin[0] == packet->idx) {
          packet->state = incomplete;
          for (l2=1;l2<8;l2++) {
              if (packet->fast_packet_len > packet->ptr) {
                  packet->outbuffer[packet->ptr++] = bin[l2];
              }
          }
          if (packet->ptr == packet->fast_packet_len) {
              packet->outbuflen = packet->fast_packet_len;
              packet->fast_packet_len = 0;
              packet->pgn = pgn;
              packet->state = complete;
              packet->packet_count++;
              gpsd_report(session->context->debug, LOG_IO,
                          "Complete fast transmission pgn= %u with length %u\n",
                          pgn, packet->outbuflen);
          } else {
              packet->idx += 1;
          }
      } else {
        // error
        // packetError = packet->idx;
          gpsd_report(session->context->debug, LOG_ERROR,
                      "Error N2K cnt = %d, idx = %u, d[0] = %u\n", 
                      packet->packet_count, packet->idx, bin[0]);
        packet->idx = 0;
        packet->fast_packet_len = 0;
        packet->state = error;
        packet->error_count++;
      }
    } else {
      // single transmission
        gpsd_report(session->context->debug, LOG_IO,
                    "Single transmission pgn= %u\n", pgn);

        // count cancled fast transfers
        if((packet->pgn == 0) && (packet->state == incomplete))
            packet->cancel_count++;
      packet->ptr = 0;
      for (l2=0;l2<binlen;l2++) {
        packet->outbuffer[packet->ptr++]= bin[l2];
      }
      packet->idx = 0;
      packet->outbuflen = binlen;
      packet->fast_packet_len = 0;
      packet->pgn = pgn;
      packet->state = complete; 
      packet->packet_count++;
    }
    packet->frame_count++;
}

void decodeline(struct gps_device_t *session, int reportformat, char * line, size_t len, int offset) {
    
    const int LEN = 255;
    uint8_t bin[LEN];
    uint8_t * startbin = 0;
    uint32_t pgn;
    uint32_t stdid, extid;
    uint8_t prio, saddr, daddr;

    struct PGN * pgns;
    int binlen = 0;
    int messagetype = 0;
    const char *hexchar = "0123456789abcdef";
    char ts[32];

    memset(ts, 0, 32);

    // reportformat 0: raw with {stdid}{extid}{len}{}{}{}{bin}
    // reportformat 1: {pgn}{bin}
    // reportformat 2: [time]: 0,1,{pgn},0,0,{len},{bin}

    if(reportformat == 0) {

        binlen = gpsd_hexpack(line, bin, LEN);
    
        stdid = getleu32(bin, 0);
        extid = getleu32(bin, 4);

        saddr = (uint8_t)(extid & 0xff);
        pgn = (extid >> 8) & 0x1ffff;
        prio = (uint8_t)((extid >> 26) & 0x7);

        // PDU1 messages with bits 0x0000ff000 between 0x00 to 0xef and make use of dest and source address
        // PDU2 messages from 0xf0 to 0xff are intended to be broadcasts
        if (((pgn & 0xff00) >> 8) < 0xf0) {
            daddr  = (uint8_t)(pgn & 0xff);
            pgn  = pgn & 0x01ff00;
        } else {
            daddr = (uint8_t)0xff;
        }
        
        
    } else if(reportformat == 1) {
        
        binlen = gpsd_hexpack(line, bin, LEN);
        pgn = getleu32(bin, 0);
        startbin = bin;
        messagetype = 2;
        
    } else if(reportformat == 2) {

        char *str1, *str2, *token, *subtoken;
        char *saveptr1, *saveptr2;
        int j;
        int tkno= 0;
        int leninline = 0;

        for (j = 1, str1 = line; ; j++, str1 = NULL) {
            token = strtok_r(str1, ":", &saveptr1);
            if (token == NULL)
                break;
            gpsd_report(session->context->debug, LOG_RAW,
                        "%d: %s\n\n", j, token);

            tkno = 0;
            for (str2 = token; ; str2 = NULL) {
                subtoken = strtok_r(str2, ",", &saveptr2);
                if (subtoken == NULL)
                    break;
                gpsd_report(session->context->debug, LOG_RAW,
                            " --> %s\n", subtoken);
                if(j == 1 && tkno == 0) {
                    memset(ts, 0, 32);
                    strcpy(ts, subtoken);
                }
                if(j == 2) {
                    if(tkno == 1) {
                        messagetype = atoi(subtoken);
                    }
                    switch(messagetype) {
                    case 2: {
                        char buffer[255];
                        // n2k
                        if(tkno == 2) {
                            pgn = atoi(subtoken);
                        }
                        if(tkno == 5) {
                            leninline = atoi(subtoken);
                        }
                        if(tkno == 6) {
                            char * end = subtoken;
                            char scbuf[512];
                            int i = 0, j = 0;
                            memset(scbuf, 0, 512);
                        
                            while(strchr(hexchar, *end)) end++;
                            strncpy(buffer, subtoken, end - subtoken); buffer[end - subtoken] = 0;
                        
                            binlen = gpsd_hexpack(buffer, bin, LEN);
                            startbin = bin;
                        
                            gpsd_report(session->context->debug, LOG_SPIN,
                                        " ---> %s with %d len (%d)\n", buffer, binlen, leninline);
            
                            for (i = 0; j < 510 && i < binlen; i++) {
                                scbuf[j++] = hexchar[(bin[i] & 0xf0) >> 4];
                                scbuf[j++] = hexchar[bin[i] & 0x0f];
                            }
                            gpsd_report(session->context->debug, LOG_SPIN,
                                        " ---> %s with %d len (%d)\n", scbuf, binlen, leninline);
                        
                        }
                        break;
                    }

                    default:
                        break;
                    }
                }
                tkno++;
            }
        }

    }


    if(messagetype == 2) {
        
        // NMEA 2000 is message type 2
        
        uint32_t pgnb = getleu32(startbin, 0);
        
        if(offset == 4) {
            
            gpsd_report(session->context->debug, LOG_INF,
                        "%s pgn: %u, pgn bin: %u\n", ts, pgn, pgnb);
            
        } else {
            
            prio = startbin[4];
            saddr = startbin[5];
            daddr = startbin[6];
            gpsd_report(session->context->debug, LOG_INF,
                        "%s pgn: %u, pgn bin: %u, prio: %u, src: %u, dest: %u\n", ts, pgn, pgnb, prio, saddr, daddr);
        }
        
        startbin += offset;
        binlen -= offset;

        /*
          char scbuf[512];
          int i = 0, j = 0;
          memset(scbuf, 0, 512);
          
          for (i = 0; j < 510 && i < binlen; i++) {
          scbuf[j++] = hexchar[(startbin[i] & 0xf0) >> 4];
          scbuf[j++] = hexchar[startbin[i] & 0x0f];
          }
          gpsd_report(session->context->debug, LOG_SPIN,
          "Parsing now %s\n", scbuf);
        */              
        nmea2000_parsemsg(session, pgn, startbin, binlen, &nmea2000_packet);

        if(nmea2000_packet.state == complete) {
            gps_mask_t mask = 0;
            pgns = vyspi_find_pgn(pgn);
            if(pgns)
                mask = (pgns->func)(nmea2000_packet.outbuffer, nmea2000_packet.outbuflen, pgns, session);

/*
            if(mask && AIS_SET) {
                struct ais_t *ais;
                ais =  &session->gpsdata.ais;

                if(ais->type == 18 && ais->mmsi == 265733160)
                    printf("%f,%f\n",
                                ais->type18.lon / AIS_LATLON_DIV, 
                                ais->type18.lat / AIS_LATLON_DIV); 
            }
*/
        }
    }
}


int main(int argc, char * argv[]) {

    int offset = 4;
    int protocol = 0;

    uint8_t opts = 0;

    char n2kSequence[255];
    
    FILE * fp;
    char filename[255];
    char * line = NULL;
    char buffer[255];
    size_t len = 0;
    ssize_t read;
    
    char c = 0;
    int loglevel = 5;
    int version = 1;

    opterr = 0;
    filename[0] = '\0';
    n2kSequence[0] = '\0';

    while ((c = getopt(argc, argv, "b:f:d:v:P:")) != -1) {

        switch (c) {
        case 'b': 
            strncpy(n2kSequence, optarg, 255);
            printf("n2k %s\n", n2kSequence);
            opts++;
            break;
        case 'f': 
            strncpy(filename, optarg, 255);
            printf("n2k filename %s\n", filename);
            opts++;
            break;
        case 'd': 
            loglevel = atoi(optarg);
            printf("n2k loglevel %d\n", loglevel);
            opts++;
            break;
        case 'P': 
            protocol = atoi(optarg);
            printf("frm protocol version %d\n", protocol);
            opts++;
            break;
        case 'v': 
            version = atoi(optarg);
            printf("log version %d\n", version);
            opts++;
            break;
        default:
            abort ();
        }
    }

    struct gps_device_t session;
    struct gps_context_t context;
    context.debug = loglevel;
    
    session.context = &context;

    gpsd_init(&session, &context, "pseudo");

    if(protocol)
        offset = 7;
    
    if(filename[0]) {
        fp = fopen(filename, "r");
        if (fp == NULL) {
            gpsd_report(session.context->debug, LOG_ERROR,
                        "file %s not found\n", filename);
            exit(1);
        }

        while ((read = getline(&line, &len, fp)) != -1) {
            gpsd_report(session.context->debug, LOG_RAW,
                        "Retrieved line %s of length %zu\n", line, read);

            if(version == 1) {
                char * end = line;
                while(strchr("0123456789abcdef", *end)) end++;
                strncpy(buffer, line, end - line); buffer[end - line] = 0;
                gpsd_report(session.context->debug, LOG_SPIN,
                            "Retrieved line %s of length %ld\n", buffer, end - line);
                decodeline(&session, version, buffer, end - line, offset);
            } else {
                decodeline(&session, version, line, read, offset);
            }
        }

        fclose(fp);
        if (line)
            free(line);
        
    } else if(n2kSequence[0]) {
        decodeline(&session, version, n2kSequence, strlen(n2kSequence), offset);
    }


/*
    int width=16; //8;
    unsigned char line[9] = "\0";
    int toscreen = 1;
    
    int pos = 0;
    unsigned char ch = ' ';

    for (pos=0; pos < msgll; pos++) {
        ch = bin[pos];
        line[pos%width] = ((ch > 31) && (ch < 128)) ? ch : '.';
        line[(pos%width)+1] = '\0';
        if (toscreen) printf("%02x ", ch);
        if (pos%width > (width-2)) {
            if (toscreen) printf("  %s\n", line);
        }
    }
    if (pos%width != 0) {
        int p;
        for (p=0; p<width-(pos%width); p++) printf("   ");	//padding
        if (toscreen) printf("  %s\n\n", line);
    }
*/
    return 0;

}
