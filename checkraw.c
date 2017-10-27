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

uint32_t frameCounter;


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

void decodeline(struct gps_device_t *session, char * line) {
    const int LEN = 255;
    uint8_t bin[LEN];
    uint32_t stdid, extid, pgn;
    uint8_t prio, saddr, daddr;

    int binlen = 0;
    const char *hexchar = "0123456789abcdef";

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

    gpsd_report(session->context->debug, LOG_INF, "s: %u, d: %u, p: %u - %u\n", saddr, daddr, prio, pgn); 
}

int main(int argc, char * argv[]) {

    uint8_t opts = 0;
    char c;
    char n2kSequence[255];
    int loglevel = 5;

    struct gps_device_t session;
    struct gps_context_t context;
    
    opterr = 0;
    n2kSequence[0] = '\0';

    while ((c = getopt(argc, argv, "b:")) != -1) {

        switch (c) {
        case 'b': 
            strncpy(n2kSequence, optarg, 255);
            printf("n2k %s\n", n2kSequence);
            opts++;
            break;
        case 'd': 
            loglevel = atoi(optarg);
            printf("n2k loglevel %d\n", loglevel);
            opts++;
            break;
        default:
            abort ();
        }
    }

    context.debug = loglevel;
    
    session.context = &context;

    gpsd_init(&session, &context, "pseudo");
    
    decodeline(&session, n2kSequence);

    return 0;
}
