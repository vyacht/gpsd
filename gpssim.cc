
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdlib.h>
#include <ctype.h>

#include <netdb.h>

#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include "gpsd.h"
#include "frame.h"
#include "utils.h"
#include "pseudon2k.h"

#include "driver_vyspi.h"

#include "bits.h"
#include "nmea2000.h"

#include <assert.h>

#define PORT 2000
uint8_t protocol_version = 1;

 // will be modified based on protocol version
uint8_t n2k_payload_offset = 4;

uint8_t report_format = 0;

extern struct nmea2000_packet nmea2000_packets[32 + 1];

int sock, status, sinlen;
struct sockaddr_in sock_in;

int port = 2000;
char * bcast = NULL;

char usb_dev[255];
int usb_sock = -1;

const char *hexchar = "0123456789abcdef";

void init(int USB);
int earth_position_from_distance(double lat1, double lon1,
                                 double course, double distance,
                                 double * lat, double * lon);

void send_frame(struct gps_device_t * session, uint8_t * bufb, uint8_t frmType,
                uint16_t written, uint32_t pgn, uint8_t prio, uint8_t src, uint8_t dest);

int make_frame(uint8_t * frm, int len,
               uint8_t * bufb, uint8_t frmType,
               uint16_t written, uint32_t pgn, uint8_t prio, uint8_t src, uint8_t dest);

static void print_data(struct gps_context_t *context,
                       unsigned char *buffer, int len);


uint8_t * parse_report2_line(struct gps_device_t *session, char * line,
                             int * binlen, uint8_t * binbuf, int binbuflen, int * messagetype);

void decode_line(struct gps_device_t *session, char * line);
void send_frame_out(struct gps_device_t * session, uint8_t * frm, int frmlen);
void decode_raw_file(struct gps_device_t * session, char * filename);
int nmea0183_clean(char * dest, int destlen, char * src, int srclen);
void decode_file(struct gps_device_t * session, char * filename);

void init(int USB) {

  struct termios tty;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( USB, &tty ) != 0 ) {
    printf("Error %d from tcgetattr %s\n", errno, strerror(errno));
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B4800);
  cfsetispeed (&tty, (speed_t)B4800);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( USB, TCIFLUSH );
  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
    printf("Error %d from tcgetattr %s\n", errno, strerror(errno));
  }
}

/* kinetic state vector */
struct ksv_t {
  double speed;        // Speed in m/s
  double latitude;     // Latitude in degrees
  double longitude;    // Longitude in degrees
};

/*
  Calculates a new position in degrees with course / distance from
  an initial point lat1/lon1.

  Formula from <http://williams.best.vwh.net/avform.htm#Rhumb>
  Initial point cannot be a pole, but GPS doesn't work at high.
  latitudes anyway so it would be OK to fail there.
  Seems to assume a spherical Earth, which means it's going
  to have a slight inaccuracy rising towards the poles.
  The if/then avoids 0/0 indeterminacies on E-W courses.
*/
int earth_position_from_distance(double lat1, double lon1,
				 double course, double distance,
				 double * lat, double * lon) {

  double tc    = DEG_2_RAD * course;
  double d     = DEG_2_RAD * (distance/60.0); // distance from nm to rad, 1nm = 1min deg
  double lat1R = DEG_2_RAD * lat1;
  double lon1R = DEG_2_RAD * lon1;


  // printf("tc= %f, d= %f, lat1R= %f, lon1R= %f\n", tc, d, lat1R, lon1R);

  *lat = lat1R + d * cos(tc);

  double q = 0.0;

  if (fabs(*lat) > GPS_PI/2.0) {
    printf("error: distance %f > %f too large. You can't go this far along this rhumb line!\n",
	   fabs(*lat), GPS_PI/2.0);
    return 1;
  }
  if (fabs(*lat - lat1R) < sqrt(1e-15)) {
    q = cos(lat1R);
  } else {
    double t1 = (tan(*lat/2.0 + GPS_PI/4.0));
    double t2 = (tan(lat1R/2.0 + GPS_PI/4.0));
    double dphi = log(t1/t2);
    q = (*lat-lat1R)/dphi;
    // printf("d = %f, %f, %f\n", d, t1, t2);
  }

  double dlon = -d * sin(tc) / q;

  *lon = RAD_2_DEG * (fmod(lon1R + dlon + GPS_PI, 2.0 * GPS_PI) - GPS_PI);
  // printf("%f, %f\n", *lat, *lon);
  *lat = RAD_2_DEG * (*lat);

  return 0;
}

ssize_t gpsd_write(struct gps_device_t *session,
                   const uint8_t *buf,
                   const size_t len)
/* pass low-level data to devices straight through */
{
    return gpsd_serial_write(session, buf, len);
}

void gpsd_throttled_report(const int errlevel UNUSED, const char * buf UNUSED) {}
void gpsd_report(const int debuglevel, const int errlevel,
                 const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, 0, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}

void gpsd_external_report(const int debuglevel UNUSED, const int errlevel UNUSED,
                          const char *fmt UNUSED, ...) {
}

struct gps_dt_t {
  int PRN;
  int elevation;
  int azimuth;
  int ss;
};

struct gps_dt_t gd[] = {
            {3,41,50,46},
			{7,60,317,49},
			{8,34,315,45},
			{11,35,148,45},
			{13,38,213,45},
			{16,27,68,43},
			{19,72,76,50},
			{23,9,182,37},
			{24,6,166,35},
			{28,9,271,36}};

static int wrap_and_send_0183(struct gps_device_t *session, char * bufp, uint8_t frmType,
                              int sock, struct sockaddr_in * sock_in, int sinlen) {

    char * pch = strtok(bufp, "\r\n");
    while(pch != NULL) {

        char buf[1024];
        sprintf(buf, "%s\r\n", pch);
        printf("%s", buf);

        if(usb_dev[0] != '\0') {

            gpsd_write(session, (uint8_t *)buf, strlen(buf));

        } else {

            uint8_t frm[255];
            ssize_t n_written;


            int frmlen = frm_toHDLC8(frm, 255, frmType, protocol_version, (uint8_t *)buf, strlen(buf));

            if((n_written = sendto(sock, frm, frmlen, 0, (struct sockaddr *)sock_in, sinlen)) < 0) {
//          if((n_written = sendto(sock, bufp, strlen(buf), 0, (struct sockaddr *)sock_in, sinlen)) < 0) {
                // if(send(sock, buf, strlen(buf), 0) < 0) {
                printf("sendto Status = %s\n", strerror(errno));
            }
            printf("Wrote %ld bytes to udp\n", n_written);
            print_data(session->context, frm, frmlen);
        }

        pch = strtok(NULL, "\r\n");
    }

    return 0;

}

int make_frame(uint8_t * frm, int len,
                uint8_t * bufb, uint8_t frmType,
                uint16_t written, uint32_t pgn, uint8_t prio, uint8_t srcid, uint8_t destid) {

    uint16_t srclen = written;

    if(frmType == FRM_TYPE_NMEA2000) {

        set8leu32(bufb, pgn, 0);

        if(protocol_version > 0) {
            assert(n2k_payload_offset == 7);
            bufb[4] = prio; // prio
            bufb[5] = srcid; // src
            bufb[6] = destid; // dest
        } else
            assert(n2k_payload_offset == 4);

        srclen += n2k_payload_offset;
    }

    return frm_toHDLC8(frm, len, frmType, protocol_version, bufb, srclen);

}

void send_frame_out(struct gps_device_t * session, uint8_t * frm, int frmlen) {

    ssize_t n_written;

    if(usb_sock < 0)  {
        if((n_written = sendto(sock, frm, frmlen, 0, (struct sockaddr *)&sock_in, sinlen)) < 0) {
            // if(send(sock, buf, strlen(buf), 0) < 0) {
            printf("sendto Status = %s\n", strerror(errno));
        }
        gpsd_report(session->context->debug, LOG_IO,
                    "Wrote %ld bytes to udp://%s:%d\n",
                    n_written, bcast, port);
        print_data(session->context, frm, frmlen);

    } else {
        n_written = write( usb_sock, frm, frmlen );
        printf("Wrote %ld bytes to %s\n", n_written, usb_dev);
        char msgbuf[4096];

        const char *hd = gpsd_hexdump(msgbuf, sizeof(msgbuf),
                                      (char *)frm, frmlen);
        (void)strlcat((char *)hd, "\n", sizeof(msgbuf));
        gpsd_report(session->context->debug, LOG_DATA, "%s\n", hd);
    }
}

void send_frame(struct gps_device_t * session, uint8_t * bufb, uint8_t frmType,
                uint16_t written, uint32_t pgn, uint8_t prio, uint8_t src, uint8_t dest) {

    uint8_t frm[255];
    char scbuf[4096];

    const char *hexchar = "0123456789abcdef";
    int i = 0, j = 0;
    memset(scbuf, 0, 4096);

    for (i = 0; j < 4096 && i < written; i++) {
        scbuf[j++] = hexchar[(bufb[i] & 0xf0) >> 4];
        scbuf[j++] = hexchar[bufb[i] & 0x0f];
    }
    gpsd_report(session->context->debug, LOG_SPIN,
                " ---> pgn %u %s with %u len\n", pgn, scbuf, written);

    int frmlen = make_frame(frm, 255, bufb, frmType, written, pgn, prio, src, dest);

    send_frame_out(session, frm, frmlen);

}

/*
  binlen is length of extract with data
  bin is the buffer to store data in where return value is a pointer to that start of its data
  binbuflen is buffer len
 */
uint8_t * parse_report2_line(struct gps_device_t *session, char * line,
                             int * binlen, uint8_t * binbuf, int binbuflen, int * messagetype) {

    char *str1, *str2, *token, *subtoken;
    char *saveptr1, *saveptr2;
    int j;
    int tkno= 0;
    int leninline = 0;
    uint8_t * startbin = NULL;
    uint32_t pgn;
    char ts[32];

    memset(ts, 0, 32);

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
                    *messagetype = atoi(subtoken);
                }
                switch(*messagetype) {
                case 2: {
                    char buffer[255];
                    // n2k
                    if(tkno == 2) {
                        pgn = atoi(subtoken);
                        gpsd_report(session->context->debug, LOG_RAW,
                                    "%s pgn: %u\n", ts, pgn);
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

                        *binlen = gpsd_hexpack(buffer, binbuf, binbuflen);
                        startbin = binbuf;

                        gpsd_report(session->context->debug, LOG_SPIN,
                                    " ---> %s with %d len (%d)\n", buffer, *binlen, leninline);

                        for (i = 0; j < 510 && i < *binlen; i++) {
                            scbuf[j++] = hexchar[(binbuf[i] & 0xf0) >> 4];
                            scbuf[j++] = hexchar[binbuf[i] & 0x0f];
                        }
                        gpsd_report(session->context->debug, LOG_SPIN,
                                    " ---> %s with %d len (%d)\n", scbuf, *binlen, leninline);

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

    return startbin;
}

void decode_line(struct gps_device_t *session, char * line) {

    const int LEN = 255;
    uint8_t bin[LEN];
    uint32_t pgn;
    uint32_t extid;
    uint8_t prio, saddr, daddr;

    struct PGN * pgns;
    int messagetype = 0;

    int binlen = 0;
    uint8_t * startbin = 0;

    struct nmea2000_packet * packet = NULL;
    struct nmea2000_raw_frame frame;
    frame.extid = 0;
    frame.len = 0;

    uint8_t * outbuffer = NULL;
    uint16_t outbuflen = 0;

    // reportformat 0: super raw from ttyATH0 - not handled here
    // reportformat 1: raw with {stdid}{extid}{len}{}{}{}{bin}
    // reportformat 2: {pgn}{bin}
    // reportformat 3: [time]: 0,1,{pgn},0,0,{len},{bin}


    if(report_format == 1) {

        binlen = gpsd_hexpack(line, bin, LEN);

        extid = getleu32(bin, 4);

        frame.extid = extid;
        frame.len = 8;
        memcpy(frame.data, bin + 12, 8);

        messagetype = 2;

        gpsd_report(session->context->debug, LOG_SPIN,
                    "Decoding line with report format %d\n", report_format);

    } else {

        if(report_format == 2) {

            binlen = gpsd_hexpack(line, bin, LEN);
            if(!memcmp(bin, "stat", 4)) {
                messagetype = 0;
            } else {
                startbin = bin;
                messagetype = 2;
            }

        } else if(report_format == 3) {

            startbin = parse_report2_line(session, line,
                                          &binlen, bin, LEN, &messagetype);
        }

        if(messagetype == 2) {

            // NMEA 2000 is message type 2

            pgn = getleu32(startbin, 0);

            if(n2k_payload_offset > 4) {
                prio = startbin[4];
                saddr = startbin[5];
                daddr = startbin[6];
                gpsd_report(session->context->debug, LOG_INF,
                            "pgn: %u, prio: %u, src: %u, dest: %u\n", pgn, prio, saddr, daddr);
            } else {
                gpsd_report(session->context->debug, LOG_INF,
                    "pgn: %u\n", pgn);
            }

            startbin += n2k_payload_offset;
            binlen -= n2k_payload_offset;

            gpsd_report(session->context->debug, LOG_RAW,
                        "pgn: %u, length: %u\n", pgn, binlen);

            if(binlen == 9) {

                // we do have len 9 as log prints out the always existing '\0' from gpsd's out buffer

                frame.extid = nmea2000_make_extid(pgn, 3, 0, 0);
                frame.len = 8;
                memcpy(frame.data, startbin, 8);

            } else {

                // report format 2 with a fully parsed package

                outbuffer = startbin;
                outbuflen = binlen;

            }
        }
    }

    if(frame.len > 0) {

        int mb;

        mb = nmea2000_parsemsg(&frame);
        packet = &nmea2000_packets[mb];

        if((mb > -1) && (packet->state == complete)) {

            outbuffer = &packet->outbuffer[0];
            outbuflen = packet->outbuflen;
            pgn = packet->pgn;
        }

        gpsd_report(session->context->debug, LOG_SPIN,
                    "Parse frame returned mb %d \n", mb);

    }

    if(outbuffer) {

        // actually we make this very complicated: extract package juts to wrap it again same way

        char scbuf[512];
        uint8_t bufb[MAX_PACKET_LENGTH*2];

        int i = 0, j = 0;
        memset(scbuf, 0, 512);

        for (i = 0; j < 510 && i < outbuflen; i++) {
            scbuf[j++] = hexchar[(outbuffer[i] & 0xf0) >> 4];
            scbuf[j++] = hexchar[outbuffer[i] & 0x0f];
        }
        gpsd_report(session->context->debug, LOG_SPIN,
                    "Parsing now %s\n", scbuf);

        gps_mask_t mask = 0;
        pgns = vyspi_find_pgn(pgn);
        if(pgns)
            mask = (pgns->func)(outbuffer, outbuflen, pgns, session);

        memcpy(bufb + n2k_payload_offset, outbuffer, outbuflen);

        if(bcast != NULL) {
            send_frame(session, bufb, FRM_TYPE_NMEA2000, outbuflen, pgn, 0x03, 0x22, 0xfe);
        }
    }

}


static void print_data(struct gps_context_t *context,
                       unsigned char *buffer, int len)
{
    int   l1, l2, ptr, atr;
        char  bu[128];
        char  au[128];
        char c;

        ptr = 0; atr = 0;
        au[0] = '\0';
        l2 = sprintf(&au[ptr], "VYSPI: :%6u:%3d : ", 0, len);
        atr += l2;
        for (l1=0;l1<len;l1++) {
            if (((l1 % 8) == 0) && (l1 != 0)) {
                gpsd_report(context->debug, LOG_IO,"%s : %s\n", au, bu);
                ptr = 0; atr = 0;
                l2 = sprintf(&au[atr], "                   : ");
                atr += l2;
            }
            if(isprint(buffer[l1])) c= buffer[l1]; else c= '.';
            sprintf(&au[atr], "%c", c);
            atr += 1;
            l2 = sprintf(&bu[ptr], "%02x ", (unsigned int)buffer[l1]);
            ptr += l2;
        }

        for(l2 = l1 % 8; l2 < 8; l2++) {
            au[atr] =  ' ';
            atr += 1;
        }

        gpsd_report(context->debug, LOG_IO, "%s : %s\n", au, bu);
}

void decode_raw_file(struct gps_device_t * session, char * filename) {

    FILE * fp = NULL;

    uint8_t buffer[1024];
    int c = 0;
    uint16_t n = 0;
    uint32_t l = 0;

    fp = fopen(filename, "r");

    if (fp == NULL) {
        gpsd_report(session->context->debug, LOG_ERROR,
                    "file %s not found\n", filename);
        exit(1);
    }

    gpsd_report(session->context->debug, LOG_INF,
                "opened file %s for reading\n", filename);

    do {

        c = fgetc(fp);
        if ((c == 0x7e) || (n >= 1024) ) {
          print_data(session->context, buffer, n);
          send_frame_out(session, buffer, n);
          n= 0;
        }
        buffer[n] = c;
        n++; l++;

    } while (c != EOF);

    gpsd_report(session->context->debug, LOG_INF,
                "file %s closed now after read.\n",
                filename);
    fclose(fp);
}

void decode_file(struct gps_device_t * session, char * filename) {

    FILE * fp = NULL;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char buffer[255];

    fp = fopen(filename, "r");

    if (fp == NULL) {
        gpsd_report(session->context->debug, LOG_ERROR,
                    "file %s not found\n", filename);
        exit(1);
    }

    gpsd_report(session->context->debug, LOG_INF,
                "opened file %s for reading\n", filename);

    while ((read = getline(&line, &len, fp)) != -1)
    {
        gpsd_report(session->context->debug, LOG_RAW,
                    "Retrieved line %s of length %zu\n", line, read);

        if(line[0] == '$') {
            continue;
        }

        // reportformat 2: {pgn}{bin}
        if(report_format == 2) {

            char * end = line;
            while(strchr("0123456789abcdef", *end)) end++;
            strncpy(buffer, line, end - line); buffer[end - line] = 0;
            gpsd_report(session->context->debug, LOG_SPIN,
                        "Retrieved line %s of length %ld\n", buffer, end - line);
            decode_line(session, buffer);

        } else {

            decode_line(session, line);

        }
    }

    gpsd_report(session->context->debug, LOG_INF,
                "file %s closed now after read with error no %d (%s).\n",
                filename, errno, strerror(errno));
    fclose(fp);
    if (line)
        free(line);

    /* TODO - single argument to decode

      } else if(n2kSequence[0]) {

        decodeline(session, version, n2kSequence, strlen(n2kSequence), offset);

      }
    */
}

// returns resulting len
int nmea0183_clean(char * dest, int destlen, char * src, int srclen) {

    unsigned int len = 0;
    char * end = NULL;
    char * endofstring = NULL; // pointer to the last char
    unsigned int n, crc = 0;
    char csum[3] = { '0', '0', '0' };

    // find end of actual string, ignore whitespace \r\n
    for (end = src+srclen-1; isspace(*end); end--)
        continue;
    endofstring = end;

    // if there is a '*' then we'll get to it here
    while (strchr("0123456789ABCDEF", *end))
        --end;

    if (*end == '*') {
        // there is a checksum
        endofstring = end - 1;
    }

    len = endofstring - src + 1;

    // calc checksum, assuming first char is $
    for (n = 1; n < len; n++)
        crc ^= src[n];
    (void)snprintf(csum, sizeof(csum), "%02X", crc);

    memset(dest, 0, destlen);
    memcpy(dest, src, len);

    dest[len + 0] = '*';
    dest[len + 1] = csum[0];
    dest[len + 2] = csum[1];
    dest[len + 3] = '\r';
    dest[len + 4] = '\n';

    len += 5;

    return len;
}

int main(int argc, char**argv) {

    uint8_t frmType = 255;
//    uint8_t frm[255];

    uint8_t opts = 0;
    int interval = 1;
    int batch_mode = 0;
    char c;
    int single_mode = 0;
    char message[2048];
    int loglevel = 5;
    char filename[255];
    int yes = 1;


    struct gps_device_t session;
    struct gps_context_t context;


    usb_dev[0] = '\0';
    filename[0] = '\0';

    while ((c = getopt(argc, argv, "Bb:c:p:f:r:i:D:t:d:D:v:")) != -1) {
        switch(c) {
        case 'D':
            if(strlen(optarg) > 3) {
                memcpy(usb_dev, optarg, strlen(optarg));
                printf("device %s\n", usb_dev);
                opts++;
            }
            break;

        case 'd':
            loglevel = atoi(optarg);
            printf("loglevel %d\n", loglevel);
            opts++;
            break;

        case 'c':
            if(strlen(optarg) > 3) {
                strcpy(message, optarg);
                printf("message=  %s\n", message);
                single_mode = 1;
                opts++;
            }
            break;

        case 't':
            if((strlen(optarg) > 6) && (strcmp(optarg, "seatalk") == 0)) {
                printf("seatalk\n");
                frmType = FRM_TYPE_ST;  // port type seatalk
                opts++;
            } else if((strlen(optarg) > 7) && (strcmp(optarg, "nmea0183") == 0)) {
                printf("nmea0183\n");
                frmType = FRM_TYPE_NMEA0183;  // port type nmea 0183
                opts++;
            } else if(strcmp(optarg, "n2k") == 0) {
                printf("nmea2000\n");
                frmType = FRM_TYPE_NMEA2000;
                opts++;
            } else if(strcmp(optarg, "cmd") == 0) {
                printf("cmd\n");
                frmType = FRM_TYPE_CMD;
                opts++;
            } else {
                printf("unknown type\n");
                opterr = 1;
            }
            break;

        case 'B':
            batch_mode = 1;
            printf("using batch mode\n");
            break;

        case 'b':
            bcast = optarg;
            break;

        case 'p':
            port = atol(optarg);
            break;

        case 'f':
            strncpy(filename, optarg, 255);
            printf("n2k filename %s\n", filename);
            opts++;
            break;

        case 'i':
            printf("interval %s\n", optarg);
            interval = atol(optarg);
            break;

        case 'r':
            report_format = atoi(optarg);
            printf("report version format %d\n", report_format);
            opts++;
            break;

        case 'v':
            protocol_version = atoi(optarg);
            printf("frm protocol version %d\n", protocol_version);
            opts++;
            break;

        default:
            break;
        }
    }

    if(frmType == 255) {
        printf("No or wrong frame type given (-t [n2k | seatalk | nmea0183 | cmd])\n");
        exit(1);
    }

    if(protocol_version) {
        printf("Switching to protocol version 1 with payload offset = 7\n");
        n2k_payload_offset = 7;
    }


    context.debug = loglevel;
    context.readonly = 0;
    session.context = &context;

    const struct gps_type_t **dp;
    int pt = 0;

    if(frmType == FRM_TYPE_NMEA0183)
        pt = NMEA_PACKET;
    else if(frmType == FRM_TYPE_NMEA2000)
        pt = VYSPI_PACKET;

    for (dp = gpsd_drivers; *dp; dp++) {
        if(pt == (*dp)->packet_type) {
            session.device_type = *dp;
        }
    }

    nmea2000_init();


    sinlen = sizeof(struct sockaddr_in);
    memset(&sock_in, 0, sinlen);

    sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    sock_in.sin_port = htons(0);
    sock_in.sin_family = PF_INET;

    /* -1 = 255.255.255.255 this is a BROADCAST address,
       a local broadcast address could also be used.
       you can comput the local broadcat using NIC address and its NETMASK
    */

    if(usb_dev[0]) {

        printf("Using device %s\n", usb_dev);

        usb_sock = open(usb_dev, O_RDWR | O_NOCTTY );
        if(usb_sock < 0)  {
            printf("Couldn't open device %s\n", usb_dev);
            exit(1);
        }
        init(usb_sock);

    } else if(bcast != NULL) {

        sock_in.sin_addr.s_addr=inet_addr(bcast); //htonl(-1); /* send message to 255.255.255.255 */
        printf("sendto using %s\n", bcast);
        // sock_in.sin_addr.s_addr= htonl(-1); /* send message to 255.255.255.255 */

        if(port > 0) {
            sock_in.sin_port = htons(port); /* port number */
            printf("sendto using port %d\n", port);
        } else {
            sock_in.sin_port = htons(PORT); /* port number */
        }

        sock_in.sin_family = PF_INET;

        status = bind(sock, (struct sockaddr *)&sock_in, sinlen);
        printf("Bind Status = %d\n", status);

        status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );
        printf("Setsockopt Status = %d\n", status);

    }

    if(filename[0]) {


        if(report_format > 0) {

            gpsd_report(session.context->debug, LOG_INF, "decoding file\n");
            decode_file(&session, filename);

        } else {

            // this is super raw format directly from the microcontroller
            gpsd_report(session.context->debug, LOG_INF, "reading microcontroller raw file\n");
            decode_raw_file(&session, filename);

        }

        exit(0);

    } else if(!usb_dev[0] && !bcast) {

        printf("Need to provide filename and/or broadcast address or device name!\n");
        exit(1);

    }

    double lat1 = 59.445424;
    double lon1 = 18.33189;

    double course = 270;
    double SOG_fix    = 3.7; // knots (nm/h)
    double STW_fix    = 3.2;
    double distance = 0;
    double lat, lon;
    double depth = 2.7;
    uint32_t counter = 0;

    char bufp[1024];
    uint8_t bufb[1024];
    size_t len = 1024;

    srand(time(NULL)); // randomize seed

    if(usb_dev[0] != '\0') {
        session.gpsdata.gps_fd = usb_sock;
    }

    session.newdata.time = timestamp();
    session.newdata.mode = 10;

    session.gpsdata.navigation.heading[compass_magnetic] = 90;
    session.gpsdata.navigation.heading[compass_true] = 90;

    session.gpsdata.navigation.distance_total = 23444;
    session.gpsdata.navigation.distance_trip = 234;

    session.gpsdata.set = NAVIGATION_SET;
    session.gpsdata.navigation.set = NAV_DIST_TRIP_PSET | NAV_DIST_TOT_PSET;

    session.gpsdata.attitude.roll = 23;

    /* initialize random seed: */
    srand (time(NULL));

    double caliberr[] = {1.3, 1.5, 1.5, 1.7, 2.0, 2.1, 2.1};

    gpsd_report(session.context->debug, LOG_INF, "Using %.1fHz \n", 1.0/(float)interval/10.0);
    gpsd_report(session.context->debug, LOG_INF, "Using protocol version %d with %d bytes payload offset\n",
                protocol_version, n2k_payload_offset);


    if(single_mode) {

        uint8_t bin[1024];
        uint8_t frm[1024];
        ssize_t n_written;

        int frmlen = 0;
        uint32_t pgn = 0;

        char* token = strtok(message, ";");

        while (token) {

            int msgll = 0;
            uint8_t off = 0;

            if(frmType == FRM_TYPE_NMEA2000) {

                msgll = gpsd_hexpack(token, bin, 1024);

                pgn = getleu32(bin, 0);
                printf("pgn = %u\n", pgn);

                off = 4; // first 4 bytes are pgn

            } else if(frmType == FRM_TYPE_NMEA0183) {

                gpsd_report(session.context->debug, LOG_IO,
                            "Cleaning %s\n", token);

                msgll = nmea0183_clean((char *)bin, 1024, token, strlen(token));
                print_data(session.context, bin, msgll + 5);

                gpsd_report(session.context->debug, LOG_IO,
                            "Cleaned %s\n", token);
            }

            if(batch_mode) {
                frmlen += make_frame(frm+frmlen, 1024-frmlen,
                                         bin+off, frmType, msgll, pgn, 0x03, 0x22, 0xfe);
            } else {
                // send each frame individually
                send_frame(&session, bin+off, frmType, msgll, pgn, 0x03, 0x22, 0xfe);
            }

            token = strtok(0, ";");
        }

        if(batch_mode) {
            if(usb_sock < 0)  {
                if((n_written = sendto(sock, frm, frmlen, 0, (struct sockaddr *)&sock_in, sinlen)) < 0) {
                    // if(send(sock, buf, strlen(buf), 0) < 0) {
                    printf("sendto Status = %s\n", strerror(errno));
                }
                printf("Wrote %ld bytes of %u byte pgn to udp://%s:%d\n", n_written, frmlen, bcast, port);
            } else {
                n_written = write( usb_sock, frm, frmlen );
                printf("Wrote %ld bytes to %s\n", n_written, usb_dev);
            }
        }

    } else while(true) {

        // get something like +/- 5% deviation and make a wrong calibrated speed sensor
        double ce = caliberr[(counter % 6)];
        SOG_fix = (counter % 6) + 1;
        STW_fix = SOG_fix*ce;

        // double sog = SOG_fix + ((rand() / (double)RAND_MAX ) - 0.5) * SOG_fix*0.1;
        // double stw = STW_fix + ((rand() / (double)RAND_MAX ) - 0.5) * STW_fix*0.1;
        double sog = 5;
        double stw = 5;

        distance = sog/3600.0; // get distance traveled per second
        depth   += ((rand() / (double)RAND_MAX) - 0.5) / 5.0;

        // yes, this asks for nm!
        earth_position_from_distance(lat1, lon1,
                                     course, distance,
                                     &lat, &lon);

        session.newdata.latitude = lat;
        session.newdata.longitude = lon;
        session.newdata.altitude = 0;

        session.gpsdata.separation = wgs84_separation(lat, lon);
        session.gpsdata.status = 1;
        session.gpsdata.satellites_used = 3;
        int i = 0;
        for(i = 0; i < MAXCHANNELS; i++) {
            session.gpsdata.used[i]        = 0;
        }
        for(i = 0; i < session.gpsdata.satellites_used; i++) {
            session.gpsdata.used[i]        = gd[i].PRN;
        }
        session.gpsdata.set |= USED_IS;

        session.gpsdata.dop.hdop = NAN;
        // session.gpsdata.set |= DOP_SET;

        session.gpsdata.status = STATUS_FIX;
        session.gpsdata.set |= STATUS_SET;

        session.newdata.mode = MODE_3D;
        session.gpsdata.set |= MODE_SET;

        session.gpsdata.set |= TIME_SET | LATLON_SET | ALTITUDE_SET;

        session.mag_var = NAN;
        session.gpsdata.navigation.speed_over_ground = sog;


        session.gpsdata.satellites_visible = 10;

        for(i = 0; i < 10; i++) {
            session.gpsdata.PRN[i]        = gd[i].PRN;
            session.gpsdata.elevation[i]  = gd[i].elevation;
            session.gpsdata.azimuth[i]    = gd[i].azimuth;
            session.gpsdata.ss[i]         = gd[i].ss;
        }

        session.gpsdata.set |= SATELLITE_SET;

        gps_merge_fix(&session.gpsdata.fix,
                      session.gpsdata.set, &session.newdata);

        if(frmType == FRM_TYPE_NMEA0183) {

            nmea_tpv_dump(&session, bufp, len);
            wrap_and_send_0183(&session, bufp, FRM_TYPE_NMEA0183, sock, (struct sockaddr_in *)&sock_in, sinlen);

        } else if(frmType == FRM_TYPE_NMEA2000) {

        }


        if(frmType == FRM_TYPE_NMEA0183) {
            nmea_sky_dump(&session, bufp, len);
            wrap_and_send_0183(&session, bufp, FRM_TYPE_NMEA0183, sock, (struct sockaddr_in *)&sock_in, sinlen);
        }

        session.gpsdata.set = NAVIGATION_SET;

        session.gpsdata.navigation.set =
            NAV_DPT_PSET | NAV_HDG_MAGN_PSET | NAV_HDG_TRUE_PSET
            | NAV_STW_PSET | NAV_ROT_PSET;
        session.gpsdata.navigation.depth = depth;

        session.gpsdata.navigation.depth_offset = 0.3;

        session.gpsdata.navigation.heading[compass_magnetic] += 5;
        session.gpsdata.navigation.heading[compass_true] += 5;

        session.gpsdata.navigation.speed_thru_water = stw;

        session.gpsdata.navigation.rate_of_turn = 17;

        // meters
        session.gpsdata.waypoint.xte = 47;

        // active to waypoint
        strcpy(session.gpsdata.waypoint.active_to, "act_wp1");
        // active from waypoint
        strcpy(session.gpsdata.waypoint.active_from, "acf_wp3");

        // Destination Waypoint Latitude
        session.gpsdata.waypoint.latitude = 59.3;
        // Destination Waypoint Longitude
        session.gpsdata.waypoint.longitude = 18.2;

        // Range to destination in m
        session.gpsdata.waypoint.range_to_destination = 2456;

        // Bearing to destination from origin in degrees True to North
        session.gpsdata.waypoint.bearing_from_org_to_destination = 357;

        // Bearing to destination from position in degrees True to North
        session.gpsdata.waypoint.bearing_from_pos_to_destination = 356;

        // Destination closing velocity in knots
        session.gpsdata.waypoint.speed_to_destination = 7;

        // Arrival Status, A = Arrival Circle Entered
        session.gpsdata.waypoint.arrival_status = waypoint_circle_entered;

        session.gpsdata.set |= WAYPOINT_SET;
        session.gpsdata.waypoint.set = WPY_XTE_PSET
            | WPY_ACTIVE_TO_PSET | WPY_ACTIVE_FROM_PSET
            | WPY_LATLON_TO_PSET | WPY_RANGE_TO_PSET | WPY_BEARING_FROM_ORG_TO_PSET
            | WPY_BEARING_FROM_POS_TO_PSET | WPY_SPEED_FROM_ORG_TO_PSET | WPY_ARRIVAL_STATUS_PSET | WPY_ETA_PSET;


        session.gpsdata.set |= ENVIRONMENT_SET;

        session.gpsdata.environment.set = ENV_WIND_APPARENT_ANGLE_PSET | ENV_WIND_APPARENT_SPEED_PSET;

	    session.gpsdata.environment.wind[wind_apparent].angle = 33.7;
        session.gpsdata.environment.wind[wind_apparent].speed = 5.5;

        session.gpsdata.environment.deviation = 0;
        session.gpsdata.environment.variation = 0;

        session.gpsdata.set |= ATTITUDE_SET;
        if(session.gpsdata.attitude.roll < 0)
            session.gpsdata.attitude.roll = -1.0 * session.gpsdata.attitude.roll;

        session.driver.aivdm.ais_channel = 'B';
        session.gpsdata.ais.mmsi = 1234566;
        session.gpsdata.ais.type = 1;
        session.gpsdata.ais.repeat = 0;

        session.gpsdata.ais.type1.lat = session.newdata.latitude * 600000.0;
        session.gpsdata.ais.type1.lon = session.newdata.longitude * 600000.0;

        session.gpsdata.ais.type1.accuracy      = 0;
        session.gpsdata.ais.type1.raim          = 0;
        session.gpsdata.ais.type1.second        = 0;

        session.gpsdata.ais.type1.course        = 37.0 * 10.0; // should be deci degrees
        session.gpsdata.ais.type1.speed         =  5.1 * 10.0;  // this is deci knots :(

	    session.gpsdata.ais.type1.radio         = 0;

        session.gpsdata.ais.type1.heading       = 37; // is only int - should be degrees
        session.gpsdata.ais.type1.turn          = 256 - (int)(sqrt((double)(37.8)) * 4.733);

        session.gpsdata.ais.type1.status        = 0;

        session.gpsdata.ais.own_mmsi            = 0;
        session.gpsdata.set  |= AIS_SET;

        session.gpsdata.engine.instance[0].speed               = 2000;              // RPM
        session.gpsdata.engine.instance[0].tilt                = 0.9;
        session.gpsdata.engine.instance[0].boost_pressure      = 16;     // pascal
        session.gpsdata.engine.instance[0].temperature         = 273 + 12;        // K
        session.gpsdata.engine.instance[0].fuel_rate           = 12;          // liter per hour
        session.gpsdata.engine.instance[0].fuel_pressure       = 55;      // pascal
        session.gpsdata.engine.instance[0].alternator_voltage  = 14.2; // V
        session.gpsdata.engine.instance[0].oil_temperature     = 273 + 78;    // K
        session.gpsdata.engine.instance[0].oil_pressure        = 12;       // pascal
        session.gpsdata.engine.instance[0].coolant_temperature = 273 + 51;// K
        session.gpsdata.engine.instance[0].coolant_pressure    = 88;   // pascal
        session.gpsdata.engine.instance[0].total_hours         = 3600;        // seconds
        session.gpsdata.engine.instance[0].torque              = 97;             // percentage
        session.gpsdata.engine.instance[0].load                = 99;               // percentage

        session.gpsdata.engine.set = ENG_PORT_PSET | ENG_SPEED_PSET | ENG_TILT_PSET
            | ENG_BOOST_PRESSURE_PSET | ENG_TEMPERATURE_PSET | ENG_FUEL_RATE_PSET
            | ENG_FUEL_PRESSURE_PSET | ENG_ALTERNATOR_VOLTAGE_PSET | ENG_OIL_TEMPERATURE_PSET
            | ENG_OIL_PRESSURE_PSET | ENG_COOLANT_TEMPERATURE_PSET | ENG_COOLANT_PRESSURE_PSET
            | ENG_TOTAL_HOURS_PSET | ENG_TORQUE_PSET | ENG_LOAD_PSET;

        session.gpsdata.set  |= ENGINE_SET;


        printf(">>> %d     %f\n", session.gpsdata.ais.type1.lat,  session.newdata.latitude * 0.06);
        printf(">>> %d     %f\n", session.gpsdata.ais.type1.lon,  session.newdata.longitude * 0.06);

        if(frmType == FRM_TYPE_NMEA0183) {

            nmea_ais_dump(&session, bufp, len);
            wrap_and_send_0183(&session, bufp, FRM_TYPE_AIS, sock, (struct sockaddr_in *)&sock_in, sinlen);

            nmea_navigation_dump(&session, bufp, len);
            wrap_and_send_0183(&session, bufp, FRM_TYPE_NMEA0183, sock, (struct sockaddr_in *)&sock_in, sinlen);

        } else if(frmType == FRM_TYPE_NMEA2000) {

            uint16_t written;
            uint32_t pgn;

            char tbuf[JSON_DATE_MAX + 1];
            unix_to_iso8601(session.newdata.time, tbuf, sizeof(tbuf));
            gpsd_report(LOG_IO, LOG_IO, "time = %s\n", tbuf);

            // GNSS System Time
            n2k_binary_126992_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // rate of turn
            n2k_binary_127251_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // depth
            n2k_binary_128267_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            n2k_binary_hdg_magnetic_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset,
                                         &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // only roll atm
            n2k_binary_attitude_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // AIS Class A (type1) position report
            n2k_129038_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // wind data
            n2k_binary_130306_dump(&session, wind_apparent,
                &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // GNSS Position Rapid Update
            n2k_binary_129025_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // GNSS Positition Data
            n2k_binary_129029_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // GNSS DOPs
            n2k_binary_129539_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // GNSS Satellites in View
            n2k_binary_129540_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // trip log
            n2k_binary_128275_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);

            // engine rapid update
            n2k_binary_127488_dump(&session, &pgn, bufb+n2k_payload_offset, len-n2k_payload_offset, &written);
            send_frame(&session, bufb, frmType, written, pgn, 0x03, 0x22, 0xfe);
        }

        printf("sog=%.2f, stw=%.2f\n",
               session.gpsdata.navigation.speed_over_ground,
               session.gpsdata.navigation.speed_thru_water);

        lat1 = lat;
        lon1 = lon;

        session.gpsdata.navigation.distance_total += distance;
        session.gpsdata.navigation.distance_trip += distance;


        session.newdata.time += interval;
        usleep(1000*100*interval);
        counter++;
    }

    if(usb_sock > -1)
        close(usb_sock);

    shutdown(sock, 2);
    close(sock);


    return 0;
}
