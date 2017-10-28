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
#include <termios.h>    // POSIX terminal control definitions

#include "gps.h"
#include "gpsd.h"
#include "gpsd_config.h"

#include "frame.h"
#include "bits.h"

void init(int USB);

void setleu32(uint8_t * b, int offset, uint32_t val) {
  b[offset]     = val;
  b[offset + 1] = val >>  8;
  b[offset + 2] = val >> 16;
  b[offset + 3] = val >> 24;
}

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

void init(int USB) {

  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( USB, &tty ) != 0 ) {
    printf("Error %d from tcgetattr %s\n", errno, strerror(errno));
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B115200);
  cfsetispeed (&tty, (speed_t)B115200);

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

int make_n2k() {

    volatile int i = 0;
    uint32_t extid = 0, pgn = 128267;
    uint8_t  daddr = 0xff;
    uint8_t  saddr = 0x1e;
    uint8_t  prio  = 0x03;


    if (((pgn & 0xff00) >> 8) < 240) {
      daddr  = (uint8_t)(pgn & 0xff);
      pgn  = pgn & 0x01ff00;
    } else {
      daddr = (uint8_t)0xff;
    }

    if(daddr < 240) 
        pgn = (pgn&0x1ff00) | (daddr & 0xff);
    extid = (pgn & 0x1ffff) << 8 | (prio & 0x07) << 26 | (saddr & 0xff);

    printf("%lx,%lu,p:%x,s:%x,d:%x\n", extid, pgn, prio, saddr, daddr);

    return 0;
}

int main(int argc, char * argv[]) {

  uint32_t speed = 4800;
  char dev[255];
  char n2kSequence[255];
  
  uint8_t protocol_version = 0;

  uint8_t cmd[255];
  uint8_t frmType = 0;
  uint8_t frm[255];

  uint16_t len = 0;
  uint8_t opts = 0;

  opterr = 0;
  char c = 0;

  while ((c = getopt(argc, argv, "c:d:t:s:p:b:")) != -1) {

      switch (c) {
      case 'd':
          if(strlen(optarg) > 3) {
              strcpy(dev, optarg);
              printf("device %s\n", dev);
              opts++;
          } 
          break;

      case 'b': 
              strncpy(n2kSequence, optarg, 255);
              printf("n2k %s\n", n2kSequence);
              opts++;
          break;

      case 'c': 
              strncpy(cmd, optarg, 255);
              printf("cmd %s\n", cmd);
              opts++;
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

      case 's':
          printf("%s speed\n", optarg);
          speed = atol(optarg);
          setleu32(cmd, 6, speed);
          speed = getleu32(cmd, 6);
          printf("%u speed\n", speed);
          opts++;
          break;
          
      case 'p': 
      {
          uint8_t p = atoi(optarg); 
          if((p == 1) || (p == 0)) {
              printf("protocol %u\n", p);
              opts++;
              protocol_version = p;
          }
      }
      break;
      case '?':
          return 1;
      default:
          abort ();
      }
  }
  
  if(opts != 4)  {
      printf("Requires all 4 options -c <command> -t [seatalk|nmea0183|n2k|cmd] -s <speed> -p [0|1] -d <device>\n");
      return 0;
  }

  int USB = open(dev, O_RDWR | O_NOCTTY );
  init(USB);
  
  if(frmType == FRM_TYPE_NMEA2000) {

    uint8_t bin[255];
    int msgll = 
       gpsd_hexpack(n2kSequence, bin, 255);
    len = frm_toHDLC8(frm, 255, 
                      frmType, protocol_version,
                      bin, msgll);


  } else {
    len = frm_toHDLC8(frm, 255, 
                      frmType, protocol_version,         // frame type cmd
                      cmd, strlen(cmd));
  }
  
  
  int n_written = write( USB, frm, len );
  
  printf("%d chars written\n", n_written);
  
  return 0;

}
