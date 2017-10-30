
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "gpsd.h"
#include "driver_vyspi.h"
#include "bits.h"

ssize_t gpsd_write(struct gps_device_t *session,
		   const uint8_t *buf,
		   const size_t len)
/* pass low-level data to devices straight through */
{
    return gpsd_serial_write(session, buf, len);
}

void gpsd_throttled_report(const int errlevel UNUSED, const char * buf UNUSED) {}
void gpsd_report(const int debuglevel, const int errlevel ,
		 const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, 0, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}
void gpsd_external_report(const int debuglevel  UNUSED, const int errlevel UNUSED,
			  const char *fmt  UNUSED, ...) {
}

int main(int argc, char**argv) {

  char c;
  char hexn2k[255];
  uint8_t msg[4096];

  static struct gps_context_t context;
  gps_context_init(&context);
  context.debug = 13;

  struct gps_device_t session;
  session.context = &context;

  while ((c = getopt(argc, argv, "s:")) != -1) {
      switch(c) {
      case 's':
          if(strlen(optarg) > 3) {
              memcpy(hexn2k, optarg, strlen(optarg));
              printf("n2k sentence: %s\n", hexn2k);
          } 
          break;

      default:
          break;
      }
  }

  int msgl = 
       gpsd_hexpack(hexn2k, msg, 4096);
 
  struct PGN *work;
  session.driver.vyspi.last_pgn = getleu32(msg, 0);

  work = vyspi_find_pgn(session.driver.vyspi.last_pgn);
  session.driver.nmea2000.workpgn = (void *) work;

  if (work != NULL) {
              
      gps_mask_t mask = (work->func)(msg + 4, msgl - 4, work, &session);

      printf("pgn: %u\n", session.driver.vyspi.last_pgn);
      gpsd_report(session.context->debug, LOG_INF, 
                  "VYSPI: work PGN found for pgn = %u\n", 
                  session.driver.vyspi.last_pgn);

  } else {
      gpsd_report(session.context->debug, LOG_ERROR, 
                  "VYSPI: no work PGN found for pgn = %u\n", 
                  session.driver.vyspi.last_pgn);
  }

  return 0;
}
