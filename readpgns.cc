#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "gps.h"
#include "gpsd.h"
#include "gpsd_config.h"

#include "frame.h"
#include "driver_vyspi.h"
#include "bits.h"

#include <unistd.h>
#include <string.h>

void gpsd_throttled_report(const int errlevel, const char * buf) {}
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

void gpsd_external_report(const int debuglevel, const int errlevel,
			  const char *fmt, ...) {
}

ssize_t gpsd_write(struct gps_device_t *session,
		   const char *buf,
		   const size_t len)
/* pass low-level data to devices, echoing it to the log window */
{
    return gpsd_serial_write(session, buf, len);
}

int main(int argc, char * argv[]) {

    struct PGN * work;
    uint32_t pgn = 0;
    int opts = 0;
    char c;
    char msg[2000];

    static struct gps_device_t device;
    struct gps_context_t context;

    device.context = &context;
    
    while ((c = getopt(argc, argv, "c:")) != -1) {
        switch(c) {
            case 'c':
                if(strlen(optarg) > 3) {
                    memcpy(msg, optarg, strlen(optarg));
                    opts++;
                }
                break;
                
            default:
                break;
        }
    }
    
    if(opts != 1) {
        
        printf("usage: readpgns -c <hex can msg>\n");
        exit(1);
    }
    
    printf("msg = %s\n", msg);
    uint8_t bin[1024];
    int msgll = gpsd_hexpack(msg, (char *)bin, 1024);

    pgn = getleu32(bin, 0);
    printf("pgn = %u\n", pgn);

    work = vyspi_find_pgn(pgn);

    if (work != NULL) {
              
        uint8_t * b = bin+4;
              
        (work->func)(b, msgll - 4, work, &device);

    } else {
        gpsd_report(5, LOG_ERROR, 
                    "VYSPI: no work PGN found for pgn = %u\n", 
                    pgn);
    }
}

