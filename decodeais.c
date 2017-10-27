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

int main(int argc, char * argv[]) {

    char n2kSequence[255];
    uint8_t bin[255];
  
    uint8_t opts = 0;

    opterr = 0;
    char c = 0;

    while ((c = getopt(argc, argv, "b:")) != -1) {

        switch (c) {
        case 'b': 
            strncpy(n2kSequence, optarg, 255);
            printf("n2k %s\n", n2kSequence);
            opts++;
            break;
        default:
            abort ();
        }
    }


    static struct gps_context_t context;
    gps_context_init(&context);
    context.debug = 13;

    struct gps_device_t session;
    session.context = &context;

    struct ais_t ais;
    struct ais_type24_queue_t type24_queue;
  
    aivdm_decode(n2kSequence, strlen(n2kSequence),
                 &session,
                 &ais,
                 13);

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
