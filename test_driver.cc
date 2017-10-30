#include "gpsd.h"
#include "frame.h"
#include "utils.h"
#include "pseudon2k.h"

#include "driver_vyspi.h"
#include "test_utils.h"

int loglevel = 5;

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

// this is a special get to just get a packet into outbuffer
ssize_t test_driver_get(struct gps_device_t *session) {
    gpsd_report(session->context->debug, LOG_INF,
                "WORKS!!!\n");
    return 0;
}

const struct gps_type_t driver_test = {
    .type_name      = "VYSPI",       /* full name of type */
    .packet_type    = VYSPI_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_STICKY,	/* remember this */
    .trigger	    = NULL,		/* detect their main sentence */
    .channels       = 12,		/* not an actual GPS at all */
    .probe_detect   = NULL,
    .get_packet     = test_driver_get,	/* how to get a packet */
    .parse_packet   = vyspi_parse_input,	/* how to interpret a packet */
    .rtcm_writer    = NULL,		/* Don't send RTCM to this */
    .event_hook     = NULL,
#ifdef RECONFIGURE_ENABLE
    .speed_switcher = NULL,		/* no speed switcher */
    .mode_switcher  = NULL,		/* no mode switcher */
    .rate_switcher  = NULL,		/* no rate switcher */
    .min_cycle      = 1,		/* nominal 1-per-second GPS cycle */
#endif /* RECONFIGURE_ENABLE */
#ifdef CONTROLSEND_ENABLE
    .control_send   = NULL,		/* how to send control strings */
#endif /* CONTROLSEND_ENABLE */
};


int main(int argc, char**argv) {

    struct gps_device_t session;
    struct gps_context_t context;

    context.debug = loglevel;
    context.readonly = 0;
    session.context = &context;

    session.device_type = &driver_test;

    gps_mask_t changed = gpsd_poll(&session);

    return 0;
}
