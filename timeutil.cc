#include <stdint.h>

#include "gpsd.h"
#include "timeutil.h"

/* work-around CLOCK_MONOTONIC_RAW definition missing in openwrt in good place */
#ifndef CLOCK_MONOTONIC_RAW
#define CLOCK_MONOTONIC_RAW 4
#endif

#define SEC2NANO 1000000000.0f

struct timespec tu_independend_starttime;

uint32_t 
tu_get_time_in_milli(struct timespec * ts)
{
    return (ts->tv_sec*1000.0f + ts->tv_nsec/SEC2NANO*1000.0f);
}

void
tu_gettime(struct timespec * ts) {
    clock_gettime(CLOCK_MONOTONIC_RAW, ts);
}

uint32_t 
tu_get_independend_time(void)
{
    return tu_get_millis_since(&tu_independend_starttime);
}

uint32_t 
tu_get_millis_since(struct timespec * ts) {

    struct timespec now;
    tu_gettime(&now);

    uint32_t totaltime = tu_get_time_in_milli(&now) 
        - tu_get_time_in_milli(ts);

    return totaltime;
}


void
tu_init_time(struct gps_context_t *context) {

    tu_gettime(&tu_independend_starttime);

    gpsd_report(context->debug, LOG_INF,
                "initializing to start time to %u with (%lu sec, %lu nsec)\n", 
                tu_get_time_in_milli(&tu_independend_starttime),
                tu_independend_starttime.tv_sec, tu_independend_starttime.tv_nsec);
}



