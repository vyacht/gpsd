#ifndef _TIMEUTIL_H_
#define _TIMEUTIL_H_

#include <time.h>

void
tu_gettime(struct timespec * ts);

uint32_t 
tu_get_time_in_milli(struct timespec * ts);

uint32_t 
tu_get_independend_time(void);

void
tu_init_time(struct gps_context_t *context);

uint32_t 
tu_get_millis_since(struct timespec * ts); 

#endif // _TIMEUTIL_H_
