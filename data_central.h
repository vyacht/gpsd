#ifndef _DATA_CENTRAL_H_
#define _DATA_CENTRAL_H_

#include "navigation.h"

struct data_central_t {
    struct nav_central_t nav;
};

void dc_merge(struct data_central_t * dc,  struct gps_device_t *device);

#endif // _DATA_CENTRAL_H_
