#ifndef _TEST_UTILS_H_
#define _TEST_UTILS_H_

#include "gpsd.h"
#include "frame.h"

void set_device_type(struct gps_device_t *session, enum frm_type_t frm);

#endif // _TEST_UTILS_H_
