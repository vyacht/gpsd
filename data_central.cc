
#include "gpsd.h"

#include "data_central.h"
#include "vy_data.h"

void dc_merge_navigation(struct nav_central_t * nav,  struct gps_device_t *device) {

    if(device->gpsdata.navigation.set & NAV_SOG_PSET) {
        vy_data_set_real(&nav->speed_over_ground,
            device->gpsdata.navigation.speed_over_ground);
    }
    if(device->gpsdata.navigation.set & NAV_COG_TRUE_PSET) {
        vy_data_set_real(&nav->course_over_ground_true,
            device->gpsdata.navigation.course_over_ground[compass_true]);
    }
    if(device->gpsdata.navigation.set & NAV_COG_MAGN_PSET) {
        vy_data_set_real(&nav->course_over_ground_true,
            device->gpsdata.navigation.course_over_ground[compass_magnetic]);
    }
}

void dc_merge(struct data_central_t * dc,  struct gps_device_t *device) {

    if(device->gpsdata.set & NAVIGATION_SET) {
        dc_merge_navigation(&dc->nav, device);
    }
}
