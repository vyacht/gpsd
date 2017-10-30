#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include "gpsd.h"
#include "timeutil.h"
#include "ring_buffer.h"
#include "navigation.h"

static void
nav_clear(struct navigation_t * nav) {

  nav->set = 0;

  // speed is knots
  nav->speed_over_ground = NAN;
  nav->eps               = NAN;		/* Speed uncertainty, meters/sec */

  nav->speed_thru_water  = NAN;

  // deg north
  nav->course_over_ground[0]= NAN;
  nav->course_over_ground[1]= NAN;

  nav->epd               = NAN;		/* Track uncertainty, degrees */

  // deg / sec
  nav->rate_of_turn      = NAN;

  // deg
  nav->rudder_angle      = NAN;

  // metric meters
  nav->depth             = NAN;
  nav->depth_offset      = NAN;

  // nm
  nav->distance_total    = NAN;
  nav->distance_trip     = NAN;

  // magnetic or true heading
  nav->heading[0]        = NAN;
  nav->heading[1]        = NAN;
}

void
nav_print_speed_over_ground(struct gps_device_t *session)
{
    // all sorts of risks here if someone keeps writing in parallel - good we are single threadded
    uint32_t i;
    double val;
    uint32_t msec;

    while(rb_peek_n(&session->gpsdata.navigation.speed_over_grounds, i, &val, &msec)) {
        gpsd_report(session->context->debug, LOG_INF,
                "sentence %u (%.4f, %u msec)\n",
                    i, val, msec);
        i++;

    }
}

void
nav_init(struct gps_device_t *device) {

    nav_clear(&device->gpsdata.navigation);

    rb_init(&device->gpsdata.navigation.speed_over_grounds);
    rb_init(&device->gpsdata.navigation.speed_thru_waters);
}

gps_mask_t
nav_set_speed_over_ground_in_knots(double value, struct gps_device_t *session) {

    uint msec;
    msec = tu_get_independend_time();

    session->gpsdata.navigation.speed_over_ground = value;
    session->gpsdata.navigation.set  |= NAV_SOG_PSET;

    rb_put(&session->gpsdata.navigation.speed_over_grounds, value, msec);

    return NAVIGATION_SET;

}

gps_mask_t
nav_set_speed_through_water_in_knots(double value, struct gps_device_t *session) {

    uint msec;
    msec = tu_get_independend_time();

    session->gpsdata.navigation.speed_thru_water = value;
    session->gpsdata.navigation.set  |= NAV_STW_PSET;

    rb_put(&session->gpsdata.navigation.speed_thru_waters, value, msec);

    return NAVIGATION_SET;

}
