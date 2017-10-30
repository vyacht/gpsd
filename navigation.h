#ifndef _NAVIGATION_H_
#define  _NAVIGATION_H_

#include "vy_data.h"

enum compass_t {
    compass_true = 0,
    compass_magnetic = 1,
};

struct nav_central_t {
    vy_data_t speed_over_ground;
    vy_data_t course_over_ground_true;
    vy_data_t course_over_ground_magnetic;
};

struct navigation_t {

#define NAV_SOG_PSET            (1llu<< 1)
#define NAV_EPS_PSET            (1llu<< 2)
#define NAV_STW_PSET	        (1llu<< 3)
#define NAV_COG_TRUE_PSET       (1llu<< 4)
#define NAV_COG_MAGN_PSET       (1llu<< 5)
#define NAV_EPD_PSET	        (1llu<< 6)
#define NAV_DPT_PSET	        (1llu<< 7)
#define NAV_DPT_OFF_PSET        (1llu<< 8)
#define NAV_DIST_TOT_PSET   	(1llu<< 9)
#define NAV_DIST_TRIP_PSET	    (1llu<<10)
#define NAV_HDG_TRUE_PSET	    (1llu<<11)
#define NAV_HDG_MAGN_PSET	    (1llu<<12)
#define NAV_ROT_PSET	        (1llu<<13)
#define NAV_RUDDER_ANGLE_PSET	(1llu<<14)

    gps_mask_t set;

    // speed is knots
    double speed_over_ground;
    rb_t speed_over_grounds;

    double eps;		/* Speed uncertainty, meters/sec */


    double speed_thru_water;
    rb_t speed_thru_waters;

    // deg north
    double course_over_ground[2];
    double epd;		/* Track uncertainty, degrees */

    // deg / sec
    double rate_of_turn;

    // deg
    double rudder_angle;

    // metric meters
    double depth;
    double depth_offset;

    // nm
    double distance_total;
    double distance_trip;

    // magnetic or true heading
    double heading[2];
};

struct gps_device_t;

void
nav_init(struct gps_device_t *device);
void
nav_print_speed_over_ground(struct gps_device_t *session);
gps_mask_t
nav_set_speed_over_ground_in_knots(double value, struct gps_device_t *session);
gps_mask_t
nav_set_speed_through_water_in_knots(double value, struct gps_device_t *session);

#endif //  _NAVIGATION_H_
