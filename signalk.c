/*
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "gpsd.h"
#include "json.h"

/*
{
  "updates":[{
    "source": {"type":"NMEA0183","src":"GLL","label":"signalk-parser-nmea0183"},
    "timestamp":"2015-11-03T10:24:33.000Z",
    "values":[{
         "path":"navigation.position",
         "value": { 
             "longitude":23.495333333333335,
             "latitude":60.04405,
             "source":{"type":"NMEA0183","src":"GLL","label":"signalk-parser-nmea0183"},
             "timestamp":"undefined"
         }
    }]
  }],
  "context":"vessels.10101010"
}
 */

/* *INDENT-OFF* */
/*@observer@*/char *unix_to_signalk(timestamp_t fixtime, /*@ out @*/
				     char isotime[], size_t len)
/* Unix UTC time to ISO8601, no timezone adjustment */
/* example: 2007-12-11T23:38:51.033Z */
{
    struct tm when;
    double integral;
    time_t intfixtime;

    modf(fixtime, &integral);
    intfixtime = (time_t) integral;
    (void)gmtime_r(&intfixtime, &when);

    (void)strftime(isotime, len, "%a %Y-%m-%dT%H:%M:%S", &when);
    return isotime;
}
/* *INDENT-ON* */

struct signalk_path_t {
    char               path[256];
    gps_mask_t         mask;     // environment, navigation
    gps_mask_t         submask;
    double             factor;   // some values require a multipler to correct units
    struct json_attr_t jattr;
};

void signalk_dpt_full_dump(const struct gps_device_t *device,
		      /*@out@*/ char *reply, size_t replylen)
{
    (void)strlcat(reply, "\"belowTransducer\":", replylen);
    (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                   "{\"value\":%.2f}", device->gpsdata.navigation.depth);
}

void signalk_add_timestamp(const struct gps_device_t *device,
		      /*@out@*/ char *reply, size_t replylen)
{
    timestamp_t ts = timestamp();
    (void)strlcat(reply, "\"timestamp\":\"", replylen); 
    unix_to_signalk(ts, 
                    reply + strlen(reply), replylen - strlen(reply));
    (void)strlcat(reply, "\"", replylen); 
}

gps_mask_t signalk_full_dump(const struct gps_device_t *device,
		      /*@out@*/ char reply[], size_t replylen)
{
    gps_mask_t reported = 0; 

    (void)snprintf(reply, replylen,
                   "{\"uuid\":\"%s\"", "10101010");
    (void)strlcat(reply, ",{\"navigation\":{", replylen);

    // add actual values
    // TODO see to either use timestamps of last seen or invalidate at times
    if (!isnan(device->gpsdata.navigation.depth)) {
        signalk_dpt_full_dump(device, reply, replylen);
    }

    (void)strlcat(reply, "}", replylen); // closing navigation
    (void)strlcat(reply, "}", replylen); // closing all

    return reported;
}


gps_mask_t signalk_update_dump(const struct gps_device_t *device,
		      /*@out@*/ char reply[], size_t replylen)
{
    gps_mask_t reported = 0; 

    (void)strlcpy(reply, "{\"updates\":[{", replylen);
    signalk_add_timestamp(device, reply, replylen);
    (void)strlcat(reply, ",\"values\":[", replylen);

    // add actual values
    uint16_t pu = 0, pt = 0;

    struct signalk_path_t path_updates[] = {
        {"environment.depth.belowTransducer", NAVIGATION_SET, NAV_DPT_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.depth,
          .dflt.real = 0.0}},
        {"navigation.rateOfTurn", NAVIGATION_SET, NAV_ROT_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.rate_of_turn,
          .dflt.real = 0.0}}, 
        /* we do this manually for now
        {"navigation.position", LATLON_SET | ALTITUDE_SET, 0xffffffff,
        {"status",        t_structobject}}, */
        {"navigation.courseOverGroundMagnetic", NAVIGATION_SET, NAV_COG_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.course_over_ground,
          .dflt.real = 0.0}},
        {"navigation.courseOverGroundTrue", NAVIGATION_SET, NAV_COG_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.course_over_ground,
          .dflt.real = 0.0}},
        {"navigation.magneticVariation", ENVIRONMENT_SET, ENV_VARIATION_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.environment.variation,
          .dflt.real = 0.0}},
        {"navigation.headingMagnetic", NAVIGATION_SET, NAV_HDG_MAGN_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.navigation.heading[compass_magnetic],
          .dflt.real = 0.0}},
        {"navigation.headingTrue", NAVIGATION_SET, NAV_HDG_TRUE_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.navigation.heading[compass_true],
          .dflt.real = 0.0}},
        {"navigation.speedOverGround", NAVIGATION_SET, NAV_SOG_PSET, 1.0/KNOTS_TO_MPS,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.speed_over_ground,
          .dflt.real = 0.0}},
        {"navigation.speedThroughWater", NAVIGATION_SET, NAV_STW_PSET, 1.0/KNOTS_TO_MPS,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.speed_thru_water,
          .dflt.real = 0.0}},
        {"navigation.log", NAVIGATION_SET, NAV_DIST_TOT_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.distance_total,
          .dflt.real = 0.0}},
        {"navigation.logTrip", NAVIGATION_SET, NAV_DIST_TRIP_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.navigation.distance_trip,
          .dflt.real = 0.0}},

        {"environment.wind.angleApparent", 
         ENVIRONMENT_SET, ENV_WIND_APPARENT_ANGLE_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.environment.wind.apparent.angle,
          .dflt.real = 0.0}},
        /* 'True' wind angle, -180 to +180 degrees from the bow. Negative numbers to port */
        {"environment.wind.angleTrue", 
         ENVIRONMENT_SET, ENV_WIND_TRUE_GROUND_ANGLE_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.environment.wind.true_north.angle,
          .dflt.real = 0.0} },
        /* The wind direction relative to true north, in compass degrees, 0 = North */
        {"environment.wind.directionTrue", 
         ENVIRONMENT_SET, ENV_WIND_TRUE_GROUND_ANGLE_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.environment.wind.true_north.angle,
          .dflt.real = 0.0} },
        {"environment.wind.directionMagnetic", 
         ENVIRONMENT_SET, ENV_WIND_TRUE_GROUND_ANGLE_PSET, 1.0,
         {"status",        t_real,  
          .addr.real = &device->gpsdata.environment.wind.magnetic_north.angle,
          .dflt.real = 0.0}},
        {"environment.wind.speedOverGround", 
         ENVIRONMENT_SET,  ENV_WIND_TRUE_GROUND_SPEED_PSET, 1.0/KNOTS_TO_MPS,
         {"status",        t_real, 
          .addr.real = &device->gpsdata.environment.wind.true_north.speed,
          .dflt.real = 0.0}},
        {"environment.wind.speedApparent", 
         ENVIRONMENT_SET, ENV_WIND_APPARENT_SPEED_PSET, 1.0/KNOTS_TO_MPS,
         {"status",        t_real, 
          .addr.real = &device->gpsdata.environment.wind.apparent.speed,
          .dflt.real = 0.0}},
        {"environment.waterTemp", ENVIRONMENT_SET, ENV_TEMP_WATER_PSET, 1.0,
         {"status",        t_real,  .addr.real = &device->gpsdata.environment.temp[temp_water],
          .dflt.real = 0.0}}
    };

    for(pu = 0; pu < 18; pu++) {
        if ((device->gpsdata.set & path_updates[pu].mask) != 0) {
            if((device->gpsdata.navigation.set & path_updates[pu].submask) != 0) {
                if(pt > 0) 
                    (void)strlcat(reply, ",{", replylen);
                else
                    (void)strlcat(reply, "{", replylen);
                (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                               "\"path\":\"%s\",\"value\":%.2f}", 
                               path_updates[pu].path, 
                               *(double *)path_updates[pu].jattr.addr.real 
                               * path_updates[pu].factor);
                reported |= NAVIGATION_SET;
                pt++;
            }
        }
    }

    (void)strlcat(reply, "]}", replylen); // close values
    (void)strlcat(reply, "],", replylen); // close updates
    (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                   "\"context\":\"vessels.%s\"", "10101010");
    (void)strlcat(reply, "}", replylen);

    return reported;
}
