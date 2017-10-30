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
#include "timeutil.h"
#include "ring_buffer.h"
#include "signalk.h"

char *unix_to_signalk(timestamp_t fixtime, /*@ out @*/
                      char isotime[], size_t len);

void signalk_add_timestamp(/*@out@*/ char *reply, size_t replylen);
void signalk_add_unixtimestamp(timestamp_t ts,
                               /*@out@*/ char *reply, size_t replylen);
void signalk_add_fixtimestamp(const struct gps_device_t *device,
                              /*@out@*/ char *reply, size_t replylen);

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
    const char *       path;
    gps_mask_t         mask;     // environment, navigation
    gps_mask_t         submask;
    double             factor;   // some values require a multipler to correct units
    double *           real;
};

void signalk_add_unixtimestamp(timestamp_t ts,
		      /*@out@*/ char *reply, size_t replylen)
{
    (void)strlcat(reply, "\"timestamp\":\"", replylen);
    unix_to_signalk(ts,
                    reply + strlen(reply), replylen - strlen(reply));
    (void)strlcat(reply, "\"", replylen);
}

void signalk_add_timestamp(/*@out@*/ char *reply, size_t replylen)
{
    timestamp_t ts = timestamp();
    signalk_add_unixtimestamp(ts, reply, replylen);
}

void signalk_add_fixtimestamp(const struct gps_device_t *device,
		      /*@out@*/ char *reply, size_t replylen)
{
    timestamp_t ts = device->gpsdata.fix.time;
    signalk_add_unixtimestamp(ts, reply, replylen);
}

void signalk_value_full_dump(const struct gps_device_t *device UNUSED,
                             int * pt, // first value on this level?
                             double value,
                             const char * name,
                             /*@out@*/ char *reply, size_t replylen)
{
    if (!isnan(value)) {
        if(*pt > 0)
            (void)strlcat(reply, ",", replylen);
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                       "\"%s\":{\"value\":%.2f}",
                       name, value);
        (*pt)++;
    }
}

gps_mask_t signalk_track_dump(const struct gps_device_t *device, uint32_t startAfter, const char field[],
                             /*@out@*/ char reply[], size_t replylen)
{
    uint32_t i = 0;
    uint8_t c = 0;
    double val;
    double scale = 1.0/KNOTS_TO_MPS;
    uint32_t msec;

    char buf[255]; // more than enough here for the moment
    int buflen = 255;

    buf[0] = '\0';
    (void)strcpy(reply, "{\"data\":[");

    const rb_t * rb = NULL;
    if(!strcmp(field, "speedOverGround"))
        rb = &device->gpsdata.navigation.speed_over_grounds;
    else if(!strcmp(field, "speedThroughWater"))
        rb = &device->gpsdata.navigation.speed_thru_waters;
    else
        goto close; // yes, my first goto in 30 years outside a kernel driver

    while(rb_peek_n(rb, i, &val, &msec)) {

        if(msec > startAfter) {
            if(c)
                (void)strlcat(buf, ",", buflen);
            c=1;
            (void)strlcat(buf, "{\"navigation\":", buflen);
            (void)snprintf(buf + strlen(buf), buflen - strlen(buf),
                           "{\"%s\":{\"value\":%.4f,\"timestamp\":%u}}}",
                           field, val*scale, msec);

            if(strlen(buf) > replylen - strlen(reply) - 50) {
                gpsd_report(device->context->debug, LOG_RAW,
                            "%s, len buf %lu, len reply: %lu\n",
                            buf, strlen(buf),  replylen - strlen(reply) - 2);
                break;
            }

            (void)strlcat(reply, buf, replylen);
            buf[0] = '\0';
        }
        i++;

    }

close:
    (void)strlcat(reply, "]", replylen);

    (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                   ",\"now\":%u", tu_get_independend_time());

    (void)strlcat(reply, "}", replylen);

    return NAVIGATION_SET;
}

gps_mask_t signalk_full_dump(const struct gps_device_t *device,
                             const struct vessel_t * vessel,
                             /*@out@*/ char reply[], size_t replylen)
{
    gps_mask_t reported = 0;
    int pt[3];
    int i = 0;

    (void)snprintf(reply, replylen,
                   "{\"uuid\":\"urn:mrn:signalk:uuid:%s\"", vessel->uuid);
    if(vessel->mmsi != 0)
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                       ",\"mmsi\":\"%09u\"", vessel->mmsi);

    (void)strlcat(reply, ",\"navigation\":{", replylen);

    // add actual values
    // TODO see to either use timestamps of last seen or invalidate at times
    pt[0] = 0;
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.rate_of_turn,
                            "rateOfTurn", reply, replylen);
    signalk_value_full_dump(device, &pt[0],
                            device->gpsdata.navigation.course_over_ground[compass_magnetic]*DEG_2_RAD,
                            "courseOverGroundMagnetic", reply, replylen);
    signalk_value_full_dump(device, &pt[0],
                            device->gpsdata.navigation.course_over_ground[compass_true]*DEG_2_RAD,
                            "courseOverGroundTrue", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.environment.variation,
                            "magneticVariation", reply, replylen);
    signalk_value_full_dump(device, &pt[0],
                            device->gpsdata.navigation.heading[compass_true]*DEG_2_RAD,
                            "headingTrue", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.heading[compass_magnetic]*DEG_2_RAD,
                            "headingMagnetic", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.speed_over_ground*KNOTS_TO_MPS,
                            "speedOverGround", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.speed_thru_water*KNOTS_TO_MPS,
                            "speedThroughWater", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.distance_total,
                            "log", reply, replylen);
    signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.distance_trip,
                            "logTrip", reply, replylen);

    if (device->gpsdata.fix.mode > MODE_NO_FIX) {
        if(pt[0] > 0)
            (void)strlcat(reply, ",", replylen);
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                       "\"position\":{\"value\":{\"longitude\":%f,\"latitude\":%f}}",
                       device->gpsdata.fix.longitude,
                       device->gpsdata.fix.latitude);
    }

    int go = 0;
    if(device->gpsdata.set & ATTITUDE_SET) { // any of the attitude parameters set?
        if(!isnan(device->gpsdata.attitude.roll)) go++;
        if(!isnan(device->gpsdata.attitude.pitch)) go++;
        if(!isnan(device->gpsdata.attitude.yaw)) go++;
    }

    if(go) {
        if(pt[0] > 0)
            (void)strlcat(reply, ",", replylen);
        pt[0]++;
        pt[1] = 0;

        (void)strlcat(reply, "\"attitude\":{", replylen);

        signalk_value_full_dump(device, &pt[1], device->gpsdata.attitude.roll*DEG_2_RAD,
                                "roll", reply, replylen);
        signalk_value_full_dump(device, &pt[1], device->gpsdata.attitude.pitch*DEG_2_RAD,
                                "pitch", reply, replylen);
        signalk_value_full_dump(device, &pt[1], device->gpsdata.attitude.yaw*DEG_2_RAD,
                                "yaw", reply, replylen);

        (void)strlcat(reply, "}", replylen); // closing attitude
    }

    go = 0;
    if(device->gpsdata.set & ENVIRONMENT_SET) { // any of the wind parameters set?

        if(!isnan(device->gpsdata.navigation.depth)) go++;
        for(i= 0; i< 5; i++) {
            if(!isnan(device->gpsdata.environment.wind[i].angle)) go++;
            if(!isnan(device->gpsdata.environment.wind[i].speed)) go++;
        }
    }

    if(go) {

        (void)strlcat(reply, "},\"environment\":{", replylen);
        pt[0] = 0;

        signalk_value_full_dump(device, &pt[0], device->gpsdata.navigation.depth,
                                "depthBelowTransducer", reply, replylen);

        if(pt[0] > 0)
            (void)strlcat(reply, ",", replylen);
        (void)strlcat(reply, "\"wind\":{", replylen);
        pt[0]++;

        pt[1] = 0;

        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_apparent].angle*DEG_2_RAD,
                                "angleApparent", reply, replylen);
        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_apparent].speed,
                                "speedApparent", reply, replylen);

        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_true_to_boat].angle*DEG_2_RAD,
                                "angleTrueWater", reply, replylen);
        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_true_to_boat].speed,
                                "speedTrue", reply, replylen);

        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_true_north].angle*DEG_2_RAD,
                                "directionTrue", reply, replylen);
        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_true_north].speed,
                                "speedOverGround", reply, replylen);

        signalk_value_full_dump(device, &pt[1], device->gpsdata.environment.wind[wind_magnetic_north].angle*DEG_2_RAD,
                                "directionMagnetic", reply, replylen);


        (void)strlcat(reply, "}", replylen); // closing wind

        signalk_value_full_dump(device, &pt[0], device->gpsdata.environment.temp[temp_water],
                                "waterTemp", reply, replylen);
    }

    (void)strlcat(reply, "}", replylen); // closing environment or previous group

    (void)strlcat(reply, "}", replylen); // closing all

    return reported;
}

void signalk_update_engine_struct(struct gps_device_t *device,
                           struct signalk_path_t * path_updates,
                           enum engine_reference_t einstance) {

    const struct signalk_path_t pus[] = {
        {"engineLoad", ENGINE_SET, ENG_LOAD_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].load},
        {"revolutions", ENGINE_SET, ENG_SPEED_PSET, 60.0,
         &device->gpsdata.engine.instance[einstance].speed},
        {"temperatur", ENGINE_SET, ENG_TEMPERATURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].temperature},
        {"oilTemperatur", ENGINE_SET, ENG_OIL_TEMPERATURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].oil_temperature},
        {"oilPressure", ENGINE_SET, ENG_OIL_PRESSURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].oil_pressure},
        {"alternatorVoltage", ENGINE_SET, ENG_ALTERNATOR_VOLTAGE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].alternator_voltage},
        {"runTime", ENGINE_SET, ENG_TOTAL_HOURS_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].total_hours},
        {"coolantTemperature", ENGINE_SET, ENG_COOLANT_TEMPERATURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].coolant_temperature},
        {"coolantPressure", ENGINE_SET, ENG_COOLANT_PRESSURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].coolant_pressure},
        {"engineTorque", ENGINE_SET, ENG_TORQUE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].torque},
        {"fuel.rate", ENGINE_SET, ENG_FUEL_RATE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].fuel_rate},
        {"fuel.pressure", ENGINE_SET, ENG_FUEL_PRESSURE_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].fuel_pressure},
        {"drive.trimState", ENGINE_SET, ENG_TILT_PSET, 1.0,
         &device->gpsdata.engine.instance[einstance].tilt},
    };
    
    memcpy(path_updates, pus, sizeof(pus));
}


gps_mask_t signalk_update_dump(struct gps_device_t *device,
                               const struct vessel_t * vessel,
                               /*@out@*/ char reply[], size_t replylen)
{
    gps_mask_t reported = 0;

    (void)strlcpy(reply, "{\"updates\":[{", replylen);

    /* in case we deal with a fix we also take the
       fix timestamp
       we hope that will not flicker too much with system time
       which should have been set correctly by GPS time anyways
    */
    if(((device->gpsdata.navigation.set & LATLON_SET) != 0)
       && (device->gpsdata.fix.mode > MODE_NO_FIX)) {
        signalk_add_fixtimestamp(device, reply, replylen);
    } else {
        signalk_add_timestamp(reply, replylen);
    }

    (void)strlcat(reply, ",\"values\":[", replylen);

    // add actual values
    uint16_t pu = 0, pt = 0;

    const struct signalk_path_t path_updates[] = {
        {"navigation.rateOfTurn", NAVIGATION_SET, NAV_ROT_PSET, 1.0,
         &device->gpsdata.navigation.rate_of_turn},
        {"navigation.courseOverGroundMagnetic", NAVIGATION_SET, NAV_COG_MAGN_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.navigation.course_over_ground[compass_magnetic]},
        {"navigation.courseOverGroundTrue", NAVIGATION_SET, NAV_COG_TRUE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.navigation.course_over_ground[compass_true]},
        {"navigation.magneticVariation", ENVIRONMENT_SET, ENV_VARIATION_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.environment.variation},
        {"navigation.headingMagnetic", NAVIGATION_SET, NAV_HDG_MAGN_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.navigation.heading[compass_magnetic]},
        {"navigation.headingTrue", NAVIGATION_SET, NAV_HDG_TRUE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.navigation.heading[compass_true]},
        {"navigation.speedOverGround", NAVIGATION_SET, NAV_SOG_PSET, 1.0*KNOTS_TO_MPS,
         &device->gpsdata.navigation.speed_over_ground},
        {"navigation.speedThroughWater", NAVIGATION_SET, NAV_STW_PSET, 1.0*KNOTS_TO_MPS,
         &device->gpsdata.navigation.speed_thru_water},
        {"navigation.log", NAVIGATION_SET, NAV_DIST_TOT_PSET, 1.0,
         &device->gpsdata.navigation.distance_total},
        {"navigation.logTrip", NAVIGATION_SET, NAV_DIST_TRIP_PSET, 1.0,
         &device->gpsdata.navigation.distance_trip},
        {"environment.depth.belowTransducer", NAVIGATION_SET, NAV_DPT_PSET, 1.0,
         &device->gpsdata.navigation.depth},
        {"environment.depth.surfaceToTransducer", NAVIGATION_SET, NAV_DPT_PSET, 1.0,
         &device->gpsdata.navigation.depth_offset},
        {"environment.wind.angleApparent",
         ENVIRONMENT_SET, ENV_WIND_APPARENT_ANGLE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.environment.wind[wind_apparent].angle},
        {"environment.wind.speedApparent",
           ENVIRONMENT_SET, ENV_WIND_APPARENT_SPEED_PSET, 1.0,
         &device->gpsdata.environment.wind[wind_apparent].speed},
        /* 'True' wind angle, -180 to +180 degrees from the bow. Negative numbers to port */
        {"environment.wind.speedTrue",
         ENVIRONMENT_SET, ENV_WIND_TRUE_TO_BOAT_SPEED_PSET, 1.0,
         &device->gpsdata.environment.wind[wind_true_to_boat].speed},
        {"environment.wind.angleTrueWater",
         ENVIRONMENT_SET, ENV_WIND_TRUE_TO_BOAT_ANGLE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.environment.wind[wind_true_to_boat].angle},
        {"environment.wind.speedOverGround",
         ENVIRONMENT_SET,  ENV_WIND_TRUE_NORTH_SPEED_PSET, 1.0,
         &device->gpsdata.environment.wind[wind_true_north].speed},
        /* The wind direction relative to true north, in compass degrees, 0 = North */
        {"environment.wind.directionTrue",
         ENVIRONMENT_SET, ENV_WIND_TRUE_NORTH_ANGLE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.environment.wind[wind_true_north].angle},
        {"environment.wind.directionMagnetic",
         ENVIRONMENT_SET, ENV_WIND_MAGN_ANGLE_PSET, 1.0*DEG_2_RAD,
         &device->gpsdata.environment.wind[wind_magnetic_north].angle},
        {"environment.waterTemp", ENVIRONMENT_SET, ENV_TEMP_WATER_PSET, 1.0,
         &device->gpsdata.environment.temp[temp_water]},
    };


    struct signalk_path_t path_updates_engine[2][13];
    signalk_update_engine_struct(device, path_updates_engine[single_or_double_port],
                                 single_or_double_port);
    signalk_update_engine_struct(device, path_updates_engine[starboard], starboard);


    for(pu = 0; pu < 19; pu++) {
        if(!isnan(*(double *)path_updates[pu].real)) {
            if ((device->gpsdata.set & path_updates[pu].mask) != 0) {
                if( (((device->gpsdata.navigation.set & path_updates[pu].submask) != 0)
                     && (path_updates[pu].mask & NAVIGATION_SET))
                    || (((device->gpsdata.environment.set & path_updates[pu].submask) != 0)
                        && (path_updates[pu].mask & ENVIRONMENT_SET)) ) {

                    if(pt > 0)
                        (void)strlcat(reply, ",{", replylen);
                    else
                        (void)strlcat(reply, "{", replylen);

                    (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                                   "\"path\":\"%s\",\"value\":%.2f}",
                                   path_updates[pu].path,
                                   *(double *)path_updates[pu].real
                                   * path_updates[pu].factor);
                    reported |= path_updates[pu].mask;
                    pt++;
                }
            }
        }
    }

    /* position is different and will be handled here */
    if((device->gpsdata.set & LATLON_SET) != 0) {
        if (device->gpsdata.fix.mode > MODE_NO_FIX) {
            if(pt > 0)
                (void)strlcat(reply, ",{", replylen);
            else
                (void)strlcat(reply, "{", replylen);
            pt++;

            (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                           "\"path\":\"navigation.position\",\"value\":{\"longitude\":%f,\"latitude\":%f}}",
                           device->gpsdata.fix.longitude,
                           device->gpsdata.fix.latitude);
            reported |= LATLON_SET;
        }
    }
    /* adding attitude */
    if((device->gpsdata.set & ATTITUDE_SET)
       && !isnan(device->gpsdata.attitude.roll)) {
            if(pt > 0)
                (void)strlcat(reply, ",{", replylen);
            else
                (void)strlcat(reply, "{", replylen);
            pt++;

            (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                           "\"path\":\"navigation.attitude\",\"value\":{\"roll\":%f}}",
                           device->gpsdata.attitude.roll);
            reported |= ATTITUDE_SET;
    }

    for(pu = 0; pu < 13; pu++) {
        int inst = single_or_double_port;
        if(!isnan(*(double *)path_updates_engine[inst][pu].real)) {
            if ((device->gpsdata.set & ENGINE_SET) != 0) {
                if((device->gpsdata.engine.set & path_updates_engine[inst][pu].submask) != 0) {

                    if(pt > 0)
                        (void)strlcat(reply, ",{", replylen);
                    else
                        (void)strlcat(reply, "{", replylen);

                    (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                               "\"path\":\"propulsion.port_engine.%s\",\"value\":%.2f}",
                               path_updates_engine[inst][pu].path,
                               *(double *)path_updates_engine[inst][pu].real
                               * path_updates_engine[inst][pu].factor);
                    reported |= ENGINE_SET;
                    pt++;
                }
            }
        }
    }


    (void)strlcat(reply, "]}", replylen); // close values
    (void)strlcat(reply, "],", replylen); // close updates
    if(vessel->mmsi != 0)
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                       "\"context\":\"vessels.urn:mrn:imo:mmsi:%u9\"", vessel->mmsi);
    else
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
                       "\"context\":\"vessels.urn:mrn:signalk:uuid:%s\"", vessel->uuid);

    (void)strlcat(reply, "}", replylen);

    return reported;
}
