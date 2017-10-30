#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

#include "gps.h"
#include "gpsd.h"
#include "utils.h"
#include "frame.h"

#include "pseudon2k.h"

/**
 *  \file pseudon2k.c contains all functions around creating N2K sentences
 *
 *  @page vy_topgns PGNs generated
 *  @brief This page shows all PGNs that are used to generate N2K sentences.
 */

/**
 *  \TOPGN 126992: GNSS System Time
 *
 *  Only reported if we have a fix.
 *
 *  \todo fix type of time source - only used GPS here
*/
void n2k_binary_126992_dump(struct gps_device_t *session, uint32_t *pgn,
                                  uint8_t bu[], size_t len, uint16_t * outlen) {

    uint16_t days = 0;

    if(len < 8) return;

    bu[0] = 0x14; // some sid

    if (session->newdata.mode > MODE_NO_FIX) {

        bu[0] = 0x14; // some sid
        bu[1] = 0; // since we probably don't know we'll use 0 for GPS here

        if (!isnan(session->newdata.time)) {
            days = (uint16_t)(session->newdata.time/(24*60*60));
            set8leu16(bu, days, 2);
            set8leu32(bu, (uint32_t)((session->newdata.time - days*24*60*60)*1e4), 4);
        }
    }

    *outlen = 8;
    *pgn = 126992;
}

/**
 *   \TOPGN 127250: NAV Vessel Heading
 */
void n2k_binary_127250_dump(struct gps_device_t *session, uint32_t *pgn,
                         enum compass_t compass,
                         uint8_t bu[], size_t len, uint16_t * outlen)
{

    if(len < 8) return;

    bu[0] = 0x14; // some sid

    if (!isnan(session->gpsdata.navigation.heading[compass]))
        set8les16(bu,
                  session->gpsdata.navigation.heading[compass] / 0.0001 / RAD_2_DEG, 1);
    else
        set8les16(bu, 0xffff, 1);

    if (!isnan(session->gpsdata.environment.deviation))
        set8les16(bu,
                  session->gpsdata.environment.deviation / 0.0001 / RAD_2_DEG, 3);
    else
        set8les16(bu, 0xffff, 3);

    if (!isnan(session->gpsdata.environment.variation))
        set8les16(bu,
                  session->gpsdata.environment.variation / 0.0001 / RAD_2_DEG,
                  5);

    else
        set8les16(bu, 0xffff, 5);

    // 1: magnetic, 0: true
    bu[7] = compass;

  *outlen = 8;
  *pgn = 127250;
}

void n2k_binary_hdg_magnetic_dump(struct gps_device_t *session, uint32_t *pgn,
                         uint8_t bu[], size_t len, uint16_t * outlen)
{
    n2k_binary_127250_dump(session, pgn,
                         compass_magnetic,
                        bu, len, outlen);
}

void n2k_binary_hdg_true_dump(struct gps_device_t *session, uint32_t *pgn,
                         uint8_t bu[], size_t len, uint16_t * outlen)
{
    n2k_binary_127250_dump(session, pgn,
                         compass_true,
                        bu, len, outlen);
}

/**
 *  \todo PGN 127251: Rate of Turn
 *
 *  NAVIGATION_SET: NAV_ROT_PSET
 */
void n2k_binary_127251_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen) {

    if(len < 8) return;

    bu[0] = 0x14; // some sid

    if (!isnan(session->gpsdata.navigation.rate_of_turn)) {
        // TODO : find explain for odd magic number 3/16 deg/min (or 3/16/60 in deg/sec)
        set8les32(bu, session->gpsdata.navigation.rate_of_turn
                  / (0.00001 * 3.0/16.0/60.0 * RAD_2_DEG), 1);
                  // per sec, let compiler optimize
    }

    *outlen = 8;
    *pgn = 127251;
}


/**
 *   \TOPGN 127257: Attitude
 */
void n2k_binary_attitude_dump(struct gps_device_t *session,uint32_t *pgn,
                                uint8_t bu[], size_t len, uint16_t * outlen)
{
    if(len < 8) return;

    bu[0] = 0x14; // some sid

    if(!(session->gpsdata.set & ATTITUDE_SET)) {
        *outlen = 0;
        return;
    }

    if (!isnan(session->gpsdata.attitude.yaw))
        set8les16(bu,
                  session->gpsdata.attitude.yaw / 0.0001 / RAD_2_DEG, 1);
    else
        set8les16(bu, 0xffff, 1);

    if (!isnan(session->gpsdata.attitude.pitch))
        set8les16(bu,
                  session->gpsdata.attitude.pitch / 0.0001 / RAD_2_DEG, 3);
    else
        set8les16(bu, 0xffff, 3);

    if (!isnan(session->gpsdata.attitude.roll))
        set8les16(bu,
                  session->gpsdata.attitude.roll / 0.0001 / RAD_2_DEG, 5);
    else
        set8les16(bu, 0xffff, 5);

    *outlen = 8;
    *pgn = 127257;

}

/**
 *  \todo PGN 127258: GNSS Magnetic Variation
 */
void n2k_binary_127258_dump(struct gps_device_t *session UNUSED, uint32_t *pgn,
                            uint8_t bu[] UNUSED, size_t len UNUSED, uint16_t * outlen) {
    *outlen = 8;
    *pgn = 127258;
}

/**
 *  \todo PGN 128259: NAV Speed
 */
void n2k_binary_127259_dump(struct gps_device_t *session UNUSED, uint32_t *pgn,
                            uint8_t bu[] UNUSED, size_t len UNUSED, uint16_t * outlen) {
    *outlen = 8;
    *pgn = 127259;
}

/*
 *   \todo PGN 127488: Engine Parameter Rapid Update
 */
void n2k_binary_127488_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen)
{
    uint8_t instance = 0;

    if(len < 8) return;


    // engine instance
    if(session->gpsdata.engine.set & ENG_PORT_PSET) {
        instance = 0;
    } else if(session->gpsdata.engine.set & ENG_STARBOARD_PSET) {
        instance = 1;
    } else
        return;

    bu[0] = instance;

    if(session->gpsdata.engine.set & ENG_SPEED_PSET)
        set8leu16(bu, session->gpsdata.engine.instance[instance].speed/0.25, 1);
    else
        set8leu16(bu, 0xffff, 1);


    if(session->gpsdata.engine.set & ENG_BOOST_PRESSURE_PSET)
        set8leu16(bu, session->gpsdata.engine.instance[instance].boost_pressure/100.0, 3);
    else
        set8leu16(bu, 0xffff, 3);

    if(session->gpsdata.engine.set & ENG_TILT_PSET)
        set8leu8(bu, session->gpsdata.engine.instance[instance].tilt, 6);
    else
        set8leu8(bu, 0xff, 6);

    *outlen = 8;
    *pgn = 127488;
}

/*
 *   \todo PGN 127489: Engine Parameter Dynamic
 */
void n2k_binary_127489_dump(struct gps_device_t *session UNUSED, uint32_t *pgn,
                            uint8_t bu[] UNUSED, size_t len, uint16_t * outlen)
{
    if(len < 8) return;



    *outlen = 8;
    *pgn = 127489;
}

/**
 *   \TOPGN 128267: NAV Water Depth
 */
void n2k_binary_128267_dump(struct gps_device_t *session, uint32_t *pgn,
                                uint8_t bu[], size_t len, uint16_t * outlen)
{
    if(len < 8) return;

    bu[0] = 0x14;

    if (!isnan(session->gpsdata.navigation.depth))
        set8leu32(bu, (uint32_t)(session->gpsdata.navigation.depth/.01), 1);
    else
        set8leu32(bu, (uint32_t)(0xffffffff), 1);

    if (!isnan(session->gpsdata.navigation.depth_offset))
        set8les16(bu, (int16_t)(session->gpsdata.navigation.depth_offset/.001), 5);
    else
        set8les16(bu, (int16_t)(0x7fff), 5);

    *outlen = 8;
    *pgn = 128267;
}

/**
 *  \TOPGN 128275: NAV Distance Log
 */
void n2k_binary_128275_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {

    set8leu16(bu, (uint16_t)0xffff, 0);  /// \todo date
    set8leu32(bu, (uint32_t)0xffffffff, 2); /// \todo time

    if (!isnan(session->gpsdata.navigation.distance_total))
        set8leu32(bu, (uint32_t)(session->gpsdata.navigation.distance_total/METERS_TO_NM), 6);
    else
        set8leu32(bu, (uint32_t)(0xffffffff), 6);

    if (!isnan(session->gpsdata.navigation.distance_trip))
        set8leu32(bu, (uint32_t)(session->gpsdata.navigation.distance_trip/METERS_TO_NM), 10);
    else
        set8leu32(bu, (uint32_t)(0xffffffff), 10);


    *outlen = 14;
    *pgn = 128275;
}


/**
 *  \TOPGN 129025: GNSS Position Rapid Update
 */
void n2k_binary_129025_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {

    if (!isnan(session->newdata.latitude)) {
        set8les32(bu, session->newdata.latitude/1e-7, 0);
    } else {
        set8les32(bu, 0x7fffffff, 0);
    }
    if (!isnan(session->newdata.longitude)) {
        set8les32(bu, session->newdata.longitude/1e-7, 4);
    } else {
        set8les32(bu, 0x7fffffff, 4);
    }

    *outlen = 8;
    *pgn = 129025;
}


/**
 *  \todo PGN 129026: GNSS COG and SOG Rapid Update
 */
void n2k_binary_129026_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen) {

    if(len < 8) return;
    bu[0] = 0x14;

    if(session->gpsdata.navigation.set  & NAV_COG_TRUE_PSET) {
        if(!isnan(session->gpsdata.navigation.course_over_ground[compass_true])) {
            bu[1] = (0 << 6) | 0x3f; // 0x3f is probably always there, 0 = true 
            set8leu16(bu, session->gpsdata.navigation.course_over_ground[compass_true] / 1e-4 / RAD_2_DEG, 2);
        }
    } else if(session->gpsdata.navigation.set  & NAV_COG_MAGN_PSET) {
        if(!isnan(session->gpsdata.navigation.course_over_ground[compass_magnetic])) {
            bu[1] = (0 << 6) | 0x3f; // 0x3f is probably always there, 1 = magnetic 
            set8leu16(bu, session->gpsdata.navigation.course_over_ground[compass_magnetic] / 1e-4 / RAD_2_DEG, 4);
        }
    }

    set8leu16(bu, 0xffff, 6);

    *outlen = 8;
    *pgn = 129026;
}


/**
 *  \todo 129029: GNSS Positition Data
 */
void n2k_binary_129029_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen) {

    if(len < 8) return;
    bu[0] = 0x14;

    if (!isnan(session->newdata.time)) {
        uint16_t days = (uint16_t)(session->newdata.time/(24*60*60));
        set8leu16(bu, days, 1);
        set8leu32(bu, (uint32_t)((session->newdata.time - days*24*60*60)*1e4), 3);
    }

    if (!isnan(session->newdata.latitude) && !isnan(session->newdata.longitude)) {
        set8les64(bu, session->newdata.latitude / 1e-16, 7);
        set8les64(bu, session->newdata.longitude / 1e-16, 15);
    } else {
        set8les64(bu, 0x7fffffffffffffff, 7);
        set8les64(bu, 0x7fffffffffffffff, 15);
    }

    if (!isnan(session->newdata.altitude)) {
        set8les64(bu, session->newdata.altitude / 1e-16, 23);
    } else {
        set8les64(bu, 0x7fffffffffffffff, 23);
    }

    // no fix is default

    /// \todo GPS100 says =no GPS, 1=GNSS fix, 2=DGNSS fix, 6=Estimated (dead reckoning).
    /* can't obviously set 3, 4 ,5 as it also maps to STATUS_FIX */
    switch (session->gpsdata.status) {
        case STATUS_NO_FIX:
            break;
        case STATUS_FIX:
            bu[31] = (1 & 0x0f) << 4;
            break;
        case STATUS_DGPS_FIX:
            bu[31] = (2 & 0x0f) << 4;
            break;
        default:
            bu[31] = (0 & 0x0f) << 4;    // STATUS_NO_FIX
            break;
    }

    if(session->gpsdata.satellites_used > 0) {
        bu[33] = session->gpsdata.satellites_used;
    } else {
        bu[33] = 0xff;
    }

    if (!isnan(session->gpsdata.dop.hdop) && !isnan(session->gpsdata.dop.pdop)) {
        set8les16(bu, session->gpsdata.dop.hdop/0.01, 34);
        set8les16(bu, session->gpsdata.dop.pdop/0.01, 36);
    } else {
        set8les16(bu, 0x7fff, 34);
        set8les16(bu, 0x7fff, 36);
    }

    if (!isnan(session->gpsdata.separation)) {
        set8les16(bu, session->gpsdata.separation*100.0, 38);
    } else {
        set8les16(bu, 0x7fff, 38);
    }

    *outlen = 42;
    *pgn = 129029;
}

/**
 *  \todo PGN 129033: Time & Date
 */
void n2k_binary_129033_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {

    if((session->gpsdata.set & TIME_SET) && !isnan(session->newdata.time)) {
        uint16_t days = floor(session->newdata.time/24/60/60);
        uint32_t secs = (session->newdata.time - days*24*60*60)*1e4;

        set8leu16(bu, days, 0);
        set8leu32(bu, secs, 2);
        set8leu32(bu, 0xffff, 6);

        *outlen = 8;
        *pgn = 129033;
    }
}

/**
 *  \todo PGN 129283: NAV Cross Track Error
 */
void n2k_binary_129283_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {
    *outlen = 8;
    bu[0] = 0x14;
    bu[1] = 0xff;

    if (!isnan(session->gpsdata.waypoint.xte)) {
        set8les32(bu, session->gpsdata.waypoint.xte * 100.0, 2);
    } else {
        set8les32(bu, 0x7fff, 2);
    }

    *pgn = 129283;
}

/**
 *  \todo PGN 129284: NAV Navdata to waypoint
 */
void n2k_binary_129284_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {

    uint8_t pc = 0x03, ce = 0x03;

    if(session->gpsdata.waypoint.arrival_status & waypoint_not_arrived) {
        pc = 0x00; ce = 0x00;
    } else {
        if(session->gpsdata.waypoint.arrival_status & waypoint_perpendicular_crossed) {
            pc = 0x01;
        }
        if(session->gpsdata.waypoint.arrival_status & waypoint_circle_entered) {
            ce = 0x01;
        }
    }

    bu[0] = 0x14;

    // distance to wp
    set8leu32(bu, (isnan(session->gpsdata.waypoint.range_to_destination)?0xffffffff:session->gpsdata.waypoint.range_to_destination), 1);

    bu[5] = 0x00 | (pc << 2) | (ce << 4) | (0x03 << 6); // we are 0=true north, and we don't know calculation type

    if(!isnan(session->gpsdata.waypoint.eta)) {
        uint16_t days = floor(session->gpsdata.waypoint.eta/24/60/60);
        uint32_t secs = (session->gpsdata.waypoint.eta - days*24*60*60)*1e4;

        set8leu32(bu, secs, 6);   // eta time
        set8leu16(bu, days,  10);      // eta date
    } else {
        set8leu32(bu, 0xffffffff, 6);   // eta time
        set8leu16(bu, 0xffff,  10);      // eta date
    }

    set8leu16(bu, (isnan(session->gpsdata.waypoint.bearing_from_org_to_destination)?0xffff:session->gpsdata.waypoint.bearing_from_org_to_destination/(0.0001 * RAD_2_DEG)), 12);
    set8leu16(bu, (isnan(session->gpsdata.waypoint.bearing_from_pos_to_destination)?0xffff:session->gpsdata.waypoint.bearing_from_pos_to_destination/(0.0001 * RAD_2_DEG)), 14);

    // origin and destination waypoint name/id
    //set8leu32(bu, session->gpsdata.waypoint.active_from, 16);  // org wp
    //set8leu32(bu, session->gpsdata.waypoint.active_to, 20);  // dest wp
    // TODO
    set8leu32(bu, 0xffffffff, 16);  // org wp
    set8leu32(bu, 0xffffffff, 20);  // org wp


    // dest lat
    if (!isnan(session->gpsdata.waypoint.latitude)) {
        set8les32(bu, session->gpsdata.waypoint.latitude/1e-7, 24);
    } else {
        set8les32(bu, 0x7fffffff, 24);
    }

    // dest lon
    if (!isnan(session->gpsdata.waypoint.longitude)) {
        set8les32(bu, session->gpsdata.waypoint.longitude/1e-7, 28);
    } else {
        set8les32(bu, 0x7fffffff, 28);
    }

    set8les16(bu, (isnan(session->gpsdata.waypoint.speed_to_destination)?0x7fff:session->gpsdata.waypoint.speed_to_destination/(0.01)), 32);

    *pgn = 129284;
    *outlen = 34;
}

/**
 *  \TOPGN 129539: GNSS DOPs
 */
void n2k_binary_129539_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {
    uint8_t req_mode;
    uint8_t act_mode;

    switch(session->gpsdata.fix.mode) {
        case MODE_NO_FIX:
            req_mode = act_mode = 0;
            break;

        case MODE_2D:
            req_mode = act_mode = 1;
            break;

        case MODE_3D:
            req_mode = act_mode = 2;
            break;

        default:
            req_mode = act_mode = 0;

    }

    bu[0] = 0x14;

    /** \todo for better translation we should actually store the
     requested set mode (req_mode) in the gpsdata structure
     as its also used in 0183 */
    bu[1] = ((req_mode & 0x07) << 0) | ((act_mode & 0x07) << 3);

    set8les16(bu, session->gpsdata.dop.hdop / 1e-2, 2);
    set8les16(bu, session->gpsdata.dop.vdop / 1e-2, 4);
    set8les16(bu, session->gpsdata.dop.tdop / 1e-2, 6);

    gpsd_report(session->context->debug, LOG_IO,
                "                   hdop:%5.2f vdop:%5.2f tdop:%5.2f req_mode:%u act_mode:%u\n",
                session->gpsdata.dop.hdop,
                session->gpsdata.dop.vdop,
                session->gpsdata.dop.tdop, req_mode, act_mode);

    *outlen = 8;
    *pgn = 129539;
}

/**
 *  \todo PGN 129540: GNSS Satellites in View
 */
void n2k_binary_129540_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen) {
    int         l1, l2;

    bu[0] = 0x14;
    bu[2] = session->gpsdata.satellites_visible;

    l2 = 0;
    for (l1=0; l1 < session->gpsdata.satellites_visible; l1++) {

        bu[3+12*l1+0] = session->gpsdata.PRN[l1];

        set8les16(bu, session->gpsdata.elevation[l1] / 1e-4 / RAD_2_DEG, 3+12*l1+1);
        set8leu16(bu, session->gpsdata.azimuth[l1] / 1e-4 / RAD_2_DEG, 3+12*l1+3);
        set8les16(bu, session->gpsdata.ss[l1] / 1e-2, 3+12*l1+5);

        bu[3+12*l1+11] = 0x0f;
        /** \todo brute force loop without even early exit */
        for(l2 = 0; l2 < MAXCHANNELS; l2++) {
            if(session->gpsdata.used[l2] == session->gpsdata.PRN[l1]) {
                /** \todo for the svt field it could be also 5 - setting 2 here for now */
                bu[3+12*l1+11] = 2;
            }
        }
    }

    *outlen = 3+12*session->gpsdata.satellites_visible+11;
    *pgn = 129540;
}


/**
 *   \TOPGN 130306: Wind
 */
void n2k_binary_130306_dump(struct gps_device_t *session, enum wind_reference_t wr, uint32_t *pgn,
                                uint8_t bu[], size_t len, uint16_t * outlen)
{
    *pgn = 0;

    if(len < 8) return;

    if(!(session->gpsdata.set & ENVIRONMENT_SET)) {
        *outlen = 0;
        return;
    }

    /*  0=True North,
        1=Magnetic North,
        2=Apparent,
        3=True to boat,
        4=True to water */

        double angle = session->gpsdata.environment.wind[wr].angle;
        double speed = session->gpsdata.environment.wind[wr].speed;

    bu[0] = 0x14;
    set8leu16(bu, (uint16_t)(!isnan(speed)?speed/.01:0xffff), 1);
    set8leu16(bu, (uint16_t)(!isnan(angle)?angle/RAD_2_DEG/0.0001:angle), 3);
    bu[5] = wr; // setting the apparent wind flag
    bu[6] = 0x00;

    *outlen = 8;
    *pgn = 130306;
}

/**
 *  \todo PGN 130311: NAV Environmental Parameters
 */
void n2k_binary_130311_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen) {
    if(len < 8) return;

    if(!(session->gpsdata.set & ENVIRONMENT_SET)) {
        *outlen = 0;
        return;
    }

    bu[0] = 0x14;
    bu[1] = 0x00;
    set8leu16(bu, 0xffff, 2);

    if(session->gpsdata.environment.set & ENV_TEMP_WATER_PSET) {
        set8leu16(bu, session->gpsdata.environment.temp[temp_water]/0.01, 2); // =  temp * 0.01 - 273.15;
        bu[1] = 0 & (0x01 << 6); // temp inst water, obviously unnecessary bit op - just for doc
    } else if(session->gpsdata.environment.set & ENV_TEMP_AIR_PSET) {
        set8leu16(bu, session->gpsdata.environment.temp[temp_air]/0.01, 2); // =  temp * 0.01 - 273.15;
        bu[1] = 1 & (0x01 << 6); // temp inst water, obviously unnecessary bit op - just for doc
    }

    // bu[1] |= xxx & (0x01 << 2);

    set8leu16(bu, 0xffff, 4); // hum
    set8leu16(bu, 0xffff, 6); // pres

    *outlen = 8;
    *pgn = 130311;
}

/**
 *  \todo PGN 130312: NAV Temperature
 */
void n2k_binary_130312_dump(struct gps_device_t *session UNUSED, uint32_t *pgn,
                            uint8_t bu[] UNUSED, size_t len UNUSED, uint16_t * outlen) {
    *outlen = 8;
    *pgn = 130312;
}

void n2k_129038_dump(struct gps_device_t *session, uint32_t *pgn,
                                uint8_t bu[], size_t len UNUSED, uint16_t * outlen)
{

    struct ais_t *ais = &session->gpsdata.ais;

    uint32_t channel = 2;

    if(session->driver.aivdm.ais_channel == 'B')
        channel = 1;

    // setting header
    bu[0] = (ais->type & 0x3f) | (ais->repeat & 0x03);
    set8leu32(bu, ais->mmsi, 1);

    switch (ais->type) {
    case 1:	/* Position Report */
    case 2:
    case 3:

        set8les32(bu, ais->type1.lon / 0.06, 5);
        set8les32(bu, ais->type1.lat / 0.06, 9);

        bu[13] = ((ais->type1.accuracy & 0x01)
                  | ((ais->type1.raim & 0x01) << 1)
                  | ((ais->type1.second & 0x3f) << 2));

        set8leu16(bu, ais->type1.course / RAD_2_DEG / 0.0001 / 10.0, 14);
        set8leu16(bu, ais->type1.speed, 16);

	    set8leu32(bu, (ais->type1.radio & 0x7ffff) | ((channel << (16 + 3)) & 0x1f), 18);

        set8leu16(bu, ais->type1.heading / RAD_2_DEG / 0.0001 / 1.0, 21);
        set8les16(bu, ais->type1.turn, 23);


        bu[25] = ais->type1.status;
        break;
    default:
        break;
    }

    *outlen = 26;
    *pgn = 129038;
}


/*
void n2k_navigation_dump(struct gps_device_t *session,
                         uint8_t bufp[], size_t len, uint16_t * outlen)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & NAVIGATION_SET) != 0) {

        if((session->gpsdata.navigation.set & NAV_STW_PSET) != 0) {}

        if(session->gpsdata.navigation.set
           & (NAV_SOG_PSET | NAV_COG_TRUE_PSET | NAV_COG_MAGN_PSET) != 0) {}

        if(((session->gpsdata.navigation.set & NAV_DIST_TOT_PSET) != 0)
           || ((session->gpsdata.navigation.set & NAV_DIST_TRIP_PSET) != 0)) {}

        if((session->gpsdata.navigation.set & NAV_DPT_PSET) != 0) {}

        if((session->gpsdata.navigation.set & NAV_HDG_MAGN_PSET) != 0)  {}

        if((session->gpsdata.navigation.set & NAV_HDG_TRUE_PSET) != 0) {}

        if((session->gpsdata.navigation.set & NAV_ROT_PSET) != 0) {}

        if((session->gpsdata.navigation.set & NAV_XTE_PSET) != 0) {}

        if((session->gpsdata.navigation.set & NAV_RUDDER_ANGLE_PSET) != 0) {}

    }
}

int n2k_environment_dump(struct gps_device_t *session,
                         uint8_t bufp[], size_t len, uint16_t * outlen)
{
    int ret = 1;

    if ((session->gpsdata.set & ENVIRONMENT_SET) != 0) {
      switch(session->gpsdata.environment.set) {

      case ENV_WIND_APPARENT_SPEED_PSET:
      case ENV_WIND_APPARENT_ANGLE_PSET:
        ret = 2;
	break;

      case ENV_WIND_TRUE_GROUND_SPEED_PSET:
      case ENV_WIND_TRUE_GROUND_ANGLE_PSET:
      case ENV_WIND_TRUE_WATER_SPEED_PSET:
      case ENV_WIND_TRUE_WATER_ANGLE_PSET:
	break;

      case ENV_TEMP_WATER_PSET:
	break;

      case ENV_TEMP_AIR_PSET:
      case ENV_VARIATION_PSET:
      case ENV_DEVIATION_PSET:
	break;

      default:
	break;
      };
    }

    return ret;
}
*/

int n2k_dump(struct gps_device_t *session, uint32_t pgn, uint8_t *bu, size_t len,
                    void (*write_handler)(struct gps_device_t *, enum frm_type_t, const uint8_t *, size_t)) {
    if(pgn) {
        set8leu32(bu, pgn, 0);
        bu[4] = 0x03; // prio
        bu[5] = session->driver.nmea2000.own_src_id;
        bu[6] = 0xff; // usually broadcast

        write_handler(session, FRM_TYPE_NMEA2000, bu, len + 7);
    }

    return 0;
}
int n2k_binary_dump(gps_mask_t changed,
                    struct gps_device_t *session,
                    void (*write_handler)(struct gps_device_t *, enum frm_type_t, const uint8_t *buf, size_t len))
{
    uint16_t written;
    uint16_t len = 280;
    uint8_t bu[280];
    uint32_t pgn;
    int i = 0;

    // get a copy of the masks to tick off
    gps_mask_t mask = changed,
        navmask = session->gpsdata.navigation.set,
        wpymask = session->gpsdata.waypoint.set,
        envmask = session->gpsdata.environment.set;

    // RMC 126992, 127250*, 127258, 129025, 129026, 129029, 129033
    if ((mask & TIME_SET) != 0) {

        if (session->newdata.mode > MODE_NO_FIX) {
            n2k_binary_126992_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        } else {
            n2k_binary_129033_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }

    }

    if((mask & LATLON_SET) != 0) {

        n2k_binary_129025_dump(session, &pgn, bu+7, len-7, &written);
        n2k_dump(session, pgn, bu, written, write_handler);

        n2k_binary_129029_dump(session, &pgn, bu+7, len-7, &written);
        n2k_dump(session, pgn, bu, written, write_handler);

    }

    if ((mask & NAVIGATION_SET) != 0) {

        if((navmask & NAV_DPT_PSET) != 0) {
            n2k_binary_128267_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }
        if((navmask & NAV_ROT_PSET) != 0) {
            n2k_binary_127251_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }
        if( (navmask & (NAV_COG_TRUE_PSET | NAV_COG_MAGN_PSET | NAV_SOG_PSET)) != 0 ) {
            n2k_binary_129026_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }

        if(navmask & NAV_HDG_MAGN_PSET) {
            n2k_binary_hdg_magnetic_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }

        if(navmask & NAV_HDG_TRUE_PSET) {
            n2k_binary_hdg_true_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }
    }

    if(mask & WAYPOINT_SET) {
        if(wpymask & (WPY_XTE_PSET | WPY_ARRIVAL_STATUS_PSET)) {
            n2k_binary_129283_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }
        if(wpymask & (WPY_ACTIVE_TO_PSET | WPY_ACTIVE_FROM_PSET
            | WPY_LATLON_TO_PSET | WPY_RANGE_TO_PSET | WPY_BEARING_FROM_ORG_TO_PSET | WPY_BEARING_FROM_POS_TO_PSET
            | WPY_SPEED_FROM_ORG_TO_PSET | WPY_ARRIVAL_STATUS_PSET | WPY_ETA_PSET)) {
            n2k_binary_129284_dump(session, &pgn, bu+7, len-7, &written);
            n2k_dump(session, pgn, bu, written, write_handler);
        }
    }

    if (mask & ENVIRONMENT_SET) {
        if(!(navmask & (NAV_HDG_TRUE_PSET | NAV_HDG_MAGN_PSET))) {
            // if neither magn or true heading is set but we do have
            // variation/deviation then we want to report it here
            if((mask & ENVIRONMENT_SET) & (envmask & (ENV_DEVIATION_PSET | ENV_VARIATION_PSET))) {
                n2k_binary_hdg_true_dump(session, &pgn, bu+7, len-7, &written);
                n2k_dump(session, pgn, bu, written, write_handler);
            }
        }


        static gps_mask_t angles[] = {
            ENV_WIND_TRUE_NORTH_ANGLE_PSET,
            ENV_WIND_MAGN_ANGLE_PSET,
            ENV_WIND_APPARENT_ANGLE_PSET,
            ENV_WIND_TRUE_TO_BOAT_ANGLE_PSET,
            ENV_WIND_TRUE_TO_WATER_ANGLE_PSET
        };

        static gps_mask_t speeds[] = {
            ENV_WIND_TRUE_NORTH_SPEED_PSET,
            ENV_WIND_MAGN_ANGLE_PSET,
            ENV_WIND_APPARENT_SPEED_PSET,
            ENV_WIND_TRUE_TO_BOAT_SPEED_PSET,
            ENV_WIND_TRUE_TO_WATER_SPEED_PSET
        };

        static enum wind_reference_t wrs[] = {
            wind_true_north,
            wind_magnetic_north,
            wind_apparent,
            wind_true_to_boat,
            wind_true_to_water
        };

        for(i = 0; i < 5; i++) {
            if((session->gpsdata.environment.set & angles[i])
               || (session->gpsdata.environment.set & speeds[i]) ) {
                    n2k_binary_130306_dump(session, wrs[i], &pgn, bu+7, len-7, &written);
                    n2k_dump(session, pgn, bu, written, write_handler);
                }
            }
        }
    return 0;

}
