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

/*
 * Support for generic binary drivers.  These functions dump NMEA for passing
 * to the client in raw mode.  They assume that (a) the public gps.h structure
 * members are in a valid state, (b) that the private members hours, minutes,
 * and seconds have also been filled in, (c) that if the private member
 * mag_var is not NAN it is a magnetic variation in degrees that should be
 * passed on, and (d) if the private member separation does not have the
 * value NAN, it is a valid WGS84 geoidal separation in meters for the fix.
 */

static double degtodm(double angle)
/* decimal degrees to GPS-style, degrees first followed by minutes */
{
    double fraction, integer;
    fraction = modf(angle, &integer);
    return floor(angle) * 100 + fraction * 60;
}

/*@ -mustdefine @*/
void gpsd_position_fix_dump(struct gps_device_t *session,
			    /*@out@*/ char bufp[], size_t len)
{
    struct tm tm;
    time_t intfixtime;

    intfixtime = (time_t) session->gpsdata.fix.time;
    (void)gmtime_r(&intfixtime, &tm);
    if (session->gpsdata.fix.mode > MODE_NO_FIX) {
	(void)snprintf(bufp, len,
		       "$GPGGA,%02d%02d%02d,%09.4f,%c,%010.4f,%c,%d,%02d,",
		       tm.tm_hour,
		       tm.tm_min,
		       tm.tm_sec,
		       degtodm(fabs(session->gpsdata.fix.latitude)),
		       ((session->gpsdata.fix.latitude > 0) ? 'N' : 'S'),
		       degtodm(fabs(session->gpsdata.fix.longitude)),
		       ((session->gpsdata.fix.longitude > 0) ? 'E' : 'W'),
		       session->gpsdata.status,
		       session->gpsdata.satellites_used);
	if (isnan(session->gpsdata.dop.hdop))
	    (void)strlcat(bufp, ",", len);
	else
	    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
			   "%.2f,", session->gpsdata.dop.hdop);
	if (isnan(session->gpsdata.fix.altitude))
	    (void)strlcat(bufp, ",", len);
	else
	    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
			   "%.2f,M,", session->gpsdata.fix.altitude);
	if (isnan(session->gpsdata.separation))
	    (void)strlcat(bufp, ",", len);
	else
	    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
			   "%.3f,M,", session->gpsdata.separation);
	if (isnan(session->mag_var))
	    (void)strlcat(bufp, ",", len);
	else {
	    (void)snprintf(bufp + strlen(bufp),
			   len - strlen(bufp),
			   "%3.2f,", fabs(session->mag_var));
	    (void)strlcat(bufp, (session->mag_var > 0) ? "E" : "W", len);
	}
	nmea_add_checksum(bufp);
    }
}

/*@ +mustdefine @*/

static void gpsd_transit_fix_dump(struct gps_device_t *session,
				  char bufp[], size_t len)
{
  /*
    === RMC - Recommended Minimum Navigation Information ===

    ------------------------------------------------------------------------------
                                                              12
            1         2 3       4 5        6  7   8   9    10 11|  13
            |         | |       | |        |  |   |   |    |  | |   |
     $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a,m,*hh<CR><LF>
    ------------------------------------------------------------------------------

    Field Number:

    1. UTC Time
    2. Status, V=Navigation receiver warning A=Valid
    3. Latitude
    4. N or S
    5. Longitude
    6. E or W
    7. Speed over ground, knots
    8. Track made good, degrees true
    9. Date, ddmmyy
    10. Magnetic Variation, degrees
    11. E or W
    12. FAA mode indicator (NMEA 2.3 and later)
    13. Checksum

    A status of V means the GPS has a valid fix that is below an internal
    quality threshold, e.g. because the dilution of precision is too high
    or an elevation mask test failed.
    */
    struct tm tm;
    time_t intfixtime;

    tm.tm_mday = tm.tm_mon = tm.tm_year = tm.tm_hour = tm.tm_min = tm.tm_sec =
	0;
    if (isnan(session->gpsdata.fix.time) == 0) {
	intfixtime = (time_t) session->gpsdata.fix.time;
	(void)gmtime_r(&intfixtime, &tm);
	tm.tm_mon++;
	tm.tm_year %= 100;
    }
#define ZEROIZE(x)	(isnan(x)!=0 ? 0.0 : x)
    /*@ -usedef @*/
    (void)snprintf(bufp, len,
		   "$GPRMC,%02d%02d%02d,%c,%09.4f,%c,%010.4f,%c,",
		   tm.tm_hour,
		   tm.tm_min,
		   tm.tm_sec,
		   session->gpsdata.status ? 'A' : 'V',
		   ZEROIZE(degtodm(fabs(session->gpsdata.fix.latitude))),
		   ((session->gpsdata.fix.latitude > 0) ? 'N' : 'S'),
		   ZEROIZE(degtodm(fabs(session->gpsdata.fix.longitude))),
		   ((session->gpsdata.fix.longitude > 0) ? 'E' : 'W'));

    if ( !isnan(session->gpsdata.navigation.speed_over_ground) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.4f,", session->gpsdata.navigation.speed_over_ground);
    } else {
      (void)strlcat(bufp, ",", len);
    }
    if ( !isnan(session->gpsdata.navigation.course_over_ground) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.3f,", session->gpsdata.navigation.course_over_ground);
    } else {
      (void)strlcat(bufp, ",", len);
    }

    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		   "%02d%02d%02d,,",
		   tm.tm_mday, tm.tm_mon, tm.tm_year);

    /*@ +usedef @*/
#undef ZEROIZE
    nmea_add_checksum(bufp);
}

static void gpsd_binary_satellite_dump(struct gps_device_t *session,
				       char bufp[], size_t len)
{
    int i;
    char *bufp2 = bufp;
    bufp[0] = '\0';

    for (i = 0; i < session->gpsdata.satellites_visible; i++) {
	if (i % 4 == 0) {
	    bufp += strlen(bufp);
	    bufp2 = bufp;
	    len -= snprintf(bufp, len,
			    "$GPGSV,%d,%d,%02d",
			    ((session->gpsdata.satellites_visible - 1) / 4) +
			    1, (i / 4) + 1,
			    session->gpsdata.satellites_visible);
	}
	bufp += strlen(bufp);
	if (i < session->gpsdata.satellites_visible)
	    len -= snprintf(bufp, len,
			    ",%02d,%02d,%03d,%02.0f",
			    session->gpsdata.PRN[i],
			    session->gpsdata.elevation[i],
			    session->gpsdata.azimuth[i],
			    session->gpsdata.ss[i]);
	if (i % 4 == 3 || i == session->gpsdata.satellites_visible - 1) {
	    nmea_add_checksum(bufp2);
	    len -= 5;
	}
    }

#ifdef ZODIAC_ENABLE
    if (session->packet.type == ZODIAC_PACKET
	&& session->driver.zodiac.Zs[0] != 0) {
	bufp += strlen(bufp);
	bufp2 = bufp;
	(void)strlcpy(bufp, "$PRWIZCH", len);
	for (i = 0; i < ZODIAC_CHANNELS; i++) {
	    len -= snprintf(bufp + strlen(bufp), len,
			    ",%02u,%X",
			    session->driver.zodiac.Zs[i],
			    session->driver.zodiac.Zv[i] & 0x0f);
	}
	nmea_add_checksum(bufp2);
    }
#endif /* ZODIAC_ENABLE */
}

static void gpsd_binary_quality_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{
    char *bufp2 = bufp;
    bool used_valid = (session->gpsdata.set & USED_IS) != 0;

    if (session->device_type != NULL && (session->gpsdata.set & MODE_SET) != 0) {
	int i, j;

	(void)snprintf(bufp, len - strlen(bufp),
		       "$GPGSA,%c,%d,", 'A', session->gpsdata.fix.mode);
	j = 0;
	for (i = 0; i < session->device_type->channels; i++) {
	    if (session->gpsdata.used[i]) {
		bufp += strlen(bufp);
		(void)snprintf(bufp, len - strlen(bufp),
			       "%02d,",
			       used_valid ? session->gpsdata.used[i] : 0);
		j++;
	    }
	}
	for (i = j; i < session->device_type->channels; i++) {
	    bufp += strlen(bufp);
	    (void)strlcpy(bufp, ",", len);
	}
	bufp += strlen(bufp);
#define ZEROIZE(x)	(isnan(x)!=0 ? 0.0 : x)
	if (session->gpsdata.fix.mode == MODE_NO_FIX)
	    (void)strlcat(bufp, ",,,", len);
	else
	    (void)snprintf(bufp, len - strlen(bufp),
			   "%.1f,%.1f,%.1f*",
			   ZEROIZE(session->gpsdata.dop.pdop),
			   ZEROIZE(session->gpsdata.dop.hdop),
			   ZEROIZE(session->gpsdata.dop.vdop));
	nmea_add_checksum(bufp2);
	bufp += strlen(bufp);
    }
    if (isfinite(session->gpsdata.fix.epx)!=0
	&& isfinite(session->gpsdata.fix.epy)!=0
	&& isfinite(session->gpsdata.fix.epv)!=0
	&& isfinite(session->gpsdata.epe)!=0) {
	struct tm tm;
	time_t intfixtime;

	tm.tm_hour = tm.tm_min = tm.tm_sec = 0;
	if (isnan(session->gpsdata.fix.time) == 0) {
	    intfixtime = (time_t) session->gpsdata.fix.time;
	    (void)gmtime_r(&intfixtime, &tm);
	}
	(void)snprintf(bufp, len - strlen(bufp),
		       "$GPGBS,%02d%02d%02d,%.2f,M,%.2f,M,%.2f,M",
		       tm.tm_hour, tm.tm_min, tm.tm_sec,
		       ZEROIZE(session->gpsdata.fix.epx),
		       ZEROIZE(session->gpsdata.fix.epy),
		       ZEROIZE(session->gpsdata.fix.epv));
	nmea_add_checksum(bufp);
    }
#undef ZEROIZE
}

static void gpsd_binary_time_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{
    struct tm tm;
    double integral;
    time_t integral_time;

    if (session->newdata.mode > MODE_NO_FIX) {
	double fractional = modf(session->newdata.time, &integral);
	integral_time = (time_t) integral;
	(void)gmtime_r(&integral_time, &tm);
	/*
	 * We pin this report to the GMT/UTC timezone.  This may be technically
	 * incorrect; our sources on ZDA suggest that it should report local
	 * timezone. But no GPS we've ever seen actually does this, because it
	 * would require embedding a location-to-TZ database in the receiver.
	 * And even if we could do that, it would make our regression tests
	 * break any time they were run in a timezone different from the one
	 * where they were generated.
	 */
	(void)snprintf(bufp, len,
		       "$GPZDA,%02d%02d%05.2f,%02d,%02d,%04d,00,00",
		       tm.tm_hour,
		       tm.tm_min,
		       (double)tm.tm_sec + fractional,
		       tm.tm_mday,
		       tm.tm_mon + 1,
		       tm.tm_year + 1900);
	nmea_add_checksum(bufp);
    }
}

static void gpsd_binary_almanac_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{
    if ( session->gpsdata.subframe.is_almanac ) {
	(void)snprintf(bufp, len,
			"$GPALM,1,1,%02d,%04d,%02x,%04x,%02x,%04x,%04x,%05x,%06x,%06x,%06x,%03x,%03x",
		       (int)session->gpsdata.subframe.sub5.almanac.sv,
		       (int)session->context->gps_week % 1024,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.svh,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.e,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.toa,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.deltai,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.Omegad,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.sqrtA,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.omega,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.Omega0,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.M0,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.af0,
		       (unsigned int)session->gpsdata.subframe.sub5.almanac.af1);
	nmea_add_checksum(bufp);
    }
}

#ifdef AIVDM_ENABLE

#define GETLEFT(a) (((a%6) == 0) ? 0 : (6 - (a%6))) 

static void gpsd_binary_ais_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{
    char type[8] = "!AIVDM";
    unsigned char data[256];
    unsigned int msg1, msg2;
    char numc[4];
    char channel;
    unsigned int left;
    unsigned int datalen;
    unsigned int offset;
  
    channel = 'A';
    if (session->driver.aivdm.ais_channel == 'B') {
        channel = 'B';
    }
 
    memset(data, 0, sizeof(data));
    datalen = ais_binary_encode(&session->gpsdata.ais, &data[0], 0);
    if (datalen > 6*60) {
	static int number1 = 0;
        msg1 = datalen / (6*60);
	if ((datalen % (6*60)) != 0) {
	    msg1 += 1;
	}
	/*@ +charint */
	numc[0] = '0' + (char)(number1 & 0x0f);
	/*@ -charint */
	numc[1] = '\0';
	number1 += 1;
	if (number1 > 9) {
	    number1 = 0;
	}
	offset = 0;
	for (msg2=1;msg2<=msg1;msg2++) {
	    unsigned char old;

	    old = '\0';
	    if (strlen((char *)&data[(msg2-1)*60]) > 60) {
	        old = data[(msg2-0)*60];
	        data[(msg2-0)*60] = '\0';
	    }
	    if (datalen >= (6*60)) {
	        left = 0;
		datalen -= 6*60;
	    } else {
	        left = GETLEFT(datalen);
	    }
	    (void)snprintf(&bufp[offset], len-offset,
			   "%s,%u,%u,%s,%c,%s,%u",
			   type,
			   msg1,
			   msg2,
			   numc,
			   channel,
			   (char *)&data[(msg2-1)*60],
			   left);

	    nmea_add_checksum(&bufp[offset]);
	    if (old != (unsigned char)'\0') {
		data[(msg2-0)*60] = old;
	    }
	    offset = (unsigned int) strlen(bufp);
	}
    } else {
        msg1 = 1;
	msg2 = 1;
	numc[0] = '\0';
        left = GETLEFT(datalen);
	(void)snprintf(bufp, len,
		       "%s,%u,%u,%s,%c,%s,%u",
		       type,
		       msg1,
		       msg2,
		       numc,
		       channel,
		       (char *)data,
		       left);

	nmea_add_checksum(bufp);
    }

    if (session->gpsdata.ais.type == 24) {
        msg1 = 1;
	msg2 = 1;
	numc[0] = '\0';

        memset(data, 0, sizeof(data));
	datalen = ais_binary_encode(&session->gpsdata.ais, &data[0], 1);
	left = GETLEFT(datalen);
	offset = (unsigned int)strlen(bufp);
	(void)snprintf(&bufp[offset], len-offset,
		       "%s,%u,%u,%s,%c,%s,%u",
		       type,
		       msg1,
		       msg2,
		       numc,
		       channel,
		       (char *)data,
		       left);
	nmea_add_checksum(bufp+offset);
    }
}
#endif /* AIVDM_ENABLE */

static void gpsd_binary_mwv_dump(struct gps_device_t *session,
                                     char bufp[], size_t len)
{
  // $--MWV,x.x,[R,T],x.x,[K/M/N]*hh<CR><LF>

  (void)snprintf(bufp, len, "$GPMWV,");

  if ( !isnan(session->gpsdata.environment.wind.apparent.angle) ) {
      double ang = session->gpsdata.environment.wind.apparent.angle;
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
                     "%.2f,%c,", ang, 'R');
  } else if ( !isnan(session->gpsdata.environment.wind.true_north.angle) ) {
      double ang = session->gpsdata.environment.wind.true_north.angle;
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
                     "%.2f,%c,", ang, 'T');
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  if ( !isnan(session->gpsdata.environment.wind.apparent.speed) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
                     "%.2f,N,", session->gpsdata.environment.wind.apparent.speed);
  } else if ( !isnan(session->gpsdata.environment.wind.true_north.speed) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
                     "%.2f,N,", session->gpsdata.environment.wind.true_north.speed);
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  (void)strlcat(bufp, "A", len);

  nmea_add_checksum(bufp);
}

static void gpsd_binary_vwr_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{
  // $--VWR,x.x,a,x.x,N,x.x,M,x.x,K*hh<CR><LF>

  (void)snprintf(bufp, len, "$GPVWR,");

  if ( !isnan(session->gpsdata.environment.wind.apparent.angle) ) {
      double ang = session->gpsdata.environment.wind.apparent.angle;
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,%c,", 
		     ang <= 180 ? ang:360.0 - ang,
		     ang <= 180 ? 'R':'L');
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  if ( !isnan(session->gpsdata.environment.wind.apparent.speed) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,N,", session->gpsdata.environment.wind.apparent.speed);
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  (void)strlcat(bufp, ",,,", len);

  nmea_add_checksum(bufp);
}

static void gpsd_binary_vtg_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--VTG,x.x,x,x.x,x.x,*hh<CR><LF>
  (void)snprintf(bufp, len, "$GPVTG,");

  if ( !isnan(session->gpsdata.navigation.course_over_ground) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,T,", session->gpsdata.navigation.course_over_ground);
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  // no magnetic at this point in time
  (void)strlcat(bufp, ",,", len);

  if ( !isnan(session->gpsdata.navigation.speed_over_ground) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,N,", session->gpsdata.navigation.speed_over_ground);
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,K", session->gpsdata.navigation.speed_over_ground * KNOTS_TO_KPH);
  } else {
      (void)strlcat(bufp, ",,", len);
      (void)strlcat(bufp, ",,", len);
  }
  nmea_add_checksum(bufp);
}

static void gpsd_binary_vhw_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>

  (void)snprintf(bufp, len, "$GPVHW,");

  if ( !isnan(session->gpsdata.navigation.heading[compass_true]) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,T,", 
		     session->gpsdata.navigation.heading[compass_true]);
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  if ( !isnan(session->gpsdata.navigation.heading[compass_magnetic]) ) {
      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,M,", 
		     session->gpsdata.navigation.heading[compass_magnetic]);
  } else {
      (void)strlcat(bufp, ",,", len);
  }

  if ( !isnan(session->gpsdata.navigation.speed_thru_water) ) {

      (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		     "%.2f,N,%.2f,K", 
		     session->gpsdata.navigation.speed_thru_water,
                     session->gpsdata.navigation.speed_thru_water * KNOTS_TO_KPH );

  } else {
      (void)strlcat(bufp, ",,,", len);
  }

  nmea_add_checksum(bufp);
}

static void gpsd_binary_dpt_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
  if (!isnan(session->gpsdata.navigation.depth_offset)) {
    if (!isnan(session->gpsdata.navigation.depth)) {
      (void)snprintf(bufp, len, "$GPDPT,%.2f,%.2f",
		     session->gpsdata.navigation.depth,
		     session->gpsdata.navigation.depth_offset);
      nmea_add_checksum(bufp);
    }
  } else if (!isnan(session->gpsdata.navigation.depth)) {
    (void)snprintf(bufp, len, "$GPDBT,%.2f,f,%.2f,M,,",
		 session->gpsdata.navigation.depth *  METERS_TO_FEET,
		 session->gpsdata.navigation.depth);
    nmea_add_checksum(bufp);
  }
}

static void gpsd_binary_hdg_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>

  (void)snprintf(bufp, len, "$GPHDG,");

  if (!isnan(session->gpsdata.navigation.heading[compass_magnetic])) {
    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		   "%.2f,",
		   session->gpsdata.navigation.heading[compass_magnetic]);
  } else {
    (void)strlcat(bufp, ",", len);
  }

  if (isnan(session->gpsdata.environment.deviation))
    (void)strlcat(bufp, ",,", len);
  else
    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		   "%.2f,%c,", 
		   fabs(session->gpsdata.environment.deviation),
		   session->gpsdata.environment.deviation > 0?'E':'W');

  if (isnan(session->gpsdata.environment.variation))
    (void)strlcat(bufp, ",,", len);
  else
    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp),
		   "%.2f,%c", 
		   fabs(session->gpsdata.environment.variation),
		   session->gpsdata.environment.variation > 0?'E':'W');

  nmea_add_checksum(bufp);
}


static void gpsd_binary_rsa_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--RSA,x.x,A,x.x,A

  if (!isnan(session->gpsdata.navigation.rudder_angle)) {
    (void)snprintf(bufp, len - strlen(bufp),
		   "$GPRSA,%.2f,A,,,", 
   		   session->gpsdata.navigation.rudder_angle);
    nmea_add_checksum(bufp);
  }

}

static void gpsd_binary_xte_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  /*
   === XTE - Cross-Track Error, Measured ===

   ------------------------------------------------------------------------------
        1 2 3   4 5 6   7
        | | |   | | |   |
    $--XTE,A,A,x.x,a,N,m,*hh<CR><LF>
   ------------------------------------------------------------------------------

   Field Number:

   1. Status
     - V = LORAN-C Blink or SNR warning
     - V = general warning flag or other navigation systems when a reliable
         fix is not available
   2. Status
     - V = Loran-C Cycle Lock warning flag
     - A = OK or not used
   3. Cross Track Error Magnitude
   4. Direction to steer, L or R
   5. Cross Track Units, N = Nautical Miles
   6. FAA mode indicator (NMEA 2.3 and later, optional)
   7. Checksum
  */

  if (!isnan(session->gpsdata.navigation.xte)) {
    (void)snprintf(bufp, len - strlen(bufp),
		   "$GPXTE,A,A,%.2f,%c,N,", 
   		   fabs(session->gpsdata.navigation.xte * METERS_TO_NM),
		   session->gpsdata.navigation.xte < 0?'R':'L');
    nmea_add_checksum(bufp);
  }

}

static void gpsd_binary_rot_dump(struct gps_device_t *session,
				     char bufp[], size_t len)
{ 
  // $--ROT,x.x,A

  if (!isnan(session->gpsdata.navigation.rate_of_turn)) {

    // from deg / sec to deg / min
    (void)snprintf(bufp, len - strlen(bufp),
		   "$GPROT,%.2f,A", 
   		   session->gpsdata.navigation.rate_of_turn * 60.0); 
    nmea_add_checksum(bufp);
  }

}

static void gpsd_binary_distance_traveled_dump(struct gps_device_t *session,
				     char bufp[], size_t len) {
  //  $--VLW,x.x,N,x.x,N*hh<CR><LF>xs
  (void)snprintf(bufp, len, "$GPVLW,");

  if (!isnan(session->gpsdata.navigation.distance_total)) {
    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp), 
		 "%.2f,N,",
		   session->gpsdata.navigation.distance_total);
  } else {
    (void)strlcat(bufp, ",,", len);
  }

  if (!isnan(session->gpsdata.navigation.distance_trip)) {
    (void)snprintf(bufp + strlen(bufp), len - strlen(bufp), 
		 "%.2f,N,",
		   session->gpsdata.navigation.distance_trip);
  } else {
    (void)strlcat(bufp, ",,", len);
  }

  nmea_add_checksum(bufp);
}

static void gpsd_binary_temp_water_dump(struct gps_device_t *session,
				     char bufp[], size_t len) {
  //  $--MTW,x.x,C*hh<CR><LF>
  if (!isnan(session->gpsdata.environment.temp[temp_water])) {
    (void)snprintf(bufp, len, "$GPMTW,%.2f,C",
		 session->gpsdata.environment.temp[temp_water]);
    nmea_add_checksum(bufp);
  }
}

/*@-compdef -mustdefine@*/
/* *INDENT-OFF* */
void nmea_tpv_dump(struct gps_device_t *session,
		   /*@out@*/ char bufp[], size_t len)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & TIME_SET) != 0)
	gpsd_binary_time_dump(session, bufp + strlen(bufp),
			      len - strlen(bufp));
    if ((session->gpsdata.set & LATLON_SET) != 0) {
	gpsd_position_fix_dump(session, bufp + strlen(bufp),
			       len - strlen(bufp));
	gpsd_transit_fix_dump(session, bufp + strlen(bufp),
			      len - strlen(bufp));
    }
    if ((session->gpsdata.set
	 & (MODE_SET | DOP_SET | USED_IS | HERR_SET | VERR_SET)) != 0)
	gpsd_binary_quality_dump(session, bufp + strlen(bufp),
				 len - strlen(bufp));
}
/* *INDENT-ON* */

void nmea_sky_dump(struct gps_device_t *session,
		   /*@out@*/ char bufp[], size_t len)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & SATELLITE_SET) != 0)
	gpsd_binary_satellite_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
}

void nmea_subframe_dump(struct gps_device_t *session,
		   /*@out@*/ char bufp[], size_t len)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & SUBFRAME_SET) != 0)
	gpsd_binary_almanac_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
}

#ifdef AIVDM_ENABLE
void nmea_ais_dump(struct gps_device_t *session,
		   /*@out@*/ char bufp[], size_t len)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & AIS_SET) != 0)
	gpsd_binary_ais_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
}
#endif /* AIVDM_ENABLE */

void nmea_navigation_dump(struct gps_device_t *session,
		   /*@out@*/ char bufp[], size_t len)
{
    bufp[0] = '\0';
    if ((session->gpsdata.set & NAVIGATION_SET) != 0) {

      if((session->gpsdata.navigation.set & NAV_STW_PSET) != 0) 
	gpsd_binary_vhw_dump(session, bufp + strlen(bufp),
					  len - strlen(bufp));

      if(((session->gpsdata.navigation.set & NAV_SOG_PSET) != 0) 
	 || ((session->gpsdata.navigation.set & NAV_COG_PSET) != 0))
	gpsd_binary_vtg_dump(session, bufp + strlen(bufp),
			       len - strlen(bufp));

      if(((session->gpsdata.navigation.set & NAV_DIST_TOT_PSET) != 0) 
	 || ((session->gpsdata.navigation.set & NAV_DIST_TRIP_PSET) != 0))
	gpsd_binary_distance_traveled_dump(session, bufp + strlen(bufp),
					   len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_DPT_PSET) != 0) 
	gpsd_binary_dpt_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_HDG_MAGN_PSET) != 0) 
	gpsd_binary_hdg_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_HDG_TRUE_PSET) != 0) 
	gpsd_binary_vhw_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_ROT_PSET) != 0) 
	gpsd_binary_rot_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_XTE_PSET) != 0) 
	gpsd_binary_xte_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));

      if((session->gpsdata.navigation.set & NAV_RUDDER_ANGLE_PSET) != 0) 
	gpsd_binary_rsa_dump(session, bufp + strlen(bufp),
			     len - strlen(bufp));
    }
}

/* returns the number of potential sentences for the data changed
*/
int nmea_environment_dump(struct gps_device_t *session, int num,
		   /*@out@*/ char bufp[], size_t len)
{
    int ret = 1;

    bufp[0] = '\0';
    if ((session->gpsdata.set & ENVIRONMENT_SET) != 0) {
      switch(session->gpsdata.environment.set) {

      case ENV_WIND_APPARENT_SPEED_PSET:
      case ENV_WIND_APPARENT_ANGLE_PSET:
        ret = 2;

        if(num == 0)
          gpsd_binary_vwr_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
        else
	  gpsd_binary_mwv_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
	break;

      case ENV_WIND_TRUE_GROUND_SPEED_PSET:
      case ENV_WIND_TRUE_GROUND_ANGLE_PSET:
      case ENV_WIND_TRUE_WATER_SPEED_PSET:
      case ENV_WIND_TRUE_WATER_ANGLE_PSET:
	gpsd_binary_mwv_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
	break;

      case ENV_TEMP_WATER_PSET:
	gpsd_binary_temp_water_dump(session, bufp + strlen(bufp),
				   len - strlen(bufp));
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

/*@+compdef +mustdefine@*/

/* pseudonmea.c ends here */
