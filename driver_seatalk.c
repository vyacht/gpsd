/*
 * Seatalk driver
 *
 * This file is Copyright (c) 2012 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#ifndef S_SPLINT_S
#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#endif /* S_SPLINT_S */

#include "gpsd.h"

#if defined(SEATALK_ENABLE)
#include "driver_seatalk.h"
#include "bits.h"

typedef gps_mask_t(* st_decoder)(uint8_t * cmdBuffer, uint8_t size, struct gps_device_t *session);

typedef struct st_phrase {
  uint8_t cmdId;
  uint8_t fixed_length;  /* if we know the length for sure 
			    we are having it here for 
			    better error handling in parse phase, 0x80 is unknown */
  st_decoder decoder;
  const char * name;
} st_phrase;

/* fixed length array - position is number of seatalk sentence */
uint8_t st_fixed_lengths[255];

gps_mask_t seatalk_parse_input(struct gps_device_t *session);

void character_skip(struct gps_packet_t *lexer);

void character_skip(struct gps_packet_t *lexer)
/* shift the input buffer from inbufptr to skip a single character */
{

    if( lexer->inbufptr - lexer->inbuffer == 0 ) {
        character_discard(lexer);
        return;
    }

    // pointer is still on next char, thus remaining is correctly set without -1!
    size_t remaining = packet_buffered_input(lexer);

    lexer->inbufptr--;
    gpsd_report(lexer->debug, LOG_RAW + 3,
		"pre inbuflen = %lu with %lu remaining, %p, %p\n",
		lexer->inbuflen, remaining, lexer->inbuffer, lexer->inbufptr);


    uint8_t c = *lexer->inbufptr;
    memmove(lexer->inbufptr, lexer->inbufptr+1, remaining);
    lexer->inbuflen--;

    gpsd_report(lexer->debug, LOG_RAW + 3,
		"post inbuflen = %lu, %p, %p\n",
		lexer->inbuflen, lexer->inbuffer, lexer->inbufptr);

    if (lexer->debug >= LOG_RAW+1) {
	char scratchbuf[MAX_PACKET_LENGTH*2+1];
	gpsd_report(lexer->debug, LOG_RAW + 2,
		    "Character %c [%02X] @ %p skipped, buffer %zu, %p = %s\n",
		    (isprint(c) ? c : '.'), c,
		    lexer->inbufptr, lexer->inbuflen, lexer->inbuffer,  
		    gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
				    (char *)lexer->inbuffer, lexer->inbuflen));
    }
}


/* Function to get parity of number n. It returns 1
   if n has odd parity, and returns 0 if n has even
   parity */
static int getParity(unsigned int n) {

    int parity = 0;
    while (n)
    {
        parity = !parity;
        n      = n & (n - 1);
    }
    return parity;
}

static void seatalk_merge_yymmdd(uint8_t yy, uint8_t mon, uint8_t mday, struct gps_device_t *session)
/* sentence supplied ddmmyy, but no century part */
{
    int year;

    /* check for century wrap */
    if (session->driver.seatalk.date.tm_year % 100 == 99 && yy == 0)
	gpsd_century_update(session, session->context->century + 100);
    year = (session->context->century + yy);

    if ( (1 > mon ) || (12 < mon ) ) {
	gpsd_report(session->context->debug, LOG_WARN,
		    "seatalk_merge_yymmdd(%d, %d, %d), malformed month\n",  
		    yy, mon, mday);
    } else if ( (1 > mday ) || (31 < mday ) ) {
	gpsd_report(session->context->debug, LOG_WARN,
		    "seatalk_merge_yymmdd(%d, %d, %d), malformed day\n", 
		    yy, mon, mday);
    } else {
	gpsd_report(session->context->debug, LOG_DATA,
		    "seatalk_merge_yymmdd(%d, %d, %d) sets year %d\n", 
		    yy, mon, mday, year);
	session->driver.seatalk.date.tm_year = year - 1900;
	session->driver.seatalk.date.tm_mon = mon - 1;
	session->driver.seatalk.date.tm_mday = mday;
    }
}

static void seatalk_merge_hhmmss(uint8_t hh, uint8_t mm, uint8_t ss, struct gps_device_t *session)
/* update from a UTC time */
{
    int old_hour = session->driver.seatalk.date.tm_hour;

    session->driver.seatalk.date.tm_hour = hh;
    if (session->driver.seatalk.date.tm_hour < old_hour)	/* midnight wrap */
	session->driver.seatalk.date.tm_mday++;
    session->driver.seatalk.date.tm_min = mm;
    session->driver.seatalk.date.tm_sec = ss;
    /*
    session->driver.seatalk.subseconds =
	safe_atof(hhmmss + 4) - session->driver.nmea.date.tm_sec;
    */
}

static gps_mask_t seatalk_print_command(uint8_t * cmdBuffer, uint8_t size,
					struct gps_device_t *session) {
  int   l1, l2, ptr;
  char  bu[128];

  ptr = 0;
  for (l1=1;l1<size;l1++) {
    if (l1 == 1) {
      ptr = 0;
      l2 = sprintf(&bu[ptr], "                   : ");
      ptr += l2;
    }
    l2 = sprintf(&bu[ptr], "0x%02x ", (unsigned int)cmdBuffer[l1]);
    ptr += l2;
  }
  gpsd_report(session->context->debug, LOG_IO,"0x%02X: %s\n", cmdBuffer[0], bu);

  return 0;
}

static gps_mask_t seatalk_update_time(struct gps_device_t *session) {

  gps_mask_t mask = 0;

  /* some ugly time stunts to cope with GPS 112/120/125 only reporting time every 10secs */
  if (session->driver.seatalk.lastts > 0) {
    session->driver.seatalk.offset = 
      timestamp() - session->driver.seatalk.lastts;

    gpsd_external_report(session->context->debug, LOG_DATA,
		"seatalk update time with offset= %2f.\n",
		session->driver.seatalk.offset);
    mask |= TIME_SET;
  }

  return mask;
}


static gps_mask_t seatalk_process_depth(uint8_t * cmdBuffer, uint8_t size, 
					struct gps_device_t *session) {
  /*
    00  02  YZ  XX XX  Depth below transducer: XXXX/10 feet
                       Flags in Y: Y&8 = 8: Anchor Alarm is active
                                   Y&4 = 4: Metric display units or
                                           Fathom display units if followed by command 65
                                   Y&2 = 2: Used, unknown meaning
                       Flags in Z: Z&4 = 4: Transducer defective
                                   Z&2 = 2: Deep Alarm is active
                                   Z&1 = 1: Shallow Depth Alarm is active
                       Corresponding NMEA sentences: DPT, DBT
  */

  if(size == 5) {

    if((cmdBuffer[1] & 0x0f) != 2) {
      printf("process error - wrong encoded data length\n");
    }

    session->gpsdata.navigation.depth = 
      getleu16(cmdBuffer, 3) / 10.0 / METERS_TO_FEET;

    session->gpsdata.navigation.set = NAV_DPT_PSET;  

    gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Depth = %f m\n", cmdBuffer[0], 
	      session->gpsdata.navigation.depth);

    if(cmdBuffer[2] & 0x80) {
      gpsd_report(session->context->debug, LOG_DATA,
		  "            Anchor alarm active\n");
    }

    if(cmdBuffer[2] & 0x40) {
      gpsd_report(session->context->debug, LOG_DATA,
		  "            Metric units for depth\n");
    }

  } else {
    printf("process error - wrong datagram length %u\n", size);
  }

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_wind_angle(uint8_t * cmdBuffer, uint8_t size UNUSED,
					struct gps_device_t *session) {
  /*
    10  01  XX  YY  Apparent Wind Angle: XXYY/2 degrees right of bow
                    Used for autopilots Vane Mode (WindTrim)
                    Corresponding NMEA sentence: MWV
  */
  session->gpsdata.environment.wind.apparent.angle = 
    getbes16(cmdBuffer, 2) / 2.0;

  gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Apparent Wind Angle = %f deg right of bow\n", cmdBuffer[0], 
	      session->gpsdata.environment.wind.apparent.angle);

  session->gpsdata.environment.set = ENV_WIND_APPARENT_ANGLE_PSET;

  return ENVIRONMENT_SET;
}

static gps_mask_t seatalk_process_wind_speed(uint8_t * cmdBuffer, uint8_t size UNUSED,
					struct gps_device_t *session) {
  /*
    11  01  XX  0Y  Apparent Wind Speed: (XX & 0x7F) + Y/10 Knots
                    Units flag: XX&0x80=0    => Display value in Knots
                                XX&0x80=0x80 => Display value in Meter/Second
                    Corresponding NMEA sentence: MWV
  */
  session->gpsdata.environment.wind.apparent.speed = 
    (double)(cmdBuffer[2] & 0x7F) + (double)(cmdBuffer[3] & 0x0F) / 10.0;

  gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Apparent Wind Speed = %d + %d/10 knts = %f knots\n", cmdBuffer[0], 
	      cmdBuffer[2] & 0x7F, cmdBuffer[3] & 0x0F,
	      session->gpsdata.environment.wind.apparent.speed);

  session->gpsdata.environment.set = ENV_WIND_APPARENT_SPEED_PSET;

  return ENVIRONMENT_SET;
}

static gps_mask_t seatalk_process_speed(uint8_t * cmdBuffer, uint8_t size,
					struct gps_device_t *session) {
  /*
    20  01  XX  XX  Speed through water: XXXX/10 Knots
                    Corresponding NMEA sentence: VHW

    26  04  XX  XX  YY  YY DE  Speed through water:
                    XXXX/100 Knots, sensor 1, current speed, valid if D&4=4
                    YYYY/100 Knots, average speed (trip/time) if D&8=0
                             or data from sensor 2 if D&8=8
                    E&1=1: Average speed calulation stopped
                    E&2=2: Display value in MPH
                    Corresponding NMEA sentence: VHW
  */

  if(cmdBuffer[0] == 0x20) {

    session->gpsdata.navigation.speed_thru_water = 
      getleu16(cmdBuffer, 2) / 10.0;

    gpsd_report(session->context->debug, LOG_DATA,
		"command %02X: Speed = %f knots\n", cmdBuffer[0], 
		session->gpsdata.navigation.speed_thru_water);

  } else if(cmdBuffer[0] == 0x26) {

    uint16_t s1 = getleu16(cmdBuffer, 2);
    uint16_t s2 = getleu16(cmdBuffer, 4);

    if(cmdBuffer[6] & 0x40) {
      // sensor 1 data is valid
      // means sensor 2 delivers speed as well, we ignore it as we don't know if its really valid
      session->gpsdata.navigation.speed_thru_water = s1 / 100.0;
      if((cmdBuffer[6] & 0x80) == 0x80) {
	gpsd_report(session->context->debug, LOG_DATA,
		    "command %02X: Speed = %f knots from sensor 1 (sensor 2 ignored)\n", cmdBuffer[0], 
		    session->gpsdata.navigation.speed_thru_water);
      } else {
        // no speed from sensor 2, maybe average, skip
	gpsd_report(session->context->debug, LOG_DATA,
		    "command %02X: Speed = %f knots from sensor 1\n", cmdBuffer[0], 
		    session->gpsdata.navigation.speed_thru_water);
      }
    } else {
      if((cmdBuffer[6] & 0x80) == 0x80) {
	// means sensor 2 delivers speed but not s1
        session->gpsdata.navigation.speed_thru_water = s2 / 100.0;
	gpsd_report(session->context->debug, LOG_DATA,
		    "command %02X: Speed = %f knots from sensor 2\n", cmdBuffer[0], 
		    session->gpsdata.navigation.speed_thru_water);
      }
    }
  }
  seatalk_print_command(cmdBuffer, size, session);
  session->gpsdata.navigation.set = NAV_STW_PSET;  

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_milage(uint8_t * cmdBuffer, uint8_t size UNUSED,
					struct gps_device_t *session) {
  /*
    21  02  XX  XX  0X  Trip Mileage: XXXXX/100 nautical miles
    22  02  XX  XX  00  Total Mileage: XXXX/10 nautical miles 
  */
  
  if(cmdBuffer[0] == 0x21) {
    session->gpsdata.navigation.distance_trip = 
      (((uint32_t)getleu16(cmdBuffer, 2) << 4) | (cmdBuffer[4] & 0x0F))/ 100.0;
    session->gpsdata.navigation.set = NAV_DIST_TRIP_PSET;
  } else {
    session->gpsdata.navigation.distance_total = getleu16(cmdBuffer, 2) / 10.0;
    session->gpsdata.navigation.set = NAV_DIST_TOT_PSET;
  }

  gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Trip/Total log = %.2fnm / %.2fnm\n", cmdBuffer[0], 
	      session->gpsdata.navigation.distance_trip,
              session->gpsdata.navigation.distance_total);

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_distlog(uint8_t * cmdBuffer, uint8_t size UNUSED,
					struct gps_device_t *session) {
  /*
    25  Z4  XX  YY  UU  VV AW  Total & Trip Log
                     total= (XX+YY*256+Z* 4096)/ 10 [max=104857.5] nautical miles
                     trip = (UU+VV*256+W*65536)/100 [max=10485.75] nautical miles
  */
  session->gpsdata.navigation.distance_trip = 
    (cmdBuffer[4] + cmdBuffer[5]*256 + (cmdBuffer[6] & 0x0F)*65536)/100.0;
  session->gpsdata.navigation.set = NAV_DIST_TRIP_PSET;

  session->gpsdata.navigation.distance_total = 
    (cmdBuffer[2] + cmdBuffer[3]*256 + ((cmdBuffer[2] & 0xF0) >> 4) * 4096)/10.0;
  session->gpsdata.navigation.set = NAV_DIST_TOT_PSET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Trip/Total log = %.2fnm / %.2fnm\n", cmdBuffer[0], 
	      session->gpsdata.navigation.distance_trip,
              session->gpsdata.navigation.distance_total);

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_watertemp(uint8_t * cmdBuffer, uint8_t size UNUSED,
					struct gps_device_t *session) {
  /*
    23  Z1  XX  YY  Water temperature (ST50): XX deg Celsius, YY deg Fahrenheit
                    Flag Z&4: Sensor defective or not connected (Z=4)
                    Corresponding NMEA sentence: MTW

    27  01  XX  XX  Water temperature: (XXXX-100)/10 deg Celsius
                    Corresponding NMEA sentence: MTW
  */
  if(cmdBuffer[0] == 0x23) {
    if((cmdBuffer[1] & 0xF0) != 0x40) { 
      session->gpsdata.environment.temp[temp_water] = cmdBuffer[2];
      session->gpsdata.environment.set = ENV_TEMP_WATER_PSET;
    }
  } else {
    session->gpsdata.environment.temp[temp_water] = (getleu16(cmdBuffer, 2) - 100)/10.0;
    session->gpsdata.environment.set = ENV_TEMP_WATER_PSET;
  }
    
  gpsd_report(session->context->debug, LOG_DATA,
	      "command %02X: Water temperature %.2f\n", cmdBuffer[0], 
              session->gpsdata.environment.temp[temp_water]);

  return ENVIRONMENT_SET;
}

static gps_mask_t seatalk_process_MOB_cancel(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 36  00  01      Cancel MOB (Man Over Board) condition

*/
  gps_mask_t mask = 0;
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk MOD cancel: \n");
  seatalk_print_command(bu, size, session);
  return mask;
}

static gps_mask_t seatalk_process_lat(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 50  Z2  XX  YY  YY  LAT position: XX degrees, (YYYY & 0x7FFF)/100 minutes
                     MSB of Y = YYYY & 0x8000 = South if set, North if cleared
                     Z= 0xA or 0x0 (reported for Raystar 120 GPS), meaning unknown
                     Stable filtered position, for raw data use command 58
                     Corresponding NMEA sentences: RMC, GAA, GLL

*/
  gps_mask_t mask = 0;

  uint16_t mm = getleu16(bu, 3);

  /*@-type@*//* splint has a bug here */
  session->driver.seatalk.lat        = bu[2] + (mm & 0x7fFF)/10000.0/0.6;
  /*@+type@*/
  if(mm & 0x8000) {
      session->driver.seatalk.lat *= -1.0;
  }
  session->driver.seatalk.lat_set = 1;

  // this will be rather reset when lat not available
  mask |= LATLON_SET;

  // my understanding of this sentence is that this is fix data
  session->gpsdata.status = STATUS_FIX;	
  mask |= STATUS_SET;

  mask |= seatalk_update_time(session);

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk latitude: %0.4f (%c)\n", 
	      session->driver.seatalk.lat, (mm & 0x8000)?'S':'N');
  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_lon(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 51  Z2  XX  YY  YY  LON position: XX degrees, (YYYY & 0x7FFF)/100 minutes
                                           MSB of Y = YYYY & 0x8000 = East if set, West if cleared
                                           Z= 0xA or 0x0 (reported for Raystar 120 GPS), meaning unknown
                     Stable filtered position, for raw data use command 58
                     Corresponding NMEA sentences: RMC, GAA, GLL

*/
  gps_mask_t mask = 0;

  uint16_t mm = getleu16(bu, 3);

  /*@-type@*//* splint has a bug here */
  session->driver.seatalk.lon       = bu[2] + (mm & 0x7fFF)/10000.0/0.6;
  /*@+type@*/
  if( (mm & 0x8000) == 0 ) {
      session->driver.seatalk.lon *= -1.0;
  }
  session->driver.seatalk.lon_set = 1;

  // this will be rather reset when lat not available
  mask |= LATLON_SET;

  // my understanding of this sentence is that this is fix data
  session->gpsdata.status = STATUS_FIX;	
  mask |= STATUS_SET;

  mask |= seatalk_update_time(session);

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk longitude: %0.4f (%c)\n", 
	      session->driver.seatalk.lon, (mm & 0x8000)?'E':'W');
  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_sog(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 52  01  XX  XX  Speed over Ground: XXXX/10 Knots
                 Corresponding NMEA sentences: RMC, VTG

*/
  session->gpsdata.navigation.speed_over_ground = getleu16(bu, 2) / 10.0;
  session->gpsdata.navigation.set = NAV_SOG_PSET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk SOG: %0.2fknots\n", session->gpsdata.navigation.speed_over_ground);
  seatalk_print_command(bu, size, session);

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_cog(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 53  U0  VW      Course over Ground (COG) in degrees:
                 The two lower  bits of  U * 90 +
                    the six lower  bits of VW *  2 +
                    the two higher bits of  U /  2 =
                    (U & 0x3) * 90 + (VW & 0x3F) * 2 + (U & 0xC) / 8
                 Corresponding NMEA sentences: RMC, VTG

*/
  session->gpsdata.navigation.course_over_ground = 
    ((bu[1] & 0x30) >> 4) * 90.0 + (bu[2] & 0x3F) * 2.0 + ((bu[1] & 0xC0) >> 6) / 2.0;
  session->gpsdata.navigation.set = NAV_COG_PSET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk cog: %0.2f\n", session->gpsdata.navigation.course_over_ground);
  seatalk_print_command(bu, size, session);

  return NAVIGATION_SET;
}

static gps_mask_t seatalk_process_time(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 54  T1  RS  HH  GMT-time: HH hours,
                           6 MSBits of RST = minutes = (RS & 0xFC) / 4
                           6 LSBits of RST = seconds =  ST & 0x3F
                 Corresponding NMEA sentences: RMC, GAA, BWR, BWC

*/
  gps_mask_t mask = 0;

  uint8_t mm = bu[2] >> 2;
  uint8_t ss = ((bu[2] & 0x03) << 4) | (bu[1] >> 4);

  seatalk_merge_hhmmss(bu[3], mm, ss, session);
  if (session->driver.seatalk.date.tm_year == 0)
    gpsd_external_report(session->context->debug, LOG_WARN,
		"can't use time until after a year was supplied.\n");
  else {

    // update when time report was last seen
    session->driver.seatalk.lastts = timestamp();
    session->driver.seatalk.offset = 0;
    gpsd_external_report(session->context->debug, LOG_DATA,
		"seatalk setting new last seen report time= %2f.\n",
		session->driver.seatalk.lastts);

    mask = TIME_SET;
  }

  gpsd_external_report(session->context->debug, LOG_DATA,
	      "seatalk time: %02d:%02d:%02d\n", bu[3], mm, ss);
  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_date(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 56  M1  DD  YY  Date: YY year, M month, DD day in month
                 Corresponding NMEA sentence: RMC

*/
  gps_mask_t mask = 0;

  seatalk_merge_yymmdd(bu[3], bu[1] >> 4, bu[2], session);

  // makes no sense to set time / date without timestamp
  // mask = TIME_SET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk date: %02d-%02d-%02d\n", bu[3], bu[1] >> 4, bu[2]);
  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_noofsats(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 57  S0  DD      Sat Info: S number of sats, DD horiz. dillution of position, if S=1 -> DD=0x94
                 Corresponding NMEA sentences: GGA, GSA

*/
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk number of sats: %d, dill = %d\n", bu[1] >> 4, bu[2]);
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_lat_lon_raw(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 58  Z5  LA XX YY LO QQ RR   LAT/LON
                 LA Degrees LAT, LO Degrees LON
                 minutes LAT = (XX*256+YY) / 1000
                 minutes LON = (QQ*256+RR) / 1000
                 Z&1: South (Z&1 = 0: North)
                 Z&2: East  (Z&2 = 0: West)
                 Raw unfiltered position, for filtered data use commands 50&51
                 Corresponding NMEA sentences: RMC, GAA, GLL

*/
  gps_mask_t mask = 0;

  /*@-type@*//* splint has a bug here */
  session->driver.seatalk.lat        = bu[2] + (bu[3] * 256 + bu[4])/100000.0/0.6;
  session->driver.seatalk.lon       = bu[5] + (bu[6] * 256 + bu[7])/100000.0/0.6;
  /*@+type@*/
  if( (bu[1] >> 4) & 0x01 ) {
      session->driver.seatalk.lat *= -1.0; // south
  }
  if( ((bu[1] >> 4) & 0x02) == 0 ) {
      session->driver.seatalk.lon *= -1.0; // west
  }

  session->driver.seatalk.lat_set = 1;
  session->driver.seatalk.lon_set = 1;
  mask |= LATLON_SET;

  /* we do not believe these raw reports are 
     tied to a GPS fix - thus we do not set the mode here */

  mask |= seatalk_update_time(session);

  gpsd_report(session->context->debug, LOG_DATA,
              "seatalk raw lat/lon = %0.2f (%c) / %0.2f (%c) (%s)\n",
              session->driver.seatalk.lat , (session->driver.seatalk.lat > 0) ? 'N' : 'S', 
              session->driver.seatalk.lon, (session->driver.seatalk.lon > 0) ? 'E' : 'W',
              gps_maskdump(mask));

  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_count_down(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 59  22  SS MM XH  Set Count Down Timer
                   MM=Minutes ( 00..3B ) ( 00 .. 63 Min ), MSB:0 Count up start flag
                   SS=Seconds ( 00..3B ) ( 00 .. 59 Sec )
                   H=Houres  ( 0..9 )   ( 00 .. 09 Houres )
                   X= Counter Mode: 0 Count up and start if MSB of MM set
                                    4 Count down
                                    8 Count down and start
                   ( Example 59 22 3B 3B 49 -> Set Countdown Timer to 9.59:59 )
 59  22  0A 00 80  Sent by ST60 in countdown mode when counted down to 10 Seconds.
*/
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk count_down: \n");
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_MOB(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 6E  07  00  00 00 00 00 00 00 00 MOB (Man Over Board), (ST80), preceded
                 by a Waypoint 999 command: 82 A5 40 BF 92 6D 24 DB
*/
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk MOB: \n");
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_target_wp(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 82  05  XX  xx YY yy ZZ zz   Target waypoint name
                 XX+xx = YY+yy = ZZ+zz = FF (allows error detection)
                 Takes the last 4 chars of name, assumes upper case only
                 Char= ASCII-Char - 0x30
                 XX&0x3F: char1
                 (YY&0xF)*4+(XX&0xC0)/64: char2
                 (ZZ&0x3)*16+(YY&0xF0)/16: char3
                 (ZZ&0xFC)/4: char4
                 Corresponding NMEA sentences: RMB, APB, BWR, BWC
*/
  char c[5];
  memset(c, 0, 5);

  c[0] = 0x30 + (bu[2] & 0x3F);
  c[1] = 0x30 + ( ((bu[4] & 0x0F) << 2) | ((bu[2] & 0xC0) >> 6) );
  c[2] = 0x30 + ( ((bu[6] & 0x03) << 4) | ((bu[4] & 0xF0) >> 4) );
  c[3] = 0x30 +   ((bu[6] & 0xFC) >> 2);

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk target waypoint name: %s\n", c);
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_compass_hdg(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 95  U6  VW  XY 0Z 00 RR 00 0T  Replaces command 84 while autopilot is in value setting mode
                   e.g. lamp intensity or response level

 84  U6  VW  XY 0Z 0M RR SS TT  Compass heading Autopilot course and
                  Rudder position (see also command 9C)
                  Compass heading in degrees:
                    The two lower  bits of  U * 90 +
                    the six lower  bits of VW *  2 +
                    number of bits set in the two higher bits of U =
                    (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
                  Turning direction:
                    Most significant bit of U = 1: Increasing heading, Ship turns right
                    Most significant bit of U = 0: Decreasing heading, Ship turns left
                  Autopilot course in degrees:
                    The two higher bits of  V * 90 + XY / 2
                  Z & 0x2 = 0 : Autopilot in Standby-Mode
                  Z & 0x2 = 2 : Autopilot in Auto-Mode
                  Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular "10" datagrams
                  Z & 0x8 = 8 : Autopilot in Track Mode
                  M: Alarms + audible beeps
                    M & 0x04 = 4 : Off course
                    M & 0x08 = 8 : Wind Shift
                  Rudder position: RR degrees (positive values steer right,
                    negative values steer left. Example: 0xFE = 2° left)
                  SS & 0x01 : when set, turns off heading display on 600R control.
                  SS & 0x02 : always on with 400G
                  SS & 0x08 : displays “NO DATA” on 600R
                  SS & 0x10 : displays “LARGE XTE” on 600R
                  SS & 0x80 : Displays “Auto Rel” on 600R
                  TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer 
*/
  gps_mask_t mask = 0;

  session->gpsdata.navigation.heading[compass_magnetic] = 
    ((bu[1] & 0x30) >> 4) * 90.0 + (bu[2] & 0x3F) * 2.0 + ((bu[1] & 0xC0) >> 6) / 2.0;
  session->gpsdata.navigation.set = NAV_HDG_MAGN_PSET;

  mask = NAVIGATION_SET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk compass heading: hdg = %0.2f\n", 
	      session->gpsdata.navigation.heading[compass_magnetic]);

  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_nav_to_wp(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 85  X6  XX  VU ZW ZZ YF 00 yf   Navigation to waypoint information
                  Cross Track Error: XXX/100 nautical miles
                   Example: X-track error 2.61nm => 261 dec => 0x105 => X6XX=5_10
                  Bearing to destination: (U & 0x3) * 90° + WV / 2°
                   Example: GPS course 230°=180+50=2*90 + 0x64/2 => VUZW=42_6
                   U&8: U&8 = 8 -> Bearing is true, U&8 = 0 -> Bearing is magnetic
                  Distance to destination: Distance 0-9.99nm: ZZZ/100nm, Y & 1 = 1
                                           Distance >=10.0nm: ZZZ/10 nm, Y & 1 = 0
                  Direction to steer: if Y & 4 = 4 Steer right to correct error
                                      if Y & 4 = 0 Steer left  to correct error
                  Example: Distance = 5.13nm, steer left: 5.13*100 = 513 = 0x201 => ZW ZZ YF=1_ 20 1_
                           Distance = 51.3nm, steer left: 51.3*10  = 513 = 0x201 => ZW ZZ YF=1_ 20 0_
                  Track control mode:
                     F= 0x1: Display x-track error and Autopilot course
                     F= 0x3: Enter Track Control Mode, i.e. lock on to GPS.
                             Display x-track error, autopilot course and bearing
                             to destination
                     F= 0x5: Display x-track error, distance to waypoint,
                             autopilot course and bearing to destination
           normal--> F= 0x7: Enter Track Control Mode, i.e. lock on to GPS.
                             Display x-track error, distance to waypoint,
                             autopilot course and bearing to destination
                     F= 0xF: As 0x7 but with x-track error alarm
                     F= 2, 4, 6, 8 ... causes data errors
                   In case of a waypoint change, sentence 85, indicating the new bearing and distance,
                   should be transmitted prior to sentence 82 (which indicates the waypoint change).
                   Corresponding NMEA sentences: RMB, APB, BWR, BWC, XTE
*/
  float xte =  ((bu[1] >> 4) | (bu[2] << 4)) / 100.0;
  float bearing = (bu[3] & 0x03) * 90.0 + ((bu[3] >> 4) | ((bu[4] &0x0F) << 4)) / 2.0;  

  float f1 = (bu[6] & 0xF0) ? 100.0 : 10.0;
  float distance = ( (bu[4] >> 4) | (bu[5] << 4) ) / f1;

  int mode = bu[6] & 0x0F;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk navigation to waypoint: xte = %0.2fnm, bearing = %0.2f, distance = %0.2f, steer = %c, mode = %d\n", 
	      xte, bearing, distance, (bu[6] & 0x40) ? 'R' : 'L', mode);
  seatalk_print_command(bu, size, session);
  return 0;
}


static gps_mask_t seatalk_process_compass_hdg_st40(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 89  U2  VW  XY  2Z  Compass heading sent by ST40 compass instrument
                     (it is read as a compass heading by the ST1000(+) or ST2000(+) autopilot)
                       Compass heading in degrees:
                         The two lower bits of  U  *  90   +
                         the six lower bits of VW  *   2   +
                         the two higher bits of U  /   2   =
                         (U & 0x3) * 90 + (VW & 0x3F) * 2 + (U & 0xC) / 2
                      Locked stear reference (only send by the ST40 compass):
                         The two higher bits of V * 90 + XY / 2
                       Z & 0x2 = 0 : St40 in Standby mode
                       Z & 0x2 = 2 : St40 in Locked stear mode
                     Corresponding NMEA sentences: HDM, HDG, HDT, VHW
*/
  gps_mask_t mask = 0;

  session->gpsdata.navigation.heading[compass_magnetic] = 
    ((bu[1] & 0x30) >> 4) * 90.0 + (bu[2] & 0x3F) * 2.0 + ((bu[1] & 0xC0) >> 6) / 2.0;
  session->gpsdata.navigation.set = NAV_HDG_MAGN_PSET;
 
  mask = NAVIGATION_SET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk compass heading ST40: hdg = %0.2f\n", 
	      session->gpsdata.navigation.heading[compass_magnetic]);

  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_compass_variation_st40(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 99  00  XX        Compass variation sent by ST40 compass instrument
                     or ST1000, ST2000, ST4000+, E-80 every 10 seconds
                     but only if the variation is set on the instrument
                     Positive XX values: Variation West, Negative XX values: Variation East
                     Examples (XX => variation): 00 => 0, 01 => -1 west, 02 => -2 west ...
                                                 FF => +1 east, FE => +2 east ...
                   Corresponding NMEA sentences: RMC, HDG
*/
  session->gpsdata.environment.variation = -1.0*(int8_t)bu[2];
  session->gpsdata.environment.set = ENV_VARIATION_PSET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk compass variation ST40: %f (%c)\n", 
	      session->gpsdata.environment.variation,
	      bu[2] <= 0 ? 'E' : 'W');

  seatalk_print_command(bu, size, session);

  return ENVIRONMENT_SET;
}

static gps_mask_t seatalk_process_hdg_rudder_pos(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 9C  U1  VW  RR    Compass heading and Rudder position (see also command 84)
                     Compass heading in degrees:
                       The two lower  bits of  U * 90 +
                       the six lower  bits of VW *  2 +
                       number of bits set in the two higher bits of U =
                       (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
                     Turning direction:
                       Most significant bit of U = 1: Increasing heading, Ship turns right
                       Most significant bit of U = 0: Decreasing heading, Ship turns left
                     Rudder position: RR degrees (positive values steer right,
                       negative values steer left. Example: 0xFE = 2° left)
                     The rudder angle bar on the ST600R uses this record
*/
  gps_mask_t mask = 0;

  session->gpsdata.navigation.heading[compass_magnetic] = 
    ((bu[1] & 0x30) >> 4) * 90.0 + (bu[2] & 0x3F) * 2.0 + ((bu[1] & 0xC0) >> 6) / 2.0;
  session->gpsdata.navigation.set          = NAV_HDG_MAGN_PSET;

  session->gpsdata.navigation.rudder_angle = bu[3];
  session->gpsdata.navigation.set          = NAV_RUDDER_ANGLE_PSET;

  mask = NAVIGATION_SET;

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk compass heading and rudder pos: hdg = %0.2f, rsa= %0.2f\n",
	      session->gpsdata.navigation.heading[compass_magnetic],
	      session->gpsdata.navigation.rudder_angle);
  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_wp_definition(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 9E  FC  49  49 03 XX AA BB YY OO PP GG HH II JJ   Waypoint definition
                   XX: Degrees LAT, YY: Degrees LON
                   min&sec LAT= AA+(BB&0x1F)*256, BB&0x80 = 0: North, BB&0x80 = 0x80: South
                   min&sec LON= OO+(PP&ßx1F)*256, PP&0x80 = 0: West,  PP&0x80 = 0x80: East
                   GG HH II JJ: Last four characters of waypoint name
*/
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk waypoint definition: \n");
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_dest_wp(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 A1  XD  49  49 GG HH II JJ C1 C2 C3 C4 C5 C6 C7 C8  Destination Waypoint Info
                   GG HH II JJ: Last four characters of waypoint name
                   C1...C8: Up to 8 characters of WP name, unused are 0
                   Longer names (> 8 chars) create an additional record:
                   X=0: single record (short name)
                   X=1: 1st record, more follows
                   X=3: last record
                   Corresponding NMEA sentences: RMB, APB, BWR, BWC
*/
  char wp[13]; memset(wp, 0, 13);
  memcpy(wp, &bu[4], 12);
  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk destination waypoint info: %s\n", wp);
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_arrival(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 A2  X4  00  WW XX YY ZZ Arrival Info
                   X&0x2=Arrival perpendicular passed, X&0x4=Arrival circle entered
                   WW,XX,YY,ZZ = Ascii char's of waypoint id.   (0..9,A..Z)
                                 Takes the last 4 chars of name, assumes upper case only
                   Corresponding NMEA sentences: APB, AAM
*/
  int i;
  char wp[5]; memset(wp, 0, 5);
  for(i = 0; i < 4; i++)
    wp[i] = bu[3 + i];
  gpsd_report(session->context->debug, LOG_DATA,
	      "%02X: seatalk arrival info: perpendicular = %s, circle = %s, wp = %s \n", 
	      bu[0], 
	      bu[1] & 0x20 ? "passed" : "not passed", 
	      bu[1] & 0x40 ? "entered" : "not entered", wp);
  seatalk_print_command(bu, size, session);
  return 0;
}

static gps_mask_t seatalk_process_gps_dgps(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {
/*
 A5                GPS and DGPS Info
 A5  57  QQ  HH AA AA GG ZZ YY DD   GPS and DGPS Fix Info
                   Signal Quality= QQ&0xF, QQ&0x10: Signal Quality available flag
                   HDOP= HH&0x7C, HH&0x80: HDOP available flag
                   Antenna Height= AA AA (big endian)
                   Number of Sats= (QQ&0xE0)/16+(HH&0x1), HH&0x2: NumSats available flag
                   GeoSeperation= GG*16 (-2048....+2047 meters)
                   Differential age=(ZZ&0xE0)/2+(YY&0xF), YY&0x10: Diff. age available flag
                   Differential Station ID=(YY&0xC0)*4+DD, YY&0x20: Diff.St.ID available flag
                   Corresponding NMEA sentences: GGA, RMC, GSV, GLL, GGA
 A5  61  04  E2    , A5 8D ..., A5 98 ..., A5 B5 ..., A5 0C...  Unknown meaning
 A5  74  ID  ID ID ID ID   GPS Info: ID numbers of satellites
 A5  XD  NN  AA EE SS MM BB FF GG OO CC DD XX YY ZZ   GPS Info: Sat Position and Signal
                   Data of up to three sattelites [1,2,3] per datagram
                   Satellite number: [1] NN&0xFE, [2] (MM&0x70)/2+(BB&0x7), [3] CC&0x3F
                   Satellite azimuth:[1] AA*2+(EE&0x1), [2] (BB&0xF8)*2+(FF&0xF), [3] (CC&0xC0)*2+DD&0x7F
                   Satellite elevation:[1] (EE&0xFE)/2, [2] (FF&0xF0)/2+GG&0x7, [3] XX&0x7F
                   Satellite signal: [1] (SS&0xFE)/2, [2] (GG&0x80)/2+OO&0x3F, [3] (YY&0xFC)/2+ZZ&0x1

                It seems that there will be 4 sat info datagrams generated, the first with X=0 carries the position and signal data of the 1st 3 satellites. The second also with X=0, but NN&0x1 set and a length of 0x0C carries the data of the next 2 satellites and then the ID numbers of the 1st 4 sats. A datagram like the 1st one, but with X=2 carries data of 3 more sats [6,7,8]. It was not possible to get more than 8 sats mapped to SeaTalk. Finally a datagram with X=7 carries the next 5 ID numbers.

                   Corresponding NMEA sentences: GSV, GSA
*/

  gps_mask_t mask = 0;

  switch(bu[1]) {
  case 0x57: {

    int diff_age = ((bu[7] & 0xE0) >> 1) | (bu[8] & 0x0F);
    int diff_id    = ((((uint16_t)bu[8]) & 0x00C0) << 2) | bu[9];

    if(bu[2] & 0x10) {
      // at least the E85001 sets the same modes 1 .. 3
      session->newdata.mode = (bu[2] & 0x0F);
      mask = MODE_SET;

      if(session->newdata.mode > 0) {
	    session->gpsdata.status = STATUS_FIX;	/* could be DGPS_FIX, we can't tell */
	    mask |= STATUS_SET;
      }
    }

    // this is somewhat unclear, could be actually that 0x8000 is a valid flag or invalid-flag
    uint16_t alt = getbeu16(bu, 4);
    if((alt != 0x8000)) {
      session->newdata.altitude = alt;
      mask |= ALTITUDE_SET;
    }

    if(bu[3] & 0x80) {
	session->gpsdata.dop.hdop = ((bu[3] & 0x7C) >> 2);
	mask |= DOP_SET;
    }

    if(bu[3] & 0x02) {
	mask |= USED_IS;
	session->gpsdata.satellites_used = ((bu[2] & 0xE0) >> 4) | ((bu[3] & 0x01));
    }

    gpsd_report(session->context->debug, LOG_DATA,
		"seatalk gps_dgps: sats= %d, signal quality= %d, hdop= %.2f, antenna height= %.2f, diff age = %d, diff id = %d\n",
		session->gpsdata.satellites_used, 
		session->newdata.mode, 
		session->gpsdata.dop.hdop, 
		session->newdata.altitude,
		diff_age, diff_id);
  }
    break;

  case 0x0C: {
      gpsd_report(session->context->debug, LOG_DATA,
                  "seatalk data: sat numbers= %u, %u\n"
                  "              sat signal = %u, %u\n",
                  ((bu[2]&0xFE) >> 1), ((bu[6]&0x70) >> 1) + (bu[7]&0x07), 
                  (bu[5]&0xFE >> 1), ((bu[9]&0x80) >> 1) + (bu[10]&0x3F));
      break;
  }

  case 0x0D: 
  case 0x2D: 
  case 0x7D: 
  case 0x8D: {
      /*
        A5  XD  NN  AA EE SS MM BB FF GG OO CC DD XX YY ZZ   GPS Info: Sat Position and Signal
        00  01  02  03 04 05 06 07 08 09 10 11 12 13 14 15
            8d  32  00 0c 00 08 01 00 04 40 0e 00 01 00 02
             Data of up to three sattelites [1,2,3] per datagram
             Satellite number:   [1] NN&0xFE, [2] (MM&0x70)/2+(BB&0x7), [3] CC&0x3F
             Satellite azimuth:  [1] AA*2+(EE&0x1), [2] (BB&0xF8)*2+(FF&0xF), [3] (CC&0xC0)*2+DD&0x7F
             Satellite elevation:[1] (EE&0xFE)/2, [2] (FF&0xF0)/2+GG&0x7, [3] XX&0x7F
             Satellite signal:   [1] (SS&0xFE)/2, [2] (GG&0x80)/2+OO&0x3F, [3] (YY&0xFC)/2+ZZ&0x1
      */
      uint8_t no = 1;
      if(((bu[1]&0xF0) == 0) && (bu[2] & 0x01)) no = 2;
      else if(((bu[1]&0xF0) == 2) && (bu[2] & 0x01)) no = 4;
      else if(((bu[1]&0xF0) == 7) && (bu[2] & 0x01)) no = 5;
      gpsd_report(session->context->debug, LOG_DATA,
                  "seatalk data %u: sat numbers= %u, %u, %u\n"
                  "                 sat azi    = %u, %u, %u\n"
                  "                 sat elev   = %u, %u, %u\n"
                  "                 sat signal = %u, %u, %u\n",
                  no,
                  ((bu[2]&0xFE) >> 1), ((bu[6]&0x70) >> 1) + (bu[7]&0x07), (bu[11]&0x3F),
                  ((bu[3]<<1) + (bu[04]&0x01)), ((bu[7]&0xF8)<<1) + (bu[8]&0xF), ((bu[11]&0xC0) << 1) + (bu[12] & 0x7F),
                  ((bu[4] & 0xFE) >> 1), ((bu[8]&0xF0)>>1) + (bu[9]&0x07), (bu[13]&0x7F),
                  ((bu[5]&0xFE) >> 1), ((bu[9]&0x80) >> 1) + (bu[10]&0x3F), ((bu[14]&0xFC) >> 1) + (bu[15]&0x01));
  }
  case 0x98: {
      /*
        A5  98  NN  AA EE SS MM BB FF GG OO    GPS Info: Sat Position and Signal
        00  01  02  03 04 05 06 07 08 09 10 
             Data of up to three sattelites [1,2,3] per datagram
             Satellite number:   [1] NN&0xFE >> 1, [2] (MM&0x70)/2+(BB&0x7), [3] CC&0x3F
             Satellite azimuth:  [1] AA*2+(EE&0x1), [2] (BB&0xF8)*2+(FF&0xF), [3] (CC&0xC0)*2+DD&0x7F
             Satellite elevation:[1] (EE&0xFE)/2, [2] (FF&0xF0)/2+GG&0x7, [3] XX&0x7F
             Satellite signal:   [1] (SS&0xFE)/2, [2] (GG&0x80)/2+OO&0x3F, [3] (YY&0xFC)/2+ZZ&0x1
      */
      uint8_t no = 6;
      gpsd_report(session->context->debug, LOG_DATA,
                  "seatalk data %u: sat numbers= %u, %u, %u\n"
                  "                 sat signal = %u, %u, %u\n",
                  no,
                  ((bu[2]&0xFE) >> 1), 0, 0,
                  0, 0, 0);
  }
      
    break;
  case 0x74: {
      gpsd_report(session->context->debug, LOG_DATA,
                  "seatalk satellite ids: %u, %u, %u, %u, %u\n", bu[2], bu[3], bu[4], bu[5], bu[6]);
  }
    break;

  default:
    break;
  };

  seatalk_print_command(bu, size, session);

  return mask;
}

static gps_mask_t seatalk_process_unkown(uint8_t * bu, uint8_t size,
					struct gps_device_t *session) {

  gpsd_report(session->context->debug, LOG_DATA,
	      "seatalk unknown: \n");
  seatalk_print_command(bu, size, session);
  return 0;
}

static int seatalk_is_command(struct gps_packet_t *lexer, uint8_t c, int parerr) {

  /* generally per definition: 
     - even parity bit set (1, high) if count of 1s is odd
     - odd parity bit set (0, low) if count of 1s is even
  
  // 9th bit for command flag is interpreted as parity bit here
     we check for even parity

  // if the command flag is set then that means that parity even bit is set in the data stream
  */

  int cmdFlag = 0;

  int parity = getParity(c); // 1 == odd, 0 == even

  if(!parity) {
    // input char has even parity
    if(parerr) {
      // and the parity error is signaled 
      // which means parity bit is 1
      cmdFlag = 1;
       
    } else {
      // and no parity error is signaled
      // which means that the parity bit is not set
      cmdFlag = 0;
    }

  } else {
    // input has odd parity
    if(parerr) {
      // and parity error is signaled
      // -> parity bit 0
      cmdFlag = 0;
    } else {
      cmdFlag = 1;
    }
  }

  gpsd_report(lexer->debug, LOG_RAW+2,
	      "%02X %s %s\n", c, cmdFlag?"(C)":"", parity?"O":"E");

  return cmdFlag;

}

/*@-usereleased@*/


static const char msg_00[] = {"Depth below transducer"};
static const char msg_10[] = {"Apparent Wind Angle"};
static const char msg_11[] = {"Apparent Wind Speed"};
static const char msg_20[] = {"Speed through water"};
static const char msg_21[] = {"Trip Milage"};
static const char msg_22[] = {"Total Milage"};
static const char msg_23[] = {"Water temperature"};
static const char msg_25[] = {"Distance log (trip / total)"};
static const char msg_26[] = {"Speed through water"};
static const char msg_27[] = {"Water temperature"};
static const char msg_36[] = {"Cancel MOB (Man Over Board) condition"};
static const char msg_50[] = {"LAT position"};
static const char msg_51[] = {"LON position"};
static const char msg_52[] = {"Speed over Ground"};
static const char msg_53[] = {"Course over Ground (COG)"};
static const char msg_54[] = {"GMT-time: HH hours"};
static const char msg_56[] = {"Date"};
static const char msg_57[] = {"Sat info"};
static const char msg_58[] = {"Raw lat/lon"};
static const char msg_59[] = {"Set count down timer"};
static const char msg_6E[] = {"MOB"};
static const char msg_82[] = {"Target waypoint name"};
static const char msg_84[] = {"Compass heading  Autopilot course and Rudder position"};
static const char msg_85[] = {"Navigation to waypoint information"};
static const char msg_89[] = {"Compass heading sent by ST40"};
static const char msg_99[] = {"Compass variation sent by ST40"};
static const char msg_9C[] = {"Compass heading and Rudder position"};
static const char msg_9E[] = {"Waypoint definition"};
static const char msg_A1[] = {"Destination Waypoint Info"};
static const char msg_A2[] = {"Arrival Info"};
static const char msg_A5[] = {"GPS and DGPS Info"};

static const char msg_FF[] = {"Unkown sentence"};


static const char msg_error [] = {"**error**"};

static struct st_phrase st_process[] = {
  {0x00,  0x02, seatalk_process_depth,        &msg_00[0]},	
  {0x10,  0x01, seatalk_process_wind_angle,   &msg_10[0]},
  {0x11,  0x01, seatalk_process_wind_speed,   &msg_11[0]},
  {0x20,  0x01, seatalk_process_speed,        &msg_20[0]},
  {0x21,  0x02, seatalk_process_milage,       &msg_21[0]},
  {0x22,  0x02, seatalk_process_milage,       &msg_22[0]},
  {0x23,  0x01, seatalk_process_watertemp,    &msg_23[0]},
  {0x25,  0x04, seatalk_process_distlog,      &msg_25[0]},
  {0x26,  0x04, seatalk_process_speed,        &msg_20[0]},
  {0x27,  0x01, seatalk_process_watertemp,    &msg_23[0]},
  {0x36,  0x00, seatalk_process_MOB_cancel,   &msg_36[0]},
  {0x50,  0x02, seatalk_process_lat,          &msg_50[0]},
  {0x51,  0x02, seatalk_process_lon,          &msg_51[0]},
  {0x52,  0x01, seatalk_process_sog,          &msg_52[0]},
  {0x53,  0x00, seatalk_process_cog,          &msg_53[0]},
  {0x54,  0x01, seatalk_process_time,         &msg_54[0]},
  {0x56,  0x01, seatalk_process_date,         &msg_56[0]},
  {0x57,  0x00, seatalk_process_noofsats,     &msg_57[0]},
  {0x58,  0x05, seatalk_process_lat_lon_raw,  &msg_58[0]},
  {0x59,  0x02, seatalk_process_count_down,   &msg_59[0]},
  {0x6E,  0x07, seatalk_process_MOB,          &msg_6E[0]},
  {0x82,  0x05, seatalk_process_target_wp,    &msg_82[0]},
  {0x84,  0x06, seatalk_process_compass_hdg,  &msg_84[0]},
  {0x85,  0x06, seatalk_process_nav_to_wp,                   &msg_85[0]},
  {0x89,  0x02, seatalk_process_compass_hdg_st40,            &msg_89[0]},
  {0x99,  0x00, seatalk_process_compass_variation_st40,      &msg_99[0]},
  {0x9c,  0x01, seatalk_process_hdg_rudder_pos,              &msg_9C[0]},
  {0x9e,  0x0C, seatalk_process_wp_definition,               &msg_9E[0]},
  {0xa1,  0x0D, seatalk_process_dest_wp,                     &msg_A1[0]},
  {0xa2,  0x04, seatalk_process_arrival,                     &msg_A2[0]},
  {0xa5,  0x80, seatalk_process_gps_dgps,                    &msg_A5[0]}
};

/*@+usereleased@*/

static gps_mask_t process_seatalk(uint8_t * cmdBuffer, uint8_t size,
					struct gps_device_t *session) {
  gps_mask_t mask = 0;

  uint8_t i = 0;

  while(i < (uint8_t)(sizeof(st_process) / sizeof(st_process[0]))) {
    if(st_process[i].cmdId == cmdBuffer[0]) {
      mask = st_process[i].decoder(cmdBuffer, size, session);
      break;
    }
    i++;
  }

  if ((mask & TIME_SET) != 0) {

    session->newdata.time = gpsd_utc_resolve(session, 
					     &session->driver.seatalk.date, 0);
    session->newdata.time += session->driver.seatalk.offset;

    gpsd_external_report(session->context->debug, LOG_DATA,
			 "time is %2f = %d-%02d-%02dT%02d:%02d:%02dZ offset = %05.2f sec\n",
			 session->newdata.time,
			 1900 + session->driver.seatalk.date.tm_year,
			 session->driver.seatalk.date.tm_mon + 1,
			 session->driver.seatalk.date.tm_mday,
			 session->driver.seatalk.date.tm_hour,
			 session->driver.seatalk.date.tm_min,
			 session->driver.seatalk.date.tm_sec,
			 session->driver.seatalk.offset);
  }

  if((mask & LATLON_SET) 
     && session->driver.seatalk.lat_set 
     && session->driver.seatalk.lon_set) {

    /* only go out with LATLON if both are set */
    session->newdata.latitude = session->driver.seatalk.lat;
    session->newdata.longitude = session->driver.seatalk.lon;

    session->driver.seatalk.lat_set = 0;
    session->driver.seatalk.lon_set = 0;

  } else {

    mask &= ~LATLON_SET;

  }

  return mask;

}

/*@+nullassign@*/


#define PARITY_WARN          1
#define PARITY_ERROR         2

#define PARITY_SET           3

#define SEATALK_CMD_PENDING  4

#define GROUND_STATE         0
#define SEATALK_COMMAND      1 << 4
#define SEATALK_LENGTH       2 << 4
#define SEATALK_PAY          3 << 4
#define SEATALK_RECOGNIZED   4 << 4

static char * seatalk_state_table[] = {
  "GROUND_STATE",
  "SEATALK_COMMAND",
  "SEATALK_LENGTH",
  "SEATALK_PAY", 
  "SEATALK_RECOGNIZED",
};

static void seatalk_nextstate(struct gps_packet_t *lexer, unsigned char c)
{
    static int n = 0;
    int parity   = 0;
    int cmd      = 0;

    /* we need to handle the parity error
       in parallel to the real state as in every real
       state there can be a parity error announced */

    if((lexer->state & SEATALK_CMD_PENDING) == 0) {

      if(lexer->state & PARITY_ERROR) {

	parity   = 1;
	lexer->state &= ~PARITY_SET;

      } else if(lexer->state & PARITY_WARN) {

	if(c == 0x00) {
	  lexer->state |= PARITY_ERROR;
	  //printf("!", buf);
	} else {
	  lexer->state &= ~PARITY_WARN;
	}

      } else {

	// default case actually
	if(c == 0xff) {
	  //printf("?", buf);
	  lexer->state |= PARITY_WARN;
	}

      }
    } else {

      // the new character c is the pushed back character from a command      
      cmd = 1;

      // just to be sure we reset parity, actually would be a bug if set
      lexer->state &= ~PARITY_SET;
    }

    if((lexer->state & PARITY_SET) == 0) {

      n++;

      if((!cmd) && seatalk_is_command(lexer, c, parity)) {
	gpsd_report(lexer->debug, LOG_RAW + 2, "command flag\n");
	cmd = 1;
      }

      // many states need to be made much more robust against errornous input
      switch(lexer->state & 0xF0) {

        // no parity warn or error char
      case GROUND_STATE:
      case SEATALK_RECOGNIZED:
	if(cmd) {
	  lexer->state = SEATALK_COMMAND;
	  lexer->length = 0x80;
	  if((st_fixed_lengths[c] & 0x80) == 0) {
	    lexer->length = st_fixed_lengths[c];
	    gpsd_report(lexer->debug, LOG_RAW + 2,
			"%08ld: fixed length assigned based on table: %lu\n",
			lexer->char_counter, lexer->length);
	  }
	} else {
	  lexer->state = GROUND_STATE;
	}
	break;

      case SEATALK_COMMAND:
	if(!cmd) {
	  lexer->state = SEATALK_LENGTH;
	  // thats how the length is defined: 
	  // total datagram length = len-field + 1 as of this position
	  if((lexer->length & 0x80) == 0) {
	    // fixed length based on lookup table
	    if(lexer->length != (c & 0x0f)) {
	      gpsd_report(lexer->debug, LOG_RAW + 2,
			  "%08ld: wrong fixed length found: %lu != %u\n",
                      lexer->char_counter, lexer->length, (c & 0x0f));
	      lexer->state = GROUND_STATE;
	      break;
	    }
	  }
	  lexer->length = c & 0x0f; 
	  gpsd_report(lexer->debug, LOG_RAW + 2,
		      "%08ld: length found: %lu\n",
		      lexer->char_counter, lexer->length);
	} else {
	  lexer->state = GROUND_STATE | SEATALK_CMD_PENDING;
	  character_pushback(lexer);
	}
	break;

      case SEATALK_LENGTH:
	if(!cmd) {
	  if (lexer->length == 0) // special case of 3 char datagrams
	    lexer->state = SEATALK_RECOGNIZED;
          else
 	    lexer->state = SEATALK_PAY;
	} else {
	  lexer->state = GROUND_STATE | SEATALK_CMD_PENDING;
	  character_pushback(lexer);
	}
	break;

      case SEATALK_PAY:
	gpsd_report(lexer->debug, LOG_RAW + 2,
		    "%08ld: length= %lu\n",
		    lexer->char_counter, lexer->length);
	if(!cmd) {
	  if (--lexer->length == 0)
	    lexer->state = SEATALK_RECOGNIZED;
	} else {
	  gpsd_report(lexer->debug, LOG_RAW + 2,
		      "%08ld: switching back to ground state with length= %lu\n",
		      lexer->char_counter, lexer->length);
	  lexer->state = GROUND_STATE | SEATALK_CMD_PENDING;
	  character_pushback(lexer);
	}
	break;

      default:
	break;
      };
    }
}

static void seatalk_packet_parse(struct gps_packet_t * lexer)
/* grab a packet from the input buffer */
{
    lexer->outbuflen = 0;
    while (packet_buffered_input(lexer) > 0) {
	/*@ -modobserver @*/
	unsigned char c = *lexer->inbufptr++;
	/*@ +modobserver @*/
	seatalk_nextstate(lexer, c);
	gpsd_report(lexer->debug, LOG_RAW + 2,
		    "%08ld: character '%c' [%02x] @ %p, new state: %s, %c%c\n",
		    lexer->char_counter, (isprint(c) ? c : '.'), c,
		    lexer->inbufptr - 1,
		    seatalk_state_table[(lexer->state & 0xF0) >> 4],
		    (lexer->state & PARITY_WARN)?'?':'-',
		    (lexer->state & PARITY_ERROR)?'!':'-');
	lexer->char_counter++;
	
	if ((lexer->state & 0xF0) == GROUND_STATE) {

	    packet_discard(lexer);

	} else if(lexer->state & PARITY_SET) {

	    character_skip(lexer);

	} else if ((lexer->state & 0xF0) == SEATALK_RECOGNIZED) {

		packet_accept(lexer, SEATALK_PACKET);
		packet_discard(lexer);
		gpsd_report(lexer->debug, LOG_RAW + 2,
		    "%08ld: ptr= %p, s= %p, %lu\n",
			    lexer->char_counter, 
			    lexer->inbufptr, 
			    lexer->inbuffer, 
			    lexer->inbuflen);
		
		break;
	}
    } // while
}



ssize_t seatalk_packet_get(struct gps_device_t *session)
/* grab a packet; return -1=>I/O error, 0=>EOF, BAD_PACKET or a length */
{
    ssize_t recvd;

    struct gps_packet_t *lexer = &session->packet;;

    /*@ -modobserver @*/
    errno = 0;

    recvd = read(session->gpsdata.gps_fd, lexer->inbuffer + lexer->inbuflen,
		 sizeof(lexer->inbuffer) - (lexer->inbuflen));
    /*@ +modobserver @*/
    if (recvd == -1) {
	if ((errno == EAGAIN) || (errno == EINTR)) {
	    gpsd_report(lexer->debug, LOG_RAW + 2, "no bytes ready\n");
	    recvd = 0;
	    /* fall through, input buffer may be nonempty */
	} else {
	    gpsd_report(lexer->debug, LOG_RAW + 2,
			"errno: %s\n", strerror(errno));
	    return -1;
	}
    } else {
	if (lexer->debug >= LOG_IO) {
	    char scratchbuf[MAX_PACKET_LENGTH*2+1];
	    gpsd_external_report(lexer->debug, LOG_IO,
				 "Read %zd chars to buffer offset %zd (total %zd): %s\n",
				 recvd, lexer->inbuflen, lexer->inbuflen + recvd,
				 gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
						 (char *)lexer->inbuffer + lexer->inbuflen, (size_t) recvd));
	}
	lexer->inbuflen += recvd;
    }
    gpsd_report(lexer->debug, LOG_SPIN,
		"seatalk_packet_get() fd %d -> %zd (%d)\n",
		session->gpsdata.gps_fd, recvd, errno);

    /*
     * Bail out, indicating no more input, only if we just received
     * nothing from the device and there is nothing waiting in the
     * packet input buffer.
     */
    if (recvd <= 0 && packet_buffered_input(lexer) <= 0) {
        gpsd_report(lexer->debug, LOG_SPIN,
		    "Seatalk: bailing out with no more input\n");
	return recvd;
    }

    /* Otherwise, consume from the packet input buffer */
    /* coverity[tainted_data] */
    seatalk_packet_parse(lexer);

    /* if input buffer is full, discard */
    if (sizeof(lexer->inbuffer) == (lexer->inbuflen)) {
	/* coverity[tainted_data] */
	packet_discard(lexer);
	lexer->state = GROUND_STATE;
        gpsd_report(lexer->debug, LOG_SPIN,
		    "Seatalk: input buffer full, discard\n");
    }

    /*
     * If we gathered a packet, return its length; it will have been
     * consumed out of the input buffer and moved to the output
     * buffer.  We don't care whether the read() returned 0 or -1 and
     * gathered packet data was all buffered or whether ot was partly
     * just physically read.
     *
     * Note: this choice greatly simplifies life for callers of
     * packet_get(), but means that they cannot tell when a nonzero
     * return means there was a successful physical read.  They will
     * thus credit a data source that drops out with being alive
     * slightly longer than it actually was.  This is unlikely to
     * matter as long as any policy timeouts are large compared to
     * the time required to consume the greatest possible amount
     * of buffered input, but if you hack this code you need to
     * be aware of the issue. It might also slightly affect
     * performance profiling.
     */
    if (lexer->outbuflen > 0)
	return (ssize_t) lexer->outbuflen;
    else
	/*
	 * Otherwise recvd is the size of whatever packet fragment we got.
	 * It can still be 0 or -1 at this point even if buffer data
	 * was consumed.
	 */
	return recvd;
}

gps_mask_t seatalk_parse_input(struct gps_device_t *session) {

  uint8_t *sentence = (uint8_t *)session->packet.outbuffer;
  gps_mask_t mask = 
    process_seatalk(sentence, session->packet.outbuflen, session);
  return mask;
}

static void seatalk_set_serial(struct gps_device_t *session) {

  struct termios tty;

  memset (&tty, 0, sizeof tty);

  if (tcgetattr(session->gpsdata.gps_fd, &session->ttyset_old) != 0) {
    gpsd_report(session->context->debug, LOG_ERROR, 
		"SEATALK tcgetattr error %d: %s\n", errno, strerror(errno));
    session->gpsdata.gps_fd = -1;
    return;
  }

  // save old parameters
  (void)memcpy(&session->ttyset,
	       &session->ttyset_old, sizeof(session->ttyset));

  memset(session->ttyset.c_cc, 0, sizeof(session->ttyset.c_cc));
  session->ttyset.c_cc[VMIN]      =   1;                  // read doesn't block (non canonical)
  session->ttyset.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

  /* Set Baud Rate */
  cfsetospeed (&session->ttyset, (speed_t)B4800);
  cfsetispeed (&session->ttyset, (speed_t)B4800);

  /* Setting other Port Stuff */

  session->ttyset.c_cflag     &=  ~CSIZE;
  session->ttyset.c_cflag     |=  CS8;

  session->ttyset.c_cflag     &=  ~CSTOPB;       // 1 stop

  session->ttyset.c_cflag     &=  ~CRTSCTS;       // no flow control
  session->ttyset.c_cflag     |= CLOCAL;     // 

  session->ttyset.c_cflag     |= CREAD;     // turn on READ

  session->ttyset.c_cflag     |=  PARENB;        // Make 8n1
  session->ttyset.c_cflag     &=  ~PARODD;        // Make 8n1

  session->ttyset.c_iflag     &=  ~IGNBRK;
  session->ttyset.c_iflag     |=  BRKINT;

  session->ttyset.c_iflag     &=  ~IGNPAR;
  session->ttyset.c_iflag     |=  PARMRK;  // this is basically the core of detecting the command start
  session->ttyset.c_iflag     |=  INPCK;

  session->ttyset.c_iflag     &=  ~ISTRIP;

  // non canonical, we want every char directly, no signals
  session->ttyset.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG);        

  // no conversions or ignoring any cr or nl or conversion between them
  session->ttyset.c_iflag     &=  ~(INLCR | IGNCR | ICRNL | IUCLC | IXANY | IXON | IMAXBEL);    
  session->ttyset.c_oflag     &=  ~(OCRNL | OFDEL | OFILL | OLCUC | ONLCR | ONLRET | ONOCR);

  session->ttyset.c_lflag     &=  ~(OPOST | XCASE);        

  // Flush Port, then applies attributes 
  tcflush( session->gpsdata.gps_fd, TCIFLUSH );

  if ( tcsetattr ( session->gpsdata.gps_fd, TCSANOW, &session->ttyset ) != 0) {
    gpsd_report(session->context->debug, LOG_ERROR, 
		"SEATALK tcsetattr error %d: %s\n", errno, strerror(errno));
    session->gpsdata.gps_fd = -1;
    return;
  }

}

static void seatalk_driver_init(struct gps_device_t * session) {

  uint8_t cmd;
  uint8_t i = 0;

  session->driver.seatalk.lon = NAN;
  session->driver.seatalk.lat = NAN;
  session->driver.seatalk.lat_set = 0;
  session->driver.seatalk.lon_set = 0;

  session->driver.seatalk.offset = 0;
  session->driver.seatalk.lastts = 0;

  memset(st_fixed_lengths, 0x80, 255);

  while(i < (uint8_t)(sizeof(st_process) / sizeof(st_process[0]))) {
    cmd = st_process[i].cmdId;
    st_fixed_lengths[cmd] = st_process[i].fixed_length;
    i++;
  }

}

#ifndef S_SPLINT_S
int seatalk_open(struct gps_device_t *session) {

  char path[strlen(session->gpsdata.dev.path)], *port;
  (void)strlcpy(path, session->gpsdata.dev.path + 5, sizeof(path));
	
  session->gpsdata.gps_fd = -1;
  port = strchr(path, ':');

  gpsd_report(session->context->debug, LOG_INF, 
	      "SEATALK open: opening device: %s\n", path);

  if(port == NULL) {

    mode_t mode = (mode_t) O_RDWR;

    session->gpsdata.gps_fd = open( path, mode | O_NOCTTY );

    seatalk_set_serial(session);
    
  } else {

    *port++ = '\0';
    gpsd_report(session->context->debug, LOG_INF, "opening TCP SEATALK test feed at %s, port %s.\n", path,
		port);

    if ((session->gpsdata.gps_fd = netlib_connectsock(AF_UNSPEC, path, port, "tcp")) < 0) {
      gpsd_report(session->context->debug, LOG_ERROR, "TCP device open error %s.\n",
		  netlib_errstr(session->gpsdata.gps_fd));

    } else 

      gpsd_report(session->context->debug, LOG_SPIN, "TCP device opened on fd %d\n", 
		  session->gpsdata.gps_fd);

  }

  seatalk_driver_init(session);

  if( session->gpsdata.gps_fd > 0 ) {
    //TODO: WRONG - but nice hack to convince gpsd
    session->sourcetype = source_can;
    session->servicetype = service_sensor;
    gpsd_switch_driver(session, "SEATALK");
  }

  return session->gpsdata.gps_fd;
}

// we are relying on gpsd_close to restore all tty parameters

#endif /* of ifndef S_SPLINT_S */


/* *INDENT-OFF* */
const struct gps_type_t driver_seatalk = {
    .type_name      = "SEATALK",       /* full name of type */
    .packet_type    = SEATALK_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_STICKY,	/* remember this */
    .trigger	    = NULL,		/* detect their main sentence */
    .channels       = 12,		/* not an actual GPS at all */
    .probe_detect   = NULL,
    .get_packet     = seatalk_packet_get,	/* how to get a packet */
    .parse_packet   = seatalk_parse_input,	/* how to interpret a packet */
    .rtcm_writer    = NULL,		/* Don't send RTCM to this */
    .event_hook     = NULL,
#ifdef RECONFIGURE_ENABLE
    .speed_switcher = NULL,		/* no speed switcher */
    .mode_switcher  = NULL,		/* no mode switcher */
    .rate_switcher  = NULL,		/* no rate switcher */
    .min_cycle      = 1,		/* nominal 1-per-second GPS cycle */
#endif /* RECONFIGURE_ENABLE */
#ifdef CONTROLSEND_ENABLE
    .control_send   = NULL,		/* how to send control strings */
#endif /* CONTROLSEND_ENABLE */
};
/* *INDENT-ON* */

/* end */

#endif /* of  defined(SEATALK_ENABLE) */
