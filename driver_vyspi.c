/*
 * NMEA2000 over CAN.
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
#if defined(VYSPI_ENABLE)
#include "driver_vyspi.h"
#include "bits.h"

#define LOG_FILE 1
#define VYSPI_RESET 0x04

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02


typedef struct PGN
    {
    unsigned int  pgn;
    unsigned int  fast;
    unsigned int  type;
    gps_mask_t    (* func)(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
    const char    *name;
    } PGN;

/*@-nullassign@*/
static void print_data(struct gps_context_t *context,
		       unsigned char *buffer, int len, PGN *pgn)
{
#ifdef LIBGPS_DEBUG
    /*@-bufferoverflowhigh@*/
    if ((libgps_debuglevel >= LOG_IO) != 0) {
	int   l1, l2, ptr;
	char  bu[128];

        ptr = 0;
        l2 = sprintf(&bu[ptr], "got data:%6u:%3d: ", pgn->pgn, len);
	ptr += l2;
        for (l1=0;l1<len;l1++) {
            if (((l1 % 20) == 0) && (l1 != 0)) {
	        gpsd_report(context->debug, LOG_IO,"%s\n", bu);
		ptr = 0;
                l2 = sprintf(&bu[ptr], "                   : ");
		ptr += l2;
            }
            l2 = sprintf(&bu[ptr], "0x%02x ", (unsigned int)buffer[l1]);
	    ptr += l2;
        }
        gpsd_report(context->debug, LOG_IO,"%s\n", bu);
    }
    /*@+bufferoverflowhigh@*/
#endif
}

static gps_mask_t get_mode(struct gps_device_t *session)
{
    if (session->driver.nmea2000.mode_valid & 1) {
        session->newdata.mode = session->driver.nmea2000.mode;
    } else {
        session->newdata.mode = MODE_NOT_SEEN;
    }
    
    if (session->driver.nmea2000.mode_valid & 2) {
        return MODE_SET | USED_IS;
    } else {
        return MODE_SET;
    }
}

static int decode_ais_header(struct gps_context_t *context,
    unsigned char *bu, int len, struct ais_t *ais, unsigned int mask)
{
    if (len > 4) {
        ais->type   = (unsigned int) ( bu[0]       & 0x3f);
	ais->repeat = (unsigned int) ((bu[0] >> 6) & 0x03);
	ais->mmsi   = (unsigned int)  getleu32(bu, 1);
	ais->mmsi  &= mask;
	gpsd_report(context->debug, LOG_INF,
		    "NMEA2000 AIS  message type %u, MMSI %09d:\n",
		    ais->type, ais->mmsi);
	printf("NMEA2000 AIS  message type %2u, MMSI %09u:\n",
	       ais->type, ais->mmsi);
	return(1);
    } else {
        ais->type   =  0;
	ais->repeat =  0;
	ais->mmsi   =  0;
	gpsd_report(context->debug, LOG_ERROR,
		    "NMEA2000 AIS  message type %u, too short message.\n",
		    ais->type);
	printf("NMEA2000 AIS  message type %u, too short message.\n",
	       ais->type);
    }
    return(0);
}


static void decode_ais_channel_info(unsigned char *bu,
				    int len,
				    unsigned int offset,
				    struct gps_device_t *session)
{
    unsigned int pos, bpos;
    uint16_t x;

    pos = offset / 8;
    bpos = offset % 8;
    if (pos >= (unsigned int)len) {
        session->driver.aivdm.ais_channel = 'A';
	return;
    }
    x = getleu16(bu, pos);
    x = (uint16_t)((x >> bpos) & 0x1f);
    switch (x) {
    case 1:
    case 3:
        session->driver.aivdm.ais_channel = 'B';
	break;
    default:
        session->driver.aivdm.ais_channel = 'A';
	break;
    }
    return;
}


static int ais_turn_rate(int rate)
{
    if (rate < 0) {
        return(-ais_turn_rate(-rate));
    }
    return((int)(4.733 * sqrt(rate * RAD_2_DEG * .0001 * 60.0)));
}

static double ais_direction(unsigned int val, double scale)
{
    if ((val == 0xffff) && (scale == 1.0)) {
        return(511.0);
    }
    return(val * RAD_2_DEG * 0.0001 * scale);
}

/*
 *   PGN 59392: ISO  Acknowledgment
 */
static gps_mask_t hnd_059392(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 Control Byte
    2 Group Function Value
    3 Reserved Bits
    4 PGN of Requested Information
  */

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 59904: ISO Request
 */
static gps_mask_t hnd_059904(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/*
 *   PGN 60928: ISO  Address Claim
 */
static gps_mask_t hnd_060928(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    /*
    ([14], [15])
     1 0             Unique Number (ISO Identity Number)
       1 
       2 bits 0-4
     2 2 bits 5-7    Manufacturer Code
       3
     3 4 bits 0-2    ECU Device Instance Lower (ISO ECU Instance)
     4 4 bits 3-7    Device Instance Upper (ISO Function Instance)
     5 5             Device Function (ISO Function)
     6 6 bits 0      Reserved
     7 6 bits 1-7    Device Class (Vehicel system)
     8 7 bits 0-3    System Instance (ISO Device Class Instance)
     9 7 bits 4-6    Industry Group
    10 7 bit  7      Reserved (ISO Self Configurable)
    */

    uint64_t ECU_name;

    uint32_t uniq;
    uint16_t manu;
    uint8_t  grp;

    // This actually all we need. We can disect this for educational purposes though.
    ECU_name = getleu64(bu, 0);    

    uniq   = (ECU_name >>  0) & 0x1fffff,
    manu   = (ECU_name >> 21) & 0x7ff,
    grp    = (getub(bu, 7) >> 4) & 0x07;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Id= %u, Manufacturer= %u, Industry Group = %u\n",
		uniq,
		manu,
		grp);

    return(0);
}


/*
 *   PGN 126208: NMEA Command/Request/Acknowledge
 */
static gps_mask_t hnd_126208(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 126464: ISO Transmit/Receive PGN List
 */
static gps_mask_t hnd_126464(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, 
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/*
 *   PGN 126996: ISO  Product Information
 */
static gps_mask_t hnd_126996(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127488: Engine Parameter Rapid Update
 */
static gps_mask_t hnd_127488(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* [17] 
    1   0   uint8  Engine instance 
    2 1-2   uint16 RPM (x .25 RPM)
    3 3-4   uint16 Boost Pressure (100Pa)
    4   6   sint8  Engine Tilt/Trim (+/- 1%)
    5 6-7   uint16 Reserved Bits 
   */
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127489: Engine Parameter Dynamic
 */
static gps_mask_t hnd_127489(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* [17]
    1      0           uint8    Engine Instance  (port, starboard, forward, etc.)
    2   1- 2           uint16   Engine Oil Pressure (100Pa)
    3   3- 4           uint16   Engine Oil Temperature (0.1K)
    4   5- 6           uint16   Engine Temperature (0.1K)
    5   7- 8           sint16   Alternator potential (0.01V)
    6   9-10           sint16   Fuel rate (cm³ / hour)
    7  11-14           uint32   Total engine hours (1sec)
    8  15-16           uint16   Engine coolant pressure (100Pa)
    9  17-18           uint16   Fuel Pressure (1000Pa)
    10    19           uint8    Not Available
    11    20 bits 0-7  bits     Engine Discrete Status 1
    12    21 bits 0-7  bits     Engine Discrete Status 2
    13    22           sint8    Percent Engine Load (1%)
    14    23           sint8    Percent Engine Torque (1%)
   */
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127505: Fluid Level
 */
static gps_mask_t hnd_127505(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* [17]
    1   0           Fluid Instance
    2   1  bits 3-7 Fluid Type
    3 1-2           Fluid Level
    4               Tank Capacity
    5 Reserved Bit
    */

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127257: Attitude
 */
static gps_mask_t hnd_127257(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* [8] The SSC200 uses this PGN to indicate the vessel’s attitude (pitch and roll). The Yaw (field 2) is not used, therefore this field always contains 0x7FFF (data not available).

           1:  SID – The sequence identifier field is used to tie related PGNs together. For example, the SSC200 will transmit identical SIDs for Vessel Heading (PGN 127250), Attitude (127257), and Rate of Turn (127251) to indicate that the readings are linked together (i.e., the data from each PGN was taken at the same time although they are reported at slightly different times).

           2:  Yaw – This field always contains a value of 0x7FFF (data not available).

           3:  Pitch – This field is used to report the vessel’s pitch.

           4:  Roll – This field is used to report the vessel’s roll.

           5:  Reserved – This field is reserved by NMEA; therefore, the SSC200 sets all bits to a logic 1.
  */
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127258: GNSS Magnetic Variation
 */
static gps_mask_t hnd_127258(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    /*
    1 Sequence ID
    2 Variation Source
    3 Reserved Bits
    4 Age of Service (Date)
    5 Variation
    6 Reserved Bits
    */ 

    uint8_t sid;
    uint8_t src;
    uint8_t reserved1;
     int8_t age;
    double  var;
    uint16_t reserved2;

    sid             = bu[0];

    src             = (bu[1] >> 0) & 0x0f;          // TODO, 5 == WMM2005 ([7]), == manual ([3])

    reserved1       = (bu[1] >> 4) & 0x0f;          // 4 bits ([7]), always 0xf

    // TODO: 0x7fff for N/A ([8])
    age             = getles16(bu, 2);          

    var             = getles16(bu, 4) * RAD_2_DEG * 0.0001;
    reserved2       = getleu16(bu, 6);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, Variation Source= %u, Reserved Bits= %u\n",
		sid,
		src,
		reserved1);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Age of Service (Date)= %u, Variation= %f, Reserved Bits= 0x%04x\n",
		age,
		var,
		reserved2);

    return(0);
}


/*
 *   PGN 129025: GNSS Position Rapid Update
 */
static gps_mask_t hnd_129025(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    /*@-type@*//* splint has a bug here */
    session->newdata.latitude = getles32(bu, 0) * 1e-7;
    session->newdata.longitude = getles32(bu, 4) * 1e-7;
    /*@+type@*/

    (void)strlcpy(session->gpsdata.tag, "129025", sizeof(session->gpsdata.tag));

    gpsd_report(session->context->debug, LOG_IO,
		"                   lat = %f, lon = %f\n",
		session->newdata.latitude,
		session->newdata.longitude);

    return LATLON_SET | get_mode(session);
}


/*
 *   PGN 129026: GNSS COG and SOG Rapid Update
 */
static gps_mask_t hnd_129026(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  // TODO 0 = True, 1 = magnetic, 2 == error, 3 == - ([1])
    uint8_t COG_Reference;
    uint8_t Reserved1;
    uint16_t Reserved2;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    session->driver.nmea2000.sid[0]  =  bu[0];

    // TODO: 0: True  ([7]), 1: Magnetic (guess)
    // observed: xfd/fc toggled
    COG_Reference = (bu[1] >> 0) & 0x03; 
    Reserved1     = (bu[1] >> 2) & 0xff;  // TODO: [7]: 6 bits in total

    /*@-type@*//* splint has a bug here */
    session->newdata.track           =  getleu16(bu, 2) * 1e-4 * RAD_2_DEG;
    session->newdata.speed           =  getleu16(bu, 4) * 1e-2;
    /*@+type@*/

    Reserved2     = getleu16(bu, 6);     // [7]: 2 bytes

    (void)strlcpy(session->gpsdata.tag, "129026", sizeof(session->gpsdata.tag));

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, cog ref= %s, track= %fdeg, speed= %fm/s; %fknots\n",
		session->driver.nmea2000.sid[0],
		(COG_Reference == 0) ? "True" : "Magnetic",
		session->newdata.track,
		session->newdata.speed,
		session->newdata.speed * 3600.0 / 1852.0);

    return SPEED_SET | TRACK_SET | get_mode(session);
}


/*
 *   PGN 126992: GNSS System Time
 */
static gps_mask_t hnd_126992(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    uint8_t        sid;

    // TODO 0=GPS,1=GLONASS,2=Radio Station,3=Local Cesium clock,4=Local Rubidium clock,5=Local Crystal clock ([1])
    uint8_t        source;   // based on reserved bits: 4 bits, 
    uint8_t        reserved; // [7] 4 bits

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    sid        = bu[0];
    source     = (bu[1] >> 0) & 0x0f; 
    reserved   = (bu[1] >> 4) & 0x0f; 

    /*@-type@*//* splint has a bug here */
    session->newdata.time = getleu16(bu, 2)*24*60*60 + getleu32(bu, 4)/1e4;
    /*@+type@*/

    char tbuf[JSON_DATE_MAX + 1];
    unix_to_iso8601(session->newdata.time, tbuf, sizeof(tbuf));

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, source = %u, time = %s\n",
		sid,
		source,
		tbuf);

    (void)strlcpy(session->gpsdata.tag, "126992", sizeof(session->gpsdata.tag));

    return TIME_SET | get_mode(session);
}


static const int mode_tab[] = {MODE_NO_FIX, MODE_2D,  MODE_3D, MODE_NO_FIX,
			       MODE_NO_FIX, MODE_NO_FIX, MODE_NO_FIX, MODE_NO_FIX};

/*
 *   PGN 129539: GNSS DOPs
 */
static gps_mask_t hnd_129539(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* 
     GPS100 will transmit identical SIDs for 126992 (System Time), 128259
     (Speed), 129026 (COG and SOG, Rapid Update), 129029 (GNSS Position Data),
     129539 (GNSS DOPs), and 129540 (GNSS Satellites in View) to indicate that the
     readings are linked together 

    2: Set Mode – This field is used to indicate the desired mode of operation: 0 = 1D. 1 =
      2D, 2 = 3D, 3 = Auto (factory default), 4-5 = Reserved, 6 = Error, 7 = Null.
    3: Op Mode – This field is used to indicate the actual current mode of operation: 0 =
      1D. 1 = 2D, 2 = 3D, 3 = Auto (factory default), 4-5 = Reserved, 6 = Error, 7 = Null
    */
    gps_mask_t mask;
    unsigned int req_mode;
    unsigned int act_mode;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    mask                             = 0;
    session->driver.nmea2000.sid[1]  = bu[0];

    session->driver.nmea2000.mode_valid |= 1;

    req_mode = (unsigned int)((bu[1] >> 0) & 0x07);
    act_mode = (unsigned int)((bu[1] >> 3) & 0x07);

    /* This is a workaround for some GARMIN plotter, actual mode auto makes no sense for me! */
    if ((act_mode == 3) && (req_mode != 3)) {
        act_mode = req_mode;
    }

    session->driver.nmea2000.mode    = mode_tab[act_mode];

    /*@-type@*//* splint has a bug here */
    session->gpsdata.dop.hdop        = getleu16(bu, 2) * 1e-2;
    session->gpsdata.dop.vdop        = getleu16(bu, 4) * 1e-2;
    session->gpsdata.dop.tdop        = getleu16(bu, 6) * 1e-2;
    /*@+type@*/
    mask                            |= DOP_SET;

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    gpsd_report(session->context->debug, LOG_IO,
		"                   SID:%02x hdop:%5.2f vdop:%5.2f tdop:%5.2f\n",
		session->driver.nmea2000.sid[1],
		session->gpsdata.dop.hdop,
		session->gpsdata.dop.vdop,
		session->gpsdata.dop.tdop);

    (void)strlcpy(session->gpsdata.tag, "129539", sizeof(session->gpsdata.tag));

    return mask | get_mode(session);
}


/*
 *   PGN 129540: GNSS Satellites in View
 */
static gps_mask_t hnd_129540(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{   
  /*
    The GPS100 uses this PGN to provide the GNSS information on current satellites in view
    tagged by sequence ID. Information includes PRN, elevation, azimuth, and SNR. Field 4
    defines the number of satellites. Fields 5 through 11 defines the satellite number and the
    information. Fields 5 through 11 are sequentially repeated for each satellite to be transmitted.
    The factory default for periodic transmission rate is once per second. The transmission of this
    PGN can be disabled (see PGN 126208 – NMEA Request Group Function – Transmission
    Periodic Rate).
    Field 1: SID – The sequence identifier field is used to tie related PGNs together. For
    example, the GPS100 will transmit identical SIDs for 126992 (System Time), 128259
    (Speed), 129026 (COG and SOG, Rapid Update), 129029 (GNSS Position Data),
    129539 (GNSS DOPs), and 129540 (GNSS Satellites in View) to indicate that the
    readings are linked together (i.e., the data from each PGN was taken at the same
    time although they are reported at slightly different times).
    2: Mode – This field always reads as 3 (Null), indicating that range residuals are used
    to calculate position, and not calculated after the position.
    3: Reserved (6 bits) – This field is reserved by NMEA; therefore, this field always
    contains a value of 0x3F (the GPS100 sets all bits to a logic 1)
    4: Number of SVs – This field is used to indicate the number of current satellites in
    view. Fields 5-11 are repeated the number of times specified by this field’s value.
    5: PRN "1" – This field is used to indicate the Satellite ID Number of the satellite (1-
    32=GPS, 33-64=SBAS, 65-96=GLONASS).
    6: Elevation "1" – This field is used to indicate the Elevation of the satellite.
    7: Azimuth "1" – This field is used to indicate the Azimuth of the satellite.
    8: SNR "1" – This field is used to indicate the Signal to Noise Ratio (SNR) of the
    satellite. 
    9: Range Residuals “1” – The GPS100 always sets this field to a value of 0x7FFFFFFF
     (data not available)
    10: PRN Status "1" – This field is used to indicate the status of the first satellite in the
     list. (0=Not Tracked, 1=Tracked but not used in solution, 2=Used in solution without
     Differential corrections, 3=Differential Corrections available, 4=Tracked with
     Differential Corrections, 5=used with Differential Corrections)
    11: Reserved (4 bits) – This field is reserved by NMEA; therefore, this field always
     contains a value of 0xF (the GPS100 sets all bits to a logic 1)
     If Field 4 contains a value greater than one, then the group of fields 5 through 11 is repeated
     until this group appears the number of times indicated by the value of Field 4. 
    */

    int         l1, l2;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    session->driver.nmea2000.sid[2]           = bu[0];
    session->gpsdata.satellites_visible       = (int)bu[2];

    for (l2=0;l2<MAXCHANNELS;l2++) {
        session->gpsdata.used[l2] = 0;
    }
    l2 = 0;
    for (l1=0;l1<session->gpsdata.satellites_visible;l1++) {
        int    svt;
        double azi, elev, snr;

	/*@-type@*//* splint has a bug here */
        elev  = getles16(bu, 3+12*l1+1) * 1e-4 * RAD_2_DEG;
        azi   = getleu16(bu, 3+12*l1+3) * 1e-4 * RAD_2_DEG;
        snr   = getles16(bu, 3+12*l1+5) * 1e-2;
	/*@+type@*/

        svt   = (int)(bu[3+12*l1+11] & 0x0f);

        session->gpsdata.elevation[l1]  = (int) (round(elev));
	session->gpsdata.azimuth[l1]    = (int) (round(azi));
        session->gpsdata.ss[l1]         = snr;
        session->gpsdata.PRN[l1]        = (int)bu[3+12*l1+0];
	if ((svt == 2) || (svt == 5)) {
	    session->gpsdata.used[l2] = session->gpsdata.PRN[l1];
	    l2 += 1;
	}
    }
    session->driver.nmea2000.mode_valid |= 2;
    return  SATELLITE_SET | USED_IS;
}


/*
 *   PGN 129541: GNSS Almanac Data
 */
static gps_mask_t hnd_129541(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf

     The GPS100 uses this PGN to provide a single transmission that contains relevant almanac
     data for the GPS. The almanac contains satellite vehicle course orbital parameters. This
     information is not considered precise and is only valid for several months at a time. GPS100
     receive almanac data directly from the satellites. GPS100 sends this PGN only when
     requested by PGN 059904 (ISO Request).
     Field 1: PRN – PRN of the satellite for which almanac data is being provided.
     2: GPS Week Number – The number of weeks since Jan 6, 1980.
     3: SV Health Bits – Bits 17-24 of each almanac page. Refer to ICD-GPS-200
     paragraph 20.3.3.5.1.3, Table 20-VII and Table 20-VIII.
     4: Eccentricity – Reference ICD-GPS-200 Table 20-VI.
     5: Almanac Reference Time – Reference ICD-GPS-200 Table 20-VI.
     6: Inclination Angle – Reference ICD-GPS-200 Table 20-VI.
     7: Rate of Right Ascension – The OMEGADOT parameter. Reference ICD-GPS-200
     Table 20-VI.
     8: Root of Semi-major Axis – Reference ICD-GPS-200 Table 20-VI.
     9: Argument of Perigee – Reference ICD-GPS-200 Table 20-VI.
     10: Longitude of Ascension Mode – Reference ICD-GPS-200 Table 20-VI.
     11: Mean Anomaly – Reference ICD-GPS-200 Table 20-VI.
     12: Clock Parameter 1 – Reference ICD-GPS-200 Table 20-VI.
     13: Clock Parameter 2 – Reference ICD-GPS-200 Table 20-VI.
     14: Reserved (2 bits) – This field is reserved by NMEA; therefore, this field always
     contains a value of 0x3 (the GPS100 sets all bits to a logic 1) 
    */

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
}

/*
 *   PGN 129029: GNSS Positition Data
 */
static gps_mask_t hnd_129029(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    mask                             = 0;
    session->driver.nmea2000.sid[3]  = bu[0];

    /*@-type@*//* splint has a bug here */
    session->newdata.time            = getleu16(bu,1) * 24*60*60 + getleu32(bu, 3)/1e4;
    /*@+type@*/
    mask                            |= TIME_SET;

    /*@-type@*//* splint has a bug here */
    session->newdata.latitude        = getles64(bu, 7) * 1e-16;  //TODO: 0x7fff ffff ffff ffff
    session->newdata.longitude       = getles64(bu, 15) * 1e-16;
    /*@+type@*/
    mask                            |= LATLON_SET;

    /*@-type@*//* splint has a bug here */
    session->newdata.altitude        = getles64(bu, 23) * 1e-6;
    /*@+type@*/
    mask                            |= ALTITUDE_SET;

//  printf("mode %x %x\n", (bu[31] >> 4) & 0x0f, bu[31]);
    // TODO GPS100 says =no GPS, 1=GNSS fix, 2=DGNSS fix, 6=Estimated (dead reckoning). 
    switch ((bu[31] >> 4) & 0x0f) {
    case 0:
        session->gpsdata.status      = STATUS_NO_FIX;
	break;
    case 1:
        session->gpsdata.status      = STATUS_FIX;
	break;
    case 2:
        session->gpsdata.status      = STATUS_DGPS_FIX;
	break;
    case 3:
    case 4:
    case 5:
        session->gpsdata.status      = STATUS_FIX; /* Is this correct ? */
	break;
    default:
        session->gpsdata.status      = STATUS_NO_FIX;
	break;
    }
    mask                            |= STATUS_SET;

    /*@-type@*//* splint has a bug here */
    session->gpsdata.separation      = getles32(bu, 38) / 100.0;
    /*@+type@*/
    session->newdata.altitude       -= session->gpsdata.separation;

    session->gpsdata.satellites_used = (int)bu[33];

    /*@-type@*//* splint has a bug here */
    session->gpsdata.dop.hdop        = getleu16(bu, 34) * 0.01;
    session->gpsdata.dop.pdop        = getleu16(bu, 36) * 0.01;
    /*@+type@*/
    mask                            |= DOP_SET;

    (void)strlcpy(session->gpsdata.tag, "129029", sizeof(session->gpsdata.tag));

    char times[JSON_DATE_MAX + 1];
    unix_to_iso8601(session->newdata.time, times, sizeof(times));
    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, time = %s, lat = %f, lon = %f, alt = %f\n",
		session->driver.nmea2000.sid[3],
		times, 
		session->newdata.latitude,
		session->newdata.longitude,
		session->newdata.altitude + session->gpsdata.separation);

    gpsd_report(session->context->debug, LOG_IO,
		"                   status = %d, sep = %f, sats = %d, hdop = %f, pdop = %f\n",
		session->gpsdata.status,
		session->gpsdata.separation,
		session->gpsdata.satellites_used,
		session->gpsdata.dop.hdop,
		session->gpsdata.dop.pdop);

    return mask | get_mode(session);
}

/*
 *   PGN 129033 Time & Date [2]
 */
static gps_mask_t hnd_129033(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask;

    uint16_t date = getleu16(bu, 0);
    uint32_t time = getleu32(bu, 2);
    uint32_t offset = getleu16(bu, 6); // TODO: 0x7fff n/a, offset in minutes

    timestamp_t ts;
    
    ts = date * 24*60*60 + time/1e4;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, 
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    char times[JSON_DATE_MAX + 1];
    unix_to_iso8601(ts, times, sizeof(times));
    gpsd_report(session->context->debug, LOG_IO,
		"                   date = %u, time = %u, ts= %s, offset = %u\n",
		date, 
		time, 
		times,
		offset);
    return(0);
}


/*
 *   PGN 129038: AIS  Class A Position Report
 */
static gps_mask_t hnd_129038(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        ais->type1.lon       = (int)          (getles32(bu, 5) * 0.06);
	ais->type1.lat       = (int)          (getles32(bu, 9) * 0.06);
	ais->type1.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
	ais->type1.raim      = (bool)         ((bu[13] >> 1) & 0x01);
	ais->type1.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
	ais->type1.course    = (unsigned int)  ais_direction((unsigned int)getleu16(bu, 14), 10.0);
	ais->type1.speed     = (unsigned int) (getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
	ais->type1.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
	ais->type1.heading   = (unsigned int)  ais_direction((unsigned int)getleu16(bu, 21), 1.0);
	ais->type1.turn      =                 ais_turn_rate((int)getles16(bu, 23));
	ais->type1.status    = (unsigned int) ((bu[25] >> 0) & 0xff);
	ais->type1.maneuver  = 0; /* Not transmitted ???? */
	decode_ais_channel_info(bu, len, 163, session);

	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129039: AIS  Class B Position Report
 */
static gps_mask_t hnd_129039(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        ais->type18.lon      = (int)          (getles32(bu, 5) * 0.06);
	ais->type18.lat      = (int)          (getles32(bu, 9) * 0.06);
	ais->type18.accuracy = (bool)         ((bu[13] >> 0) & 0x01);
	ais->type18.raim     = (bool)         ((bu[13] >> 1) & 0x01);
	ais->type18.second   = (unsigned int) ((bu[13] >> 2) & 0x3f);
	ais->type18.course   = (unsigned int)  ais_direction((unsigned int) getleu16(bu, 14), 10.0);
	ais->type18.speed    = (unsigned int) (getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
	ais->type18.radio    = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
	ais->type18.heading  = (unsigned int)  ais_direction((unsigned int) getleu16(bu, 21), 1.0);    
	ais->type18.reserved = 0;
	ais->type18.regional = (unsigned int) ((bu[24] >> 0) & 0x03);
	ais->type18.cs	     = (bool)         ((bu[24] >> 2) & 0x01);
	ais->type18.display  = (bool)         ((bu[24] >> 3) & 0x01);
	ais->type18.dsc      = (bool)         ((bu[24] >> 4) & 0x01);
	ais->type18.band     = (bool)         ((bu[24] >> 5) & 0x01);
	ais->type18.msg22    = (bool)         ((bu[24] >> 6) & 0x01);
	ais->type18.assigned = (bool)         ((bu[24] >> 7) & 0x01);
	decode_ais_channel_info(bu, len, 163, session);

	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129040: AIS Class B Extended Position Report
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129040(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        uint16_t length, beam, to_bow, to_starboard;
	int l;

        ais->type19.lon          = (int)          (getles32(bu, 5) * 0.06);
	ais->type19.lat          = (int)          (getles32(bu, 9) * 0.06);
	ais->type19.accuracy     = (bool)         ((bu[13] >> 0) & 0x01);
	ais->type19.raim         = (bool)         ((bu[13] >> 1) & 0x01);
	ais->type19.second       = (unsigned int) ((bu[13] >> 2) & 0x3f);
	ais->type19.course       = (unsigned int)  ais_direction((unsigned int) getleu16(bu, 14), 10.0);
	ais->type19.speed        = (unsigned int) (getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
	ais->type19.reserved     = (unsigned int) ((bu[18] >> 0) & 0xff);
	ais->type19.regional     = (unsigned int) ((bu[19] >> 0) & 0x0f);
	ais->type19.shiptype     = (unsigned int) ((bu[20] >> 0) & 0xff);
	ais->type19.heading      = (unsigned int)  ais_direction((unsigned int) getleu16(bu, 21), 1.0);
	length                   =                 getleu16(bu, 24);
	beam                     =                 getleu16(bu, 26);
        to_starboard             =                 getleu16(bu, 28);
        to_bow                   =                 getleu16(bu, 30);
	if ((length == 0xffff) || (to_bow       == 0xffff)) {
	    length       = 0;
	    to_bow       = 0;
	}
	if ((beam   == 0xffff) || (to_starboard == 0xffff)) {
	    beam         = 0;
	    to_starboard = 0;
	}
	ais->type19.to_bow       = (unsigned int) (to_bow/10);
	ais->type19.to_stern     = (unsigned int) ((length-to_bow)/10);
	ais->type19.to_port      = (unsigned int) ((beam-to_starboard)/10);
	ais->type19.to_starboard = (unsigned int) (to_starboard/10);
	ais->type19.epfd         = (unsigned int) ((bu[23] >> 4) & 0x0f);
	ais->type19.dte          = (unsigned int) ((bu[52] >> 0) & 0x01);
	ais->type19.assigned     = (bool)         ((bu[52] >> 1) & 0x01);
	for (l=0;l<AIS_SHIPNAME_MAXLEN;l++) {
	    ais->type19.shipname[l] = (char) bu[32+l];
	}
	ais->type19.shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;
	decode_ais_channel_info(bu, len, 422, session);

	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129794: AIS Class A Static and Voyage Related Data
 */
static gps_mask_t hnd_129794(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        uint16_t  length, beam, to_bow, to_starboard, date;
	int       l;
	uint32_t  time;
	time_t    date1;
        struct tm date2;

        ais->type5.ais_version   = (unsigned int) ((bu[73] >> 0) & 0x03);
	ais->type5.imo           = (unsigned int)  getleu32(bu,  5);
	if (ais->type5.imo == 0xffffffffU) {
	    ais->type5.imo       = 0;
	}
	ais->type5.shiptype      = (unsigned int) ((bu[36] >> 0) & 0xff);
	length                   =                 getleu16(bu, 37);
	beam                     =                 getleu16(bu, 39);
        to_starboard             =                 getleu16(bu, 41);
        to_bow                   =                 getleu16(bu, 43);
	if ((length == 0xffff) || (to_bow       == 0xffff)) {
	    length       = 0;
	    to_bow       = 0;
	}
	if ((beam   == 0xffff) || (to_starboard == 0xffff)) {
	    beam         = 0;
	    to_starboard = 0;
	}
	ais->type5.to_bow        = (unsigned int) (to_bow/10);
	ais->type5.to_stern      = (unsigned int) ((length-to_bow)/10);
	ais->type5.to_port       = (unsigned int) ((beam-to_starboard)/10);
	ais->type5.to_starboard  = (unsigned int) (to_starboard/10);
	ais->type5.epfd          = (unsigned int) ((bu[73] >> 2) & 0x0f);
	date                     =                 getleu16(bu, 45);
	time                     =                 getleu32(bu, 47);
        date1                    = (time_t)       (date*24*60*60);
	(void) gmtime_r(&date1, &date2);
	ais->type5.month         = (unsigned int) (date2.tm_mon+1);
	ais->type5.day           = (unsigned int) (date2.tm_mday);
	ais->type5.minute        = (unsigned int) (time/(10000*60));
	ais->type5.hour          = (unsigned int) (ais->type5.minute/60);
	ais->type5.minute        = (unsigned int) (ais->type5.minute-(ais->type5.hour*60));

	ais->type5.draught       = (unsigned int) (getleu16(bu, 51)/10);
	ais->type5.dte           = (unsigned int) ((bu[73] >> 6) & 0x01);

	for (l=0;l<7;l++) {
	    ais->type5.callsign[l] = (char) bu[9+l];
	}
	ais->type5.callsign[7]   = (char) 0;

	for (l=0;l<AIS_SHIPNAME_MAXLEN;l++) {
	    ais->type5.shipname[l] = (char) bu[16+l];
	}
	ais->type5.shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;

	for (l=0;l<20;l++) {
	    ais->type5.destination[l] = (char) bu[53+l];
	}
	ais->type5.destination[20] = (char) 0;
#if NMEA2000_DEBUG_AIS
	printf("AIS: MMSI:  %09u\n",
	       ais->mmsi);
	printf("AIS: name:  %-20.20s i:%8u c:%-8.8s b:%6u s:%6u p:%6u s:%6u dr:%4.1f\n",
	       ais->type5.shipname,
	       ais->type5.imo,
	       ais->type5.callsign,
	       ais->type5.to_bow,
	       ais->type5.to_stern,
	       ais->type5.to_port,
	       ais->type5.to_starboard,
	       ais->type5.draught/10.0);
	printf("AIS: arival:%-20.20s at %02u-%02u-%04d %02u:%0u\n",
	       ais->type5.destination, 
	       ais->type5.day,
	       ais->type5.month,
	       date2.tm_year+1900,
	       ais->type5.hour,
	       ais->type5.minute);
#endif /* of #if NMEA2000_DEBUG_AIS */
	decode_ais_channel_info(bu, len, 592, session);
        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129798: AIS SAR Aircraft Position Report
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129798(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        ais->type9.lon       = (int)          (getles32(bu, 5) * 0.06);
	ais->type9.lat       = (int)          (getles32(bu, 9) * 0.06);
	ais->type9.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
	ais->type9.raim      = (bool)         ((bu[13] >> 1) & 0x01);
	ais->type9.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
	ais->type9.course    = (unsigned int)  ais_direction((unsigned int) getleu16(bu, 14), 10.0);
	ais->type9.speed     = (unsigned int) (getleu16(bu, 16) * MPS_TO_KNOTS * 0.01 / 0.1);
	ais->type9.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
	ais->type9.alt       = (unsigned int) (getleu64(bu, 21)/1000000);
	ais->type9.regional  = (unsigned int) ((bu[29] >> 0) & 0xff);
	ais->type9.dte	     = (unsigned int) ((bu[30] >> 0) & 0x01);
/*      ais->type9.spare     = (bu[30] >> 1) & 0x7f; */
	ais->type9.assigned  = 0; /* Not transmitted ???? */
	decode_ais_channel_info(bu, len, 163, session);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129802: AIS Safty Related Broadcast Message
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129802(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0x3fffffff) != 0) {
        int                   l;

/*      ais->type14.channel = (bu[ 5] >> 0) & 0x1f; */
	for (l=0;l<36;l++) {
	    ais->type14.text[l] = (char) bu[6+l];
	}
	ais->type14.text[36] = (char) 0;
	decode_ais_channel_info(bu, len, 40, session);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129809: AIS Class B CS Static Data Report, Part A
 */
static gps_mask_t hnd_129809(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
        int                   l;
	int                   index   = session->driver.aivdm.context[0].type24_queue.index;
	struct ais_type24a_t *saveptr = &session->driver.aivdm.context[0].type24_queue.ships[index];

	gpsd_report(session->context->debug, LOG_PROG,
		    "NMEA2000: AIS message 24A from %09u stashed.\n",
		    ais->mmsi);

	for (l=0;l<AIS_SHIPNAME_MAXLEN;l++) {
	    ais->type24.shipname[l] = (char) bu[ 5+l];
	    saveptr->shipname[l] = (char) bu[ 5+l];
	}
	ais->type24.shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;
	saveptr->shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;
	
	saveptr->mmsi = ais->mmsi;

	index += 1;
	index %= MAX_TYPE24_INTERLEAVE;
	session->driver.aivdm.context[0].type24_queue.index = index;

	decode_ais_channel_info(bu, len, 200, session);

	ais->type24.part = part_a;
	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129810: AIS Class B CS Static Data Report, Part B
 */
static gps_mask_t hnd_129810(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (decode_ais_header(session->context, bu, len, ais, 0xffffffffU) != 0) {
      int l, i;

	ais->type24.shiptype = (unsigned int) ((bu[ 5] >> 0) & 0xff);

	for (l=0;l<7;l++) {
	    ais->type24.vendorid[l] = (char) bu[ 6+l];
	}
	ais->type24.vendorid[7] = (char) 0;

	for (l=0;l<7;l++) {
	    ais->type24.callsign[l] = (char) bu[13+l];
	}
	ais->type24.callsign[7] = (char )0;

	if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
	    ais->type24.mothership_mmsi   = (unsigned int) (getleu32(bu, 28));
	} else {
	    uint16_t length, beam, to_bow, to_starboard;

	    length                        =                 getleu16(bu, 20);
	    beam                          =                 getleu16(bu, 22);
	    to_starboard                  =                 getleu16(bu, 24);
	    to_bow                        =                 getleu16(bu, 26);
	    ais->type24.dim.to_bow        = (unsigned int) (to_bow/10);
	    ais->type24.dim.to_stern      = (unsigned int) ((length-to_bow)/10);
	    ais->type24.dim.to_port       = (unsigned int) ((beam-to_starboard)/10);
	    ais->type24.dim.to_starboard  = (unsigned int) (to_starboard/10);
	    if ((length == 0xffff) || (to_bow       == 0xffff)) {
	        length       = 0;
		to_bow       = 0;
	    }
	    if ((beam   == 0xffff) || (to_starboard == 0xffff)) {
	        beam         = 0;
		to_starboard = 0;
	    }
	}

	for (i = 0; i < MAX_TYPE24_INTERLEAVE; i++) {
	    if (session->driver.aivdm.context[0].type24_queue.ships[i].mmsi == ais->mmsi) {
	        for (l=0;l<AIS_SHIPNAME_MAXLEN;l++) {
		    ais->type24.shipname[l] = (char)(session->driver.aivdm.context[0].type24_queue.ships[i].shipname[l]);
		}
		ais->type24.shipname[AIS_SHIPNAME_MAXLEN] = (char) 0;

		gpsd_report(session->context->debug, LOG_PROG,
			    "NMEA2000: AIS 24B from %09u matches a 24A.\n",
			    ais->mmsi);
		/* prevent false match if a 24B is repeated */
		session->driver.aivdm.context[0].type24_queue.ships[i].mmsi = 0;
#if NMEA2000_DEBUG_AIS
		printf("AIS: MMSI:  %09u\n", ais->mmsi);
		printf("AIS: name:  %-20.20s v:%-8.8s c:%-8.8s b:%6u s:%6u p:%6u s:%6u\n",
		       ais->type24.shipname,
		       ais->type24.vendorid,
		       ais->type24.callsign,
		       ais->type24.dim.to_bow,
		       ais->type24.dim.to_stern,
		       ais->type24.dim.to_port,
		       ais->type24.dim.to_starboard);
#endif /* of #if NMEA2000_DEBUG_AIS */

		decode_ais_channel_info(bu, len, 264, session);
		ais->type24.part = both;
		return(ONLINE_SET | AIS_SET);
	    }
	}
#if NMEA2000_DEBUG_AIS
	printf("AIS: MMSI  :  %09u\n", ais->mmsi);
	printf("AIS: vendor:  %-8.8s c:%-8.8s b:%6u s:%6u p:%6u s:%6u\n",
	       ais->type24.vendorid,
	       ais->type24.callsign,
	       ais->type24.dim.to_bow,
	       ais->type24.dim.to_stern,
	       ais->type24.dim.to_port,
	       ais->type24.dim.to_starboard);
#endif /* of #if NMEA2000_DEBUG_AIS */
	decode_ais_channel_info(bu, len, 264, session);
	ais->type24.part = part_b;
	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 127506: PWR DC Detailed Status
 */
static gps_mask_t hnd_127506(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/*
 *   PGN 127508: Battery Status
 */
static gps_mask_t hnd_127508(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /* [17]
    1        Battery Instance
    2 1-2    Battery Voltage
    3        Battery Current
    4        Battery Case Temperature
    5        SID
    */

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127513: PWR Battery Configuration Status
 */
static gps_mask_t hnd_127513(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/*
 *   PGN 127245: NAV Rudder
 */
static gps_mask_t hnd_127245(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
   [2], [5]
   1 Rudder Instance, generated by rudder instance switch, can have values 0 .. 15
   2 Direction Order, 0 if no order provided
   3 Reserved Bits,   1s
   4 Angle Order,     0x7fff is n/a
   5 Position,        signed rudder angle in units of 0.0001 radians
   6 Reserved Bits
   PGN 127245 - Rudder, may be used to report rudder position or it may be used to set a rudder demand. [6]
   Only Position is mapped to RSA [4]
   */
    uint8_t rudder_inst;
    uint8_t dir_order;
    uint8_t reserved1;
    uint8_t angle_order;
    double  position;
    uint8_t reserved2;

    // TODO: these are only wild guesses since we know byte pos of "position"
    rudder_inst  = bu[0] & 0x0f;       // TODO: which bits lower/higher 0 .. 15
    dir_order    = bu[1] & 0xff;       // TODO: guess would be 1 bit?
    reserved1    = bu[1];
    angle_order  = getles16(bu, 2);    // TODO N/A
    position     = getles16(bu, 4) * 0.0001 * RAD_2_DEG; // corresponds to RSA port rudder sensor

    reserved2    = getub(bu, 6);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Rudder Instance= %u, Direction Order= %u, Reserved Bits= %u\n",
		rudder_inst,
		dir_order,
		reserved1);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Angle Order = %u, Position = %f, Reserved Bits= %u\n",
		angle_order,
		position,
		reserved2);

    return(0);
}


/*
 *   PGN 127250: NAV Vessel Heading
 */
static gps_mask_t hnd_127250(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 SID
    2 Heading Sensor Reading
    3 Deviation
    4 Variation
    5 Heading Sensor Reference
    6 Reserved Bits
    */

    uint8_t sid;
    uint16_t hdg;
    int16_t dev;
    int16_t var;
    uint8_t ref; // Heading Sensor Reference (M&T?)
    // [3] says ref = 1 for magnetic north (how many bits?)

    print_data(session->context, bu, len, pgn);

    sid = getub(bu, 0);

    /*@-type@*/
    hdg = getleu16(bu, 1);
    session->gpsdata.attitude.heading = hdg * RAD_2_DEG * 0.0001;

//  printf("ATT 0:%8.3f\n",session->gpsdata.attitude.heading);
    dev = getles16(bu, 3);
    if (dev != 0x07fff) {
        session->gpsdata.attitude.heading += dev * RAD_2_DEG * 0.0001;
    }
//  printf("ATT 1:%8.3f %6x\n",session->gpsdata.attitude.heading, aux);
    var = getles16(bu, 5);
    if (var != 0x07fff) {
        session->gpsdata.attitude.heading += var * RAD_2_DEG * 0.0001;
    }

    // observed b0000 0001, 1 = magnetic (see also [3]), 0 = true
    ref = getub(bu, 7) & 0x01; // ??? len, only a couple of bits

    /*@+type@*/
//  printf("ATT 2:%8.3f %6x\n",session->gpsdata.attitude.heading, aux);
    session->gpsdata.attitude.mag_st = '\0';
    session->gpsdata.attitude.pitch = NAN;
    session->gpsdata.attitude.pitch_st = '\0';
    session->gpsdata.attitude.roll = NAN;
    session->gpsdata.attitude.roll_st = '\0';
    session->gpsdata.attitude.yaw = NAN;
    session->gpsdata.attitude.yaw_st = '\0';
    session->gpsdata.attitude.dip = NAN;
    session->gpsdata.attitude.mag_len = NAN;
    session->gpsdata.attitude.mag_x = NAN;
    session->gpsdata.attitude.mag_y = NAN;
    session->gpsdata.attitude.mag_z = NAN;
    session->gpsdata.attitude.acc_len = NAN;
    session->gpsdata.attitude.acc_x = NAN;
    session->gpsdata.attitude.acc_y = NAN;
    session->gpsdata.attitude.acc_z = NAN;
    session->gpsdata.attitude.gyro_x = NAN;
    session->gpsdata.attitude.gyro_y = NAN;
    session->gpsdata.attitude.temp = NAN;
    session->gpsdata.attitude.depth = NAN;

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    char hdgs[255];
    char devs[255];
    char vars[255];
    sprintf(hdgs, "%f", hdg * RAD_2_DEG * 0.0001);
    sprintf(devs, "%f", dev * RAD_2_DEG * 0.0001);
    sprintf(vars, "%f", var * RAD_2_DEG * 0.0001);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %d, Heading Sensor Reading= %s, Deviation= %s, Variation= %s, Heading Sensor Reference= %s (%u), Reserved = %u\n", 
		sid, 
		hdg != 0x7fff ? hdgs : "-",
		dev != 0x7fff ? devs : "-",
		var != 0x7fff ? vars : "-",
		ref == 1? "Magnetic":"True",
		ref,
		0);

    return(ONLINE_SET | ATTITUDE_SET);
}

/*
 *   PGN 128237: Heading/Track Control 
 */
static gps_mask_t hnd_127237(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf

    1 Rudder Limit Exceeded
    2 Off-Heading Limit Exceeded
    3 Off-Track Limit Exceeded
    4 Override
    5 Steering Mode
    6 Turn Mode
    7 Heading Reference
    8 Reserved Bits
    9 Commanded Rudder Direction
    10 Commanded Rudder Angle
    11 Heading-To-Steer (Course)
    12 Track
    13 Rudder Limit
    14 Off-Heading Limit
    15 Radius of Turn Order
    16 Rate of Turn Order
    17 Off-Track Limit
    18 Vessel Heading
    */

    uint8_t rudder_limit_exceeded;
    uint8_t off_heading_limit_exceeded;
    uint8_t off_track_limit_exceeded;
    uint8_t override;
    uint8_t steering_mode;
    uint8_t turn_mode;
    uint8_t heading_reference;
    uint8_t reserved;

    uint8_t commanded_rudder_direction;
    uint16_t commanded_rudder_angle;

    double heading_to_steer;
    uint16_t track;

    uint16_t rudder_limit;
    uint16_t off_heading_limit;
    uint16_t radius_of_turn_order;
    uint16_t rate_of_turn_order;
    uint16_t off_track_limit;
    uint16_t vessel_heading;


    rudder_limit_exceeded        = bu[0];
    off_heading_limit_exceeded   = bu[0];
    off_track_limit_exceeded     = bu[0];
    override                     = bu[0];
    steering_mode                = bu[0];
    turn_mode                    = bu[0];
    heading_reference            = bu[0];
    reserved                     = bu[0];

    commanded_rudder_direction   = bu[0];
    commanded_rudder_angle       = getleu16(bu, 3);

    heading_to_steer             = getleu16(bu, 5) * 0.0001 * RAD_2_DEG;
    track                        = getleu16(bu, 7);

    rudder_limit                 = getleu16(bu, 9);
    off_heading_limit            = getleu16(bu, 11);
    radius_of_turn_order         = getleu16(bu, 13);
    rate_of_turn_order           = getleu16(bu, 15);
    off_track_limit              = getleu16(bu, 17);
    vessel_heading               = getleu16(bu, 19);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, 
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   Rudder Limit Exceeded= %u, Off-Heading Limit Exceeded = %u, Off-Track Limit Exceeded= %u\n",
		rudder_limit_exceeded,
		off_heading_limit_exceeded,
		off_track_limit_exceeded);
		
    gpsd_report(session->context->debug, LOG_IO, 
		"                   Override= %u, Steering Mode= %u, Turn Mode= %u, Heading Reference= %u\n",
		override,
		steering_mode,
		turn_mode,
		heading_reference);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   Reserved Bits= %u, Commanded Rudder Direction= %u, Commanded Rudder Angle= %u\n",
		reserved,
		commanded_rudder_direction,
		commanded_rudder_angle);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   Heading-To-Steer (Course)= %f, Track= %u, Rudder Limit= %u, Off-Heading Limit= %u\n",
		heading_to_steer,
		track,
		rudder_limit,
		off_heading_limit);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   Radius of Turn Order= %u, Rate of Turn Order= %u, Off-Track Limit= %u, Vessel Heading= %u\n",
		radius_of_turn_order,
		rate_of_turn_order,
		off_track_limit,
		vessel_heading);

    return(0);
}

/*
 *   PGN 128251: Rate of Turn
 */
static gps_mask_t hnd_127251(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    uint8_t sid;
    double rot;
    
    sid = getub(bu, 0);

    // TODO : find explain for odd magic number 3/16 deg/min (or 3/16/60 in deg/sec)
    rot = getles32(bu, 1) * 0.00001 * 3.0/16.0/60.0 * RAD_2_DEG; // per sec, let compiler optimize

    print_data(session->context, bu, len, pgn);

    gpsd_report(session->context->debug, LOG_DATA, 
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   sid = %d, rate = %f deg/s\n", 
		sid, 
		rot);
    return(0);
}


/*
 *   PGN 128259: NAV Speed
 */
static gps_mask_t hnd_128259(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    const double nm = 1852; // nm in meter

    print_data(session->context, bu, len, pgn);

    uint8_t sid = getub(bu, 0);
    uint16_t speed_water = getleu16(bu, 1);   // 1 x 10-2 m/s
    uint16_t speed_ground = getleu16(bu, 3);  // [3] says not available means 0xffff, usually not used
    /*
      http://askjackrabbit.typepad.com/ask_jack_rabbit/page/6/

      The allowable reference values are:

      1) Paddle Wheel
      2) Pitot Tube
      3) Doppler Log
      4) Correlation Log (Ultra-Sound)
      5) EM Log (Electro-Mechanical)
    */

    uint16_t type = getleu16(bu, 5);          // TODO: len in bytes? what does it mean?
    uint8_t reserved = getub(bu, 7);          // TODO: len in bits? all 1 for DST100UM

    gpsd_report(session->context->debug, LOG_DATA, 
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   sid = %d, speed= %f m/s, %f knots, %f km/h, speed ground= %u, type= %u\n", 
		sid, 
		speed_water * 0.01, 
		speed_water * 0.01 * 3600.0/nm, 
		speed_water * 0.01 * 3.6,
		speed_ground,
		type);

    return(0);
}


/*
 *   PGN 128267: NAV Water Depth
 */
static gps_mask_t hnd_128267(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);

    uint8_t sid = getub(bu, 0); // can be tied to 128259
    uint32_t depth = getleu32(bu, 1);  // 1x10-2m
    int16_t offset = getles16(bu, 5); // TODO: 0x7fff N/A, 1x10-3m
    uint8_t res = getub(bu, 7); // reserved, len in bits?

    session->gpsdata.attitude.heading = NAN;
    session->gpsdata.attitude.pitch = NAN;
    session->gpsdata.attitude.pitch_st = '\0';
    session->gpsdata.attitude.roll = NAN;
    session->gpsdata.attitude.roll_st = '\0';
    session->gpsdata.attitude.yaw = NAN;
    session->gpsdata.attitude.yaw_st = '\0';
    session->gpsdata.attitude.dip = NAN;
    session->gpsdata.attitude.mag_len = NAN;
    session->gpsdata.attitude.mag_x = NAN;
    session->gpsdata.attitude.mag_y = NAN;
    session->gpsdata.attitude.mag_z = NAN;
    session->gpsdata.attitude.acc_len = NAN;
    session->gpsdata.attitude.acc_x = NAN;
    session->gpsdata.attitude.acc_y = NAN;
    session->gpsdata.attitude.acc_z = NAN;
    session->gpsdata.attitude.gyro_x = NAN;
    session->gpsdata.attitude.gyro_y = NAN;
    session->gpsdata.attitude.temp = NAN;
    /*@i@*/session->gpsdata.attitude.depth = depth *.01;

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO, 
		"                   SID = %d, depth = %f m, offset = %f m\n",
		sid, 
		depth * 0.01,
		(float)offset * 0.001);

    return(ONLINE_SET | ATTITUDE_SET);
}


/*
 *   PGN 128275: NAV Distance Log
 */
static gps_mask_t hnd_128275(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    [9]: date in Days since January 1, 1970, Date is relative to UTC Time.
         time 24 hour clock, 0 = midnight, time is in UTC, range 0 to 86,401 s
	 distance resolution 1m
   */
    uint16_t date = getleu16(bu, 0); // TODO: 0xffff == not used
    uint32_t time = getleu32(bu, 4); // TODO: 0xffffffff == not used
    uint32_t dist = getleu32(bu, 6); // total cumulative distance in meter 
    uint32_t rest = getleu32(bu, 10); // distance since last reset in meter

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    char d[255]; char t[255];
    if(date == 0xffff)
      sprintf(d, "%u", date);
    else 
      sprintf(d, "-");
    if(time == 0xffffffff) 
      sprintf(t, "%u", time);
    else 
      sprintf(t, "-");

    gpsd_report(session->context->debug, LOG_IO,
		"                   date = %s, time = %s, dist= %um, dist since reset = %um\n",
		d,
		t, 
		dist,
		rest);

    return(0);
}


/*
 *   PGN 129283: NAV Cross Track Error
 */
static gps_mask_t hnd_129283(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 SID
    2 XTE Mode
    3 Reserve
    4 Navigation Terminated
    5 XTE
    6 Reserved
  */

    uint8_t sid;
    uint8_t mode;
    uint8_t reserve;
    uint8_t terminated;
    double  xte;
    uint16_t reserved;

    sid          = getub(bu, 0);
    mode         = getub(bu, 1);           // TODO: bits ? meaning?
    reserve      = getub(bu, 1);           // TODO: bits ?
    terminated   = getub(bu, 1);           // TODO: bits ? 
    xte          = getles32(bu, 2) * 0.01; // TODO: signed / unsigned, seen both
    reserved     = getleu16(bu, 7);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, XTE Mode = %u, reserve = %d, \n",
		sid,
		mode,
		reserve);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Navigation Terminated= %d, XTE= %fm, res = %u\n",
		terminated,
		xte,
		reserved);

    return(0);
}


/*
 *   PGN 129284: NAV Navigation Data
 */
static gps_mask_t hnd_129284(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
     1   0             SID
     2 1-4             Distance to Destination Waypoint
     3   5   bits 0-1  Course/Bearing Ref.
     4   5   bits 2-3  Perpendicular Crossed
     5   5   bits 4-5  Arrival Circle Entered
     4   5   bits 2-3  Perpendicular Crossed
     5   5   bits 4-5  Arrival Circle Entered
     6   5   bits 6-7  Calculation Type
     7 6-9             ETA Time
     8 10-11           ETA Date
     9 12-13           Bearing, Origin To Destination Waypoint
    10 14-15           Bearing, Position To Destination Waypoint
    11 16-19           Origin Waypoint Number
    12 20-23           Destination Waypoint Number
    13 24-27           Destination Wpt Latitude
    14 28-31           Destination Wpt Longitude
    15 32-33           Waypoint Closing Velocity
  */
   
    uint8_t sid;
    double dtwp;
    uint8_t course_bearing_ref;
    uint8_t perpendicular_crossed;
    uint8_t arrival_circle_entered;
    uint8_t calc_type;
    uint32_t eta_time;
    uint16_t eta_date;
    timestamp_t eta;
    double brg_org2wpt;
    double brg_pos2wpt;
    uint32_t org_wpt_number;
    uint32_t dest_wpt_number;
    double dest_wpt_lat;
    double dest_wpt_lon;
    double wpt_closing_velocity;

    char etas[JSON_DATE_MAX + 1];
    char dtwps[256];

    sid                     = bu[0];
    dtwp                    = getleu32(bu, 1) * 0.01; // TODO: 0xffffffff for n/a
    if(dtwp == 0xffffffff) {
      strcpy(dtwps, "-");
    } else {
      sprintf(dtwps, "%f", dtwp);
    }

    // lets assume 2 bit pattern based on bits toggled 
    // guess work here
    course_bearing_ref      = (bu[5] >> 0) & 0x03; // likely M/T

    // observed b0000 0100
    perpendicular_crossed   = (bu[5] >> 2) & 0x03;      

    // observed b00010 000                 
    arrival_circle_entered  = (bu[5] >> 4) & 0x03;      

    // only a guess
    calc_type               = (bu[5] >> 6) & 0x03;

    // haven't seen this, bytes are a guess/match to other time/date fields
    eta_time                = getleu32(bu,  6);
    eta_date                = getleu16(bu, 10);

    if(eta_date == 0xffff && eta_time == 0xffffffff) {
      strcpy(etas, "-");
    } else {
      if(eta_date == 0xffff) {
	eta_date = 0;
      }
      if(eta_time == 0xffffffff) {
	eta_time = 0;
      }
      eta                     = eta_date*24*60*60 + eta_time/1e4;
      unix_to_iso8601(eta, etas, sizeof(etas));
    }

    // seems 0xffff for not set
    brg_org2wpt             = getleu16(bu, 12) * 0.0001 * RAD_2_DEG; // connected to 127237
    brg_pos2wpt             = getleu16(bu, 14) * 0.0001 * RAD_2_DEG; // connected to 127237

    org_wpt_number          = getleu32(bu, 16);        // TODO: 0xffffff for n/a
    dest_wpt_number         = getleu32(bu, 20);

    dest_wpt_lat            = getles32(bu, 24) * 1e-7; // TODO: 0x7fffffff for n/a
    dest_wpt_lon            = getles32(bu, 28) * 1e-7; // TODO: 0x7fffffff for n/a

    wpt_closing_velocity    = getleu16(bu, 32) * 0.01; // TODO: 0x7fff for n/a
    
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, Dist to Dest Wpt= %sm, Course/Bearing Ref.= %u\n",
		sid,
		dtwps,
		course_bearing_ref);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Perpendicular Crossed= %u, Arrival Circle Entered= %u, Calculation Type= %u\n",
		perpendicular_crossed,
		arrival_circle_entered,
		calc_type);

    gpsd_report(session->context->debug, LOG_IO,
		"                   ETA= %s\n", etas);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Bearing, Org To Dest Wpt= %f, Bearing, Pos To Dest Wpt= %f\n",
		brg_org2wpt,
		brg_pos2wpt);

    gpsd_report(session->context->debug, LOG_IO,
		"                   Org Wpt #= %u, Dest Wpt #= %u, Dest Wpt Lat= %f, Dest Wpt Lon= %f, Wpt Closing Velocity= %fm/s\n",
		org_wpt_number,
		dest_wpt_number,
		dest_wpt_lat,
		dest_wpt_lon,
		wpt_closing_velocity);

    return(0);
}


/*
 *   PGN 129285: NAV Navigation - Route/WP Information
 */
static gps_mask_t hnd_129285(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 Start RPS#
    2 nItems
    3 Database ID
    4 Route ID
    5 Navigation direction in route
    6 Supplementary Route/WP data available
    7 Reserved bits
    8 Route Name
    9 Reserved
    10 WPID
    11 WP Name
    12 WP Latitude
    13 WP Longitude
    14 Fields 10 thru 13 repeat as needed
    */
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 129291: Set & Drift, Rapid Update
 */
static gps_mask_t hnd_129291(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 SID
    2 Set Reference
    3 Reserved Bits
    4 Set
    5 Drift
    6 Reserved Bits1SID
    2 Set Reference
    3 Reserved Bits
    4 Set
    5 Drift
    6 Reserved Bits
    */
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 130306: NAV Wind Data
 *   http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/
 */
static gps_mask_t hnd_130306(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 Sequence ID
    2 Wind Speed
    3 Wind Direction
    4 Wind Reference
    5 Reserv

    0) Theoretical Wind (ground referenced, referenced to True North; calculated using COG/SOG)
    1) Theoretical Wind (ground referenced, referenced to Magnetic North; calculated using COG/SOG)
    2) Apparent Wind (relative to the vessel centerline)
    3) Theoretical (Calculated to Centerline of the vessel, referenced to ground; calculated using COG/SOG)
    4) Theoretical (Calculated to Centerline of the vessel, referenced to water; calculated using Heading/Speed through Water)

  */

    static char * wind_ref[] = {
      "True North",
      "Magnetic North",
      "Apparent",
      "True Vessel",
      "Water Vessel"
    };

    uint8_t sid;
    double speed;
    double dir;
    uint8_t ref;
    uint8_t res;

    sid          = getub(bu, 0);
    speed        = getleu16(bu, 1) * 0.01;
    dir          = getleu16(bu, 3) * RAD_2_DEG * 0.0001; // 0.0001 rads per second ([10])
    ref          = getub(bu, 5) & 0x07;         // see res 24 - 21 bits
    res          = getub(bu, 6);                // 21 bits ([19]), makes ref == 3 bits

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, wind speed = %f m/s, dir = %f, ref = %s, res = %u\n", 
		sid,
		speed,
		dir,
		(ref < 5) ? wind_ref[ref] : "-",
		res);
    return(0);
}


/*
 *   PGN 130310: NAV Water Temp., Outside Air Temp., Atmospheric Pressure
 */
static gps_mask_t hnd_130310(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 Sequence ID
    2 Water Temp
    3 Outside Ambient Air Temp.
    4 Atmospheric Pressure
    5 Reserved Bits
  */

    uint8_t sid;
    int16_t water;
    int16_t air;
    uint16_t pres;
    uint8_t res;

    sid          = getub(bu, 0);
    water        = getles16(bu, 1);
    air          = getles16(bu, 3);
    pres         = getleu16(bu, 5);
    res          = getsb(bu, 7);

    #define PSI_2_BAR 1.0/14.50377

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, water = %f C, outside = %f C, pressure = %f mbar, res = %u\n", 
		sid,
		water * 0.01 - 273.15,
		air * 0.01 - 273.15,
		pres * PSI_2_BAR * 0.001, 
		res);
    return(0);
}


/*
 *   PGN 130311: NAV Environmental Parameters
 */
static gps_mask_t hnd_130311(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*[2], [10]
    rework of 130310 and should be used for new designs
    1 Sequence ID
    2 Temperature Instance - 0x01 == outside temperature
    3 Humidity Instance    - WSO100: 0x01 == outside humidity. 
    4 Temperature          - outside air units of 0.01°K. 
    5 Humidity             – relative humidity in units of 0.004%
    6 Atmospheric Pressure - units of 100Pa
    
  */
    uint8_t sid;
    uint8_t temp_inst;
    uint8_t hum_inst;
    uint16_t temp;
    int16_t hum;  
    int16_t pres;

    sid = getub(bu, 0);
    temp_inst     = getub(bu, 1);     // TODO: which instances
    hum_inst      = getub(bu, 1);
    temp          = getleu16(bu, 2);
    hum           = getleu16(bu, 4);  // TODO: s/u? 0x7fff for n/a?
    pres          = getleu16(bu, 6);  // TODO: s/u? 0xffff for n/a?

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, temp inst= %d, hum inst= %d, temp = %f C, hum = %f%%, pres = %f\n", 
		sid,
		temp_inst,
		hum_inst,
		temp * 0.01 - 273.15,
		hum * 0.004,
		pres * 0.0001);

    return(0);
}

/*
 *   PGN 130312: NAV Temperature
 *   deprecated in NMEA 3.0
 */
static gps_mask_t hnd_130312(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
  /*
    1 SID
    2 Temperature Instance
    3 Temperature Source
    4 Actual Temperature
    5 Set Temperature
    6 Reserve

    [12] about temp instances (?)
     0 Water
     1 Air outside
     2 Air inside
     3 Engine room
     4 Main cabin
     5 Live well
     6 Bait well
     7 Refrigeration
     8 Heating system
    13 Freezer
    
    [12] somehow suggests that instance no is user selectable in devices like GST 10
    */

    uint8_t sid;
    uint8_t temp_inst;
    uint8_t temp_src;
    int16_t temp;
    int16_t temp_set;
    uint8_t reserve;

    static char * temp_src_list[] = {
      "Water",
      "Air"};

    sid           = getub(bu, 0);
    temp_inst     = getub(bu, 1);
    temp_src      = getub(bu, 2);
    temp          = getles16(bu, 3);
    temp_set      = getles16(bu, 5);  // TODO: 0xffff for n/a?
    reserve       = getub(bu, 7);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, temp inst = %u, temp src = %s, temp = %f, temp set = %f, res = %u\n", 
		sid,
		temp_inst,
		(temp_src < 2) ? temp_src_list[temp_src] : " not listed yet ",
		temp * 0.01 - 273.15,
		temp_set * 0.01 - 273.15,
		reserve);

    return(0);
}

/*@-usereleased@*/
static const char msg_059392[] = {"ISO Acknowledgment"};
static const char msg_059904[] = {"ISO Request"};
static const char msg_060928[] = {"ISO  Address Claim"};
static const char msg_126208[] = {"NMEA Command/Request/Acknowledge"};
static const char msg_126464[] = {"ISO  Transmit/Receive PGN List"};
static const char msg_126992[] = {"GNSS System Time"};
static const char msg_126996[] = {"ISO  Product Information"};

static const char msg_127506[] = {"PWR DC Detailed Status"};
static const char msg_127508[] = {"PWR Battery Status"};
static const char msg_127513[] = {"PWR Battery Configuration Status"};

static const char msg_127258[] = {"GNSS Magnetic Variation"};
static const char msg_129025[] = {"GNSS Position Rapid Update"};
static const char msg_129026[] = {"GNSS COG and SOG Rapid Update"};
static const char msg_129029[] = {"GNSS Positition Data"};
static const char msg_129539[] = {"GNSS DOPs"};
static const char msg_129540[] = {"GNSS Satellites in View"};
static const char msg_129541[] = {"GNSS Almanac Data"};

static const char msg_129038[] = {"AIS  Class A Position Report"};
static const char msg_129039[] = {"AIS  Class B Position Report"};
static const char msg_129040[] = {"AIS  Class B Extended Position Report"};
static const char msg_129794[] = {"AIS  Class A Static and Voyage Related Data"};
static const char msg_129798[] = {"AIS  SAR Aircraft Position Report"};
static const char msg_129802[] = {"AIS  Safty Related Broadcast Message"};
static const char msg_129809[] = {"AIS  Class B CS Static Data Report, Part A"};
static const char msg_129810[] = {"AIS  Class B CS Static Data Report, Part B"};

static const char msg_127237[] = {"Heading/Track Control"};
static const char msg_127245[] = {"NAV Rudder"};
static const char msg_127250[] = {"NAV Vessel Heading"};
static const char msg_127251[] = {"Rate of Turn"};
static const char msg_127257[] = {"Attitude"};
static const char msg_128259[] = {"NAV Speed"};
static const char msg_128267[] = {"NAV Water Depth"};
static const char msg_128275[] = {"NAV Distance Log"};

static const char msg_129033[] = {"Time & Date"};

static const char msg_129283[] = {"NAV Cross Track Error"};
static const char msg_129284[] = {"NAV Navigation Data"};
static const char msg_129285[] = {"NAV Navigation - Route/WP Information"};
static const char msg_129291[] = {"NAV Set & Drift, Rapid Update"};

static const char msg_130306[] = {"NAV Wind Data"};
static const char msg_130310[] = {"NAV Water Temp., Outside Air Temp., Atmospheric Pressure"};
static const char msg_130311[] = {"NAV Temperature"};
static const char msg_130312[] = {"NAV Temperature"};

static const char msg_error [] = {"**error**"};

static PGN gpspgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},

		       {127258, 0, 0, hnd_127258, &msg_127258[0]},

		       {129025, 0, 1, hnd_129025, &msg_129025[0]},
		       {129026, 0, 1, hnd_129026, &msg_129026[0]},
		       {129029, 1, 1, hnd_129029, &msg_129029[0]},

		       {129283, 0, 0, hnd_129283, &msg_129283[0]},
		       {129284, 1, 0, hnd_129284, &msg_129284[0]},
		       {129285, 1, 0, hnd_129285, &msg_129285[0]},
		       {129539, 0, 1, hnd_129539, &msg_129539[0]},
		       {129540, 1, 1, hnd_129540, &msg_129540[0]},
		       {129541, 1, 1, hnd_129541, &msg_129541[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};

static PGN aispgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},
		       {129038, 1, 2, hnd_129038, &msg_129038[0]},
		       {129039, 1, 2, hnd_129039, &msg_129039[0]},
		       {129040, 1, 2, hnd_129040, &msg_129040[0]},
		       {129794, 1, 2, hnd_129794, &msg_129794[0]},
		       {129798, 1, 2, hnd_129798, &msg_129798[0]},
		       {129802, 1, 2, hnd_129802, &msg_129802[0]},
		       {129809, 1, 2, hnd_129809, &msg_129809[0]},
		       {129810, 1, 2, hnd_129810, &msg_129810[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};

static PGN pwrpgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},
		       {127506, 1, 3, hnd_127506, &msg_127506[0]},
		       {127508, 1, 3, hnd_127508, &msg_127508[0]},
		       {127513, 1, 3, hnd_127513, &msg_127513[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};

static PGN navpgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 59904, 0, 0, hnd_059904, &msg_059904[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},

		       {127237, 0, 0, hnd_127237, &msg_127237[0]},
		       {127251, 0, 0, hnd_127251, &msg_127251[0]},

		       {127245, 0, 4, hnd_127245, &msg_127245[0]},
		       {127250, 0, 4, hnd_127250, &msg_127250[0]},
		       {127257, 0, 0, hnd_127257, &msg_127257[0]},
		       {127258, 0, 0, hnd_127258, &msg_127258[0]},

		       {128259, 0, 4, hnd_128259, &msg_128259[0]},
		       {128267, 0, 4, hnd_128267, &msg_128267[0]},
		       {128275, 1, 4, hnd_128275, &msg_128275[0]},

		       {129033, 1, 1, hnd_129033, &msg_129033[0]},

		       {129283, 0, 0, hnd_129283, &msg_129283[0]},
		       {129284, 1, 0, hnd_129284, &msg_129284[0]},
		       {129285, 1, 0, hnd_129285, &msg_129285[0]},

		       {129291, 0, 0, hnd_129291, &msg_129291[0]},

		       {130306, 0, 4, hnd_130306, &msg_130306[0]},
		       {130310, 0, 4, hnd_130310, &msg_130310[0]},
		       {130311, 0, 4, hnd_130311, &msg_130311[0]},
		       {130312, 0, 4, hnd_130312, &msg_130312[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};


/*@+usereleased@*/

/*@-immediatetrans@*/
static /*@null@*/ PGN *vyspi_search_pgnlist(unsigned int pgn, PGN *pgnlist)
{
    int l1;
    PGN *work = NULL;

    l1 = 0;
    while (pgnlist[l1].pgn != 0) {
        if (pgnlist[l1].pgn == pgn) {
	    work = &pgnlist[l1];
	    break;
	} else {
	    l1 = l1 + 1;
	    }
	}
    return work;
}
/*@+immediatetrans@*/

static void vyspi_report_packet(struct gps_packet_t *pkg) {

  unsigned int ret = 0;

  while(ret < pkg->inbuflen)  {
    int i = 0;
    for (i = 0; i < 8; i++) {
      if(ret + i < pkg->inbuflen)
	printf("0x%02X ", (uint8_t)pkg->inbuffer[ret + i]);
      else
	printf("0x00 ");
    }
    printf(" : ");
    for (i = 0; i < 8; i++) {
      if(ret + i < pkg->inbuflen)
	printf("%c", 
	       pkg->inbuffer[ret + i] < 127 
	       && pkg->inbuffer[ret + i] > 31 ? pkg->inbuffer[ret + i] : '.');
      else
	printf(".");
    }
    ret += 8;
    printf("\n");
  }
}

static PGN *vyspi_find_pgn(uint32_t pgn) {

      PGN *pgnlist;
      PGN *work = NULL;

      pgnlist = &gpspgn[0];
      work = vyspi_search_pgnlist(pgn, pgnlist);
      if (work == NULL) {
	pgnlist = &aispgn[0];
	work = vyspi_search_pgnlist(pgn, pgnlist);
      }
      if (work == NULL) {
	pgnlist = &pwrpgn[0];
	work = vyspi_search_pgnlist(pgn, pgnlist);
      }
      if (work == NULL) {
	pgnlist = &navpgn[0];
	work = vyspi_search_pgnlist(pgn, pgnlist);
      }

      return work;
}

static void vyspi_packet_discard(struct gps_packet_t *lexer)
/* shift the input buffer to discard all data up to current input pointer */
{
    size_t discard = lexer->inbufptr - lexer->inbuffer;
    size_t remaining = lexer->inbuflen - discard;
    lexer->inbufptr = memmove(lexer->inbuffer, lexer->inbufptr, remaining);
    lexer->inbuflen = remaining;
    if (lexer->debug >= LOG_RAW+1) {
	char scratchbuf[MAX_PACKET_LENGTH*2+1];
	gpsd_report(lexer->debug, LOG_RAW + 1,
		    "Packet discard of %zu, chars remaining is %zu\n",
		    discard, remaining);
    }
}

static void vyspi_packet_accept(struct gps_packet_t *lexer, int packet_type)
/* packet grab succeeded, move to output buffer */
{
    size_t packetlen = lexer->inbufptr - lexer->inbuffer;

    if (packetlen < sizeof(lexer->outbuffer)) {
	memcpy(lexer->outbuffer, lexer->inbuffer, packetlen);
	lexer->outbuflen = packetlen;
	lexer->outbuffer[packetlen] = '\0';
	lexer->type = packet_type;
	if (lexer->debug >= LOG_RAW+1) {
	    char scratchbuf[MAX_PACKET_LENGTH*2+1];
	    gpsd_report(lexer->debug, LOG_RAW+1,
			"Packet type %d accepted %zu = %s\n",
			packet_type, packetlen,
			gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
					(char *)lexer->outbuffer,
					lexer->outbuflen));
	}
    } else {
	gpsd_report(lexer->debug, LOG_ERROR,
		    "Rejected too long packet type %d len %zu\n",
		    packet_type, packetlen);
    }
}

static size_t vyspi_packetlen( struct gps_packet_t *lexer ) {
  return lexer->inbufptr - lexer->inbuffer + lexer->inbuflen;
}

static void vyspi_preparse(struct gps_device_t *session) {

  static char * typeNames [] = {
    "UNKOWN", "NMEA0183", "NMEA2000"
  };

  struct gps_packet_t *lexer = &session->packet;

  size_t packetlen = vyspi_packetlen(lexer);

  gpsd_report(session->context->debug, LOG_DATA, 
	      "VYSPI: preparse called with packet len = %u\n", packetlen);

  // one extra for reading both, len and type/origin
  while(packet_buffered_input(lexer)) {

    uint8_t b = *lexer->inbufptr++;
    uint8_t pkgType =  b & 0x0F;
    uint8_t pkgOrg  =  (b >> 5) & 0x0F;

    b = *lexer->inbufptr++;
    uint8_t pkgLen  =  b & 0xFF;

    gpsd_report(session->context->debug, LOG_DATA, "VYSPI: ptype= %s, org= %d, len= %d\n", 
		((pkgType > PKG_TYPE_NMEA2000) && (pkgType < PKG_TYPE_NMEA0183)) 
		? typeNames[0] : typeNames[pkgType], 
		pkgOrg, pkgLen);

    if((lexer->inbuffer + lexer->inbuflen < lexer->inbufptr) || (pkgLen <= 0)) {
      // discard 
      gpsd_report(session->context->debug, LOG_WARN, "VYSPI: input too short\n");
      lexer->inbufptr = lexer->inbuffer + lexer->inbuflen;
      vyspi_packet_discard(lexer);
      break;
    }

    if(pkgType == PKG_TYPE_NMEA2000) {

      if(lexer->inbuflen < lexer->inbufptr - lexer->inbuffer + pkgLen + 8) {
	gpsd_report(session->context->debug, LOG_WARN, "VYSPI: exit prematurely: %d + 8 + %d > %d\n",
		    (lexer->inbufptr - lexer->inbuffer), pkgLen, packetlen);
	// discard 
	lexer->inbufptr = lexer->inbuffer + lexer->inbuflen;
	vyspi_packet_discard(lexer);
	break;
      }

      session->driver.vyspi.last_pgn = getleu32(lexer->inbufptr, 0);
      lexer->inbufptr += 4;

      uint32_t pkgid = getleu32(lexer->inbufptr, 0);
      lexer->inbufptr += 4;

      gpsd_report(session->context->debug, LOG_DATA, 
		  "VYSPI: PGN = %u, pid= %u, org= %u, len= %u\n", 
		  session->driver.vyspi.last_pgn, pkgid, pkgOrg, pkgLen);

      memcpy(lexer->outbuffer, lexer->inbufptr, pkgLen);
      lexer->outbuflen = pkgLen;
      lexer->outbuffer[pkgLen] = '\0';
      lexer->type = NMEA2000_PACKET;

      lexer->inbufptr += pkgLen;

      PGN *work;
      work = vyspi_find_pgn(session->driver.vyspi.last_pgn);

      session->driver.nmea2000.workpgn = (void *) work;

      vyspi_packet_discard(lexer);

      break;

    } else if (pkgType == PKG_TYPE_NMEA0183) {
      
      if(lexer->inbuflen < lexer->inbufptr - lexer->inbuffer + pkgLen) {
	gpsd_report(session->context->debug, LOG_WARN, "VYSPI: exit prematurely: %d + %d > %d\n",
		    (lexer->inbufptr - lexer->inbuffer), pkgLen, packetlen);
	// discard 
	lexer->inbufptr = lexer->inbuffer + lexer->inbuflen;
	break;
      }

      gpsd_report(session->context->debug, LOG_DATA, "VYSPI: org= %d, len= %d\n", 
		  pkgOrg, pkgLen);


      memcpy(lexer->outbuffer, lexer->inbuffer, pkgLen);
      lexer->outbuflen = pkgLen;
      lexer->outbuffer[pkgLen] = '\0';
      lexer->type = NMEA_PACKET;

      // length is packet length
      lexer->inbufptr += pkgLen;

      vyspi_packet_discard(lexer);

      break;

    } else {

      gpsd_report(session->context->debug, LOG_ERROR, "UNKOWN: len= %d\n", 
		  pkgLen);	  

      // discard 
      lexer->inbufptr = lexer->inbuffer + lexer->inbuflen;

      vyspi_packet_discard(lexer);

      break;
    }
  }

}

static ssize_t vyspi_get(struct gps_device_t *session)
{   
  int fd = session->gpsdata.gps_fd;
  struct gps_packet_t * pkg = &session->packet;

  ssize_t          status;

  errno = 0;
  // we still need to process old package before getching a new)
  if(!packet_buffered_input(pkg)) {

    gpsd_report(session->context->debug, LOG_IO, "VYSPI reading from device\n");

    status = read(fd, pkg->inbuffer + pkg->inbuflen,
		sizeof(pkg->inbuffer) - (pkg->inbuflen));

    pkg->outbuflen = 0;

    if(status == -1) {
      if ((errno == EAGAIN) || (errno == EINTR)) {
	gpsd_report(session->context->debug, LOG_IO, "no bytes ready\n");
	status = 0;
	/* fall through, input buffer may be nonempty */
      } else {
	gpsd_report(session->context->debug, LOG_ERROR, 
		    "errno: %s\n", strerror(errno));
	return -1;
      }
    }

    if(status <= 0) {
      gpsd_report(session->context->debug, LOG_DATA, 
		  "VYSPI: exit with len in bytes= %d, errno= %d\n", 
		  status, errno);
      return 0;
    }

    pkg->inbuflen = status;
    pkg->inbufptr = pkg->inbuffer;

  }

  vyspi_preparse(session);


  if (pkg->outbuflen > 0) {
    if ((session->driver.nmea2000.workpgn == NULL) 
	&& (session->packet.type == NMEA2000_PACKET)) {
      return 0;
    }

    return (ssize_t)pkg->outbuflen;
  } else {
    /*
     * Otherwise recvd is the size of whatever packet fragment we got.
     * It can still be 0 or -1 at this point even if buffer data
     * was consumed.
     */
    return status;
  }

    //  vyspi_report_packet(pkg);

    /* Consume packet from the input buffer 
       - no lex parsing here to detect packet borders. 
       The SPI driver only sends one whole 
       packet at a time anyways. */

  /*
    memcpy(session->packet.outbuffer, pkg->inbuffer, len);
    packet_reset(pkg);

    session->packet.outbuflen = len;
    session->packet.type = VYSPI_PACKET;

    gpsd_report(session->context->debug, LOG_DATA, 
		"VYSPI: len = %d, bytes= %d, errno= %d\n", 
		len, status, errno);
  }

  return len;
  */
}

/*@-mustfreeonly@*/
static gps_mask_t vyspi_parse_input(struct gps_device_t *session)
{    
  gps_mask_t mask = 0;
  struct gps_packet_t * pkg = &session->packet;

  PGN *work = NULL;

  static char * typeNames [] = {
    "UNKOWN", "NMEA0183", "NMEA2000"
  };

  uint8_t packet_len = pkg->outbuflen;
  uint8_t * buf = pkg->outbuffer;

  uint8_t len = 0;

  gpsd_report(session->context->debug, LOG_ERROR, 
	      "VYSPI: parse_input called with packet len = %d\n", packet_len);

  // one extra for reading both, len and type/origin
  while(len + 1 < packet_len) {

    uint8_t pkgType = (uint8_t)buf[len] & 0x0F;
    uint8_t pkgOrg  = (uint8_t)((buf[len] & 0xF0) >> 5);
    uint8_t pkgLen =  (uint8_t)buf[len + 1] & 0xFF;

    gpsd_report(session->context->debug, LOG_DATA, "VYSPI: ptype= %s, org= %d, len= %d\n", 
		((pkgType > PKG_TYPE_NMEA2000) && (pkgType < PKG_TYPE_NMEA0183)) 
		? typeNames[0] : typeNames[pkgType], 
		pkgOrg, pkgLen);

    // skip 2 byte header now
    len += 2;
 
    if((pkgLen <= 0) || (packet_len <= len)) break;

    if(pkgType == PKG_TYPE_NMEA2000) {

      if(len + pkgLen + 8 > packet_len) {
	gpsd_report(session->context->debug, LOG_WARN, "VYSPI: exit prematurely: %d + 8 + %d > %d\n",
		    len, pkgLen, packet_len);
	break;
      }

      session->driver.vyspi.last_pgn = getleu32(buf, len);
      uint32_t pkgid = getleu32(buf, len + 4);

      gpsd_report(session->context->debug, LOG_DATA, 
		  "VYSPI: PGN = %u, pid= %u, org= %u, len= %u\n", 
		   session->driver.vyspi.last_pgn, pkgid, pkgOrg, pkgLen);

      work = vyspi_find_pgn( session->driver.vyspi.last_pgn );

      len += 8;

      if (work != NULL) {
	mask |= (work->func)(&session->packet.outbuffer[len], 
			     (int)pkgLen, work, session);
      } else {
	gpsd_report(session->context->debug, LOG_ERROR, 
		    "VYSPI: no work PGN found for pgn = %u\n", 
		    session->driver.vyspi.last_pgn);
      }

      // length is packet length + 8 bytes pgn/pid
      len += pkgLen;

    } else if (pkgType == PKG_TYPE_NMEA0183) {
      
      if(len + pkgLen >= packet_len) 
	break;

      gps_mask_t st = 0;

      gpsd_report(session->context->debug, LOG_DATA, "VYSPI: org= %d, len= %d\n", 
		  pkgOrg, pkgLen);

      char sentence[NMEA_MAX + 1];

      memset(sentence, 0, sizeof(sentence));
      memcpy(sentence, (char *)&session->packet.outbuffer[len], pkgLen);

      if (sentence[strlen(sentence)-1] != '\n')
	gpsd_report(session->context->debug, LOG_IO, "<= GPS: %s\n", sentence);
      else
	gpsd_report(session->context->debug, LOG_IO, "<= GPS: %s", sentence);

      if ((st= nmea_parse(sentence, session)) == 0) {
	gpsd_report(session->context->debug, LOG_WARN, "unknown sentence: \"%s\"\n",	sentence);
      } 

      mask |= st;
	
      // length is packet length
      len += pkgLen;

    } else {

      gpsd_report(session->context->debug, LOG_ERROR, "UNKOWN: len= %d\n", 
		  pkgLen);	  

      break;
    }
  }

  //    session->packet.outbuflen = 0;

    return mask;
}
/*@+mustfreeonly@*/

/*@+nullassign@*/

#ifndef S_SPLINT_S
int vyspi_open(struct gps_device_t *session) {

  char path[strlen(session->gpsdata.dev.path)], *port;
  socket_t dsock;
  (void)strlcpy(path, session->gpsdata.dev.path + 8, sizeof(path));
	
  session->gpsdata.gps_fd = -1;
  port = strchr(path, ':');

  gpsd_report(session->context->debug, LOG_INF, "Device path = %s.\n", path);

  if (port == NULL) {

    if(path[0] == '/')
      gpsd_report(session->context->debug, LOG_INF, "Assuming SPI device %s.\n", path);

    dsock = open(path, O_RDWR, 0);

    gpsd_report(session->context->debug, LOG_INF, "SPI device %s opened with sock = %d.\n", path, dsock);

    if(dsock) {
      // usefull to send a reset to the device
      ioctl(dsock, VYSPI_RESET, NULL);
    }

  } else {

    *port++ = '\0';
    gpsd_report(session->context->debug, LOG_INF, "opening TCP VYSPI feed at %s, port %s.\n", path,
		port);
    if ((dsock = netlib_connectsock(AF_UNSPEC, path, port, "tcp")) < 0) {
      gpsd_report(session->context->debug, LOG_ERROR, "TCP device open error %s.\n",
		  netlib_errstr(dsock));
      return -1;
    } else
      gpsd_report(session->context->debug, LOG_SPIN, "TCP device opened on fd %d\n", dsock);

  }

  gpsd_switch_driver(session, "VYSPI");
  session->gpsdata.gps_fd = dsock;
  session->sourcetype = source_can;
  session->servicetype = service_sensor;

  return session->gpsdata.gps_fd;
}

#endif /* of ifndef S_SPLINT_S */

/*@-mustdefine@*/
const char /*@ observer @*/ *gpsd_vyspidump(struct gps_device_t *device) {

  char *scbuf = device->msgbuf;
  size_t scbuflen = sizeof(device->msgbuf);

  char *binbuf = (char *)device->packet.outbuffer;
  size_t binbuflen = device->packet.outbuflen;

  printf("VYSPI: gpsd_vyspidump %u entered with len = %d\n", 
	 device->driver.vyspi.last_pgn, binbuflen);

  size_t j = 0;
  size_t i = 0;

  size_t maxlen =
    (size_t) ((binbuflen >
	       MAX_PACKET_LENGTH) ? MAX_PACKET_LENGTH : binbuflen);

  printf("VYSPI: gpsd_vyspidump called with packet len = %d\n", 
	 binbuflen);


  if (NULL == binbuf || 0 == binbuflen) {
    scbuf[0] = '\0';
    return scbuf;
  }

  if(PKG_TYPE_NMEA2000 == PKG_TYPE_NMEA2000) {

    char tmp[255];
    sprintf(tmp, "%s,%u,%u,%u,%u,%u,", 
	    "now", 0, 
	    device->driver.vyspi.last_pgn, 
	    0, 0, 
	    binbuflen);

    for (i = 0; i < strlen(tmp) && i < scbuflen; i++) {
      scbuf[j++] = tmp[i];
    }

    char * ibuf = (const char *)&binbuf[0];
    const char *hexchar = "0123456789abcdef";

    /*@ -shiftimplementation @*/
    for (i = 0; i < maxlen && i * 3 < scbuflen - 3; i++) {
      scbuf[j++] = hexchar[(ibuf[i] & 0xf0) >> 4];
      scbuf[j++] = hexchar[ibuf[i] & 0x0f];
      scbuf[j++] = ',';
    }
    /*@ +shiftimplementation @*/
    j--; // back last ',' to overwrite with '\0'
    scbuf[j] = '\0';

    /*
  } else if (pkgType == PKG_TYPE_NMEA0183) {
      
    if(len + pkgLen >= binbuflen) 
      break;

    printf("VYSPI: NMEA 0183 org= %d, len= %d\n", 
	   pkgOrg, pkgLen);

    memcpy(scbuf, (char *)&binbuf[len], pkgLen);
    j += pkgLen;

    scbuf[j++] = '\0';

    // length is packet length
    len += pkgLen;
    */
  }

  return scbuf;
}
/*@+mustdefine@*/

/* *INDENT-OFF* */
const struct gps_type_t driver_vyspi = {
    .type_name      = "VYSPI",       /* full name of type */
    .packet_type    = VYSPI_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_STICKY,	/* remember this */
    .trigger	    = NULL,		/* detect their main sentence */
    .channels       = 12,		/* not an actual GPS at all */
    .probe_detect   = NULL,
    .get_packet     = vyspi_get,	/* how to get a packet */
    .parse_packet   = vyspi_parse_input,	/* how to interpret a packet */
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

#endif /* of  defined(VYSPI_ENABLE) */
