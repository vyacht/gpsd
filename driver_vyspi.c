/**
 *  \file driver_vyspi.c This is the main driver for vyacht microcontroller protocol.
 *
 *  @page vy_pgns PGNs
 *  @brief PGNs page
 */
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
#include <stddef.h>
#include <limits.h>
#include <assert.h>

#include "gpsd.h"
#if defined(VYSPI_ENABLE)
#include "frame.h"
#include "driver_vyspi.h"
#include "bits.h"

#include "json.h"
#include "utils.h"
#include "navigation.h"
#include "timeutil.h"

#define LOG_FILE 1
#define VYSPI_RESET 0x04

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02
#define PKG_TYPE_ST       0x04

uint32_t vy_port_speeds[] = {
  PORT_SPEED_4800,
  PORT_SPEED_38400,
  PORT_SPEED_115200
};

uint32_t last_error_stats_report = 0;


/*
   pgn, fast, type, receive, transmit, func, name

   we only list those that report data into structure or read from it as pseudo n2k
*/

static gps_mask_t hnd_059392(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_059904(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_060928(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_126208(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_126464(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_126992(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_126996(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129025(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129026(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129029(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129539(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129540(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129541(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129038(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129039(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129040(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129793(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129794(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129798(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129802(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129809(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129810(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130842(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127506(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127508(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127513(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127488(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127489(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127493(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127505(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127237(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127245(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127250(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127257(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127251(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_127258(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_128259(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_128267(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_128275(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129033(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129283(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129284(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129285(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_129291(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130306(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130310(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130311(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130312(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
static gps_mask_t hnd_130850(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);

static gps_mask_t hnd_unknown(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);

static struct PGN pgnlist[] = {
    { 59392, 0, 0, 0, 0, hnd_059392, "ISO Acknowledgment"},
    { 59904, 0, 0, 1, 1, hnd_059904, "ISO Request"},
    { 60928, 0, 0, 1, 1, hnd_060928, "ISO  Address Claim"},
    {126208, 0, 0, 0, 0, hnd_126208, "NMEA Command/Request/Acknowledge"},
    {126464, 1, 0, 0, 1, hnd_126464, "ISO  Transmit/Receive PGN List"},
    {126996, 1, 0, 0, 0, hnd_126996, "ISO  Product Information"},
    {129025, 0, 1, 1, 1, hnd_129025, "GNSS Position Rapid Update"},
    {129026, 0, 1, 1, 1, hnd_129026, "GNSS COG and SOG Rapid Update"},
    {129029, 1, 1, 1, 0, hnd_129029, "GNSS Positition Data"},
    {126992, 0, 0, 0, 1, hnd_126992, "GNSS System Time"},
    {129539, 0, 1, 1, 0, hnd_129539, "GNSS DOPs"},
    {129540, 1, 1, 1, 0, hnd_129540, "GNSS Satellites in View"},
    {129541, 1, 1, 0, 0, hnd_129541, "GNSS Almanac Data"},
    {129038, 1, 2, 1, 0, hnd_129038, "AIS  Class A Position Report"},
    {129039, 1, 2, 1, 0, hnd_129039, "AIS  Class B Position Report"},
    {129040, 1, 2, 1, 0, hnd_129040, "AIS  Class B Extended Position Report"},
    {129793, 1, 2, 0, 0, hnd_129793, "AIS  UTC and Date Report"},
    {129794, 1, 2, 1, 0, hnd_129794, "AIS  Class A Static and Voyage Related Data"},
    {129798, 1, 2, 1, 0, hnd_129798, "AIS  SAR Aircraft Position Report"},
    {129802, 1, 2, 1, 0, hnd_129802, "AIS  Safty Related Broadcast Message"},
    {129809, 1, 2, 1, 0, hnd_129809, "AIS  Class B CS Static Data Report, Part A"},
    {129810, 1, 2, 1, 0, hnd_129810, "AIS  Class B CS Static Data Report, Part B"},
    {130842, 1, 2, 1, 0, hnd_130842, "AIS  Class B Static Data Report (msg 24, Part A and B)"},
    {127506, 1, 3, 0, 0, hnd_127506, "PWR DC Detailed Status"},
    {127508, 1, 3, 0, 0, hnd_127508, "PWR Battery Status"},
    {127513, 1, 3, 0, 0, hnd_127513, "PWR Battery Configuration Status"},
    {127488, 0, 0, 1, 0, hnd_127488, "Engine Parameters, Rapid"},
    {127489, 1, 3, 1, 0, hnd_127489, "Engine Parameters, Dynamic"},
    {127493, 1, 3, 0, 0, hnd_127493, "Transmissions Parameters, Dynamic"},
    {127505, 1, 3, 0, 0, hnd_127505, "Fluid "},
    {127237, 0, 0, 0, 0, hnd_127237, "Heading/Track Control"},
    {127245, 0, 4, 1, 0, hnd_127245, "NAV Rudder"},
    {127250, 0, 4, 1, 1, hnd_127250, "NAV Vessel Heading"},
    {127257, 0, 0, 1, 0, hnd_127257, "Attitude"},
    {127251, 0, 0, 1, 0, hnd_127251, "Rate of Turn"},
    {127258, 0, 0, 1, 0, hnd_127258, "GNSS Magnetic Variation"},
    {128259, 0, 4, 1, 0, hnd_128259, "NAV Speed"},
    {128267, 0, 4, 1, 0, hnd_128267, "NAV Water Depth"},
    {128275, 1, 4, 1, 0, hnd_128275, "NAV Distance Log"},
    {129033, 1, 1, 0, 1, hnd_129033, "Time & Date"},
    {129283, 0, 0, 1, 0, hnd_129283, "NAV Cross Track Error"},
    {129284, 1, 0, 0, 0, hnd_129284, "NAV Navigation Data"},
    {129285, 1, 0, 0, 0, hnd_129285, "NAV Navigation - Route/WP Information"},
    {129291, 0, 0, 0, 0, hnd_129291, "NAV Set & Drift, Rapid Update"},
    {130306, 0, 4, 1, 1, hnd_130306, "NAV Wind Data"},
    {130310, 0, 4, 1, 0, hnd_130310, "NAV Water Temp., Outside Air Temp., Atmospheric Pressure"},
    {130311, 0, 4, 1, 0, hnd_130311, "NAV Temperature"},
    {130312, 0, 4, 1, 0, hnd_130312, "NAV Temperature"},
    {130850, 0, 0, 0, 0, hnd_130850, "Simnet Autopilot Commands"},

    {     0, 0, 0, 0, 0, hnd_unknown, "Unkown sentence"},

};

ssize_t vyspi_write_with_protocol(struct gps_device_t *session,
                                  enum frm_type_t frm_type,
                                  const uint8_t *buf,
                                  const size_t len,
                                  const uint8_t protocol_version);

// some functions from packet.c we only use here
extern void packet_accept(struct gps_packet_t *lexer, int packet_type);
extern void nextstate(struct gps_packet_t *lexer, unsigned char c);
extern void character_discard(struct gps_packet_t *lexer);

extern gps_mask_t seatalk_parse_input(struct gps_device_t *session);
extern gps_mask_t process_seatalk(uint8_t * cmdBuffer, uint8_t size,
                                  struct gps_device_t *session);

extern bool aivdm_decode(const char *buf, size_t buflen,
                         struct gps_device_t *session,
                         struct ais_t *ais,
                         int debug);

static void print_data(struct gps_context_t *context,
                       unsigned char *buffer, int len, struct PGN *pgn);


// from driver_seatalk.c
extern void character_skip(struct gps_packet_t *lexer);

int vy_port_list_read(struct gps_device_t *session, struct devconfig_t * dev);
int vy_port2cmd(struct device_port_t * vy, uint8_t *cmd);

static uint16_t crt_060928(uint8_t bufp[], size_t len,
                           uint8_t prio, uint8_t src, uint8_t dest,
                           struct gps_device_t *session);

static void vyspi_claim_our_source_addr(struct gps_device_t *session);

/*@-nullassign@*/
static void print_data(struct gps_context_t *context,
		       unsigned char *buffer, int len, struct PGN *pgn)
{
#ifdef LIBGPS_DEBUG
    /*@-bufferoverflowhigh@*/
    if ((libgps_debuglevel >= LOG_IO) != 0) {
        int   l1, l2, ptr;
        char  bu[128];

        ptr = 0;
        l2 = sprintf(&bu[ptr], "VYSPI: :%6u:%3d: ", pgn->pgn, len);
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

static int vy_decode_ais_header(struct gps_device_t *device,
    unsigned char *bu, int len, struct ais_t *ais, unsigned int mask)
{
    ais->own_mmsi = 0;
    if (len > 4) {
        ais->type   = (unsigned int) ( bu[0]       & 0x3f);
        ais->repeat = (unsigned int) ((bu[0] >> 6) & 0x03);
        ais->mmsi   = (unsigned int)  getleu32(bu, 1);
        ais->mmsi  &= mask;
        if(ais->mmsi == device->gpsdata.own_mmsi)
            ais->own_mmsi = 1;
        gpsd_report(device->context->debug, LOG_INF,
                    "VY:NMEA2000 AIS message type %u, own MMSI %09u, %s MMSI %09u:\n",
                    ais->type, device->gpsdata.own_mmsi, ais->own_mmsi?"own":"", ais->mmsi);
        return(1);
    } else {
        ais->type   =  0;
        ais->repeat =  0;
        ais->mmsi   =  0;
        gpsd_report(device->context->debug, LOG_ERROR,
                    "VY:NMEA2000 AIS message type %u, too short message.\n",
                    ais->type);
    }
    return(0);
}


static void vy_decode_ais_channel_info(unsigned char *bu,
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


static int vy_ais_turn_rate(int rate)
{
    if(rate == 0x7fff) {
        return AIS_TURN_NOT_AVAILABLE;
    }
    if (rate < 0) {
        return(-vy_ais_turn_rate(-rate));
    }
    return((int)(4.733 * sqrt(rate * RAD_2_DEG * .0001 * 60.0)));
}

static double vy_ais_direction(unsigned int val, double scale)
{
    if (val == 0xffff) {
        if(((int)scale) == 1) {
            return(AIS_HEADING_NOT_AVAILABLE);
        } else {
            return(AIS_COURSE_NOT_AVAILABLE);
        }
    }
    return(val * RAD_2_DEG * 0.0001 * scale);
}

/*
 *   PGN 59392: ISO Acknowledgment
 */
static gps_mask_t hnd_059392(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO Ack\n");
    return(0);
}

/*
 *   PGN 59904: ISO Request
 */
static uint16_t crt_059904(uint8_t bufp[], size_t len,
                           uint8_t prio, uint8_t src, uint8_t dest,
                           uint32_t pgn,
                           struct gps_device_t *session)
{
    if(len < 8+7)
        return 0;
    set8leu32(bufp, 59904, 0);

    bufp[4]= prio;
    bufp[5]= src;
    bufp[6]= dest;

    memset(bufp + 7, 0, 8);
    set8leu16(bufp, (uint16_t)(pgn & 0xffff), 7);
    set8leu8(bufp, (uint8_t)((pgn >> 16) & 0xffff), 7 + 2);

    gpsd_report(session->context->debug, LOG_DATA,
                "creating pgn %6u requesting pgn %u:\n", 59904, pgn);

    return 8+7;
}

static void vyspi_send_product_information(struct gps_device_t *session) {

    static const char * modelid = "vyacht network router";
    static const char * serial_code = "130524";
    static const char * model_version = "9.4";

    char software_version[32];

    uint8_t bu[134 + 7];

    sprintf(software_version, "%s", VERSION);

    memset(bu, 0xff, 134);
    set8leu32(bu, 126996, 0);

    bu[4]= 0x06;
    bu[5]= session->driver.nmea2000.own_src_id;
    bu[6]= 0xff;

    set8leu16(bu, (uint16_t)(0x0514), 7); // stole this from NGW-1
    set8leu16(bu, (uint16_t)(0x2c69), 9); // stole this from NGW-1
    memcpy(bu + 11, modelid, strlen(modelid));
    memcpy(bu + 43, software_version, strlen(software_version));
    memcpy(bu + 75, model_version, strlen(model_version));
    memcpy(bu + 107, serial_code, strlen(serial_code));
    set8leu8(bu, 0x00, 139);             // stole cert level from NGW-1
    set8leu8(bu, 0x01, 140);             // stole load from NGW-1

    // using protocol version 1
    vyspi_write_with_protocol(session, FRM_TYPE_NMEA2000, bu, 134 + 7, 1);

    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - sent product information from src= %u\n",
                session->driver.nmea2000.own_src_id);
}

static void vyspi_send_pgn_list(struct gps_device_t *session) {

    uint8_t bu[223];
    uint16_t l1, l2;
    uint8_t r;

    struct PGN pgn;

    memset(bu, 0xff, 223);
    set8leu32(bu, 126464, 0);

    bu[4]= 0x06;
    bu[5]= session->driver.nmea2000.own_src_id;
    bu[6]= 0xff;

    for(r = 0; r < 2; r++) {

        bu[7] = r; // 0= transmit, 1= receive

        l1 = 0; l2= 0;
        //TODO - not sure what happens if we need to report more one day
        while ((pgnlist[l1].pgn != 0) && (8+l2*3 + 3 < 223)) {
            if ( ((r == 0) && pgnlist[l1].transmit)
                 || ((r == 1) && pgnlist[l1].receive)) {
                     set8leu24(bu, pgnlist[l1].pgn, 8+l2*3);
                     l2++;
            }
            l1++;
        }

        // using protocol version 1
        vyspi_write_with_protocol(session, FRM_TYPE_NMEA2000, bu, 8+l2*3, 1);

        pgn.pgn = 126464;
        print_data(session->context, bu, 8+l2*3, &pgn);
    }

    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - sent receive & transmit list from src= %u\n",
                session->driver.nmea2000.own_src_id);
}

/**
 *   PGN 59904: ISO Request
 */
static gps_mask_t hnd_059904(unsigned char *bu, int len UNUSED, struct PGN *pgn, struct gps_device_t *session)
{
    uint32_t request_pgn =
        (uint32_t)( (uint16_t)getleu16(bu, 0) | ((uint16_t)(bu[2]) << 16));

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - PGN requested= %u\n", request_pgn);

    if((session->driver.vyspi.dest == 0xff)
       || (session->driver.vyspi.dest == session->driver.nmea2000.own_src_id)) {

        if(request_pgn == 60928) {
            vyspi_claim_our_source_addr(session);
        } else if(request_pgn == 126996) {
            vyspi_send_product_information(session);
        } else if(request_pgn == 126464) {
            vyspi_send_pgn_list(session);
        }
    }

    return(0);
}


static void vyspi_claim_our_source_addr(struct gps_device_t *session)
{
        // make a claim back with lower ecu name and same src addr
        uint8_t cmd[64];
        uint16_t len;

        if(session->driver.nmea2000.own_src_id >= 240)
            return;

        len = crt_060928(cmd, 64,
                         0x06,
                         session->driver.nmea2000.own_src_id,
                         255, // its a broadcast to all
                         session);

        // using protocol version 1
        vyspi_write_with_protocol(session, FRM_TYPE_NMEA2000, cmd, len, 1);

        gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - claimed source src= %u\n",
                    session->driver.nmea2000.own_src_id);

        // nothing more to do here
        // - if other nodes "complains" it will send address claims and we'll just react to that
}

static void vyspi_addr_claim_call(struct gps_device_t *session)
{
    // init source addresses
    uint16_t e = 0;
    for(e = 0; e < 256; e++) {
        session->gpsdata.ecu_names[e] = 0;
    }

    // make an iso claim address call to enumerate nodes out there
    uint8_t cmd[64];
    size_t len = 0;

    len = crt_059904(cmd, 64,
                     0x06,
                     0xfe, // Source Address is 254 (NULL address), as required by protocol
                     255, // its a broadcast to all
                     60928,
                     session);

    gpsd_report(session->context->debug, LOG_INF,
                "NMEA 2000 ISO - making an address claim call.\n");

    // using protocol version 1
    vyspi_write_with_protocol(session, FRM_TYPE_NMEA2000, cmd, len, 1);
}

/*
 *
 */
static void vyspi_claim_free_source_addr(struct gps_device_t *session)
{
    // now we need to send our own address claim based on a free address
    uint16_t e = 0;
    session->driver.nmea2000.own_src_id = 255;
    for(e = 1; e < 256; e++) {
        if((session->gpsdata.ecu_names[e] == 0)
           && (session->gpsdata.src_addr_seen[e] == 0)){
            // this is a free source address
            session->driver.nmea2000.own_src_id = e;
            break;

        } else {
            uint64_t ecu = session->gpsdata.ecu_names[e];

            uint32_t uniq   = (ecu >>  0) & 0x1fffff;
            uint16_t manu   = (ecu >> 21) & 0x7ff;

            if(ecu != 0)
                gpsd_report(session->context->debug, LOG_INF,
                            "Search free src addr: 0x%02x in use by %u:%u\n", e, manu, uniq);
            else
                gpsd_report(session->context->debug, LOG_INF,
                            "Search free src addr: 0x%02x in use by unkown device\n", e);

        }
    }

    gpsd_report(session->context->debug, LOG_INF,
                "Claim (new) source address %u.\n",
                session->driver.nmea2000.own_src_id);

    if(session->driver.nmea2000.own_src_id < 240) {
        vyspi_claim_our_source_addr(session);
    }
}

static void vyspi_get_ecu_name(uint64_t * ecu_name, struct gps_device_t *session) {

    *ecu_name = 0xc0f0823fe76b4b35;

    *ecu_name &= ~((uint64_t)0x7ff << 21);
    *ecu_name &= ~((uint64_t)0x1fffff << 0);

    *ecu_name |= ((uint64_t)session->driver.nmea2000.manufactureid << 21);
    *ecu_name |= ((uint64_t)session->driver.nmea2000.deviceid << 0);
}

/*
 *   PGN 60928: ISO  Address Claim
 */
static uint16_t crt_060928(uint8_t bufp[], size_t len,
                       uint8_t prio, uint8_t src, uint8_t dest,
                       struct gps_device_t *session)
{
    if(len < 8+7)
        return 0;

    uint64_t ECU_name;
    vyspi_get_ecu_name(&ECU_name, session);

    set8leu32(bufp, 60928, 0);

    bufp[4]= prio;
    bufp[5]= src;
    bufp[6]= dest;

    set8leu64(bufp, ECU_name, 7);

    gpsd_report(session->context->debug, LOG_DATA,
                "creating pgn %6d with ecu name %lu:\n", 60928, ECU_name);

    return 8+7;
}

/*
 *   PGN 60928: ISO  Address Claim
 */
static gps_mask_t hnd_060928(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    uint8_t  dc, ilo, ihi;

    // This actually all we need. We can disect this for educational purposes though.
    ECU_name = getleu64(bu, 0);
    uniq   = (ECU_name >>  0) & 0x1fffff;
    manu   = (ECU_name >> 21) & 0x7ff;
    ilo    = bu[4] & 0x07;
    ihi    = (bu[4] >> 3) & 0x1F;
    grp    = (getub(bu, 7) >> 4) & 0x07;
    dc     = (getub(bu, 6) >> 1) & 0x7F;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - Id= %u, Manufacturer= %u, Industry Group = %u, "
                "Device Class = %u, "
                "Device Instance Lower = %u, "
                "Device Instance Upper = %u, "
                "Device Function = %u\n",
                uniq,
                manu,
                grp, dc, ilo, ihi, bu[5]);

    if(session->gpsdata.dev.protocol_version) {
        // record that we have seen this source address with this ecu name
        // no test needed, always 0 <= src <= 255 due to uint8_t
        session->gpsdata.ecu_names[session->driver.vyspi.src] = ECU_name;
    }

    if(session->driver.nmea2000.own_src_id == session->driver.vyspi.src) {

        gpsd_report(session->context->debug, LOG_INF,
                    "NMEA 2000 ISO - source address 0x%02x claims our own source address.\n",
                    session->driver.nmea2000.own_src_id);

        // someone claims our source address
        uint64_t our_ecu_name;
        vyspi_get_ecu_name(&our_ecu_name, session);

        // lower wins, then claim a new one if we are higer
        if(ECU_name < our_ecu_name) {
            gpsd_report(session->context->debug, LOG_INF,
                        "NMEA 2000 ecu name higher, we loose.\n");
            // all info should be collected already
            vyspi_claim_free_source_addr(session);
        } else {
            gpsd_report(session->context->debug, LOG_INF,
                        "NMEA 2000 ecu name higher, we win.\n");
            // reclaim our so that other node knows
            vyspi_claim_our_source_addr(session);
        }
    }

    return(0);
}


/*
 *   PGN 126208: NMEA Command/Request/Acknowledge
 */
static gps_mask_t hnd_126208(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
                "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - Commandd/Request/Ack\n");
    return(0);
}

/*
 *   PGN 126464: ISO Transmit/Receive PGN List
 */
static gps_mask_t hnd_126464(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    int i = 0;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if(session->context->debug >= LOG_IO) {
        gpsd_report(session->context->debug, LOG_IO,
                    "                   NMEA 2000 ISO - Transmit/Receive PGN List\n");

        if(bu[0] == 0) {
            gpsd_report(session->context->debug, LOG_IO,
                        "                   NMEA 2000 ISO - Transmit\n");
        } else {
            gpsd_report(session->context->debug, LOG_IO,
                        "                   NMEA 2000 ISO - Receive\n");
        }

        // staring at 0 as we need last 3 of 4 bytes read starting at 1
        for(i = 1; i < len; i+= 3) {
            uint32_t p = getleu24(bu, i);
            gpsd_report(session->context->debug, LOG_IO,
                        "                   [%u] %u\n", i, p);
        }
    }

    return(0);
}


/*
 *   PGN 126996: ISO  Product Information
 */
static gps_mask_t hnd_126996(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    uint16_t n2k_version = getleu16(bu, 0);
    uint16_t prod_code = getleu16(bu, 2);

    char model_id[33];
    char software_version[33];
    char model_version[33];
    char model_serial_code[33];

    uint8_t cert_level = bu[132];
    uint8_t load_equivalency = bu[133];

    memset(model_id, 0, 33);
    memset(software_version, 0, 33);
    memset(model_version, 0, 33);
    memset(model_serial_code, 0, 33);

    memcpy(model_id, bu + 4, 32);
    memcpy(software_version, bu + 36, 32);
    memcpy(model_version, bu + 68, 32);
    memcpy(model_serial_code, bu + 100, 32);

    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_IO,
                "                   NMEA 2000 ISO - Product Information\n");
    gpsd_report(session->context->debug, LOG_IO,
                "                   n2k version= %u, product code= %u\n",
                n2k_version, prod_code);
    gpsd_report(session->context->debug, LOG_IO,
                "                   cert level= %u, load= %u\n",
                cert_level, load_equivalency);
    gpsd_report(session->context->debug, LOG_IO,
                "                   model id          = %s\n", model_id);
    gpsd_report(session->context->debug, LOG_IO,
                "                   software version  = %s\n", software_version);
    gpsd_report(session->context->debug, LOG_IO,
                "                   model version     = %s\n", model_version);
    gpsd_report(session->context->debug, LOG_IO,
                "                   model serial code = %s\n", model_serial_code);

    return(0);
}


static gps_mask_t setles8value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                          double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    int8_t val = (int8_t)bu[pos];

    if(val != 0x7f) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }

    return mask;
}


static gps_mask_t setleu8value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                         double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    uint8_t val = (uint8_t)bu[pos];
    if(val != 0xff) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }
    return mask;
}

static gps_mask_t setleu16value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                          double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    uint16_t val = getleu16(bu, pos);
    if(val != 0xffff) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }
    return mask;
}

static gps_mask_t setles16value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                          double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    int16_t val = getles16(bu, pos);
    if(val != 0x7fff) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }
    return mask;
}

static gps_mask_t setleu32value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                          double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    uint32_t val = getleu32(bu, pos);
    if(val != 0xffffffff) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }
    return mask;
}

static gps_mask_t setles32value(unsigned char * bu, int pos, gps_mask_t set, gps_mask_t pset, double factor,
                          double * dest_val, gps_mask_t * dest_set) {

    gps_mask_t mask = 0;
    int32_t val = getles32(bu, pos);
    if(val != 0x7fffffff) {
        *dest_val = val*factor;
        *dest_set |= pset;
        mask = set;
    }
    return mask;
}

/*
 *   PGN 127488: Engine Parameter Rapid Update
 */
static gps_mask_t hnd_127488(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /* [17]
    1   0   uint8  Engine instance
    2 1-2   uint16 RPM (x .25 RPM)
    3 3-4   uint16 Boost Pressure (100Pa)
    4   6   sint8  Engine Tilt/Trim (+/- 1%)
    5 6-7   uint16 Reserved Bits
   */

    gps_mask_t pset = 0;
    gps_mask_t mask = 0;
    uint8_t instance  = bu[0];

    if(instance == 0)
        pset = ENG_PORT_PSET;
    else if(instance == 1)
        pset = ENG_STARBOARD_PSET;
    else {
        gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d): unkown engine instance %d\n",
                    pgn->pgn, session->driver.nmea2000.unit, instance);
        return 0;
    }

    mask |= setleu16value(bu, 1, ENGINE_SET, pset | ENG_SPEED_PSET, 0.25,
                  &session->gpsdata.engine.instance[instance].speed, &session->gpsdata.engine.set);

    mask |= setleu16value(bu, 3, ENGINE_SET, pset | ENG_BOOST_PRESSURE_PSET, 100.0,
                  &session->gpsdata.engine.instance[instance].boost_pressure, &session->gpsdata.engine.set);

    mask |= setleu8value(bu, 6, ENGINE_SET, pset | ENG_TILT_PSET, 1,
                  &session->gpsdata.engine.instance[instance].tilt, &session->gpsdata.engine.set);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n",
                pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO, "instance= %s, speed= %0.2f rpm, boost pres= %0.2f, tilt= %0.2f\n",
                instance?"starboard":"single/port",
                session->gpsdata.engine.instance[instance].speed,
                session->gpsdata.engine.instance[instance].boost_pressure,
                session->gpsdata.engine.instance[instance].tilt);

    if(mask > 0)
        return ONLINE_SET | mask;

    return 0;
}

/*
 *   PGN 127489: Engine Parameter Dynamic
 */
static gps_mask_t hnd_127489(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /* [17]
    1      0           uint8    Engine Instance  (port, starboard, forward, etc.)
    2   1- 2           uint16   Engine Oil Pressure (100Pa)
    3   3- 4           uint16   Engine Oil Temperature (0.1K)
    4   5- 6           uint16   Engine Temperature (0.01K)
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

    gps_mask_t pset = 0;
    gps_mask_t mask = 0;
    uint8_t instance  = bu[0];

    if(instance == 0)
        pset = ENG_PORT_PSET;
    else if(instance == 1)
        pset = ENG_STARBOARD_PSET;
    else
        return 0;

    mask |= setleu16value(bu, 1, ENGINE_SET, pset | ENG_OIL_PRESSURE_PSET, 100.0,
                  &session->gpsdata.engine.instance[instance].oil_pressure, &session->gpsdata.engine.set);
    mask |= setleu16value(bu, 3, ENGINE_SET, pset | ENG_OIL_TEMPERATURE_PSET, 0.1,
                  &session->gpsdata.engine.instance[instance].oil_temperature, &session->gpsdata.engine.set);
    mask |= setleu16value(bu, 5, ENGINE_SET, pset | ENG_TEMPERATURE_PSET, 0.01,
                  &session->gpsdata.engine.instance[instance].temperature, &session->gpsdata.engine.set);
    mask |= setleu16value(bu, 7, ENGINE_SET, pset | ENG_ALTERNATOR_VOLTAGE_PSET, 0.01,
                  &session->gpsdata.engine.instance[instance].alternator_voltage, &session->gpsdata.engine.set);

    mask |= setleu16value(bu, 9, ENGINE_SET, pset | ENG_FUEL_RATE_PSET, 1000,
                  &session->gpsdata.engine.instance[instance].fuel_rate, &session->gpsdata.engine.set);
    mask |= setleu32value(bu, 11, ENGINE_SET, pset | ENG_TOTAL_HOURS_PSET, 1.0,
                  &session->gpsdata.engine.instance[instance].total_hours, &session->gpsdata.engine.set);
    mask |= setleu16value(bu, 15, ENGINE_SET, pset | ENG_COOLANT_PRESSURE_PSET, 1000.0,
                  &session->gpsdata.engine.instance[instance].coolant_pressure, &session->gpsdata.engine.set);
    mask |= setleu16value(bu, 17, ENGINE_SET, pset | ENG_FUEL_PRESSURE_PSET, 0.01,
                  &session->gpsdata.engine.instance[instance].fuel_pressure, &session->gpsdata.engine.set);

    // TODO - descrete statuses

    mask |= setles8value(bu, 22, ENGINE_SET, pset | ENG_TORQUE_PSET, 0.01,
                 &session->gpsdata.engine.instance[instance].torque, &session->gpsdata.engine.set);
    mask |= setles8value(bu, 23, ENGINE_SET, pset | ENG_LOAD_PSET, 0.01,
                 &session->gpsdata.engine.instance[instance].load, &session->gpsdata.engine.set);


    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    gpsd_report(session->context->debug, LOG_IO, "instance= %s, oil pres= %0.2f, oil temp= %0.2f, temp= %0.2f (%04x)\n",
                instance?"starboard":"single/port",
                session->gpsdata.engine.instance[instance].oil_pressure,
                session->gpsdata.engine.instance[instance].oil_temperature - 273.15,
                session->gpsdata.engine.instance[instance].temperature - 273.15,
                getleu16(bu, 5));
    gpsd_report(session->context->debug, LOG_IO, "volt= %0.2fV, fule rate= %0.2fL/h, run time= %0.2fsecs, coolant pres= %0.2f\n",
                session->gpsdata.engine.instance[instance].alternator_voltage,
                session->gpsdata.engine.instance[instance].fuel_rate,
                session->gpsdata.engine.instance[instance].total_hours,
                session->gpsdata.engine.instance[instance].coolant_pressure);
    gpsd_report(session->context->debug, LOG_IO, "fuel press= %0.2fpa, torque= %0.2f%%, load= %0.2f%%\n",
                session->gpsdata.engine.instance[instance].fuel_pressure,
                session->gpsdata.engine.instance[instance].torque,
                session->gpsdata.engine.instance[instance].load);


    if(mask > 0)
        return ONLINE_SET | mask;
    return 0;
}

/*
 *   PGN 127493: Transmission Parameters, Dynamic
 */
static gps_mask_t hnd_127493(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*
 *   PGN 127505: Fluid Level
 */
static gps_mask_t hnd_127505(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
static gps_mask_t hnd_127257(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /* [8] The SSC200 uses this PGN to indicate the vessel’s attitude (pitch and roll). The Yaw (field 2) is not used, therefore this field always contains 0x7FFF (data not available).

           1:  SID – The sequence identifier field is used to tie related PGNs together. For example, the SSC200 will transmit identical SIDs for Vessel Heading (PGN 127250), Attitude (127257), and Rate of Turn (127251) to indicate that the readings are linked together (i.e., the data from each PGN was taken at the same time although they are reported at slightly different times).

           2:  Yaw – This field always contains a value of 0x7FFF (data not available).

           3:  Pitch – This field is used to report the vessel’s pitch.

           4:  Roll – This field is used to report the vessel’s roll.

           5:  Reserved – This field is reserved by NMEA; therefore, the SSC200 sets all bits to a logic 1.
  */
    gps_mask_t mask = 0;
    uint8_t sid    = bu[0];

    // testing with a67
    int16_t yaw   = getles16(bu, 1);
    int16_t pitch = getles16(bu, 3);
    int16_t roll  = getles16(bu, 5);

    if(yaw == 0x7fff) {
      session->gpsdata.attitude.yaw = NAN;
    } else
      session->gpsdata.attitude.yaw = yaw * 0.0001 * RAD_2_DEG;

    if(roll == 0x7fff) {
      session->gpsdata.attitude.roll = NAN;
    } else
      session->gpsdata.attitude.roll = roll * 0.0001 * RAD_2_DEG;

    if(pitch == 0x7fff) {
      session->gpsdata.attitude.pitch = NAN;
    } else
      session->gpsdata.attitude.pitch = pitch * 0.0001 * RAD_2_DEG;

    mask |= (ONLINE_SET | ATTITUDE_SET);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
                "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
                "                   SID= %u, yaw=%f, roll=%f, pitch= %f\n",
                sid,
                (session->gpsdata.attitude.yaw==NAN?0:session->gpsdata.attitude.yaw),
                (session->gpsdata.attitude.yaw==NAN?0:session->gpsdata.attitude.roll),
                (session->gpsdata.attitude.yaw==NAN?0:session->gpsdata.attitude.pitch));

    return mask;
}

/*
 *   PGN 127258: GNSS Magnetic Variation
 */
static gps_mask_t hnd_127258(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    int16_t age;
    double  var;
    uint16_t reserved2;

    sid             = bu[0];

    src             = (bu[1] >> 0) & 0x0f;          // TODO, 5 == WMM2005 ([7]), == manual ([3])

    reserved1       = (bu[1] >> 4) & 0x0f;          // 4 bits ([7]), always 0xf

    // TODO: 0x7fff for N/A ([8])
    age             = getleu16(bu, 2); // this is age in days from Jan 1, 1970

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


/**
 *   \PGN 129025: GNSS Position Rapid Update
 */
static gps_mask_t hnd_129025(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    int64_t lat;
    int64_t lon;
    gps_mask_t mask = 0;

    lat = getles32(bu, 0);
    lon = getles32(bu, 4);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if((lat != 0x7fffffff) && (lon != 0x7fffffff)) {
      /*@-type@*//* splint has a bug here */
      session->newdata.latitude =  lat * 1e-7;
      session->newdata.longitude = lon * 1e-7;
      /*@+type@*/
      mask = LATLON_SET | get_mode(session);
    }

    (void)strlcpy(session->gpsdata.tag, "129025", sizeof(session->gpsdata.tag));

    gpsd_report(session->context->debug, LOG_IO,
		"                   lat = %f, lon = %f\n",
		(lat != 0x7fffffff)?session->newdata.latitude:NAN,
		(lon != 0x7fffffff)?session->newdata.longitude:NAN);

    return mask;
}


/**
 *   \PGN 129026: GNSS COG and SOG Rapid Update
 *   http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf
 */
static gps_mask_t hnd_129026(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    // TODO 0 = True, 1 = magnetic, 2 == error, 3 == - ([1])
    uint8_t COG_Reference;
    //    uint8_t Reserved1;
    //uint16_t Reserved2;
    uint16_t track;
    uint16_t speed;
    gps_mask_t mask = 0;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    session->driver.nmea2000.sid[0]  =  bu[0];

    // TODO: 0: True  ([7]), 1: Magnetic (guess)
    // observed: xfd/fc toggled, bu[1] should always be 0x3f
    COG_Reference = (bu[1] >> 6) & 0x03;
    //Reserved1     = (bu[1] >> 2) & 0xff;  // TODO: [7]: 6 bits in total
    int f = (COG_Reference == 0) ? compass_true : compass_magnetic;

    track = getleu16(bu, 2);
    speed = getleu16(bu, 4);

    /*@-type@*//* splint has a bug here */
    if(track != 0xffff) {

        session->gpsdata.navigation.course_over_ground[f] = track * 1e-4 * RAD_2_DEG;
        if(COG_Reference == 0) {
            session->gpsdata.navigation.set  = NAV_COG_TRUE_PSET;
        } else {
            session->gpsdata.navigation.set  = NAV_COG_MAGN_PSET;
        }
        mask = NAVIGATION_SET;
    }

    if(speed != 0xffff) {
        mask |= nav_set_speed_over_ground_in_knots(speed * 1e-2 / KNOTS_TO_MPS,
                                               session);
    }
    /*@+type@*/

    //Reserved2     = getleu16(bu, 6);     // [7]: 2 bytes

    (void)strlcpy(session->gpsdata.tag, "129026", sizeof(session->gpsdata.tag));

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, cog ref= %s, track= %.2f deg, speed= %.2f m/s; %.2f knots\n",
		session->driver.nmea2000.sid[0],
		(COG_Reference == 0) ? "True" : "Magnetic",
		(track != 0xffff)?session->gpsdata.navigation.course_over_ground[f] : 0.0,
		(speed != 0xffff)?session->gpsdata.navigation.speed_over_ground * KNOTS_TO_MPS : 0.0,
		(speed != 0xffff)?session->gpsdata.navigation.speed_over_ground : 0.0);

    return mask;
}

/**
 *   \PGN 129029: GNSS Positition Data
 */
static gps_mask_t hnd_129029(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask;
    int64_t lat;
    int64_t lon;
    int64_t alt;
    int32_t sep;
    int16_t pdop;
    int16_t hdop;


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

    lat = getles64(bu, 7);
    lon = getles64(bu, 15);

    if((lat != 0x7fffffffffffffff) && (lon !=  0x7fffffffffffffff)) {
      session->newdata.latitude        = lat * 1e-16;
      session->newdata.longitude       = lon * 1e-16;

      /*@+type@*/
      mask                            |= LATLON_SET;
    }

    /*@-type@*//* splint has a bug here */
    alt = getles64(bu, 23);
    if(alt != 0x7fffffffffffffff) {
      session->newdata.altitude        = alt * 1e-6;
      /*@+type@*/
      mask                            |= ALTITUDE_SET;
    }

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

    /// \todo nmea 0183 driver sets mode here as well based on attitude

    /*@-type@*//* splint has a bug here */
    sep = getles32(bu, 38);
    if(sep != 0x7fffffff) {
      session->gpsdata.separation      = sep / 100.0;
      /*@+type@*/
      if(alt != 0x7fffffffffffffff) {
	session->newdata.altitude       -= session->gpsdata.separation;
      }
    }

    if(bu[33] != 0xff) {
      session->gpsdata.satellites_used = (int)bu[33];
    }

    /*@-type@*//* splint has a bug here */
    hdop = getles16(bu, 34);
    pdop = getles16(bu, 36);

    if((hdop != 0x7fff) && (pdop != 0x7fff)) {
      session->gpsdata.dop.hdop        = hdop * 0.01;
      session->gpsdata.dop.pdop        = pdop * 0.01;
      /*@+type@*/
      mask                            |= DOP_SET;
    }

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
		"                   status = %d, sep = %f, sats = %d, hdop = %f, pdop = %f, mode=%u\n",
		session->gpsdata.status,
		session->gpsdata.separation,
		session->gpsdata.satellites_used,
		session->gpsdata.dop.hdop,
		session->gpsdata.dop.pdop,
        session->driver.nmea2000.mode);

    return mask | get_mode(session);
}

/**
 * \PGN 126992: GNSS System Time
 */
static gps_mask_t hnd_126992(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    uint8_t        sid;

    // TODO 0=GPS,1=GLONASS,2=Radio Station,3=Local Cesium clock,4=Local Rubidium clock,5=Local Crystal clock ([1])
    uint8_t        source;   // based on reserved bits: 4 bits,
    //    uint8_t        reserved; // [7] 4 bits

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    sid        = bu[0];
    source     = (bu[1] >> 0) & 0x0f;
    //reserved   = (bu[1] >> 4) & 0x0f;

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

/**
 *   \PGN 129539: GNSS DOPs
 */
static gps_mask_t hnd_129539(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    int16_t hdop, vdop, tdop;

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

    hdop = getles16(bu, 2);
    vdop = getles16(bu, 4);
    tdop = getles16(bu, 6);

    /*@-type@*//* splint has a bug here */
    if(hdop != 0x7fff) {
      session->gpsdata.dop.hdop        = hdop * 1e-2;
      mask                            |= DOP_SET;
    }
    if(vdop != 0x7fff) {
      session->gpsdata.dop.vdop        = vdop * 1e-2;
      mask                            |= DOP_SET;
    }
    if(tdop != 0x7fff) {
      session->gpsdata.dop.tdop        = tdop * 1e-2;
      mask                            |= DOP_SET;
    }
    /*@+type@*/

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    gpsd_report(session->context->debug, LOG_IO,
		"                   SID:%02x hdop:%5.2f vdop:%5.2f tdop:%5.2f mode:%u\n",
		session->driver.nmea2000.sid[1],
		session->gpsdata.dop.hdop,
		session->gpsdata.dop.vdop,
		session->gpsdata.dop.tdop,
        session->driver.nmea2000.mode);

    (void)strlcpy(session->gpsdata.tag, "129539", sizeof(session->gpsdata.tag));

    return mask | get_mode(session);
}


/**
 *   \PGN 129540: GNSS Satellites in View
 */
static gps_mask_t hnd_129540(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

   gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d): satellites visible: %d\n",
               pgn->pgn, session->driver.nmea2000.unit,
               session->gpsdata.satellites_visible);

    l2 = 0;
    for (l1=0; l1 < session->gpsdata.satellites_visible; l1++) {
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
static gps_mask_t hnd_129541(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    return 0;
}


/**
 * \todo PGN 129033 Time & Date [2]
 * this is missing the correct setting of the mask
 */
static gps_mask_t hnd_129033(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask = 0;

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
    return mask;
}


/*
 *   PGN 129038: AIS  Class A Position Report
 */
static gps_mask_t hnd_129038(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;
    int32_t lat, lon;
    uint32_t speed;
    uint16_t course;
    char channel;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"vy pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {

        lon = getles32(bu, 5);
        lat = getles32(bu, 9);
        course = getleu16(bu, 14);
        speed = getleu16(bu, 16);

        if(lon != 0x7fffffff) {
            ais->type1.lon = (int)(lon * 0.06);
        } else {
            ais->type1.lon = AIS_LON_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LON \n");
        }

        if(lat != 0x7fffffff) {
            ais->type1.lat = (int)(lat * 0.06);
        } else {
            ais->type1.lat = AIS_LAT_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LAT \n");
        }

	ais->type1.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
	ais->type1.raim      = (bool)         ((bu[13] >> 1) & 0x01);
	ais->type1.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
	ais->type1.course    = (unsigned int)  vy_ais_direction((unsigned int)course, 10.0);

    if(speed != 0xffff)  {
        ais->type1.speed     = (unsigned int) (speed * MPS_TO_KNOTS * 0.01 / 0.1);
        if(ais->type1.speed > AIS_SPEED_FAST_MOVER) {
            ais->type1.speed = AIS_SPEED_FAST_MOVER;
        }
    }
    else {
        ais->type1.speed     = AIS_SPEED_NOT_AVAILABLE;
        gpsd_report(session->context->debug, LOG_DATA,
                    "NOT AVAILABLE: SPEED \n");
    }

	ais->type1.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
	ais->type1.heading   = (unsigned int)  vy_ais_direction((unsigned int)getleu16(bu, 21), 1.0);
	ais->type1.turn      =                 vy_ais_turn_rate((int)getles16(bu, 23));
	ais->type1.status    = (unsigned int) ((bu[25] >> 0) & 0xff);
	ais->type1.maneuver  = 0; /* Not transmitted ???? */

	vy_decode_ais_channel_info(bu, len, 163, session);

    if(ais->type1.heading == AIS_HEADING_NOT_AVAILABLE) {
        gpsd_report(session->context->debug, LOG_DATA,
                    "NOT AVAILABLE: HEADING \n");
    }
    if(ais->type1.course == AIS_COURSE_NOT_AVAILABLE) {
        gpsd_report(session->context->debug, LOG_DATA,
                    "NOT AVAILABLE: COURSE \n");
    }
    if(ais->type1.turn == AIS_TURN_NOT_AVAILABLE) {
        gpsd_report(session->context->debug, LOG_DATA,
                    "NOT AVAILABLE: TURN \n");
    }

    channel = 'A';
    if (session->driver.aivdm.ais_channel == 'B') {
        channel = 'B';
    }
	gpsd_report(session->context->debug, LOG_IO,
                "                 CLASS %c: lon = %f, lat = %f, course = %u, speed = %u, hdg= %u, turn=%d\n",
                channel,
                ais->type1.lon / AIS_LATLON_DIV,
                ais->type1.lat / AIS_LATLON_DIV,
                ais->type1.course,
                ais->type1.speed,
                ais->type1.heading,
                ais->type1.turn);

	return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129039: AIS  Class B Position Report
 */
static gps_mask_t hnd_129039(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;
    int32_t lat, lon;
    uint16_t speed;
    char channel;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {

        lon = getles32(bu, 5);
        lat = getles32(bu, 9);

        if(lon != 0x7fffffff) {
            ais->type18.lon = (int)(lon * 0.06);
        } else {
            ais->type18.lon = AIS_GNS_LON_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LON \n");
        }

        if(lat != 0x7fffffff) {
            ais->type18.lat = (int)(lat * 0.06);
        } else {
            ais->type18.lat = AIS_GNS_LAT_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LAT \n");
        }

        speed = getleu16(bu, 16);

        ais->type18.accuracy = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type18.raim     = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type18.second   = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type18.course   = (unsigned int)  vy_ais_direction((unsigned int) getleu16(bu, 14), 10.0);

        if(speed != 0xffff) {
            ais->type18.speed    = (unsigned int) (speed * MPS_TO_KNOTS * 0.01 / 0.1);
        } else {
            ais->type18.speed    = AIS_SPEED_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: SPEED\n");
        }
        ais->type18.radio    = (unsigned int) (getleu32(bu, 18) & 0x7ffff);
        ais->type18.heading  = (unsigned int)  vy_ais_direction((unsigned int) getleu16(bu, 21), 1.0);
        ais->type18.reserved = 0;
        ais->type18.regional = (unsigned int) ((bu[24] >> 0) & 0x03);
        ais->type18.cs	     = (bool)         ((bu[24] >> 2) & 0x01);
        ais->type18.display  = (bool)         ((bu[24] >> 3) & 0x01);
        ais->type18.dsc      = (bool)         ((bu[24] >> 4) & 0x01);
        ais->type18.band     = (bool)         ((bu[24] >> 5) & 0x01);
        ais->type18.msg22    = (bool)         ((bu[24] >> 6) & 0x01);
        ais->type18.assigned = (bool)         ((bu[24] >> 7) & 0x01);

        vy_decode_ais_channel_info(bu, len, 163, session);

        channel = 'A';
        if (session->driver.aivdm.ais_channel == 'B') {
            channel = 'B';
        }
        gpsd_report(session->context->debug, LOG_IO,
                    "                  CLASS %c: lon = %f, lat = %f, course = %d, speed = %u, ch = %c\n",
                    channel,
                    ais->type18.lon / AIS_LATLON_DIV,
                    ais->type18.lat / AIS_LATLON_DIV,
                    ais->type18.course,
                    ais->type18.speed,
                    session->driver.aivdm.ais_channel);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129040: AIS Class B Extended Position Report
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129040(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;
    int32_t lat, lon;
    uint16_t speed;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {
        uint16_t length, beam, to_bow, to_starboard;
        int l;

        lon = getles32(bu, 5);
        lat = getles32(bu, 9);

        if(lon != 0x7fffffff) {
            ais->type19.lon = (int)(lon * 0.06);
        } else {
            ais->type19.lon = AIS_GNS_LON_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LON \n");
        }

        if(lat != 0x7fffffff) {
            ais->type19.lat = (int)(lat * 0.06);
        } else {
            ais->type19.lat = AIS_GNS_LAT_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LAT \n");
        }

        speed = getleu16(bu, 16);

        ais->type19.accuracy     = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type19.raim         = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type19.second       = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type19.course       = (unsigned int)  vy_ais_direction((unsigned int) getleu16(bu, 14), 10.0);
        if(speed != 0xffff) {
            ais->type19.speed    = (unsigned int) (speed * MPS_TO_KNOTS * 0.01 / 0.1);
        } else {
            ais->type19.speed    = AIS_SPEED_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: SPEED\n");
        }
        ais->type19.reserved     = (unsigned int) ((bu[18] >> 0) & 0xff);
        ais->type19.regional     = (unsigned int) ((bu[19] >> 0) & 0x0f);
        ais->type19.shiptype     = (unsigned int) ((bu[20] >> 0) & 0xff);
        ais->type19.heading      = (unsigned int)  vy_ais_direction((unsigned int) getleu16(bu, 21), 1.0);

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
        vy_decode_ais_channel_info(bu, len, 422, session);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}

/*
 *   PGN 129793: AIS UTC and Date Report
 */
static gps_mask_t hnd_129793(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {
    }
    return(0);
}

/*
 *   PGN 129794: AIS Class A Static and Voyage Related Data
 */
static gps_mask_t hnd_129794(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {
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
        vy_decode_ais_channel_info(bu, len, 592, session);
        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129798: AIS SAR Aircraft Position Report
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129798(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;
    int32_t lat, lon;
    uint16_t speed;
    uint64_t alt;


    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {

        lon = getles32(bu, 5);
        lat = getles32(bu, 9);
        speed = getleu16(bu, 16);
        alt = getleu64(bu, 21);

        if(lon != 0x7fffffff) {
            ais->type9.lon = (int)(lon * 0.06);
        } else {
            ais->type9.lon = AIS_LON_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LON \n");
        }

        if(lat != 0x7fffffff) {
            ais->type9.lat = (int)(lat * 0.06);
        } else {
            ais->type9.lat = AIS_LAT_NOT_AVAILABLE;
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: LAT \n");
        }

        ais->type9.accuracy  = (bool)         ((bu[13] >> 0) & 0x01);
        ais->type9.raim      = (bool)         ((bu[13] >> 1) & 0x01);
        ais->type9.second    = (unsigned int) ((bu[13] >> 2) & 0x3f);
        ais->type9.course    = (unsigned int)  vy_ais_direction((unsigned int) getleu16(bu, 14), 10.0);

        if(speed != 0xffff) {
            ais->type9.speed     = (unsigned int) (speed * MPS_TO_KNOTS * 0.01 / 0.1);
        } else {
            gpsd_report(session->context->debug, LOG_DATA,
                        "NOT AVAILABLE: SPEED\n");
            ais->type9.speed     = AIS_SAR_SPEED_NOT_AVAILABLE;
        }
        ais->type9.radio     = (unsigned int) (getleu32(bu, 18) & 0x7ffff);

        if(alt != 0xffffffffffffffff) {
            ais->type9.alt       = (unsigned int) (alt/1000000);
            if(ais->type9.alt > AIS_ALT_HIGH)
                ais->type9.alt = AIS_ALT_HIGH;
        } else {
            ais->type9.alt = AIS_ALT_NOT_AVAILABLE;
        }
        ais->type9.regional  = (unsigned int) ((bu[29] >> 0) & 0xff);
        ais->type9.dte	     = (unsigned int) ((bu[30] >> 0) & 0x01);
/*      ais->type9.spare     = (bu[30] >> 1) & 0x7f; */
        ais->type9.assigned  = 0; /* Not transmitted ???? */
        vy_decode_ais_channel_info(bu, len, 163, session);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129802: AIS Safty Related Broadcast Message
 */
/* No test case for this message at the moment */
static gps_mask_t hnd_129802(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0x3fffffff) != 0) {
        int                   l;

/*      ais->type14.channel = (bu[ 5] >> 0) & 0x1f; */
	for (l=0;l<36;l++) {
	    ais->type14.text[l] = (char) bu[6+l];
	}
	ais->type14.text[36] = (char) 0;
	vy_decode_ais_channel_info(bu, len, 40, session);

        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129809: AIS Class B CS Static Data Report, Part A
 */
static gps_mask_t hnd_129809(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
                "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {
        int                   l;
        int                   index   = session->driver.aivdm.context[0].type24_queue.index;
        struct ais_type24a_t *saveptr = &session->driver.aivdm.context[0].type24_queue.ships[index];

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

        gpsd_report(session->context->debug, LOG_PROG,
                    "NMEA2000: AIS message 24A from %09u stashed: %s.\n",
                    ais->mmsi, saveptr->shipname);


        vy_decode_ais_channel_info(bu, len, 200, session);

        ais->type24.part = part_a;
        return(ONLINE_SET | AIS_SET);
    }
    return(0);
}


/*
 *   PGN 129810: AIS Class B CS Static Data Report, Part B
 */
static gps_mask_t hnd_129810(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    struct ais_t *ais;

    ais =  &session->gpsdata.ais;
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if (vy_decode_ais_header(session, bu, len, ais, 0xffffffffU) != 0) {
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
            gpsd_report(session->context->debug, LOG_DATA,
                        "mmsi: %09u length: %u, beam: %u\n",
                        ais->mmsi, length, beam);
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

                gpsd_report(session->context->debug, LOG_DATA,
                            "AIS: MMSI:  %09u\n", ais->mmsi);
                gpsd_report(session->context->debug, LOG_DATA,
                            "AIS: name:  %-20.20s v:%-8.8s c:%-8.8s b:%6u s:%6u p:%6u s:%6u\n",
                            ais->type24.shipname,
                            ais->type24.vendorid,
                            ais->type24.callsign,
                            ais->type24.dim.to_bow,
                            ais->type24.dim.to_stern,
                            ais->type24.dim.to_port,
                            ais->type24.dim.to_starboard);

                vy_decode_ais_channel_info(bu, len, 264, session);
                ais->type24.part = both;
                return(ONLINE_SET | AIS_SET);
            }
        }

        gpsd_report(session->context->debug, LOG_DATA,
                    "AIS: MMSI  :  %09u\n", ais->mmsi);

        gpsd_report(session->context->debug, LOG_DATA,
                    "AIS: vendor:  %-8.8s c:%-8.8s b:%6u s:%6u p:%6u s:%6u\n",
                    ais->type24.vendorid,
                    ais->type24.callsign,
                    ais->type24.dim.to_bow,
                    ais->type24.dim.to_stern,
                    ais->type24.dim.to_port,
                    ais->type24.dim.to_starboard);

        vy_decode_ais_channel_info(bu, len, 264, session);
        ais->type24.part = part_b;
        return(ONLINE_SET | AIS_SET);
    }

    return(0);
}

/*
 *   PGN 130842: Simnet AIS Class B Static Data Report, Part A&B
 */
static gps_mask_t hnd_130842(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if(len == 0x1d)
        return hnd_129809(bu, len, pgn, session);
    else if(len == 0x25)
        return hnd_129810(bu, len, pgn, session);
    return(0);
}


/*
 *   PGN 127506: PWR DC Detailed Status
 */
static gps_mask_t hnd_127506(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/*
 *   PGN 127508: Battery Status
 */
static gps_mask_t hnd_127508(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
static gps_mask_t hnd_127513(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


/**
 *   \PGN 127245: NAV Rudder
 */
static gps_mask_t hnd_127245(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

    // corresponds to RSA port rudder sensor
    position     = getles16(bu, 4) * 0.0001 * RAD_2_DEG;

    reserved2    = getub(bu, 6);

    session->gpsdata.navigation.rudder_angle = position;
    session->gpsdata.navigation.set          = NAV_RUDDER_ANGLE_PSET;

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

    return NAVIGATION_SET;
}


/**
 *   \PGN 127250: NAV Vessel Heading
 */
static gps_mask_t hnd_127250(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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
    double heading = 0.0;
    uint8_t ref = 0; // Heading Sensor Reference (M&T?)
    const size_t buflen = 255;
    // [3] says ref = 1 for magnetic north (how many bits?)

    print_data(session->context, bu, len, pgn);

    sid = getub(bu, 0);

    /*@-type@*/
    hdg = getleu16(bu, 1);
    if (hdg != 0xffff) {
        heading = hdg * RAD_2_DEG * 0.0001;
    }

    dev = getles16(bu, 3);
    if (dev != 0x7fff) {
        session->gpsdata.environment.set       |= ENV_DEVIATION_PSET;
        session->gpsdata.environment.deviation = dev * RAD_2_DEG * 0.0001;
    }

    var = getles16(bu, 5);
    if (var != 0x7fff) {
        session->gpsdata.environment.set       |= ENV_VARIATION_PSET;
        session->gpsdata.environment.variation = var * RAD_2_DEG * 0.0001;
    }

    // observed b0000 0001, 1 = magnetic (see also [3]), 0 = true
    ref = getub(bu, 7) & 0x01; // ??? len, only a couple of bits

    /*@+type@*/
    if (hdg != 0xffff) {
        if(ref == 1) {
            session->gpsdata.navigation.set = NAV_HDG_MAGN_PSET;
            session->gpsdata.navigation.heading[compass_magnetic] = heading;
        } else {
            session->gpsdata.navigation.set = NAV_HDG_TRUE_PSET;
            session->gpsdata.navigation.heading[compass_true] = heading;
        }
    } else {
        session->gpsdata.navigation.set = 0;
        session->gpsdata.navigation.heading[compass_magnetic] = NAN;
        session->gpsdata.navigation.heading[compass_true] = NAN;
    }

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    char bufp[buflen];
    memset(bufp, 0, buflen);

    if (hdg == 0xffff) {
      (void)sprintf(bufp, "Heading= %s, ", "-");
      (void)sprintf(bufp + strlen(bufp), "Sensor= %s, ", "-");
    } else {
      (void)snprintf(bufp, buflen, "Heading= %.2f, ", heading);
      (void)snprintf(bufp + strlen(bufp), buflen - strlen(bufp), "Sensor= %.2f, ",
		     hdg  * RAD_2_DEG * 0.0001);
    }
    if (dev == 0x7fff) {
      (void)sprintf(bufp + strlen(bufp), "Deviation= %s, ", "-");
    } else {
      (void)snprintf(bufp + strlen(bufp), buflen - strlen(bufp),
		     "Deviation= %.2f, ",
		     dev * RAD_2_DEG * 0.0001);
    }
    if (var == 0x7fff) {
      (void)sprintf(bufp + strlen(bufp), "Variation= %s", "-");
    } else {
      (void)snprintf(bufp + strlen(bufp), buflen - strlen(bufp), "Variation= %.2f", var * RAD_2_DEG * 0.0001);
    }

    gpsd_report(session->context->debug, LOG_IO,
		"                   VY: SID = %d, %s, Heading Sensor Reference= %s (%u), Reserved = %u\n",
		sid,
		bufp,
		ref == 1? "Magnetic":"True",
		ref,
		0);

    return (ONLINE_SET | NAVIGATION_SET | ENVIRONMENT_SET);
}

/**
 * \todo PGN 128237: Heading/Track Control
 * is missing a mask and a storage structure for storing the results
 */
static gps_mask_t hnd_127237(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

/**
 *   \PGN 128251: Rate of Turn
 */
static gps_mask_t hnd_127251(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    uint8_t sid;
    double rot;

    sid = getub(bu, 0);

    // TODO : find explain for odd magic number 3/16 deg/min (or 3/16/60 in deg/sec)
    rot = getles32(bu, 1) * 0.00001 * 3.0/16.0/60.0 * RAD_2_DEG; // per sec, let compiler optimize

    session->gpsdata.navigation.set              = NAV_ROT_PSET;
    session->gpsdata.navigation.rate_of_turn     = rot;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   sid = %d, rate = %f deg/s\n",
		sid,
		rot);

    return NAVIGATION_SET;
}


/**
 *   \PGN 128259: NAV Speed
 */
static gps_mask_t hnd_128259(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    const double nm = 1852; // nm in meter
    gps_mask_t mask = 0;

    print_data(session->context, bu, len, pgn);

    uint8_t sid = getub(bu, 0);
    uint16_t speed_water = getleu16(bu, 1);   // 1 x 10-2 m/s
    // [3] says not available means 0xffff, usually not used
    uint16_t speed_ground = getleu16(bu, 3);

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
    //    uint8_t reserved = getub(bu, 7);          // TODO: all 1 for DST100UM

    if(speed_water != 0xffff) {
        session->gpsdata.navigation.set = NAV_STW_PSET;
        session->gpsdata.navigation.speed_thru_water = speed_water * 0.01;
        mask |= NAVIGATION_SET;
    } else {
        session->gpsdata.navigation.speed_thru_water = NAN;
    }

    if(speed_ground != 0xffff) {
        session->gpsdata.navigation.set = NAV_SOG_PSET;
        session->gpsdata.navigation.speed_over_ground = speed_ground * 0.01;
        mask |= NAVIGATION_SET;
    } else {
        session->gpsdata.navigation.speed_over_ground = NAN;
    }

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

    return mask;
}


/**
 *   \PGN 128267: NAV Water Depth
 */
static gps_mask_t hnd_128267(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);

    uint8_t sid = getub(bu, 0); // can be tied to 128259
    uint32_t depth = getleu32(bu, 1);  // 1x10-2m
    int16_t offset = getles16(bu, 5); // TODO: 0x7fff N/A, 1x10-3m
    //    uint8_t res = getub(bu, 7); // reserved, len in bits?


    if(depth == 0xffffffff) {
      session->gpsdata.navigation.depth = NAN;
    } else {
      session->gpsdata.navigation.depth = depth *.01;
      session->gpsdata.navigation.set = NAV_DPT_PSET;
    }

    if(offset == 0x7fff) {
      session->gpsdata.navigation.depth_offset = NAN;
    } else {
      session->gpsdata.navigation.depth_offset = offset * 0.001;
      session->gpsdata.navigation.set |= NAV_DPT_OFF_PSET;
    }

    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %d, depth = %f m, offset = %f m\n",
		sid,
	        session->gpsdata.navigation.depth,
		session->gpsdata.navigation.depth_offset);

    return (ONLINE_SET | NAVIGATION_SET);
}


/**
 *   \PGN 128275: NAV Distance Log
 */
static gps_mask_t hnd_128275(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

    session->gpsdata.navigation.distance_total = dist * METERS_TO_NM;
    session->gpsdata.navigation.set = NAV_DIST_TOT_PSET;

    session->gpsdata.navigation.distance_trip  = rest * METERS_TO_NM;
    session->gpsdata.navigation.set |= NAV_DIST_TRIP_PSET;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    char d[255]; char t[255];
    if(date != 0xffff)
      sprintf(d, "%u", date);
    else
      sprintf(d, "-");
    if(time != 0xffffffff)
      sprintf(t, "%u", time);
    else
      sprintf(t, "-");

    gpsd_report(session->context->debug, LOG_IO,
		"                   date = %s, time = %s, dist= %um, dist since reset = %um\n",
		d,
		t,
		dist,
		rest);

    return NAVIGATION_SET;
}


/**
 * \PGN 129283: NAV Cross Track Error
 */
static gps_mask_t hnd_129283(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /*
    1 SID
    2 XTE Mode
    3 Reserve
    4 Navigation Terminated
    5 XTE
    6 Reserved
  */

    gps_mask_t mask = 0, pset = 0;

    uint8_t  sid;
    uint8_t  mode;
    uint8_t  reserve;
    uint8_t  terminated;
    uint16_t reserved;

    sid          = getub(bu, 0);
    mode         = getub(bu, 1);           // TODO: bits ? meaning?
    reserve      = getub(bu, 1);           // TODO: bits ?
    terminated   = getub(bu, 1);           // TODO: bits ?

    mask |= setles32value(bu, 2, WAYPOINT_SET, pset | WPY_XTE_PSET, 0.01,
                  &session->gpsdata.waypoint.xte, &session->gpsdata.waypoint.set);

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
		(session->gpsdata.waypoint.set & WPY_XTE_PSET)?session->gpsdata.waypoint.xte:NAN,
		reserved);

    return mask;
}


/**
 * \todo PGN 129284: NAV Navigation Data
 * missing mask and storage structure
 */
static gps_mask_t hnd_129284(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

    gps_mask_t mask = 0, pset= 0;
    uint8_t sid;
    uint8_t course_bearing_ref;
    uint8_t perpendicular_crossed;
    uint8_t arrival_circle_entered;
    uint8_t calc_type;
    uint32_t eta_time;
    uint16_t eta_date;

    uint32_t org_wpt_number;
    uint32_t dest_wpt_number;
    int32_t dest_wpt_lat;
    int32_t dest_wpt_lon;

    char etas[JSON_DATE_MAX + 1];

    sid                     = bu[0];
    // lets assume 2 bit pattern based on bits toggled
    // guess work here
    course_bearing_ref      = (bu[5] >> 0) & 0x03; // 0= true, 1=magnetic

    // observed b0000 0100
    perpendicular_crossed   = (bu[5] >> 2) & 0x03; // b11 is unkown

    // observed b00010 000
    arrival_circle_entered  = (bu[5] >> 4) & 0x03; // b11 is unkown

    // only a guess
    calc_type               = (bu[5] >> 6) & 0x03; // b11 is unkown, 0=Great Circle,1=Rhumb Line

    if(arrival_circle_entered == 1) {
        session->gpsdata.waypoint.arrival_status = waypoint_circle_entered;
        session->gpsdata.waypoint.set  |= WPY_ARRIVAL_STATUS_PSET;
        mask |= WAYPOINT_SET;
    }
    if(perpendicular_crossed == 1) {
        session->gpsdata.waypoint.arrival_status |= waypoint_perpendicular_crossed;
        session->gpsdata.waypoint.set  |= WPY_ARRIVAL_STATUS_PSET;
        mask |= WAYPOINT_SET;
    }


    // haven't seen this, bytes are a guess/match to other time/date fields
    eta_time                = getleu32(bu,  6);
    eta_date                = getleu16(bu, 10);

    if((eta_date == 0xffff) && (eta_time == 0xffffffff)) {
        strcpy(etas, "-");
    } else {
        if(eta_date == 0xffff) {
            eta_date = 0;
        }
        if(eta_time == 0xffffffff) {
            eta_time = 0;
        }
        session->gpsdata.waypoint.eta = eta_date*24*60*60 + eta_time/1e4;
        session->gpsdata.waypoint.set  |= WPY_ETA_PSET;
        mask |= WAYPOINT_SET;
    }

    mask |= setleu32value(bu,  1, WAYPOINT_SET, pset | WPY_RANGE_TO_PSET, 0.01,
                  &session->gpsdata.waypoint.range_to_destination, &session->gpsdata.waypoint.set);

    mask |= setleu16value(bu, 12, WAYPOINT_SET, pset | WPY_BEARING_FROM_ORG_TO_PSET, 0.0001 * RAD_2_DEG,
                  &session->gpsdata.waypoint.bearing_from_org_to_destination, &session->gpsdata.waypoint.set);

    mask |= setleu16value(bu, 14, WAYPOINT_SET, pset | WPY_BEARING_FROM_POS_TO_PSET, 0.0001 * RAD_2_DEG,
                  &session->gpsdata.waypoint.bearing_from_pos_to_destination, &session->gpsdata.waypoint.set);

    org_wpt_number          = getleu32(bu, 16);        // TODO: 0xffffff for n/a
    dest_wpt_number         = getleu32(bu, 20);

    dest_wpt_lat            = getles32(bu, 24);
    dest_wpt_lon            = getles32(bu, 28);

    if((dest_wpt_lat != 0x7fffffff) && (dest_wpt_lon != 0x7fffffff)) {
        session->gpsdata.waypoint.latitude = dest_wpt_lat * 1e-7;
        session->gpsdata.waypoint.longitude = dest_wpt_lon * 1e-7;
        session->gpsdata.waypoint.set |= WPY_LATLON_TO_PSET;
        mask |= WAYPOINT_SET;
    }

    mask |= setles16value(bu, 32, WAYPOINT_SET, pset | WPY_SPEED_FROM_ORG_TO_PSET, 0.01,
                  &session->gpsdata.waypoint.speed_to_destination, &session->gpsdata.waypoint.set);

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    if(session->context->debug >= LOG_IO) {
        if((eta_date < 0xffff) || (eta_time < 0xffffffff))
            unix_to_iso8601(session->gpsdata.waypoint.eta, etas, sizeof(etas));


        gpsd_report(session->context->debug, LOG_IO,
            "                   SID= %u, Perpendicular Crossed= %u, Arrival Circle Entered= %u, Calculation Type= %u\n",
            sid,
            perpendicular_crossed,
            arrival_circle_entered,
            calc_type);

        gpsd_report(session->context->debug, LOG_IO,
            "                   ETA= %s\n", etas);

        gpsd_report(session->context->debug, LOG_IO,
            "                   Bearing, Org To Dest Wpt= %f, Bearing, Pos To Dest Wpt= %f, Course/Bearing Ref.= %u\n",
            (session->gpsdata.waypoint.set & WPY_BEARING_FROM_ORG_TO_PSET)?session->gpsdata.waypoint.bearing_from_org_to_destination:NAN,
            (session->gpsdata.waypoint.set & WPY_BEARING_FROM_POS_TO_PSET)?session->gpsdata.waypoint.bearing_from_pos_to_destination:NAN,
            (course_bearing_ref != 3)?course_bearing_ref:3);

        gpsd_report(session->context->debug, LOG_IO,
            "                   Org Wpt #= %u, Dest Wpt #= %u, Dest Wpt Lat= %f, Dest Wpt Lon= %f, Dist to Dest Wpt= %fm, Wpt Closing Velocity= %fm/s\n",
            org_wpt_number,
            dest_wpt_number,
            (session->gpsdata.waypoint.set & WPY_LATLON_TO_PSET)?session->gpsdata.waypoint.latitude:NAN,
            (session->gpsdata.waypoint.set & WPY_LATLON_TO_PSET)?session->gpsdata.waypoint.longitude:NAN,
            (session->gpsdata.waypoint.set & WPY_RANGE_TO_PSET)?session->gpsdata.waypoint.range_to_destination:NAN,
            (session->gpsdata.waypoint.set & WPY_SPEED_FROM_ORG_TO_PSET)?session->gpsdata.waypoint.speed_to_destination:NAN);
    }

    return(0);
}


/*
 *   PGN 129285: NAV Navigation - Route/WP Information
 */
static gps_mask_t hnd_129285(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

/**
 * \todo PGN 129291: Set & Drift, Rapid Update
 * missing everything
 */
static gps_mask_t hnd_129291(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

/**
 *   \PGN 130306: NAV Wind Data
 *   http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/
 */
static gps_mask_t hnd_130306(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

    According to

    http://www.maretron.com/support/manuals/USB100UM_1.6.pdf

    0 - 1 are translated to MWD and 2 - 4 to MWV:

    MWD,dir,T,dir,M,speed,N,speed,M
    MWV,dir,ref,speed,M,A

    Speed is in knots.

  */

    static char * wind_ref[] = {
      "True North",
      "Magnetic North",
      "Apparent",
      "True to Boat",
      "True to Water"
    };

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

    gps_mask_t pset = 0, mask = 0, pset_s = 0, pset_a = 0;
    double * speed = NULL;
    double * angle = NULL;

    uint8_t sid;
    uint8_t ref = 0xff;
    uint8_t res;

    sid          = getub(bu, 0);
    ref          = getub(bu, 5) & 0x07;         // see res 24 - 21 bits
    res          = getub(bu, 6);                // 21 bits ([19]), makes ref == 3 bits

    if(ref > 0x04)
        return 0;

    // accidentally ref and enum wind_reference are identical
    angle = &session->gpsdata.environment.wind[ref].angle;
    speed = &session->gpsdata.environment.wind[ref].speed;
    pset_s = speeds[ref];
    pset_a = angles[ref];

    mask |= setleu16value(bu, 1, ENVIRONMENT_SET, pset|pset_s, 0.01,
                    speed, &session->gpsdata.environment.set);
    mask |= setleu16value(bu, 3, ENVIRONMENT_SET, pset|pset_a, RAD_2_DEG * 0.0001,
                    angle, &session->gpsdata.environment.set);


    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
        "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
        "                   SID = %u, wind speed = %f m/s, dir = %f, ref = %s (%02x), res = %u\n",
        sid,
        (!isnan(session->gpsdata.environment.wind[ref].speed))?session->gpsdata.environment.wind[ref].speed:NAN,
        (!isnan(session->gpsdata.environment.wind[ref].angle))?session->gpsdata.environment.wind[ref].angle:NAN,
        (ref < 5) ? wind_ref[ref] : "-", ref, res);

    return ONLINE_SET | mask;
}


/**
 *   \PGN 130310: NAV Water Temp., Outside Air Temp., Atmospheric Pressure
 */
static gps_mask_t hnd_130310(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /*
    1 Sequence ID
    2 Water Temp
    3 Outside Ambient Air Temp.
    4 Atmospheric Pressure
    5 Reserved Bits
  */

    uint8_t sid;
    uint16_t pres;
    uint8_t res;

    gps_mask_t mask = 0, pset = 0;

    sid          = getub(bu, 0);
    pres         = getleu16(bu, 5);
    res          = getsb(bu, 7);

    #define PSI_2_BAR 1.0/14.50377

    // Kelvin
    mask |= setleu16value(bu, 1, ENVIRONMENT_SET, pset|ENV_TEMP_WATER_PSET, 0.01,
                    &session->gpsdata.environment.temp[temp_water], &session->gpsdata.environment.set);
    mask |= setleu16value(bu, 3, ENVIRONMENT_SET, pset|ENV_TEMP_AIR_PSET, 0.01,
                    &session->gpsdata.environment.temp[temp_air], &session->gpsdata.environment.set);



    if(pres != 0xffff) {
    }

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID = %u, water = %f C, outside = %f C, pressure = %f mbar, res = %u\n",
		sid,
		(!isnan(session->gpsdata.environment.temp[temp_water]))?session->gpsdata.environment.temp[temp_water] + KELVIN_2_CELSIUS:NAN,
        (!isnan(session->gpsdata.environment.temp[temp_air]))?session->gpsdata.environment.temp[temp_air] + KELVIN_2_CELSIUS:NAN,
		pres * PSI_2_BAR * 0.001,
		res);

    return mask;
}


/**
 *   \PGN 130311: NAV Environmental Parameters
 */
static gps_mask_t hnd_130311(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
  /*[2], [10]
    rework of 130310 and should be used for new designs
    1 Sequence ID
    2 Temperature Instance - 0x00 == sea, 0x01 == outside temperature
    3 Humidity Instance    - WSO100: 0x01 == outside humidity.
    4 Temperature          - outside air units of 0.01°K.
    5 Humidity             – relative humidity in units of 0.004%
    6 Atmospheric Pressure - units of 100Pa
  */

    gps_mask_t mask = 0, pset = 0;

    uint8_t sid;
    uint8_t temp_inst;
    uint8_t hum_inst;
    int16_t hum;
    int16_t pres;

    enum temp_reference_t tr;

    sid = getub(bu, 0);
    temp_inst     = getub(bu, 1) & (0x01 << 6);     // TODO: which temp instances
    hum_inst      = getub(bu, 1) & (0x01 << 2);     // TODO: which hum instances
    hum           = getleu16(bu, 4);  // TODO: s/u? 0x7fff for n/a?
    pres          = getleu16(bu, 6);  // TODO: s/u? 0xffff for n/a?

    if(temp_inst == 0x00) {
        tr = temp_water;
        mask |= setleu16value(bu, 2, ENVIRONMENT_SET, pset|ENV_TEMP_WATER_PSET, 0.01,
                        &session->gpsdata.environment.temp[temp_water], &session->gpsdata.environment.set);
    } else if(temp_inst == 0x01) {
        tr = temp_air;
        mask |= setleu16value(bu, 2, ENVIRONMENT_SET, pset|ENV_TEMP_AIR_PSET, 0.01,
                        &session->gpsdata.environment.temp[temp_air], &session->gpsdata.environment.set);
    } else
        return 0;

    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, temp inst= %d, hum inst= %d, temp = %f C, hum = %f%%, pres = %f\n",
		sid,
		temp_inst,
		hum_inst,
		(!isnan(session->gpsdata.environment.temp[temp_air]))?session->gpsdata.environment.temp[temp_air] + KELVIN_2_CELSIUS:NAN,
		hum / 0.004,
		pres * 0.0001);

    return mask;
}

/**
 * \todo  PGN 130312: NAV Temperature
 *   deprecated in NMEA 3.0
 *   missing storage and mask
 */
static gps_mask_t hnd_130312(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
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

    gps_mask_t mask = 0;

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

    if(0) {
        // TODO - not sure yet with instance/source/value
        session->gpsdata.environment.temp[temp_water] =  temp * 0.01 - 273.15;
        session->gpsdata.environment.set |= ENV_TEMP_WATER_PSET;
        mask |= ENVIRONMENT_SET;
    }

    gpsd_report(session->context->debug, LOG_IO,
		"                   SID= %u, temp inst = %u, temp src = %s, temp = %f, temp set = %f, res = %u\n",
		sid,
		temp_inst,
		(temp_src < 2) ? temp_src_list[temp_src] : " not listed yet ",
		temp * 0.01 - 273.15,
		temp_set * 0.01 - 273.15,
		reserve);

    return mask;
}

/**
 * \todo  PGN 130850: Simnet AP command
 */
static gps_mask_t hnd_130850(unsigned char *bu, int len UNUSED, struct PGN *pgn UNUSED, struct gps_device_t *session)
{
    uint16_t code   = getleu16(bu, 0);
    uint8_t  prop_id = getub(bu, 2);
                                        // ignoring 3, 4
    uint8_t  dev_id = getub(bu, 5);   // device
    uint16_t event  = getleu16(bu, 6);  // 1a = change direction
    uint8_t  dir_id = getub(bu, 8);   // direction
    uint16_t rad    = getleu16(bu, 9);  //  * RAD_2_DEG * 0.0001; // 0.0001 rads per second

    uint16_t man = (code >> 5);         // 11 bit manu
                                        //  2 bit - ignored
    uint16_t ind = (code >> 0) & 0x07; //  3 bit industry

    /*
    "Simnet: Event Command: AP command", 130850, false, 12, 0,
    { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
        , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
        , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
        , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=2", "AP command" }
        , { "B", BYTES(2), 1, false, 0, "" }
        , { "Controlling Device", BYTES(1), 1, false, 0, "" }
        , { "Event", BYTES(2), RES_LOOKUP, false, LOOKUP_SIMNET_AP_EVENTS, "" }
        , { "Direction", BYTES(1), RES_LOOKUP, false, LOOKUP_SIMNET_DIRECTION, "" }
        , { "Angle", BYTES(2), RES_RADIANS, false, "rad", "" }
        , { "G", BYTES(1), 1, false, 0, "" }
        , { 0 }
    }
    */

    gpsd_report(session->context->debug, LOG_IO,
                "                   manu = %u, ind= %u, prop= %u, dev= %u, ev= %u, dir= %u, deg= %.02f\n",
                man, ind, prop_id, dev_id, event, dir_id, rad*RAD_2_DEG * 0.0001);

    return 0;
}


/*
 *   Unknown PGN
 */
static gps_mask_t hnd_unknown(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session)
{
    print_data(session->context, bu, len, pgn);
    gpsd_report(session->context->debug, LOG_DATA,
		"pgn %6d(%3d): unkown\n", pgn->pgn, session->driver.nmea2000.unit);

    return 0;
}

#if 0
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
#endif

struct PGN *vyspi_find_pgn(uint32_t pgn) {

    struct PGN *work = NULL;
    int l1;

    l1 = 0;
    while (pgnlist[l1].pgn != 0) {
        if (pgnlist[l1].pgn == pgn) {
            work = &pgnlist[l1];
            break;
        } else {
            l1 = l1 + 1;
	    }
	}

    // should never be NULL, just catching last one which is unknown
    return work;
}

static void vyspi_reset_outbuffer(struct gps_packet_t *lexer) {

    uint16_t cnt = 0;
    lexer->out_count = 0;
    lexer->outbuflen = 0;

    for(cnt = 0; cnt < MAX_OUT_BUF_RECORDS; cnt++) {
        lexer->out_type[cnt] = 0;
        lexer->out_offset[cnt] = 0;
        lexer->out_len[cnt] = 0;
    }
}

static void vyspi_n_discard(struct gps_packet_t *lexer, uint8_t nchars)
/* shift the input buffer to discard all data up to current input pointer */
{
  size_t discard = nchars;
  if(discard > (size_t)(lexer->inbufptr - lexer->inbuffer)) {
    discard = (size_t)(lexer->inbufptr - lexer->inbuffer);
  }

  size_t remaining = (size_t)(lexer->inbuflen - discard);
  memmove(lexer->inbuffer, lexer->inbuffer + discard, remaining);
  lexer->inbufptr = lexer->inbufptr - discard;
  lexer->inbuflen = remaining;

  if (lexer->debug >= LOG_RAW+1) {
    char scratchbuf[MAX_PACKET_LENGTH*2+1];
    gpsd_report(lexer->debug, LOG_RAW+1,
		"Packet type %d discarded %lu chars remaining %lu = %s\n",
		lexer->type, discard, remaining,
		gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
				(char *)lexer->inbuffer,
				lexer->inbufptr - lexer->inbuffer));
  }
}

static void vyspi_packet_discard(struct gps_packet_t *lexer)
/* shift the input buffer to discard all data up to current input pointer */
{
    vyspi_n_discard(lexer, lexer->inbufptr - lexer->inbuffer);
}

static void vyspi_packet_accept(struct gps_packet_t *lexer, int packet_type)
/* packet grab succeeded, move to output buffer */
{

    uint16_t newoffset = 0;
    uint8_t cnt = lexer->out_count;

    size_t packetlen = lexer->inbufptr - lexer->inbuffer;

    // there is 2 bytes checksum
    if(lexer->frm_version) // they are not in the package len but in frm_length
        packetlen -= 2;
    assert(packetlen == lexer->frm_length);

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

    if(packetlen != lexer->frm_length) {
        char scratchbuf[MAX_PACKET_LENGTH*2+1];
        gpsd_report(lexer->debug, LOG_ERROR,
                    "Fatal error with packet length %zu != frame length %u\n%s\n",
                    packetlen, lexer->frm_length,
                    gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
                                    (char *)lexer->inbuffer,
                                    max(packetlen, lexer->frm_length)));
    }

    if(cnt > 0)
        newoffset = lexer->out_offset[cnt - 1] + lexer->out_len[cnt - 1];

    if (newoffset + packetlen < sizeof(lexer->outbuffer)) {

        memcpy(lexer->outbuffer + newoffset, lexer->inbuffer, packetlen);

        // we add a '\0' for all and also non-text packages
        lexer->outbuffer[newoffset + packetlen] = '\0';
        lexer->outbuflen += packetlen + 1;

        lexer->out_offset[cnt] = newoffset;
        lexer->out_new_version[cnt] = lexer->frm_version;
        lexer->out_len[cnt] = packetlen;

        lexer->out_type[cnt] = lexer->frm_type;
        if(lexer->frm_type == FRM_TYPE_NMEA0183)
            if(packet_type == AIVDM_PACKET)
                lexer->out_type[cnt] = FRM_TYPE_AIS;

        lexer->type = VYSPI_PACKET;
        lexer->out_count++;

        if (lexer->debug >= LOG_DATA) {
            char scratchbuf[MAX_PACKET_LENGTH*2+1];
            gpsd_report(lexer->debug, LOG_DATA, // LOG_RAW+1,
                        "vy-packet no %u type %d with frame type %u accepted %zu = %s\n",
                        cnt, packet_type, lexer->out_type[cnt], packetlen,
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


enum
{
#include "packet_states.h"
};

static int vyspi_packet_parse(struct gps_packet_t *lexer, unsigned char c) {

    char *state_table[] = {
#include "packet_names.h"
    };

    nextstate(lexer, c);
    gpsd_report(lexer->debug, LOG_RAW + 2,
                "%08ld: character '%c' [%02x], new state: %s\n",
                lexer->char_counter, (isprint(c) ? c : '.'), c,
                state_table[lexer->state]);

    lexer->char_counter++;

    if (lexer->state == GROUND_STATE) {
        character_skip(lexer);
    } else if (lexer->state == COMMENT_RECOGNIZED) {
        lexer->state = GROUND_STATE;
        return COMMENT_PACKET;
    }

#ifdef NMEA_ENABLE
    else if (lexer->state == NMEA_RECOGNIZED) {
        /*
         * $PASHR packets have no checksum. Avoid the possibility
         * that random garbage might make it look like they do.
         */
        if (strncmp((const char *)lexer->inbuffer, "$PASHR,", 7) != 0) {
            bool checksum_ok = true;
            char csum[3] = { '0', '0', '0' };
            char *end;
            /*
             * Back up past any whitespace.  Need to do this because
             * at least one GPS (the Firefly 1a) emits \r\r\n
             */
            for (end = (char *)lexer->inbufptr - 1; isspace(*end); end--)
                continue;
            while (strchr("0123456789ABCDEF", *end))
                --end;
            if (*end == '*') {
                unsigned int n, crc = 0;
                for (n = 1; (char *)lexer->inbuffer + n < end; n++)
                    crc ^= lexer->inbuffer[n];
                (void)snprintf(csum, sizeof(csum), "%02X", crc);
                checksum_ok = (csum[0] == toupper(end[1])
                               && csum[1] == toupper(end[2]));
            }
            if (!checksum_ok) {
                char scratchbuf[MAX_PACKET_LENGTH*2+1];
                gpsd_report(lexer->debug, LOG_WARN,
                            "NMEA packet accepted = %s\n",
                            gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
                                            (char *)lexer->outbuffer,
                                            lexer->outbuflen));
                gpsd_report(lexer->debug, LOG_WARN,
                            "bad checksum in NMEA packet; expected %s.\n",
                            csum);
                lexer->state = GROUND_STATE;
                return BAD_PACKET;
            }
        }

        /* checksum passed or not present */
#ifdef AIVDM_ENABLE
        if (strncmp((char *)lexer->inbuffer, "!AIVDM", 6) == 0)
            return AIVDM_PACKET;
        else if (strncmp((char *)lexer->inbuffer, "!AIVDO", 6) == 0)
            return AIVDM_PACKET;
        else if (strncmp((char *)lexer->inbuffer, "!BSVDM", 6) == 0)
            return AIVDM_PACKET;
        else if (strncmp((char *)lexer->inbuffer, "!BSVDO", 6) == 0)
            return AIVDM_PACKET;
        else
#endif /* AIVDM_ENABLE */
            return NMEA_PACKET;

    }
#endif /* NMEA_ENABLE */

    return -2;
}


static void vyspi_preparse_serial(struct gps_device_t *session) {

    static char * type_names [] = {
        "COMMAND", "NMEA0183", "NMEA2000", "SEATALK", "AIS"
    };

    struct gps_packet_t *lexer = &session->packet;
    int packet_type = -2;

    gpsd_report(session->context->debug, LOG_RAW + 1,
                "VYSPI: preparse serial called with input len = %lu and ptr at %lu\n",
                lexer->inbuflen, lexer->inbufptr - lexer->inbuffer);
    // one extra for reading both, len and type/origin

    vyspi_reset_outbuffer(lexer);

    while(packet_buffered_input(lexer)) {

        uint8_t * bp = lexer->inbufptr; // need this for storing back
        uint8_t b = *lexer->inbufptr++;
        
        gpsd_report(session->context->debug, LOG_RAW + 1,
                    "VYSPI: preparse serial [%c] %02x @ %p state= %u\n",
                    (isprint(b) ? b : '.'), b, lexer->inbufptr, lexer->frm_state);

        if(b == 0x7d) {
            lexer->frm_7dflag = 1;
            character_skip(lexer);
            continue;
        }

        // an unchanged 0x7e is always a frame start, escaped or not
        if(b == 0x7e) {
            vyspi_packet_discard(lexer);
            lexer->frm_length  = 0;
            lexer->frm_read    = 0;
            lexer->frm_version = 0;
            lexer->frm_7dflag  = 0;

            lexer->frm_act_checksum    = 0;
            lexer->frm_shall_checksum  = 0;

            lexer->frm_state   = FRM_TYPE;
            continue;
        }

        if(lexer->frm_state != FRM_CS)
            lexer->frm_act_checksum ^= b;

        if(lexer->frm_7dflag) {
            lexer->frm_7dflag = 0;
            b ^= (1 << 5);
            *bp = b;
        }

        switch(lexer->frm_state) {
        case FRM_TYPE:
            // new version marker set?
            if(b&0x80) {
                b = b & 0x7f;
                lexer->frm_version = 1;
            }
            if(b < FRM_TYPE_MAX) {
                lexer->frm_type  = b;
                lexer->frm_length = 0;
                if(lexer->frm_version)
                    lexer->frm_state = FRM_RESERVED;
                else
                    lexer->frm_state = FRM_LEN;
            } else {
                lexer->frm_length = 0;
                lexer->frm_version = 0;
                lexer->frm_state = FRM_GND;
                vyspi_packet_discard(lexer);
            }
            break;

        case FRM_RESERVED:
            lexer->frm_state = FRM_PORT;
            lexer->frm_reserved = b;
            break;

        case FRM_PORT:
            lexer->frm_port = b;
            lexer->frm_state = FRM_LEN;
            break;

        case FRM_LEN:
            // if MSB is set we have a 2 byte len (well, 15 bit)
            // we store in big endian

            if(lexer->frm_length > 0) {
                // if MSB is set in len already then we are in second byte
                lexer->frm_state = FRM_START;

                // add low byte
                lexer->frm_length |= (b << 7);
                lexer->frm_offset = 3;
                if(lexer->frm_version)
                    lexer->frm_offset += 2;
                vyspi_n_discard(lexer, lexer->frm_offset);

            } else {
                // if its not set in len, then this is low byte and maybe only byte
                lexer->frm_length = b & 0x7f;
                if(!(b & 0x80)) {
                    // even the last byte and only byte
                    lexer->frm_state = FRM_START;
                    lexer->frm_offset = 2;
                    if(lexer->frm_version)
                        lexer->frm_offset += 2;
                    vyspi_n_discard(lexer, lexer->frm_offset);
                }
            }

            break;

        case FRM_END:
            // odd if we got here
            lexer->frm_length = 0;
            lexer->frm_version = 0;
            vyspi_packet_discard(lexer);
            break;

        case FRM_START:

            /* trying to avoid a new buffer and changing existing code;
               thus parsing inline here */
            if(lexer->frm_type == FRM_TYPE_NMEA0183) {
                lexer->type = VYSPI_PACKET;
                packet_type = vyspi_packet_parse(lexer, b);
                if(packet_type > -2) {
                    gpsd_report(session->context->debug, LOG_RAW,
                                "VYSPI: preparse serial discovered a packet type = %d\n",
                                packet_type);

                }
            }

            lexer->frm_read++;

            /* we may take inbufptr and inbuffer as a criteria here since
             * we are always having the recent frame start at inbuffer
             * even if there is multiple frames in the buffer
             */
            if((size_t)(lexer->inbufptr - lexer->inbuffer) >= lexer->frm_length) {
                // frame is complete
                gpsd_report(session->context->debug, LOG_RAW,
                            "VYSPI: preparse serial discovered complete frame with len %u >= %lu\n",
                            lexer->frm_length, lexer->inbufptr - lexer->inbuffer);
                if(lexer->frm_version) {
                    lexer->frm_read= 0;
                    lexer->frm_state = FRM_CS;
                }
                else
                    lexer->frm_state = FRM_END;

            }
            break;

        case FRM_CS:
            lexer->frm_shall_checksum = b << (8 * lexer->frm_read);
            if(lexer->frm_read > 0) {
                lexer->frm_state = FRM_END;
            }
            lexer->frm_read++;
            break;

        default:
            // FRM_GND where we do nothing but wait for 0x7e
            break;
        }

/*
        if(((lexer->frm_type == FRM_TYPE_NMEA0183)
           || (lexer->frm_type == FRM_TYPE_AIS)) && (packet_type > -2)) {
            vyspi_packet_accept(lexer, packet_type);
            vyspi_packet_discard(lexer);
            break;
        }
*/

        if(lexer->frm_state == FRM_END) {

            gpsd_report(session->context->debug, LOG_RAW,
                        "VYSPI: preparse serial complete frame type %s version %u with len %u, %lu\n",
                        type_names[lexer->frm_type],
                        lexer->frm_version,
                        lexer->frm_length,
                        lexer->inbufptr - lexer->inbuffer);


            if((lexer->frm_type == FRM_TYPE_NMEA0183)
               || (lexer->frm_type == FRM_TYPE_AIS) ) {
                /* we parsed packet inline during FRM_START
                   - just discard here from inbuffer */
                vyspi_packet_accept(lexer, VYSPI_PACKET);
                vyspi_packet_discard(lexer);

            } else if((lexer->frm_type == FRM_TYPE_NMEA2000)
                      || (lexer->frm_type == FRM_TYPE_ST)
                      || (lexer->frm_type == FRM_TYPE_CMD)) {

                // accept the NMEA 2000 or Seatalk 1 packet
                vyspi_packet_accept(lexer, VYSPI_PACKET);
                vyspi_packet_discard(lexer);
            }

            /* TODO - this break prevents that multiple sentences that are all
                read in one read() will be processed at once - which might or
                might not be desirable */
            break;
        }
    }
}

static void vyspi_preparse_spi(struct gps_device_t *session) {

  static char * typeNames [] = {
    "UNKOWN", "NMEA0183", "NMEA2000"
  };

  struct gps_packet_t *lexer = &session->packet;

  size_t packetlen = vyspi_packetlen(lexer);

  gpsd_report(session->context->debug, LOG_DATA,
	      "VYSPI: preparse called with packet len = %lu\n", packetlen);

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

      if((size_t)lexer->inbuflen < (size_t)(lexer->inbufptr - lexer->inbuffer) + pkgLen + 8) {
          gpsd_report(session->context->debug, LOG_WARN, "VYSPI: exit prematurely: %ld + 8 + %d > %lu\n",
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

      struct PGN *work;
      work = vyspi_find_pgn(session->driver.vyspi.last_pgn);

      session->driver.nmea2000.workpgn = (void *) work;

      vyspi_packet_discard(lexer);

      break;

    } else if (pkgType == PKG_TYPE_NMEA0183) {

        if(lexer->inbuflen < (unsigned int)(lexer->inbufptr - lexer->inbuffer + pkgLen)) {
          gpsd_report(session->context->debug, LOG_WARN, "VYSPI: exit prematurely: %ld + %d > %lu\n",
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

  ssize_t          status = 0;

  errno = 0;

  // we still need to process old package before getching a new

  /*
  #FIXME - issues seen so far:

    1. if there is data to read every time we come here and also packet_buffered_input is 0 then
        we will hang forever in the loop scanning fragments.
    2. if we abort the fragment loop after each full packet then data is queing up and never processed
       - mostly because we are waiting for FD_SET to become active again before we get here again
       - new data is being read instead and only one package processed

    Best is probably to see here

    - if its the first fragment and read data then
    - return the amount of data still in the queue so that we can process it in the fragment loop

  #TODO - potential bug if this is fragment > 0 and last fragment was read and completely parsed
  with no bytes remaining in input buffer - but no "signal given" to read more
  this will in this situation wrongly read more data
  */

  if(!packet_buffered_input(pkg)) {

      status = read(fd, pkg->inbuffer + pkg->inbuflen,
                    sizeof(pkg->inbuffer) - (pkg->inbuflen));

      gpsd_report(session->context->debug, LOG_IO,
                  "VYSPI reading from device with status %zd\n", status);

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
      } else {
          if (session->context->debug >= LOG_IO) {
              char scratchbuf[MAX_PACKET_LENGTH*2+1];
              gpsd_report(session->context->debug, LOG_IO,
                          "Read %zd chars to buffer offset %zd (total %zd): %s\n",
                          status, pkg->inbuflen, pkg->inbuflen + status,
                          gpsd_packetdump(scratchbuf, sizeof(scratchbuf),
                                          (char *)pkg->inbuffer + pkg->inbuflen, (size_t)status));
          }
      }

      if(status <= 0) {
          gpsd_report(session->context->debug, LOG_WARN,
                      "VYSPI: exit with len in bytes= %lu, errno= %d\n",
                      status, errno);
          return 0;
      }

      if(session->gpsdata.dev.isSerial) {
          pkg->inbuflen += status;
      } else {
          // is SPI
          pkg->inbuflen = status;
          pkg->inbufptr = pkg->inbuffer;
      }
  } else {
      gpsd_report(session->context->debug, LOG_DATA,
                  "not reading new data - processing queue with %lu bytes remaining\n",
                  packet_buffered_input(pkg));

      if (session->context->debug >= LOG_DATA) {
          char scratchbuf[MAX_PACKET_LENGTH*2+1];
          gpsd_report(session->context->debug, LOG_DATA, // LOG_RAW+1,
                      "bytes remaining: %s\n",
                      gpsd_packetdump(scratchbuf,  sizeof(scratchbuf),
                                      (char *)pkg->inbuffer, packet_buffered_input(pkg)));
      }
  } // if(!packet_buffered_input(pkg))

  if(session->gpsdata.dev.isSerial) {
      vyspi_preparse_serial(session);
  } else {
      vyspi_preparse_spi(session);
  }

  if (pkg->outbuflen > 0) {
      if ((session->driver.nmea2000.workpgn == NULL)
          && (session->packet.type == NMEA2000_PACKET)) {
          gpsd_report(session->context->debug, LOG_DATA,
                      "VYSPI: exit with 0 with with no known PGN in N2k\n");
          return 0;
      }

      gpsd_report(session->context->debug, LOG_RAW,
        "VYSPI: exit with outbuf len = %lu and %lu bytes remaining\n",
        pkg->outbuflen,
        packet_buffered_input(pkg));
      return (ssize_t)pkg->outbuflen;
  } else {
      /*
       * Otherwise recvd is the size of whatever packet fragment we got.
       * It can still be 0 or -1 at this point even if buffer data
       * was consumed.
       */
      gpsd_report(session->context->debug, LOG_RAW,
        "VYSPI: exit with outbuf len = 0 and %lu bytes read and %lu bytes remaining\n",
                  status, packet_buffered_input(pkg));
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
static gps_mask_t vyspi_parse_serial_input(struct gps_device_t *session)
{
  gps_mask_t mask = 0;
  struct gps_packet_t * lexer = &session->packet;
  uint8_t ct = 0;

  struct PGN *work = NULL;

  static char * typeNames [] = {
      "COMMAND", "NMEA0183", "NMEA2000", "SEATALK", "AIS", "UNKOWN"
  };

  gpsd_report(session->context->debug, LOG_RAW,
              "VYSPI: parse_input called with packet len = %lu and %u frames\n",
              lexer->outbuflen, lexer->out_count);


  for(ct = 0; ct < lexer->out_count; ct++) {

      gpsd_report(session->context->debug, LOG_DATA, "VYSPI: type= %s, len= %u\n",
                  (lexer->out_type[ct] < FRM_TYPE_MAX)
                  ? typeNames[lexer->out_type[ct]] : typeNames[FRM_TYPE_MAX],
                  lexer->out_len[ct]);

      if(lexer->out_type[ct] == FRM_TYPE_NMEA2000) {

          uint8_t offset = 4;

          // new version NMEA 2000 has 7 byte offset
          if(lexer->out_new_version[ct])
              offset = 7;

          if(offset > lexer->out_len[ct]) {
              gpsd_report(session->context->debug, LOG_WARN,
                          "VYSPI: exit prematurely: %u > %lu\n",
                          offset, lexer->outbuflen);
              return 0;
          }

          session->driver.vyspi.last_pgn =
              getleu32(lexer->outbuffer, lexer->out_offset[ct] + 0);

          if(lexer->out_new_version[ct]){

              // this info must be available when parsing
              // the actual data in the PGN specific functions

              session->driver.vyspi.prio = getub(lexer->outbuffer, lexer->out_offset[ct] + 4);
              session->driver.vyspi.src = getub(lexer->outbuffer, lexer->out_offset[ct] + 5);
              session->driver.vyspi.dest = getub(lexer->outbuffer, lexer->out_offset[ct] + 6);
              gpsd_report(session->context->debug, LOG_DATA,
                          "VYSPI: version 2 PGN = %u, prio= %u, src= %u, dest=%u\n",
                          session->driver.vyspi.last_pgn,
                          session->driver.vyspi.prio,
                          session->driver.vyspi.src,
                          session->driver.vyspi.dest);

              session->gpsdata.src_addr_seen[session->driver.vyspi.src] = 1;

          } else {
              gpsd_report(session->context->debug, LOG_DATA,
                          "VYSPI: version 1 PGN = %u\n",
                          session->driver.vyspi.last_pgn);
          }

          work = vyspi_find_pgn( session->driver.vyspi.last_pgn );

          if (work != NULL) {

              unsigned char * b =
                  session->packet.outbuffer + offset + lexer->out_offset[ct];

              mask |= (work->func)(b, lexer->out_len[ct] - offset, work, session);

          } else {
              gpsd_report(session->context->debug, LOG_ERROR,
                          "VYSPI: no work PGN found for pgn = %u\n",
                          session->driver.vyspi.last_pgn);
          }

      } else if (lexer->out_type[ct] == FRM_TYPE_NMEA0183) {

          gpsd_report(session->context->debug, LOG_IO, "<= GPS: %s\n",
                      lexer->outbuffer + lexer->out_offset[ct]);

          mask |= nmea_parse_len((char *)lexer->outbuffer + lexer->out_offset[ct],
                                 lexer->out_len[ct],
                                 session);

      } else if (lexer->out_type[ct] == FRM_TYPE_AIS) {

          // TODO - handle multiple AIS sentences in one sentence
          if (aivdm_decode
              ((char *)session->packet.outbuffer + lexer->out_offset[ct],
               lexer->out_len[ct],
               session, &session->gpsdata.ais,
               session->context->debug)) {
              mask = ONLINE_SET | AIS_SET;
          } else
              mask = ONLINE_SET;

      } else if (lexer->out_type[ct] == FRM_TYPE_ST) {

          gpsd_report(session->context->debug, LOG_RAW,
                      "VYSPI: Seatalk len= %u (or %lu)\n",
                      lexer->out_len[ct], lexer->outbuflen);

          mask |= process_seatalk(session->packet.outbuffer + lexer->out_offset[ct],
                                  lexer->out_len[ct], session);

      } else if (lexer->out_type[ct] == FRM_TYPE_CMD) {

          if(memcmp(session->packet.outbuffer + lexer->out_offset[ct], "stat", 4) == 0) {
              gpsd_report(session->context->debug, LOG_DATA, "DATA with STATS\n");
              if(memcmp(session->packet.outbuffer + lexer->out_offset[ct] + 4, "n2k", 3) == 0) {
                  uint32_t error_count = getleu32(session->packet.outbuffer, lexer->out_offset[ct] + 7);
                  uint32_t packet_count = getleu32(session->packet.outbuffer, lexer->out_offset[ct] + 11);
                  uint32_t frame_count = getleu32(session->packet.outbuffer, lexer->out_offset[ct] + 15);
                  gpsd_report(session->context->debug, LOG_DATA,
                              "DATA with N2K: packets= %u, frames= %u, errors= %u\n",
                              packet_count, frame_count, error_count);
              }
          } else {
                  gpsd_report(session->context->debug, LOG_ERROR, "UNKOWN CMD: %s len= %u\n",
                              session->packet.outbuffer + lexer->out_offset[ct], lexer->out_len[ct]);
          }

      } else {

          gpsd_report(session->context->debug, LOG_ERROR, "UNKOWN: len= %u\n",

                      lexer->out_len[ct]);

      }
  }

  // calling it here - just to make sure we are not missing a beat
  // vyspi_handle_time_trigger(session);

  return mask;
}

void vyspi_handle_time_trigger(struct gps_device_t *session)
{
    if(!session->driver.nmea2000.enable_writing) {
        // nothing to do here currently if we do not want to enablle writing
        gpsd_report(session->context->debug, LOG_RAW,
                    "N2K not marked for write handling. Ignoring startup sequence.\n");
        return;
    }

    // are we actually ready to start the n2k node?
    if(session->gpsdata.dev.node_state == node_init) {

        if((session->gpsdata.dev.protocol_version == 0)
           && session->packet.frm_version) {

            // first of all - we saw version > 0 protocol, lets switch
            session->gpsdata.dev.protocol_version = session->packet.frm_version;

            if(session->driver.nmea2000.own_src_id < 0xff) {

                // if we have a configured source addr lets use that and claim directly

                gpsd_report(session->context->debug, LOG_INF,
                            "NMEA 2000 node starting. Claiming stored source address 0x%02x.\n",
                            session->driver.nmea2000.own_src_id);

                vyspi_claim_our_source_addr(session);

                // no need for node starting phase
                session->gpsdata.dev.node_state = node_ready;

            } else {

                gpsd_report(session->context->debug, LOG_INF,
                            "NMEA 2000 node starting. No pre-configured source address. Making address claim call.\n");

                // waiting for incoming answers now
                session->gpsdata.dev.node_state = node_starting;
                tu_gettime(&session->gpsdata.dev.node_state_time);

                // make a poll call to all other devices
                vyspi_addr_claim_call(session);
            }

        }

    } else if(session->gpsdata.dev.node_state == node_starting) {

        // give it 2 seconds to collect other node's addresses
        if(tu_get_millis_since(&session->gpsdata.dev.node_state_time) > 2000) {

            gpsd_report(session->context->debug, LOG_INF,
                        "NMEA 2000 node waited 2 sec for others to report. Claiming source address now.\n");

            vyspi_claim_free_source_addr(session);

            session->gpsdata.dev.node_state = node_ready;
        }
    }
}
/*@+mustfreeonly@*/

static gps_mask_t vyspi_parse_spi_input(struct gps_device_t *session)
{
  gps_mask_t mask = 0;
  struct gps_packet_t * pkg = &session->packet;

  struct PGN *work = NULL;

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

  return mask;
}
/*@+mustfreeonly@*/

static gps_mask_t vyspi_parse_input(struct gps_device_t *session) {
  if(session->gpsdata.dev.isSerial) {
    return vyspi_parse_serial_input(session);
  } else {
    return vyspi_parse_spi_input(session);
  }
}

/*@+nullassign@*/
static void vyspi_set_serial(struct gps_device_t *session, speed_t speed) {

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
  /* we do not want to block at all as read() will be called after
     select() with following reads (fragments > 0) which might not
     deliver data.

     Not ideal but currently read() returning 0 bytes is used to terminate
     the fragment poll loop in gpsd_multipoll().
  */
  // session->ttyset.c_cc[VMIN]      =   1;                  // read doesn't block (non canonical)
  // session->ttyset.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

  /* Set Baud Rate */
  if(cfsetospeed (&session->ttyset, (speed_t)speed) < 0) {
    gpsd_report(session->context->debug, LOG_ERROR,
		"Failed to set new speed %d: %s\n", errno, strerror(errno));
  }
  if(cfsetispeed (&session->ttyset, (speed_t)speed) < 0) {
    gpsd_report(session->context->debug, LOG_ERROR,
		"Failed to set new speed %d: %s\n", errno, strerror(errno));
  }

  /* Setting other Port Stuff */
  session->ttyset.c_cflag     &=  ~CSIZE;

  session->ttyset.c_cflag     &=  ~CRTSCTS;       // no flow control

  session->ttyset.c_cflag     |=  CS8;
  session->ttyset.c_cflag     &=  ~CSTOPB;       // 1 stop

  session->ttyset.c_cflag     |= CLOCAL;     //

  session->ttyset.c_cflag     |= CREAD;     // turn on READ


  // no ignoring of something
  session->ttyset.c_iflag     &=  ~IGNBRK;
  session->ttyset.c_iflag     &=  ~BRKINT;

  // no parity check, no marking of parity errors
  session->ttyset.c_iflag     |=   IGNPAR;
  session->ttyset.c_iflag     &=  ~PARMRK;
  session->ttyset.c_iflag     &=  ~INPCK;

  session->ttyset.c_cflag     &=  ~PARENB;        // Make 8n1
  session->ttyset.c_cflag     &=  ~PARODD;        // Make 8n1

  // no clearing of high bit
  session->ttyset.c_iflag     &=  ~ISTRIP;

  // non canonical, we want every char directly, no signals
  session->ttyset.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG);

  // no conversions or ignoring any cr or nl or conversion between them
  session->ttyset.c_iflag     &=  ~(INLCR | IGNCR | ICRNL | IXANY | IXON | IMAXBEL);
  session->ttyset.c_oflag     &=  ~(OCRNL | OFDEL | OFILL | ONLCR | ONLRET | ONOCR);

  session->ttyset.c_lflag     &=  ~(OPOST);

#ifndef __APPLE__
  // these are just missing from header files (man termios mentions them though)
  session->ttyset.c_iflag     &=  ~(IUCLC);
  session->ttyset.c_oflag     &=  ~(OLCUC);
  session->ttyset.c_lflag     &=  ~(XCASE);
#endif

  // Flush Port, then applies attributes
  tcflush( session->gpsdata.gps_fd, TCIFLUSH );

  if ( tcsetattr ( session->gpsdata.gps_fd, TCSANOW, &session->ttyset ) != 0) {
    gpsd_report(session->context->debug, LOG_ERROR,
		"SEATALK tcsetattr error %d: %s\n", errno, strerror(errno));
    session->gpsdata.gps_fd = -1;
    return;
  }

}

int vy_port_list_read(struct gps_device_t *session, struct devconfig_t * dev) {

    int i, j, status = 0;
    int port_speed_matched = 0;

    for(i = 0; i < dev->port_count; i++) {

        struct device_port_t * port = &dev->portlist[i];

        if(strcmp(port->type_str, "nmea2000") == 0) {
            continue;
        }

        if(strcmp(port->type_str, "seatalk") == 0) {

            port->type = PORT_TYPE_SEATALK;

        } else if(strcmp(dev->portlist[i].type_str, "nmea0183") == 0) {

            port->type = PORT_TYPE_NMEA0183;

        } else {

            gpsd_report(session->context->debug, LOG_ERROR,
                        "Unkown or illegal port type '%s'\n", port->type_str);
            return -1;
        }

        if(port->type == PORT_TYPE_SEATALK) {
            if((port->speed != 0) && (port->speed != 4800)) {
                gpsd_report(session->context->debug, LOG_WARN,
                            "Ignoring odd port speed for seatalk!\n");
            }
        } else if(port->type == PORT_TYPE_NMEA0183) {

            for(j = 0; j < NITEMS(vy_port_speeds); j++) {
                if(port->speed == vy_port_speeds[j]) {
                    port_speed_matched = 1;
                    break;
                }
            }

            if(!port_speed_matched) {
                gpsd_report(session->context->debug, LOG_ERROR,
                            "NMEA0183 requires legal port speed %d!\n", port->speed);
                return -1;
            }
        }

        gpsd_report(session->context->debug, LOG_INF,
                    "port %d: %s @ %d baud\n",
                    port->no,
                    port->type_str,
                    port->type == PORT_TYPE_SEATALK?4800:port->speed);
    }

    if (status != 0) {
        puts(json_error_string(status));
        return -1;
    }

    return 0;
}

int vy_port2cmd(struct device_port_t * vy, uint8_t *cmd) {

  memcpy(cmd, "stty", 4);

  cmd[5] = vy->type;
  cmd[4] = vy->no;

  set8leu32(cmd, vy->speed, 6);

  return 0;
}


ssize_t vyspi_write_with_protocol(struct gps_device_t *session,
                                  enum frm_type_t frm_type,
                                  const uint8_t *buf,
                                  const size_t len,
                                  const uint8_t protocol_version)
/* pass low-level data to devices straight through */
{
    gpsd_report(session->context->debug, LOG_INF,
                "vyspi_write: %s (%s) ports= %d\n",
                buf, session->gpsdata.dev.path, session->gpsdata.dev.port_count);

    uint8_t frm[255];
    if(len == 0)
        return 0;

    size_t frmlen = frm_toHDLC8(frm, 255, frm_type, protocol_version, buf, len);
    gpsd_serial_write(session, (const char *)frm, frmlen);

    session->driver.vyspi.bytes_written_frm[frm_type] += frmlen;
    session->driver.vyspi.bytes_written_raw[frm_type] += len;

    if(session->context->debug >= LOG_IO) {

        int i = 0;
        
        struct timespec now;
        tu_gettime(&now);

        uint32_t nowms = tu_get_time_in_milli(&now);
        uint32_t diff = nowms - session->driver.vyspi.bytes_written_last_ms;
        double rate = 1000.0*((double)(frmlen))/((double)diff);
        
        gpsd_report(session->context->debug, LOG_IO,
                    "Wrote %f bytes/s (%0.2fkBit/s) as %lu bytes in %u ms\n",
                    rate, rate*8.0/1024.0,
                    frmlen, diff);
        session->driver.vyspi.bytes_written_last_ms = nowms;
        
        if(session->driver.vyspi.bytes_written_last_sec + 1000 < nowms) {
            const char * ftn[5] = { "CMD", "183", "N2K", "ST ", "AIS"};

            for(i = 0; i < 5; i++) {
                double rate_f = 1000.0*((double)(session->driver.vyspi.bytes_written_frm[i]))/
                    ((double)(nowms - session->driver.vyspi.bytes_written_last_sec));
                double rate_r = 1000.0*((double)(session->driver.vyspi.bytes_written_raw[i]))/
                    ((double)(nowms - session->driver.vyspi.bytes_written_last_sec));
                gpsd_report(session->context->debug, LOG_IO,
                            "%s   %0.2f (%0.2f) kBit/s with %u (%u) bytes in %u ms\n",
                            ftn[i],
                            rate_f*8.0/1024.0, rate_r*8.0/1024.0,
                            session->driver.vyspi.bytes_written_frm[i],
                            session->driver.vyspi.bytes_written_raw[i],
                            (nowms - session->driver.vyspi.bytes_written_last_sec));
                
                session->driver.vyspi.bytes_written_frm[i] = 0;
                session->driver.vyspi.bytes_written_raw[i] = 0;
            }
            gpsd_report(session->context->debug, LOG_IO,
                        "    last= %u ms, now= %u ms\n",
                        session->driver.vyspi.bytes_written_last_sec, nowms);
            session->driver.vyspi.bytes_written_last_sec = nowms;
        }
    }
    
    return len;
}

ssize_t vyspi_write(struct gps_device_t *session,
                    enum frm_type_t frm_type,
                    const uint8_t *buf,
                    const size_t len)
/* pass low-level data to devices straight through */
{
    return vyspi_write_with_protocol(session, frm_type, buf, len,
                              session->gpsdata.dev.protocol_version);
}

#ifndef S_SPLINT_S

int vyspi_init(struct gps_device_t *session) {

    int i = 0;

    gpsd_report(session->context->debug, LOG_INF,
                    "initializing configuration for device '%s'.\n",
		session->gpsdata.dev.path);

    if(vy_port_list_read(session, &session->gpsdata.dev) != 0) {

        gpsd_report(session->context->debug, LOG_ERROR,
                    "Error reading port configuration. Assuming defaults.\n");
        return 1;
    }

    for(i = 0; i < 5; i++) {
        session->driver.vyspi.bytes_written_frm[i] = 0; /* collecting stats per type */ 
        session->driver.vyspi.bytes_written_raw[i] = 0; /* net amount of data w/o frm overhead */
    }
    session->driver.vyspi.bytes_written_last_ms = 0;
    session->driver.vyspi.bytes_written_last_sec = 0;

    uint8_t cmd[255];
    size_t len = 0;

    for (i = 0; i < session->gpsdata.dev.port_count; i++) {

        vy_port2cmd(&session->gpsdata.dev.portlist[i], cmd);

        gpsd_report(session->context->debug, LOG_INF,
                    "setting port configuration for port '%d'.\n",
                    session->gpsdata.dev.portlist[i].no);

        vyspi_write(session, FRM_TYPE_CMD, cmd, 10);
    }

    // send start command to stm32

    gpsd_report(session->context->debug, LOG_INF,
                "Sending start command.\n");

    if(session->driver.nmea2000.enable_writing) {
        gpsd_report(session->context->debug, LOG_INF,
                "NMEA 2000 - enable writing - requesting new frame protocol.\n");
        memcpy(cmd, "vers", 4);
        cmd[4] = 2;
        vyspi_write(session, FRM_TYPE_CMD, cmd, 5);
    }

    memcpy(cmd, "n2kl", 4);
    cmd[4] = 4;

    set8leu32(cmd, 129038, 5);
    set8leu32(cmd, 129039, 9);
    set8leu32(cmd, 129040, 13);
    set8leu32(cmd, 129029, 17);

    vyspi_write(session, FRM_TYPE_CMD, cmd, 21);

    memcpy(cmd, "n2kl", 4);
    cmd[4] = 3;

    set8leu32(cmd, 129284, 5);
    set8leu32(cmd, 130934, 9);  // unkown
    set8leu32(cmd, 129041, 13);  // ATON
    set8leu32(cmd, 130850, 17);  // Simnet AP Command

    vyspi_write(session, FRM_TYPE_CMD, cmd, 17);

    memcpy(cmd, "strt", 4);
    vyspi_write(session, FRM_TYPE_CMD, cmd, 4);

    /*
      memcpy(cmd, "stty", 4);
      cmd[4] = 0; 
      cmd[5] = PORT_TYPE_HOST;
      set8leu32(cmd, 921600, 6);
      vyspi_write(session, FRM_TYPE_CMD, cmd, 10);

      vyspi_set_serial(session, (speed_t)B921600);
    */

    return 0;
}

/*
  opens either a

  1. tcp/ip connection <host>:<port>
  2. SPI /dev/vyspi0.0
  3. Serial /dev/ttyXX:<config>
 */
int vyspi_open(struct gps_device_t *session) {

  char path[strlen(session->gpsdata.dev.path) + 1], *port;

  socket_t dsock;

  // path starts with 8 characters: "vyspi://"
  (void)strlcpy(path, session->gpsdata.dev.path + 8, sizeof(path));

  // invalidate
  INVALIDATE_SOCKET(session->gpsdata.gps_fd);

  // this is port in case of TCP/IP or detailed port configuration for serial

  gpsd_report(session->context->debug, LOG_INF,
		"Device path = %s.\n", path);

  // SPI or serial start with "/dev"
  if(path[0] == '/') {

    port = strchr(path, '?');
    if(port != NULL)
      *port++ = '\0';

    gpsd_report(session->context->debug, LOG_INF,
		"Assuming SPI or serial device %s.\n", path);

    if ((dsock = open(path, O_RDWR | O_NOCTTY)) == -1) {
      gpsd_report(session->context->debug, LOG_ERROR,
		  "read-only device open failed: %s\n",
		  strerror(errno));
      return -1;
    }

    session->gpsdata.gps_fd = dsock;
    gpsd_report(session->context->debug, LOG_INF,
		"Device %s opened with sock = %d.\n", path, dsock);

    // ugly hack:
    if(strncmp(path, "/dev/vyspi", 10) == 0) {

        // assuming SPI
        if(session->gpsdata.gps_fd) {
            // usefull to send a reset to the device
            ioctl(dsock, VYSPI_RESET, NULL);
        }
        session->gpsdata.dev.isSerial = 0;
        gpsd_report(session->context->debug, LOG_INF,
                    "Opened %s as SPI device.\n", path);

    } else {

        gpsd_report(session->context->debug, LOG_INF,
                    "opening serial feed at %s.\n", path);
        session->gpsdata.dev.isSerial = 1;
        vyspi_set_serial(session, (speed_t)B115200);

        /* fetch additional port parameter to configure uC using classical URL:
           ?{"ports":{[...]}*/
    }


  } else {

    // assuming TCP/IP

      if ((port = strchr(path, ':')) == NULL) {
          port = DEFAULT_GPSD_PORT;
      } else
          *port++ = '\0';

      gpsd_report(session->context->debug, LOG_INF,
                  "opening UDP VYSPI feed at %s, port %s.\n", path, port);

      if ((session->gpsdata.gps_fd = netlib_connectsock(AF_UNSPEC, path, port, "udp")) < 0) {
          gpsd_report(session->context->debug, LOG_ERROR, "UDP device open error %s.\n",
                      netlib_errstr(session->gpsdata.gps_fd));
          return -1;
      } else
          gpsd_report(session->context->debug, LOG_SPIN,
                      "TCP device opened on fd %d\n", session->gpsdata.gps_fd);
      session->gpsdata.dev.isSerial = 1;
  }

  gpsd_switch_driver(session, "VYSPI");
  session->sourcetype = source_can;
  session->servicetype = service_sensor;

  struct timespec now;
  tu_gettime(&now);

  last_error_stats_report = tu_get_time_in_milli(&now);

  return session->gpsdata.gps_fd;
}

#endif /* of ifndef S_SPLINT_S */

/*@-mustdefine@*/
const char /*@ observer @*/ *gpsd_vyspidump(struct gps_device_t *device) {

  char *scbuf = device->msgbuf;
  size_t scbuflen = sizeof(device->msgbuf);

  char *binbuf = (char *)device->packet.outbuffer;
  size_t binbuflen = device->packet.outbuflen;

  /*
  gpsd_report(session->context->debug, LOG_SPIN,
	      "VYSPI: gpsd_vyspidump %u entered with len = %ld\n",
	      device->driver.vyspi.last_pgn, binbuflen);
  */

  size_t j = 0;
  size_t i = 0;
  int cnt = 0;

  size_t maxlen =
    (size_t) ((binbuflen >
	       MAX_PACKET_LENGTH) ? MAX_PACKET_LENGTH : binbuflen);

  if (NULL == binbuf || 0 == binbuflen) {
    scbuf[0] = '\0';
    return scbuf;
  }

    /*
    lexer->out_offset[cnt] = newoffset;
    lexer->out_new_version[cnt] = lexer->frm_version;
    lexer->out_len[cnt] = packetlen;
    */

    struct timespec now;
    tu_gettime(&now);

    uint32_t nowms = tu_get_time_in_milli(&now);
    const char * ibuf = (const char *)&binbuf[0];

    for(cnt = 0; cnt < device->packet.out_count; cnt++) {
        char tmp[255];
        sprintf(tmp, "[%.4f]: %d,%u,%u,%u,%u,%d,",
                nowms/1000.0,
                cnt,
                device->packet.out_type[cnt],
                device->driver.vyspi.last_pgn,
                0, 0,
                device->packet.out_len[cnt]);
        j = 0;

        for (i = 0; i < strlen(tmp) && j < scbuflen; i++) {
            scbuf[j++] = tmp[i];
        }

        if (device->packet.out_type[cnt] == FRM_TYPE_CMD) {

            if(memcmp(&ibuf[device->packet.out_offset[cnt]], "stat", 4) == 0) {
                if(memcmp(&ibuf[device->packet.out_offset[cnt] + 4], "n2k", 3) == 0) {
                    uint32_t error_count = getleu32(&ibuf[device->packet.out_offset[cnt]], 7);
                    uint32_t packet_count = getleu32(&ibuf[device->packet.out_offset[cnt]], 11);
                    uint32_t frame_count = getleu32(&ibuf[device->packet.out_offset[cnt]], 15);
                    snprintf(&scbuf[j], scbuflen - j, "packets= %u, frames= %u, errors= %u\n",
                                packet_count, frame_count, error_count);
                }
                else if(memcmp(&ibuf[device->packet.out_offset[cnt] + 4], "n2e", 3) == 0) {
                    uint16_t overrun_count = getleu16(&ibuf[device->packet.out_offset[cnt]], 7);
                    uint32_t cancel_count = getleu32(&ibuf[device->packet.out_offset[cnt]], 9);
                    snprintf(&scbuf[j], scbuflen - j, "overrun= %u, cancel= %u\n",
                                overrun_count, cancel_count);
                }
            }

        } else {

            const char *hexchar = "0123456789abcdef";

            for (i = 0; j < scbuflen - 2
                     && i + device->packet.out_offset[cnt] < maxlen
                     && i < device->packet.out_len[cnt]; i++) {

                scbuf[j++] = hexchar[(ibuf[i + device->packet.out_offset[cnt]] & 0xf0) >> 4];
                scbuf[j++] = hexchar[ibuf[i + device->packet.out_offset[cnt]] & 0x0f];
            }
            j--; // back last ',' to overwrite with '\0'
            scbuf[j] = '\0';
        }
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
