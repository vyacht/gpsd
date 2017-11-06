/*
 * Combined NMEA 0183 & NMEA 2000 over SPI.
 *
 * The entry points for driver_vyspi
 *
 */

#ifndef _DRIVER_VYSPI_H_
#define _DRIVER_VYSPI_H_

#if defined(VYSPI_ENABLE)

#include "frame.h"

int vyspi_open(struct gps_device_t *session);
int vyspi_init(struct gps_device_t *session);

gps_mask_t vyspi_parse_input(struct gps_device_t *session);
ssize_t vyspi_get(struct gps_device_t *session);
ssize_t vyspi_preparse(struct gps_device_t *session);

extern void vyspi_handle_time_trigger(struct gps_device_t *session);

const char *gpsd_vyspidump(struct gps_device_t *);
ssize_t vyspi_write(struct gps_device_t *,
                    enum frm_type_t,
                    const uint8_t *,
                    const size_t);

extern void vyspi_packet_accept(struct gps_packet_t *lexer, int packet_type);                    

struct PGN {
    uint32_t  pgn;
    uint8_t  fast;
    uint8_t  type;
    uint8_t  receive;
    uint8_t transmit;
    gps_mask_t    (* func)(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
    const char    *name;
};

struct PGN * vyspi_find_pgn(uint32_t pgn);

#endif /* of defined(VYSPI_ENABLE) */

#endif /* of ifndef _DRIVER_VYSPI_H_ */
