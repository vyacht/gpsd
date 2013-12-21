/*
 * Combined NMEA 0183 & NMEA 2000 over SPI.
 *
 * The entry points for driver_vyspi
 *
 */

#ifndef _DRIVER_VYSPI_H_
#define _DRIVER_VYSPI_H_

#if defined(VYSPI_ENABLE)

int vyspi_open(struct gps_device_t *session);

#endif /* of defined(VYSPI_ENABLE) */

#endif /* of ifndef _DRIVER_VYSPI_H_ */
