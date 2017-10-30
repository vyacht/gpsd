/*
 * The entry points for driver_seatalk
 *
 */

#ifndef _DRIVER_SEATALK_H_
#define _DRIVER_SEATALK_H_

ssize_t seatalk_packet_get(struct gps_device_t *session);
gps_mask_t seatalk_parse_input(struct gps_device_t *session);

int seatalk_open(struct gps_device_t *session);

#endif /* of ifndef _DRIVER_SEATALK_H_ */
