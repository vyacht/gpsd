#ifndef _NAVIGATION_H_
#define  _NAVIGATION_H_

void
nav_init(struct gps_device_t *device);
void
nav_print_speed_over_ground(struct gps_device_t *session);
gps_mask_t
nav_set_speed_over_ground_in_knots(double value, struct gps_device_t *session);
gps_mask_t
nav_set_speed_through_water_in_knots(double value, struct gps_device_t *session);

#endif //  _NAVIGATION_H_
