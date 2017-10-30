#ifndef _SIGNAL_K_
#define _SIGNAL_K_

gps_mask_t signalk_track_dump(const struct gps_device_t *device, uint32_t startAfter, const char field[],
                             /*@out@*/ char reply[], size_t replylen);

gps_mask_t signalk_full_dump(const struct gps_device_t *device,
                          const struct vessel_t * vessel,
                          /*@out@*/ char reply[], size_t replylen);

#endif // _SIGNAL_K_
