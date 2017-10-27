#ifndef _PSEUDON2K_H_
#define _PSEUDON2K_H_

void n2k_binary_126992_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127250_dump(struct gps_device_t *session, uint32_t *pgn,
                            enum compass_t compass,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127251_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127258_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127259_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_hdg_magnetic_dump(struct gps_device_t *session, uint32_t *pgn,
                                  uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_hdg_true_dump(struct gps_device_t *session, uint32_t *pgn,
                              uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_attitude_dump(struct gps_device_t *session,uint32_t *pgn,
                              uint8_t bu[], size_t len, uint16_t * outlen);


void n2k_binary_128267_dump(struct gps_device_t *session, uint32_t *pgn,
                           uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_128275_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);


void n2k_binary_129025_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129026_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129029_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129283_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129539_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129540_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_129038_dump(struct gps_device_t *session, uint32_t *pgn,
                  uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_130306_dump(struct gps_device_t *session, enum wind_reference_t, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127488_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_127489_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len, uint16_t * outlen);

void n2k_binary_129033_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen);

void n2k_binary_129284_dump(struct gps_device_t *session, uint32_t *pgn,
                            uint8_t bu[], size_t len UNUSED, uint16_t * outlen);

void n2k_binary_130311_dump(struct gps_device_t *session, uint32_t *pgn,
    uint8_t bu[], size_t len UNUSED, uint16_t * outlen);

void n2k_binary_130312_dump(struct gps_device_t *session, uint32_t *pgn,
    uint8_t bu[], size_t len UNUSED, uint16_t * outlen);

int n2k_binary_dump(gps_mask_t changed,
                    struct gps_device_t *session,
                    void (*write_handler)(struct gps_device_t *, enum frm_type_t, const char *buf, size_t len));

#endif
