#ifndef _UTILS_H_
#define _UTILS_H_

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02
#define PKG_TYPE_ST       0x04
#define PKG_TYPE_BAD      0x08 // mark bad packets

void set8leu32(uint8_t * bu, uint32_t value, int offset);
void set16leu32(uint16_t * bu, uint32_t value, int offset);

uint32_t get8leu32(uint8_t * bu, int offset);

#endif
