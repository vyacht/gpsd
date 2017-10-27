#ifndef _UTILS_H_
#define _UTILS_H_

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02
#define PKG_TYPE_ST       0x04
#define PKG_TYPE_BAD      0x08 // mark bad packets

void set8leu8(uint8_t * bu, uint8_t value, int offset);
void set8leu16(uint8_t * bu, uint16_t value, int offset);
void set8leu24(uint8_t * bu, uint32_t value, int offset);
void set8leu32(uint8_t * bu, uint32_t value, int offset);
void set8leu64(uint8_t * bu, uint64_t value, int offset);

void set16leu32(uint16_t * bu, uint32_t value, int offset);

void set8les8(uint8_t * bu, int8_t value, int offset);
void set8les16(uint8_t * bu, int16_t value, int offset);
void set8les32(uint8_t * bu, int32_t value, int offset);
void set8les64(uint8_t * bu, int64_t value, int offset);

uint32_t get8leu32(uint8_t * bu, int offset);

#endif
