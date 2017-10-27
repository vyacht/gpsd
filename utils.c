#include <stdint.h>

#include "utils.h"

void set8leu8(uint8_t * bu, uint8_t value, int offset) {
    bu[offset + 0] = value;
}

void set8leu16(uint8_t * bu, uint16_t value, int offset) {
    bu[offset + 0] = value;
    bu[offset + 1] = value >>  8;
}

void set8leu24(uint8_t * bu, uint32_t value, int offset) {
    set8leu16(bu, (uint16_t)(value & 0xffff), offset);
    bu[offset + 2] = (value >>  16) & 0x00ff;
}

void set8leu32(uint8_t * bu, uint32_t value, int offset) {
    set8leu16(bu, (uint16_t)(value & 0xffff), offset);
    set8leu16(bu, (uint16_t)((value >> 16) & 0xffff), offset + 2);
}

void set8leu64(uint8_t * bu, uint64_t value, int offset) {
    set8leu32(bu, (uint32_t)(value & 0xffffffff), offset);
    set8leu32(bu, (uint32_t)((value >> 32) & 0xffffffff), offset + 4);
}

void set8les32(uint8_t * bu, int32_t value, int offset) {
    bu[offset + 0] = value;    
    bu[offset + 1] = value >>  8;    
    bu[offset + 2] = value >> 16;    
    bu[offset + 3] = value >> 24;    
}

void set16leu32(uint16_t * bu, uint32_t value, int offset) {
  bu[offset + 0] = 0x00ff & value;    
  bu[offset + 1] = 0x00ff & value >>  8;    
  bu[offset + 2] = 0x00ff & value >> 16;    
  bu[offset + 3] = 0x00ff & value >> 24;    
}

void set8les8(uint8_t * bu, int8_t value, int offset) {
    bu[offset + 0] = 0xff & value;
}

void set8les16(uint8_t * bu, int16_t value, int offset) {
  bu[offset + 0] = 0xff & value;    
  bu[offset + 1] = 0xff & value >>  8;    
}

void set8les64(uint8_t * bu, int64_t value, int offset) {
    set8les32(bu, (int32_t)(value & 0xffffffff), offset);
    set8les32(bu, (int32_t)((value >> 32) & 0xffffffff), offset + 4);
}

uint32_t get8leu32(uint8_t * bu, int offset) {
  uint32_t value = 
        (bu[offset + 0])
      | (bu[offset + 1] << 8)
      | (bu[offset + 2] << 16)
      | (bu[offset + 3] << 24);

  return value;
}
