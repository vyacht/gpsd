#include <stdint.h>

#include "utils.h"

void set8leu32(uint8_t * bu, uint32_t value, int offset) {
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

uint32_t get8leu32(uint8_t * bu, int offset) {
  uint32_t value = 
        (bu[offset + 0])
      | (bu[offset + 1] << 8)
      | (bu[offset + 2] << 16)
      | (bu[offset + 3] << 24);

  return value;
}
