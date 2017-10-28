#include <stdio.h>
#include <stdint.h>

#include "utils.h"

int main(int argc, char * argv[]) {

    uint8_t bu[8];
    uint8_t i = 0;

    uint64_t val64;
    uint32_t val32;

    uint8_t *p = (uint8_t *)&val64;

    for(i = 0; i < 8; i++) {
        *p = i; p++;
    }

    p = (uint8_t *)&val32;
    for(i = 0; i < 4; i++) {
        *p = i; p++;
    }
    
    set8leu64(bu, val64, 0);

    for(i = 0; i < 8; i++) 
        printf("0x%02x ", bu[i]);
    printf("\n");
    for(i = 0; i < 8; i++) {
        printf("0x%02x ", (uint8_t)(val64 & 0xff)); 
        val64 >>= 8;
    }
    printf("\n");
    printf("\n");
    
    set8leu32(bu, val32, 0);

    for(i = 0; i < 8; i++) 
        printf("0x%02x ", bu[i]);
    printf("\n");
    for(i = 0; i < 4; i++) {
        printf("0x%02x ", (uint8_t)(val32 & 0xff)); 
        val32 >>= 8;
    }
    printf("\n");
    
    return 0;
}
