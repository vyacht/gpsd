#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "frame.h"


frmBuffer_t frmBuffer;

void testFrame(uint8_t * frm, uint16_t len) {

    uint8_t c = 0, r = 0, c1 = 0;
    uint8_t i = 0, j = 0;

    frm_init(&frmBuffer);

    for(i = 0; i < len; i++) {
        c = frm[i];
        r = frm_put(&frmBuffer, c);
        printf("%c %d %d %d %d %lu\n", 
               isprint(c) ? c : '.', 
               frmBuffer.state, frmBuffer.type,
               frmBuffer.len, r,
               frmBuffer.ptr - frmBuffer.data);
        if(r) {
            uint8_t buf[255];
            memset(buf, 1, 255);
            memcpy(buf, frmBuffer.data, frmBuffer.ptr - frmBuffer.data);
            buf[frmBuffer.ptr - frmBuffer.data] = 0;
            if(r == 1) {
                printf("tot frm len = %u\n", frmBuffer.len);
                printf("act cs      = 0x%04x\n", frmBuffer.act_checksum);
                printf("shall cs    = 0x%04x\n", frmBuffer.shall_checksum);

                printf("frm complete >");
                for(j = 0; j < frmBuffer.len; j++) { 
                    c1 = (isprint(buf[j]) ? buf[j] : '.');
                    c1 = (buf[j] == 0) ? '_' : c1;
                    printf("%c", c1);
                }
                printf("<\n");
            }
        }
    }

    printf("\n");
}

void print_frame(uint8_t * frm, uint16_t len) {
    uint16_t i = 0, j= 0;
    uint8_t c;

    while(i < len) {
      
      printf("% 3d  ", j+i);
      while((j < 8) && (j + i < len)) {
          c = frm[j+i];
          printf("0x%02x ", c);
          j++;
      }
      printf("   "); 
      j= 0;
      while((j < 8) && (j + i < len)) {
          c = frm[j+i];
          printf("%c", isprint(c) ? c : '.');
          j++;
      }
      j= 0;
      printf("\n");
      i += 8;
  }
}

int main(int argc, char * argv[]) {

  uint8_t gf1[] = "____this is a co}Mplete frame}~\n";
  uint8_t gf2[] = "___this is a co}Mplete frame}~\n";


  gf1[2] = 0x80 | (uint8_t)(sizeof(gf1) - 4 - 4 - 1); // 1 escape char
  gf1[3] = 0x00;
  gf1[0] = 0x7e;
  gf1[1] = FRM_TYPE_CMD;

  // escaping a tilde
  gf2[23] = 0x7d;
  gf2[24] = 0x7e ^ (1 << 5);

  gf2[2] = ~0x80 & (uint8_t)(sizeof(gf2) - 3 - 4 - 2); // 2 escape char
  gf2[0] = 0x7e;
  gf2[1] = FRM_TYPE_CMD;

  uint8_t frm[255];
  memcpy(frm, gf1, sizeof(gf1));
  memcpy(frm + sizeof(gf1), gf2, sizeof(gf2));

  testFrame(frm, 255);

  // testing 249 long frame (> 127)
  uint8_t tmp[255];

  memset(tmp, 0, 255);
  memcpy(tmp, gf1, sizeof(gf1));

  uint16_t tl = 241;
  uint16_t totfrmlen = 249;

  uint16_t i = 0, j = 0, c= 0;
  for(i = sizeof(gf1); i < 255; i++) {
    tmp[i] = 'x';
  }

  printf("\nNEXT\n\n");

  // version 0 frame
  totfrmlen = frm_toHDLC8(frm, 255, FRM_TYPE_CMD, 0, tmp, 241);
  print_frame(frm, 255);
  printf("\n\n\n");

  totfrmlen = frm_toHDLC8(frm, 255, FRM_TYPE_CMD, 1, tmp, 241);
  print_frame(frm, 255);

/*  if(tl & 0x80) {

    frm[2] = (0xff & tl) | 0x80;
    frm[3] = ((0xff80 & tl) >> 7);

  } else {

    frm[2] = 0xff & tl;

  }
*/

  testFrame(frm, 255);


  return 0;

}
