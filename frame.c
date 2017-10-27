#include <stdint.h>
#include <stdio.h>

#include "frame.h"

void frm_init(frmBuffer_t * frmBuffer) {
  frmBuffer->ptr    = frmBuffer->data;
  frmBuffer->len    = 0;
  frmBuffer->frm7d  = 0;
  frmBuffer->state  = FRM_GND;
  frmBuffer->new_version = 0;
  frmBuffer->act_checksum = 0;
  frmBuffer->shall_checksum = 0;
  frmBuffer->read = 0;
}


/**
   to HDLC using a 16 bit destination buffer

   destlen is max destination buffer len
   srclen is payload length (<= max src len)

   returns the total length of the frame
 */
uint16_t frm_toHDLC16(uint16_t * dest, 
		      uint16_t destlen,
		      uint8_t frameType,
		      uint8_t * src, 
		      uint16_t srclen) {

  if(destlen < 4)
    return 0;

  uint16_t q = 1;

  // mark frame start
  dest[0] = 0x007e;    
  dest[1] = frameType;

  // do we require two bytes for length?
  if(srclen & 0xff80) {

    dest[2] = 0x00ff & ((0xff & srclen) | 0x80);
    dest[3] = 0x00ff & ((0xff80 & srclen) >> 7);
    q = 4;

  } else {

    dest[2] = 0x00ff & srclen;
    q = 3;

  }

  uint16_t i = 0;
  for(i = 0; i < srclen; i++) {

    if(destlen < q)
      return 0;

    // do we need escaping?
    if(((src[i] & 0xff) == 0x7d) || ((src[i] & 0xff) == 0x7e)) {

      // leave headroom for extra escape
      if(destlen < q + 1)
	return 0;

      dest[q] = 0x007d; 
      q++; 
      // flip bit 5
      dest[q] = 0x00ff & (src[i] ^ (1 << 5));    
    } else 
      dest[q] = 0x00ff & src[i];    
    q++;
  }
  
  return q;
}

void frm_addByte(uint8_t * dest, // destination buffer
                     uint16_t * q,   // position
                     uint8_t b) {    // byte to set 

    if(((b & 0xff) == 0x7d) || ((b & 0xff) == 0x7e)) {
      dest[*q] = 0x007d; 
      (*q)++; 
      // flip bit 5
      dest[*q] = 0x00ff & (b ^ (1 << 5));    
    } else 
      dest[*q] = 0x00ff & b;    
    (*q)++; 
}

/**
   to HDLC using a 8 bit

   destlen is max destination buffer len
   srclen is payload length (<= max src len)

   returns the total length of the frame
 */
uint16_t frm_toHDLC8(uint8_t * dest, 
		     uint16_t destlen,
		     uint8_t frameType,
		     uint8_t frameVersion,
		     const uint8_t * src, 
		     const uint16_t srclen) {

    uint16_t q = 1;
    uint16_t checksum = 0;

    // mark frame start
    dest[0] = 0x7e;    
    dest[1] = frameType;
    q= 2;

    if(frameVersion > 0) {
        dest[1] |= 0x80; // frame type and the new version marker
        dest[2] = 0;
        dest[3] = 0; // port
        q= 4;
    }

    if(srclen & 0x80) {

        frm_addByte(dest, &q, (0xff & srclen) | 0x80);
        frm_addByte(dest, &q, ((0xff80 & srclen) >> 7));

    } else {

        frm_addByte(dest, &q, (0xff & srclen));
    }

    // payload
    uint16_t i = 0;
    for(i = 0; i < srclen; i++) {

        // leave headroom for extra escape
        if(destlen < q + 1)
            return 0;

        frm_addByte(dest, &q, src[i]);
    }

    if(frameVersion > 0) {

        uint16_t i = 0;

        if(destlen < q + 2)
            return 0;
        for(i = 1; i < q; i++) 
            if(dest[i] != 0x7d)
                checksum ^= dest[i]; 

        frm_addByte(dest, &q, checksum & 0x00ff);
        frm_addByte(dest, &q, (checksum >> 8) & 0x00ff);
    }

    return q;
}

// handles a single character received from host
int frm_put(frmBuffer_t * frm, uint8_t c) {

    uint8_t b = c;

    if(b == 0x7d) {
        frm->frm7d = 1;
        return 0;
    }

    // an unchanged 0x7e is always frame start, escape char or not
    if(b == 0x7e) {
        frm_init(frm);
        frm->state = FRM_TYPE;
        return 0;
    }

    // checksum is calculated without esc char but on the escaped value
    if(frm->state != FRM_CS)
        frm->act_checksum ^= b;

    if(frm->frm7d) {
        frm->frm7d = 0;
        b ^= (1 << 5);
    }

    switch(frm->state) {
    case FRM_TYPE:
        // new version marker set?
        if(b&0x80) {
            b = b & 0x7f;
            frm->new_version = 1;
        }
        if(b < FRM_TYPE_MAX) {
            frm->type  = b;
            frm->state = FRM_LEN;
            if(frm->new_version)
                frm->state = FRM_RESERVED;
            else
                frm->state = FRM_LEN;
        } else {
            frm_init(frm);
        }
        break;


    case FRM_RESERVED:
        frm->state = FRM_PORT;
        frm->reserved = b;
        break;
        
    case FRM_PORT:
        frm->port = b;
        frm->state = FRM_LEN;
        break;

    case FRM_LEN:
        // if MSB is set we have a 2 byte len (well, 15 bit)
        // we store in big endian

        if(frm->len > 0) {
            // if MSB is set in len already then we are in second byte
            frm->state = FRM_START;
            // printf("frm->len > 0\n");
            // add low byte
            frm->len |= (b << 7);
            // printf("b = %02x\n", b);
            
        } else {
            // if its not set in len, then this is low byte and maybe only byte
            frm->len = b & 0x7f;
            // printf("frm->len <= 0 %02x\n", frm->len);
            if(!(b & 0x80)) {
                // even the last byte and only byte
                frm->state = FRM_START;
                // printf("FRM_START\n");
            }
        }

        break;

    case FRM_END:
        frm_init(frm);
        break;

    case FRM_START:
        
        *frm->ptr++ = b;
        frm->read++;

        if((frm->ptr - frm->data) >= FRM_BUFFER_SIZE - 1) {
            frm_init(frm);
            return 0;
        }
        if((frm->ptr - frm->data) >= frm->len) {
            // frame is complete
            if(frm->new_version) {
                frm->read= 0;
                frm->state = FRM_CS;
            }
            else {
                frm->state = FRM_END;
                return 1;
            }
        }
        break;

    case FRM_CS:
        frm->shall_checksum |= b << (8 * frm->read);
        if(frm->read > 0) {
            frm->state = FRM_END;
            return 1;
        }
        frm->read++;
        break;


    default:
        // FRM_GND where we do nothing but wait for 0x7e
        break;
    }

    return 0;
}

