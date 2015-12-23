#ifndef _FRM_H_
#define _FRM_H_

/**
  HDLC

  0x7e marks a frame start
  0x7d marks an escape, escaped char will have 5th bit toggled (1 << 5)

  A frame has the following fields
  
  0      0x7e
  1      payload type
  2[,3]  1 to 2 byte payload length (not frame len)
  3,4..N payload

  Payload length is encoded as follows:

  - big endian
  - bit 7 (MSB) of byte 1 is 1 if 2 bytes

  This means: 

  - all payload > 127 bytes require 2 byte payload length

  - frame byte 2 is payload len low byte
  - frame byte 3 can be payload len high byte

  Advantages (even over permanent 2 byte encoding):

  - optionally can always set bit 7 MSB to 1 and have byte 2 of len = 0x00
  - easier to have in statemachine without needing another state

 */

// frame state
enum {
  FRM_GND       = 0, // ground and initial state, waiting for 7e
  FRM_START     = 1, // we are entering payload
  FRM_TYPE      = 2, // type of payload
  FRM_LEN       = 3, // length of payload without escape char
  FRM_END       = 4  // marks the end of a complete frame
};

// frame types
enum {
  FRM_TYPE_CMD      = 0, 
  FRM_TYPE_NMEA0183 = 1, 
  FRM_TYPE_NMEA2000 = 2, 
  FRM_TYPE_ST       = 3,
  FRM_TYPE_AIS      = 4,
  FRM_TYPE_MAX      = 5
};

#define FRM_BUFFER_SIZE 512

typedef struct __frmBuffer {

  uint8_t frm7d;   // last char was escape
  uint8_t state;   // frame state
  uint8_t type;    // payload type

  uint16_t len;    // length of payload without esc chars

  uint8_t data[FRM_BUFFER_SIZE];
  uint8_t * ptr;   // ptr to the currently added char of payload

} frmBuffer_t;

/**
 init and reset buffer
 */
void frm_init(frmBuffer_t * frmBuffer);

/**
 process a single character, statemachine
 */
int frm_put(frmBuffer_t * frmBuffer, uint8_t c);

/**
   from to HDLC in 8 bit

   destlen is max destination buffer len
   srclen is payload length (<= max src len)

   returns the total length of the frame
 */
uint16_t frm_toHDLC8(uint8_t * dest, 
		     uint16_t destlen,
		     uint8_t frameType,
		     uint8_t * src, 
		     uint16_t srclen);

/**
   from to HDLC in 16 bit

   destlen is max destination buffer len
   srclen is payload length (<= max src len)

   returns the total length of the frame
 */
uint16_t frm_toHDLC16(uint16_t * dest, 
		      uint16_t destlen,
		      uint8_t frameType,
		      uint8_t * src, 
		      uint16_t srclen);


#endif // _FRM_H_
