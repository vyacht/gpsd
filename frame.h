#ifndef _FRM_H_
#define _FRM_H_

/**
  HDLC

  0x7e marks a frame start
  0x7d marks an escape, escaped char will have 5th bit toggled (1 << 5)

  A frame version 1 has the following fields
  
  0      0x7e
  1      payload type
  2[,3]  1 to 2 byte payload length (not frame len)
  3,4..N payload

  0183 payload is just the sentence
  2000 is first 4 bytes PGN and then data (fast transitions to 1 packet)

  A frame version 2 has the following fields
  
  0       0x7e
  1       payload frame type & 0x80 to mark new version 
  2       reserved for e.g. version or e.g. markers 
  3       source port when sending to host, dest port when sent from host
  4[,5]   1 to 2 byte payload length N (not frame len)
  6..N  payload
  N+1,N+2 checksum XOR of bytes 1 .. N un-escaped

  0183 payload is the sentence
  2000 is 4 bytes PGN, prio, source, dest 
       and then data (fast transitions to 1 packet).

       to host (router) source is device sending
       from host source is host id sending

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

# ifdef __cplusplus
extern "C" {
# endif

// frame state
enum {
  FRM_GND       = 0, // ground and initial state, waiting for 7e
  FRM_START     = 1, // we are entering payload
  FRM_TYPE      = 2, // type of payload
  FRM_RESERVED  = 3, // reserved byte
  FRM_PORT      = 4, // port byte
  FRM_LEN       = 5, // length of payload without escape char
  FRM_CS        = 6, // we are entering the checksum part
  FRM_END       = 7  // marks the end of a complete frame
};

// frame types
enum frm_type_t {
  FRM_TYPE_CMD      = 0, 
  FRM_TYPE_NMEA0183 = 1, 
  FRM_TYPE_NMEA2000 = 2, 
  FRM_TYPE_ST       = 3,
  FRM_TYPE_AIS      = 4,
  FRM_TYPE_MAX      = 5
};

#define FRM_BUFFER_SIZE 512

typedef struct __frmBuffer {

    uint8_t frm7d;        // last char was escape
    uint8_t new_version;  // new version marker? (0 or 1)
    uint8_t reserved;     // reserved (byte 2)
    uint8_t port;         // source port when send to host, dest port when from host
    uint16_t act_checksum;    // xor checksum 
    uint16_t shall_checksum;    // xor checksum 
    uint8_t state;        // frame state
    uint8_t type;         // payload type
    
    uint16_t len;         // length of payload without esc chars
    uint16_t read;        // payload or checksum len actually read 

    uint8_t data[FRM_BUFFER_SIZE];
    uint8_t * ptr;        // ptr to the currently added char of payload

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
		     uint8_t frameVersion,
		     const uint8_t * src, 
		     const uint16_t srclen);

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

# ifdef __cplusplus
}
# endif

#endif // _FRM_H_
