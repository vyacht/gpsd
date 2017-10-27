#ifndef _NMEA2000_H_
#define _NMEA2000_H_

#include <stdint.h>

// fast transmissions have max 223 bytes of data
#define NMEA2000_MAX_PACKET_LENGTH 223

struct nmea2000_raw_frame {
    uint32_t extid;
    uint8_t len;
    uint8_t data[8];
}; 

typedef enum {
    unused     = 0,
    complete   = 1, 
    error      = 2,  
    incomplete = 3
} nmea2000_state;

struct nmea2000_packet {
    uint16_t idx;
    uint32_t ptr;
    uint32_t fast_packet_len;
    uint8_t outbuffer[NMEA2000_MAX_PACKET_LENGTH];
    uint32_t outbuflen;
    uint32_t pgn;
    uint8_t prio;
    uint8_t daddr;
    uint8_t saddr;
    uint32_t state;
} ;

extern int nmea2000_parsemsg(struct nmea2000_raw_frame * frame);
extern void nmea2000_init(void);
extern uint32_t nmea2000_make_extid(uint32_t pgn, uint8_t prio, uint8_t saddr, uint8_t daddr);

extern uint32_t nmea2000_packet_count;            // count number of all packets completed (fast and single)
extern uint32_t nmea2000_packet_fast_count;       // number of fast packets completed
extern uint32_t nmea2000_packet_error_count;      // number of packets that had an error and were aborted
extern uint32_t nmea2000_packet_cancel_count;     // number of fast transmissions thar were cancled or interrupted
extern uint32_t nmea2000_frame_count;             // number of frames handled
extern uint32_t nmea2000_packet_transfer_count;   // number of packets delivered

#endif // _NMEA2000_H_

