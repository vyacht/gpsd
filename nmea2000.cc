/*
*/

#include <stdint.h>
#include <string.h>

#include "nmea2000.h"

// TODO - must be better solution for getting printing into both: stm and linux
// #include "printf.h"
#include "gpsd.h"

uint8_t vy_printf(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    gpsd_report(0, 10, format, ap);
    va_end(ap);
    return 0;
}


uint32_t nmea2000_packet_count;            // count number of all packets completed (fast and single)
uint32_t nmea2000_packet_fast_count;       // number of fast packets completed
uint32_t nmea2000_frame_count;             // number of frames handled
uint32_t nmea2000_packet_error_count;      // number of packets that had an error and were aborted
uint32_t nmea2000_packet_cancel_count;     // number of fast transmissions thar were cancled or interrupted
uint32_t nmea2000_packet_transfer_count;   // number of packets delivered

/* 32 index is reserved for single transmissions 
   0 - x1F/31 is for multiple fast transmissions from multiple devices 

   Mailboxes for fast transmissions are organized in a ring of 
   mailboxes 0 - 31/0x1F. 
   A new fast transmission is just fetching the next mailbox from the ring. 
   If there is a pending transmission then that pending transmission is 
   canceled under the assumption that it never finished. 

   This approach avoids a cumbersom cleaning process of never finished
   fast transmissions.

   Assumptions: 

      i) 1 "mailbox" is enough for all single transmissions. 
      ii) fast mailboxes can be identified by their the source device 

   to i) frames come from a serial line, we do have the queue as a buffer and
   frames are processed 1 by 1 from the queue. Even if multiple devices send 
   single transmissions quickly they cannot interrupt each other.

   to ii) We need multiple "mailboxes" for fast transmissions which
   will be disrupted from fast and single transmissions of other devices. 
   But N2K requires source ids to be unique and fast transmissions shall not 
   be disrupted by fast transmissions of the same device.
*/
struct nmea2000_packet nmea2000_packets[32 + 1];

/* This array identifies the source addresses fast transmission 
   mailbox during a fast transmission. Obviously it could be 254 
   indexes only as source address 255 is reserved for "to all". */
uint8_t saddr_packet[255];

uint8_t free_mailbox_counter;

uint32_t n2k_fixed_fast_list[] = {
    126464,
    126996,
    127489,
    128275,  // distance log as provided by DST800
    129038,
    129039,
    129540,
    129542,
    129793,
    129794,
    129798,
    129802,
    129809,
    129810,
    130935, // observed in AIS
    130842, // observed in AIS
    262161,
    262384,
    262386,  // observed when testing ROT with actisense

    0
};

uint32_t n2k_dynamic_fast_list[255];

void nmea2000_init_fast_list(void) {
    uint16_t i = 0;
    for(i= 0; i < 255; i++)
        n2k_dynamic_fast_list[i] = 0;
}

void nmea2000_init() {

    uint8_t i = 0;
    struct nmea2000_packet * p = NULL;
    
    nmea2000_packet_count          = 0;          
    nmea2000_packet_fast_count     = 0;     
    nmea2000_packet_error_count    = 0;      
    nmea2000_packet_cancel_count   = 0;
    nmea2000_packet_transfer_count = 0;
    nmea2000_frame_count           = 0;      

    free_mailbox_counter = 0;

    memset(saddr_packet, 0, 255);
    for(i = 0; i < 33; i++) {
        p = &nmea2000_packets[i];
        p->pgn   = 0;
        p->saddr = 0;
        p->daddr = 0;
        p->ptr   = 0;
        p->idx   = 0;
        p->outbuflen = 0;
        
        p->fast_packet_len = 0;
        p->state = unused;
    }

    nmea2000_init_fast_list();
}

int nmea2000_isfast(uint32_t pgn) {
    
    uint16_t cnt = 0;
    
    while(n2k_fixed_fast_list[cnt] > 0) {
        if(pgn == n2k_fixed_fast_list[cnt])
            return 1;
        cnt++;
    }
    
    cnt = 0;
    while(n2k_dynamic_fast_list[cnt] > 0) {
        if(pgn == n2k_dynamic_fast_list[cnt])
            return 1;
        cnt++;
    }

    return 0;
}

uint32_t nmea2000_make_extid(uint32_t pgn, uint8_t prio, uint8_t saddr, uint8_t daddr) {

    uint32_t ppgn = pgn;
    uint8_t  pdaddr = 0;

    // PDU1 or PDU2?
    if (((ppgn & 0xff00) >> 8) < 0xf0) {
      pdaddr  = daddr;
      ppgn  = ppgn & 0x01ff00;
    } else {
      pdaddr = 0x00;
    }

    return
           (prio & 0x07) << 26 
        | ((ppgn & 0x1ffff) << 8) 
        |  (pdaddr << 8) 
        |  (saddr & 0xff);
}


/* Return value is the number of the mailbox that is complete: 

   -1 for none (during fast transmission or an error 
    0 for single transmission 
    n > 0 for completed fast transmissions
*/
int nmea2000_parsemsg(struct nmea2000_raw_frame * frame) {

    uint8_t  l2 = 0;
    uint8_t mb  = 0;
    uint32_t pgn;
    uint8_t  prio;
    uint8_t  daddr;
    uint8_t  saddr;
    
    struct nmea2000_packet * packet = NULL;

    saddr = (uint8_t)(frame->extid & 0xff);
    pgn = (frame->extid >> 8) & 0x1ffff;
    prio = (uint8_t)((frame->extid >> 26) & 0x7);

    // PDU1 messages with bits 0x0000ff000 between 0x00 to 0xef and make use of dest and source address
    // PDU2 messages from 0xf0 to 0xff are intended to be broadcasts
    if (((pgn & 0xff00) >> 8) < 0xf0) {
        daddr  = (uint8_t)(pgn & 0xff);
        pgn  = pgn & 0x01ff00;
    } else {
        daddr = (uint8_t)0xff;
    }

    /*
    vy_printf("I: <= N2K %lx,%lu,p:%02x,s:%02x,d:%02x,x:%02x %02x %02x %02x %02x %02x %02x %02x \n", 
                frame->extid, pgn, prio, saddr, daddr, 
                frame->data[0], frame->data[1], frame->data[2], frame->data[3], 
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
    */

    nmea2000_frame_count++;


    // is this a fast transmission (list of pgn from gpsd)
    if(nmea2000_isfast(pgn)) {
        
      if((frame->data[0] & 0x1f) == 0) {
          // start of fast transmission, need to get a free mailbox

          if(frame->data[1] > NMEA2000_MAX_PACKET_LENGTH) {
              
              vy_printf("I: <= N2K %lu,fi:%02x,l:%u,s:%02x,d:%02x - ERROR\n", 
                        pgn, (uint16_t)frame->data[0], (uint16_t)frame->data[1], saddr, daddr);
              nmea2000_packet_error_count++;
              return -1;
          }
          
          
          mb = free_mailbox_counter & 0x1F;
          free_mailbox_counter++;
          
          packet = &nmea2000_packets[mb];
          saddr_packet[saddr] = mb;

          if(packet->state == incomplete)
              nmea2000_packet_cancel_count++;

          vy_printf("I: <= N2K %lu,s:%02x,mb:%u,fi:%02x,pl:%u\n", 
                    pgn, saddr, (uint16_t)mb, (uint16_t)frame->data[0], (uint16_t)frame->data[1]);
          
          packet->state = incomplete;
          
          packet->fast_packet_len = frame->data[1];
          
          packet->idx = frame->data[0] + 1;  // record next indexes position
                                             // can be > 0! 
          
          packet->ptr = 0;
          packet->pgn = 0;                   // use this sign for whole fast trans being done
          packet->saddr = saddr;             // recording saddr to track packet owner
          
          for (l2=2;l2<8;l2++) {
              // no worries about the ptr becoming to large here
              packet->outbuffer[packet->ptr++]= frame->data[l2];
          }

          return mb;
          
      } else {
          // continue pending fast transmission

          if(saddr_packet[saddr] > 0x1F) {
              vy_printf("I: <= N2K N2K %lu,s:%02x,os:%02x - ERROR MB\n", 
                        pgn, saddr, (uint16_t)saddr_packet[saddr]);
              nmea2000_packet_error_count++;
              return -1;
          }

          mb = saddr_packet[saddr];

          // fetch mailbox for this transmission
          packet = &nmea2000_packets[mb];

          if(packet->saddr != saddr) {
              // could be a stale canceled packet that was taken by a new saddr
              vy_printf("I: <= N2K N2K %lu,s:%02x,mb:%u,os:%02x - STALE\n", 
                        pgn, saddr, (uint16_t)mb, (uint16_t)packet->saddr);
              return -1;
          }
          
          if(frame->data[0] == packet->idx) {

              // still incomplete
              packet->state = incomplete;
          
              for (l2=1; l2<8; l2++) {
                  if (packet->fast_packet_len > packet->ptr) {
                      packet->outbuffer[packet->ptr++] = frame->data[l2];
                  }
              }
              if (packet->ptr >= packet->fast_packet_len) {
              
                  packet->outbuflen = packet->fast_packet_len;
                  packet->prio  = prio;
                  packet->daddr = daddr;
                  packet->state = complete;
                  packet->pgn   = pgn;
              
                  packet->fast_packet_len = 0;
                  packet->idx = 0;
                  packet->ptr = 0;
                  nmea2000_packet_count++;
              
                  vy_printf("I: <= N2K %lu,s:%02x,mb:%u,fi:%02x,fl:%u\n", 
                            pgn, saddr, (uint16_t)mb, (uint16_t)frame->data[0],(uint16_t)frame->len);
              } else {
              
                  vy_printf("I: <= N2K %lu,s:%02x,mb:%u,fi:%02x\n", 
                            pgn, saddr, (uint16_t)mb, (uint16_t)frame->data[0]);
                  packet->idx += 1;

              }
              
              return mb;
          
          } else {
              
              // error - missing or wrong index
              vy_printf("I: <= N2K %lu,s:%02x,mb:%u,pi:%02x,fi:%02x - ERROR\n", 
                        pgn, saddr, (uint16_t)mb, packet->idx, (uint16_t)frame->data[0]);

              packet->idx = 0;
              packet->fast_packet_len = 0;
              packet->state = error;
              nmea2000_packet_error_count++;

              // error
              return -1;
          } // frame->Data[0] == packet->idx
      } // start/continue fast transmission
      
    } else {
        // single transmission

        if(frame->len > 8) {
            nmea2000_packet_error_count++;
            return -1;
        }

        vy_printf("I: <= N2K %lu,s:%02x\n", pgn, saddr);
        packet = &nmea2000_packets[32];
        
        packet->ptr = 0;
        for (l2=0; l2 < frame->len && l2 < 8; l2++) {
            packet->outbuffer[packet->ptr++]= frame->data[l2];
        }
        packet->idx = 0;
        packet->outbuflen = frame->len;
        packet->fast_packet_len = 0;
        packet->pgn = pgn;
        packet->prio = prio;
        packet->daddr = daddr;
        packet->saddr = saddr;
        packet->state = complete;
        
        nmea2000_packet_count++;

        // this will always be '0' as its the number of the single transmission mailbox
        return 32;
    }

    return -1;
}
