
/* Sample TCP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "gps.h"
#include "frame.h"

static int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

void checksum(char * csum, char * buf, uint16_t len) {

  uint32_t n, crc = 0;
  for (n = 1; n < len; n++)
    crc ^= buf[n];
  (void)snprintf(csum, sizeof(csum), "%02X", crc);

}

void gen_ROT(uint8_t *b, uint32_t * len) {

  char csum[3] = { '0', '0', '0' };
  int l;

  const char s[] = "$GPROT,12.7,A";

 // package type and origin
  b[0] = 0x01 | (0 << 5);

  strcpy(&b[2], s);
  l = strlen(s) + 1 + 2;

  checksum(csum, b, *len);
  b[l++] = '*';
  b[l++] = csum[0];
  b[l++] = csum[1];
  b[l++] = '\r';
  b[l++] = '\n';
  b[l] = '0';

  *len = l; 

  b[1] = (uint8_t)(l-2);         // package len
}

void setleu32(uint8_t * b, int offset, uint32_t val) {
  b[offset]     = val;
  b[offset + 1] = val >>  8;
  b[offset + 2] = val >> 16;
  b[offset + 3] = val >> 24;
}

void setles16(uint8_t * b, int offset, int16_t val) {
  b[offset]     = val & 0xff;
  b[offset + 1] = (val >> 8) & 0xff;
}

/*
2,129026,2,255,8,12,fc,70,76,ff,ff,ff,ff
2,129026,2,255,8,13,fc,ff,ff,f8,a1,ff,ff
2,129026,2,255,8,14,fd,ff,ff,f8,a1,ff,ff
2,127250,2,255,8,14,ff,ff,ff,7f,00,00,fc
3,129029,2,255,43,14,3a,34,ff,ff,ff,ff,ff,ff,ff,ff,ff,ff,ff,7f,ff,ff,ff,ff,ff,ff,ff,7f,ff,ff,ff,ff,ff,ff,ff,7f,00,fc,ff,ff,7f,ff,7f,ff,ff,ff,7f,ff
3,129033,2,255,8,3a,34,ff,ff,ff,ff,ff,7f
3,126992,2,255,8,14,f0,3a,34,ff,ff,ff,ff
*/

void gen(uint8_t *b, uint32_t * buflen, uint8_t pkglen, 
	 uint32_t pgn, uint8_t prio, uint8_t src, uint8_t dest) {

    return;
    /*
  // package type and origin
  b[0] = 0x02 | (0 << 5);
  b[1] = (uint8_t)pkglen; // package len

  setleu32(b, 2, pgn);
  setleu32(b, 6, 12);
    */
}

void gen_129029(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 43, 129029, 2, 0, 0);

  uint8_t sentence[] = {0x14,0x3a,0x34,0xff,0xff,0xff,0xff,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0x00,
			0xfc,0xff,0xff,0x7f,0xff,0x7f,0xff,0xff,
			0xff,0x7f,0xff};

  memcpy(&b[0], sentence, 43);

  *buflen = 43;
}

void gen_129033(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 129033, 2, 0, 0);

  uint8_t sentence[] = {0x3a,0x34,0xff,0xff,0xff,0xff,0xff,0x7f};

  memcpy(&b[0], sentence, 8);

  *buflen = 8;
}

/*
 *   PGN 129025: GNSS Position Rapid Update
 */
void gen_129025(uint8_t *b, uint32_t * buflen) {

    int64_t lat = val / 1e-7;
    int64_t lon = val / 1e-7;
    
    setles32(b, 0, lat);
    setles32(b, 4, lon);

    *buflen = 8;
}

/*
 *   PGN 129026: GNSS COG and SOG Rapid Update
 *   http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf
 */
void gen_129026(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 129026, 2, 0, 0);

  uint8_t sentence[] = {0x12,0xfc,0x70,0x76,0xff,0xff,0xff,0xff};

  memcpy(&b[0], sentence, 8);

  *buflen = 8;
}

/*
 *   PGN 127250: NAV Vessel Heading
 */
void gen_127250(uint8_t *b, uint32_t * buflen, 
                uint8_t sid, double heading, double variation, double variation) {

  gen(b, buflen, 8, 127250, 2, 0, 0);

  uint8_t sentence[] = {0x14,0xff,0xff,0xff,0x7f,0x00,0x00,0xfc};

  uint16_t hdg = (uint16_t)(heading / 0.0001 / RAD_2_DEG);
  uint16_t dev = (uint16_t)(deviation / 0.0001 / RAD_2_DEG);
  uint16_t var = (uint16_t)(variation / 0.0001 / RAD_2_DEG);

  sentence[0] = sid;
  setleu16(sentence, 1, hdg);
  setleu16(sentence, 3, dev);
  setleu16(sentence, 5, var);

  memcpy(&b[0], sentence, 8);

  *buflen = 8;
}

void gen_127251(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 127251, 2, 0, 0);

  uint8_t sentence[] = {0xc4,0x20,0x6d,0x00,0x00,0xff,0xff,0xff};

  memcpy(&b[0], sentence, 8);

  *buflen = 8;
}

void gen_127257(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 127257, 2, 0, 0);

  int16_t roll = (uint16_t)(23.7 / 0.0001 / RAD_2_DEG);
  printf("roll = %d\n", roll);

  uint8_t sentence[] = {0x01,0xff,0x7f,0xff,0x7f,0xff,0x7f,0x00};
  setles16(sentence, 1, 0x7fff);
  setles16(sentence, 3, 0x7fff);
  setles16(sentence, 5, roll);

  memcpy(&b[0], sentence, 8);

  *buflen = 8;
}

char hexstr[] = "7e013121414956444d2c312c312c2c412c4834324d68606c55693268686c6a69493d6d696b6b303030423035302c302a31370d0a";

// faulty one leading to seatalk wrong datagram length
// char hexstr[] = "7e030500020102037e07012c7e0301897e0301627e0302314d7e0301a27e0302304d7e0301897e0301527e0301337e030150";

// faulty one even leading to a crash
//char hexstr[] = "7e0301487e030224147e0301257e0301357e030342c9c57e0301627e03012c7e0301897e0301627e03012c7e0301897e0301627e03032cc9a57e0301b27e03";

//char hexstr[] = 
//  "7e03012a7e0301827e03020d217e0301d27e0301417e0301477e03028ab17e03012c7e0301897e0301627e0301327e03019a7e0301347e03024c267e0301497e0301aa7e";

int main(int argc, char**argv)
{
   int listenfd,connfd,n;
   uint32_t len, len1;
   struct sockaddr_in servaddr,cliaddr;
   socklen_t clilen;
   pid_t     childpid;
   unsigned char msg[255];
   unsigned char msgl[4096];
   uint32_t pgn;

   int msgll = 
       gpsd_hexpack(hexstr, msgl, 4096);

   signal(SIGINT, intHandler);

   listenfd=socket(AF_INET,SOCK_STREAM,0);

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
   servaddr.sin_port=htons(32000);
   bind(listenfd,(struct sockaddr *)&servaddr,sizeof(servaddr));

   listen(listenfd, 1024);

   while(keepRunning)
   {
      clilen=sizeof(cliaddr);

      connfd = accept(listenfd, (struct sockaddr *)&cliaddr, &clilen);

      printf("ACCEPT\n");

      if ((childpid = fork()) == 0)
      {
          close (listenfd);
          uint8_t c;
          double heading = 25.0;

          while(keepRunning)
          {
              memset(msg, 0, 255);
              switch(c % 5) {
              case 0: gen_127251(msg, &len); pgn= 127251; break;
              case 1: gen_127257(msg, &len); pgn= 127257; break;
              case 2: gen_129026(msg, &len); pgn= 129026; break;
              case 3: gen_129029(msg, &len); pgn= 129029; break;
              case 4: gen_129033(msg, &len); pgn= 129033; break;
              default: break;
              }

              //gen_ROT(msg, &len);
              
              //printf("msg len= %d (%s)\n", msgll, msgl);

              // TODO: lots of copying, probably better with toHDLC with offset
            uint8_t dataHDLC[255];

            gen_127250(msg, &len, 0x14, heading, 0, 0); pgn= 127250;
            heading += 5;
            printf("Heading= %.2f\n", heading);

            setleu32(dataHDLC, 0, pgn);
            memcpy(&(dataHDLC[4]), msg, len);

            uint8_t frm[512]; uint32_t in = 0;
            in = frm_toHDLC8(frm, 512, FRM_TYPE_NMEA2000, dataHDLC, len+4);

            // test if double sentence also works
            gen_127257(msg, &len);
            setleu32(dataHDLC, 0, 127257);
            memcpy(&(dataHDLC[4]), msg, len);
            in += 
                frm_toHDLC8(frm + in, 512, FRM_TYPE_NMEA2000, dataHDLC, len+4);

            uint32_t pos = 0; uint8_t ch;
            int width=16; //8;
            unsigned char line[9] = "\0";
            for (pos=0; pos < in; pos++) {
              ch = frm[pos];
              line[pos%width] = ((ch > 31) && (ch < 128)) ? ch : '.';
              line[(pos%width)+1] = '\0';
              printf("%02x ", ch);
                if (pos%width > (width-2)) {
                  printf("  %s\n", line);
                }
              }
              //printf("msg len= %d (%s)\n", frm, in);
              printf("\n");
              sendto(connfd, frm, in, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
              //sendto(connfd, msgl, msgll, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
              //sendto(connfd, msg, len, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
              usleep(1000*500);
              c++;
          }
          
      }
      close(connfd);
   }
}

