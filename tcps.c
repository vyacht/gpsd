
/* Sample TCP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

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

  // package type and origin
  b[0] = 0x02 | (0 << 5);
  b[1] = (uint8_t)pkglen; // package len

  setleu32(b, 2, pgn);
  setleu32(b, 6, 12);

}

void gen_129029(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 43, 129029, 2, 0, 0);

  uint8_t sentence[] = {0x14,0x3a,0x34,0xff,0xff,0xff,0xff,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0xff,
			0xff,0xff,0xff,0xff,0xff,0xff,0x7f,0x00,
			0xfc,0xff,0xff,0x7f,0xff,0x7f,0xff,0xff,
			0xff,0x7f,0xff};

  memcpy(&b[10], sentence, 43);

  *buflen = 53;
}

void gen_129033(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 129033, 2, 0, 0);

  uint8_t sentence[] = {0x3a,0x34,0xff,0xff,0xff,0xff,0xff,0x7f};

  memcpy(&b[10], sentence, 8);

  *buflen = 18;
}

void gen_129026(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 129026, 2, 0, 0);

  uint8_t sentence[] = {0x12,0xfc,0x70,0x76,0xff,0xff,0xff,0xff};

  memcpy(&b[10], sentence, 8);

  *buflen = 18;
}

void gen_127250(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 127250, 2, 0, 0);

  uint8_t sentence[] = {0x14,0xff,0xff,0xff,0x7f,0x00,0x00,0xfc};

  memcpy(&b[10], sentence, 8);

  *buflen = 18;
}

void gen_127251(uint8_t *b, uint32_t * buflen) {

  gen(b, buflen, 8, 127251, 2, 0, 0);

  uint8_t sentence[] = {0xc4,0x20,0x6d,0x00,0x00,0xff,0xff,0xff};

  memcpy(&b[10], sentence, 8);

  *buflen = 18;
}

int main(int argc, char**argv)
{
   int listenfd,connfd,n;
   uint32_t len;
   struct sockaddr_in servaddr,cliaddr;
   socklen_t clilen;
   pid_t     childpid;
   unsigned char msg[255];

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

         while(keepRunning)
         {
	   memset(msg, 0, 255);
	   switch(c % 5) {
	   case 0: gen_127251(msg, &len); break;
	   case 1: gen_127250(msg, &len); break;
	   case 2: gen_129026(msg, &len); break;
	   case 3: gen_129029(msg, &len); break;
	   case 4: gen_129033(msg, &len); break;
	   default: break;
	   }

	     //gen_ROT(msg, &len);

	   printf("msg len= %d (%s)\n", len, msg);

            sendto(connfd, msg, len, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
	    usleep(1000*500);
	    c++;
         }
         
      }
      close(connfd);
   }
}

