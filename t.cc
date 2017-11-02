#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "gps.h"
#include "bits.h"
#include "utils.h"

int checksum() {

  char buf[] = "$IIMTW,00,C";
  char csum[3] = { '0', '0', '0' };
  unsigned int n = 1, crc;

  for (n = 1; buf[n] != '\0'; n++)
    crc ^= buf[n];
  (void)snprintf(csum, sizeof(csum), "%02X", crc);

  printf("%s\n", csum);

}

int ashex(uint32_t pgn) {
  uint8_t b[4];
  set8leu32(b, pgn, 0);
  printf("%02x%02x%02x%02x\n", b[0], b[1], b[2], b[3]);
}

void makeextid(uint32_t pgn, uint8_t prio, uint8_t saddr, uint8_t daddr) {

    uint32_t extid = 0;
    uint32_t ppgn = pgn;
    uint8_t  pdaddr;

    // PDU1 or PDU2?
    if (((ppgn & 0xff00) >> 8) < 0xf0) {
      pdaddr  = daddr;
      ppgn  = ppgn & 0x01ff00;
    } else {
      pdaddr = 0x00;
    }

    extid =
           (prio & 0x07) << 26
        | ((ppgn & 0x1ffff) << 8)
        |  (pdaddr << 8)
        |  (saddr & 0xff);

    printf("n2k: extid= %08x, pgn= %u/%x, prio= %u, src= %u, dest= %u\n",
           extid, pgn, pgn, prio, saddr, daddr);
}

int latitude() {
  unsigned char l[4] = {0xe4, 0xba, 0x56, 0x23};

  int lat = getles32(l, 0) * 0.06;

  printf("%f\n", lat/AIS_LATLON_DIV);
}

int bitwise() {
    uint64_t ECU_name = 0xdcfd21220082320c;
    uint16_t manu = 270;
    uint8_t i= 0;

    ECU_name &= ~((uint64_t)0x7ff << 21);
    ECU_name |= ((uint64_t)manu << 21);

    for(i = 0; i < 8; i++) {
      printf("0x%02X ", ((uint8_t *)&ECU_name)[i]);
    }
    printf("\n");
}

int eoffile(){
   long nc;
   nc = 0;

   char * line = NULL;
   size_t len = 0;
   ssize_t read;
   FILE * fp = NULL;

   fp = fopen("test/log-20170723-clean.bin", "r");
   if (fp == NULL)
     exit(EXIT_FAILURE);

//   while (getchar() != EOF)
//       ++nc;

   while ((read = getline(&line, &len, fp)) != -1) {
     printf("Retrieved line of length %zu :\n", read);
     printf("%s", line);
     ++nc;
   }

   fclose(fp);
   if(line)
     free(line);

   printf("%ld\n", nc);
}

int makeextids() {
    makeextid(59904, 6, 1, 255);
    makeextid(60928, 6, 0x25, 255);
    makeextid(59904, 6, 1, 0x25);
    makeextid(129283, 3, 1, 0xff);
    makeextid(129540, 6, 1, 0xff);
    makeextid(128275, 6, 25, 0xff);
    makeextid(65311, 7, 1, 0xff);
    return 0;
}

int test_3bytes() {
    uint8_t b[4] = { 0, 0xee, 0, 0x77 };
    uint32_t pgn =
        (uint32_t)( ((uint16_t)getleu16(b, 0) << 16) | ((uint16_t)(b[2]) << 16));          // only 3 bytes long!

    printf("pgn %u %u\n", b[2], pgn);

}

int test_read126464() {
    int i = 0;
    uint8_t b[10] = { 0, 0, 0xe8, 0, 0, 0xea, 0, 0, 0xeb, 0 };
    for(i = 1; i< sizeof(b); i+= 3) {
        uint32_t pgn = getleu24(b, i);          // only 3 bytes long!
        printf("pgn %u\n", pgn);
    }

    set8leu24(b, 0x000102, 1);

    for(i = 1; i< sizeof(b); i++) {
        printf("[%u] 0x%02x\n", i, b[i]);
    }

    uint32_t t = getleu24(b, 1);
    printf("0x000102 eq 0x%06x\n", t);

}

int test_read0183end() {
    const char * t[] = {
        "$GPRMC,163923,A,5157.6196,N,00116.6354,E,0.3305,,170716,,*00\r\n",
        "$\r\n",
        "$\\\n",
        "$GPRMC,163923,A,5157.6196,N,00116.6354,E,0.3305,,170716,,*00\t",
    };

    int i= 0, c= 0, c1= 0;

    char buf[255], reply[255];
    char * end;

    for(i = 0; i< 4; i++) {

        strcpy(buf, t[i]);
        strcpy(reply, buf);

        c1 = strlen(buf) - 1;
        for (c = c1; isspace(buf[c]) && (c > 0); c--)
            continue;
        c1 = c+1;
        for(c= c1; (c < strlen(buf)) && (c1 < 255-1); c++) {
            if(buf[c] == '\r') {
                reply[c1++] = '\\';
                reply[c1++] = 'r';
            } else if(buf[c] == '\n') {
                reply[c1++] = '\\';
                reply[c1++] = 'n';
            } else {
                c1 += sprintf(&reply[c1], "\\%02x", buf[c]);
            }
        }
        reply[c1] = '\0';

        printf(">%s<\n", reply);
    }
    return 0;
}

int test_safeatof() {
    const char * t[] = {
	"!$%%§nm",
	"asdasd",
	"123f3",
        "123333"
	};
    int i = 0;

    for(i = 0; i < 4; i++)
	printf("%f\n", safe_atof(t[i]));
    return 0;
}

static double degtodm(double angle)
/* decimal degrees to GPS-style, degrees first followed by minutes */
{
    double fraction, integer;
    fraction = modf(angle, &integer);
    return floor(angle) * 100 + fraction * 60;
}

void test_seatalk_lat_lon() {

    uint8_t bu[] = {0x58, 0x05, 0x32, 0xb1, 0xac, 0x01, 0x5d, 0x45};
    //uint8_t bu[] = {0x51, 0x02, 0x01, 0x86, 0x0f, 0x00, 0x00, 0x00};

    double lat = 0;
    double lon = 0;

    if(bu[0] == 0x58) {
        lat        = bu[2] +  (bu[3] * 256 + bu[4])/100000.0/0.6;
        lon       = bu[5] + ( bu[6] * 256 + bu[7])/100000.0/0.6;
        /*@+type@*/
        if( (bu[1] >> 4) & 0x01 ) {
            lat *= -1.0; // south
        }
        if( ((bu[1] >> 4) & 0x02) == 0 ) {
            lon *= -1.0; // west
        }
    } else if(bu[0] == 0x51) {

        uint16_t mm = getleu16(bu, 3);

        /*@-type@*//* splint has a bug here */
        lon       = bu[2] + (mm & 0x7fFF)/10000.0/0.6;
        /*@+type@*/
        if( (mm & 0x8000) == 0 ) {
            lon *= -1.0;
        }
    }

    printf("lat: %f, lon = %010.4f\n", degtodm(fabs(lat)), degtodm(fabs(lon)));
}

void test_numbers() {
    //double lat = 13.0343;
    //double lon = 13.030;
    double lat = 1677.875;
    // double lat = 5926.7254;
    double d = 0;
    double m = 100.0 * modf(lat / 100.0, &d);
    lat = d + m / 60.0;

    double lon = 13.030;

    printf("d: %f, m: %f, lat: %f, lon = %010.4f\n", d, m, degtodm(fabs(lat)), degtodm(fabs(lon)));
}

int main(int argc, char * argv[]) {
//    test_read0183end();
//    test_safeatof();
//    test_seatalk_lat_lon();
    test_numbers();
}

//18eaff01,59904,p:06,s:01,d:ff,x:00 ee 00 00 00 00 00 00
//18eeff25,60928,p:06,s:25,d:ff,x:ec e7 e4 10 00 82 78 c0
//18ea2501,59904,p:06,s:01,d:25,x:00 ee 01 00 00 00 00 00
//0df90301,129283,p:03,s:01,d:ff,x:ff 7f ff ff ff 7f ff ff
//19fa0401,129540,p:06,s:01,d:ff,x:e0 0f 20 ff 01 1d 7e 70
//19f51325,128275,p:06,s:25,d:ff,x:c2 00 ff ff ff ff ff ff
//1cff1f01,65311,p:07,s:01,d:ff,x:3b 9f ff 00 00 ff ff ff

///18eaff01,59904,p:06,s:01,d:ff,x:00 ee 00 00 00 00 00 00
//18eeff25,60928,p:06,s:25,d:ff,x:ec e7 e4 10 00 82 78 c0

//18eaff01,59904
//18eeff25,60928
