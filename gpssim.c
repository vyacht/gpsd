
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "gpsd.h"

#define PORT 2000


/* kinetic state vector */
struct ksv_t {
  double speed;        // Speed in m/s 
  double latitude;     // Latitude in degrees 
  double longitude;    // Longitude in degrees 
};

/*
  Calculates a new position in degrees with course / distance from 
  an initial point lat1/lon1.

  Formula from <http://williams.best.vwh.net/avform.htm#Rhumb>
  Initial point cannot be a pole, but GPS doesn't work at high.
  latitudes anyway so it would be OK to fail there.
  Seems to assume a spherical Earth, which means it's going
  to have a slight inaccuracy rising towards the poles.
  The if/then avoids 0/0 indeterminacies on E-W courses.  
*/
int earth_position_from_distance(double lat1, double lon1, 
				 double course, double distance,
				 double * lat, double * lon) {

  double tc    = DEG_2_RAD * course;
  double d     = DEG_2_RAD * (distance/60.0); // distance from nm to rad, 1nm = 1min deg
  double lat1R = DEG_2_RAD * lat1;
  double lon1R = DEG_2_RAD * lon1;
  

  // printf("tc= %f, d= %f, lat1R= %f, lon1R= %f\n", tc, d, lat1R, lon1R);

  *lat = lat1R + d * cos(tc);

  double q = 0.0;

  if (fabs(*lat) > GPS_PI/2.0) {
    printf("error: distance %f > %f too large. You can't go this far along this rhumb line!\n", 
	   fabs(*lat), GPS_PI/2.0);
    return 1;
  }
  if (fabs(*lat - lat1R) < sqrt(1e-15)) {
    q = cos(lat1R);
  } else {
    double t1 = (tan(*lat/2.0 + GPS_PI/4.0));
    double t2 = (tan(lat1R/2.0 + GPS_PI/4.0));
    double dphi = log(t1/t2);
    q = (*lat-lat1R)/dphi;
    // printf("d = %f, %f, %f\n", d, t1, t2);
  }

  double dlon = -d * sin(tc) / q;

  *lon = RAD_2_DEG * (fmod(lon1R + dlon + GPS_PI, 2.0 * GPS_PI) - GPS_PI);
  // printf("%f, %f\n", *lat, *lon);
  *lat = RAD_2_DEG * (*lat);
  
  return 0;
}

ssize_t gpsd_write(struct gps_device_t *session,
		   const char *buf,
		   const size_t len)
/* pass low-level data to devices straight through */
{
    return gpsd_serial_write(session, buf, len);
}

void gpsd_throttled_report(const int errlevel, const char * buf) {}
void gpsd_report(const int debuglevel, const int errlevel,
		 const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, 0, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}
void gpsd_external_report(const int debuglevel, const int errlevel,
			  const char *fmt, ...) {
}

struct gps_dt_t {
  int PRN;
  int elevation;
  int azimuth;
  int ss;
};


struct gps_dt_t gd[] = {{3,41,50,46}, 
			{7,60,317,49}, 
			{8,34,315,45},
			{11,35,148,45},
			{13,38,213,45},
			{16,27,68,43},
			{19,72,76,50},
			{23,9,182,37},
			{24,6,166,35},
			{28,9,271,36}};

int main(int argc, char**argv) {

  int sock, status, sinlen;
  struct sockaddr_in sock_in;

  int yes = 1;
  int port = 2000;
  int filter = 0;
  char c;
  char * bcast = NULL;

  while ((c = getopt(argc, argv, "b:p:f")) != -1) {
    switch(c) {
    case 'b':
      bcast = optarg;
      break;
    case 'p':
      port = atol(optarg);
      break;
    case 'f':
      filter = 1;
      break;

    default:
      break;
    }
  }

  sinlen = sizeof(struct sockaddr_in);
  memset(&sock_in, 0, sinlen);

  sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
  sock_in.sin_port = htons(0);
  sock_in.sin_family = PF_INET;

  /* -1 = 255.255.255.255 this is a BROADCAST address,
     a local broadcast address could also be used.
     you can comput the local broadcat using NIC address and its NETMASK 
  */

  if(bcast != NULL) {
    sock_in.sin_addr.s_addr=inet_addr(bcast); //htonl(-1); /* send message to 255.255.255.255 */
    printf("sendto using %s\n", bcast);
  } else {
    sock_in.sin_addr.s_addr= htonl(-1); /* send message to 255.255.255.255 */
    printf("sendto using 255.255.255.255\n");
  }
  if(port > 0) {
    sock_in.sin_port = htons(port); /* port number */
    printf("sendto using port %d\n", port);
  } else {
    sock_in.sin_port = htons(PORT); /* port number */
  }

  sock_in.sin_family = PF_INET;

  status = bind(sock, (struct sockaddr *)&sock_in, sinlen);
  printf("Bind Status = %d\n", status);

  status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );
  printf("Setsockopt Status = %d\n", status);

  double lat1 = 59.445424;
  double lon1 = 18.33189;

  double course = 270;
  double speed  = 3.7; // knots (nm/h)
  double distance = speed/3600.0; // nm / s
  double lat, lon;

  struct gps_device_t session;
  char bufp[1024];
  size_t len = 1024;

  session.newdata.time = timestamp();
  session.newdata.mode = 10;

  
  while(true) {

    earth_position_from_distance(lat1, lon1, 
				 course, distance,
				 &lat, &lon);


    session.newdata.latitude = lat;
    session.newdata.longitude = lon;
    session.newdata.altitude = 0;

    session.gpsdata.set = TIME_SET | LATLON_SET | ALTITUDE_SET | MODE_SET;
    gps_merge_fix(&session.gpsdata.fix,
		  session.gpsdata.set, &session.newdata);

    session.gpsdata.separation = wgs84_separation(lat, lon);
    session.gpsdata.status = 1;
    session.gpsdata.satellites_used = 3;
    session.gpsdata.dop.hdop = NAN;
    session.mag_var = NAN;

    nmea_tpv_dump(&session, bufp, len);

    char * pch = strtok(bufp, "\r\n");
    while(pch != NULL) {
      char buf[1024];
      sprintf(buf, "%s\r\n", pch);
      if(!((strncmp(buf, "$GPG", 4) != 0) && filter)) {
	printf("%s", buf);
	if(sendto(sock, buf, strlen(buf), 0, (struct sockaddr *)&sock_in, sinlen) < 0) {
	// if(send(sock, buf, strlen(buf), 0) < 0) {
	  printf("sendto Status = %s\n", strerror(errno));
	}
      }
      pch = strtok(NULL, "\r\n");
    }


    session.gpsdata.satellites_visible = 10;

    int i = 0;
    for(i = 0; i < 10; i++) {
      session.gpsdata.PRN[i]        = gd[i].PRN;
      session.gpsdata.elevation[i]  = gd[i].elevation;
      session.gpsdata.azimuth[i]    = gd[i].azimuth;
      session.gpsdata.ss[i]         = gd[i].ss;
    }

    session.gpsdata.set = SATELLITE_SET;
    nmea_sky_dump(&session, bufp, len);

    pch = strtok(bufp, "\r\n");
    while(pch != NULL) {
      char buf[1024];
      sprintf(buf, "%s\r\n", pch);
      if(!((strncmp(buf, "$GPG", 4) != 0) && filter)) {
	printf("%s", buf);
	if(sendto(sock, buf, strlen(buf), 0, (struct sockaddr *)&sock_in, sinlen) < 0) {
	// if(send(sock, buf, strlen(buf), 0) < 0) {
	  printf("sendto Status = %s\n", strerror(errno));
	}
      }
      pch = strtok(NULL, "\r\n");
    }
    

    session.gpsdata.set = NAVIGATION_SET;

    session.gpsdata.navigation.set = NAV_DPT_PSET | NAV_HDG_MAGN_PSET | NAV_HDG_TRUE_PSET;
    session.gpsdata.navigation.depth = 2.7;
    session.gpsdata.navigation.depth_offset = 0.3;

    session.gpsdata.navigation.heading[compass_magnetic] = 90;
    session.gpsdata.navigation.heading[compass_true] = 90;
    session.gpsdata.navigation.speed_thru_water = speed;
    session.gpsdata.environment.deviation = 0;
    session.gpsdata.environment.variation = 0;

    nmea_navigation_dump(&session, bufp, len);

    pch = strtok(bufp, "\r\n");
    while(pch != NULL) {
      char buf[1024];
      sprintf(buf, "%s\r\n", pch);
      if(!((strncmp(buf, "$GPG", 4) != 0) && filter)) {
	printf("%s", buf);
	if(sendto(sock, buf, strlen(buf), 0, (struct sockaddr *)&sock_in, sinlen) < 0) {
	  printf("sendto Status = %s\n", strerror(errno));
	}
      }
      pch = strtok(NULL, "\r\n");
    }

    lat1 = lat;
    lon1 = lon;

    session.newdata.time += 1;
    usleep(1000*1000);
  }

  shutdown(sock, 2);
  close(sock);


  return 0;
}
