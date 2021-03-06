/*
 * This is the main sequence of the gpsd daemon. The IO dispatcher, main
 * select loop, and user command handling lives here.
 *
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>		/* for select() */
#include <sys/select.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <ctype.h>
#include <setjmp.h>
#include <assert.h>
#include <pwd.h>
#include <grp.h>
#include <math.h>
#include <syslog.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#ifndef S_SPLINT_S
#include <netdb.h>
#ifndef AF_UNSPEC
#include <sys/socket.h>
#endif /* AF_UNSPEC */
#ifndef INADDR_ANY
#include <netinet/in.h>
#endif /* INADDR_ANY */
#include <sys/un.h>
#include <arpa/inet.h>     /* for htons() and friends */
#endif /* S_SPLINT_S */
#ifndef S_SPLINT_S
#include <unistd.h>
#endif /* S_SPLINT_S */

/* for getifaddr */
#include <netdb.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <net/if.h>            /* IFF_LOOPBACK */

#include "gpsd_config.h"

#include "gpsd.h"
#include "sockaddr.h"
#include "gps_json.h"
#include "revision.h"
#include "frame.h"
#include "bits.h"

#include "navigation.h"
#include "driver_vyspi.h"
#include "timeutil.h"
#include "signalk.h"
#include "pseudon2k.h"

#if defined(SYSTEMD_ENABLE)
#include "sd_socket.h"
#endif
#include "websocket.h"

/*
 * The name of a tty device from which to pick up whatever the local
 * owning group for tty devices is.  Used when we drop privileges.
 */
#if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__)
#define PROTO_TTY "/dev/tty00"	/* correct for *BSD */
#else
#define PROTO_TTY "/dev/ttyS0"	/* correct for Linux */
#endif

/*
 * Timeout policy.  We can't rely on clients closing connections
 * correctly, so we need timeouts to tell us when it's OK to
 * reclaim client fds.  COMMAND_TIMEOUT fends off programs
 * that open connections and just sit there, not issuing a WATCH or
 * doing anything else that triggers a device assignment.  Clients
 * in watcher or raw mode that don't read their data will get dropped
 * when throttled_write() fills up the outbound buffers and the
 * NOREAD_TIMEOUT expires.
 *
 * RELEASE_TIMEOUT sets the amount of time we hold a device
 * open after the last subscriber closes it; this is nonzero so a
 * client that does open/query/close will have time to come back and
 * do another single-shot query, if it wants to, before the device is
 * actually closed.  The reason this matters is because some Bluetooth
 * GPSes not only shut down the GPS receiver on close to save battery
 * power, they actually shut down the Bluetooth RF stage as well and
 * only re-wake it periodically to see if an attempt to raise the
 * device is in progress.  The result is that if you close the device
 * when it's powered up, a re-open can fail with EIO and needs to be
 * tried repeatedly.  Better to avoid this...
 *
 * DEVICE_REAWAKE says how long to wait before repolling after a zero-length
 * read. It's there so we avoid spinning forever on an EOF condition.
 *
 * DEVICE_RECONNECT sets interval on retries when (re)connecting to
 * a device.
 */
#define COMMAND_TIMEOUT		60*15
#define TCP_GRACE_TIMEOUT	1
#define NOREAD_TIMEOUT		60*3
#define RELEASE_TIMEOUT		60
#define DEVICE_REAWAKE		0.01
#define DEVICE_RECONNECT	2

#define QLEN			5

/*
 * If ntpshm is enabled, we renice the process to this priority level.
 * For precise timekeeping increase priority.
 */
#define NICEVAL	-10

#if defined(FIXED_PORT_SPEED) || !defined(SOCKET_EXPORT_ENABLE)
    /*
     * Force nowait in two circumstances:
     *
     * (1) If we're running with FIXED_PORT_SPEED we're some sort
     * of embedded configuration where we don't want to wait for connect
     *
     * (2) Socket export has been disabled.  In this case we have no
     * way to know when client apps are watching the export channels,
     * so we need to be running all the time.
     */
#define FORCE_NOWAIT
#endif /* defined(FIXED_PORT_SPEED) || !defined(SOCKET_EXPORT_ENABLE) */

/* IP version used by the program */
/* AF_UNSPEC: all
 * AF_INET: IPv4 only
 * AF_INET6: IPv6 only
 */
#ifdef IPV6_ENABLE
static const int af = AF_UNSPEC;
#else
static const int af = AF_INET;
#endif

int no_timeouts;

#define AFCOUNT 2

static fd_set all_fds;
static int maxfd;
#ifndef FORCE_GLOBAL_ENABLE
static bool listen_global = false;
#endif /* FORCE_GLOBAL_ENABLE */
#ifndef FORCE_NOWAIT
#define NOWAIT nowait
static bool nowait = false;
#else /* FORCE_NOWAIT */
#define NOWAIT true
#endif /* FORCE_NOWAIT */
static jmp_buf restartbuf;
static struct gps_context_t context;
#if defined(SYSTEMD_ENABLE)
static int sd_socket_count = 0;
#endif

/* work around the unfinished ipv6 implementation on hurd */
#ifdef __GNU__
#ifndef IPV6_TCLASS
#define IPV6_TCLASS 61
#endif
#endif

uint32_t last_bytes_send = 0,
    last_bytes_send_report_ms = 0;
uint32_t last_bytes_send_second = 0,
    last_bytes_send_second_report_ms = 0;


const char *gpsd_canboatdump(char *scbuf, size_t scbuflen, struct gps_device_t *device);

static int max_subscriber_loglevel = LOG_ERROR - 1; // -1 is LOG_ERROR
static void set_max_subscriber_loglevel(void);

static volatile sig_atomic_t signalled;

static void onsig(int sig)
{
    /* just set a variable, and deal with it in the main loop */
    signalled = (sig_atomic_t) sig;
}

ssize_t gpsd_write(struct gps_device_t *session,
       const char *buf,
       const size_t len)
/* pass low-level data to devices straight through */
{
    if (session == NULL ||
        session->context == NULL || session->context->readonly)
        return 0;
    return gpsd_serial_write(session, buf, len);
}

void gpsd_external_report(const int debuglevel, const int errlevel,
     const char *fmt, ...)
{
    if((debuglevel < errlevel) && (max_subscriber_loglevel < errlevel))
      return;

    va_list ap;
    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, max_subscriber_loglevel, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}

void gpsd_report(const int debuglevel, const int errlevel,
     const char *fmt, ...)
{
    if(debuglevel < errlevel)
      return;

    va_list ap;
    va_start(ap, fmt);
    gpsd_labeled_report(debuglevel, LOG_ERROR - 1, errlevel, "gpsd:", fmt, ap);
    va_end(ap);
}

static void typelist(void)
/* list installed drivers and enabled features */
{
    const struct gps_type_t **dp;

    for (dp = gpsd_drivers; *dp; dp++) {
    if ((*dp)->packet_type == COMMENT_PACKET)
        continue;
#ifdef RECONFIGURE_ENABLE
    if ((*dp)->mode_switcher != NULL)
        (void)fputs("n\t", stdout);
    else
        (void)fputc('\t', stdout);
    if ((*dp)->speed_switcher != NULL)
        (void)fputs("b\t", stdout);
    else
        (void)fputc('\t', stdout);
    if ((*dp)->rate_switcher != NULL)
        (void)fputs("c\t", stdout);
    else
        (void)fputc('\t', stdout);
    if ((*dp)->packet_type > NMEA_PACKET)
        (void)fputs("*\t", stdout);
    else
        (void)fputc('\t', stdout);
#endif /* RECONFIGURE_ENABLE */
    (void)puts((*dp)->type_name);
    }
    (void)printf("# n: mode switch, b: speed switch, "
    "c: rate switch, *: non-NMEA packet type.\n");
#if defined(SOCKET_EXPORT_ENABLE)
    (void)printf("# Socket export enabled.\n");
#endif
#if defined(SHM_EXPORT_ENABLE)
    (void)printf("# Shared memory export enabled.\n");
#endif
#if defined(DBUS_EXPORT_ENABLE)
    (void)printf("# DBUS export enabled\n");
#endif
#if defined(TIMEHINT_ENABLE)
    (void)printf("# Time service features enabled.\n");
#endif
#if defined(PPS_ENABLE)
    (void)printf("# PPS enabled.\n");
#endif
    exit(EXIT_SUCCESS);
}

static void usage(void)
{
    (void)printf("usage: gpsd [-b] [-n] [-N] [-D n] [-F sockfile] [-G] [-P pidfile] [-S port] [-h] device...\n\
  Options include: \n\
  -b		     	    = bluetooth-safe: open data sources read-only\n\
  -n			    = don't wait for client connects to poll GPS\n\
  -N			    = don't go into background\n\
  -F sockfile		    = specify control socket location\n"
#ifndef FORCE_GLOBAL_ENABLE
"  -G         		    = make gpsd listen on INADDR_ANY\n"
#endif /* FORCE_GLOBAL_ENABLE */
"  -P pidfile	      	    = set file to record process ID \n\
  -D integer (default 0)    = set debug level \n\
  -S integer (default %s) = set port for daemon \n\
  -h		     	    = help message \n\
  -V			    = emit version and exit.\n\
A device may be a local serial device for GPS input, or a URL in one \n\
of the following forms:\n\
     tcp://host[:port]\n\
     udp://host[:port]\n\
     {dgpsip|ntrip}://[user:passwd@]host[:port][/stream]\n\
     gpsd://host[:port][/device][?protocol]\n\
in which case it specifies an input source for device, DGPS or ntrip data.\n\
\n\
The following driver types are compiled into this gpsd instance:\n",
     DEFAULT_GPSD_PORT);
    typelist();
}

#ifdef CONTROL_SOCKET_ENABLE
static socket_t filesock(char *filename)
{
    struct sockaddr_un addr;
    socket_t sock;

    /*@ -mayaliasunique -usedef @*/
    if (BAD_SOCKET(sock = socket(AF_UNIX, SOCK_STREAM, 0))) {
        gpsd_report(context.debug, LOG_ERROR, "Can't create device-control socket\n");
        return -1;
    }
    (void)strlcpy(addr.sun_path, filename, sizeof(addr.sun_path));
    addr.sun_family = (sa_family_t)AF_UNIX;
    if (bind(sock, (struct sockaddr *)&addr, (int)sizeof(addr)) < 0) {
        gpsd_report(context.debug, LOG_ERROR, "can't bind to local socket %s\n", filename);
        (void)close(sock);
        return -1;
    }
    if (listen(sock, QLEN) == -1) {
        gpsd_report(context.debug, LOG_ERROR, "can't listen on local socket %s\n", filename);
        (void)close(sock);
        return -1;
    }
    /*@ +mayaliasunique +usedef @*/

    /* coverity[leaked_handle] This is an intentional allocation */
    return sock;
}
#endif /* CONTROL_SOCKET_ENABLE */



static struct vessel_t vessel;

static struct gps_device_t devices[MAXDEVICES];

static void adjust_max_fd(int fd, bool on)
/* track the largest fd currently in use */
{
    if (on) {
    if (fd > maxfd)
        maxfd = fd;
    }
#if !defined(LIMITED_MAX_DEVICES) && !defined(LIMITED_MAX_CLIENT_FD)
    /*
     * I suspect there could be some weird interactions here if
     * either of these were set lower than FD_SETSIZE.  We'll avoid
     * potential bugs by not scavenging in this case at all -- should
     * be OK, as the use case for limiting is SBCs where the limits
     * will be very low (typically 1) and the maximum size of fd
     * set to scan through correspondingly small.
     */
    else {
    if (fd == maxfd) {
        int tfd;

        for (maxfd = tfd = 0; tfd < FD_SETSIZE; tfd++)
    if (FD_ISSET(tfd, &all_fds))
        maxfd = tfd;
    }
    }
#endif /* !defined(LIMITED_MAX_DEVICES) && !defined(LIMITED_MAX_CLIENT_FD) */
}

#ifdef SOCKET_EXPORT_ENABLE
#ifndef IPTOS_LOWDELAY
#define IPTOS_LOWDELAY 0x10
#endif

static socket_t passivesock_af(int af, char *service, char *tcp_or_udp, char * bcast, int qlen)
/* bind a passive command socket for the daemon */
{
    volatile socket_t s;
    /*
     * af = address family,
     * service = IANA protocol name or number.
     * tcp_or_udp = TCP or UDP
     * qlen = maximum wait-queue length for connections
     */
    struct servent *pse;
    struct protoent *ppe;	/* splint has a bug here */
    sockaddr_t sat;
    int sin_len = 0;
    int type, proto, one = 1;
    in_port_t port;
    char *af_str = "";
    int yes = 1;
    const int dscp = IPTOS_LOWDELAY; /* Prioritize packet */
    INVALIDATE_SOCKET(s);
    if ((pse = getservbyname(service, tcp_or_udp)))
    port = ntohs((in_port_t) pse->s_port);
    // cppcheck-suppress unreadVariable
    else if ((port = (in_port_t) atoi(service)) == 0) {
    gpsd_report(context.debug, LOG_ERROR,
        "can't get \"%s\" service entry.\n", service);
    return -1;
    }
    ppe = getprotobyname(tcp_or_udp);
    if (strcmp(tcp_or_udp, "udp") == 0) {
        type = SOCK_DGRAM;
        /*@i@*/ proto = (ppe) ? ppe->p_proto : IPPROTO_UDP;
    } else {
        type = SOCK_STREAM;
        /*@i@*/ proto = (ppe) ? ppe->p_proto : IPPROTO_TCP;
    }

    /*@ -mustfreefresh +matchanyintegral @*/
    switch (af) {
    case AF_INET:
        sin_len = sizeof(sat.sa_in);

        memset((char *)&sat.sa_in, 0, sin_len);
        sat.sa_in.sin_family = (sa_family_t) AF_INET;
#ifndef S_SPLINT_S
#ifndef FORCE_GLOBAL_ENABLE
        if (!listen_global)
            sat.sa_in.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        else {
#endif /* FORCE_GLOBAL_ENABLE */
            if(bcast)
                sat.sa_in.sin_addr.s_addr = inet_addr(bcast);
            else
                sat.sa_in.sin_addr.s_addr = htonl(INADDR_ANY);
        }
        sat.sa_in.sin_port = htons(port);
#endif /* S_SPLINT_S */

        af_str = "IPv4";
        /* see PF_INET6 case below */
        s = socket(PF_INET, type, proto);
        if (s > -1 ) {
            /*@-unrecog@*/
            /* Set packet priority */
            if (setsockopt(s, IPPROTO_IP, IP_TOS, &dscp, sizeof(dscp)) == -1)
                gpsd_report(context.debug, LOG_WARN,
                            "Warning: SETSOCKOPT TOS failed\n");
        }
        if (s < 0 ) {
                gpsd_report(context.debug, LOG_ERROR,
                            "opening socket failed %s\n", strerror(errno));
        }
        /*@+unrecog@*/

        break;
#ifdef IPV6_ENABLE
    case AF_INET6:
#ifndef S_SPLINT_S
        sin_len = sizeof(sat.sa_in6);

        memset((char *)&sat.sa_in6, 0, sin_len);
        sat.sa_in6.sin6_family = (sa_family_t) AF_INET6;
#ifndef FORCE_GLOBAL_ENABLE
        if (!listen_global)
            sat.sa_in6.sin6_addr = in6addr_loopback;
        else {
#endif /* FORCE_GLOBAL_ENABLE */
            if(bcast) {
                if (inet_ntop(AF_INET6, &sat.sa_in6.sin6_addr, bcast, INET6_ADDRSTRLEN) == NULL) {
                    gpsd_report(context.debug, LOG_ERROR,
                                "Error: IPV6 address resolution failed.\n");
                    return -1;
                }
            }
            else
                sat.sa_in6.sin6_addr = in6addr_any;
        }
        sat.sa_in6.sin6_port = htons(port);

        /*
         * Traditionally BSD uses "communication domains", named by
         * constants starting with PF_ as the first argument for
         * select.  In practice PF_INET has the same value as AF_INET
         * (on BSD and Linux, and probably everywhere else).  POSIX
         * leaves much of this unspecified, but requires that AF_INET
         * be recognized.  We follow tradition here.
         */
        af_str = "IPv6";
        s = socket(PF_INET6, type, proto);

        /*
         * On some network stacks, including Linux's, an IPv6 socket
         * defaults to listening on IPv4 as well. Unless we disable
         * this, trying to listen on in6addr_any will fail with the
         * address-in-use error condition.
         */
        if (s > -1) {
            int on = 1;
            if (setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, &on, sizeof(on)) == -1) {
                gpsd_report(context.debug, LOG_ERROR,
                            "Error: SETSOCKOPT IPV6_V6ONLY\n");
                (void)close(s);
                return -1;
            }
            /* Set packet priority */
            if (setsockopt(s, IPPROTO_IPV6, IPV6_TCLASS, &dscp, sizeof(dscp)) == -1)
                gpsd_report(context.debug, LOG_WARN,
                            "Warning: SETSOCKOPT TOS failed\n");
        }
#endif /* S_SPLINT_S */
        break;
#endif  /* IPV6_ENABLE */
    default:
        gpsd_report(context.debug, LOG_ERROR,
                    "unhandled address family %d\n", af);
        return -1;
    }
    gpsd_report(context.debug, LOG_IO,
                "opening %s socket\n", af_str);

    if (BAD_SOCKET(s)) {
        gpsd_report(context.debug, LOG_ERROR,
                    "can't create %s socket\n", af_str);
        return -1;
    }
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&one,
                   (int)sizeof(one)) == -1) {
        gpsd_report(context.debug, LOG_ERROR,
                    "Error: SETSOCKOPT SO_REUSEADDR\n");
        (void)close(s);
        return -1;
    }
    if (bind(s, &sat.sa, sin_len) < 0) {
        gpsd_report(context.debug, LOG_ERROR,
                    "can't bind to %s port %s, %s\n", af_str,
                    service, strerror(errno));
        if (errno == EADDRINUSE) {
            gpsd_report(context.debug, LOG_ERROR,
                        "maybe gpsd is already running!\n");
        }
        (void)close(s);
        return -1;
    }
    if (type == SOCK_STREAM && listen(s, qlen) == -1) {
        gpsd_report(context.debug, LOG_ERROR, "can't listen on port %s\n", service);
        (void)close(s);
        return -1;
    }

    if(bcast && (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int))) < 0) {
        gpsd_report(context.debug, LOG_ERROR, "can't broadcast on port %s\n", service);
        (void)close(s);
        return -1;
    }

    gpsd_report(context.debug, LOG_SPIN, "passivesock_af() -> %d\n", s);
    return s;
    /*@ +mustfreefresh -matchanyintegral @*/
}

/* *INDENT-OFF* */
static int passivesocks(char *service, char *tcp_or_udp,
    int qlen, /*@out@*/socket_t socks[])
{
    int numsocks = AFCOUNT;
    int i;

    for (i = 0; i < AFCOUNT; i++)
    INVALIDATE_SOCKET(socks[i]);

#if defined(SYSTEMD_ENABLE)
    if (sd_socket_count > 0) {
        for (i = 0; i < AFCOUNT && i < sd_socket_count - 1; i++) {
            socks[i] = SD_SOCKET_FDS_START + i + 1;
        }
        return sd_socket_count - 1;
    }
#endif

    if (AF_UNSPEC == af || (AF_INET == af))
        socks[0] = passivesock_af(AF_INET, service, tcp_or_udp, NULL, qlen);

    if (AF_UNSPEC == af || (AF_INET6 == af))
        socks[1] = passivesock_af(AF_INET6, service, tcp_or_udp, NULL, qlen);

    for (i = 0; i < AFCOUNT; i++)
    if (socks[i] < 0)
        numsocks--;

    /* Return the number of succesfully opened sockets
     * The failed ones are identified by negative values */
    return numsocks;
}
/* *INDENT-ON* */

static struct interface_t interfaces[MAXINTERFACES];

/*
 * Opens the udp socks that are being written to.
 * (as opposed to UDB device type reading sockets)
 *
 * Currently only: UDP broadcast, all ipv4 interfaces if global,
 * no ipv6, even if that would make more sense on ip4 bridged interfaces
 */

static int udpsocks(void)
{
    int yes = 1, status = 0;
    socket_t sock;

    struct interface_t * it = NULL;
    for (it = interfaces; it < interfaces + MAXINTERFACES; it++) {
        if((it->name[0] != '\0')
           && (strcmp(it->proto, "udp") == 0) && (it->port > 0)) {

            sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0 ) {
                gpsd_report(context.debug, LOG_ERROR,
                            "UDP broadcast opening socket failed %s\n", strerror(errno));
                return -1;
            }

            status = bind(sock, (struct sockaddr *)&it->bcast, sizeof(struct sockaddr_in));
            if(status < 0) {
                gpsd_report(context.debug, LOG_ERROR,
                            "UDP broadcast bind failed with %s\n",
                            strerror(errno));
                return -1;
            }

            status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );
            if(status < 0) {
                gpsd_report(context.debug, LOG_ERROR,
                            "UDP broadcast setsockopt failed with %s\n",
                            strerror(errno));
                return -1;
            }
            it->sock = sock;
        }
    }

    return 0;
}

struct subscriber_t
{
    int fd;			/* client file descriptor. -1 if unused */
    timestamp_t active;		/* when subscriber last polled for data */
    struct policy_t policy;	/* configurable bits */
    pthread_mutex_t mutex;	/* serialize access to fd */

    enum wsState state;
    enum wsFrameType frameType;
};
ssize_t throttled_write(struct subscriber_t *sub, const char *buf, size_t len);

#define LIMITED_MAX_CLIENTS 16
#ifdef LIMITED_MAX_CLIENTS
#define MAXSUBSCRIBERS LIMITED_MAX_CLIENTS
#else
#error
#endif

#define subscribed(sub, devp)    (sub->policy.watcher && (sub->policy.devpath[0]=='\0' || strcmp(sub->policy.devpath, devp->gpsdata.dev.path)==0))

static struct subscriber_t subscribers[MAXSUBSCRIBERS];	/* indexed by client file descriptor */

#define UNALLOCATED_FD	-1

static void lock_subscriber(struct subscriber_t *sub)
{
    (void)pthread_mutex_lock(&sub->mutex);
}

static void unlock_subscriber(struct subscriber_t *sub)
{
    (void)pthread_mutex_unlock(&sub->mutex);
}

static /*@null@*//*@observer@ */ struct subscriber_t *allocate_client(void)
/* return the address of a subscriber structure allocated for a new session */
{
    int si;

#if UNALLOCATED_FD == 0
#error client allocation code will fail horribly
#endif
    for (si = 0; si < NITEMS(subscribers); si++) {
        if (subscribers[si].fd == UNALLOCATED_FD) {
            subscribers[si].fd = 0;	/* mark subscriber as allocated */

            subscribers[si].policy.raw       = false;
            subscribers[si].policy.nmea      = false;
            subscribers[si].policy.canboat   = false;
            subscribers[si].policy.watcher   = true;
            subscribers[si].policy.json      = false;
            subscribers[si].policy.signalk   = false;
            subscribers[si].policy.protocol  = tcp;
            subscribers[si].policy.loglevel  = LOG_ERROR - 1;

            subscribers[si].state = WS_STATE_OPENING;
            subscribers[si].frameType = WS_INCOMPLETE_FRAME;
            return &subscribers[si];
        }
    }
    return NULL;
}

static void detach_client(struct subscriber_t *sub)
/* detach a client and terminate the session */
{
    char *c_ip;
    lock_subscriber(sub);
    if (sub->fd == UNALLOCATED_FD) {
    unlock_subscriber(sub);
    return;
    }
    c_ip = netlib_sock2ip(sub->fd);
    (void)shutdown(sub->fd, SHUT_RDWR);
    gpsd_report(context.debug, LOG_SPIN,
    "close(%d) in detach_client()\n",
    sub->fd);
    (void)close(sub->fd);
    gpsd_report(context.debug, LOG_INF,
    "detaching %s (sub %d, fd %d) in detach_client\n",
    c_ip, sub_index(sub), sub->fd);
    FD_CLR(sub->fd, &all_fds);
    adjust_max_fd(sub->fd, false);
    sub->active         = (timestamp_t)0;
    sub->policy.watcher = false;
    sub->policy.json    = false;
    sub->policy.signalk = false;
    sub->policy.nmea    = false;
    sub->policy.canboat = false;
    sub->policy.raw     = 0;
    sub->policy.scaled  = false;
    sub->policy.timing  = false;
    sub->policy.split24 = false;
    sub->policy.protocol = tcp;
    sub->policy.loglevel = LOG_ERROR - 1;
    sub->policy.devpath[0] = '\0';

    // websocket & http specific
    sub->state = WS_STATE_OPENING;
    sub->frameType = WS_INCOMPLETE_FRAME;

    sub->fd = UNALLOCATED_FD;
    unlock_subscriber(sub);
    set_max_subscriber_loglevel();
    /*@+mustfreeonly@*/
}

static bool isWebsocket(struct subscriber_t *sub) {
    return (sub->policy.protocol == websocket);
}

static const char * getProtocolName(struct subscriber_t *sub) {
    char *protocol_table[] = {
        "tcp", "ws", "http"
    };
    if((sub->policy.protocol > 0) && (sub->policy.protocol <= http))
        return protocol_table[sub->policy.protocol];
    return "";
}

static ssize_t throttled_write_(struct subscriber_t *sub, const char *buf,
           size_t len)
/* write to client -- throttle if it's gone or we're close to buffer overrun */
{
    ssize_t status;

    if (context.debug >= LOG_RAW) {
        if (isprint(buf[0]))
            gpsd_report(context.debug, LOG_CLIENT,
                        "=> %sclient(%d): %s\n",
                        getProtocolName(sub),
                        sub_index(sub),
                        buf);
        else {
            const char *cp; char buf2[MAX_PACKET_LENGTH * 3];
            buf2[0] = '\0';
            for (cp = buf; cp < buf + len; cp++)
                (void)snprintf(buf2 + strlen(buf2),
                               sizeof(buf2) - strlen(buf2),
                               "%02x", (unsigned int)(*cp & 0xff));
            gpsd_report(context.debug, LOG_CLIENT,
                        "===> %sclient(%d): %s\n",
                        getProtocolName(sub),
                        sub_index(sub),
                        buf2);
        }
    }

#if defined(PPS_ENABLE)
    gpsd_acquire_reporting_lock();
#endif /* PPS_ENABLE */
    status = send(sub->fd, buf, len, 0);
#if defined(PPS_ENABLE)
    gpsd_release_reporting_lock();

#endif /* PPS_ENABLE */
    if (status == (ssize_t) len)
        return status;
    else if (status > -1) {
        gpsd_report(context.debug, LOG_INF,
                    "short write disconnecting client(%d)\n",
                    sub_index(sub));
        detach_client(sub);
        return 0;
    } else if (errno == EAGAIN || errno == EINTR)
        return 0;		/* no data written, and errno says to retry */
    else if (errno == EBADF)
        gpsd_report(context.debug, LOG_WARN, "client(%d) has vanished.\n", sub_index(sub));
    else if (errno == EWOULDBLOCK
             && timestamp() - sub->active > NOREAD_TIMEOUT)
        gpsd_report(context.debug, LOG_INF,
                    "client(%d) timed out.\n", sub_index(sub));
    else
        gpsd_report(context.debug, LOG_INF,
                    "client(%d) write: %s\n",
                    sub_index(sub), strerror(errno));
    detach_client(sub);
    return status;
}

ssize_t throttled_write(struct subscriber_t *sub, const char *buf,
           size_t len) {
    if(isWebsocket(sub)) {
      uint8_t reply[GPS_JSON_RESPONSE_MAX + 1];
      size_t replylen = GPS_JSON_RESPONSE_MAX;
      wsMakeFrame(buf, len, reply, &replylen, WS_TEXT_FRAME);
      return throttled_write_(sub, reply, replylen);
    } else
      return throttled_write_(sub, buf, len);
}

static void set_max_subscriber_loglevel() {

  int dl = 0;

  struct subscriber_t *sub;
  for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    /*@-nullderef@*/
    if (sub == NULL || sub->active == 0)
      continue;

    if(dl < sub->policy.loglevel)
      dl = sub->policy.loglevel;
  }

  max_subscriber_loglevel = dl;
}


void gpsd_throttled_report(const int errlevel, const char * buf) {

  struct subscriber_t *sub;
  for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    /*@-nullderef@*/
    if (sub == NULL || sub->active == 0)
      continue;

    if(errlevel <= sub->policy.loglevel) {
      (void)throttled_write(sub, buf, strlen(buf));
    }
  }
}

static void notify_watchers(struct gps_device_t *device,
        bool onjson, bool onpps,
        const char *sentence, ...)
/* notify all JSON-watching clients of a given device about an event */
{
    va_list ap;
    char buf[BUFSIZ];
    struct subscriber_t *sub;

    va_start(ap, sentence);
    (void)vsnprintf(buf, sizeof(buf), sentence, ap);
    va_end(ap);

    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++)
    if (sub->active != 0 && subscribed(sub, device)) {
        if ((onjson && sub->policy.json) || (onpps && sub->policy.pps))
    (void)throttled_write(sub, buf, strlen(buf));
    }
}
#endif /* SOCKET_EXPORT_ENABLE */

static void deactivate_device(struct gps_device_t *device)
/* deactivate device, but leave it in the pool (do not free it) */
{
#ifdef SOCKET_EXPORT_ENABLE
    notify_watchers(device, true, false,
        "{\"class\":\"DEVICE\",\"path\":\"%s\",\"activated\":0}\r\n",
        device->gpsdata.dev.path);
#endif /* SOCKET_EXPORT_ENABLE */
    if (!BAD_SOCKET(device->gpsdata.gps_fd)) {
    FD_CLR(device->gpsdata.gps_fd, &all_fds);
    adjust_max_fd(device->gpsdata.gps_fd, false);
#if defined(PPS_ENABLE) && defined(TIOCMIWAIT)
#endif /* defined(PPS_ENABLE) && defined(TIOCMIWAIT) */
#ifdef NTPSHM_ENABLE
    ntpshm_link_deactivate(device);
#endif /* NTPSHM_ENABLE */
    gpsd_deactivate(device);
    }
}

#if defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE)
/* *INDENT-OFF* */
/*@null@*//*@observer@*/ static struct gps_device_t *find_device(/*@null@*/const char
     *device_name)
/* find the device block for an existing device name */
{
    struct gps_device_t *devp;

    for (devp = devices; devp < devices + MAXDEVICES; devp++)
    {
        if (allocated_device(devp) && NULL != device_name &&
            strcmp(devp->gpsdata.dev.path, device_name) == 0)
            return devp;
    }
    return NULL;
}
/* *INDENT-ON* */
#endif /* defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE) */

static bool open_device( /*@null@*/struct gps_device_t *device)
{
    if (NULL == device || gpsd_activate(device, O_OPTIMIZE) < 0) {
        return false;
    }

#ifdef NTPSHM_ENABLE
    /*
     * Now is the right time to grab the shared memory segment(s)
     * to communicate the navigation message derived and (possibly)
     * 1PPS derived time data to ntpd/chrony.
     */
    ntpshm_link_activate(device);
    gpsd_report(context.debug, LOG_INF,
    "NTPD ntpshm_link_activate: %d\n",
    (int)device->shmIndex >= 0);
#endif /* NTPSHM_ENABLE */

    gpsd_report(context.debug, LOG_INF,
    "device %s activated\n", device->gpsdata.dev.path);
    FD_SET(device->gpsdata.gps_fd, &all_fds);
    adjust_max_fd(device->gpsdata.gps_fd, true);
    return true;
}

bool gpsd_add_device(const char *device_name, bool flag_nowait)
/* add a device to the pool; open it right away if in nowait mode */
{
    struct gps_device_t *devp;
    bool ret = false;
    /* we can't handle paths longer than GPS_PATH_MAX, so don't try */
    if (strlen(device_name) >= GPS_PATH_MAX) {
    gpsd_report(context.debug, LOG_ERROR,
        "ignoring device %s: path length exceeds maximum %d\n",
        device_name, GPS_PATH_MAX);
    return false;
    }
    /* stash devicename away for probing when the first client connects */
    for (devp = devices; devp < devices + MAXDEVICES; devp++)
        if (!allocated_device(devp)) {
            gpsd_init(devp, &context, device_name);
#ifdef NTPSHM_ENABLE
            ntpshm_session_init(devp);
#endif /* NTPSHM_ENABLE */

            gpsd_report(context.debug, LOG_INF,
                        "stashing device %s at slot %d\n",
                        device_name, (int)(devp - devices));
            if (!flag_nowait) {
                devp->gpsdata.gps_fd = UNALLOCATED_FD;
                ret = true;
            } else {
                ret = open_device(devp);
            }
#ifdef SOCKET_EXPORT_ENABLE
            notify_watchers(devp, true, false,
                            "{\"class\":\"DEVICE\",\"path\":\"%s\",\"activated\":%lf}\r\n",
                            devp->gpsdata.dev.path, timestamp());
#endif /* SOCKET_EXPORT_ENABLE */
            break;
    }
    return ret;
}

#ifdef CONTROL_SOCKET_ENABLE
/*@ observer @*/ static char *snarfline(char *p, /*@out@*/ char **out)
/* copy the rest of the command line, before CR-LF */
{
    char *q;
    static char stash[BUFSIZ];

    /*@ -temptrans -mayaliasunique @*/
    for (q = p; isprint(*p) && !isspace(*p) && /*@i@*/ (p - q < BUFSIZ - 1);
     p++)
    continue;
    (void)memcpy(stash, q, (size_t) (p - q));
    stash[p - q] = '\0';
    *out = stash;
    return p;
    /*@ +temptrans +mayaliasunique @*/
}

static void handle_control(int sfd, char *buf)
/* handle privileged commands coming through the control socket */
{
    char *stash;
    struct gps_device_t *devp;

     /*
      * The only other place in the code that knows about the format
      * of the + and - commands is the gpsd_control() function in
      * gpsdctl.c.  Be careful about keeping them in sync, or
      * hotplugging will have mysterious failures.
      */
    /*@ -sefparams @*/
    if (buf[0] == '-') {
    /* remove device named after - */
    (void)snarfline(buf + 1, &stash);
    gpsd_report(context.debug, LOG_INF,
        "<= control(%d): removing %s\n", sfd, stash);
    if ((devp = find_device(stash))) {
        deactivate_device(devp);
        free_device(devp);
        ignore_return(write(sfd, "OK\n", 3));
    } else
        ignore_return(write(sfd, "ERROR\n", 6));
    } else if (buf[0] == '+') {
    /* add device named after + */
    (void)snarfline(buf + 1, &stash);
    if (find_device(stash)) {
        gpsd_report(context.debug, LOG_INF,
    "<= control(%d): %s already active \n", sfd,
    stash);
        ignore_return(write(sfd, "ERROR\n", 6));
    } else {
        gpsd_report(context.debug, LOG_INF,
    "<= control(%d): adding %s\n", sfd, stash);
        if (gpsd_add_device(stash, NOWAIT))
    ignore_return(write(sfd, "OK\n", 3));
        else
    ignore_return(write(sfd, "ERROR\n", 6));
    }
    } else if (buf[0] == '!') {
    /* split line after ! into device=string, send string to device */
    char *eq;
    (void)snarfline(buf + 1, &stash);
    eq = strchr(stash, '=');
    if (eq == NULL) {
        gpsd_report(context.debug, LOG_WARN,
    "<= control(%d): ill-formed command \n",
    sfd);
        ignore_return(write(sfd, "ERROR\n", 6));
    } else {
        *eq++ = '\0';
        if ((devp = find_device(stash))) {
    if (devp->context->readonly || (devp->sourcetype <= source_blockdev)) {
        gpsd_report(context.debug, LOG_WARN,
    "<= control(%d): attempted to write to a read-only device\n",
    sfd);
        ignore_return(write(sfd, "ERROR\n", 6));
    } else {
        gpsd_report(context.debug, LOG_INF,
    "<= control(%d): writing to %s \n", sfd,
    stash);
        if (write(devp->gpsdata.gps_fd, eq, strlen(eq)) <= 0) {
    gpsd_report(context.debug, LOG_WARN, "<= control(%d): write to device failed\n",
        sfd);
    ignore_return(write(sfd, "ERROR\n", 6));
        } else {
    ignore_return(write(sfd, "OK\n", 3));
        }
    }
        } else {
    gpsd_report(context.debug, LOG_INF,
        "<= control(%d): %s not active \n", sfd,
        stash);
    ignore_return(write(sfd, "ERROR\n", 6));
        }
    }
    } else if (buf[0] == '&') {
    /* split line after & into dev=hexdata, send unpacked hexdata to dev */
    char *eq;
    (void)snarfline(buf + 1, &stash);
    eq = strchr(stash, '=');
    if (eq == NULL) {
        gpsd_report(context.debug, LOG_WARN,
    "<= control(%d): ill-formed command\n",
    sfd);
        ignore_return(write(sfd, "ERROR\n", 6));
    } else {
        size_t len;
        *eq++ = '\0';
        len = strlen(eq) + 5;
        if ((devp = find_device(stash)) != NULL) {
    if (devp->context->readonly || (devp->sourcetype <= source_blockdev)) {
        gpsd_report(context.debug, LOG_WARN,
    "<= control(%d): attempted to write to a read-only device\n",
    sfd);
        ignore_return(write(sfd, "ERROR\n", 6));
                } else {
        int st;
                    /* NOTE: this destroys the original buffer contents */
                    st = gpsd_hexpack(eq, eq, len);
                    if (st <= 0) {
                        gpsd_report(context.debug, LOG_INF,
                                    "<= control(%d): invalid hex string (error %d).\n",
                                    sfd, st);
                        ignore_return(write(sfd, "ERROR\n", 6));
                    } else {
                        gpsd_report(context.debug, LOG_INF,
                                    "<= control(%d): writing %d bytes fromhex(%s) to %s\n",
                                    sfd, st, eq, stash);
                        if (write(devp->gpsdata.gps_fd, eq, (size_t) st) <= 0) {
                            gpsd_report(context.debug, LOG_WARN,
    "<= control(%d): write to device failed\n",
                                        sfd);
                            ignore_return(write(sfd, "ERROR\n", 6));
                        } else {
                            ignore_return(write(sfd, "OK\n", 3));
                        }
                    }
    }
        } else {
    gpsd_report(context.debug, LOG_INF,
        "<= control(%d): %s not active\n", sfd,
        stash);
    ignore_return(write(sfd, "ERROR\n", 6));
        }
    }
    } else if (strstr(buf, "?devices")==buf) {
    /* write back devices list followed by OK */
    for (devp = devices; devp < devices + MAXDEVICES; devp++) {
        char *path = devp->gpsdata.dev.path;
        ignore_return(write(sfd, path, strlen(path)));
        ignore_return(write(sfd, "\n", 1));
    }
    ignore_return(write(sfd, "OK\n", 3));
    } else {
    /* unknown command */
    ignore_return(write(sfd, "ERROR\n", 6));
    }
    /*@ +sefparams @*/
}
#endif /* CONTROL_SOCKET_ENABLE */

#ifdef SOCKET_EXPORT_ENABLE
static bool awaken(struct gps_device_t *device)
/* awaken a device and notify all watchers*/
{
    /* open that device */
    if (!initialized_device(device)) {
    if (!open_device(device)) {
        gpsd_report(context.debug, LOG_WARN,
    "%s: open failed\n",
    device->gpsdata.dev.path);
        free_device(device);
        return false;
    }
    }

    if (!BAD_SOCKET(device->gpsdata.gps_fd)) {
    gpsd_report(context.debug, LOG_PROG,
        "device %d (fd=%d, path %s) already active.\n",
        (int)(device - devices),
        device->gpsdata.gps_fd, device->gpsdata.dev.path);
    return true;
    } else {
    if (gpsd_activate(device, O_OPTIMIZE) < 0) {
        gpsd_report(context.debug, LOG_ERROR,
    "%s: device activation failed.\n",
    device->gpsdata.dev.path);
        return false;
    } else {
        gpsd_report(context.debug, LOG_RAW,
    "flagging descriptor %d in assign_channel()\n",
    device->gpsdata.gps_fd);
        FD_SET(device->gpsdata.gps_fd, &all_fds);
        adjust_max_fd(device->gpsdata.gps_fd, true);
        return true;
    }
    }
}

#ifdef RECONFIGURE_ENABLE
static bool privileged_user(struct gps_device_t *device)
/* is this channel privileged to change a device's behavior? */
{
    /* grant user privilege if he's the only one listening to the device */
    struct subscriber_t *sub;
    int subcount = 0;
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    if (subscribed(sub, device))
        subcount++;
    }
    /*
     * Yes, zero subscribers is possible. For example, gpsctl talking
     * to the daemon connects but doesn't necessarily issue a ?WATCH
     * before shipping a request, which means it isn't marked as a
     * subscribers,
     */
    return subcount <= 1;
}

static void set_serial(struct gps_device_t *device,
           speed_t speed, char *modestring)
/* set serial parameters for a device from a speed and modestring */
{
    unsigned int stopbits = device->gpsdata.dev.stopbits;
    char parity = device->gpsdata.dev.parity;
    int wordsize = 8;

    if (strchr("78", *modestring) != NULL) {
    while (isspace(*modestring))
        modestring++;
    wordsize = (int)(*modestring++ - '0');
    if (strchr("NOE", *modestring) != NULL) {
        parity = *modestring++;
        while (isspace(*modestring))
    modestring++;
        if (strchr("12", *modestring) != NULL)
    stopbits = (unsigned int)(*modestring - '0');
    }
    }

    gpsd_report(context.debug, LOG_PROG,
    "set_serial(%s,%u,%s) %c%d\n",
    device->gpsdata.dev.path,
    (unsigned int)speed, modestring, parity, stopbits);
    /* no support for other word sizes yet */
    /* *INDENT-OFF* */
    if (wordsize == (int)(9 - stopbits)
    && device->device_type->speed_switcher != NULL) {
    if (device->device_type->speed_switcher(device, speed, parity, (int)stopbits)) {
        /*
         * Deep black magic is required here. We have to
         * allow the control string time to register at the
         * GPS before we do the baud rate switch, which
         * effectively trashes the UART's buffer.
         *
         * This definitely fails below 40 milliseconds on a
         * BU-303b. 50ms is also verified by Chris Kuethe on
         *  Pharos iGPS360 + GSW 2.3.1ES + prolific
         *  Rayming TN-200 + GSW 2.3.1 + ftdi
         *  Rayming TN-200 + GSW 2.3.2 + ftdi
         * so it looks pretty solid.
         *
         * The minimum delay time is probably constant
         * across any given type of UART.
         */
        (void)tcdrain(device->gpsdata.gps_fd);
        (void)usleep(50000);
        gpsd_set_speed(device, speed, parity, stopbits);
    }
    }
    /* *INDENT-ON* */
}
#endif /* RECONFIGURE_ENABLE */

#ifdef SOCKET_EXPORT_ENABLE
static void json_devicelist_dump(char *reply, size_t replylen)
{
    struct gps_device_t *devp;
    (void)strlcpy(reply, "{\"class\":\"DEVICES\",\"devices\":[", replylen);
    for (devp = devices; devp < devices + MAXDEVICES; devp++)
    if (allocated_device(devp)
        && strlen(reply) + strlen(devp->gpsdata.dev.path) + 3 <
        replylen - 1) {
        char *cp;
        json_device_dump(devp,
         reply + strlen(reply), replylen - strlen(reply));
        cp = reply + strlen(reply);
        *--cp = '\0';
        *--cp = '\0';
        (void)strlcat(reply, ",", replylen);
    }

    if (reply[strlen(reply) - 1] == ',')
        reply[strlen(reply) - 1] = '\0';
    (void)strlcat(reply, "]}\r\n", replylen);
}
#endif /* SOCKET_EXPORT_ENABLE */

static void rstrip(char *str)
/* strip trailing \r\n\t\SP from a string */
{
    char *strend;
    strend = str + strlen(str) - 1;
    while (isspace(*strend)) {
        *strend = '\0';
        --strend;
    }
}

/*
 *  Forward rules
 *
 *  Most of this is for backwards compatibility were we only forwarded
 *  pseudo (translated) sentences to NMEA out.
 *
 *  1. Unkown devices (NULL) such as from wifi will be forwarded
 *     (we'll invent rules for that later on)
 *
 *  2. Legacy source devices (without port policies) will be forwarded
 *     if they are translated (backwards compatible)
 *
 *  3. VYSPI sources and destinations will be forwarded
 *     (VYSPI as source covered by 2. already)
 *
 *  4. At this stage of this filter chain only NMEA source devices should be left:
 *     reject all destination devices that don't have port policies
 *     (again backwards compatibility where we only forwarded translated/pseudo sentences)
 *
 *  5. check actual forward policies
 */
static int gpsd_device_forward(struct gps_device_t * srcdev,
       struct gps_device_t * destdev) {
    // 1.
    if(!srcdev)
    return 1;

    // 2.
    if(srcdev->gpsdata.dev.port_count == 0)
        if(srcdev->device_type->packet_type != NMEA_PACKET)
            return 1;

    // 3.
    if((srcdev->device_type->packet_type == VYSPI_PACKET)
       || (destdev->device_type->packet_type == VYSPI_PACKET)) {
        return 1;
    }

    if(destdev->gpsdata.dev.port_count == 0)
        return 0;

    // don't need to go through loop below if there are no policies
    if(srcdev->gpsdata.dev.port_count == 0)
        return 0;

    uint8_t d = 0;
    while(d < MAXDEVICES) {
    if(destdev && strcmp(srcdev->gpsdata.dev.portlist[0].forward[d],
     destdev->gpsdata.dev.portlist[0].name) == 0)
    return 1;
        d++;
    }

    return 0;
}

/*
 *  Reject rules
 *
 *  This includes also backwards compatibility for cases where didn't have
 *  reject and accept policies at all.
 *
 *  1. Don't reject write to output if there is no rule (backwards compatibility)
 *  2. Reject only to a certain device if all its ports have output rejected
 *     In the the case of 2 or more ports the device driver itself (VYSPI) needs to decide.
 */
static int gpsd_device_rejects(struct gps_device_t * devp) {

    uint8_t n = 0;
    uint8_t rejects = 0;

    if(devp->gpsdata.dev.port_count == 0)
        return 0;

    for(n = 0; n < devp->gpsdata.dev.port_count; n++) {
        if(devp->gpsdata.dev.portlist[n].output == device_policy_reject) {
            rejects++;
        }
    }

    if(rejects == devp->gpsdata.dev.port_count) {
        return 1;
    }

    return 0;
}

static void gpsd_device_write(struct gps_device_t * srcdev,
                              enum frm_type_t frm_type,
      const char *buf, size_t len) {

    struct gps_device_t *devp;
    for (devp = devices; devp < devices + MAXDEVICES; devp++) {

    if (allocated_device(devp)) {

    if(gpsd_device_rejects(devp)) {
                gpsd_report(context.debug, LOG_RAW,
                            "gpsd_write to device %s rejected\n", devp->gpsdata.dev.path);
                continue;
            }

    gpsd_report(context.debug, LOG_RAW,
    "gpsd_write to device %s accepted\n", devp->gpsdata.dev.path);

    const struct gps_type_t *dt = devp->device_type;
    if(dt == NULL) continue;

    if(gpsd_device_forward(srcdev, devp)) {

                if(dt->packet_type == VYSPI_PACKET) {

                    if(!devp->context->readonly || (frm_type == FRM_TYPE_NMEA2000))
                        (void)vyspi_write(devp, frm_type, buf, (size_t)len);

                } else if(dt->packet_type == NMEA_PACKET) {

    (void)gpsd_write(devp, buf, (size_t)len);
    gpsd_report(context.debug, LOG_IO,
    "gpsd_write: %s (%s > %s)\n", buf,
    srcdev->gpsdata.dev.path, devp->gpsdata.dev.path);

    }
    }
    }
    }
}

static void gpsd_udp_write(const char *buf, size_t len) {

    struct interface_t * it;
    for (it = interfaces; it < interfaces + MAXINTERFACES; it++) {

        if( (it->name[0] != '\0')
           && (strcmp(it->proto, "udp") == 0) ) {

            if(sendto(it->sock, buf, len, 0,
                      (struct sockaddr *)&it->bcast, sizeof(struct sockaddr_in)) < 0) {

                gpsd_report(context.debug, LOG_ERROR,
                            "gpsd_udp_write: %s (%s, %d) failed (%s).\n", buf,
                            it->name, it->port,
                            strerror(errno));

            }
            // TODO IP ADDR
            gpsd_report(context.debug, LOG_IO,
                        "gpsd_udp_write: %s (%lu)\n", buf, len);
        }
    }

}

static ssize_t handle_websocket_request(struct subscriber_t *sub,
    const char *buf,
    char *reply, size_t replylen)
{
    uint8_t *data = NULL;
    size_t dataSize = 0;
    size_t len = 0;

    struct handshake hs;
    nullHandshake(&hs);

//    gpsd_report(context.debug, LOG_INF, "incomming frame: %s\n", buf);

    if (sub->state == WS_STATE_OPENING) {
        gpsd_report(context.debug, LOG_INF,
                    "Handling a HTTP handshake.\n");
        sub->frameType = wsParseHandshake((uint8_t *)buf, 0, &hs);
        if(sub->frameType == WS_PREFLIGHTED_FRAME) {

            // TODO preper timestamp
            len = snprintf(reply, replylen,
                           "HTTP/1.1 204 No Content\r\n"
                           "X-Powered-By: Express\r\n"
                           "Access-Control-Allow-Origin: *\r\n"
                           "Access-Control-Allow-Methods: GET,HEAD,PUT,PATCH,POST,DELETE\r\n"
                           "Access-Control-Allow-Headers: content-type\r\n"
                           "Date: Wed, 20 Jan 2016 12:39:21 GMT\r\n"
                           "Connection: keep-alive\r\n\r\n");

            gpsd_report(context.debug, LOG_INF,
                        "returning OPTIONS: %s\n", reply);
            return throttled_write(sub, reply, len);
        }
    } else {
        sub->frameType = wsParseInputFrame(buf, 0, &data, &dataSize);
        gpsd_report(context.debug, LOG_INF,
                    "incoming frame with %s\n", data);
    }

    if (sub->frameType == WS_INCOMPLETE_FRAME) {
        gpsd_report(context.debug, LOG_ERROR,
                    "Incomplete frame or buffer too small\n");
        return 0;
    }

    if(sub->frameType == WS_ERROR_FRAME) {
        gpsd_report(context.debug, LOG_ERROR,
                    "Error in incoming frame\n");

        if (sub->state == WS_STATE_OPENING) {
            len = snprintf(reply, replylen,
                           "HTTP/1.1 400 Bad Request\r\n"
                           "%s%s\r\n\r\n",
                           versionField,
                           version);
            sub->frameType = WS_INCOMPLETE_FRAME;
            return throttled_write(sub, reply, len);

        } else {
            wsMakeFrame(NULL, 0, reply, &len, WS_CLOSING_FRAME);
            sub->state = WS_STATE_CLOSING;
            sub->frameType = WS_INCOMPLETE_FRAME;
            return throttled_write(sub, reply, len);
        }
    }

    if (sub->state == WS_STATE_OPENING) {
        assert((sub->frameType == WS_OPENING_FRAME)
               || (sub->frameType == WS_GET_FRAME));
        if ((sub->frameType == WS_OPENING_FRAME)
            || (sub->frameType == WS_GET_FRAME)) {

            bool raw     = 0;
            bool nmea    = false;
            bool signalk = false;
            bool track   = false;
            int debug    = 0;
            uint32_t startAfter = 0;
            char field[255];
            uint8_t pcnt = 0;

            gpsd_report(context.debug, LOG_INF,
                        "incoming resource request with %s\n", hs.resource);

            while((hs.params[pcnt].param[0] != '\0')
                  && (pcnt < WS_MAX_PARAM_NO)) {
                gpsd_report(context.debug, LOG_INF,
                            "parameter %s = %s\n",
                            hs.params[pcnt].param,
                            hs.params[pcnt].value);
                if(strncmp(hs.params[pcnt].param, "track", 5) == 0)
                    track = true;
                if(strncmp(hs.params[pcnt].param, "startAfter", 10) == 0)
                    startAfter = atol(hs.params[pcnt].value);
                if(strncmp(hs.params[pcnt].param, "field", 10) == 0)
                    strncpy(field, hs.params[pcnt].value, 254);
                pcnt++;
            }

            // if resource is right, generate answer handshake and send it
            if (strncmp(hs.resource, "/signalk", 8) == 0) {

                signalk = true;


            } else if (strcmp(hs.resource, "/raw") == 0) {
                raw = 1;
                nmea = true;
            } else if (strncmp(hs.resource, "/debug", 6) == 0) {

                debug = 5;
                if(strlen(hs.resource) >= 14) {
                    if(strncmp(hs.resource + 6, "?level=", 7) == 0) {
                        debug = atoi(hs.resource + 13);
                        gpsd_report(context.debug, LOG_INF,
                                    "incoming resource request with loglevel %d\n",
                                    debug);
                    }
                }


            } else {
                gpsd_report(context.debug, LOG_INF,
                            "404 Not Found: %s\n", hs.resource);
                len = snprintf((char *)reply, replylen,
                               "HTTP/1.1 404 Not Found\r\n\r\n");
                throttled_write(sub, reply, len);
                return -1;
            }

            /* TODO: a new sub is in nmea == true and
               raw writes to sub might interfer with http protocol
               - investigate: introduce grace period for each new connection
                              if within a short time window after connect no
                              HTTP specific headers are send by client its safer
                              to assume raw TCP client

                              problem: if client sends nothing we'll never wake up on poll/select
                              timeouts for select/poll maybe? or wakeup poll?


               TODO: define a clear sub->sub state for http protocol

               TODO: distingusih between signalk full and update,
               currently even a single GET might lead to updates being send
               - or restrict updates to web sockets in report_signalk?
               - investigate: how is GET method defined wrt close after receiving result?

               TODO: assuming that GET requests usually open a new connection
               for each call despite keep-alive, browser will keep still connections alive
               we can thus quickly run out of connections if server doens't close them
            */

            sub->policy.json      = false;
            sub->policy.signalk   = signalk;
            sub->policy.nmea      = nmea;
            sub->policy.watcher   = true;
            sub->policy.raw       = raw;
            sub->policy.loglevel  = debug;
            set_max_subscriber_loglevel();

            if(sub->frameType == WS_GET_FRAME) {
                char content[GPS_JSON_RESPONSE_MAX - 256];
                size_t contentlen = GPS_JSON_RESPONSE_MAX - 256; // leaving headroom for header

                // TODO hier muss natürlich ein gemergter gesamt datensatz aller devices hin
                if(track)
                    signalk_track_dump(devices, startAfter, field, content, contentlen);
                else
                    signalk_full_dump(devices, &vessel, content, contentlen);

                len = snprintf(reply, replylen,
                               "HTTP/1.1 200 OK\r\n"
                               "Content-Length: %lu\r\n"
                               "Connection: close\r\n"
                               "Access-Control-Allow-Origin: *\r\n"
                               "Content-Type: application/json\r\n\r\n%s",
                               strlen(content), content);
                gpsd_report(context.debug, LOG_INF,
                            "returning GET (%lu): %s\n", replylen, reply);

                sub->policy.protocol  = http;

                return throttled_write(sub, reply, len);
            }


            len = replylen;
            wsGetHandshakeAnswer(&hs, reply, &len);
            freeHandshake(&hs);

            // careful: this needs to be send as tcp - not as a ws frame!
            ssize_t status = throttled_write(sub, reply, len);

            sub->policy.protocol  = websocket;

            sub->state = WS_STATE_NORMAL;
            sub->frameType = WS_INCOMPLETE_FRAME;
            gpsd_report(context.debug, LOG_INF,
                        "answering handshake to %sclient: %s\n",
                        "ws", reply);
            return status;
        }
    } else {
        if (sub->frameType == WS_CLOSING_FRAME) {
            sub->policy.protocol = tcp;
            gpsd_report(context.debug, LOG_INF, "closing frame\n");
            if (sub->state == WS_STATE_CLOSING) {
                return -1;
            } else {
                len = replylen;
                wsMakeFrame((const char *)NULL, 0, reply, &len, WS_CLOSING_FRAME);
                throttled_write(sub, reply, len);
                return -1;
            }
        } else if (sub->frameType == WS_TEXT_FRAME) {
            len = replylen;
            wsMakeFrame("echo", 6, reply, &len, WS_TEXT_FRAME);
            sub->frameType = WS_INCOMPLETE_FRAME;
            return 0; // throttled_write(sub, reply, len);
        }

        // we should never get here
        return -1;
    }

    return -1;
}

static void handle_request(struct subscriber_t *sub,
       const char *buf, const char **after,
       char *reply, size_t replylen)
{
    struct gps_device_t *devp;
    const char *end = NULL;

    /*
     * There's a splint limitation that parameters can be declared
     * @out@ or @null@ but not, apparently, both.  This collides with
     * the (admittedly tricky) way we use endptr. The workaround is to
     * declare it @null@ and use -compdef around the JSON reader calls.
     */
    /*@-compdef@*/
    /*
     * See above...
     */
    /*@-nullderef -nullpass@*/

    if (buf[0] == '?')
        ++buf;
    if (strncmp(buf, "DEVICES;", 8) == 0) {
        buf += 8;
        json_devicelist_dump(reply, replylen);
    } else if (strncmp(buf, "WATCH", 5) == 0
           && (buf[5] == ';' || buf[5] == '=')) {
        const char *start = buf;
        buf += 5;
        if (*buf == ';') {
            ++buf;
        } else {
            int status = json_watch_read(buf + 1, &sub->policy, &end);
#ifndef TIMING_ENABLE
            sub->policy.timing = false;
#endif /* TIMING_ENABLE */
            if (end == NULL)
                buf += strlen(buf);
            else {
                if (*end == ';')
                    ++end;
                buf = end;
            }
            if (status != 0) {
                (void)snprintf(reply, replylen,
                    "{\"class\":\"ERROR\",\"message\":\"Invalid WATCH: %s\"}\r\n",
                    json_error_string(status));
                gpsd_report(context.debug, LOG_ERROR, "response: %s\n", reply);
            } else if (sub->policy.watcher) {
                if (sub->policy.devpath[0] == '\0') {
                    /* awaken all devices */
                    for (devp = devices; devp < devices + MAXDEVICES; devp++)
                    if (allocated_device(devp)) {
                        (void)awaken(devp);
                        if (devp->sourcetype == source_gpsd) {
                            (void)gpsd_write(devp, "?", 1);
                            (void)gpsd_write(devp, start, (size_t)(end-start));
                        }
                    }
                } else {
                    devp = find_device(sub->policy.devpath);
                    if (devp == NULL) {
                        (void)snprintf(reply, replylen,
                            "{\"class\":\"ERROR\",\"message\":\"No such device as %s\"}\r\n",
                            sub->policy.devpath);
                        gpsd_report(context.debug, LOG_ERROR,
                            "response: %s\n", reply);
                        goto bailout;
                    } else if (awaken(devp)) {
                        if (devp->sourcetype == source_gpsd) {
                            (void)gpsd_write(devp, "?", 1);
                            (void)gpsd_write(devp, start, (size_t)(end-start));
                        }
                    } else {
                        (void)snprintf(reply, replylen,
                            "{\"class\":\"ERROR\",\"message\":\"Can't assign %s\"}\r\n",
                            sub->policy.devpath);
                        gpsd_report(context.debug, LOG_ERROR,
                            "response: %s\n", reply);
                        goto bailout;
                    }
                }
            }
        }
        /* display a device list and the user's policy */
        json_devicelist_dump(reply + strlen(reply), replylen - strlen(reply));
        json_watch_dump(&sub->policy,
            reply + strlen(reply), replylen - strlen(reply));

    } else if (strncmp(buf, "DEVICE", 6) == 0
               && (buf[6] == ';' || buf[6] == '=')) {
        struct devconfig_t devconf;
        buf += 6;
        devconf.path[0] = '\0';	/* initially, no device selection */
        if (*buf == ';') {
            ++buf;
        } else {
#ifdef RECONFIGURE_ENABLE
            struct gps_device_t *device;
            /* first, select a device to operate on */
            int status = json_device_read(buf + 1, &devconf, &end);
            if (end == NULL)
                buf += strlen(buf);
            else {
                if (*end == ';')
                    ++end;
                buf = end;
            }
            device = NULL;
            /*@-branchstate@*/
            if (status != 0) {
                (void)snprintf(reply, replylen,
                               "{\"class\":\"ERROR\",\"message\":\"Invalid DEVICE: \"%s\"}\r\n",
                               json_error_string(status));
                gpsd_report(context.debug, LOG_ERROR, "response: %s\n", reply);
                goto bailout;
        } else {
    if (devconf.path[0] != '\0') {
        /* user specified a path, try to assign it */
        device = find_device(devconf.path);
        /* do not optimize away, we need 'device' later on! */
        if (device == NULL || !awaken(device)) {
    (void)snprintf(reply, replylen,
           "{\"class\":\"ERROR\",\"message\":\"Can't open %s.\"}\r\n",
           devconf.path);
    gpsd_report(context.debug, LOG_ERROR,
        "response: %s\n", reply);
    goto bailout;
        }
    } else {
        /* no path specified */
        int devcount = 0;
        for (devp = devices; devp < devices + MAXDEVICES; devp++)
    if (allocated_device(devp)) {
        device = devp;
        devcount++;
    }
        if (devcount == 0) {
    (void)strlcat(reply,
          "{\"class\":\"ERROR\",\"message\":\"Can't perform DEVICE configuration, no devices attached.\"}\r\n",
          replylen);
    gpsd_report(context.debug, LOG_ERROR,
        "response: %s\n", reply);
    goto bailout;
        } else if (devcount > 1) {
    (void)snprintf(reply + strlen(reply),
           replylen - strlen(reply),
           "{\"class\":\"ERROR\",\"message\":\"No path specified in DEVICE, but multiple devices are attached.\"}\r\n");
    gpsd_report(context.debug, LOG_ERROR,
        "response: %s\n", reply);
    goto bailout;
        }
        /* we should have exactly one device now */
    }
    if (!privileged_user(device))
        (void)snprintf(reply + strlen(reply),
       replylen - strlen(reply),
       "{\"class\":\"ERROR\",\"message\":\"Multiple subscribers, cannot change control bits on %s.\"}\r\n",
       device->gpsdata.dev.path);
    else if (device->device_type == NULL)
        (void)snprintf(reply + strlen(reply),
       replylen - strlen(reply),
       "{\"class\":\"ERROR\",\"message\":\"Type of %s is unknown.\"}\r\n",
       device->gpsdata.dev.path);
    else {
        const struct gps_type_t *dt = device->device_type;
        bool no_serial_change =
    (devconf.baudrate == DEVDEFAULT_BPS)
    && (devconf.parity == DEVDEFAULT_PARITY)
    && (devconf.stopbits == DEVDEFAULT_STOPBITS);

        /* interpret defaults */
        if (devconf.baudrate == DEVDEFAULT_BPS)
    devconf.baudrate =
        (uint) gpsd_get_speed(device);
        if (devconf.parity == DEVDEFAULT_PARITY)
    devconf.stopbits = device->gpsdata.dev.stopbits;
        if (devconf.stopbits == DEVDEFAULT_STOPBITS)
    devconf.stopbits = device->gpsdata.dev.stopbits;
        if (isnan(devconf.cycle))
    devconf.cycle = device->gpsdata.dev.cycle;

        /* now that channel is selected, apply changes */
        if (devconf.driver_mode != device->gpsdata.dev.driver_mode
    && devconf.driver_mode != DEVDEFAULT_NATIVE
    && dt->mode_switcher != NULL)
    dt->mode_switcher(device, devconf.driver_mode);
        if (!no_serial_change) {
    char serialmode[3];
    serialmode[0] = devconf.parity;
    serialmode[1] = '0' + devconf.stopbits;
    serialmode[2] = '\0';
    set_serial(device,
       (speed_t) devconf.baudrate, serialmode);
        }
        if (devconf.cycle != device->gpsdata.dev.cycle
    && devconf.cycle >= dt->min_cycle
    && dt->rate_switcher != NULL)
    if (dt->rate_switcher(device, devconf.cycle))
        device->gpsdata.dev.cycle = devconf.cycle;
    }
        }
        /*@+branchstate@*/
#else /* RECONFIGURE_ENABLE */
        (void)snprintf(reply + strlen(reply), replylen - strlen(reply),
       "{\"class\":\"ERROR\",\"message\":\"Device configuration support not compiled.\"}\r\n");
#endif /* RECONFIGURE_ENABLE */
    }
    /* dump a response for each selected channel */
    for (devp = devices; devp < devices + MAXDEVICES; devp++)
        if (!allocated_device(devp))
            continue;
        else if (devconf.path[0] != '\0' && devp != NULL
            && strcmp(devp->gpsdata.dev.path, devconf.path) != 0)
            continue;
        else {
            json_device_dump(devp,
                reply + strlen(reply),
                replylen - strlen(reply));
        }
    } else if (strncmp(buf, "POLL;", 5) == 0) {
        char tbuf[JSON_DATE_MAX+1];
        int active = 0;
        buf += 5;
        for (devp = devices; devp < devices + MAXDEVICES; devp++)
            if (allocated_device(devp) && subscribed(sub, devp))
        if ((devp->observed & GPS_TYPEMASK) != 0)
            active++;
        (void)snprintf(reply, replylen,
               "{\"class\":\"POLL\",\"time\":\"%s\",\"active\":%d,\"tpv\":[",
               unix_to_iso8601(timestamp(), tbuf, sizeof(tbuf)), active);
        for (devp = devices; devp < devices + MAXDEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if ((devp->observed & GPS_TYPEMASK) != 0) {
                    json_tpv_dump(devp, &sub->policy,
                        reply + strlen(reply),
                        replylen - strlen(reply));
                        rstrip(reply);
                        (void)strlcat(reply, ",", replylen);
                }
            }
        }
        if (reply[strlen(reply) - 1] == ',')
            reply[strlen(reply) - 1] = '\0';	/* trim trailing comma */
        (void)strlcat(reply, "],\"gst\":[", replylen);
        for (devp = devices; devp < devices + MAXDEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if ((devp->observed & GPS_TYPEMASK) != 0) {
                    json_noise_dump(&devp->gpsdata,
                        reply + strlen(reply),
                        replylen - strlen(reply));
                    rstrip(reply);
                    (void)strlcat(reply, ",", replylen);
                }
            }
        }
        if (reply[strlen(reply) - 1] == ',')
            reply[strlen(reply) - 1] = '\0';	/* trim trailing comma */
        (void)strlcat(reply, "],\"sky\":[", replylen);
        for (devp = devices; devp < devices + MAXDEVICES; devp++) {
            if (allocated_device(devp) && subscribed(sub, devp)) {
                if ((devp->observed & GPS_TYPEMASK) != 0) {
                    json_sky_dump(&devp->gpsdata,
                        reply + strlen(reply),
                        replylen - strlen(reply));
                    rstrip(reply);
                    (void)strlcat(reply, ",", replylen);
                }
            }
        }
        if (reply[strlen(reply) - 1] == ',')
            reply[strlen(reply) - 1] = '\0';	/* trim trailing comma */
            (void)strlcat(reply, "]}\r\n", replylen);
    } else if (strncmp(buf, "VERSION;", 8) == 0) {
        buf += 8;
        json_version_dump(reply, replylen);
    } else {
        const char *errend;
        errend = buf + strlen(buf) - 1;
        while (isspace(*errend) && errend > buf)
            --errend;
        (void)snprintf(reply, replylen,
           "{\"class\":\"ERROR\",\"message\":\"Unrecognized request '%.*s'\"}\r\n",
            (int)(errend - buf), buf);
        gpsd_report(context.debug, LOG_ERROR, "ERROR response: %s\n", reply);
        buf += strlen(buf);
    }

    bailout:
        *after = buf;
}

/*@-mustdefine@*/
const char /*@ observer @*/ *gpsd_canboatdump(char *scbuf, size_t scbuflen,
      struct gps_device_t *device)
{
    size_t i;
    int32_t j, jj = 0;
    uint16_t ct = 0;
    const char *hexchar = "0123456789abcdef";

    // no sense in coparing to outbuffer as a fixed size array here
    if (0 == device->packet.outbuflen)
        return "";

    for(ct = 0; ct < device->packet.out_count; ct++) {

        if(device->packet.out_type[ct] == FRM_TYPE_NMEA2000) {

            if(4 > device->packet.out_len[ct]) {
                continue;
            }

            uint32_t pgn =
                getleu32(device->packet.outbuffer + device->packet.out_offset[ct], 0);

            const uint8_t *ibuf =
                device->packet.outbuffer + 4 + device->packet.out_offset[ct];

            jj = snprintf((char *)(&scbuf[j]), scbuflen - j,
                          "%s,3,%u,2,255,%u",
                         "2016-04-01-07:54:13.595",
                         pgn, device->packet.out_len[ct] - 4);
            if(jj < 0)
                continue;
            j += jj;

            /*@ -shiftimplementation @*/
            for (i = 0; i < (size_t)device->packet.out_len[ct] - 4 && i * 2 < scbuflen - 2; i++) {
                scbuf[j++] = ',';
                scbuf[j++] = hexchar[(ibuf[i] & 0xf0) >> 4];
                scbuf[j++] = hexchar[ibuf[i] & 0x0f];
            }
            /*@ +shiftimplementation @*/
            scbuf[j++] = '\r';
            scbuf[j++] = '\n';
            scbuf[j] = '\0';
        }
    }
    return scbuf;
}
/*@+mustdefine@*/

static void raw_report_write(struct subscriber_t *sub, struct gps_device_t *device) {

    if (TEXTUAL_PACKET_TYPE(device->packet.type)
    && (sub->policy.raw > 0 || sub->policy.nmea)) {
    (void)throttled_write(sub,
      (char *)device->packet.outbuffer,
      device->packet.outbuflen);
    return;
    }

    if ((VYSPI_PACKET == device->packet.type)
    && (sub->policy.raw > 0 || sub->policy.nmea)) {

        uint16_t cnt = 0;

        for(cnt = 0; cnt < device->packet.out_count; cnt++) {
            if((device->packet.out_type[cnt] == FRM_TYPE_AIS)
               || (FRM_TYPE_NMEA0183 == device->packet.out_type[cnt])) {
                (void)throttled_write(sub,
                                      (char *)(device->packet.outbuffer + device->packet.out_offset[cnt]),
                                      device->packet.out_len[cnt]);
                gpsd_report(context.debug, LOG_DATA,
                            "<= RAWREPORT write vyspi %s - len=%d\n",
                            device->gpsdata.dev.path,
                            device->packet.out_len[cnt]);
            }
        }
    }

    /*
     * Also, simply copy if user has specified
     * super-raw mode.
     */
    if (sub->policy.raw > 1) {
        (void)throttled_write(sub,
                              (char *)device->packet.outbuffer,
                              device->packet.outbuflen);
        return;
    }

#ifdef BINARY_ENABLE
#ifdef VYSPI_ENABLE
#endif
    if (device->packet.type == VYSPI_PACKET) {
        if(sub->policy.canboat == 1) {
            const char * hd =
                gpsd_canboatdump(device->msgbuf, sizeof(device->msgbuf),
                                 device);
            (void)throttled_write(sub, (char *)hd, strlen(hd));
        }
        if (sub->policy.raw == 1) {
            const char *hd = gpsd_vyspidump(device);
            if(strlen(hd) > 0) {
                (void)strlcat((char *)hd, "\r\n", sizeof(device->msgbuf));
                (void)throttled_write(sub, (char *)hd, strlen(hd));
            }
        }
    } else {
        /*
         * Maybe the user wants a binary packet hexdumped.
         */
        if (sub->policy.raw == 1) {
            const char *hd =
            gpsd_hexdump(device->msgbuf, sizeof(device->msgbuf),
                         (char *)device->packet.outbuffer,
                         device->packet.outbuflen);
            (void)strlcat((char *)hd, "\r\n", sizeof(device->msgbuf));
            (void)throttled_write(sub, (char *)hd, strlen(hd));
        }
    }
#endif /* BINARY_ENABLE */
}

static void raw_report(struct gps_device_t *device)
/* report a raw packet to a subscriber */
{
    struct subscriber_t *sub;

    gpsd_report(context.debug, LOG_DATA,
                "<= RAWREPORT %s\n",
                device->gpsdata.dev.path);
    /* *INDENT-OFF* */
    /*
     * NMEA and other textual sentences are simply
     * copied to all clients that are in raw or nmea
     * mode.
     */
    /* update all subscribers associated with this device */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    /*@-nullderef@*/
    if (sub == NULL || sub->active == 0 || !subscribed(sub, device))
    continue;

    raw_report_write(sub, device);
    }

    if (TEXTUAL_PACKET_TYPE(device->packet.type))  {

        (void)gpsd_device_write(device, FRM_TYPE_NMEA0183,
                                (char *)device->packet.outbuffer,
                                device->packet.outbuflen);
        (void)gpsd_udp_write((char *)device->packet.outbuffer,
                             device->packet.outbuflen);
        return;
    }

    if(VYSPI_PACKET == device->packet.type) {
        uint16_t cnt = 0;

        for(cnt = 0; cnt < device->packet.out_count; cnt++) {
            if((device->packet.out_type[cnt] == FRM_TYPE_AIS)
               || (FRM_TYPE_NMEA0183 == device->packet.out_type[cnt])) {

                (void)gpsd_device_write(device,  FRM_TYPE_NMEA0183,
                                        (char *)(device->packet.outbuffer + device->packet.out_offset[cnt]),
                                        device->packet.out_len[cnt]);
                (void)gpsd_udp_write((char *)(device->packet.outbuffer + device->packet.out_offset[cnt]),
                                     device->packet.out_len[cnt]);
            }
        }
    }
    gpsd_report(context.debug, LOG_DATA,
                "<= RAWREPORT done %s\n",
                device->gpsdata.dev.path);
}

static void pseudonmea_write(gps_mask_t changed,
                             char *buf, size_t len,
                             struct gps_device_t *device) {

    /* some e.g. ST sentences can't be translated and buf len might be 0.
       instead of catching that for every single pseudonmea sentence
       we just catch an empty buffer here (which would cause trouble with many clients */
    if(len <= 0) return;

    struct subscriber_t *sub;
    /* update all subscribers associated with this device */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
        /*@-nullderef@*/
        if (sub == NULL || sub->active == 0 || !subscribed(sub, device))
            continue;
        if (sub->policy.watcher && sub->policy.nmea) {
            if (changed & DATA_IS) {
                (void)throttled_write(sub, buf, len);
            }
        }
    }
    if (changed & DATA_IS) {
    gpsd_device_write(device, FRM_TYPE_NMEA0183, buf, len);
        (void)gpsd_udp_write((char *)buf, len);
    }
}

static void pseudon2k_report(gps_mask_t changed,
                             struct gps_device_t *device)
/* report pseudo-N2K in appropriate circumstances */
{
    gpsd_report(context.debug, LOG_DATA,
                "<= PSEUDON2K %s\n",
                device->gpsdata.dev.path);
    int go = 0;

    if(VYSPI_PACKET == device->packet.type) {
        uint16_t cnt = 0;
        for(cnt = 0; cnt < device->packet.out_count; cnt++) {
            if(device->packet.out_type[cnt] != FRM_TYPE_NMEA2000) {
                go = 1;
                break;
            }
        }
    } else go = 1;

    if(go)
        n2k_binary_dump(changed, device, gpsd_device_write);
    else
        gpsd_report(context.debug, LOG_DATA,
                    "<= PSEUDON2K: no translatable data found.\n");
}

static void pseudonmea_report(gps_mask_t changed,
                              struct gps_device_t *device)
/* report pseudo-NMEA in appropriate circumstances */
{
    gpsd_report(context.debug, LOG_DATA,
                "<= PSEUDONMEA %s\n",
                device->gpsdata.dev.path);

    int go = 0;
    int nmea0183_ais = device->packet.out_count;

    if(VYSPI_PACKET == device->packet.type) {
        uint16_t cnt = 0;
        for(cnt = 0; cnt < device->packet.out_count; cnt++) {
            if((device->packet.out_type[cnt] == FRM_TYPE_NMEA2000)
               || (device->packet.out_type[cnt] == FRM_TYPE_ST)){
                go = 1;
                break;
            } else if((device->packet.out_type[cnt] == FRM_TYPE_AIS)
                      || (device->packet.out_type[cnt] == FRM_TYPE_NMEA0183)) {
                nmea0183_ais--;
            }
        }
        // we only had NMEA 0183 packages, no need to generate from data
        if(nmea0183_ais <= 0) {
            return;
        }
    }

    
    if (((GPS_PACKET_TYPE(device->packet.type)
         && !TEXTUAL_PACKET_TYPE(device->packet.type))) || go) {

        char buf[MAX_PACKET_LENGTH * 3 + 2];

        if ((changed & REPORT_IS) != 0) {
            nmea_tpv_dump(device, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= GPS (binary tpv) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);
        }

        if ((changed & SATELLITE_SET) != 0) {
            nmea_sky_dump(device, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= GPS (binary sky) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);
        }

        if ((changed & SUBFRAME_SET) != 0) {
            nmea_subframe_dump(device, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= GPS (binary subframe) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);
        }
#ifdef AIVDM_ENABLE
        if ((changed & AIS_SET) != 0) {
            nmea_ais_dump(device, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= AIS (binary ais) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);
        }
#endif /* AIVDM_ENABLE */
        if ((changed & ENVIRONMENT_SET) != 0) {
            int num = nmea_environment_dump(device, 0, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= GPS (binary environment) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);

            if(num > 1) {
                nmea_environment_dump(device, 1, buf, sizeof(buf));
                gpsd_external_report(context.debug, LOG_DATA,
                                     "<= GPS (binary environment) %s: %s\n",
                                     device->gpsdata.dev.path, buf);
                pseudonmea_write(changed, buf, strlen(buf), device);
            }
        }
        if ((changed & NAVIGATION_SET) != 0) {
            nmea_navigation_dump(device, buf, sizeof(buf));
            gpsd_external_report(context.debug, LOG_DATA,
                                 "<= GPS (binary navigation) %s: %s\n",
                                 device->gpsdata.dev.path, buf);
            pseudonmea_write(changed, buf, strlen(buf), device);
        }
    }
    gpsd_report(context.debug, LOG_DATA,
                "<= PSEUDONMEA done %s\n",
                device->gpsdata.dev.path);
}
#endif /* SOCKET_EXPORT_ENABLE */

static void signalk_report(gps_mask_t changed,
      struct gps_device_t *device)
{
    if (!(changed & DATA_IS)) {
        return;
    }

    char buf[MAX_PACKET_LENGTH * 3 + 2];
    struct subscriber_t *sub;
    gps_mask_t reported = 0;

    if (changed & DATA_IS) {
        reported = signalk_update_dump(device, &vessel, buf, sizeof(buf));
    }

    // nothing to report - NOTE: this flag needs to be set in signalk-dump
    if(!reported) {
        gpsd_report(context.debug, LOG_DATA,
                    "<= SIGNALK nothing to report\n");
        return;
    }

    /* update all subscribers associated with this device
       we are not sending to http protocol which requires explicit GET requests
     */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
        /*@-nullderef@*/
        if (sub == NULL || sub->active == 0 || !subscribed(sub, device))
            continue;
        if (sub->policy.watcher && sub->policy.signalk
            && sub->policy.protocol != http) {
            if (changed & DATA_IS) {
                gpsd_external_report(context.debug, LOG_INF,
                                     "signalk update: %s\n",
                                     buf);
                (void)throttled_write(sub, buf, strlen(buf));
            }
        }
    }
}

static void all_reports(struct gps_device_t *device, gps_mask_t changed)
/* report on the corrent packet from a specified device */
{
#ifdef SOCKET_EXPORT_ENABLE
    struct subscriber_t *sub;

    /* add any just-identified device to watcher lists */
    if ((changed & DRIVER_IS) != 0) {
    bool listeners = false;
    for (sub = subscribers;
         sub < subscribers + MAXSUBSCRIBERS; sub++)
        if (sub->active != 0
    && sub->policy.watcher
    && subscribed(sub, device))
    listeners = true;
    if (listeners) {
        (void)awaken(device);
    }
    }

    /* handle laggy response to a firmware version query */
    if ((changed & (DEVICEID_SET | DRIVER_IS)) != 0) {
    assert(device->device_type != NULL);
    {
        char id2[GPS_JSON_RESPONSE_MAX];
        json_device_dump(device, id2, sizeof(id2));
        notify_watchers(device, true, false, id2);
    }
    }
#endif /* SOCKET_EXPORT_ENABLE */

    /*
     * If the device provided an RTCM packet, repeat it to all devices.
     */
    if ((changed & RTCM2_SET) != 0 || (changed & RTCM3_SET) != 0) {
    if (device->packet.outbuflen > RTCM_MAX) {
        gpsd_report(context.debug, LOG_ERROR,
    "overlong RTCM packet (%zd bytes)\n",
    device->packet.outbuflen);
    } else {
        struct gps_device_t *dp;
        for (dp = devices; dp < devices+MAXDEVICES; dp++) {
    if (allocated_device(dp)) {
/* *INDENT-OFF* */
        if (dp->device_type->rtcm_writer != NULL) {
    if (dp->device_type->rtcm_writer(dp,
         (const char *)device->packet.outbuffer,
         device->packet.outbuflen) == 0)
        gpsd_report(context.debug, LOG_ERROR,
    "Write to RTCM sink failed\n");
    else {
        gpsd_report(context.debug, LOG_IO,
    "<= DGPS: %zd bytes of RTCM relayed.\n",
    device->packet.outbuflen);
    }
        }
/* *INDENT-ON* */
    }
        }
    }
    }


#ifdef NTPSHM_ENABLE
    /*
     * Time is eligible for shipping to NTPD if the driver has
     * asserted PPSTIME_IS at any point in the current cycle.
     */
    if ((changed & CLEAR_IS)!=0)
    device->ship_to_ntpd = false;
    if ((changed & PPSTIME_IS)!=0)
    device->ship_to_ntpd = true;
    /*
     * Only update the NTP time if we've seen the leap-seconds data.
     * Else we may be providing GPS time.
     */
    if ((changed & TIME_SET) == 0) {
    gpsd_report(context.debug, LOG_PROG, "NTP: No time this packet\n");
    } else if (isnan(device->newdata.time)) {
    gpsd_report(context.debug, LOG_PROG, "NTP: bad new time\n");
    } else if (device->newdata.time == device->last_fixtime.real) {
    gpsd_report(context.debug, LOG_PROG, "NTP: Not a new time\n");
    } else if (!device->ship_to_ntpd) {
    gpsd_report(context.debug, LOG_PROG, "NTP: No precision time report\n");
    } else {
    /*@-compdef@*/
    struct timedrift_t td;
    ntpshm_latch(device, &td);
    (void)ntpshm_put(device, device->shmIndex, &td);
    /*@+compdef@*/
    }
#endif /* NTPSHM_ENABLE */

    /*
     * If no reliable end of cycle, must report every time
     * a sentence changes position or mode. Likely to
     * cause display jitter.
     */
    if (!device->cycle_end_reliable && (changed & (LATLON_SET | MODE_SET))!=0)
    changed |= REPORT_IS;

    /* a few things are not per-subscriber reports */
    if ((changed & REPORT_IS) != 0) {
#ifdef NETFEED_ENABLE
    if (device->gpsdata.fix.mode == MODE_3D) {
        struct gps_device_t *dgnss;
        /*
         * Pass the fix to every potential caster, here.
         * netgnss_report() individual caster types get to
         * make filtering decisiona.
         */
        for (dgnss = devices; dgnss < devices + MAXDEVICES; dgnss++)
    if (dgnss != device)
        netgnss_report(&context, device, dgnss);
    }
#endif /* NETFEED_ENABLE */
#if defined(DBUS_EXPORT_ENABLE) && !defined(S_SPLINT_S)
    if (device->gpsdata.fix.mode > MODE_NO_FIX)
        send_dbus_fix(device);
#endif /* defined(DBUS_EXPORT_ENABLE) && !defined(S_SPLINT_S) */
    }

#ifdef SHM_EXPORT_ENABLE
    if ((changed & (REPORT_IS|GST_SET|SATELLITE_SET|SUBFRAME_SET|
        ATTITUDE_SET|RTCM2_SET|RTCM3_SET|AIS_SET)) != 0)
    shm_update(&context, &device->gpsdata);
#endif /* SHM_EXPORT_ENABLE */

    /* report n2k packages to n2k device, node should never be ready if not write enabled */
    if(device->gpsdata.dev.node_state == node_ready)
        pseudon2k_report(changed, device);
    else
        gpsd_report(context.debug, LOG_RAW,
                    "N2K not ready or in readonly. Not reporting in N2K.\n");


    /* report pseudonmea packages to devices and users subscribed */
    pseudonmea_report(changed, device);

    /* report out updates to signalk subscribers */
    signalk_report(changed, device);

    /* report raw packets to users subscribed to those */
    raw_report(device);

    if(context.debug >= LOG_IO) {
        struct timespec now;
        tu_gettime(&now);

        uint32_t nowms = tu_get_time_in_milli(&now);
        double rate = 1000.0*((double)(device->gpsdata.bytes_send - last_bytes_send))/
            ((double)(nowms - last_bytes_send_report_ms));
        gpsd_report(context.debug, LOG_IO,
                    "Wrote %f bytes/s (%0.2fkBit/s) as %u bytes in %u ms\n", rate, rate*8.0/1024.0,
                    device->gpsdata.bytes_send - last_bytes_send,
                    (nowms - last_bytes_send_report_ms));
        last_bytes_send = device->gpsdata.bytes_send;
        last_bytes_send_report_ms = nowms;

        if(last_bytes_send_second_report_ms + 1000 > nowms) {
            rate = 1000.0*((double)(device->gpsdata.bytes_send - last_bytes_send_second))/
                ((double)(nowms - last_bytes_send_second_report_ms));
            gpsd_report(context.debug, LOG_IO,
                        "Wrote %f bytes/s (%0.2fkBit/s) as %u bytes in last %0.2f secs\n",
                        rate, rate*8.0/1024.0,
                        device->gpsdata.bytes_send - last_bytes_send_second,
                        (nowms - last_bytes_send_second_report_ms)/1000.0);
            last_bytes_send_second = device->gpsdata.bytes_send;
            last_bytes_send_second_report_ms = nowms;
        }
    }


#ifdef SOCKET_EXPORT_ENABLE
    /* update all subscribers associated with this device */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    /*@-nullderef@*/
    if (sub == NULL || sub->active == 0 || !subscribed(sub, device))
        continue;

#ifdef PASSTHROUGH_ENABLE
    /* this is for passing through JSON packets */
    /* not even the slightest clue why it was reporting here
       though "changed & PASSTHROUGH" is 0
       - comment out since it kills websockets with binary junk
    if ((changed & PASSTHROUGH_IS) != 0) {
        (void)strlcat((char *)device->packet.outbuffer,
      "\r\n",
      sizeof(device->packet.outbuffer));
        (void)throttled_write(sub,
      (char *)device->packet.outbuffer,
      device->packet.outbuflen+2);
        continue;
    }
    */
#endif /* PASSTHROUGH_ENABLE */

    /* some listeners may be in watcher mode */
    if (sub->policy.watcher) {
        if (changed & DATA_IS) {
    /* guard keeps mask dumper from eating CPU */
    if (context.debug >= LOG_PROG)
        gpsd_report(context.debug, LOG_PROG,
    "Changed mask: %s with %sreliable cycle detection\n",
    gps_maskdump(changed),
    device->cycle_end_reliable ? "" : "un");
    if ((changed & REPORT_IS) != 0)
        gpsd_report(context.debug, LOG_PROG,
    "time to report a fix\n");

    if (sub->policy.json) {
        char buf[GPS_JSON_RESPONSE_MAX * 4];

        if ((changed & AIS_SET) != 0)
    if (device->gpsdata.ais.type == 24
        && device->gpsdata.ais.type24.part != both
        && !sub->policy.split24)
        continue;

        json_data_report(changed,
         device, &sub->policy,
         buf, sizeof(buf));
        if (buf[0] != '\0')
    (void)throttled_write(sub, buf, strlen(buf));

    }
        }
    }
    /*@+nullderef@*/
    } /* subscribers */
#endif /* SOCKET_EXPORT_ENABLE */
}

static void handle_gpsd_cleanstring(const char *buf, char * reply) {

    uint16_t c= 0, c1= 0;

    c1 = strlen(buf) - 1;
    for (c = c1; isspace(buf[c]) && (c > 0); c--)
        continue;
    c1 = c+1;

    for(c= c1; (c < strlen(buf)) && (c1 < strlen(reply) - 2); c++) {
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
}

#ifdef SOCKET_EXPORT_ENABLE
static int handle_gpsd_request(struct subscriber_t *sub, const char *buf)
/* execute GPSD requests from a buffer */
{
    char reply[GPS_JSON_RESPONSE_MAX + 1];
    struct subscriber_t *othersub = NULL;

    reply[0] = '\0';
    if (((strncmp(buf, "GET ", 4) == 0) || (strncmp(buf, "OPTIONS ", 8) == 0))
        || isWebsocket(sub)) {
        // switching to web socket mode
        // handle_websocket_request does its own writes
        gpsd_report(context.debug, LOG_PROG,
                    "handle web socket request\n");
        return handle_websocket_request(sub, buf,
                                        reply + strlen(reply),
                                        sizeof(reply) - strlen(reply));

    } else {
        if (buf[0] == '?') {
            const char *end;
            for (end = buf; *buf != '\0'; buf = end) {
                if (isspace(*buf))
                    end = buf + 1;
                else {
                    handle_request(sub, buf, &end,
                                   reply + strlen(reply),
                                   sizeof(reply) - strlen(reply));
                    return (int)throttled_write(sub, reply, strlen(reply));

                }
            }
        } else if (buf[0] == '$') {
            //TODO should probably do a bit more of a sanity check before sending to serial
            strncpy(reply, buf, strlen(buf));
            if(strlen(buf) > 5) {
                // reply[1] = 'G'; reply[2] = 'P';
                gpsd_device_write(NULL, FRM_TYPE_NMEA0183, reply, strlen(reply));
            }

            handle_gpsd_cleanstring(buf, reply);

            // copy to web sockets for monitoring as well
            for (othersub = subscribers; othersub < subscribers + MAXSUBSCRIBERS; othersub++) {
                if (othersub == NULL || othersub->active == 0
                    || !(othersub->policy.protocol == websocket) || !(othersub->policy.nmea))
                        continue;
                throttled_write(othersub, reply, strlen(reply));
            }
        }
    }
    return 0;
}
#endif /* SOCKET_EXPORT_ENABLE */

#ifdef PPS_ENABLE
static void ship_pps_drift_message(struct gps_device_t *session,
       struct timedrift_t *td)
/* on PPS interrupt, ship a drift message to all clients */
{
#ifdef SOCKET_EXPORT_ENABLE
    /*@-type@*//* splint is confused about struct timespec */
    notify_watchers(session, true, true,
        "{\"class\":\"PPS\",\"device\":\"%s\",\"real_sec\":%ld, \"real_nsec\":%ld,\"clock_sec\":%ld,\"clock_nsec\":%ld}\r\n",
        session->gpsdata.dev.path,
        td->real.tv_sec, td->real.tv_nsec,
        td->clock.tv_sec, td->clock.tv_nsec);
    /*@+type@*/
    return;
#endif /* SOCKET_EXPORT_ENABLE */
}
#endif /* PPS_ENABLE */


#ifdef __UNUSED_AUTOCONNECT__
#define DGPS_THRESHOLD	1600000	/* max. useful dist. from DGPS server (m) */
#define SERVER_SAMPLE	12	/* # of servers within threshold to check */

struct dgps_server_t
{
    double lat, lon;
    char server[257];
    double dist;
};

static int srvcmp(const void *s, const void *t)
{
    return (int)(((const struct dgps_server_t *)s)->dist - ((const struct dgps_server_t *)t)->dist);	/* fixes: warning: cast discards qualifiers from pointer target type */
}

static void netgnss_autoconnect(struct gps_context_t *context,
    double lat, double lon, const char *serverlist)
/* tell the library to talk to the nearest DGPSIP server */
{
    struct dgps_server_t keep[SERVER_SAMPLE], hold, *sp, *tp;
    char buf[BUFSIZ];
    FILE *sfp = fopen(serverlist, "r");

    if (sfp == NULL) {
    gpsd_report(context.debug, LOG_ERROR, "no DGPS server list found.\n");
    return;
    }

    for (sp = keep; sp < keep + SERVER_SAMPLE; sp++) {
    sp->dist = DGPS_THRESHOLD;
    sp->server[0] = '\0';
    }
    /*@ -usedef @*/
    while (fgets(buf, (int)sizeof(buf), sfp)) {
    char *cp = strchr(buf, '#');
    if (cp != NULL)
        *cp = '\0';
    if (sscanf(buf, "%32lf %32lf %256s", &hold.lat, &hold.lon, hold.server) ==
        3) {
        hold.dist = earth_distance(lat, lon, hold.lat, hold.lon);
        tp = NULL;
        /*
         * The idea here is to look for a server in the sample array
         * that is (a) closer than the one we're checking, and (b)
         * furtherest away of all those that are closer.  Replace it.
         * In this way we end up with the closest possible set.
         */
        for (sp = keep; sp < keep + SERVER_SAMPLE; sp++)
    if (hold.dist < sp->dist
        && (tp == NULL || hold.dist > tp->dist))
        tp = sp;
        if (tp != NULL)
    memcpy(tp, &hold, sizeof(struct dgps_server_t));
    }
    }
    (void)fclose(sfp);

    if (keep[0].server[0] == '\0') {
    gpsd_report(context.debug, LOG_ERROR,
        "no DGPS servers within %dm.\n",
        (int)(DGPS_THRESHOLD / 1000));
    return;
    }
    /*@ +usedef @*/

    /* sort them and try the closest first */
    qsort((void *)keep, SERVER_SAMPLE, sizeof(struct dgps_server_t), srvcmp);
    for (sp = keep; sp < keep + SERVER_SAMPLE; sp++) {
    if (sp->server[0] != '\0') {
        gpsd_report(context.debug, LOG_INF,
    "%s is %dkm away.\n", sp->server,
    (int)(sp->dist / 1000));
        if (dgpsip_open(context, sp->server) >= 0)
    break;
    }
    }
}
#endif /* __UNUSED_AUTOCONNECT__ */

static void gpsd_terminate(struct gps_context_t *context)
/* finish cleanly, reverting device configuration */
{
    int dfd;

    for (dfd = 0; dfd < MAXDEVICES; dfd++) {
    if (allocated_device(&devices[dfd])) {
        (void)gpsd_wrap(&devices[dfd]);
    }
    }
#ifdef PPS_ENABLE
    context->pps_hook = NULL;	/* tell any PPS-watcher thread to die */
#endif /* PPS_ENABLE */
}

static struct subscriber_t *
gpsd_accept_client_socket(fd_set *rfds, int sock) {

    sockaddr_t fsin;
    struct subscriber_t *client = NULL;

    if (sock >= 0 && FD_ISSET(sock, rfds)) {
    socklen_t alen = (socklen_t) sizeof(fsin);
    /*@+matchanyintegral@*/
    socket_t ssock =
        accept(sock, (struct sockaddr *)&fsin, &alen);
    /*@+matchanyintegral@*/

    if (BAD_SOCKET(ssock))
        gpsd_report(context.debug, LOG_ERROR,
                        "accept: %s\n", strerror(errno));
    else {
        int opts = fcntl(ssock, F_GETFL);
        static struct linger linger = { 1, RELEASE_TIMEOUT };
        char *c_ip;

        if (opts >= 0)
                (void)fcntl(ssock, F_SETFL, opts | O_NONBLOCK);

        c_ip = netlib_sock2ip(ssock);
        client = allocate_client();
        if (client == NULL) {
                gpsd_report(context.debug, LOG_ERROR,
                            "Client %s connect on fd %d -"
                            "no subscriber slots available\n", c_ip,
                            ssock);
                (void)close(ssock);
        } else
                if (setsockopt
                    (ssock, SOL_SOCKET, SO_LINGER, (char *)&linger,
                     (int)sizeof(struct linger)) == -1) {
                    gpsd_report(context.debug, LOG_ERROR,
                                "Error: SETSOCKOPT SO_LINGER\n");
                    (void)close(ssock);
                    detach_client(client);
                    client = NULL;
                } else {
                    // char announce[GPS_JSON_RESPONSE_MAX];
                    FD_SET(ssock, &all_fds);
                    adjust_max_fd(ssock, true);
                    client->fd = ssock;
                    client->active = timestamp();
                    gpsd_report(context.debug, LOG_INF,
                                "client %s (%d) connect on fd %d\n", c_ip,
                                sub_index(client), ssock);
                    /* remove annyoing version dump for users
                       json_version_dump(announce, sizeof(announce));
                       (void)throttled_write(client, announce,
                       strlen(announce));
                    */
                }
    }
    FD_CLR(sock, rfds);
    }

    return client;
}

/*@ -mustfreefresh @*/
int main(int argc, char *argv[])
{
    /* some of these statics suppress -W warnings due to longjmp() */
#ifdef SOCKET_EXPORT_ENABLE
    static char *gpsd_service = NULL;	/* this static pacifies splint */
    struct subscriber_t *sub;
#endif /* SOCKET_EXPORT_ENABLE */
#ifdef CONTROL_SOCKET_ENABLE
    static socket_t csock;
    fd_set control_fds, rfds;
    socket_t cfd;
    static char *control_socket = NULL;
#endif /* CONTROL_SOCKET_ENABLE */
#if defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE)
    sockaddr_t fsin;
#endif /* defined(SOCKET_EXPORT_ENABLE) || defined(CONTROL_SOCKET_ENABLE) */
    static char *pid_file = NULL;
    struct gps_device_t *device;
    int i, option;
    int msocks[2] = {-1, -1};
    int canboat_socks[2] = {-1, -1};
    bool go_background = true;
    volatile bool in_restart;

    no_timeouts = 0;

    context.debug = 0;
    gps_context_init(&context);

#ifdef CONTROL_SOCKET_ENABLE
    INVALIDATE_SOCKET(csock);
#endif /* CONTROL_SOCKET_ENABLE */
#ifdef PPS_ENABLE
    context.pps_hook = ship_pps_drift_message;
#endif /* PPS_ENABLE */

    while ((option = getopt(argc, argv, "F:D:S:bGhlNnP:V")) != -1) {
    switch (option) {
    case 'D':
        context.debug = (int)strtol(optarg, 0, 0);
#ifdef CLIENTDEBUG_ENABLE
        gps_enable_debug(context.debug, stderr);
#endif /* CLIENTDEBUG_ENABLE */
        break;
#ifdef CONTROL_SOCKET_ENABLE
    case 'F':
        control_socket = optarg;
        break;
#endif /* CONTROL_SOCKET_ENABLE */
    case 'N':
        go_background = false;
        break;
    case 'b':
        context.readonly = true;
        break;
#ifndef FORCE_GLOBAL_ENABLE
    case 'G':
        listen_global = true;
        break;
#endif /* FORCE_GLOBAL_ENABLE */
    case 'l':		/* list known device types and exit */
        typelist();
        break;
    case 'S':
#ifdef SOCKET_EXPORT_ENABLE
        gpsd_service = optarg;
#endif /* SOCKET_EXPORT_ENABLE */
        break;
    case 'n':
#ifndef FORCE_NOWAIT
        nowait = true;
#endif /* FORCE_NOWAIT */
        break;
    case 'P':
        pid_file = optarg;
        break;
    case 'V':
        (void)printf("gpsd: %s (revision %s)\n", VERSION, REVISION);
        exit(EXIT_SUCCESS);
    case 'h':
    case '?':
    default:
        usage();
        exit(EXIT_SUCCESS);
    }
    }

#ifdef SYSTEMD_ENABLE
    sd_socket_count = sd_get_socket_count();
    if (sd_socket_count > 0 && control_socket != NULL) {
        gpsd_report(context.debug, LOG_WARN,
                    "control socket passed on command line ignored\n");
        control_socket = NULL;
    }
#endif

#if defined(CONTROL_SOCKET_ENABLE) || defined(SYSTEMD_ENABLE)
    if (
#ifdef CONTROL_SOCKET_ENABLE
    control_socket == NULL
#endif
#if defined(CONTROL_SOCKET_ENABLE) && defined(SYSTEMD_ENABLE)
    &&
#endif
#ifdef SYSTEMD_ENABLE
    sd_socket_count <= 0
#endif
    && optind >= argc) {
    gpsd_report(context.debug, LOG_ERROR,
        "can't run with neither control socket nor devices\n");
    exit(EXIT_FAILURE);
    }

    /*
     * Control socket has to be created before we go background in order to
     * avoid a race condition in which hotplug scripts can try opening
     * the socket before it's created.
     */
#ifdef SYSTEMD_ENABLE
    if (sd_socket_count > 0) {
        csock = SD_SOCKET_FDS_START;
        FD_SET(csock, &all_fds);
        adjust_max_fd(csock, true);
    }
#endif
#ifdef CONTROL_SOCKET_ENABLE
    if (control_socket) {
    (void)unlink(control_socket);
    if (BAD_SOCKET(csock = filesock(control_socket))) {
        gpsd_report(context.debug, LOG_ERROR,
    "control socket create failed, netlib error %d\n",
    csock);
        exit(EXIT_FAILURE);
    } else
        gpsd_report(context.debug, LOG_SPIN,
    "control socket %s is fd %d\n",
    control_socket, csock);
    FD_SET(csock, &all_fds);
    adjust_max_fd(csock, true);
    gpsd_report(context.debug, LOG_PROG,
        "control socket opened at %s\n",
        control_socket);
    }
#endif /* CONTROL_SOCKET_ENABLE */
#else
    if (optind >= argc) {
    gpsd_report(context.debug, LOG_ERROR,
        "can't run with no devices specified\n");
    exit(EXIT_FAILURE);
    }
#endif /* defined(CONTROL_SOCKET_ENABLE) || defined(SYSTEMD_ENABLE) */


    /* might be time to daemonize */
    /*@-unrecog@*/
    if (go_background) {
    /* not SuS/POSIX portable, but we have our own fallback version */
    if (daemon(0, 0) != 0)
        gpsd_report(context.debug, LOG_ERROR,
    "demonization failed: %s\n",strerror(errno));
    }
    /*@+unrecog@*/

    if (pid_file) {
    FILE *fp;

    if ((fp = fopen(pid_file, "w")) != NULL) {
        (void)fprintf(fp, "%u\n", (unsigned int)getpid());
        (void)fclose(fp);
    } else {
        gpsd_report(context.debug, LOG_ERROR,
    "Cannot create PID file: %s.\n", pid_file);
    }
    }

    openlog("gpsd", LOG_PID, LOG_USER);
    gpsd_report(context.debug, LOG_INF, "launching (Version %s)\n", VERSION);

#ifdef SOCKET_EXPORT_ENABLE
    /*@ -observertrans @*/
    if (!gpsd_service)
    gpsd_service =
        getservbyname("gpsd", "tcp") ? "gpsd" : DEFAULT_GPSD_PORT;
    /*@ +observertrans @*/
    if (passivesocks(gpsd_service, "tcp", QLEN, msocks) < 1) {
        gpsd_report(context.debug, LOG_ERR,
                    "command sockets creation failed, netlib errors %d, %d\n",
                    msocks[0], msocks[1]);
        exit(EXIT_FAILURE);
    }
    gpsd_report(context.debug, LOG_INF, "listening on port %s\n", gpsd_service);
#endif /* SOCKET_EXPORT_ENABLE */
#ifdef SOCKET_EXPORT_ENABLE
    /*@ +observertrans @*/
    if (passivesocks("2112", "tcp", QLEN, canboat_socks) < 1) {
        gpsd_report(context.debug, LOG_ERR,
                    "command sockets creation failed, netlib errors %d, %d\n",
                    canboat_socks[0], canboat_socks[1]);
        exit(EXIT_FAILURE);
    }
    gpsd_report(context.debug, LOG_INF, "listening on port %s\n", "2112");
#endif /* SOCKET_EXPORT_ENABLE */

#ifdef NTPSHM_ENABLE
    if (getuid() == 0) {
    errno = 0;
    // nice() can ONLY succeed when run as root!
    // do not even bother as non-root
    if (nice(NICEVAL) == -1 && errno != 0)
        gpsd_report(context.debug, LOG_INF, "NTPD Priority setting failed.\n");
    }
    /*
     * By initializing before we drop privileges, we guarantee that even
     * hotplugged devices added *after* we drop privileges will be able
     * to use segments 0 and 1.
     */
    (void)ntpshm_context_init(&context);
#endif /* NTPSHM_ENABLE */

#if defined(DBUS_EXPORT_ENABLE) && !defined(S_SPLINT_S)
    /* we need to connect to dbus as root */
    if (initialize_dbus_connection()) {
    /* the connection could not be started */
    gpsd_report(context.debug, LOG_ERROR,
        "unable to connect to the DBUS system bus\n");
    } else
    gpsd_report(context.debug, LOG_PROG,
        "successfully connected to the DBUS system bus\n");
#endif /* defined(DBUS_EXPORT_ENABLE) && !defined(S_SPLINT_S) */

#ifdef SHM_EXPORT_ENABLE
    /* create the shared segment as root so readers can't mess with it */
    if (!shm_acquire(&context)) {
    gpsd_report(context.debug, LOG_ERROR,
        "shared-segment creation failed,\n");
    } else
    gpsd_report(context.debug, LOG_PROG,
        "shared-segment creation succeeded,\n");
#endif /* SHM_EXPORT_ENABLE */


    /*
     * We open devices specified on the command line *before* dropping
     * privileges in case one of them is a serial device with PPS support
     * and we need to set the line discipline, which requires root.
     */
    in_restart = false;
    for (i = optind; i < argc; i++) {
        if (!gpsd_add_device(argv[i], NOWAIT)) {
            gpsd_report(context.debug, LOG_ERROR,
                        "initial GPS device %s open failed\n",
                        argv[i]);
        }
    }

    struct interface_t * it = NULL;
    for (it = interfaces; it < interfaces + MAXINTERFACES; it++) {
        it->sock = -1; it->name[0] = '\0';
    }

    /*
     * Read additional configuration information here:
     * forward rules, interface accept/reject rules, etc.
     */
    config_parse(interfaces, &vessel, devices);

    for (device = devices; device < devices + MAXDEVICES; device++) {

        if (!allocated_device(device))
            continue;

        device->gpsdata.own_mmsi = vessel.mmsi;

        gpsd_report(context.debug, LOG_INF, "checking device (%s) %s\n",
                    device->device_type?"yes":"null", device->gpsdata.dev.path);
        if((device->device_type) && (device->device_type->packet_type == VYSPI_PACKET)) {
            vyspi_init(device);
        }
    }

    gpsd_report(context.debug, LOG_INF,
                "Device init done.\n");

    if (udpsocks() < 0) {

        gpsd_report(context.debug, LOG_ERR,
                    "UDP sockets creation failed.\n");
        exit(EXIT_FAILURE);
    }

    tu_init_time(&context);

    char jsonbuf[GPS_JSON_RESPONSE_MAX + 1];
    json_devicelist_dump(jsonbuf, GPS_JSON_RESPONSE_MAX);
    gpsd_report(context.debug, LOG_INF,
                "%s\n", jsonbuf);


    /* drop privileges */
    if (0 == getuid()) {
    struct passwd *pw;
    struct stat stb;

    /* make default devices accessible even after we drop privileges */
    for (i = optind; i < argc; i++)
        /* coverity[toctou] */
        if (stat(argv[i], &stb) == 0)
    (void)chmod(argv[i], stb.st_mode | S_IRGRP | S_IWGRP);
    /*
     * Drop privileges.  Up to now we've been running as root.
     * Instead, set the user ID to 'nobody' (or whatever the gpsd
     * user set by thre build is) and the group ID to the owning
     * group of a prototypical TTY device. This limits the scope
     * of any compromises in the code.  It requires that all GPS
     * devices have their group read/write permissions set.
     */
    /*@-nullpass@*/
    if (setgroups(0, NULL) != 0)
        gpsd_report(context.debug, LOG_ERROR,
    "setgroups() failed, errno %s\n",
    strerror(errno));
    /*@+nullpass@*/
    /*@-type@*/
#ifdef GPSD_GROUP
    {
        struct group *grp = getgrnam(GPSD_GROUP);
        if (grp)
    if (setgid(grp->gr_gid) != 0)
        gpsd_report(context.debug, LOG_ERROR,
    "setgid() failed, errno %s\n",
    strerror(errno));
    }
#else
    if ((optind < argc && stat(argv[optind], &stb) == 0)
        || stat(PROTO_TTY, &stb) == 0) {
        gpsd_report(context.debug, LOG_PROG,
    "changing to group %d\n", stb.st_gid);
        if (setgid(stb.st_gid) != 0)
    gpsd_report(context.debug, LOG_ERROR,
        "setgid() failed, errno %s\n",
        strerror(errno));
    }
#endif
    pw = getpwnam(GPSD_USER);
    if (pw)
        if (setuid(pw->pw_uid) != 0)
    gpsd_report(context.debug, LOG_ERROR,
        "setuid() failed, errno %s\n",
        strerror(errno));
    /*@+type@*/
    }
    gpsd_report(context.debug, LOG_INF,
    "running with effective group ID %d\n", getegid());
    gpsd_report(context.debug, LOG_INF,
    "running with effective user ID %d\n", geteuid());

#ifdef SOCKET_EXPORT_ENABLE
    for (i = 0; i < NITEMS(subscribers); i++) {
    subscribers[i].fd = UNALLOCATED_FD;
#ifndef S_SPLINT_S
    (void)pthread_mutex_init(&subscribers[i].mutex, NULL);
#endif /* S_SPLINT_S */
    }
#endif /* SOCKET_EXPORT_ENABLE*/

    /*@-compdef -compdestroy@*/
    {
    struct sigaction sa;

    sa.sa_flags = 0;
#ifdef __COVERITY__
    /*
     * Obsolete and unused.  We're only doing this to pacify Coverity
     * which otherwise throws an UNINIT event here. Don't swap with the
     * handler initialization, they're unioned on some architectures.
     */
    sa.sa_restorer = NULL;
#endif /* __COVERITY__ */
    sa.sa_handler = onsig;
    (void)sigfillset(&sa.sa_mask);
    (void)sigaction(SIGHUP, &sa, NULL);
    (void)sigaction(SIGINT, &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    (void)sigaction(SIGQUIT, &sa, NULL);
    (void)signal(SIGPIPE, SIG_IGN);
    }
    /*@+compdef +compdestroy@*/

    /* daemon got termination or interrupt signal */
    if (setjmp(restartbuf) > 0) {
    gpsd_terminate(&context);
    in_restart = true;
    gpsd_report(context.debug, LOG_WARN, "gpsd restarted by SIGHUP\n");
    }

    signalled = 0;

    for (i = 0; i < AFCOUNT; i++) {
        if (msocks[i] >= 0) {
            FD_SET(msocks[i], &all_fds);
            adjust_max_fd(msocks[i], true);
        }
        if (canboat_socks[i] >= 0) {
            FD_SET(canboat_socks[i], &all_fds);
            adjust_max_fd(canboat_socks[i], true);
        }
    }
#ifdef CONTROL_SOCKET_ENABLE
    FD_ZERO(&control_fds);
#endif /* CONTROL_SOCKET_ENABLE */

    /* initialize the GPS context's time fields */
    gpsd_time_init(&context, time(NULL));

    /*
     * If we got here via SIGINT, reopen any command-line devices. PPS
     * through these won't work, as we've dropped privileges and can
     * no longer change line disciplines.
     */
    if (in_restart)
    for (i = optind; i < argc; i++) {
      if (!gpsd_add_device(argv[i], NOWAIT)) {
    gpsd_report(context.debug, LOG_ERROR,
        "GPS device %s open failed\n",
        argv[i]);
        }
    }

    gpsd_report(context.debug, LOG_INF,
    "gpsd with max %d subscribers\n", MAXSUBSCRIBERS);

    while (0 == signalled) {
#ifdef EFDS
    fd_set efds;
#endif /* EFDS */
    switch(gpsd_await_data(&rfds, maxfd, &all_fds, context.debug))
    {
    case AWAIT_TIMEOUT:
            no_timeouts++;
        break;
    case AWAIT_GOT_INPUT:
            no_timeouts = 0;
        break;
    case AWAIT_NOT_READY:
#ifdef EFDS
        for (device = devices; device < devices + MAXDEVICES; device++)
    if (FD_ISSET(device->gpsdata.gps_fd, &efds)) {
        deactivate_device(device);
        free_device(device);
    }
#endif /* EFDS*/
        continue;
    case AWAIT_FAILED:
        exit(EXIT_FAILURE);
    }

#ifdef SOCKET_EXPORT_ENABLE
    /* always be open to new client connections */
    for (i = 0; i < AFCOUNT; i++) {
        struct subscriber_t *client = NULL;
        gpsd_accept_client_socket(&rfds, msocks[i]);

        client =
            gpsd_accept_client_socket(&rfds, canboat_socks[i]);
        if(client != NULL)
            client->policy.canboat = true;
    }
#endif /* SOCKET_EXPORT_ENABLE */

#ifdef CONTROL_SOCKET_ENABLE
    /* also be open to new control-socket connections */
    if (csock > -1 && FD_ISSET(csock, &rfds)) {
        socklen_t alen = (socklen_t) sizeof(fsin);
        /*@+matchanyintegral@*/
        socket_t ssock = accept(csock, (struct sockaddr *)&fsin, &alen);
        /*@-matchanyintegral@*/

        if (BAD_SOCKET(ssock))
    gpsd_report(context.debug, LOG_ERROR,
        "accept: %s\n", strerror(errno));
        else {
    gpsd_report(context.debug, LOG_INF,
        "control socket connect on fd %d\n",
        ssock);
    FD_SET(ssock, &all_fds);
    FD_SET(ssock, &control_fds);
    adjust_max_fd(ssock, true);
        }
        FD_CLR(csock, &rfds);
    }

    /* read any commands that came in over the control socket */
    for (cfd = 0; cfd < FD_SETSIZE; cfd++)
        if (FD_ISSET(cfd, &control_fds)) {
    char buf[BUFSIZ];
    ssize_t rd;

    while ((rd = read(cfd, buf, sizeof(buf) - 1)) > 0) {
        buf[rd] = '\0';
        gpsd_report(context.debug, LOG_CLIENT,
    "<= control(%d): %s\n", cfd, buf);
        /* coverity[tainted_data] Safe, never handed to exec */
        handle_control(cfd, buf);
    }
    gpsd_report(context.debug, LOG_SPIN,
        "close(%d) of control socket\n", cfd);
    (void)close(cfd);
    FD_CLR(cfd, &all_fds);
    FD_CLR(cfd, &control_fds);
    adjust_max_fd(cfd, false);
        }
#endif /* CONTROL_SOCKET_ENABLE */

    /* poll all active devices */
    for (device = devices; device < devices + MAXDEVICES; device++)
        if (allocated_device(device) && device->gpsdata.gps_fd > 0) {

            if(device->device_type && (device->device_type->packet_type == VYSPI_PACKET)) {
               if(no_timeouts > 2) {
                	gpsd_report(context.debug, LOG_WARN,
    "No of timeouts > 0: %d.\n", no_timeouts);
           }
               if(no_timeouts > 7) {
                   gpsd_report(context.debug, LOG_WARN,
    "No of timeouts > 7 - re-activating device\n");
                   no_timeouts = 0;
                   // we assume every config on device is lost and we do re-init
                   vyspi_init(device);
               }
            }

            switch (gpsd_multipoll(FD_ISSET(device->gpsdata.gps_fd, &rfds),
                                   device, all_reports, DEVICE_REAWAKE))
            {
            case DEVICE_READY:
                FD_SET(device->gpsdata.gps_fd, &all_fds);
                adjust_max_fd(device->gpsdata.gps_fd, true);
                break;
            case DEVICE_UNREADY:
                FD_CLR(device->gpsdata.gps_fd, &all_fds);
                adjust_max_fd(device->gpsdata.gps_fd, false);
                break;
            case DEVICE_UNCHANGED:
                gpsd_report(context.debug, LOG_SPIN,
                            "device unchanged\n");
                break;
            case DEVICE_ERROR:
            case DEVICE_EOF:
                deactivate_device(device);
                break;
            default:
                break;
            }

            // TODO - find a better place for this - many fragments scanned can have this being called rarely
            if(device->device_type && (device->device_type->packet_type == VYSPI_PACKET)) {
                gpsd_report(device->context->debug, LOG_RAW,
                            "VYSPI should access time trigger.\n");
                vyspi_handle_time_trigger(device);
            }
        }



#ifdef __UNUSED_AUTOCONNECT__
    if (context.fixcnt > 0 && !context.autconnect) {
        for (device = devices; device < devices + MAXDEVICES; device++) {
    if (device->gpsdata.fix.mode > MODE_NO_FIX) {
        netgnss_autoconnect(&context,
    device->gpsdata.fix.latitude,
    device->gpsdata.fix.longitude);
        context.autconnect = True;
        break;
    }
        }
    }
#endif /* __UNUSED_AUTOCONNECT__ */

#ifdef SOCKET_EXPORT_ENABLE
    /* accept and execute commands for all clients */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
        if (sub->active == 0)
            continue;

        lock_subscriber(sub);
        if (FD_ISSET(sub->fd, &rfds)) {
            char buf[BUFSIZ];
            int buflen;

            unlock_subscriber(sub);

            gpsd_report(context.debug, LOG_PROG,
                        "checking client(%d)\n",
                        sub_index(sub));
            if ((buflen =
                 (int)recv(sub->fd, buf, sizeof(buf) - 1, 0)) <= 0) {
                gpsd_report(context.debug, LOG_ERR,
                            "recv from client(%d) returned %d: %s\n",
                            sub_index(sub), buflen, strerror(errno));
                detach_client(sub);
            } else {
                if (buf[buflen - 1] != '\n')
                    buf[buflen++] = '\n';
                buf[buflen] = '\0';
                gpsd_report(context.debug, LOG_CLIENT,
                            "<= client(%d): %s, len=%d\n", sub_index(sub), buf, buflen);

                /*
                 * When a command comes in, update subscriber.active to
                 * timestamp() so we don't close the connection
                 * after COMMAND_TIMEOUT seconds. This makes
                 * COMMAND_TIMEOUT useful.
                 */
                sub->active = timestamp();
                if (handle_gpsd_request(sub, buf) < 0)
                    detach_client(sub);
            }
        } else {
            unlock_subscriber(sub);

            if (!sub->policy.watcher
                && timestamp() - sub->active > COMMAND_TIMEOUT) {
                gpsd_report(context.debug, LOG_WARN,
                            "client(%d) timed out on command wait.\n",
                            sub_index(sub));
                detach_client(sub);
            }
            if (sub->fd != UNALLOCATED_FD) {
                if (sub->policy.watcher && (sub->policy.protocol == tcp)
                    && timestamp() - sub->active > TCP_GRACE_TIMEOUT) {

                    if(!sub->policy.canboat)
                        sub->policy.nmea = true;

                    gpsd_report(context.debug, LOG_INF,
                                "client(%d) timed out on HTTP wait. Locking to raw TCP now.\n",
                                sub_index(sub));
                }
            }
        }
    }

    /*
     * Mark devices with an identified packet type but no
     * remaining subscribers to be closed in RELEASE_TIME seconds.
     * See the explanation of RELEASE_TIME for the reasoning.
     *
     * Re-poll devices that are disconnected, but have potential
     * subscribers in the same cycle.
     */
    for (device = devices; device < devices + MAXDEVICES; device++) {

        bool device_needed = NOWAIT;

        if (!allocated_device(device))
            continue;

        if (!device_needed)
    for (sub=subscribers; sub<subscribers+MAXSUBSCRIBERS; sub++) {
        if (sub->active == 0)
    continue;
        device_needed = subscribed(sub, device);
        if (device_needed)
    break;
    }

        if (!device_needed && device->gpsdata.gps_fd > -1 &&
        device->packet.type != BAD_PACKET) {
    if (device->releasetime == 0) {
        device->releasetime = timestamp();
        gpsd_report(context.debug, LOG_PROG,
    "device %d (fd %d) released\n",
    (int)(device - devices),
    device->gpsdata.gps_fd);
    } else if (timestamp() - device->releasetime >
    RELEASE_TIMEOUT) {
        gpsd_report(context.debug, LOG_PROG,
    "device %d closed\n",
    (int)(device - devices));
        gpsd_report(context.debug, LOG_RAW,
    "unflagging descriptor %d\n",
    device->gpsdata.gps_fd);
        deactivate_device(device);
    }
        }

        if (device_needed && BAD_SOCKET(device->gpsdata.gps_fd) &&
        (device->opentime == 0 ||
        timestamp() - device->opentime > DEVICE_RECONNECT)) {
    device->opentime = timestamp();
    gpsd_report(context.debug, LOG_INF,
        "reconnection attempt on device %d\n",
        (int)(device - devices));
    (void)awaken(device);
        }
    }
#endif /* SOCKET_EXPORT_ENABLE */
    }

    /* if we make it here, we got a signal... deal with it */
    /* restart on SIGHUP, clean up and exit otherwise */
    if (SIGHUP == (int)signalled)
    longjmp(restartbuf, 1);

    gpsd_report(context.debug, LOG_WARN,
    "received terminating signal %d.\n", signalled);

    gpsd_terminate(&context);

    gpsd_report(context.debug, LOG_WARN, "exiting.\n");

#ifdef SOCKET_EXPORT_ENABLE
    /*
     * A linger option was set on each client socket when it was
     * created.  Now, shut them down gracefully, letting I/O drain.
     * This is an attempt to avoid the sporadic race errors at the ends
     * of our regression tests.
     */
    for (sub = subscribers; sub < subscribers + MAXSUBSCRIBERS; sub++) {
    if (sub->active != 0)
        detach_client(sub);
    }
#endif /* SOCKET_EXPORT_ENABLE */

#ifdef SHM_EXPORT_ENABLE
    shm_release(&context);
#endif /* SHM_EXPORT_ENABLE */

#ifdef CONTROL_SOCKET_ENABLE
    if (control_socket)
    (void)unlink(control_socket);
#endif /* CONTROL_SOCKET_ENABLE */
    if (pid_file)
    (void)unlink(pid_file);
    return 0;
}

/*@ +mustfreefresh @*/
