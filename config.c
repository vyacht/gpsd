#include <stdio.h>
#include <string.h>
#include <uci.h>
#include <stdlib.h>
#include <unistd.h>  /* file check with access */

#include "gpsd_config.h"
#include "gpsd.h"

/* for getifaddr */
#include <netdb.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <net/if.h>            /* IFF_LOOPBACK */

/* inet_ntoa */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <errno.h>
#include <uuid/uuid.h>

#ifndef FORCE_GLOBAL_ENABLE
static bool listen_global = true;
#endif /* FORCE_GLOBAL_ENABLE */

static struct uci_context *uci_ctx;
static int uci_debuglevel = 0;
struct uci_context * config_init(void);

#define DEFAULT_UDP_BROADCAST_PORT 2000

static void
config_uci_debuglevel(struct gps_device_t * devices) {
	
	// find the right device for this interface section
	struct gps_device_t *devp = devices;
	while(devp < devices + MAXDEVICES) {
	  
		if (allocated_device(devp)) {
			uci_debuglevel = devp->context->debug;
			break;
		}
		
		devp++;
	}

}

static struct gps_device_t *
config_device_by_devicename(struct gps_device_t * devices, const char * devname) {
	
	// find the right device for this interface section
	struct gps_device_t *devp = devices;
    
    if((NULL == devname) || (devices == NULL)) {
        return NULL;
    }
    
	while(devp < devices + MAXDEVICES) {
	  
        if(allocated_device(devp)) {
            
            if(strcmp(devp->gpsdata.dev.path, devname) == 0) {
                return devp;
            }
        }

        devp++;
	}

	return NULL;
}

static struct gps_device_t *
config_device_by_portname(struct gps_device_t * devices, const char * portname) {
	
	// find the right device for this interface section
	struct gps_device_t *devp = devices;
    
    if((NULL == portname) || (devices == NULL)) {
        return NULL;
    }
    
	while(devp < devices + MAXDEVICES) {
	  
        if(allocated_device(devp)) {
            
            uint8_t p = 0;
            for(p = 0; p < devp->gpsdata.dev.port_count; p++) {
                if(strcmp(devp->gpsdata.dev.portlist[p].name, portname) == 0) {
                    return devp;
                }
            }
        }

        devp++;
	}

	return NULL;
}

static struct device_port_t *
config_port_by_portname(struct gps_device_t * devices, const char * name) {
	
	// find the right device for this interface section
	struct gps_device_t *devp = devices;

    if(NULL == name) {
        return NULL;
    }
	while(devp < devices + MAXDEVICES) {
	  
        if(allocated_device(devp)) {

            uint8_t p = 0;
            for(p = 0; p < devp->gpsdata.dev.port_count; p++) {
                if(strcmp(devp->gpsdata.dev.portlist[p].name, name) == 0) {
                    return &devp->gpsdata.dev.portlist[p];
                }
            }
        }
        
        devp++;
	}

	return NULL;
}

static void
config_get_device_policy(const char * val, device_policy_t * policy) {
	
	if(val && strcmp(val, "ACCEPT") == 0) {
		*policy= device_policy_accept;
	} else if(val && strcmp(val, "ACCEPT") == 0) {
		*policy= device_policy_reject;
	}
}

static void 
config_parse_interface_option(struct device_port_t * port, const char * name, struct uci_option *o) {

	struct uci_element *e;

	if (o->type == UCI_TYPE_LIST) {
		uci_foreach_element(&o->v.list, e) {
			printf("l: %s %s\n", name, e->name);
		}
		return;
	} 

	if (!o || o->type != UCI_TYPE_STRING)
		return;


	if(strcmp(name, "input") == 0) {
		
	  gpsd_report(uci_debuglevel, LOG_INF, 
		      "input: %s\n", o->v.string);
	  config_get_device_policy(o->v.string, &port->input);
	  
	} else if(strcmp(name, "output") == 0) {
		
	  gpsd_report(uci_debuglevel, LOG_INF, 
		      "output: %s\n", o->v.string);
	  config_get_device_policy(o->v.string, &port->output);
	  
	} else if(strcmp(name, "type") == 0) {
		
	  gpsd_report(uci_debuglevel, LOG_INF, 
		      "type: %s\n", o->v.string);

      strcpy(port->type_str, o->v.string);
      
	} else if(strcmp(name, "speed") == 0) {
		
	  gpsd_report(uci_debuglevel, LOG_INF, 
		      "speed: %s\n", o->v.string);
      port->speed = atoi(o->v.string);
	}

}

struct interface_t * 
config_next_free_interface(struct interface_t * ints) {

    struct interface_t * it;
    for (it = ints; it < ints + MAXINTERFACES; it++) {
        if(it->name[0] == '\0') {
            return it;
        }
    }
    return NULL;
}


/* 
 * parse an udp/tcp specific interface
 * 
 * at this point in time only port option is supported
 */
static void
config_parse_proto_interface(struct interface_t * intf,
					   struct uci_section *s, const char * name) {

	gpsd_report(uci_debuglevel, LOG_INF, 
				"parsing protocol interface section %s\n", name); 

    strncpy(intf->name, name, DEVICE_SHORTNAME_MAX);

    intf->port = DEFAULT_UDP_BROADCAST_PORT;

	// parse and assign all options
	struct uci_element *e;
	uci_foreach_element(&s->options, e) {

		struct uci_option *o = uci_to_option(e);

        if (!o || o->type != UCI_TYPE_STRING)
            continue;

        if(strcmp(e->name, "proto") == 0) {

            gpsd_report(uci_debuglevel, LOG_INF, 
                        "proto: %s\n", o->v.string);
            strncpy(intf->proto, o->v.string, 16);

        } else if(strcmp(e->name, "port") == 0) {

            gpsd_report(uci_debuglevel, LOG_INF, 
                        "port: %s\n", o->v.string);

            int port = atol(o->v.string);
            intf->port = port;

        } else if(strcmp(e->name, "ipaddr") == 0) {

            gpsd_report(uci_debuglevel, LOG_INF, 
                        "ipaddr: %s\n", o->v.string);

            intf->ipaddr.sin_addr.s_addr = inet_addr(o->v.string);
        }
    }
}

static int config_ifaddrs(struct interface_t * intfs,
                          struct uci_section *section, const char * name) {

    /* collect all interfaces */
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        gpsd_report(uci_debuglevel, LOG_ERROR, 
                    "failed to open getifaddrs (%d)\n", errno);
        return -1;
    }

    /* Walk through linked list, maintaining head pointer so we
       can free list later */
    
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (ifa->ifa_flags & IFF_LOOPBACK) {
            if (listen_global)
                continue;
        }
        else {
            if (!listen_global)
                continue;
        }

        if (family == AF_INET || family == AF_INET6) {

            s = getnameinfo(ifa->ifa_addr,
                            (family == AF_INET) ? sizeof(struct sockaddr_in) :
                            sizeof(struct sockaddr_in6),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

            if (s != 0) {
                gpsd_report(uci_debuglevel, LOG_ERROR, 
                            "failed to getnameinfo() (%s)\n", gai_strerror(s));
                return -1;
            }

            if(family == AF_INET) {
                char straddr[INET_ADDRSTRLEN];
                strcpy(straddr, inet_ntoa(((struct sockaddr_in*)ifa->ifa_dstaddr)->sin_addr));
                gpsd_report(uci_debuglevel, LOG_INF, 
                            "adding udp host %s on %s with broadcast %s\n",
                            host,
                            inet_ntoa(((struct sockaddr_in*)ifa->ifa_addr)->sin_addr),
                            straddr);

                struct interface_t * it = NULL;
                it = config_next_free_interface(intfs);
                if(it) {

                    config_parse_proto_interface(it, section, name);

                    it->bcast.sin_addr.s_addr = 
                        ((struct sockaddr_in*)ifa->ifa_dstaddr)->sin_addr.s_addr;
                    it->bcast.sin_port = htons(it->port);
                    it->bcast.sin_family = (sa_family_t) AF_INET;

                    it->ipaddr.sin_addr.s_addr = 
                        ((struct sockaddr_in*)ifa->ifa_addr)->sin_addr.s_addr;
                }
            }

            if(family == AF_INET6) {
                char straddr[INET6_ADDRSTRLEN];
                struct sockaddr_in6 * addr = ((struct sockaddr_in6*)ifa->ifa_addr);
                inet_ntop(AF_INET6, &addr->sin6_addr, straddr, sizeof(straddr));

                gpsd_report(uci_debuglevel, LOG_INF, 
                            "ignoring IPV6 host %s with broadcast %s\n",
                            host,
                            straddr);
            }
        }
    }

    freeifaddrs(ifaddr);
    return 0;
}

/*
  parse an interface section
  
 */
static void
config_parse_interface(struct interface_t * ints, struct gps_device_t *devices,
					   struct uci_section *s, const char * name) {

    int16_t portno  = -1;
    
	gpsd_report(uci_debuglevel, LOG_INF, 
				"parsing interface section %s\n", name); 
	
	const char *devname = NULL;
	devname = uci_lookup_option_string(uci_ctx, s, "device");

	if (!devname) {
        /*
         * no device option found, so this is either a udp interface or wrong
         */
        const char *proto = 
            uci_lookup_option_string(uci_ctx, s, "proto");

        if(!proto) {
            gpsd_report(uci_debuglevel, LOG_INF, 
                        "no protocol found in interface section %s\n", name); 
            return;
        }


        const char *ipaddr = 
            uci_lookup_option_string(uci_ctx, s, "ipaddr");

        if(!ipaddr) {

            /* no ipaddr found means that we are using all ip addresses
               found */
            config_ifaddrs(ints, s, name);

        } else {

            // UNTESTED - at least only little tested
            struct interface_t * it = NULL;
            it = config_next_free_interface(ints);
            if(it)
                config_parse_proto_interface(it, s, name);

            it->bcast.sin_addr.s_addr = 
                it->ipaddr.sin_addr.s_addr;
            it->bcast.sin_port = htons(it->port);
            it->bcast.sin_family = (sa_family_t) AF_INET;
        }
        return;
    }
    
	const char *portnos = NULL;
	portnos = uci_lookup_option_string(uci_ctx, s, "port");

    if(portnos) {
        portno = atol(portnos);
        if((portno < 0) || (portno >= 4)) {
            gpsd_report(uci_debuglevel, LOG_ERROR, 
                        "interface %s with illegal port number %s\n", name, portnos); 
            return;
        }
    }

    if((strlen(devname) <= 8) || (strncmp(devname, "vyspi://", 8) != 0)) {
        if(portno > -1) {
            gpsd_report(uci_debuglevel, LOG_ERROR, 
                        "interface %s contains port configuration %s for non-vyspi device\n", name, portnos); 
            return;
        }
    }
    
        
	// find the right device for this interface section
	struct gps_device_t *devp =
		config_device_by_devicename(devices, devname);

	if(!devp) {
		gpsd_report(uci_debuglevel, LOG_ERROR, 
					"interface section %s for unkown device %s\n", name, devname); 
        return;
    }
 
    if((strlen(devname) > 8) && (strncmp(devname, "vyspi://", 8) != 0)) {
        if(devp->gpsdata.dev.port_count > 0) {
            gpsd_report(uci_debuglevel, LOG_ERROR, 
                        "Multiple interfaces for non-vyspi device %s found.\n", devname);
            return;
        }
    }
    
    
    struct device_port_t * port = NULL;
    
    port = &devp->gpsdata.dev.portlist[devp->gpsdata.dev.port_count];
    gpsd_report(uci_debuglevel, LOG_INF, 
                "port: %d\n", portno);
        
    port->no = portno;
    
    devp->gpsdata.dev.port_count++;
        
    // assign the short name
    if(name) {
        gpsd_report(uci_debuglevel, LOG_INF, 
                    "interface name %s for device %s and port %d\n", name, devp->gpsdata.dev.path, portno); 
        strcpy(port->name, name);
    }
        
	// parse and assign all options
	struct uci_element *e;
	uci_foreach_element(&s->options, e) {
		struct uci_option *o = uci_to_option(e);

		config_parse_interface_option(port, e->name, o);
	}
}

static void
config_add_forward(struct device_port_t *src, const char * portname) {
	
	uint8_t n = 0;
	while(n < MAXDEVICES) {
		if(src->forward[n][0] == '\0') {
			strcpy(src->forward[n], portname);
			break;
		}
		n++;
	}
}

/*
 * forward sections only have src & dest options
 */
static void
config_parse_forward(struct gps_device_t *devices, struct uci_section *s) {

	struct device_port_t * srcport = NULL;
	struct device_port_t * destport = NULL;
		
	struct uci_element *e;
	uci_foreach_element(&s->options, e) {
		
		struct uci_option *o = uci_to_option(e);

		// check option is really a string (and not e.g. a list
		if (!o || o->type != UCI_TYPE_STRING) {
			gpsd_report(uci_debuglevel, LOG_WARN, 
						"none-string option %s found in section: %s\n",
						e->name, s->type); 
			return;
		}

		// source option
		if(strcmp(e->name, "src") == 0) {
			
			srcport =
				config_port_by_portname(devices, o->v.string);
			
            if(!srcport) {
                gpsd_report(uci_debuglevel, LOG_WARN, 
                            "no forward source %s found\n",
                            o->v.string);
                return;
            }
            
		} else if(strcmp(e->name, "dest") == 0) {
			
			destport =
				config_port_by_portname(devices, o->v.string);
            
            if(!destport) {
                gpsd_report(uci_debuglevel, LOG_WARN, 
                            "no forward destination %s found\n",
                            o->v.string);
                return;
            }
			
		} else {
			gpsd_report(uci_debuglevel, LOG_WARN, 
						"forward section with unkown option %s %s\n",
						e->name, o->v.string); 
		}
	}
	
	gpsd_report(uci_debuglevel, LOG_INF, 
				"forward source %s to destination %s\n",
				srcport->name, destport->name);

	config_add_forward(srcport, destport->name);
}

void
config_add_boat_section(struct uci_package * pkg, 
                        struct uci_ptr * ptr,
                        bool add_section) {

    struct uci_section * section = NULL;
    int ret = UCI_OK;
    uuid_t uuid;

    // generate
    uuid_generate_time_safe(uuid);
    // unparse (to string)
    char uuid_str[37]; // ex. "1b4e28ba-2fa1-11d2-883f-0016d3cca427" + "\0"
    uuid_unparse_lower(uuid, uuid_str);

    if(add_section) {
        char buf2[32];

        // , , section type, 
        uci_add_section(uci_ctx, pkg, "configuration", &section);
        gpsd_report(uci_debuglevel, LOG_INF, 
                    "added section %s %s\n",
                    section->type, section->e.name);

        strcpy(buf2, "gpsd.");
        strcat(buf2, section->e.name);
        if (uci_lookup_ptr(uci_ctx, ptr, buf2, true) != UCI_OK) {
            gpsd_report(uci_debuglevel, LOG_ERROR, 
                        "config error when looking up new section %s\n",
                        buf2);
            return;
        }

        // section was created with default name, rename
        ptr->value = "boat";
        ret = uci_rename(uci_ctx, ptr);
        ptr->section = section->e.name;
    }

    /* if section is not added then ptr already
       should have the details about the section */

    ptr->option  = "uuid";
    ptr->value   = uuid_str;

    ret = uci_set(uci_ctx, ptr);
    if(ret == UCI_OK) {
        gpsd_report(uci_debuglevel, LOG_INF, 
                    "saving new values\n");
        uci_save(uci_ctx, ptr->p);
    }

    uci_commit(uci_ctx, &pkg, false);
}

void
config_handle_boat_section() {

    struct uci_package * pkg = NULL;
    struct uci_ptr ptr;
    char buf2[32];

    if (uci_load(uci_ctx, "gpsd", &pkg)) {
        gpsd_report(uci_debuglevel, LOG_ERROR, 
                    "failed to open config file\n"); 
    }

    strcpy(buf2, "gpsd.boat.uuid");
    if (uci_lookup_ptr(uci_ctx, &ptr, buf2, true) != UCI_OK) {
        gpsd_report(uci_debuglevel, LOG_ERROR, 
                    "config error when looking up section %s\n",
                    buf2);
        return;
    }

    struct uci_element *e;
    e = ptr.last;
    switch(e->type) {
    case UCI_TYPE_SECTION:
        printf("section %s %s\n", ptr.s->type, e->name);
        // configuration.boat section found - but no option
        config_add_boat_section(pkg, &ptr, false);
        break;
    case UCI_TYPE_OPTION:
        // option found, all ok
        printf("option %s\n", ptr.option);
        break;
    default:
        // configuration.boat section not found - need to add it
        config_add_boat_section(pkg, &ptr, true);
        break;
    }
}

struct uci_context *
config_init() {

    struct uci_context *ctx;
    ctx = uci_alloc_context ();
  
    /* for testing only */
    if( access("./systemd/gpsd", F_OK ) != -1 ) {
        uci_set_confdir(ctx, "./systemd");
    }

    return ctx;
}

/*
config gpsd 'core'
	option port '2947'
	option listen_globally 'true'
	option enabled 'true'
	list device '/dev/ttyS0'
	list device 'st:///dev/ttyS1'

config interface 'port1'
	option device '/dev/ttyS0'
	option input 'ACCEPT'
	option output 'ACCEPT'

config interface 'port2'
	option device 'st://127.0.0.1:32000'
	option input 'ACCEPT'

config forward
	option src port1
	option dest port1

config forward
	option src port2
	option dest port1
 */

int config_parse(struct interface_t * interfaces, 
                 struct gps_device_t *devices) {
	
	struct uci_package *uci_network;
	struct uci_element *e;
	uint8_t n = 0;
	
	uci_ctx = config_init();

    if (uci_load(uci_ctx, "gpsd", &uci_network)) {
        gpsd_report(uci_debuglevel, LOG_ERROR, 
                    "failed to open config file\n"); 
        return 1;
    }

	config_uci_debuglevel(devices);
    
	// walk through all interface sections
	uci_foreach_element(&uci_network->sections, e) {
		
		struct uci_section *s = uci_to_section(e);

		// treat the port sections first
		if ((n < MAXDEVICES) && !strcmp(s->type, "interface")) {
			config_parse_interface(interfaces, devices, s, e->name);
		}
		
	}
  
	// walk through all foward sections
	uci_foreach_element(&uci_network->sections, e) {
		
		struct uci_section *s = uci_to_section(e);

		// treat the port sections first
		if (!strcmp(s->type, "forward")) {
			config_parse_forward(devices, s);
		}
	}

    uci_unload(uci_ctx, uci_network);

    config_handle_boat_section();

	uci_free_context (uci_ctx);

	return 0;
}
