#include <stdio.h>
#include <string.h>
#include <uci.h>
#include <stdlib.h>

#include "gpsd_config.h"
#include "gpsd.h"

static struct uci_context *uci_ctx;
static int uci_debuglevel = 0;
struct uci_package * config_init(void);

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

/*
  parse an interface section
  
 */
static void
config_parse_interface(struct gps_device_t *devices,
					   struct uci_section *s, const char * name) {

    int16_t portno  = -1;
    
	gpsd_report(uci_debuglevel, LOG_INF, 
				"parsing interface section %s\n", name); 
	
	const char *devname = NULL;
	devname = uci_lookup_option_string(uci_ctx, s, "device");

	if (!devname)
		return;
    
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

struct uci_package *
config_init() {

  struct uci_context *ctx;
  struct uci_package *p;

  ctx = uci_alloc_context ();
  
  // uci_set_confdir(ctx, "./systemd");
  // uci_set_savedir(ctx, "./systemd");
  
    if (uci_load(ctx, "gpsd", &p)) {
        gpsd_report(uci_debuglevel, LOG_ERROR, 
                    "failed to open config file\n"); 
        return (struct uci_package *)NULL;
    }

    uci_ctx = ctx;

    return p;
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

int config_parse(struct gps_device_t *devices) {
	
	struct uci_package *uci_network;
	struct uci_element *e;
	uint8_t n = 0;
	
	config_uci_debuglevel(devices);
	
	uci_network = config_init();

	if(!uci_network) return 1;
  
	// walk through all interface sections
	uci_foreach_element(&uci_network->sections, e) {
		
		struct uci_section *s = uci_to_section(e);

		// treat the port sections first
		if ((n < MAXDEVICES) && !strcmp(s->type, "interface")) {
			config_parse_interface(devices, s, e->name);
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

	uci_free_context (uci_ctx);
	return 0;
}
