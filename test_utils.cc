#include "test_utils.h"

void set_device_type(struct gps_device_t *session, enum frm_type_t frmType) {

    const struct gps_type_t **dp;
    int pt = VYSPI_PACKET;

    /*
    if(frmType == FRM_TYPE_NMEA0183)
        pt = NMEA_PACKET;
    else if(frmType == FRM_TYPE_NMEA2000)
        pt = VYSPI_PACKET;
    */
    
    for (dp = gpsd_drivers; *dp; dp++) {
        if(pt == (*dp)->packet_type) {
            session->device_type = *dp;
        }
    }
}
