#ifndef _VY_DATA_H_
#define _VY_DATA_H_

#include <stdint.h>
#include <stdbool.h>

#include "frame.h"
#include "ring_buffer.h"

#ifndef S_SPLINT_S
typedef uint64_t gps_mask_t;
#else
typedef /*@unsignedintegraltype@*/ unsigned long long gps_mask_t;
#endif /* S_SPLINT_S */


typedef enum vy_type {
    vy_integer, vy_uinteger, vy_real,
          vy_time, vy_position } vy_type_t;

// just an idea to use unit as well just like type
typedef enum vy_unit {
    vy_mps, vy_knots, vy_mph } vy_unit_t;

typedef struct vy_position {
    double lat;
    double lon;
} vy_position_t;

typedef struct vy_data {

    vy_type_t type;
    vy_unit_t unit;
    const char * name;
    const char * description;

    union {
        int32_t integer;
        uint32_t uinteger;
        double real;
        vy_position_t position;
    } value;

    uint32_t last_seen;
    uint8_t  source;
    uint8_t  device;
    enum frm_type_t frame_type;

} vy_data_t;

#ifdef __cplusplus
extern "C" {
#endif
bool vy_data_is_set(vy_data_t *);
double vy_data_get_real(vy_data_t *);
void vy_data_set_real(vy_data_t *, double);
#ifdef __cplusplus
}
#endif


#endif // _VY_DATA_H_
