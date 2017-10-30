#include "vy_data.h"
#include "timeutil.h"

/*
vy_type_t type;
vy_unit_t unit;

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
*/

void vy_data_set_real(vy_data_t * dst, double value) {

    dst->type = vy_real;

    dst->value.real = value;
    dst->last_seen = tu_get_independend_time();

}

double vy_data_get_real(vy_data_t * dst) {

    return dst->value.real;
}

bool vy_data_is_set(vy_data_t * data) {
    return true;
}
