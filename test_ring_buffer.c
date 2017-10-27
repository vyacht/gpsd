#include "ring_buffer.h"

int main(int argc, char * argv[]) {

    int i;
    int j = 0;
    rb_t rb;
    double value;
    uint32_t msec;

    rb_init(&rb);

    for(i = 0; i < RB_MAX_SIZE - 10; i++) {
        rb_put(&rb, (double)j, j);
        j++;
    }

    printf("filling %d elements: len= %u, in=%u, out=%u\n", 
           RB_MAX_SIZE - 10,
           rb_len(&rb), 
           rb.in, rb.out);

    for(i = 0; i < RB_MAX_SIZE - 10; i++) {
        rb_put(&rb, (double)j, j);
        j++;
    }

    printf("filling %d elements: len= %u, in=%u, out=%u\n", 
           RB_MAX_SIZE - 10,
           rb_len(&rb), 
           rb.in, rb.out);

    if(rb_peek_n(&rb, 11, &value, &msec)) {
        printf("getting element at %u: value=%f, msec=%u\n", 
               11,
               value, msec);
    }
    if(rb_peek_n(&rb, 0, &value, &msec)) {
        printf("getting element at %u: value=%f, msec=%u\n", 
               0,
               value, msec);
    }
    if(rb_peek_n(&rb, RB_MAX_SIZE -1, &value, &msec)) {
        printf("getting element at %u: value=%f, msec=%u\n", 
               RB_MAX_SIZE -1,
               value, msec);
    }

    return 0;
}
