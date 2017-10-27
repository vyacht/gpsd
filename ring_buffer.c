#include <stdint.h>
#include <string.h>
#include "ring_buffer.h"

/**
 * we are currently single threaded and don't need memory barriers
 */
#if defined(__x86_64__) || defined(__i386)
#define smp_wmb() asm volatile("" ::: "memory")
#else
#define smp_wmb() // TODO
#endif
//void smp_wmb() {}

/*
  helper to calculate the unused elements in a rb
 */
static inline unsigned int rb_unused(rb_t *rb)
{
        return (rb->mask + 1) - (rb->in - rb->out);
}

/**
 * rb_init - initilizes a rb structure
 * @rb: address of the rb to be used
 */
void rb_init(rb_t * rb) 
{
        rb->in    = 0;
        rb->out   = 0;
        rb->mask  = RB_MAX_SIZE - 1;
}

void rb_reset(rb_t * rb) 
{
        rb->in    = rb->out   = 0;
}

/**
 * rb_size - returns the size of the rb in elements
 * @rb: address of the rb to be used
 */
unsigned int rb_size(rb_t * rb)
{
    return ((rb)->mask + 1);
}

/**
 * rb_len - returns the number of used elements in the rb
 * @rb: address of the rb to be used
 */
unsigned int rb_len(rb_t * rb)
{
    // two usigned will not make this negative 
    // even if in or out overflow as long as in is ahead of out
    return rb->in - rb->out;
}

/**
 * krb_is_empty - returns true if the rb is empty
 * @rb: address of the rb to be used
 */
int rb_is_empty(rb_t * rb)
{
     return rb->in == rb->out;
}

/**
 * rb_is_full - returns true if the rb is full
 * @rb: address of the rb to be used
 */
int rb_is_full(rb_t * rb)
{
     return rb_len(rb) > rb->mask;
}

/**
 * rb_capacity - returns the number free space for new elements
 * @rb: address of the rb to be used
 */
unsigned int rb_free(rb_t * rb)
{
     return rb_unused(rb);
}

int rb_put(rb_t * rb, double val, uint32_t msec)
{
        int ret;
        ret = rb_is_full(rb);
        rb->data[rb->in & rb->mask].val = val;
        rb->data[rb->in & rb->mask].msec = msec;
        smp_wmb();
        rb->in++;

        if (ret) {
            rb->out++;
        }
        return ret;
}

int rb_peek_n(rb_t * rb, uint32_t n, double * val, uint32_t * msec)
{  
    int ret;   
    ret = !rb_is_empty(rb) && (n < rb_len(rb));
    if (ret) { 
        *val =
            rb->data[(rb->out + n) & rb->mask].val;
        *msec =
            rb->data[(rb->out + n) & rb->mask].msec;
    }
    return ret;
}

int rb_get(rb_t * rb, double * val, uint32_t * msec)
{  
    int ret;   
    ret = !rb_is_empty(rb);   
    if (ret) { 
        *val =
            rb->data[rb->out & rb->mask].val;
        *msec =
            rb->data[rb->out & rb->mask].msec;
        smp_wmb();
        rb->out++;
    }
    return ret;
}



