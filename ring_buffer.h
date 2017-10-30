#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RB_MAX_SIZE 4096

typedef struct {
    double val;
    uint32_t msec;
} double_value_t;

typedef struct {
    uint32_t    in;
    uint32_t    out;
    uint32_t    mask;
    uint32_t    max_size;
    double_value_t data[RB_MAX_SIZE];
} rb_t;

/**
 * rb_init - initilizes a rb structure
 * @rb: address of the rb to be used
 */
void rb_init(rb_t * rb);

/**
 * rb_resets - resets a rb structure
 * 	requires a protected and exclusive access 
 * @fifo: address of the rb to be used
 */
void rb_reset(rb_t * rb);

/**
 * rb_size - returns the size of the rb in elements
 * @rb: address of the rb to be used
 */
unsigned int rb_size(const rb_t * rb);

/**
 * rb_capacity - returns the number free space for new elements
 * @rb: address of the rb to be used
 */
unsigned int rb_free(const rb_t * rb);

/**
 * rb_len - returns the number of used elements in the rb
 * @rb: address of the rb to be used
 */
unsigned int rb_len(const rb_t * rb);

/**
 * krb_is_empty - returns true if the rb is empty
 * @rb: address of the rb to be used
 */
int rb_is_empty(const rb_t * rb);

/**
 * krb_is_full - returns true if the rb is full
 * @rb: address of the rb to be used
 */
int rb_is_full(const rb_t * rb);

/**
 * rb_put - put data into the rb
 * @rb: address of the rb to be used
 * @val: the data to be added
 *
 * This macro copies the given value into the rb.
 * It returns 0 if the rb was full. Otherwise it returns the number
 * processed elements.
 *
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these macro.
 */
int rb_put(rb_t * rb, double value, uint32_t msec);

/**
 * rb_peek_n - get data from the rb at a position between 0 .. len
 * @rb: address of the rb to be used
 * @n: position to peek at
 * @val: the var where to store the data to be added
 * @msec: the var where to store the data to be added
 *
 * This macro reads the data from the rb.
 * It returns 0 if the rb was empty. Otherwise it returns the number
 * processed elements.
 *
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these macro.
 */
int rb_peek_n(const rb_t * rb, uint32_t n, double * val, uint32_t * msec);

/**
 * rb_get - get data from the rb
 * @rb: address of the rb to be used
 * @val: the var where to store the data to be added
 *
 * This macro reads the data from the rb.
 * It returns 0 if the rb was empty. Otherwise it returns the number
 * processed elements.
 *
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these macro.
 */
int rb_get(rb_t * rb, double * value, uint32_t * msec);

#ifdef __cplusplus
}
#endif

#endif // _RING_BUFFER_H_
