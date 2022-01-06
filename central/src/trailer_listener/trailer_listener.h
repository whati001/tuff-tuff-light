#ifndef TRAILER_LISTENER_H
#define TRAILER_LISTENER_H

#include <device.h>
#include <drivers/gpio.h>
#include <inttypes.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>

// helper macro to check if some value is null
#define CHECK_NULL(name, ret)                                                        \
    if (NULL == name)                                                                \
    {                                                                                \
        printk("Error: passed null name to trailer_listener_get_values function\n"); \
        err = ret;                                                                   \
        goto cleanup;                                                                \
    }

/*
 * Define how many values we will read and store later
 */
#define TRAILER_LISTENER_VALUES_LEN 5

/*
 * Define all the possible signals
 *  in addition, we will read them
 *  exactly within this order from the GPIO port
 */
enum SIGNAL
{
    RUNNING = 0,
    REVERSE,
    BREAK,
    TURN_LEFT,
    TURN_RIGHT
};
static enum SIGNAL signals;

struct trailer_listener
{
    uint8_t values[TRAILER_LISTENER_VALUES_LEN];
    uint8_t (*map_internal_state)(uint8_t *vals, uint8_t len, enum SIGNAL signals);
};

/*
 * Init something
 */
int trailer_listener_init(struct trailer_listener *listener, uint8_t (*map_internal_state)(uint8_t *vals, uint8_t len, enum SIGNAL signals));

/*
 * Read the state and store the result into the passed listener instance
 */
int trailer_listener_read_state(struct trailer_listener *listener);

/*
 * Simply return the values which get fetched by the
 * trailer_listener_read_state function
 * this function will copy the values
 */
int trailer_listener_get_values(struct trailer_listener *listener, uint8_t *values);

/*
 * Map the values to an uint8_t state, logic is based via map_internal_state cb
 * if not define, return -1
 */
int trailer_listener_get_mapped_state(struct trailer_listener *listener, uint8_t *state);

#endif