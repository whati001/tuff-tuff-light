#ifndef TRAILER_LISTENER_H
#define TRAILER_LISTENER_H

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <inttypes.h>
#include <sys/util.h>
#include <stdio.h>
#include <string.h>

// helper macro to check if some value is null
#define CHECK_NULL(name, ret)                                                     \
    if (NULL == name)                                                             \
    {                                                                             \
        printk("Error: passed null name to trailer_listener_get_raw function\n"); \
        err = ret;                                                                \
        goto cleanup;                                                             \
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
    REVERSE = 0,
    BREAK,
    TURN_LEFT,
    TURN_RIGHT,
    RUNNING
};
static enum SIGNAL signals;

struct trailer_listener
{
    uint8_t values[TRAILER_LISTENER_VALUES_LEN];
    uint8_t (*map_internal_state)(uint8_t *vals, uint8_t len, enum SIGNAL signals);
    void (*interrupt_cb)(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    uint8_t state_changed;
};

struct trailer_listner_interrupt_data
{
    struct trailer_listener *listener;
};

/*
 * Init something
 */
int trailer_listener_init(struct trailer_listener *listener, uint8_t (*map_internal_state)(uint8_t *vals, uint8_t len, enum SIGNAL signals));

/*
 * Register interrupt callbacks for gpio pins
 */
int trailer_listener_register_interrupt_cb(struct trailer_listener *listener);

/*
 * Internal and default callback for trailer listener gpio interrupts
 */
void _trailer_listener_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/*
 * Read the state and store the result into the passed listener instance
 */
int trailer_listener_poll_state(struct trailer_listener *listener);

/*
 * Simply return the values which get fetched by the
 * trailer_listener_poll_state function
 * this function will copy the values
 */
int trailer_listener_get_raw(struct trailer_listener *listener, uint8_t *values);

/*
 * Map the values to an uint8_t state, logic is based via map_internal_state cb
 * if not define, return -1
 */
int trailer_listener_get_state(struct trailer_listener *listener, uint8_t *state);

#endif