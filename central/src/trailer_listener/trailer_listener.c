#include "trailer_listener.h"

static const struct gpio_dt_spec reverse = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static const struct gpio_dt_spec breaks = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static const struct gpio_dt_spec turn_left = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0});
static const struct gpio_dt_spec turn_right = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0});

int trailer_listener_init(struct trailer_listener *listener, uint8_t (*map_internal_state)(uint8_t *vals, uint8_t len, enum SIGNAL signals))
{
    int err = 0;
    CHECK_NULL(listener, -1);

    memset(&listener->values, 0, TRAILER_LISTENER_VALUES_LEN);
    listener->map_internal_state = NULL;
    if (map_internal_state)
    {
        listener->map_internal_state = map_internal_state;
    }

    if (!device_is_ready(reverse.port) || !device_is_ready(breaks.port) || !device_is_ready(turn_left.port) || !device_is_ready(turn_right.port))
    {
        printk("Error: GPIO ports not ready\n");
        err = -2;
        goto cleanup;
    }

    printk("Gpio device ready lets define the pins we need for reading\n");

    err = gpio_pin_configure_dt(&reverse, GPIO_INPUT);
    if (err != 0)
    {
        printk("Error: Failed to set GPIO_INPUT\n");
        goto cleanup;
    }
    err = gpio_pin_configure_dt(&breaks, GPIO_INPUT);
    if (err != 0)
    {
        printk("Error: Failed to set GPIO_INPUT\n");
        goto cleanup;
    }
    err = gpio_pin_configure_dt(&turn_left, GPIO_INPUT);
    if (err != 0)
    {
        printk("Error: Failed to set GPIO_INPUT\n");
        goto cleanup;
    }
    err = gpio_pin_configure_dt(&turn_right, GPIO_INPUT);
    if (err != 0)
    {
        printk("Error: Failed to set GPIO_INPUT\n");
        goto cleanup;
    }

cleanup:
    if (err < 0)
    {
        memset(listener, 0, sizeof(struct trailer_listener));
    }
    return err;
}

int trailer_listener_read_state(struct trailer_listener *listener)
{
    int err = 0;
    CHECK_NULL(listener, -1);

    listener->values[RUNNING] = 1;
    listener->values[REVERSE] = gpio_pin_get_dt(&reverse);
    listener->values[BREAK] = gpio_pin_get_dt(&breaks);
    listener->values[TURN_LEFT] = gpio_pin_get_dt(&turn_left);
    listener->values[TURN_RIGHT] = gpio_pin_get_dt(&turn_right);

cleanup:
    return err;
}

int trailer_listener_get_mapped_state(struct trailer_listener *listener, uint8_t *state)
{
    int err = 0;
    CHECK_NULL(listener, -1);

    if (listener->map_internal_state)
    {
        *state = listener->map_internal_state(listener->values, TRAILER_LISTENER_VALUES_LEN, signals);
    }
    else
    {
        *state = -1;
    }

cleanup:
    return err;
}

int trailer_listener_get_values(struct trailer_listener *listener, uint8_t *values)
{
    int err = 0;
    CHECK_NULL(listener, -1);
    CHECK_NULL(values, -2);

    for (uint8_t i = 0; i < TRAILER_LISTENER_VALUES_LEN; i++)
    {
        *(values + i) = listener->values[i];
    }

cleanup:
    if (err < 0)
    {
        memset(values, 0, TRAILER_LISTENER_VALUES_LEN);
    }
    return err;
}