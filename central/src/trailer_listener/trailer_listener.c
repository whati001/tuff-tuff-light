#include "trailer_listener.h"

// gpios from which we need to read from to get current state
// the array position should fit the SIGNAL enum postions
// there is no need to define an input pin for running
// we are always in running mode if we are connected with a peripheral
static const struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0})};

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

    // all the devices should share the same gpio port, thus we just need to check ones
    if (!device_is_ready(gpios[REVERSE].port))
    {
        printk("Error: GPIO port %s is not ready\n", gpios[REVERSE].port->name);
        err = -2;
        goto cleanup;
    }
    printk("GPIO port is ready, configure all the pins properly.\n");
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        err = gpio_pin_configure_dt(&gpios[i], GPIO_INPUT);
        if (err)
        {
            printk("Error: Failed to configure gpio pin: %d\n", gpios[i].pin);
            goto cleanup;
        }
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

    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        listener->values[i] = gpio_pin_get_dt(&gpios[i]);
    }
    listener->values[RUNNING] = 1;

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