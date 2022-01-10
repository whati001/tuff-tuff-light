#include "trailer_listener.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(trailer_listener);

// gpios from which we need to read from to get current state
// the array position should fit the SIGNAL enum postions
// there is no need to define an input pin for running
// we are always in running mode if we are connected with a peripheral
static const struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0})};

static struct gpio_callback gpio_cb_data;
static struct trailer_listener *interrupt_active;

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
        LOG_ERR("GPIO port %s is not ready", gpios[REVERSE].port->name);
        err = -2;
        goto cleanup;
    }

    LOG_INF("GPIO port is ready, configure all the pins properly.");
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        err = gpio_pin_configure_dt(&gpios[i], GPIO_INPUT);
        if (err)
        {
            LOG_ERR("Failed to configure gpio pin: %d\n", gpios[i].pin);
            goto cleanup;
        }
    }
    listener->interrupt_cb = _trailer_listener_gpio_cb;

cleanup:
    if (err < 0)
    {
        memset(listener, 0, sizeof(struct trailer_listener));
    }
    return err;
}

void _trailer_listener_gpio_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    enum SIGNAL signal;
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        if (pins == BIT(gpios[i].pin))
        {
            signal = i;
        }
    }
    LOG_DBG("Received interrupt by pin: %d which is signal: %d", gpios[signal].pin, signal);
    if (interrupt_active)
    {
        interrupt_active->values[signal] ^= 1;
    }
}

int trailer_listener_register_interrupt_cb(struct trailer_listener *listener)
{
    int err = 0;
    uint32_t int_bit_mask = 0;
    LOG_INF("Start to configure interrupt for all trailer listener gpio pins");
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        err = gpio_pin_interrupt_configure_dt(&gpios[i], GPIO_INT_EDGE_BOTH);
        if (err)
        {
            LOG_ERR("Failed to configure interrupt for trailer listener pin: %d", gpios[i].pin);
            goto cleanup;
        }
        LOG_DBG("Configured interrupt for gpio pin: %d", gpios[i].pin);
        int_bit_mask |= BIT(gpios[i].pin);
    }

    gpio_init_callback(&gpio_cb_data, listener->interrupt_cb, int_bit_mask);
    gpio_add_callback(gpios[0].port, &gpio_cb_data);
    interrupt_active = listener;
    LOG_INF("Successfully configured all trailer listener gpio intrrupts");

cleanup:

    return err;
}

int trailer_listener_poll_state(struct trailer_listener *listener)
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

int trailer_listener_get_state(struct trailer_listener *listener, uint8_t *state)
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

int trailer_listener_get_raw(struct trailer_listener *listener, uint8_t *values)
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