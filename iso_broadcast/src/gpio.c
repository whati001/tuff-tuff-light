#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "gpio.h"

LOG_MODULE_REGISTER(ttl_gpio, LOG_LEVEL_INF);

// ttl gpio thread objects
#define TTL_GPIO_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_gpio_thread_stack, TTL_GPIO_STACK_SIZE);
static struct k_thread ttl_gpio_thread_data;

// ttl gpio hardware mapping
enum GPIO_MAPPING
{
    REVERSE = 0,
    BREAK,
    TURN_LEFT,
    TURN_RIGHT,
    RUNNING
};
static const struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0})};

// ttl gpio runtime variables
#define TTL_GPIO_VALUES_LEN 5
static uint8_t running;
static uint8_t values[TTL_GPIO_VALUES_LEN];
static ttl_state_t ttl_state;
static ttl_gpio_upd_state_cb_t ttl_gpio_upd_state_cb;

static void ttl_gpio_map_state()
{
    if (values[BREAK])
    {
        ttl_state.parts.left.parts.state = TTL_LIGHT_BREAK;
        ttl_state.parts.right.parts.state = TTL_LIGHT_BREAK;
    }
    else if (values[TURN_LEFT])
    {
        ttl_state.parts.left.parts.state = TTL_LIGHT_TURN;
        ttl_state.parts.right.parts.state = TTL_LIGHT_DRIVE;
    }
    else if (values[TURN_RIGHT])
    {
        ttl_state.parts.left.parts.state = TTL_LIGHT_DRIVE;
        ttl_state.parts.right.parts.state = TTL_LIGHT_TURN;
    }
    else if (values[REVERSE])
    {
        ttl_state.parts.left.parts.state = TTL_LIGHT_REVERSE;
        ttl_state.parts.right.parts.state = TTL_LIGHT_REVERSE;
    }
    else
    {
        ttl_state.parts.left.parts.state = TTL_LIGHT_DRIVE;
        ttl_state.parts.right.parts.state = TTL_LIGHT_DRIVE;
    }
}

static int ttl_gpio_enable()
{
    int ret = 0;

    memset(values, 0, TTL_GPIO_VALUES_LEN);
    if (!device_is_ready(gpios[REVERSE].port))
    {
        LOG_ERR("GPIO port %s is not ready", gpios[REVERSE].port->name);
        return TTL_ERR;
    }

    LOG_INF("GPIO port is ready, configure all the pins properly.");
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        ret = gpio_pin_configure_dt(&gpios[i], GPIO_INPUT);
        if (ret)
        {
            LOG_ERR("Failed to configure gpio pin: %d\n", gpios[i].pin);
            return TTL_ERR;
        }
    }

    return TTL_OK;
}

static int ttl_gpio_upd_state()
{
    if (ttl_gpio_upd_state_cb)
    {
        return (*ttl_gpio_upd_state_cb)(ttl_state);
    }
    return TTL_OK;
}

static void ttl_gpio_poll_port()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++)
    {
        values[i] = gpio_pin_get_dt(&gpios[i]);
    }
}

static int ttl_gpio_poll()
{
    int ret;
    while (running)
    {
        ttl_gpio_poll_port();
        LOG_DBG("Polled current ttl gpio values:[%d,%d,%d,%d]", values[0], values[1], values[2], values[3]);

        ttl_gpio_map_state();
        LOG_DBG("ttl_state_t: {left: %d, right: %d}", ttl_state.parts.left.parts.state, ttl_state.parts.right.parts.state);

        ret = ttl_gpio_upd_state();
        if (TTL_OK != ret)
        {
            LOG_ERR("Failed to update current polled state");
            continue;
        }

        k_msleep(TTL_POLLING_INTERVAL_MS);
    }

    return TTL_OK;
}

static void ttl_gpio_thread_main()
{
    int ret = 0;
    LOG_INF("TTLight starts to initiate the GPIO Stack");
    ret = ttl_gpio_enable();
    if (TTL_OK != ret)
    {
        LOG_ERR("Failed to initiate the TTLight GPIO stack");
        return;
    }
    LOG_INF("TTLight started the GPIO Stack successfully");

    LOG_INF("TTLight starts to poll current GPIO state");
    ret = ttl_gpio_poll();
    if (TTL_OK != ret)
    {
        LOG_ERR("Failed to poll the current TTLight GPIO state");
        return;
    }
    LOG_INF("TTLight finished to poll current GPIO state");

    return;
}

void ttl_gpio_register_cb(ttl_gpio_upd_state_cb_t cb)
{
    if (cb)
    {
        ttl_gpio_upd_state_cb = cb;
    }
}

int ttl_gpio_init()
{
    running = 1;
    ttl_gpio_upd_state_cb = NULL;
    ttl_state.entire = 0;
    ttl_state.parts.left.parts.magic = 0x27;
    ttl_state.parts.right.parts.magic = 0x11;
    /* Start a thread to offload disk ops */
    k_thread_create(&ttl_gpio_thread_data, ttl_gpio_thread_stack,
                    TTL_GPIO_STACK_SIZE,
                    (k_thread_entry_t)ttl_gpio_thread_main, NULL, NULL, NULL,
                    -5, 0, K_NO_WAIT);
    k_thread_name_set(&ttl_gpio_thread_data, "ttl_ble_worker");

    return TTL_OK;
}