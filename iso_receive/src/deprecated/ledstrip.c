#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>

#include "gpio.h"

#define DT_DRV_COMPAT nrf52840dk

LOG_MODULE_REGISTER(ttl_gpio, LOG_LEVEL_INF);

// ttl gpio thread objects
#define TTL_GPIO_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_gpio_thread_stack, TTL_GPIO_STACK_SIZE);
static struct k_thread ttl_gpio_thread_data;

static ttl_light_orientiation_t orientation;
static uint8_t running;
static uint8_t ttl_state_changed;
static ttl_state_t ttl_state;

static K_SEM_DEFINE(sem_ttl_state, 1, 1);

#define STRIP_NUM_PIXELS DT_PROP(DT_ALIAS(led_strip), chain_length)
static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

#define TTF_LEDS_DT DT_NODELABEL(signal_leds)
#define TTF_BREAK_LED_DT DT_CHILD(TTF_LEDS_DT, break_led)
static const struct gpio_dt_spec gpio_break = GPIO_DT_SPEC_GET_BY_IDX(TTF_BREAK_LED_DT, gpios, 0);

// define led strip
#define RGBW(_r, _g, _b, _w)                       \
    {                                              \
        .r = (_r), .g = (_g), .b = (_b), .w = (_w) \
    }

static const struct led_rgb color_modes[] = {
    RGBW(0x00, 0x00, 0x00, 0xff), /* reverse */
    RGBW(0xff, 0x00, 0x00, 0x00), /* break */
    RGBW(0xff, 0xff, 0x00, 0x00), /* turn */
    RGBW(0x40, 0x00, 0x00, 0x00)  /* drive */
};
struct led_rgb pixels[STRIP_NUM_PIXELS];

static int ttl_gpio_enable()
{
    if (!device_is_ready(strip))
    {
        LOG_ERR("LED strip device %s is not ready", strip->name);
        return TTL_ERR;
    }
    LOG_INF("Found LED strip device %s", strip->name);
    memset(pixels, 0, sizeof(struct led_rgb) * STRIP_NUM_PIXELS);

    if (!gpio_is_ready_dt(&gpio_break))
    {
        LOG_ERR("BREAK device is not ready");
        return TTL_ERR;
    }
    if (gpio_pin_configure_dt(&gpio_break, GPIO_OUTPUT_ACTIVE) < 0)
    {
        LOG_ERR("BREAK device is not ready");
        return TTL_ERR;
    }
    gpio_pin_set(gpio_break.port, gpio_break.pin, 0);

    return TTL_OK;
}

static uint8_t ttl_gpio_map_aio_state()
{
    uint8_t state = TTL_LIGHT_DRIVE;
    if (ttl_state.parts.bits.breaks)
    {
        state = TTL_LIGHT_BREAK;
    }
    else if (orientation == ORI_LEFT && ttl_state.parts.bits.lturn)
    {
        state = TTL_LIGHT_TURN;
    }
    else if (orientation == ORI_RIGHT && ttl_state.parts.bits.rturn)
    {
        state = TTL_LIGHT_TURN;
    }
    else if (ttl_state.parts.bits.reverse)
    {
        state = TTL_LIGHT_REVERSE;
    }
    else
    {
        state = TTL_LIGHT_DRIVE;
    }

    return state;
}

static int ttl_gpio_set()
{
    ttl_state_t state;
    uint8_t state_changed;
    while (running)
    {
        k_sem_take(&sem_ttl_state, K_FOREVER);
        state_changed = ttl_state_changed;
        // TODO: add logic to differentiate if this is a right or left light
        state = ttl_state;
        k_sem_give(&sem_ttl_state);

        if (state_changed)
        {
            LOG_INF("Received trailer light state update to:");
            PRINT_TTL_STATE(state);
            ttl_state_changed = 0;

            // TODO: update all connected light components
            // for AIO light
            uint8_t aio_light_state = ttl_gpio_map_aio_state(state);

            for (uint8_t idx = 0; idx < ARRAY_SIZE(pixels); idx++)
            {
                memcpy(&pixels[idx], &color_modes[aio_light_state], sizeof(struct led_rgb));
            }

            LOG_ERR("Update LED srip\n");
            int ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            if (ret)
            {
                LOG_ERR("Failed to update led strip");
            }

            // for BREAK light
            if (aio_light_state == TTL_LIGHT_BREAK)
            {
                LOG_INF("Set BREAK gpio to high");
                gpio_pin_set(gpio_break.port, gpio_break.pin, 1);
            }
            else
            {
                LOG_INF("Set BREAK gpio to low");
                gpio_pin_set(gpio_break.port, gpio_break.pin, 0);
            }
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

    LOG_INF("TTLight starts to set current GPIO state");
    ret = ttl_gpio_set();
    if (TTL_OK != ret)
    {
        LOG_ERR("Failed to set the current TTLight GPIO state");
        return;
    }
    LOG_INF("TTLight finished to set current GPIO state");

    return;
}

int ttl_gpio_init()
{
    running = 1;
    orientation = ORI_LEFT;
    // TODO: Set to running
    ttl_state.entire = 0;
    ttl_state_changed = 0;
    /* Start a thread to offload disk ops */
    k_thread_create(&ttl_gpio_thread_data, ttl_gpio_thread_stack,
                    TTL_GPIO_STACK_SIZE,
                    (k_thread_entry_t)ttl_gpio_thread_main, NULL, NULL, NULL,
                    -5, 0, K_NO_WAIT);
    k_thread_name_set(&ttl_gpio_thread_data, "ttl_ble_worker");

    return TTL_OK;
}

int inline ttl_gpio_upd_status(ttl_state_t state)
{
    if (state.entire != ttl_state.entire)
    {
        k_sem_take(&sem_ttl_state, K_FOREVER);
        ttl_state_changed = 1;
        ttl_state = state;
        k_sem_give(&sem_ttl_state);
    }

    return TTL_OK;
}