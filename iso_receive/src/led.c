#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "led.h"

#define DT_DRV_COMPAT nrf52840dk

LOG_MODULE_REGISTER(ttl_led, LOG_LEVEL_INF);

// ttl gpio thread objects
#define TTL_LED_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_led_thread_stack, TTL_LED_STACK_SIZE);
static struct k_thread ttl_led_thread_data;

static ttl_light_orientiation_t orientation;
static uint8_t running;
static uint8_t ttl_state_changed;
static ttl_state_t ttl_state;

static K_SEM_DEFINE(sem_ttl_state, 1, 1);

// PWM LEDs devices
struct ttl_pwm_led_t
{
    const struct pwm_dt_spec pwm;
    const uint32_t pwm_periode;
    const uint32_t pwm_duty;
};

static struct ttl_pwm_led_t ttl_pwm_leds[] = {
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_break)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY},
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_dirpointer)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY},
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_reverse)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY},
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_drive)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY}

};

static int ttl_led_enable()
{
    int ret = TTL_OK;
    for (uint8_t idx = 0; idx < ARRAY_SIZE(ttl_pwm_leds); idx++)
    {
        ret = device_is_ready(ttl_pwm_leds[idx].pwm.dev);
        if (false == ret)
        {
            LOG_ERR("PWM LED device is not ready with name: %s and channel: %d", ttl_pwm_leds[idx].pwm.dev->name, ttl_pwm_leds[idx].pwm.channel);
            return TTL_ERR;
        }
        LOG_INF("PWM LED device is ready device with name:  %s and channel: %d", ttl_pwm_leds[idx].pwm.dev->name, ttl_pwm_leds[idx].pwm.channel);

        ret = pwm_set_dt(&ttl_pwm_leds[idx].pwm, ttl_pwm_leds[idx].pwm_periode, ttl_pwm_leds[idx].pwm_duty);
        if (ret)
        {
            LOG_ERR("Error %d: failed to set pulse width\n", ret);
            return TTL_ERR;
        }
    }

    // init state
    ttl_state_changed = 1;
    ttl_state.entire = 0;
    ttl_state.parts.bits.rdrive = 1;
    ttl_state.parts.bits.ldrive = 1;

    return TTL_OK;
}

static uint8_t ttl_led_map_aio_state()
{
    uint8_t state = TTL_LIGHT_DRIVE;
    // if (ttl_state.parts.bits.breaks)
    // {
    //     state = TTL_LIGHT_BREAK;
    // }
    // else if (orientation == ORI_LEFT && ttl_state.parts.bits.lturn)
    // {
    //     state = TTL_LIGHT_TURN;
    // }
    // else if (orientation == ORI_RIGHT && ttl_state.parts.bits.rturn)
    // {
    //     state = TTL_LIGHT_TURN;
    // }
    // else if (ttl_state.parts.bits.reverse)
    // {
    //     state = TTL_LIGHT_REVERSE;
    // }
    // else
    // {
    //     state = TTL_LIGHT_DRIVE;
    // }

    return state;
}

static int ttl_led_set()
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

            // k_sleep(K_SECONDS(4));
            // ttl_led_disconnect_channel(&pwmled_break);
            // k_sleep(K_SECONDS(4));
            // ttl_led_connect_channel(&pwmled_break);

            // // TODO: update all connected light components
            // // for AIO light
            // uint8_t aio_light_state = ttl_led_map_aio_state(state);

            // for (uint8_t idx = 0; idx < ARRAY_SIZE(pixels); idx++)
            // {
            //     memcpy(&pixels[idx], &color_modes[aio_light_state], sizeof(struct led_rgb));
            // }

            // LOG_ERR("Update LED srip\n");
            // int ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
            // if (ret)
            // {
            //     LOG_ERR("Failed to update led strip");
            // }

            // // for BREAK light
            // if (aio_light_state == TTL_LIGHT_BREAK)
            // {
            //     LOG_INF("Set BREAK gpio to high");
            //     gpio_pin_set(gpio_break.port, gpio_break.pin, 1);
            // }
            // else
            // {
            //     LOG_INF("Set BREAK gpio to low");
            //     gpio_pin_set(gpio_break.port, gpio_break.pin, 0);
            // }
        }

        k_msleep(TTL_POLLING_INTERVAL_MS);
    }

    return TTL_OK;
}

static void ttl_led_thread_main()
{
    int ret = 0;
    LOG_INF("TTLight starts to initiate the GPIO Stack");
    ret = ttl_led_enable();
    if (TTL_OK != ret)
    {
        LOG_ERR("Failed to initiate the TTLight GPIO stack");
        return;
    }
    LOG_INF("TTLight started the GPIO Stack successfully");

    LOG_INF("TTLight starts to set current GPIO state");
    ret = ttl_led_set();
    if (TTL_OK != ret)
    {
        LOG_ERR("Failed to set the current TTLight GPIO state");
        return;
    }
    LOG_INF("TTLight finished to set current GPIO state");

    return;
}

int ttl_led_init()
{
    running = 1;
    orientation = ORI_LEFT;
    // TODO: Set to running
    ttl_state.entire = 0;
    ttl_state_changed = 0;
    /* Start a thread to offload disk ops */
    k_thread_create(&ttl_led_thread_data, ttl_led_thread_stack,
                    TTL_LED_STACK_SIZE,
                    (k_thread_entry_t)ttl_led_thread_main, NULL, NULL, NULL,
                    -5, 0, K_NO_WAIT);
    k_thread_name_set(&ttl_led_thread_data, "ttl_ble_worker");

    return TTL_OK;
}

int inline ttl_led_upd_status(ttl_state_t state)
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