#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <haly/nrfy_pwm.h>
#include <nrfx.h>
#include <nrfx_pwm.h>
#include <zephyr/drivers/pwm.h>

#include "led.h"

#define DT_DRV_COMPAT nrf52840dk

LOG_MODULE_REGISTER(ttl_led, LOG_LEVEL_INF);

// ttl gpio thread objects
#define TTL_LED_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_led_thread_stack, TTL_LED_STACK_SIZE);
static struct k_thread ttl_led_thread_data;

static ttl_state_t ttl_state;

static K_SEM_DEFINE(sem_ttl_state, 1, 1);
static K_SEM_DEFINE(sem_ttl_update, 1, 1);

// PWM LEDs devices
struct ttl_pwm_led_t {
  const struct pwm_dt_spec pwm;
  const uint32_t pwm_periode;
  const uint32_t pwm_duty;
};

enum ttl_pwm_led_idx {
  TTL_PWM_IDX_BREAK = 0,
  TTL_PWM_IDX_DIRPOINTER,
  // TTL_PWM_IDX_REVERSE, // not supported yet
  TTL_PWM_IDX_DRIVE,
};

static struct ttl_pwm_led_t ttl_pwm_leds[] = {
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_break)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY(100)},
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_turn)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY(100)},
    {.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_drive)),
     .pwm_periode = TTL_PWM_LED_PERIODE,
     .pwm_duty = TTL_PWM_LED_DUTY(100)},
};

/*
 * @brief Very nasty functions to connect and disconnect the PWM channels, as
 * explained in the NRF documentation:
 * https://infocenter.nordicsemi.com/topic/ps_nrf52840/pwm.html?cp=5_0_0_5_16_4_22#register.PSEL.OUT-0-3
 * This allows us to disable and enable the PWM signal on the GPIO port without
 * the need to reconfigure the entire PWM instance. However, these hacky
 * functions only work if the first member of the dev->config struct is the
 * nrfx_pwm_t instance. The device config struct is private in the C file, hence
 * we are unable to access its definition. But because the first required
 * information is a the beginning of the config struct we simply access the
 * nrfx_pwm_t directly, because they own the same address in memory.
 */
static void inline ttl_led_connect(struct ttl_pwm_led_t *led) {
  int ret = pwm_set_dt(&led->pwm, led->pwm_periode, led->pwm_duty);
  if (ret) {
    LOG_ERR("Error %d: failed to set pulse width\n", ret);
    return;
  }

  const struct pwm_dt_spec *pwm = &led->pwm;
  nrfx_pwm_t *nrfx_pwm = (nrfx_pwm_t *)pwm->dev->config;
  nrfx_pwm->p_reg->PSEL.OUT[pwm->channel] &= ~(0x1 << PWM_PSEL_OUT_CONNECT_Pos);
}

static void inline ttl_led_disconnect(struct ttl_pwm_led_t *led) {
  const struct pwm_dt_spec *pwm = &led->pwm;
  nrfx_pwm_t *nrfx_pwm = (nrfx_pwm_t *)pwm->dev->config;
  nrfx_pwm->p_reg->PSEL.OUT[pwm->channel] |= (0x1 << PWM_PSEL_OUT_CONNECT_Pos);
}

static void ttl_led_disconnect_all() {
  for (uint8_t idx = 0; idx < ARRAY_SIZE(ttl_pwm_leds); idx++) {
    ttl_led_disconnect(&ttl_pwm_leds[idx]);
  }
}

static int ttl_led_loop() {
  ttl_state_t state;

  while (true) {
    // block until other signals that we have to update our leds
    LOG_INF("Take update sem");
    k_sem_take(&sem_ttl_update, K_FOREVER);

    LOG_INF("Take ttl state sem");
    // fetch new value
    k_sem_take(&sem_ttl_state, K_FOREVER);
    state = ttl_state;
    k_sem_give(&sem_ttl_state);

    LOG_INF("Received trailer light state update to:");
    PRINT_TTL_STATE(state);

    // disconnect all LEDs
    ttl_led_disconnect_all();

    // TODO: add logic to differentiate if this is a right or left light
    if (state.parts.bits.rturn) {
      LOG_INF("Enable DIRECTION LIGHT");
      ttl_led_connect(&ttl_pwm_leds[TTL_PWM_IDX_DIRPOINTER]);
    }
    // TODO: change to a if
    else if (state.parts.bits.breaks) {
      LOG_INF("Enable BREAK LIGHT");
      ttl_led_connect(&ttl_pwm_leds[TTL_PWM_IDX_BREAK]);
      // } else if (state.parts.bits.reverse) {
      //   LOG_INF("Enable REVERSE LIGHT");
      //   ttl_led_connect(&ttl_pwm_leds[TTL_PWM_IDX_REVERSE]);
    } else if (state.parts.bits.ldrive) {
      LOG_INF("Enable DRIVE LIGHT");
      ttl_led_connect(&ttl_pwm_leds[TTL_PWM_IDX_DRIVE]);
    } else {
      LOG_INF("Nothing to enable???");
    }
  }

  return TTL_OK;
}

ttl_err_t ttl_led_init() {
  int ret = TTL_OK;

  LOG_INF("TTLight starts to initiate the GPIO stack");
  for (uint8_t idx = 0; idx < ARRAY_SIZE(ttl_pwm_leds); idx++) {
    ret = device_is_ready(ttl_pwm_leds[idx].pwm.dev);
    if (false == ret) {
      LOG_ERR("PWM LED device is not ready with name: %s and channel: %d",
              ttl_pwm_leds[idx].pwm.dev->name, ttl_pwm_leds[idx].pwm.channel);
      ret = TTL_ERR;
      break;
    }
    LOG_INF("PWM LED device is ready device with name:  %s and channel: %d",
            ttl_pwm_leds[idx].pwm.dev->name, ttl_pwm_leds[idx].pwm.channel);

    ret = pwm_set_dt(&ttl_pwm_leds[idx].pwm, ttl_pwm_leds[idx].pwm_periode,
                     ttl_pwm_leds[idx].pwm_duty);
    if (ret) {
      LOG_ERR("Error %d: failed to set pulse width\n", ret);
      ret = TTL_ERR;
      break;
    }
  }

  if (TTL_OK != ret) {
    LOG_ERR("Failed to initiate the TTLight GPIO stack");
    return ret;
  }
  LOG_INF("TTLight initiated the GPIO stack successfully");

  // init state
  ttl_state.entire = 0;
  ttl_state.parts.bits.rdrive = 1;
  ttl_state.parts.bits.ldrive = 1;

  // ttl_led_disconnect_all();

  return ret;
}

ttl_err_t ttl_led_run() {
  /* Start a thread to offload disk ops */
  k_thread_create(&ttl_led_thread_data, ttl_led_thread_stack,
                  TTL_LED_STACK_SIZE, (k_thread_entry_t)ttl_led_loop, NULL,
                  NULL, NULL, 2, 0, K_NO_WAIT);
  k_thread_name_set(&ttl_led_thread_data, "ttl_ble_worker");

  return TTL_OK;
}

ttl_err_t ttl_led_terminate() {
  // kill the thread
  k_thread_abort(&ttl_led_thread_data);

  // set all LEDs to off
  for (uint8_t idx = 0; idx < ARRAY_SIZE(ttl_pwm_leds); idx++) {
    pwm_set_dt(&ttl_pwm_leds[idx].pwm, ttl_pwm_leds[idx].pwm_periode, 0);
  }
  ttl_led_disconnect_all();

  return TTL_OK;
}

ttl_err_t inline ttl_led_upd_status(ttl_state_t state) {
  if (state.entire != ttl_state.entire) {
    k_sem_take(&sem_ttl_state, K_FOREVER);
    ttl_state = state;
    k_sem_give(&sem_ttl_state);

    // signal led thread to update led state
    k_sem_give(&sem_ttl_update);
  }

  return TTL_OK;
}