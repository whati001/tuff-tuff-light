#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gpio.h"

LOG_MODULE_REGISTER(ttl_gpio, LOG_LEVEL_INF);

// ttl GPIO thread objects
#define TTL_GPIO_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_gpio_thread_stack, TTL_GPIO_STACK_SIZE);
static struct k_thread ttl_gpio_thread_data;

// ttl GPIO hardware mapping
enum GPIO_MAPPING { BREAK = 0, TURN_LEFT, TURN_RIGHT };
static const struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(ttl_break), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(ttl_tleft), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(ttl_tright), gpios, {0})};

// ttl gpio runtime variables
#define TTL_GPIO_VALUES_LEN 3
static uint8_t values[TTL_GPIO_VALUES_LEN];
static ttl_state_t ttl_state;
static ttl_upd_state_cb_t ttl_gpio_upd_state_cb;

/**
 * @brief Map GPIO input value to ttl state
 */
static void ttl_gpio_map_state(void) {
  ttl_state.entire = 0;
  ttl_state.parts.magic = 0x27;
  ttl_state.parts.reserved = 0x11;

  ttl_state.parts.bits.breaks = values[BREAK] & 0x01;
  ttl_state.parts.bits.lturn = values[TURN_LEFT] & 0x01;
  ttl_state.parts.bits.rturn = values[TURN_RIGHT] & 0x01;
  ttl_state.parts.bits.ldrive = 1;
  ttl_state.parts.bits.rdrive = 1;
  // TODO: not supported yet
  ttl_state.parts.bits.reverse = 0;
  ttl_state.parts.bits.fog = 0;
}

/**
 * @brief Configure GPIO pins as specified in devicetree file
 */
static int ttl_gpio_configure(void) {
  int ret = 0;

  memset(values, 0, ARRAY_SIZE(values));
  if (!device_is_ready(gpios[BREAK].port)) {
    LOG_ERR("GPIO port %s is not ready", gpios[BREAK].port->name);
    return TTL_ERR;
  }

  LOG_INF("GPIO port is ready, configure all the pins properly.");
  for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++) {
    ret = gpio_pin_configure_dt(&gpios[i], GPIO_INPUT);
    if (ret) {
      LOG_ERR("Failed to configure gpio pin: %d\n", gpios[i].pin);
      return TTL_ERR;
    }
  }

  return TTL_OK;
}

/**
 * @brief Run GPIO update function is set
 */
static int ttl_gpio_upd_state() {
  if (ttl_gpio_upd_state_cb) {
    return (*ttl_gpio_upd_state_cb)(ttl_state);
  }
  return TTL_OK;
}

/**
 * @brief Poll current TTL state from GPIO pins
 */
static void ttl_gpio_poll_port(void) {
  for (uint8_t i = 0; i < ARRAY_SIZE(gpios); i++) {
    values[i] = gpio_pin_get_dt(&gpios[i]);
  }
}

/**
 * @brief Main GPIO thread loop
 */
static void ttl_gpio_loop(void) {
  int ret;

  while (true) {
    ttl_gpio_poll_port();
    LOG_DBG("Polled current ttl gpio values:[%d,%d,%d]", values[0], values[1],
            values[2]);

    ttl_gpio_map_state();

    ret = ttl_gpio_upd_state();
    if (TTL_OK != ret) {
      LOG_ERR("Failed to update current polled state");
      continue;
    }

    k_msleep(CONFIG_TTL_GPIO_POLLING_INTERVAL_MS);
  }
}

void ttl_gpio_register_cb(ttl_upd_state_cb_t cb) {
  if (cb) {
    ttl_gpio_upd_state_cb = cb;
  }
}

ttl_err_t ttl_gpio_init() {
  int ret = 0;

  LOG_INF("TTLight starts to initiate the GPIO Stack");
  ret = ttl_gpio_configure();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to initiate the TTLight GPIO stack");
    return TTL_ERR;
  }

  ttl_gpio_upd_state_cb = NULL;
  ttl_state.entire = 0;

  LOG_INF("TTLight started the GPIO Stack successfully");

  return TTL_OK;
}

ttl_err_t ttl_gpio_run() {
  /* Start a thread to sample TTLight GPIO state independently from the main
   * thread*/
  k_thread_create(&ttl_gpio_thread_data, ttl_gpio_thread_stack,
                  TTL_GPIO_STACK_SIZE, (k_thread_entry_t)ttl_gpio_loop, NULL,
                  NULL, NULL, 1, 0, K_NO_WAIT);
  k_thread_name_set(&ttl_gpio_thread_data, "ttl_ble_worker");

  return TTL_OK;
}

ttl_err_t ttl_gpio_terminate() {
  k_thread_abort(&ttl_gpio_thread_data);
  return TTL_OK;
}