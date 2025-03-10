
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>

#include "ble.h"
#include "led.h"
#include "ttl.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

/**
 * @brief Reset pin, used by spring resistor to wakeup the boot
 */
static const struct gpio_dt_spec reboot_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(reboot_pin), gpios);

/**
 * @brief Reset pin, used by the accelerometer to wakeup the boot
 */
static const struct gpio_dt_spec booster_enable_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(booster_pin), gpios);

/**
 * @brief Overwrite ASSERT handler
 * @param reason - The reason for the fatal error
 * @param esf    - Exception context, with details and partial or full register
 *                 state when the error occurred. May in some cases be NULL.
 */
void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf) {
  ARG_UNUSED(esf);
  LOG_ERR("Assert received, reboot system");
  sys_reboot(SYS_REBOOT_COLD);
  CODE_UNREACHABLE; /* LCOV_EXCL_LINE */
}

/**
 * @brief Power down TTL to save energy.
 * This function activates the acceleration monitoring and wakeups the MCU once
 * a movement is recognized.
 */
static int ttl_power_down() {
  int err;

  // stop all components
  ttl_ble_terminate();
  ttl_led_terminate();
  LOG_INF("Shutting down the system");

  // disable booster for leds
  err = gpio_pin_set_dt(&booster_enable_pin, 0);
  if (err < 0) {
    LOG_ERR("Could not enable booster GPIO (%d)\n", err);
    return TTL_ERR;
  }

  // wait some time before shuting down the system
  k_msleep(2000);

  // overwrite gpio interrupt pin config to be a user wakeup pin
  err = gpio_pin_configure_dt(&reboot_pin, GPIO_INPUT);
  if (err < 0) {
    LOG_ERR("Could not configure sw0 GPIO (%d)\n", err);
    return TTL_ERR;
  }

  err = gpio_pin_interrupt_configure_dt(&reboot_pin, GPIO_INT_LEVEL_ACTIVE);
  if (err < 0) {
    LOG_ERR("Could not configure sw0 GPIO interrupt (%d)\n", err);
    return TTL_ERR;
  }

  sys_poweroff();

  return TTL_OK;
}

/**
 * @brief Main function of the TTL receiver
 */
int main(void) {
  ttl_err_t err = 0;
  int64_t last_packet_time_ms;
  int64_t elapsed_time_ms;

  LOG_INF("Starting TTLight Controller\n");

  err = ttl_ble_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight BLE stack\n");
    return TTL_ERR;
  }
  LOG_INF("Initialized TTLight BLE stack properly");

  err = ttl_led_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
    goto exit;
  }
  ttl_ble_register_cb(ttl_led_upd_status);
  LOG_INF("Initialized TTLight LED stack properly");

  err = ttl_led_run();
  if (TTL_OK != err) {
    LOG_ERR("Failed to start TTLight LED stack");
    return TTL_ERR;
  }
  LOG_INF("Started TTLight LED stack properly");

  err = ttl_ble_run();
  if (TTL_OK != err) {
    LOG_ERR("Failed to start TTLight BLE stack");
    return TTL_ERR;
  }
  LOG_INF("Started TTLight BLE stack properly");

  // enable booster for leds
  err = gpio_pin_configure_dt(&booster_enable_pin, GPIO_OUTPUT);
  if (err < 0) {
    LOG_ERR("Could not configure booster GPIO (%d)\n", err);
    return TTL_ERR;
  }
  err = gpio_pin_set_dt(&booster_enable_pin, 1);
  if (err < 0) {
    LOG_ERR("Could not enable booster GPIO (%d)\n", err);
    return TTL_ERR;
  }

exit:
  while (1) {
    last_packet_time_ms = ttl_ble_latest_packet_datetime_ms();
    elapsed_time_ms = k_uptime_delta(&last_packet_time_ms);
    LOG_INF("Elapsed time(ms): %lld", elapsed_time_ms);
    k_msleep(1000);

    if (elapsed_time_ms > (CONFIG_TTL_SHUTDOWN_TIMEOUT_SEC * 1000)) {
      err = ttl_power_down();
      if (TTL_OK != err) {
        LOG_ERR("Failed to shutdown, let's force a restart of the board");
        k_msleep(2000);
        sys_reboot(SYS_REBOOT_COLD);
      }
    }
  }
  return 0;
}