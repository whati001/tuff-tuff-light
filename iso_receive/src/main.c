
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>

#define DONGLE 0

#if DONGLE == 1
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#endif

#include "accel.h"
#include "ble.h"
#include "led.h"
#include "ttl.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

/**
 * @brief Reset pin, used by the accelerometer to wakeup the boot
 */
static const struct gpio_dt_spec reboot_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(reboot_pin), gpios);

/**
 * @brief Enable USB logging. Only relevant for nrf52840dongle
 */
void enable_usb_logging() {
#if DONGLE == 1
  const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
  uint32_t dtr = 0;

  if (usb_enable(NULL)) {
    return;
  }

  /* Poll if the DTR flag was set */
  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    /* Give CPU resources to low priority threads. */
    k_sleep(K_MSEC(100));
  }
#endif
}

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
  int err = ttl_accel_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight ACCEL stack\n");
    return TTL_ERR;
  }
  int rc = gpio_pin_configure_dt(&reboot_pin, GPIO_INPUT);
  if (rc < 0) {
    LOG_ERR("Could not configure sw0 GPIO (%d)\n", rc);
    return TTL_ERR;
  }

  rc = gpio_pin_interrupt_configure_dt(&reboot_pin, GPIO_INT_LEVEL_ACTIVE);
  if (rc < 0) {
    LOG_ERR("Could not configure sw0 GPIO interrupt (%d)\n", rc);
    return TTL_ERR;
  }

  // perform some power down if no ble was found for a given time
  sys_poweroff();

  return TTL_OK;
}

/**
 * @brief Main function of the TTL receiver
 */
int main(void) {
  int err = 0;
  int64_t last_packet_time;
  int64_t elapsed_time;

  LOG_INF("Starting TTLight Controller\n");
  enable_usb_logging();

  err = ttl_ble_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight BLE stack\n");
    return TTL_ERR;
  }

  err = ttl_led_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
    goto exit;
  }
  ttl_ble_register_cb(ttl_led_upd_status);

exit:
  while (1) {
    last_packet_time = ttl_ble_latest_packet_date();
    elapsed_time = k_uptime_delta(&last_packet_time);
    LOG_INF("Elapsed time: %lld", elapsed_time);
    k_msleep(10000);
    if (elapsed_time >= (CONFIG_TTL_SHUTDOWN_TIMEOUT * 1000)) {
      err = ttl_power_down();
      if (TTL_OK != err) {
        LOG_ERR("Failed to shutdown, let's force a restart of the board");
        sys_reboot(SYS_REBOOT_COLD);
      }
    }
  }
  return 0;
}