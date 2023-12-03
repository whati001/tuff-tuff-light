
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>

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

static const struct gpio_dt_spec reboot_pin =
    GPIO_DT_SPEC_GET(DT_NODELABEL(reboot_pin), gpios);

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

static int ttl_power_down() {
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

int main(void) {
  int err = 0;
  LOG_INF("Starting TTLight Controller\n");
  enable_usb_logging();

  // err = ttl_ble_init();
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

  err = ttl_accel_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight ACCEL stack\n");
    goto exit;
  }

  k_msleep(1000);
  ttl_power_down();

exit:
  while (1) {
    k_msleep(1000);
    struct sensor_value val;
    ttl_accel_read(&val);
  }
  return 0;
}