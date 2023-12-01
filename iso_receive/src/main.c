
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DONGLE 0

#if DONGLE == 1
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#endif

#include "ble.h"
#include "led.h"
#include "ttl.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

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

const struct device *const adx345 = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

void ttl_read_accel_sample() {
  struct sensor_value accel[3];
  int ret = sensor_sample_fetch(adx345);
  if (ret < 0) {
    LOG_ERR("Failed to fetch accel sensor data\n");
  }

  if (!ret) {
    LOG_WRN("No data to sample\n");
  }

  ret = sensor_channel_get(adx345, SENSOR_CHAN_ACCEL_XYZ, &accel[0]);
  if (ret < 0) {
    LOG_ERR("Failed to read accel sensor data\n");
  }

  LOG_INF("AccelData[x: %f, y:%f, z:%f]", sensor_value_to_double(&accel[0]),
          sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));
}

void ttl_accel_active(const struct device *dev,
                      const struct sensor_trigger *trigger) {
  LOG_INF("ACCEL ACTIVE");
}

void ttl_accel_inactive(const struct device *dev,
                        const struct sensor_trigger *trigger) {
  LOG_INF("ACCEL NOT ACTIVE");
}

void ttl_accel_waterfall(const struct device *dev,
                         const struct sensor_trigger *trigger) {
  LOG_INF("WATERFALL INT");
  ttl_read_accel_sample();
}

void ttl_accel_data_ready(const struct device *dev,
                          const struct sensor_trigger *trigger) {
  LOG_INF("DATA READY INT");
  ttl_read_accel_sample();
}
static int ttl_acc_enable() {
  if (!device_is_ready(adx345)) {
    LOG_ERR("%s: device not ready.\n", adx345->name);
    return TTL_ERR;
  }

  // # DATA READY INT
  // struct sensor_trigger data_ready;
  // data_ready.type = SENSOR_TRIG_DATA_READY;
  // sensor_trigger_set(adx345, &data_ready, ttl_accel_data_ready);

  // # FIFO WATERFALL INT -> requires FIFO enabled
  // const struct sensor_value waterfall_limit = {.val1 = 10, .val2 = 0};
  // sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_WATERFALL_LEVEL,
  //                 &waterfall_limit);
  // struct sensor_trigger waterfall;
  // waterfall.type = SENSOR_TRIG_FIFO_WATERMARK;
  // sensor_trigger_set(adx345, &waterfall, ttl_accel_waterfall);

  // # ACTIVE INT
  const struct sensor_value active_mg = {.val1 = 3000, .val2 = 0};
  sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ACTIVE_THRESH,
                  &active_mg);
  struct sensor_trigger active_trigger;
  active_trigger.type = SENSOR_TRIG_MOTION;
  sensor_trigger_set(adx345, &active_trigger, ttl_accel_active);

  // # INACTIVE INT
  const struct sensor_value inactive_mg = {.val1 = 1500, .val2 = 0};
  sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_INACTIVE_THRESH,
                  &inactive_mg);
  const struct sensor_value inactive_time = {.val1 = 5000, .val2 = 0};
  sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_INACTIVE_TIME,
                  &inactive_time);
  struct sensor_trigger inactive_trigger;
  inactive_trigger.type = SENSOR_TRIG_STATIONARY;
  sensor_trigger_set(adx345, &inactive_trigger, ttl_accel_inactive);

  // # LINK ACTIVE AND INACTIVE
  sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_LINK_MOVEMENT,
                  NULL);
  while (1) {
    k_msleep(1000);
    // ttl_read_accel_sample();
  }
  return 0;
}

int main(void) {
  int err = 0;
  LOG_INF("Starting TTLight Controller\n");
  enable_usb_logging();

  // err = ttl_ble_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight BLE stack\n");
  }

  err = ttl_led_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
  }
  ttl_ble_register_cb(ttl_led_upd_status);

  err = ttl_acc_enable();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight Acceleromater stack\n");
  }

  while (1) {
    k_msleep(1000);
  }
  return 0;
}