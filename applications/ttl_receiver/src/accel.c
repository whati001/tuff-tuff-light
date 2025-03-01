
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "accel.h"

LOG_MODULE_REGISTER(ttl_accel, LOG_LEVEL_INF);

const struct device *const adx345 = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

static void ttl_accel_active(const struct device *dev,
                             const struct sensor_trigger *trigger) {
  LOG_INF("ACCEL ACTIVE");
}

ttl_err_t ttl_accel_init(void) {
  int rc;
  if (!device_is_ready(adx345)) {
    LOG_ERR("%s: device not ready.\n", adx345->name);
    return TTL_ERR;
  }

  // register active interrupt
  const struct sensor_value active_mg = {.val1 = 29, .val2 = 419950};
  rc = sensor_attr_set(adx345, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ACTIVE_THRESH,
                       &active_mg);
  if (0 != rc) {
    LOG_ERR("Failed to set accel active threshold value");
    return TTL_ERR;
  }

  struct sensor_trigger active_trigger;
  active_trigger.type = SENSOR_TRIG_MOTION;
  rc = sensor_trigger_set(adx345, &active_trigger, ttl_accel_active);
  if (0 != rc) {
    LOG_ERR("Failed to activate accel active trigger");
    return TTL_ERR;
  }

  return TTL_OK;
}

ttl_err_t ttl_accel_read(struct sensor_value *val) {
  struct sensor_value accel[3];
  int ret = sensor_sample_fetch(adx345);
  if (ret < 0) {
    LOG_ERR("Failed to fetch accel sensor data\n");
    return TTL_ERR;
  }

  if (!ret) {
    LOG_WRN("No data to sample\n");
    return TTL_OK;
  }

  ret = sensor_channel_get(adx345, SENSOR_CHAN_ACCEL_XYZ, &accel[0]);
  if (ret < 0) {
    LOG_ERR("Failed to read accel sensor data\n");
    return TTL_ERR;
  }

  LOG_INF("AccelData[x: %f, y:%f, z:%f]", sensor_value_to_double(&accel[0]),
          sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));

  return TTL_OK;
}