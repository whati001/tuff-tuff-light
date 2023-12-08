#ifndef ACCEL_H
#define ACCEL_H

#include <zephyr/drivers/sensor/adxl345.h>

/**
 * @brief Enable the TTLight accel stack
 * @return TTL_OK on success, else some error code
 */
int ttl_accel_init();

/**
 * @brief Read a new sensor value from the accel
 * @param[out] val - read sensor value
 * @return TTL_OK on success, else some error code
 */
int ttl_accel_read(struct sensor_value *val);

#endif