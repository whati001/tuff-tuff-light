#ifndef ACCEL_H
#define ACCEL_H

#include "ttl.h"
#include <zephyr/drivers/sensor/adxl345.h>

/**
 * @brief Enable the TTLight accel stack
 * @return TTL_OK on success, else some error code
 */
ttl_err_t ttl_accel_init(void);

/**
 * @brief Read a new sensor value from the accel
 * @param[out] val - read sensor value
 * @return TTL_OK on success, else some error code
 */
ttl_err_t ttl_accel_read(struct sensor_value *val);

#endif