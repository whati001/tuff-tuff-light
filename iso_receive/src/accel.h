#ifndef ACCEL_H
#define ACCEL_H

#include <zephyr/drivers/sensor.h>

/**
 * @brief Enable the TTLight accel stack
 */
int ttl_accel_init();

/**
 * @brief Read a new sensor value from the accel
 */
int ttl_accel_read(struct sensor_value *val);

#endif