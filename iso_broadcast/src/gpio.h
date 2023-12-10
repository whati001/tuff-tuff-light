#ifndef GPIO_H
#define GPIO_H

#include "ttl.h"

/**
 * @brief Enable the TTLight GPIO stack
 * This function initiates the GPIO port
 * and starts to poll the current state
 */
int ttl_gpio_init();

/**
 * @brief Register callback function which should be get triggered once the
 * internal ttl state changes.
 * @param[in] cb - callback to execute on ttl state changes
 */
void ttl_gpio_register_cb(ttl_upd_state_cb_t cb);

#endif