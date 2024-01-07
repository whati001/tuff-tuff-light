#ifndef GPIO_H
#define GPIO_H

#include "ttl.h"

/**
 * @brief Enable the TTLight GPIO stack. This function initiates the GPIO port
 * and brings up a dedicated thread to sample the GPIO state. Please use
 * @ttl_gpio_start and @ttl_gpio_stop to enable and disable sampling.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_gpio_init();

/**
 * @brief Start sampling the current TTLight GPIO state in dedicated thread.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_gpio_start();

/**
 * @brief Stop sampling the current TTLight GPIO state in dedicated thread. This
 * function does not destroy the thread.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_gpio_stop();

/**
 * @brief Register callback function which should be get triggered once the
 * internal ttl state changes.
 * @param[in] cb - callback to execute on ttl state changes
 */
void ttl_gpio_register_cb(ttl_upd_state_cb_t cb);

#endif