#ifndef GPIO_H
#define GPIO_H

#include "ttl.h"

/**
 * @brief Initiate the TTL GPIO module.
 * This function prepares the internal data struct and configures the GPIO pins.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_gpio_init(void);

/**
 * @brief Start sampling the current TTLight GPIO state in dedicated thread.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_gpio_run(void);

/**
 * @brief Stop sampling the current TTLight GPIO state in dedicated thread. This
 * function does not destroy the thread.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_gpio_terminate(void);

/**
 * @brief Register callback function which should be get triggered once the
 * internal ttl state changes.
 *
 * @param[in] cb - callback to execute on ttl state changes
 */
void ttl_gpio_register_cb(ttl_upd_state_cb_t cb);

#endif