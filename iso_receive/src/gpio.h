#ifndef GPIO_H
#define GPIO_H

#include "ttl.h"

/**
 * @brief Enable the TTLight GPIO stack
 * This function initiates the GPIO port
 * and starts to poll the current state
 */
int ttl_gpio_init();

int ttl_gpio_upd_status(ttl_state_t state);

#endif