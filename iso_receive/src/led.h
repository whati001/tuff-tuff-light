#ifndef LED_H
#define LED_H

#include "ttl.h"

// from datasheet:
//  * periode: 1kHz = 1ms
//  * duty: 10%
#define TTL_PWM_LED_PERIODE 1000000
#define TTL_PWM_LED_DUTY(x) (TTL_PWM_LED_PERIODE / 100 * (x - 1))

/**
 * @brief Enable the TTLight GPIO stack
 * This function initiates the GPIO port
 * and starts to poll the current state
 */
int ttl_led_init();

int ttl_led_upd_status(ttl_state_t state);

#endif