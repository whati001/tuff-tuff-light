#ifndef LED_H
#define LED_H

#include "ttl.h"

// from datasheet:
//  * periode: 1kHz = 1ms
//  * duty: 10%
#define TTL_PWM_LED_PERIODE 1000000
#define TTL_PWM_LED_DUTY(x) (TTL_PWM_LED_PERIODE / 100 * (x - 1))

/**
 * @brief Enable the TTLight LED stack and start a new thread to update LED
 * state periodically. The actually update the LED state, please use
 * @ttl_led_start and @ttl_led_stop.
 * @return TTL_OK on success, else TTL_ERR
 */
ttl_err_t ttl_led_init();

/**
 * @brief Start periodically updating the external LEDs with the current TTLight
 * state.
 * @return TTL_OK on success, else TTL_ERR
 */
ttl_err_t ttl_led_run();

/**
 * @brief Stop periodically updating the external LEDs with the current TTLight
 * state.
 * @return TTL_OK on success, else TTL_ERR
 */
ttl_err_t ttl_led_terminate();

/**
 * @brief Callback function to update internal led status.
 * By executing this callback status, other threads can request a led change.
 * @param[in] state - new ttl state to light up
 * @return TTL_OK on success, else TTL_ERR
 */
ttl_err_t ttl_led_upd_status(ttl_state_t state);

#endif