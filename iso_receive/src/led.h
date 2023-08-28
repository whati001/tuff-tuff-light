#ifndef GPIO_H
#define GPIO_H

#include <zephyr/drivers/pwm.h>
#include <nrfx_pwm.h>
#include <nrfx.h>
#include <haly/nrfy_pwm.h>

#include "ttl.h"

#define TTL_PWM_LED_PERIODE 250000000
#define TTL_PWM_LED_DUTY (250000000 / 8)

/**
 * @brief Enable the TTLight GPIO stack
 * This function initiates the GPIO port
 * and starts to poll the current state
 */
int ttl_led_init();

int ttl_led_upd_status(ttl_state_t state);

/*
 * @brief Very nasty functions to connect and disconnect the PWM channels, as
 * explained in the NRF documentation: https://infocenter.nordicsemi.com/topic/ps_nrf52840/pwm.html?cp=5_0_0_5_16_4_22#register.PSEL.OUT-0-3
 * This allows us to disable and enable the PWM signal on the GPIO port without
 * the need to reconfigure the entire PWM instance. However, these hacky functions
 * only work if the first member of the dev->config struct is the nrfx_pwm_t instance.
 * The device config struct is private in the C file, hence we are unable to access its definition.
 * But because the first required information is a the beginning of the config struct we simply
 * access the nrfx_pwm_t directly, because they own the same address in memory.
 */
static void inline ttl_led_connect_channel(const struct pwm_dt_spec *pwm)
{
    nrfx_pwm_t *nrfx_pwm = (nrfx_pwm_t *)pwm->dev->config;
    nrfx_pwm->p_reg->PSEL.OUT[pwm->channel] &= ~(0x1 << PWM_PSEL_OUT_CONNECT_Pos);
}

static void inline ttl_led_disconnect_channel(const struct pwm_dt_spec *pwm)
{
    nrfx_pwm_t *nrfx_pwm = (nrfx_pwm_t *)pwm->dev->config;
    nrfx_pwm->p_reg->PSEL.OUT[pwm->channel] |= (0x1 << PWM_PSEL_OUT_CONNECT_Pos);
}

#endif