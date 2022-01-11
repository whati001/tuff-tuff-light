#include <zephyr.h>
#include <logging/log.h>
#include <inttypes.h>

#include "trailer_listener.h"
#include "ttl_central.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define SLEEP_TIME_MS 50

static struct trailer_listener listener;
static uint8_t left_state = RUNNING;
static uint8_t right_state = RUNNING;

uint8_t map_listner_state(uint8_t *vals, uint8_t len, uint8_t *left_state, uint8_t *right_state)
{
    if (vals[BREAK])
    {
        *left_state = TTL_LIGHT_BREAK;
        *right_state = TTL_LIGHT_BREAK;
    }
    else if (vals[TURN_LEFT])
    {
        *left_state = TTL_LIGHT_TURN;
        *right_state = TTL_LIGHT_RUNNING;
    }
    else if (vals[TURN_RIGHT])
    {
        *left_state = TTL_LIGHT_RUNNING;
        *right_state = TTL_LIGHT_TURN;
    }
    else if (vals[REVERSE])
    {
        *left_state = TTL_LIGHT_REVERSE;
        *right_state = TTL_LIGHT_REVERSE;
    }
    else
    {
        *left_state = TTL_LIGHT_RUNNING;
        *right_state = TTL_LIGHT_RUNNING;
    }

    return 0;
}

void main()
{

    int err = 0;
    uint8_t state = 0;

    // create a new instance for the listner
    err = trailer_listener_init(&listener, map_listner_state);
    if (err)
    {
        LOG_ERR("Failed to initiated trailer listener, stop app");
        return;
    }
    LOG_INF("Initiated new trailer listener instance properly");
    err = trailer_listener_register_interrupt_cb(&listener);
    if (err)
    {
        LOG_ERR("Failed to initialize listner interrupts");
        return;
    }

    err = ttl_central_connect();
    if (err)
    {
        LOG_ERR("Failed to connect to right tuff tuff trailer light");
        return;
    }

    while (1)
    {
        k_msleep(SLEEP_TIME_MS);
        if (listener.state_changed)
        {
            listener.state_changed = 0;

            err = trailer_listener_get_state(&listener, &left_state, &right_state);
            if (err)
            {
                LOG_ERR("Failed to read trailer state, skip update");
                continue;
            }
            LOG_INF("Received map state: %d", state);

            err = ttl_right_sent_state(&right_state);
            if (err)
            {
                LOG_ERR("Failed to send updated state to right tuff tuff light");
                continue;
            }
            err = ttl_left_sent_state(&left_state);
            if (err)
            {
                LOG_ERR("Failed to send updated state to right tuff tuff light");
                continue;
            }
            LOG_INF("Send updated state %d to tuff tuff lights", state);
        }
    }
}