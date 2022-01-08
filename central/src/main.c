#include <zephyr.h>
#include <logging/log.h>
#include <inttypes.h>

#include "trailer_listener.h"
#include "ttl_central.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define SLEEP_TIME_MS 10

static struct trailer_listener listener;

uint8_t map_listner_state(uint8_t *vals, uint8_t len, enum SIGNAL signals)
{
    // for (uint8_t i = 0; i < len; i++)
    // {
    //     LOG_INF("MapListnerState[%d] = %d", i, vals[i]);
    // }
    if (vals[BREAK])
    {
        return BREAK;
    }
    if (vals[TURN_LEFT] || vals[TURN_RIGHT])
    {
        return TURN_LEFT;
    }
    if (vals[REVERSE])
    {
        return REVERSE;
    }
    return RUNNING - 1;
}

void main()
{

    int err = 0;
    uint8_t state = 0;
    uint8_t old_state = 0;
    // create a new instance for the listner
    err = trailer_listener_init(&listener, map_listner_state);
    if (err)
    {
        LOG_ERR("Failed to initiated trailer listener, stop app");
        return;
    }
    LOG_INF("Initiated new trailer listener instance properly");

    err = ttl_central_connect();
    if (err)
    {
        LOG_ERR("Failed to connect to right tuff tuff trailer light");
        return;
    }

    while (1)
    {
        k_msleep(SLEEP_TIME_MS);
        err = trailer_listener_read_state(&listener);
        if (err)
        {
            LOG_ERR("Failed to read trailer state, skip update");
            continue;
        }
        err = trailer_listener_get_mapped_state(&listener, &state);
        if (err)
        {
            LOG_ERR("Failed to read trailer state, skip update");
            continue;
        }
        LOG_INF("Received map state: %d", state);

        if (old_state != state)
        {
            err = ttl_right_sent_state(&state);
            if (err)
            {
                LOG_ERR("Failed to send updated state to right tuff tuff light");
                continue;
            }
            LOG_INF("Send updated state %d to tuff tuff lights", state);
            old_state = state;
        }
    }
}