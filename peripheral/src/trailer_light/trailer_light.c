#include <logging/log.h>
#include "trailer_light.h"

#define LOG_MODULE_NAME trailer_light
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STRIP_NUM_PIXELS DT_PROP(DT_NODELABEL(led_strip), chain_length)
static const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(led_strip));

struct led_rgb pixels[STRIP_NUM_PIXELS];
static uint8_t state_value = 0;

int trailer_light_init()
{
    int err = 0;
    if (!device_is_ready(strip))
    {
        LOG_ERR("LED strip device %s is not ready", strip->name);
        err = -1;
        goto cleanup;
    }
    LOG_INF("Found LED strip device %s", strip->name);

cleanup:
    memset(pixels, 0, sizeof(struct led_rgb) * STRIP_NUM_PIXELS);
    state_value = 0;
    return err;
}

int trailer_light_update(enum STATES state)
{
    int err = 0;
    state_value = state;
    LOG_INF("Received trailer light state update to %d", state_value);

    for (uint8_t idx = 0; idx < ARRAY_SIZE(pixels); idx++)
    {
        memcpy(&pixels[idx], &color_modes[state_value], sizeof(struct led_rgb));
    }

    err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
    if (err)
    {
        LOG_ERR("Failed to update led strip");
    }

    return err;
}
