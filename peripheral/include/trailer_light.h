#ifndef TRAILER_LIGHT_H
#define TRAILER_LIGHT_H

#include <device.h>
#include <drivers/spi.h>
#include <drivers/led_strip.h>

// define led strip
#define RGB(_r, _g, _b)                 \
    {                                   \
        .r = (_r), .g = (_g), .b = (_b) \
    }

static const struct led_rgb color_modes[] = {
    RGB(0xff, 0xff, 0xff), /* reverse */
    RGB(0xff, 0x00, 0x00), /* break */
    RGB(0xff, 0xff, 0x00), /* indicator */
    RGB(0x0f, 0x00, 0x00), /* normal */
};

enum STATES
{
    REVERSE = 0,
    BREAK,
    TURN,
    DRIVE
};
#define DEF_STATE DRIVE
#define NO_HOST_STATE DEF_STATE

struct trailer_light
{
    uint8_t state;
};

int trailer_light_init();

int trailer_light_update(enum STATES state);

#endif