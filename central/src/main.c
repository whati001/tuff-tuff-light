#include <zephyr.h>
#include <sys/printk.h>
#include <inttypes.h>

#include "trailer_listener/trailer_listener.h"

#define SLEEP_TIME_MS 1000

static struct trailer_listener listener;

uint8_t map_listner_state(uint8_t *vals, uint8_t len, enum SIGNAL signals)
{
    for (uint8_t i = 0; i < len; i++)
    {
        printk("MapListnerState[%d] = %d\n", i, vals[i]);
    }
    return 0;
}

int main()
{

    int err = 0;
    uint8_t state = 0;
    // create a new instance for the listner
    err = trailer_listener_init(&listener, map_listner_state);
    if (err)
    {
        printk("Failed to initiated trailer listener, stop app\n");
        return;
    }
    printk("Initiated new trailer listener instance properly\n");

    while (1)
    {
        err = trailer_listener_read_state(&listener);
        if (err)
        {
            printk("Error: Failed to read trailer state, skip update\n");
            continue;
        }
        err = trailer_listener_get_mapped_state(&listener, &state);
        if (err)
        {
            printk("Error: Failed to read trailer state, skip update\n");
            continue;
        }
        printk("Received map state: %d\n", state);

        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}