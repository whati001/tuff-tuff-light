
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ttf.h"
#include "ble.h"

int main(void)
{
    int err = 0;
    printk("Starting TTF-Light Broadcaster\n");

    err = ttf_ble_init();
    if (TTF_OK != err)
    {
        printk("Failed to initialize the TTF-Light BLE stack\n");
    }
    printk("Started TTLight Broadcaster\n");

    return 0;
}