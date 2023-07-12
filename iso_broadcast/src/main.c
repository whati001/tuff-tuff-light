
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ttl.h"
#include "ble.h"
#include "gpio.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

int main(void)
{
    int err = 0;
    LOG_INF("Starting TTLight Controller\n");

    err = ttl_ble_init();
    if (TTL_OK != err)
    {
        LOG_ERR("Failed to initialize the TTLight BLE stack\n");
    }

    err = ttl_gpio_init();
    if (TTL_OK != err)
    {
        LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
    }
    ttl_gpio_register_cb(ttl_ble_upd_status);

    return 0;
}