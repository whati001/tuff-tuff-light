
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ttf.h"
#include "ble.h"

LOG_MODULE_REGISTER(ttf_main, LOG_LEVEL_INF);

int main(void)
{
    int err = 0;
    LOG_INF("Starting TTF-Light Controller\n");

    err = ttf_ble_init();
    if (TTF_OK != err)
    {
        LOG_ERR("Failed to initialize the TTF-Light BLE stack\n");
    }

    uint32_t state = 0;
    while (1)
    {
        ttf_ble_upd_status(state);
        state++;

        k_msleep(1000);
    }
    return 0;
}