#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "gpio.h"

LOG_MODULE_REGISTER(ttl_gpio, LOG_LEVEL_INF);

// ttl gpio thread objects
#define TTL_GPIO_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_gpio_thread_stack, TTL_GPIO_STACK_SIZE);
static struct k_thread ttl_gpio_thread_data;

static uint8_t running;
static ttl_state_t ttl_state;

static K_SEM_DEFINE(sem_ttl_state, 1, 1);

static void ttl_gpio_thread_main()
{
    // int ret = 0;
    // LOG_INF("TTLight starts to initiate the GPIO Stack");
    // ret = ttl_gpio_enable();
    // if (TTL_OK != ret)
    // {
    //     LOG_ERR("Failed to initiate the TTLight GPIO stack");
    //     return;
    // }
    // LOG_INF("TTLight started the GPIO Stack successfully");

    // LOG_INF("TTLight starts to poll current GPIO state");
    // ret = ttl_gpio_poll();
    // if (TTL_OK != ret)
    // {
    //     LOG_ERR("Failed to poll the current TTLight GPIO state");
    //     return;
    // }
    // LOG_INF("TTLight finished to poll current GPIO state");
    while (1)
    {
        k_sem_take(&sem_ttl_state, K_FOREVER);
        LOG_INF("TTL-State: left.magic: %x, left.state: %d, right.magic: %x, right.state: %d\n", ttl_state.parts.left.parts.magic, ttl_state.parts.left.parts.state, ttl_state.parts.right.parts.magic, ttl_state.parts.right.parts.state);
        k_sem_give(&sem_ttl_state);

        k_msleep(100);
    }
    return;
}

int ttl_gpio_init()
{
    running = 1;
    ttl_state.entire = 0;
    /* Start a thread to offload disk ops */
    k_thread_create(&ttl_gpio_thread_data, ttl_gpio_thread_stack,
                    TTL_GPIO_STACK_SIZE,
                    (k_thread_entry_t)ttl_gpio_thread_main, NULL, NULL, NULL,
                    -5, 0, K_NO_WAIT);
    k_thread_name_set(&ttl_gpio_thread_data, "ttl_ble_worker");

    return TTL_OK;
}

int inline ttl_gpio_upd_status(ttl_state_t state)
{
    if (state.entire != ttl_state.entire)
    {
        k_sem_take(&sem_ttl_state, K_FOREVER);
        ttl_state = state;
        k_sem_give(&sem_ttl_state);
    }

    return TTL_OK;
}