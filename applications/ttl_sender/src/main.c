
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble.h"
#include "gpio.h"
#include "ttl.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

int main(void) {
  ttl_ret_t ret = 0;
  LOG_INF("Initiating TTLight Controller\n");

  ret = ttl_ble_init();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to initialize the TTLight BLE stack");
    return ret;
  }

  ret = ttl_gpio_init();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to initialize the TTLight GPIO stack");
    return ret;
  }

  ret = ttl_gpio_run();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to start continuously TTLState sampling");
    return TTL_ERR;
  }

  ret = ttl_ble_run();
  if (TTL_OK != ret) {
    LOG_ERR(
        "Failed to start continuously sending current TTL state via    BLE\n");
    return TTL_ERR;
  }

  while (1) {
    LOG_INF("TTLight Controller is running\n");
    k_sleep(K_MSEC(1000));
  }
  return TTL_OK;
}