
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble.h"
#include "gpio.h"
#include "ttl.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

int main(void) {
  int err = 0;
  LOG_INF("Starting TTLight Controller\n");

  err = ttl_ble_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight BLE stack\n");
    return TTL_ERR;
  }

  err = ttl_ble_start();
  if (TTL_OK != err) {
    LOG_ERR("Failed to start the TTLight BLE advertisement\n");
    return TTL_ERR;
  }

  err = ttl_gpio_init();
  if (TTL_OK != err) {
    LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
    return TTL_ERR;
  }
  ttl_gpio_register_cb(ttl_ble_upd_status);

  err = ttl_gpio_start();
  if (TTL_OK != err) {
    LOG_ERR("Failed to start continuously TTLState sampling");
    return TTL_ERR;
  }

  return TTL_OK;
}