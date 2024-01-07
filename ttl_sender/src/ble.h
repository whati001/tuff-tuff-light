#ifndef BLE_H
#define BLE_H

#include "ttl.h"

/**
 * @brief Initiate the TTLight BLE stack to use extended periodic advertisement
 * mode.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_ble_init();

/**
 * @brief Start periodic extended advertisement of TTLState via BLE.
 * Please initiate the TTLight BLE stack via @ttl_ble_init before starting the
 * advertisement.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_ble_start();

/**
 * @brief Stop periodic extended advertisement of TTLState via BLE. If
 * advertisement is not started yet, the function returns silently.
 * @return TTL_OK on success, else TTL_ERR
 */
int tll_ble_stop();

/**
 * @brief Small helper function to update the current
 * ttl state variable. After the call, the BLE stack
 * will broadcast the updated status value.
 * @return TTL_OK on success, else TTL_ERR
 */
int ttl_ble_upd_status(ttl_state_t state);

#endif