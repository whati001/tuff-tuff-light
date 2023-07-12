#ifndef BLE_H
#define BLE_H

/**
 * @brief Enable the TTLight BLE stack
 * This will enable the entire BLE with ISO stack
 * and initialize a new BIS and starts sending the current
 * ttl state to the receivers
 */
int ttl_ble_init();

/**
 * @brief Small helper function to update the current
 * ttl state variable. After the call, the BLE stack
 * will broadcast the updated status value
 */
int ttl_ble_upd_status(ttl_state_t state);

#endif