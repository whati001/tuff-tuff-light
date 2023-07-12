#ifndef BLE_H
#define BLE_H

/**
 * @brief Enable the TTLight BLE stack
 * This will enable the entire BLE with ISO stack
 * and initialize a new BIS and starts sending the current
 * ttl state to the receivers
 */
int ttl_ble_init();

void ttl_ble_register_cb(ttl_upd_state_cb_t cb);

#endif