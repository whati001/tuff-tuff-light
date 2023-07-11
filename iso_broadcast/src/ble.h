#ifndef BLE_H
#define BLE_H

/**
 * @brief Enable the TTF-Light BLE stack
 * This will enable the entire BLE with ISO stack
 * and initialize a new BIS and starts sending the current
 * ttf state to the receivers
 */
int ttf_ble_init();

/**
 * @brief Small helper function to update the current
 * ttf state variable. After the call, the BLE stack
 * will broadcast the updated status value
 */
int ttf_ble_upd_status(ttf_state_t state);

#endif