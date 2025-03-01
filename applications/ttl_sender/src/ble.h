#ifndef BLE_H
#define BLE_H

#include "ttl.h"

/**
 * @brief Initiate TTL BLE module.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_ble_init(void);

/**
 * @brief Run the TTLight BLE logic.
 * This includes spawning a new thread, enabling BLE, creating an ISO channel
 * and sending the current ttl state. The ttl state can be updated via function
 * #ttl_ble_upd_status.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_ble_run(void);

/**
 * @brief Small helper function to update the current
 * ttl state variable. After the call, the BLE stack
 * will broadcast the updated status value
 *
 * @param[in] state - ttl state to update
 * @return #ttl_err_t
 */
ttl_err_t ttl_ble_upd_status(ttl_state_t state);

#endif