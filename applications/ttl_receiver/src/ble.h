#ifndef BLE_H
#define BLE_H

#include "ttl.h"
#include <stdbool.h>
#include <sys/types.h>

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
 * @brief Terminates the TTL BLE module.
 *
 * @return #ttl_err_t
 */
ttl_err_t ttl_ble_terminate(void);

/**
 * @brief Register new callback function, triggered per received data package
 *
 * @param[in] cb - callback to register
 */
void ttl_ble_register_cb(ttl_upd_state_cb_t cb);

/**
 * @brief Query if the ble stack is currently connected to some iso channel
 *
 * @return true if connected else false
 */
bool ttl_ble_is_connected(void);

/**
 * @brief Query the latest received ble iso packet datetime.
 * Please consider that this date will be the thread start time if no iso
 * connection was made yet.
 *
 * @return time_t of last packet
 */
int64_t ttl_ble_latest_packet_datetime(void);

#endif