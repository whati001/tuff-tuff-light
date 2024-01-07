#ifndef BLE_H
#define BLE_H

#include "ttl.h"
#include <stdbool.h>
#include <sys/types.h>

/**
 * @brief Enable the TTLight BLE stack
 * This will enable the entire BLE with ISO stack
 * and initialize a new BIS and starts sending the current
 * ttl state to the receivers
 * @return TTL_OK on success, else some error code
 */
int ttl_ble_init();

/**
 * @brief Register new callback function, triggered per received data package
 * @param[in] cb - callback to register
 */
void ttl_ble_register_cb(ttl_upd_state_cb_t cb);

/**
 * @brief Query if the ble stack is currently connected to some iso channel
 * @return true if connected else false
 */
bool ttl_ble_is_connected();

/**
 * @brief Query the latest received ble iso packet date.
 * Please consider that this date will be the thread start time if no iso
 * connection was made yet.
 * @return time_t of last packet
 */
int64_t ttl_ble_latest_packet_date();

#endif