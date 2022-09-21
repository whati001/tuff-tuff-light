/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log.h>

#include "sstate.h"
#include "trailer_light.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// this should be half of the host transmission speed
// furthermore, if the connection to the host, it does twice of this time to output the NO_HOST_STATE
#define SLEEP_MS_TIME 100

// current state and marker if the value was updated used by the trailer-light object
static uint8_t tl_state = DRIVE;
static uint8_t tl_update = 0;

// active bluetooth connection if established
static struct bt_conn *current_conn;

// forward declaration of the ble methods
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

// ble connection callbacks
struct bt_conn_cb bluetooth_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

// ble remote callbacks
struct bt_remote_service_cb remote_callbacks = {
	.data_received = on_data_received,
};

// ble on_connected callback implementation
void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		LOG_ERR("Failed to establish ble connection with err: %d", err);
		return;
	}
	LOG_INF("Successfully ble connection established with central");
	current_conn = bt_conn_ref(conn);
}

// ble on_disconnect callback implementation
void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected ble connection (reason: %d) with ble central", reason);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

// ble remote on_data callback -> passed to custom ble GATT called sstate
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	uint8_t recv = *data;
	LOG_DBG("Received data on conn %p. Len: %d", (void *)conn, len);
	LOG_INF("Received State: %d", recv);

	if (recv >= 0 && recv <= 3)
	{
		// update the state and marker value
		tl_state = recv;
		tl_update = 1;

		int err = trailer_light_update(tl_state);
		if (err)
		{
			LOG_ERR("Failed to update trailer light for action id: %d", tl_state);
		}
	}
	else
	{
		LOG_WRN("Received out of range signal code from central");
	}
}

// main function, start ble and advertise custom GATT (sstate)
void main(void)
{
	int err = 0;

	// start BLE interface and advertizing
	err = sstate_init(&bluetooth_callbacks, &remote_callbacks);
	if (err)
	{
		LOG_ERR("Failed to initiate ble service, returned %d", err);
		return;
	}
	LOG_INF("Finished to initialize the BLE interface and started to advertize the custom GATT");

	// start the trailer light interface -> allows to push states to the leds
	err = trailer_light_init();
	if (err)
	{
		LOG_ERR("Failed to initiate trailer light, returned %d", err);
		return;
	}
	LOG_INF("Finished to initialize trailer light interface");

	// TODO: maybe it's a better idea to use a timer instead of this loop
	// the idea is, that the host updates the current state twice as fast as the
	// peripheral updates the lights, hence the tl_update should be never 0 and
	// always tl_state should always hold the current status.
	// if no update was received from the host, the NO_HOSTE_STATE will be shown
	while (1)
	{
		k_msleep(SLEEP_MS_TIME);
		if (0 == tl_update)
		{
			tl_state = NO_HOST_STATE;
			LOG_WRN("No update received from host, set state to NO_HOST_STATE");
		}

		tl_update = 0;
		int err = trailer_light_update(tl_state);
		if (err)
		{
			LOG_ERR("Failed to update trailer light to new state: %d", tl_state);
		}
	}
}
