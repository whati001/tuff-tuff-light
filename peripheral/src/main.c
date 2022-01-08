/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log.h>

#include "sstage.h"
#include "trailer_light.h"

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/*
 * Ble specific configuration and callbacks
 */
static struct bt_conn *current_conn;

void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);

struct bt_conn_cb bluetooth_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};
struct bt_remote_service_cb remote_callbacks = {
	.data_received = on_data_received,
};

void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		LOG_ERR("connection err: %d", err);
		return;
	}
	LOG_INF("Ble connection with central established");
	current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason: %d) with ble central", reason);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

/*
 * callback function passed to ble sstage_service
 * code which get executed if some data is received by the central
 */
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	uint8_t action = *data;
	LOG_INF("Received data on conn %p. Len: %d", (void *)conn, len);
	LOG_INF("ActionCode: %d", action);

	if (action >= 0 && action <= 3)
	{
		int err = trailer_light_update(action);
		if (err)
		{
			LOG_ERR("Failed to update trailer light for action id: %d\n", action);
		}
	}
	else
	{
		LOG_WRN("Received out of range signal code from central");
	}
}

void main(void)
{
	int err = 0;

	err = sstage_init(&bluetooth_callbacks, &remote_callbacks);
	if (err)
	{
		LOG_ERR("Failed to initiate ble service, returned %d", err);
		return;
	}
	err = trailer_light_init();
	if (err)
	{
		LOG_ERR("Failed to initiate trailer light, returned %d", err);
		return;
	}

	err = trailer_light_update(RUNNING);
	if (err)
	{
		LOG_ERR("Failed to initiated trailer light into RUNNING state");
		return;
	}

	LOG_INF("Started tuff tuff light peripheral successfully");
}
