/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/byteorder.h>

#include "ttf.h"

LOG_MODULE_REGISTER(ttf_ble, LOG_LEVEL_INF);

#define BUF_ALLOC_TIMEOUT (10)						 /* milliseconds */
#define BIG_TERMINATE_TIMEOUT_US (60 * USEC_PER_SEC) /* microseconds */
#define BIG_SDU_INTERVAL_US (10000)

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, 1,
						  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
						  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);
static K_SEM_DEFINE(sem_iso_data, CONFIG_BT_ISO_TX_BUF_COUNT,
					CONFIG_BT_ISO_TX_BUF_COUNT);

static uint8_t running;
static uint16_t seq_num;
static ttf_state_t ttf_state;
static K_SEM_DEFINE(sem_ttf_state, 1, 1);
static uint8_t iso_data[sizeof(ttf_state)] = {0};

// ttf ble thread objects
#define TTF_BLE_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttf_ble_thread_stack, TTF_BLE_STACK_SIZE);
static struct k_thread ttf_ble_thread_data;

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	seq_num = 0U;
	k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
	k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	k_sem_give(&sem_iso_data);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = sizeof(uint32_t), /* bytes */
	.rtn = 1,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
	.tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
	.ops = &iso_ops,
	.qos = &bis_iso_qos,
};

static struct bt_iso_chan *bis[] = {
	&bis_iso_chan};

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = 1,
	.bis_channels = bis,
	.interval = BIG_SDU_INTERVAL_US, /* in microseconds */
	.latency = 10,					 /* in milliseconds */
	.packing = 0,					 /* 0 - sequential, 1 - interleaved */
	.framing = 0,					 /* 0 - unframed, 1 - framed */
};

static int ttf_ble_enable()
{
	struct bt_le_ext_adv *adv;
	struct bt_iso_big *big;
	int err;

	LOG_INF("Starting to initiate BLE stack for TTF-Light");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return TTF_ERR;
	}
	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
	if (err)
	{
		LOG_ERR("Failed to create advertising set (err %d)", err);
		return TTF_ERR;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err)
	{
		LOG_ERR("Failed to set periodic advertising parameters (err %d)", err);
		return TTF_ERR;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err)
	{
		LOG_ERR("Failed to enable periodic advertising (err %d)", err);
		return TTF_ERR;
	}

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err)
	{
		LOG_ERR("Failed to start extended advertising (err %d)", err);
		return TTF_ERR;
	}

	/* Create BIG */
	err = bt_iso_big_create(adv, &big_create_param, &big);
	if (err)
	{
		LOG_ERR("Failed to create BIG (err %d)", err);
		return TTF_ERR;
	}

	LOG_INF("Waiting for BIG complete chan...");
	err = k_sem_take(&sem_big_cmplt, K_FOREVER);
	if (err)
	{
		LOG_ERR("failed (err %d)", err);
		return TTF_ERR;
	}
	LOG_INF("BIG create complete chan.\n");

	return TTF_OK;
}

static int ttf_ble_broadcast()
{
	while (running)
	{
		struct net_buf *buf;
		int ret;

		buf = net_buf_alloc(&bis_tx_pool, K_MSEC(BUF_ALLOC_TIMEOUT));
		if (!buf)
		{
			printk("Data buffer allocate timeout on channel");
			return 0;
		}

		ret = k_sem_take(&sem_iso_data, K_FOREVER);
		if (ret)
		{
			printk("k_sem_take for ISO data sent failed\n");
			net_buf_unref(buf);
			return 0;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		// NOTE: we do not check the error code, because it anyway waits FOREVER
		k_sem_take(&sem_ttf_state, K_FOREVER);
		sys_put_le32(ttf_state, iso_data);
		k_sem_give(&sem_ttf_state);
		net_buf_add_mem(buf, iso_data, sizeof(iso_data));

		ret = bt_iso_chan_send(&bis_iso_chan, buf, seq_num, BT_ISO_TIMESTAMP_NONE);
		if (ret < 0)
		{
			printk("Unable to broadcast data on channel with error code: %d", ret);
			net_buf_unref(buf);
			return 0;
		}

		printk("Sending value %u\n", ttf_state);
		seq_num++;
	}

	return TTF_OK;
}

static int ttf_ble_thread_main()
{
	int ret = 0;
	LOG_INF("TTF-Light starts to initiate the BLE Stack");
	ret = ttf_ble_enable();
	if (TTF_OK != ret)
	{
		LOG_ERR("Failed to initiate the TTF-Light BLE stack");
		return ret;
	}
	LOG_INF("TTF-Light started the BLE Stack successfully");

	LOG_INF("TTF-Light starts to broadcast current state");
	ret = ttf_ble_broadcast();
	if (TTF_OK != ret)
	{
		LOG_ERR("Failed to broadcast the current TTF-Light BLE state");
		return ret;
	}
	LOG_INF("TTF-Light finished to broadcast current state");

	return TTF_OK;
}

int ttf_ble_init()
{
	running = 1;
	/* Start a thread to offload disk ops */
	k_thread_create(&ttf_ble_thread_data, ttf_ble_thread_stack,
					TTF_BLE_STACK_SIZE,
					(k_thread_entry_t)ttf_ble_thread_main, NULL, NULL, NULL,
					-5, 0, K_NO_WAIT);
	k_thread_name_set(&ttf_ble_thread_data, "ttf_ble_worker");

	return TTF_OK;
}

int ttf_ble_upd_status(ttf_state_t state)
{
	k_sem_take(&sem_ttf_state, K_FOREVER);
	ttf_state = state;
	k_sem_give(&sem_ttf_state);

	return TTF_OK;
}