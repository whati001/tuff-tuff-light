
/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ttl.h"
#include "zephyr/kernel.h"

LOG_MODULE_REGISTER(ttl_ble, LOG_LEVEL_INF);

// ttl ble thread objects
#define TTL_BLE_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_ble_thread_stack, TTL_BLE_STACK_SIZE);
static struct k_thread ttl_ble_thread_data;

static ttl_upd_state_cb_t ttl_upd_state_cb;
static int64_t ttl_iso_last_data;
static bool ttl_ble_connected;

static bool per_adv_found;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static struct bt_le_per_adv_sync_param sync_create_param;
static struct bt_le_per_adv_sync *sync;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN 30

static bool data_cb(struct bt_data *data, void *user_data) {
  char *name = user_data;
  uint8_t len;

  switch (data->type) {
  case BT_DATA_NAME_SHORTENED:
  case BT_DATA_NAME_COMPLETE:
    len = MIN(data->data_len, NAME_LEN - 1);
    memcpy(name, data->data, len);
    name[len] = '\0';
    return false;
  default:
    return true;
  }
}

static const char *phy2str(uint8_t phy) {
  switch (phy) {
  case 0:
    return "No packets";
  case BT_GAP_LE_PHY_1M:
    return "LE 1M";
  case BT_GAP_LE_PHY_2M:
    return "LE 2M";
  case BT_GAP_LE_PHY_CODED:
    return "LE Coded";
  default:
    return "Unknown";
  }
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf) {
  char le_addr[BT_ADDR_LE_STR_LEN];
  char name[NAME_LEN];

  (void)memset(name, 0, sizeof(name));

  bt_data_parse(buf, data_cb, name);

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
  LOG_DBG("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i %s "
          "C:%u S:%u D:%u SR:%u E:%u Prim: %s, Secn: %s, "
          "Interval: 0x%04x (%u ms), SID: %u\n",
          le_addr, info->adv_type, info->tx_power, info->rssi, name,
          (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
          phy2str(info->primary_phy), phy2str(info->secondary_phy),
          info->interval, info->interval * 5 / 4, info->sid);

  if (!per_adv_found && info->interval) {
    per_adv_found = true;

    per_sid = info->sid;
    bt_addr_le_copy(&per_addr, info->addr);

    k_sem_give(&sem_per_adv);
  }
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync,
                    struct bt_le_per_adv_sync_synced_info *info) {
  char le_addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

  printk("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
         "Interval 0x%04x (%u ms), PHY %s\n",
         bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
         info->interval * 5 / 4, phy2str(info->phy));

  k_sem_give(&sem_per_sync);
  ttl_ble_connected = true;
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                    const struct bt_le_per_adv_sync_term_info *info) {
  char le_addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

  printk("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated\n",
         bt_le_per_adv_sync_get_index(sync), le_addr);

  k_sem_give(&sem_per_sync_lost);
  ttl_ble_connected = false;
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
                    const struct bt_le_per_adv_sync_recv_info *info,
                    struct net_buf_simple *buf) {
  char le_addr[BT_ADDR_LE_STR_LEN];
  char data_str[129];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
  bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

  LOG_DBG("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
          "RSSI %i, CTE %u, data length %u, data: %s\n",
          bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
          info->rssi, info->cte_type, buf->len, data_str);

  // TODO: use some better parsing logic
  // BLE advertizes the data as following length|id|data0|..|dataN
  // so the ttl_state look as follows: 05|FF|..data..
  ttl_state_t state;
  uint16_t ttl_state_size = sizeof(ttl_state_t) + 2;

  if (buf->len == ttl_state_size) {
    state.entire = sys_get_le32(buf->data + 2);

    if (ttl_upd_state_cb) {
      (*ttl_upd_state_cb)(state);
      ttl_iso_last_data = k_uptime_get();
    }
  }
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb, .term = term_cb, .recv = recv_cb};

int ttl_ble_init() {
  int err;
  ttl_upd_state_cb = NULL;
  ttl_ble_connected = false;
  ttl_iso_last_data = k_uptime_get();

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)\n", err);
    return TTL_ERR;
  }

  bt_le_scan_cb_register(&scan_callbacks);
  LOG_INF("Scan callbacks registered successfully");

  bt_le_per_adv_sync_cb_register(&sync_callbacks);
  LOG_INF("Periodic Advertising callbacks registered successfully");

  return TTL_OK;
}

static void ttl_ble_run() {
  int err;
  do {
    // wait until we have found a periodic advertiser
    per_adv_found = false;
    err = k_sem_take(&sem_per_adv, K_FOREVER);
    if (err) {
      LOG_ERR("Failed to find a periodic advertiser, due to timeout (err %d)",
              err);
      return;
    }
    LOG_INF("Found valid TTLight periodic advertising.");

    bt_addr_le_copy(&sync_create_param.addr, &per_addr);
    sync_create_param.options = 0;
    sync_create_param.sid = per_sid;
    sync_create_param.skip = 0;
    sync_create_param.timeout = 0xaa;
    err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
    if (err) {
      LOG_ERR("Failed to create periodic sync (err %d)", err);
      return;
    }
    LOG_INF("Creating Periodic TTLight Advertising Sync successfully");

    LOG_INF("Waiting for periodic sync with TTLight advertiser");
    err = k_sem_take(&sem_per_sync, TIMEOUT_SYNC_CREATE);
    if (err) {
      LOG_ERR("failed (err %d)\n", err);

      LOG_ERR("Deleting Periodic Advertising Sync...");
      err = bt_le_per_adv_sync_delete(sync);
      if (err) {
        LOG_ERR("Failed to delete periodic sync (err %d)\n", err);
        return;
      }
    }
    LOG_INF("Periodic TTLight sync established.\n");

    LOG_INF("Monitor if periodic TTLight sync is not lost");
    err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
    if (err) {
      LOG_ERR("Failed to monitor TTLight sync (err %d)\n", err);
      return;
    }
    printk("Periodic TTLight sync lost, start searching from new");

  } while (true);
}

int ttl_ble_start() {
  int err;

  /* Start scanning for coded BLE advertiser */
  LOG_INF("Requested to start scanning for BLE periodic advertiser");
  err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
  if (err) {
    LOG_ERR("Failed to start BLE scanning (err %d)\n", err);
    return TTL_ERR;
  }

  /* Start a thread to reconnect in case of disconnect */
  k_thread_create(&ttl_ble_thread_data, ttl_ble_thread_stack,
                  TTL_BLE_STACK_SIZE, (k_thread_entry_t)ttl_ble_run, NULL, NULL,
                  NULL, -5, 0, K_NO_WAIT);
  k_thread_name_set(&ttl_ble_thread_data, "ttl_ble_worker");

  return TTL_OK;
}

int ttl_ble_stop() {
  int err;
  LOG_INF("Requested to stop BLE scanning for periodic advertiser");

  // kill the thread
  k_thread_abort(&ttl_ble_thread_data);

  // disable BLE scanning
  err = bt_le_scan_stop();
  if (err) {
    LOG_ERR("Failed to disable TTLight BLE periodic advertiser scan");
    return TTL_ERR;
  }
  LOG_INF("Stopped BLE scanning for periodic advertiser");

  // terminate sync channel if not null
  if (NULL != sync && true == ttl_ble_connected) {
    err = bt_le_per_adv_sync_delete(sync);
    if (err) {
      LOG_ERR("Failed to delete sync (err %d)", err);
      return TTL_ERR;
    }
  }

  return TTL_OK;
}

void ttl_ble_register_cb(ttl_upd_state_cb_t cb) {
  if (cb) {
    ttl_upd_state_cb = cb;
  }
}

bool ttl_ble_is_connected() { return ttl_ble_connected; }
int64_t ttl_ble_latest_packet_date() { return ttl_iso_last_data; }
