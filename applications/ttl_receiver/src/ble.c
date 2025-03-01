
/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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
static int64_t ttl_iso_last_datetime;
static bool ttl_iso_connected;
#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN 30

#define BT_LE_SCAN_CUSTOM                                                      \
  BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, BT_LE_SCAN_OPT_NONE,                \
                   BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW)

#define PA_RETRY_COUNT 6

static bool per_adv_found;
static bool per_adv_lost;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint32_t per_interval_us;

static uint32_t iso_recv_count;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, 1);
static K_SEM_DEFINE(sem_big_sync_lost, 0, 1);

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
          "Interval: 0x%04x (%u us), SID: %u",
          le_addr, info->adv_type, info->tx_power, info->rssi, name,
          (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
          (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
          phy2str(info->primary_phy), phy2str(info->secondary_phy),
          info->interval, BT_CONN_INTERVAL_TO_US(info->interval), info->sid);

  if (!per_adv_found && info->interval) {
    per_adv_found = true;

    per_sid = info->sid;
    per_interval_us = BT_CONN_INTERVAL_TO_US(info->interval);
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

  LOG_INF("PER_ADV_SYNC[%u]: [DEVICE]: %s synced, "
          "Interval 0x%04x (%u ms), PHY %s",
          bt_le_per_adv_sync_get_index(sync), le_addr, info->interval,
          info->interval * 5 / 4, phy2str(info->phy));

  k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
                    const struct bt_le_per_adv_sync_term_info *info) {
  char le_addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

  LOG_INF("PER_ADV_SYNC[%u]: [DEVICE]: %s sync terminated",
          bt_le_per_adv_sync_get_index(sync), le_addr);

  per_adv_lost = true;
  k_sem_give(&sem_per_sync_lost);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
                    const struct bt_le_per_adv_sync_recv_info *info,
                    struct net_buf_simple *buf) {
  char le_addr[BT_ADDR_LE_STR_LEN];
  char data_str[129];

  bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
  bin2hex(buf->data, buf->len, data_str, sizeof(data_str));

  LOG_INF("PER_ADV_SYNC[%u]: [DEVICE]: %s, tx_power %i, "
          "RSSI %i, CTE %u, data length %u, data: %s",
          bt_le_per_adv_sync_get_index(sync), le_addr, info->tx_power,
          info->rssi, info->cte_type, buf->len, data_str);
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync,
                       const struct bt_iso_biginfo *biginfo) {
  char le_addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(biginfo->addr, le_addr, sizeof(le_addr));

  LOG_INF("BIG INFO[%u]: [DEVICE]: %s, sid 0x%02x, "
          "num_bis %u, nse %u, interval 0x%04x (%u ms), "
          "bn %u, pto %u, irc %u, max_pdu %u, "
          "sdu_interval %u us, max_sdu %u, phy %s, "
          "%s framing, %sencrypted",
          bt_le_per_adv_sync_get_index(sync), le_addr, biginfo->sid,
          biginfo->num_bis, biginfo->sub_evt_count, biginfo->iso_interval,
          (biginfo->iso_interval * 5 / 4), biginfo->burst_number,
          biginfo->offset, biginfo->rep_count, biginfo->max_pdu,
          biginfo->sdu_interval, biginfo->max_sdu, phy2str(biginfo->phy),
          biginfo->framing ? "with" : "without",
          biginfo->encryption ? "" : "not ");

  k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .recv = recv_cb,
    .biginfo = biginfo_cb,
};

/**
 * @brief Callback function once ISO data has been received via BLE
 */
static void iso_recv(struct bt_iso_chan *chan,
                     const struct bt_iso_recv_info *info, struct net_buf *buf) {
  ttl_state_t state; /* only valid if the data is a counter */

  if (buf->len == sizeof(state)) {
    state.entire = sys_get_le32(buf->data);

    if ((iso_recv_count % CONFIG_TTL_STATE_PRINT_INTERVAL) == 0) {
      LOG_INF("Incoming data channel %p flags 0x%x seq_num %u ts %u len %u "
              "data: %x",
              chan, info->flags, info->seq_num, info->ts, buf->len,
              state.entire);
      PRINT_TTL_STATE(state);
    }

    // execute update state callback if set
    if (ttl_upd_state_cb) {
      (*ttl_upd_state_cb)(state);
    }
    // update last received datetime
    ttl_iso_last_datetime = k_uptime_get();
  }
  iso_recv_count++;
}

static void iso_connected(struct bt_iso_chan *chan) {
  LOG_INF("ISO Channel %p connected", chan);
  ttl_iso_connected = true;
  k_sem_give(&sem_big_sync);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
  LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
  if (reason != BT_HCI_ERR_OP_CANCELLED_BY_HOST) {
    ttl_iso_connected = false;
    k_sem_give(&sem_big_sync_lost);
  }
}

static struct bt_iso_chan_ops iso_ops = {
    .recv = iso_recv,
    .connected = iso_connected,
    .disconnected = iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx_qos[1];

static struct bt_iso_chan_qos bis_iso_qos[] = {
    {
        .rx = &iso_rx_qos[0],

    },
};

static struct bt_iso_chan bis_iso_chan[] = {{
    .ops = &iso_ops,
    .qos = &bis_iso_qos[0],
}};

static struct bt_iso_chan *bis[] = {
    &bis_iso_chan[0],
};

static struct bt_iso_big_sync_param big_sync_param = {
    .bis_channels = bis,
    .num_bis = 1,
    .bis_bitfield = (BIT_MASK(1) << 1),
    .mse = BT_ISO_SYNC_MSE_ANY, /* any number of subevents */
    .sync_timeout = 100,        /* in 10 ms units */
};

static void reset_semaphores(void) {
  k_sem_reset(&sem_per_adv);
  k_sem_reset(&sem_per_sync);
  k_sem_reset(&sem_per_sync_lost);
  k_sem_reset(&sem_per_big_info);
  k_sem_reset(&sem_big_sync);
  k_sem_reset(&sem_big_sync_lost);
}
/**
 * @brief Main TTL BLE thread loop
 */
void ttl_ble_thread_main() {
  struct bt_le_per_adv_sync_param sync_create_param;
  struct bt_le_per_adv_sync *sync;
  struct bt_iso_big *big;
  uint32_t sem_timeout_us;
  int err;

  iso_recv_count = 0;

  LOG_INF("Starting Synchronized Receiver Demo");

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(NULL);
  if (err) {
    LOG_INF("Bluetooth init failed (err %d)", err);
    return;
  }

  LOG_INF("Scan callbacks register...");
  bt_le_scan_cb_register(&scan_callbacks);
  LOG_INF("success.");

  LOG_INF("Periodic Advertising callbacks register...");
  bt_le_per_adv_sync_cb_register(&sync_callbacks);
  LOG_INF("Success.");

  do {
    reset_semaphores();
    per_adv_lost = false;

    LOG_INF("Start scanning...");
    err = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("success.");

    LOG_INF("Waiting for periodic advertising...");
    per_adv_found = false;
    err = k_sem_take(&sem_per_adv, K_FOREVER);
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("Found periodic advertising.");

    LOG_INF("Stop scanning...");
    err = bt_le_scan_stop();
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("success.");

    LOG_INF("Creating Periodic Advertising Sync...");
    bt_addr_le_copy(&sync_create_param.addr, &per_addr);
    sync_create_param.options = 0;
    sync_create_param.sid = per_sid;
    sync_create_param.skip = 0;
    /* Multiple PA interval with retry count and convert to unit of 10 ms */
    sync_create_param.timeout =
        (per_interval_us * PA_RETRY_COUNT) / (10 * USEC_PER_MSEC);
    sem_timeout_us = per_interval_us * PA_RETRY_COUNT;
    err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("success.");

    LOG_INF("Waiting for periodic sync...");
    err = k_sem_take(&sem_per_sync, K_USEC(sem_timeout_us));
    if (err) {
      LOG_INF("failed (err %d)", err);

      LOG_INF("Deleting Periodic Advertising Sync...");
      err = bt_le_per_adv_sync_delete(sync);
      if (err) {
        LOG_INF("failed (err %d)", err);
        return;
      }
      continue;
    }
    LOG_INF("Periodic sync established.");

    LOG_INF("Waiting for BIG info...");
    err = k_sem_take(&sem_per_big_info, K_USEC(sem_timeout_us));
    if (err) {
      LOG_INF("failed (err %d)", err);

      if (per_adv_lost) {
        continue;
      }

      LOG_INF("Deleting Periodic Advertising Sync...");
      err = bt_le_per_adv_sync_delete(sync);
      if (err) {
        LOG_INF("failed (err %d)", err);
        return;
      }
      continue;
    }
    LOG_INF("Periodic sync established.");

  big_sync_create:
    LOG_INF("Create BIG Sync...");
    err = bt_iso_big_sync(sync, &big_sync_param, &big);
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("success.");

    LOG_INF("Waiting for BIG sync chan ...");
    err = k_sem_take(&sem_big_sync, TIMEOUT_SYNC_CREATE);
    if (err) {
      break;
    }
    LOG_INF("BIG sync chan successful.");
    if (err) {
      LOG_INF("failed (err %d)", err);

      LOG_INF("BIG Sync Terminate...");
      err = bt_iso_big_terminate(big);
      if (err) {
        LOG_INF("failed (err %d)", err);
        return;
      }
      LOG_INF("done.");

      goto per_sync_lost_check;
    }
    LOG_INF("BIG sync established.");

    LOG_INF("Waiting for BIG sync lost chan...");
    err = k_sem_take(&sem_big_sync_lost, K_FOREVER);
    if (err) {
      LOG_INF("failed (err %d)", err);
      return;
    }
    LOG_INF("BIG sync lost.");

  per_sync_lost_check:
    LOG_INF("Check for periodic sync lost...");
    err = k_sem_take(&sem_per_sync_lost, K_NO_WAIT);
    if (err) {
      /* Periodic Sync active, go back to creating BIG Sync */
      goto big_sync_create;
    }
    LOG_INF("Periodic sync lost.");
  } while (true);

  return;
}

ttl_err_t ttl_ble_init(void) {
  ttl_upd_state_cb = NULL;
  ttl_iso_connected = false;
  return TTL_OK;
}

ttl_err_t ttl_ble_run(void) {
  ttl_iso_last_datetime = k_uptime_get();

  /* Start a thread to offload disk ops */
  k_thread_create(&ttl_ble_thread_data, ttl_ble_thread_stack,
                  TTL_BLE_STACK_SIZE, (k_thread_entry_t)ttl_ble_thread_main,
                  NULL, NULL, NULL, -5, 0, K_NO_WAIT);
  k_thread_name_set(&ttl_ble_thread_data, "ttl_ble_worker");

  return TTL_OK;
}

ttl_err_t ttl_ble_terminate(void) {
  ttl_err_t ret = TTL_OK;

  // TODO(akarner): not sure if we can kill it like this
  ret = bt_disable();
  if (ret != TTL_OK) {
    return TTL_ERR;
  }

  return TTL_OK;
}
void ttl_ble_register_cb(ttl_upd_state_cb_t cb) {
  if (cb) {
    ttl_upd_state_cb = cb;
  }
}

bool ttl_ble_is_connected(void) { return ttl_iso_connected; }
int64_t ttl_ble_latest_packet_datetime(void) { return ttl_iso_last_datetime; }
