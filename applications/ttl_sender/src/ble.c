#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ttl.h"
#include "zephyr/kernel.h"

LOG_MODULE_REGISTER(ttl_ble, LOG_LEVEL_DBG);

#define BUF_ALLOC_TIMEOUT_MS (10)
#define BIG_TERMINATE_TIMEOUT_US (60 * USEC_PER_SEC)
#define BIG_SDU_INTERVAL_US (10000)
#define BIS_ISO_CHAN_COUNT 1

NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, BIS_ISO_CHAN_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static K_SEM_DEFINE(sem_big_cmplt, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_big_term, 0, BIS_ISO_CHAN_COUNT);
static K_SEM_DEFINE(sem_iso_data, CONFIG_BT_ISO_TX_BUF_COUNT,
                    CONFIG_BT_ISO_TX_BUF_COUNT);

static uint16_t seq_num;
static ttl_state_t ttl_state;
static K_SEM_DEFINE(sem_ttl_state, 1, 1);
static uint8_t iso_data[sizeof(ttl_state)] = {0};

// ttl ble thread objects
#define TTL_BLE_STACK_SIZE (2048)
static K_KERNEL_STACK_DEFINE(ttl_ble_thread_stack, TTL_BLE_STACK_SIZE);
static struct k_thread ttl_ble_thread_data;

static void iso_connected(struct bt_iso_chan *chan) {
  LOG_INF("ISO Channel %p connected", chan);
  seq_num = 0U;
  k_sem_give(&sem_big_cmplt);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason) {
  LOG_INF("ISO Channel %p disconnected with reason 0x%02x", chan, reason);
  k_sem_give(&sem_big_term);
}

static void iso_sent(struct bt_iso_chan *chan) { k_sem_give(&sem_iso_data); }

static struct bt_iso_chan_ops iso_ops = {
    .connected = iso_connected,
    .disconnected = iso_disconnected,
    .sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
    .sdu = sizeof(uint32_t), /* bytes */
    .rtn = 2,
    .phy = BT_GAP_LE_PHY_1M,
};

static struct bt_iso_chan_qos bis_iso_qos = {
    .tx = &iso_tx_qos,
};

static struct bt_iso_chan bis_iso_chan = {
    .ops = &iso_ops,
    .qos = &bis_iso_qos,
};

static struct bt_iso_chan *bis[] = {&bis_iso_chan};

static struct bt_iso_big_create_param big_create_param = {
    .num_bis = BIS_ISO_CHAN_COUNT,
    .bis_channels = bis,
    .interval = BIG_SDU_INTERVAL_US, /* in microseconds */
    .latency = 10,                   /* in milliseconds */
    .packing = 0,                    /* 0 - sequential, 1 - interleaved */
    .framing = 0,                    /* 0 - unframed, 1 - framed */
};

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int ttl_ble_enable() {
  struct bt_le_ext_adv *adv;
  struct bt_iso_big *big;
  int err;

  LOG_INF("Starting to initiate BLE stack for TTLight");

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return TTL_ERR;
  }

  /* Create a non-connectable advertising set */
  err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, NULL, &adv);
  if (err) {
    LOG_ERR("Failed to create advertising set (err %d)", err);
    return TTL_ERR;
  }

  /* Set advertising data to have complete local name set */
  err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_ERR("Failed to set advertising data (err %d)\n", err);
    return TTL_ERR;
  }

  /* Set periodic advertising parameters */
  err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
  if (err) {
    LOG_ERR("Failed to set periodic advertising parameters (err %d)", err);
    return TTL_ERR;
  }

  /* Enable Periodic Advertising */
  err = bt_le_per_adv_start(adv);
  if (err) {
    LOG_ERR("Failed to enable periodic advertising (err %d)", err);
    return TTL_ERR;
  }

  /* Start extended advertising */
  err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
  if (err) {
    LOG_ERR("Failed to start extended advertising (err %d)", err);
    return TTL_ERR;
  }

  /* Create BIG */
  err = bt_iso_big_create(adv, &big_create_param, &big);
  if (err) {
    LOG_ERR("Failed to create BIG (err %d)", err);
    return TTL_ERR;
  }

  LOG_INF("Waiting for BIG complete chan...");
  err = k_sem_take(&sem_big_cmplt, K_FOREVER);
  if (err) {
    LOG_ERR("failed (err %d)", err);
    return TTL_ERR;
  }
  LOG_INF("BIG create complete chan.\n");

  return TTL_OK;
}

static int ttl_ble_broadcast() {
  while (true) {
    struct net_buf *buf;
    int ret;

    buf = net_buf_alloc(&bis_tx_pool, K_FOREVER);
    if (!buf) {
      LOG_INF("Data buffer allocate timeout on channel");
      return 0;
    }

    ret = k_sem_take(&sem_iso_data, K_FOREVER);
    if (ret) {
      LOG_INF("k_sem_take for ISO data sent failed\n");
      net_buf_unref(buf);
      return 0;
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
    // NOTE: we do not check the error code, because it anyway waits FOREVER
    k_sem_take(&sem_ttl_state, K_FOREVER);
    sys_put_le32(ttl_state.entire, iso_data);
    k_sem_give(&sem_ttl_state);
    net_buf_add_mem(buf, iso_data, sizeof(iso_data));
    // LOG_INF("Broadcasting data with len: %u", buf->len);

    ret = bt_iso_chan_send(&bis_iso_chan, buf, seq_num);
    if (ret < 0) {
      LOG_INF("Unable to broadcast data on channel with error code: %d", ret);
      net_buf_unref(buf);
      return 0;
    }

    if ((seq_num % CONFIG_TTL_STATE_PRINT_INTERVAL) == 0) {
      PRINT_TTL_STATE(ttl_state);
    }
    seq_num++;
  }

  return TTL_OK;
}

/**
 * @brief Main TTL BLE thread loop
 */
static void ttl_ble_thread_main() {
  int ret = 0;
  LOG_INF("TTLight starts to initiate the BLE Stack");
  ret = ttl_ble_enable();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to initiate the TTLight BLE stack");
    return;
  }
  LOG_INF("TTLight started the BLE Stack successfully");

  LOG_INF("TTLight starts to broadcast current state");
  ret = ttl_ble_broadcast();
  if (TTL_OK != ret) {
    LOG_ERR("Failed to broadcast the current TTLight BLE state");
    return;
  }
  LOG_INF("TTLight finished to broadcast current state");

  return;
}

ttl_err_t ttl_ble_init(void) {
  // nothing special to do, just to keep interface straight
  return TTL_OK;
}

ttl_err_t ttl_ble_run(void) {
  /* Start a thread to offload disk ops */
  k_thread_create(&ttl_ble_thread_data, ttl_ble_thread_stack,
                  TTL_BLE_STACK_SIZE, (k_thread_entry_t)ttl_ble_thread_main,
                  NULL, NULL, NULL, -5, 0, K_NO_WAIT);
  k_thread_name_set(&ttl_ble_thread_data, "ttl_ble_worker");

  return TTL_OK;
}

ttl_err_t ttl_ble_upd_status(ttl_state_t state) {
  if (state.entire != ttl_state.entire) {
    LOG_INF("Updated TTLight state to:");
    PRINT_TTL_STATE(state);

    k_sem_take(&sem_ttl_state, K_FOREVER);
    ttl_state = state;
    k_sem_give(&sem_ttl_state);
  }

  return TTL_OK;
}