/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ttl.h"

LOG_MODULE_REGISTER(ttl_ble, LOG_LEVEL_INF);

static struct {
  bool initiated;
  bool running;
} ttl_ble = {0, 0};

static ttl_state_t ttl_state;

static struct bt_le_ext_adv *adv;
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, &ttl_state, sizeof(ttl_state_t)),
};

int ttl_ble_init() {
  int err;
  LOG_INF("TTLight starts to initiate the BLE Stack");

  // #TODO: stop advertising is currently running

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return TTL_ERR;
  }
  /* Create a coded non-connectable non-scannable advertising set */
  err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CODED_NCONN_NAME, NULL, &adv);
  if (err) {
    LOG_ERR("Failed to create advertising set (err %d)", err);
    return TTL_ERR;
  }

  /* Set periodic advertising parameters */
  err = bt_le_per_adv_set_param(
      adv, BT_LE_PER_ADV_PARAM(BT_GAP_PER_ADV_FAST_INT_MIN_1,
                               BT_GAP_PER_ADV_FAST_INT_MAX_1,
                               BT_LE_PER_ADV_OPT_NONE));
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

  /* Set Periodic Advertising Data */
  err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
  if (err) {
    LOG_INF("Failed (err %d)", err);
    return 0;
  }

  LOG_INF("Extended periodic advertisement channel created.");

  /* Set Periodic Advertising Data */
  err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
  if (err) {
    LOG_INF("Failed (err %d)", err);
    return 0;
  }

  ttl_ble.initiated = true;

  return TTL_OK;
}

int ttl_ble_start() {
  int err;
  LOG_INF("TTLight requested to advertise TTLState via BLE Stack");

  if (false == ttl_ble.initiated) {
    LOG_ERR("TTLight BLE stack not initiated yet");
    return TTL_ERR;
  }

  /* Start extended advertising */
  err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
  if (err) {
    LOG_ERR("Failed to start extended advertising (err %d)", err);
    return TTL_ERR;
  }

  LOG_INF("TTLight started to advertise TTLState via BLE");
  ttl_ble.running = true;

  return TTL_OK;
}

int tll_ble_stop() {
  int err = 0;
  LOG_INF("TTLight requested to stop advertising TTLState via BLE Stack");

  if (true == ttl_ble.initiated && true == ttl_ble.running) {
    /* Stop extended advertising */
    err = bt_le_ext_adv_stop(adv);
    if (err) {
      LOG_INF("Failed to stop extended advertising "
              "(err %d)",
              err);
      return TTL_ERR;
    }
  }

  LOG_INF("TTLight stopped to advertise TTLState via BLE");
  ttl_ble.running = false;
  ttl_ble.initiated = false;

  return TTL_OK;
}

int ttl_ble_upd_status(ttl_state_t state) {
  int err;

  if (false == ttl_ble.initiated) {
    LOG_WRN("TTLight BLE stack not initiated yet");
    return TTL_ERR;
  }

  if (state.entire != ttl_state.entire) {
    LOG_INF("Updated TTLight state to:");
    PRINT_TTL_STATE(state);

    ttl_state = state;
    err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
    if (err) {
      LOG_INF("Failed (err %d)", err);
      return 0;
    }
  }

  return TTL_OK;
}
