#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <zephyr.h>
#include <errno.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <settings/settings.h>
#include <logging/log.h>

/** @brief UUID of the Simple State Service. **/
#define BT_UUID_SSTATE_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

/** @brief UUID of the Message Characteristic. **/
#define BT_UUID_SSTATE_CHANGE_CHRC_VAL \
    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

#define BT_UUID_SSTATE_SERVICE BT_UUID_DECLARE_128(BT_UUID_SSTATE_SERVICE_VAL)
#define BT_UUID_SSTATE_CHANGE_CHRC BT_UUID_DECLARE_128(BT_UUID_SSTATE_CHANGE_CHRC_VAL)

// struct bt_gatt_write_params ttl_left_light_change_params;

int ttl_central_connect();

int ttl_right_sent_state(uint8_t *state);

int ttl_left_sent_state(uint8_t *state);

#endif