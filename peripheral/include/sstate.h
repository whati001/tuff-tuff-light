
#include <zephyr.h>
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

/** @brief UUID of the SimpleState Service. **/
#define BT_UUID_TLIGHT_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

/** @brief UUID of the Message Characteristic. **/
#define BT_UUID_TLIGHT_CHANGE_CHRC_VAL \
	BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc48)

#define BT_UUID_TLIGHT_SERVICE BT_UUID_DECLARE_128(BT_UUID_TLIGHT_SERVICE_VAL)
#define BT_UUID_TLIGHT_CHANGE_CHRC BT_UUID_DECLARE_128(BT_UUID_TLIGHT_CHANGE_CHRC_VAL)

/**
 * @brief struct for internal ble service actions
 * because the service only needs to react on received data a single callback function is needed
 * this function will be passed from the main/app code to react accordingly to the received data
 */
struct bt_remote_service_cb
{
	void (*data_received)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};

/*
 * Simple stage service initiation function
 */
int sstate_init(struct bt_conn_cb *bt_cb, struct bt_remote_service_cb *remote_cb);
