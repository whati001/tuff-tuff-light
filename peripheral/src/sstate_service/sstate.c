
#include "sstate.h"

// register logger for this file named "sstate"
#define LOG_MODULE_NAME sstate
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// redefine the proj.conf device name
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// struct for all remote ble service actions ->  only data update is needed
static struct bt_remote_service_cb remote_callbacks;

// advertise data which will get send to central
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)};

// scan request data which will get send to central
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_TLIGHT_SERVICE_VAL),
};

/* Declarations */

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// define ble service, only data receive is needed
BT_GATT_SERVICE_DEFINE(remote_srv,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_TLIGHT_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_TLIGHT_CHANGE_CHRC,
                                              BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              NULL, on_write, NULL), );

/* Callbacks */

static ssize_t on_write(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        const void *buf,
                        uint16_t len,
                        uint16_t offset,
                        uint8_t flags)
{
    LOG_INF("Received data, handle %d, conn %p",
            attr->handle, (void *)conn);

    if (remote_callbacks.data_received)
    {
        remote_callbacks.data_received(conn, buf, len);
    }
    return len;
}

void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("bt_ready returned %d", err);
    }
    LOG_INF("Bluetooth should be ready");
}

int sstate_init(struct bt_conn_cb *bt_cb, struct bt_remote_service_cb *remote_cb)
{
    int err;
    LOG_INF("Initializing bluetooth...");

    if (bt_cb == NULL || remote_cb == NULL)
    {
        return -NRFX_ERROR_NULL;
    }

    // register bluetooth initialization callbacks like connect, disconnect, etc.
    bt_conn_cb_register(bt_cb);
    // register service specific callbacks
    remote_callbacks.data_received = remote_cb->data_received;

    // enable bt_enable and execute callback bt_ready afterwards
    err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("bt_enable returned %d", err);
        return err;
    }

    // on nrf5340dk, we need to wait otherwise we try to advertise before the bt stack is ready
    LOG_INF("Wait for 1 seconds");
    k_sleep(K_SECONDS(1));

    // start advertising, we pass the ad -> advertise data and sd -> scan req data
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Couldn't start advertising (err = %d)", err);
        return err;
    }

    return err;
}