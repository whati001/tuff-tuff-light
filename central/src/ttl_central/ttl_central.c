#include "ttl_central.h"
#include <errno.h>
#include <zephyr.h>
#include <sys/byteorder.h>
#include <sys/printk.h>

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
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(ttl_central);
struct bt_gatt_write_params ttl_right_light_change_params;

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", log_strdup(addr));
}

static void pairing_confirm(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    bt_conn_auth_pairing_confirm(conn);

    LOG_INF("Pairing confirmed: %s", log_strdup(addr));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", log_strdup(addr),
            bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing failed conn: %s, reason %d", log_strdup(addr),
            reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel,
    .pairing_confirm = pairing_confirm,
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed};

static void discovery_complete(struct bt_gatt_dm *dm,
                               void *context)
{
    // struct bt_nus_client *nus = context;
    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);

    // bt_nus_handles_assign(dm, nus);
    // bt_nus_subscribe_receive(nus);

    // link gatt service characteristics with my central
    const struct bt_gatt_dm_attr *gatt_service_attr =
        bt_gatt_dm_service_get(dm);
    const struct bt_gatt_service_val *gatt_service =
        bt_gatt_dm_attr_service_val(gatt_service_attr);
    const struct bt_gatt_dm_attr *gatt_chrc;
    const struct bt_gatt_dm_attr *gatt_desc;

    // check one more time of the service uuid matches
    if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_SSTATE_SERVICE))
    {
        LOG_ERR("UUID tlight does not match in discover char");
        // return -ENOTSUP;
        return;
    }
    LOG_INF("Getting handles from tlight service.");

    /* NUS RX Characteristic */
    gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_SSTATE_CHANGE_CHRC);
    if (!gatt_chrc)
    {
        LOG_ERR("Missing tlight change characteristic.");
        // return -EINVAL;
        return;
    }
    /* NUS RX */
    gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_SSTATE_CHANGE_CHRC);
    if (!gatt_desc)
    {
        LOG_ERR("Missing tlight change value descriptor in characteristic.");
        // return -EINVAL;
        return;
    }
    ttl_right_light_change_handle = gatt_desc->handle;
    LOG_INF("Found tlight change characteristic.");

    bt_gatt_dm_data_release(dm);

    LOG_INF("Service discovery done");
}

static void discovery_service_not_found(struct bt_conn *conn,
                                        void *context)
{
    LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
                            int err,
                            void *context)
{
    LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found = discovery_error,
};
static void gatt_discover(struct bt_conn *conn)
{
    int err;

    if (conn != ttl_right_light)
    {
        return;
    }

    LOG_INF("Started to discover gatt server table");
    err = bt_gatt_dm_start(conn,
                           BT_UUID_SSTATE_SERVICE,
                           &discovery_cb,
                           NULL);
    //    &nus_client);
    if (err)
    {
        LOG_ERR("could not start the discovery procedure, error "
                "code: %d",
                err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!err)
    {
        LOG_INF("MTU exchange done");
    }
    else
    {
        LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err)
    {
        LOG_INF("Failed to connect to %s (%d)", log_strdup(addr),
                conn_err);

        if (ttl_right_light == conn)
        {
            bt_conn_unref(ttl_right_light);
            ttl_right_light = NULL;

            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if (err)
            {
                LOG_ERR("Scanning failed to start (err %d)",
                        err);
            }
        }

        return;
    }

    LOG_INF("Connected: %s", log_strdup(addr));

    // prepare the parameters for MTU exchange
    static struct bt_gatt_exchange_params exchange_params;
    // this is just the callback function after the MTU was exchanged
    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err)
    {
        LOG_WRN("MTU exchange failed (err %d)", err);
    }

    gatt_discover(conn);

    err = bt_scan_stop();
    if ((!err) && (err != -EALREADY))
    {
        LOG_ERR("Stop LE scan failed (err %d)", err);
    }
    LOG_INF("Stopped scanning");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr),
            reason);

    if (ttl_right_light != conn)
    {
        return;
    }

    bt_conn_unref(ttl_right_light);
    ttl_right_light = NULL;

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err)
    {
        LOG_ERR("Scanning failed to start (err %d)",
                err);
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d",
            log_strdup(addr), connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn)
{
    ttl_right_light = bt_conn_ref(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
                scan_connecting_error, scan_connecting);

static int scan_init(void)
{
    int err;
    // init scan init params -> set that the filter neeeds to match
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };
    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    // register tlight uuid service filter
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_SSTATE_SERVICE);
    if (err)
    {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err)
    {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }

    LOG_INF("Scan module initialized");
    return err;
}

static void on_sent(struct bt_conn *conn, uint8_t err,
                    struct bt_gatt_write_params *params)
{
    LOG_INF("Send out data to tuff tuff light");
}

int ttl_central_connect()
{
    int err = 0;
    ttl_right_light_change_handle = 0;
    ttl_left_light_change_handle = 0;

    LOG_INF("Start to connect to tuff tuff light device");

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err)
    {
        LOG_ERR("Failed to register authorization callbacks.");
        goto cleanup;
    }
    // enable bluetooth
    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        goto cleanup;
    }
    LOG_INF("Bluetooth initialized");
    // load some logic, do not know yet which one
    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    // register callbacks for bluetooth events
    bt_conn_cb_register(&conn_callbacks);

    // init scanning -> set filters
    err = scan_init();
    if (err)
    {
        LOG_ERR("Failed to init ble scan");
        goto cleanup;
    }

    // start scanning and connect if a suiteable is found
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err)
    {
        LOG_ERR("Scanning failed to start (err %d)", err);
        goto cleanup;
    }
    LOG_INF("Scanning successfully started");

    // initiate the change params
    ttl_right_light_change_params.func = on_sent;
    ttl_right_light_change_params.offset = 0;
    ttl_right_light_change_params.length = 1;

cleanup:
    return err;
}

int ttl_right_sent_state(uint8_t *state)
{
    int err = 0;

    ttl_right_light_change_params.handle = ttl_right_light_change_handle;
    ttl_right_light_change_params.data = state;

    if (NULL == ttl_right_light)
    {
        LOG_WRN("No connection to tuff tuff light right exists, skip update");
        err = -1;
        goto cleanup;
    }
    err = bt_gatt_write(ttl_right_light, &ttl_right_light_change_params);
    if (err)
    {
        LOG_ERR("Failed to send data via ble");
    }

cleanup:
    return err;
}

int ttl_left_sent_state(uint8_t *state)
{
    int err = 0;

cleanup:
    return err;
}