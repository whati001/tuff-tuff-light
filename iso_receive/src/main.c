
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#define DONGLE 0

#if DONGLE == 1
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#endif

#include "ttl.h"
#include "ble.h"
#include "led.h"

LOG_MODULE_REGISTER(ttl_main, LOG_LEVEL_INF);

void enable_usb_logging()
{
#if DONGLE == 1
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL))
	{
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr)
	{
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}
#endif
}

const struct device *const accelerometer = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

static int ttl_acc_enable()
{
	if (!device_is_ready(accelerometer)) {
		LOG_ERR("%s: device not ready.\n", accelerometer->name);
		return TTL_ERR;
	}

	struct sensor_value accel[3];
	while(1)
	{
		k_msleep(1000);
		int ret = sensor_sample_fetch(accelerometer);
		if (ret < 0)
		{
			LOG_ERR("Failed to fetch accel sensor data\n");
			continue;
		}

		if (!ret)
		{
			LOG_WRN("No data to sample\n");
			continue;
		}

		ret = sensor_channel_get(accelerometer, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (ret < 0)
		{
			LOG_ERR("Failed to read accel sensor data\n");
			continue;
		}
		LOG_INF("AccelData[x: %d, y:%d, z:%d]", (int)sensor_value_to_double(&accel[0]), (int)sensor_value_to_double(&accel[1]), (int)sensor_value_to_double(&accel[2]));
	}
}

int main(void)
{
	int err = 0;
	LOG_INF("Starting TTLight Controller\n");
	enable_usb_logging();

	// err = ttl_ble_init();
	if (TTL_OK != err)
	{
		LOG_ERR("Failed to initialize the TTLight BLE stack\n");
	}

	err = ttl_led_init();
	if (TTL_OK != err)
	{
		LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
	}
	ttl_ble_register_cb(ttl_led_upd_status);

	err = ttl_acc_enable();
	if (TTL_OK != err)
	{
		LOG_ERR("Failed to initialize the TTLight Acceleromater stack\n");
	}

	return 0;
}