
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DONGLE 0

#if DONGLE == 1
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#endif

#include "ttl.h"
#include "ble.h"
#include "gpio.h"

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

int main(void)
{
	int err = 0;
	LOG_INF("Starting TTLight Controller\n");
	enable_usb_logging();

	err = ttl_ble_init();
	if (TTL_OK != err)
	{
		LOG_ERR("Failed to initialize the TTLight BLE stack\n");
	}

	err = ttl_gpio_init();
	if (TTL_OK != err)
	{
		LOG_ERR("Failed to initialize the TTLight GPIO stack\n");
	}
	ttl_ble_register_cb(ttl_gpio_upd_status);

	return 0;
}