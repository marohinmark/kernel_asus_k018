/*
 * platform_btlpm: btlpm platform data initialization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/init.h>
#include <linux/pm_runtime.h>
#include <asm/bcm_bt_lpm.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>

#define UART_PORT_NO 0 /* Bluetooth is using UART port number 0 */

#define LPM_ON

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = -EINVAL,
	.gpio_host_wake = -EINVAL,
	.int_host_wake = -EINVAL,
	.gpio_enable = -EINVAL,
#ifdef FOR_2076
	.gpio_reset = -EINVAL,
#endif
	.port = UART_PORT_NO,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

static int __init bluetooth_init(void)
{

	int error_reg;

	/* Get the GPIO numbers from the SFI table */


#ifdef FOR_2076
	bcm_bt_lpm_pdata.gpio_reset = get_gpio_by_name("GPS_RESET_N");
	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_reset)) {
		pr_err("%s: gpio %s not found\n", __func__, "GPS_RESET_N");
		return -ENODEV;
	}

	bcm_bt_lpm_pdata.gpio_enable = get_gpio_by_name("GPS_ENABLE");
	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_enable)) {
		pr_err("%s: gpio %s not found\n", __func__, "GPS_ENABLE");
		return -ENODEV;
	}
#else
	bcm_bt_lpm_pdata.gpio_enable = get_gpio_by_name("BT_EN");
	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_enable)) {
		pr_err("%s: gpio %s not found\n", __func__, "BT_EN");
		return -ENODEV;
	}
#endif

#ifdef LPM_ON
	bcm_bt_lpm_pdata.gpio_host_wake = get_gpio_by_name("BT_HOSTWAKE_SOC");
	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_host_wake)) {
		pr_err("%s: gpio %s not found\n", __func__, "BT_HOSTWAKE_SOC");
		return -ENODEV;
	}

	bcm_bt_lpm_pdata.int_host_wake =
				gpio_to_irq(bcm_bt_lpm_pdata.gpio_host_wake);

	bcm_bt_lpm_pdata.gpio_wake = get_gpio_by_name("BT_BTWAKE_SOC");
	if (!gpio_is_valid(bcm_bt_lpm_pdata.gpio_wake)) {
		pr_err("%s: gpio %s not found\n", __func__, "BT_BTWAKE_SOC");
		return -ENODEV;
	}

	pr_debug("%s: gpio_wake %d, gpio_host_wake %d\n", __func__,
		bcm_bt_lpm_pdata.gpio_wake, bcm_bt_lpm_pdata.gpio_host_wake);
#endif

	error_reg = platform_device_register(&bcm_bt_lpm_device);
	if (error_reg < 0) {
		pr_err("%s: platform_device_register for %s failed\n",
					__func__, bcm_bt_lpm_device.name);
		return -ENODEV;
	}
	return 0;
}

device_initcall(bluetooth_init);
