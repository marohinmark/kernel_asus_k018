/* Copyright (c) 2013, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static char serialno[32] = {0,};
int __init asustek_androidboot_serialno(char *s)
{
	int n;
	if (*s == '=')
		s++;
	n = snprintf(serialno, sizeof(serialno), "%s", s);
	serialno[n] = '\0';

	return 1;
}
__setup("androidboot.serialno", asustek_androidboot_serialno);

struct asustek_pcbid_platform_data {
        const char *UUID;
};

struct asustek_pcbid_platform_data asustek_pcbid_pdata = {
	.UUID = serialno,
};

static struct resource resources_asustek_pcbid[] = {

};

static struct platform_device asustek_pcbid_device = {
	.name		= "asustek_pcbid",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_asustek_pcbid),
	// CT add : Here is an empty resource. 
	.resource = resources_asustek_pcbid,
	.dev = {
		.platform_data = &asustek_pcbid_pdata,
	}
};

void __init asustek_add_pcbid_devices(void)
{
	printk("asustek_add_pcbid_devices+\n");
	platform_device_register(&asustek_pcbid_device);
	printk("asustek_add_pcbid_devices-\n");
}

core_initcall(asustek_add_pcbid_devices);


