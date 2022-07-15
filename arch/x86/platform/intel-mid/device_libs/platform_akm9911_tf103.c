/*
 * platform_akm9911.c: compass platform data initilization file
 *
 * (C) Copyright 2013
 * Author : ASUS
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_akm9911_tf103.h"
#include <linux/akm09911_tf103.h>
//brook>>
//#define AKM09911_GPIO_RSTN    140
//brook<<


void *akm9911_platform_data_tf103(void *info)
{

	static struct akm09911_platform_data akm9911_pdata;

//	akm9911_pdata.gpio_DRDY	= 0;
//	akm9911_pdata.layout	= 4;
//	akm9911_pdata.gpio_RSTN	= AKM09911_GPIO_RSTN;

	return &akm9911_pdata;

}
