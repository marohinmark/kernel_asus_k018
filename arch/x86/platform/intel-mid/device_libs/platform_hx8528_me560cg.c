/*
 * platform_hx8528.c: hx8528 platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/hx8528_me560cg.h>
#include "platform_hx8528_me560cg.h"

void *hx8528_platform_data(void *info)
{
	static struct himax_i2c_platform_data himax_pdata;

	himax_pdata.abs_x_max			= 1080; 
	himax_pdata.abs_y_max			= 1920;
	himax_pdata.rst_gpio			= get_gpio_by_name("ts_rst");
	himax_pdata.intr_gpio			= get_gpio_by_name("ts_int");
	himax_pdata.tp_pwr_gpio                 = get_gpio_by_name("TP_PWR_EN");
	return &himax_pdata;
}
