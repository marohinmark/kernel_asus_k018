/*
 * platform_hx8528.c: hx8528 platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/hx8528_fe170cg.h>
#include "platform_hx8528_fe170cg.h"
#include <linux/HWVersion.h>

extern int Read_HW_ID(void);
static int HW_ID;

void *hx8528_platform_data(void *info)
{
    static struct himax_i2c_platform_data himax_pdata;

    HW_ID = Read_HW_ID();
    if(HW_ID == PROJ_FE170CG_HWID_SR)
    {
        himax_pdata.abs_x_max			= 800;
        himax_pdata.abs_y_max			= 1280;
    }
    else
    {
        himax_pdata.abs_x_max			= 600;
        himax_pdata.abs_y_max			= 1024;
    }

    himax_pdata.rst_gpio			= get_gpio_by_name("TS_RST_N");
    himax_pdata.intr_gpio			= get_gpio_by_name("TS_INT_N");
    return &himax_pdata;
}
