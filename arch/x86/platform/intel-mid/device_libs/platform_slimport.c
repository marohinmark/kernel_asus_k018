/*
 * platform_slimport.c: slimport platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/slimport_device.h>
#include "platform_slimport.h"
#include <asm/intel_scu_pmic.h>

#define VCCA100CNT_REG		0x0d0
#define VCCA100CNT_1V_ON	0xFF
#define VCCA100CNT_1V_OFF	0x2D

#define GPIO_MYDP_1V_POWER  158
static int slimport_power(int on){
    printk("%s: on=%d\n", __func__, on);
    intel_scu_ipc_iowrite8(VCCA100CNT_REG, VCCA100CNT_1V_ON);
}

void *slimport_platform_data(void *info)
{
	static struct anx7808_platform_data anx7808_pdata;

      anx7808_pdata.gpio_p_dwn = get_gpio_by_name("MYDP_EN_N");
      anx7808_pdata.gpio_reset = get_gpio_by_name("MYDP_RESETN_N");
      anx7808_pdata.gpio_int = get_gpio_by_name("MYDP_INTP");
      anx7808_pdata.gpio_cbl_det = get_gpio_by_name("MYDP_CBL_IN");
      anx7808_pdata.gpio_v10_ctrl = GPIO_MYDP_1V_POWER;
      anx7808_pdata.power_on = NULL;

	return &anx7808_pdata;
}
