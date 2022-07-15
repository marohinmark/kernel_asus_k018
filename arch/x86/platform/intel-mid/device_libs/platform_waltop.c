/*
 * platform_waltop.c: waltop platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/platform_data/waltop_device.h>
#include "platform_waltop.h"

void *waltop_platform_data(void *info)
{
	static struct waltop_platform_data waltop_pdata;

    waltop_pdata.x_max = WALTOP_MAX_X;
    waltop_pdata.y_max = WALTOP_MAX_Y;
    waltop_pdata.p_max = WALTOP_MAX_P;
    waltop_pdata.p_minTipOn = WALTOP_MIN_P_TIPON;
    waltop_pdata.gpio_reset = get_gpio_by_name("EM_RST_N");
    waltop_pdata.gpio_int = get_gpio_by_name("EM_INT");
    waltop_pdata.gpio_pdct= get_gpio_by_name("EM_PDCT");

	return &waltop_pdata;
}