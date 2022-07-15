/*
 * Copyright 2013 ASUS Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Liwei Liu <liwei_liu@asus.com>
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/sfi.h>

#include "dsi_mod_auo_b101ean01.h"
#include "tx358774.h"
struct i2c_client *bridge_i2c_client;
struct i2c_adapter *bridge_i2c_adapter;

#define B101EAN01_DEBUG 1

#define LVDS_STBY 79        //GP_AON_079  ++++
#define RESET_INNO 90       // GP_AON_090 ++++
#define VDD_3V3_LCD 108     //96+12 GP_CORE_012
#define LCD_BL_EN 0x7E      //PMIC:BACKLIGHT_EN
#define VDD_1V8_LVDS 36     //GP_AON_036
#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49  //LED_BL_PWM GP_AON_049
#define PWM_BASE_UNIT 0x444 //5,000Hz

#define PMIC_GPIO_BACKLIGHT_EN  0x7E
#define PMIC_GPIO_PANEL_EN      0x7F


static struct mdfld_dsi_config *b101ean01_dsi_config;

struct b101ean01_vid_data{
	u8 project_id;
	unsigned int gpio_lcd_en;
	unsigned int gpio_bl_en;
	unsigned int gpio_stb1_en;
	unsigned int gpio_stb2_en;
	unsigned int gpio_lcd_rst;
};
static struct b101ean01_vid_data gpio_settings_data;

union sst_pwmctrl_reg {
        struct {
                u32 pwmtd:8;
                u32 pwmbu:22;
                u32 pwmswupdate:1;
                u32 pwmenable:1;
        } part;
        u32 full;
};


static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}

static int b101ean01_vid_gpio_reset(void) {

        printk("[b101ean01] %s enter\n", __func__);
        if (gpio_direction_output(VDD_3V3_LCD, 0))
            gpio_set_value_cansleep(VDD_3V3_LCD, 0);
 
        //if(gpio_direction_output(VDD_1V8_LVDS, 0))
        //    gpio_set_value_cansleep(VDD_1V8_LVDS, 0);

        if(gpio_direction_output(LVDS_STBY, 0))
            gpio_set_value_cansleep(LVDS_STBY, 0);

        if (gpio_direction_output(RESET_INNO, 0))
            gpio_set_value_cansleep(RESET_INNO, 0);

        mdelay(100);
        printk("[b101ean01] %s return\n", __func__);
        return 0;
}

static int b101ean01_vid_gpio_control(int on){
	printk("[b101ean01] %s enter\n", __func__);
	int reset_inno;

	reset_inno = RESET_INNO;

	if(on){
           printk("[b101ean01] %s on\n", __func__);

           if(gpio_get_value(VDD_1V8_LVDS)==0)  {
               if(gpio_direction_output(VDD_1V8_LVDS, 1))
                   gpio_set_value_cansleep(VDD_1V8_LVDS, 1);
           }

           usleep_range(100,150);
           if(gpio_direction_output(LVDS_STBY,1))
               gpio_set_value_cansleep(LVDS_STBY, 1);

           usleep_range(10,15);//T4 delay 10us

           if (gpio_direction_output(RESET_INNO, 1))
               gpio_set_value_cansleep(RESET_INNO, 1);

           if (gpio_direction_output(VDD_3V3_LCD, 1))
                gpio_set_value_cansleep(VDD_3V3_LCD, 1);

	}else{
           printk("[b101ean01] %s off\n", __func__);
           if (gpio_direction_output(RESET_INNO, 0))
               gpio_set_value_cansleep(RESET_INNO, 0);//GP_CORE
           mdelay(10);//T4 delay

           if(gpio_direction_output(LVDS_STBY,0))
               gpio_set_value_cansleep(LVDS_STBY, 0);

           //if(gpio_direction_output(VDD_1V8_LVDS, 0))
           //    gpio_set_value_cansleep(VDD_1V8_LVDS, 0);

            if (gpio_direction_output(VDD_3V3_LCD, 0))
                gpio_set_value_cansleep(VDD_3V3_LCD, 0);//GP_CORE

	}

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	printk("[b101ean01] %s return\n", __func__);
	return 0;

}

static
int tc35876x_bridge_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	PSB_DEBUG_ENTRY("\n");
	printk("[b101ean01]8-2: %s enter\n", __func__);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

        bridge_i2c_adapter = client->adapter;

	printk("[b101ean01] %s return\n", __func__);
	return 0;
}

static int tc35876x_bridge_remove(struct i2c_client *client)
{
	PSB_DEBUG_ENTRY("\n");
	printk("[b101ean01] %s enter\n", __func__);
//	gpio_free(GPIO_MIPI_BRIDGE_RESET);
//	gpio_free(GPIO_MIPI_LCD_BL_EN);
	bridge_i2c_client = NULL;
	printk("[b101ean01] %s return\n", __func__);
	return 0;
}


static const struct i2c_device_id tc35876x_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc35876x_bridge_id);

static
struct i2c_driver tc35876x_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = tc35876x_bridge_id,
	.probe = tc35876x_bridge_probe,
	.remove = tc35876x_bridge_remove,
};



static int bridge_hack_create_device(void)
{
	int ret = 0;
	printk("[b101ean01]8-1: %s enter\n", __func__);
	struct i2c_adapter *adapter;

	ret = i2c_add_driver(&tc35876x_bridge_i2c_driver);
	if (ret) {
		printk("[b101ean01] add bridge I2C driver faild\n");
		return 0;
	}

	printk("[b101ean01] %s return\n", __func__);
	return 0;
}

int tc358774_regw(struct i2c_client *client, u16 reg, u32 value)
{
	//printk("[b101ean01] %s enter\n", __func__);
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};

	struct i2c_msg msgs[] = {
		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));

	if (r < 0) {
		printk("[b101ean01] i2c error write reg:0x%08x\n",reg);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
                printk("[b101ean01] i2c error  ARRAY_SIZE(msgs) write\n");
		return -EAGAIN;
	}

        //printk("i2c command write pass \n");

	//printk("[b101ean01] %s return\n", __func__);
	return 0;
}

int tc358774_regr(struct i2c_client *client, u16 reg, u32 *value)
{
	//printk("[b101ean01] %s enter\n", __func__);
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	u8 rx_data[4];

	struct i2c_msg msgs[] = {
		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},

		{
			.addr = BRIDGE_I2C_ADDR,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(bridge_i2c_adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
                printk("[b101ean01] i2c error read reg:0x%08x\n",reg);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
                printk("[b101ean01] i2c error ARRAY_SIZE(msgs)  read\n");
		return -EAGAIN;
	}

        printk("i2c command read pass \n");

	*value = rx_data[3] << 24 | rx_data[2] << 16 |
		rx_data[1] << 8 | rx_data[0];
	//printk("[b101ean01] %s return\n", __func__);
	return 0;

}

void tc358774_configure_lvds_bridge_read(void)

{
	struct i2c_client *i2c = bridge_i2c_client;
	u32 id = 0;
        u32 result = 0;
	tc358774_regr(i2c, IDREG, &id);
	printk("tc358774 ID 0x%08x\n", id);

	tc358774_regr(i2c, PPI_TX_RX_TA, &result); //0x00030005
	printk("tc358774 PPI_TX_RX_TA 0x%08x\n", result);
	tc358774_regr(i2c, PPI_LPTXTIMCNT, &result);//0x00000003
        printk("tc358774 PPI_LPTXTIMCNT 0x%08x\n", result);
	tc358774_regr(i2c, PPI_D0S_CLRSIPOCOUNT, &result);
        printk("tc358774 PPI_D0S_CLRSIPOCOUNT 0x%08x\n", result);
	tc358774_regr(i2c, PPI_D1S_CLRSIPOCOUNT, &result);
        printk("tc358774 PPI_D1S_CLRSIPOCOUNT 0x%08x\n", result);
	tc358774_regr(i2c, PPI_D2S_CLRSIPOCOUNT, &result);
        printk("tc358774 PPI_D2S_CLRSIPOCOUNT 0x%08x\n", result);
	tc358774_regr(i2c, PPI_D3S_CLRSIPOCOUNT, &result);
        printk("tc358774 PPI_D3S_CLRSIPOCOUNT 0x%08x\n", result);
	tc358774_regr(i2c, PPI_LANEENABLE, &result);
        printk("tc358774 PPI_LANEENABLE 0x%08x\n", result);
	tc358774_regr(i2c, DSI_LANEENABLE, &result);
        printk("tc358774 DSI_LANEENABLE 0x%08x\n", result);
	tc358774_regr(i2c, PPI_STARTPPI, &result);
        printk("tc358774 PPI_STARTPPI 0x%08x\n", result);
	tc358774_regr(i2c, DSI_STARTPPI, &result);
        printk("tc358774 DSI_STARTPPI 0x%08x\n", result);
	

	tc358774_regr(i2c, VPCTRL, &result);
        printk("tc358774 VPCTRL 0x%08x\n", result);
	tc358774_regr(i2c, HTIM1, &result);
        printk("tc358774 HTIM1 0x%08x\n", result);
	tc358774_regr(i2c, HTIM2, &result);
        printk("tc358774 HTIM2 0x%08x\n", result);
	tc358774_regr(i2c, VTIM1, &result);
        printk("tc358774 VTIM1, 0x%08x\n", result);
	tc358774_regr(i2c, VTIM2, &result);
        printk("tc358774 VTIM2 0x%08x\n", result);
	tc358774_regr(i2c, VFUEN, &result);
        printk("tc358774 VFUEN 0x%08x\n", result);
	tc358774_regr(i2c, LVPHY0, &result);
        printk("tc358774 LVPHY0 0x%08x\n", result);
	tc358774_regr(i2c, LVPHY0, &result);
        printk("tc358774 LVPHY0 0x%08x\n", result);
	tc358774_regr(i2c, SYSRST, &result);
        printk("tc358774 SYSRST 0x%08x\n", result);
	

	tc358774_regr(i2c, LVMX0003, &result);
        printk("tc358774 LVMX0003 0x%08x\n", result);
	tc358774_regr(i2c, LVMX0407, &result);
        printk("tc358774 LVMX0407 0x%08x\n", result);
	tc358774_regr(i2c, LVMX0811, &result);
        printk("tc358774 LVMX0811 0x%08x\n", result);
	tc358774_regr(i2c, LVMX1215, &result);
        printk("tc358774 LVMX1215 0x%08x\n", result);
	tc358774_regr(i2c, LVMX1619, &result);
        printk("tc358774 LVMX1619_TA 0x%08x\n", result);
	tc358774_regr(i2c, LVMX2023, &result);
        printk("tc358774 LVMX2023 0x%08x\n", result);
	tc358774_regr(i2c, LVMX2427, &result);
        printk("tc358774 LVMX2427 0x%08x\n", result);
	

	tc358774_regr(i2c, LVCFG, &result);
        printk("tc358774 LVCFG 0x%08x\n", result);

        tc358774_regr(i2c, 0x0214, &result);
        printk("tc358774 DSI_LANESTATUS0 0x%08x\n", result);

        tc358774_regr(i2c, 0x0500, &result);
        printk("tc358774 SYSSTAT 0x%08x\n", result);
}



void tc358774_configure_lvds_bridge(void)
{
	printk("[b101ean01] %s enter\n", __func__);

	struct i2c_client *i2c = bridge_i2c_client;
        int err = 0;
	u32 id = 0;

	err = tc358774_regr(i2c, IDREG, &id);
	if(err < 0)
	    printk("[b101ean01] %s tc358774_regr err : \n", __func__,err);
	
	printk("tc358774 ID 0x%08x\n", id);
        //Following 10 setting should be pefromed in LP mode

        tc358774_regw(i2c, PPI_TX_RX_TA, 0x00020003); //0x00030005
        tc358774_regw(i2c, PPI_LPTXTIMCNT, 0x00000002);//0x00000003
        tc358774_regw(i2c, PPI_D0S_CLRSIPOCOUNT, 0x00000008); // Ray modify 0x00000008
        tc358774_regw(i2c, PPI_D1S_CLRSIPOCOUNT, 0x00000008); // Ray modify 0x00000008
        tc358774_regw(i2c, PPI_D2S_CLRSIPOCOUNT, 0x00000008); // Ray modify 0x00000008
        tc358774_regw(i2c, PPI_D3S_CLRSIPOCOUNT, 0x00000008); // Ray modify 0x00000008
        tc358774_regw(i2c, PPI_LANEENABLE, 0x0000001F);
        tc358774_regw(i2c, DSI_LANEENABLE, 0x0000001F);
        tc358774_regw(i2c, PPI_STARTPPI, 0x00000001);
        tc358774_regw(i2c, DSI_STARTPPI, 0x00000001);

        //TC358764/65XBG Timing and mode setting

        tc358774_regw(i2c, VPCTRL, 0x02000120);
        tc358774_regw(i2c, HTIM1, 0x00300020);
        tc358774_regw(i2c, HTIM2, 0x00400500);
        tc358774_regw(i2c, VTIM1, 0x00050004);
        tc358774_regw(i2c, VTIM2, 0x00070320);
        tc358774_regw(i2c, VFUEN, 0x00000001);
        tc358774_regw(i2c, LVPHY0, 0x00448006);
        usleep_range(100,150);
        tc358774_regw(i2c, LVPHY0, 0x00048006);
        tc358774_regw(i2c, SYSRST, 0x00000004);
        //TC358764/65XBG LVDS Color mapping setting

        tc358774_regw(i2c, LVMX0003, 0x03020100); //0x03020100
        tc358774_regw(i2c, LVMX0407, 0x08050704); //0x08050704
        tc358774_regw(i2c, LVMX0811, 0x0F0E0A09); //0x0F0E0A09
        tc358774_regw(i2c, LVMX1215, 0x100D0C0B); //0x100D0C0B
        tc358774_regw(i2c, LVMX1619, 0x12111716); //0x12111716
        tc358774_regw(i2c, LVMX2023, 0x1B151413); //0x1B151413
        tc358774_regw(i2c, LVMX2427, 0x061A1918); //0x061A1918

        //TC358764/65XBG LVDS enable
        tc358774_regw(i2c, LVCFG, 0x00000031); //Ray modify  0x00000031  0x00000431

	printk("[b101ean01] %s return\n", __func__);
}

static int b101ean01_send_otp_cmds(struct mdfld_dsi_config *dsi_config)
{

        printk("[b101ean01]:10 13 %s enter\n", __func__);
        usleep_range(20000,25000);
        tc358774_configure_lvds_bridge();
        printk("[b101ean01] %s return\n", __func__);
	return 0;
}

static void
b101ean01_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	DRM_INFO("[DISPLAY]3 %s: Enter\n", __func__);
#if 1
        /* Reconfig lane configuration */
        dsi_config->lane_count = 4;
        dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
        dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

        hw_ctx->cck_div = 0;
        hw_ctx->pll_bypass_mode = 0;
        hw_ctx->mipi_control = 0x18;
        hw_ctx->intr_en = 0xffffffff;
        hw_ctx->hs_tx_timeout = 0xffffff;
        hw_ctx->lp_rx_timeout = 0xffffff;
        hw_ctx->turn_around_timeout = 0x22;
        hw_ctx->device_reset_timer = 0xffff;
        hw_ctx->high_low_switch_count = 0x1B;
        hw_ctx->init_count = 0x0;
        hw_ctx->eot_disable = 0x0;
        hw_ctx->lp_byteclk = 0x3;
        hw_ctx->clk_lane_switch_time_cnt = 0x1E000D;
        hw_ctx->dphy_param = 0x3A0F2D13; //0x3A0F2D13;

        /* Setup video mode format */
        hw_ctx->video_mode_format = 0xe;

        /* Set up func_prg, RGB888(0x200) */
        hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

        /* Setup mipi port configuration */
        hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
#endif

#if 0
        /* Reconfig lane configuration */
        dsi_config->lane_count = 4;
        dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
        //dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

        /* Reconfig lane configuration */
	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;
	hw_ctx->video_mode_format = 0x6;
	hw_ctx->dphy_param = 0x160d3610;

        /* Setup video mode format */
        hw_ctx->video_mode_format = 0xe;

        /* Set up func_prg, RGB888(0x200) */
        hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

        /* Setup mipi port configuration */
        hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
#endif

	b101ean01_dsi_config = dsi_config;

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
#if GAMMA_USER_SETTING
		mdfld_intel_crtc_set_gamma(dev, &gamma_settings);
#endif
	}

	DRM_INFO("[DISPLAY]3 %s: End\n", __func__);
}


static int b101ean01_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
                        DRM_INFO("%s: panel is detected!\n", __func__);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}
        pr_debug("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);

	return status;
}


static int b101ean01_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
        printk("[b101ean01]14: %s enter\n", __func__);
	    struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err, i;
	//struct b101ean01_vid_data *pdata = &gpio_settings_data;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

        /*Add or not*/
        //usleep_range(10000, 15000);
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}
        usleep_range(100000, 150000);
        pwm_enable();
        intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

        DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);

	return 0;
}

static int b101ean01_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
        printk("[b101ean01] %s enter\n", __func__);

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err, i;
	struct b101ean01_vid_data *pdata = &gpio_settings_data;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

        /* Turn off the backlight*/
        pwm_disable();
        intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0);
        usleep_range(1000, 1500);
	  
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	usleep_range(180000, 200000);

        b101ean01_vid_gpio_control(0);
        DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);
    
    return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX 0x63
#define BRIGHTNESS_LEVEL_MAX 100
#define BACKLIGHT_DUTY_FACTOR	0xff

#define BRI_SETTING_MIN 30
#define BRI_SETTING_MAX 255

static int bl_prev_level = 0;
static int b101ean01_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (level <= 0) {
		duty_val = 0;
	} else if (level > 0 && (level < BRI_SETTING_MIN)) {
		duty_val = BRI_SETTING_MIN;
	} else if ((level >= BRI_SETTING_MIN) && (level <= BRI_SETTING_MAX)) {
		duty_val = level;
	} else if (level > BRI_SETTING_MAX)
		duty_val = BRI_SETTING_MAX;

	pwm_configure(duty_val);

        /* TFCG-252 [TF103CG][Display] Remove Intel dpst daemon(Display Power Saving Technology) change backlight's log
        To Detect the dpst daemon change backlight or other change the backlight.
        If Intel dpst daemon change the backlight , by pass the log.
        */

        if( duty_val > (bl_prev_level+10) || duty_val < (bl_prev_level-10)  || level==0 ){
            DRM_INFO("[DISP] brightness level = %d , duty_val = %d\n", level, duty_val);
            bl_prev_level = duty_val;
        }
        return 0;
}

static int b101ean01_vid_reset(struct mdfld_dsi_config *dsi_config)
{
      printk("[b101ean01]9 12: %s enter\n", __func__);

      DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
      b101ean01_vid_gpio_control(1);

      if (gpio_direction_output(RESET_INNO, 0))
          gpio_set_value_cansleep(RESET_INNO, 0);

      usleep_range(500, 1000);

      if (gpio_direction_output(RESET_INNO, 1))
          gpio_set_value_cansleep(RESET_INNO, 1);

      usleep_range(500, 1000);

      pr_debug("[DISPLAY] %s: Enter\n", __func__);
      printk("[b101ean01] %s return\n", __func__);

	return 0;
}

static struct drm_display_mode *b101ean01_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;
        printk("[b101ean01]5: %s enter\n", __func__);

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	mode->hdisplay = 1280;
        mode->vdisplay = 800;
#if 1
        mode->hsync_start = mode->hdisplay + 32;  /* HFP */   //32 front porch                                    //64
        mode->hsync_end = mode->hsync_start + 32;  /* HSW */   //+20 pulse width Fixme: Event mode has no end pkt? //32
        mode->htotal = mode->hsync_end + 64;      /* HBP */   // back porch                                       //32
        mode->vsync_start = mode->vdisplay + 4;   /* VFP */   //                                                  //8-4
        mode->vsync_end = mode->vsync_start + 4;  /* VSW */   //+4 Fixme: Event mode has no end pkt?              //4
        mode->vtotal = mode->vsync_end + 8;       /* VBP */   //                                                  //8+4
#endif
#if 0
        mode->hsync_start = mode->hdisplay + 33 ;  /* HFP */   //32 front porch                                    //64
        mode->hsync_end = mode->hsync_start + 0;  /* HSW */   //+20 pulse width Fixme: Event mode has no end pkt? //32
        mode->htotal = mode->hsync_end + 64;      /* HBP */   // back porch                                       //32
        mode->vsync_start = mode->vdisplay + 8;   /* VFP */   //                                                  //8
        mode->vsync_end = mode->vsync_start + 0;  /* VSW */   //+4 Fixme: Event mode has no end pkt?              //4
        mode->vtotal = mode->vsync_end + 8;       /* VBP */   //                                                  //4
#endif

        mode->vrefresh = 60;
        mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;

        mode->type |= DRM_MODE_TYPE_PREFERRED;
        drm_mode_set_name(mode);
        drm_mode_set_crtcinfo(mode, 0);

        pr_debug("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);
        return mode;

}

static void b101ean01_vid_get_panel_info(int pipe, struct panel_info *pi)
{
        printk("[b101ean01]6 %s enter\n", __func__);
        pi->width_mm = 216;
        pi->height_mm = 136;
        printk("[b101ean01] %s return\n", __func__);
}

static int b101ean01_vid_gpio_init(void)
{

        printk("[b101ean01]2: %s enter\n", __func__);
        int ret = 0;

        gpio_request(RESET_INNO,"RESET_INNO");   //90 GP_AON_090
        gpio_request(LVDS_STBY,"LVDS_STBY");     //79 GP_AON_079
        gpio_request(VDD_3V3_LCD,"VDD_3V3_LCD"); //96+12 GP_CORE_012
        gpio_request(VDD_1V8_LVDS,"VDD_1V8_LVDS"); //GP_AON_036
        intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

        pr_debug("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);

       return 0;
}

static int b101ean01_vid_brightness_init(void)
{
	int ret = 0;
	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	return ret;
}

#ifdef B101EAN01_DEBUG

static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    printk("[b101ean01] %s enter\n", __func__);
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(b101ean01_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);

    DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);
    printk("[b101ean01] %s return\n", __func__);
    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}

static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(b101ean01_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

    DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);
    printk("[b101ean01] %s return\n", __func__);
    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
    return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

static ssize_t send_i2c_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    printk("[b101ean01] %s enter\n", __func__);

    struct i2c_client *i2c = bridge_i2c_client;
    int x0=0, x1=0;
    int send_i2c_ret = -1;
    sscanf(buf, "%x,%x", &x0, &x1);
    send_i2c_ret = tc358774_regw(i2c, x0, x1);

    DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_i2c_ret);
    printk("[b101ean01] %s return\n", __func__);
    return count;
}

static ssize_t send_i2c_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
    tc358774_configure_lvds_bridge_read();

    return snprintf(buf, PAGE_SIZE, "Read i2c Done\n");
}

DEVICE_ATTR(send_mipi_b101ean01,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_b101ean01,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);
DEVICE_ATTR(send_i2c_b101ean01,S_IRUGO | S_IWUSR, send_i2c_show,send_i2c_store);


static struct attribute *b101ean01_attrs[] = {
        &dev_attr_send_mipi_b101ean01.attr,
        &dev_attr_read_mipi_b101ean01.attr,
        &dev_attr_send_i2c_b101ean01.attr,
        NULL
};

static struct attribute_group b101ean01_attr_group = {
        .attrs = b101ean01_attrs,
        .name = "b101ean01",
};

#endif

void b101ean01_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret;
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode =  b101ean01_vid_get_config_mode;//5 OK
	p_funcs->get_panel_info = b101ean01_vid_get_panel_info;//6 OK
	p_funcs->reset = b101ean01_vid_reset;//9 12
	p_funcs->drv_ic_init = b101ean01_send_otp_cmds;//10 13
	p_funcs->dsi_controller_init = b101ean01_vid_dsi_controller_init; //8
	p_funcs->detect = b101ean01_vid_detect; //7
	p_funcs->power_on = b101ean01_vid_power_on; //14
	p_funcs->power_off = b101ean01_vid_power_off;//11
	p_funcs->set_brightness = b101ean01_set_brightness; //11 15


	ret = b101ean01_vid_gpio_init();//2 OK
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = b101ean01_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

#if 0
        b101ean01_vid_gpio_reset();
        b101ean01_vid_gpio_control(1);
#endif
        bridge_hack_create_device();

#ifdef B101EAN01_DEBUG
        sysfs_create_group(&dev->dev->kobj, &b101ean01_attr_group);
#endif
        DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
        printk("[b101ean01] %s return\n", __func__);

}

static int b101ean01_vid_lcd_probe(struct platform_device *pdev)
{
        int ret = 0;
        printk("[b101ean01] %s enter\n", __func__);

	DRM_INFO("%s: b101ean01 panel detected\n", __func__);
	intel_mid_panel_register(b101ean01_vid_init);
        printk("[b101ean01] %s return\n", __func__);

	return 0;
}

struct platform_driver b101ean01_vid_lcd_driver = {
	.probe	= b101ean01_vid_lcd_probe,
	.driver	= {
		.name	= "B101EAN01",
		.owner	= THIS_MODULE,
	},
};
