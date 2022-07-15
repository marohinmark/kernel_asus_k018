/*
 * platform_mt9m114.c: MT9M114 with iCatch 7002A ISP platform data initilization file
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
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"
#include <linux/HWVersion.h>
//FW_BSP++, 0522 load from host
#include <linux/lnw_gpio.h>
//FW_BSP--, 0522 load from host

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
#define VEMMC2_VAL 2850000

extern int Read_HW_ID(void);

static int camera_power_1p2_en;
static int camera_power_1p8_en;
static int camera_reset;
static int camera_suspend;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc_on;

static struct regulator *vprog1_reg;
//static struct regulator *vprog2_reg;
static struct regulator *vemmc_reg;

/*
 * ME560CG Camera - MT9M114 platform data
 */
static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	return 0;
}

static int mt9m114_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz
	int ret = 0;
	v4l2_info(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
	return ret;
}

static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret;
	v4l2_info(sd, "%s: ++\n",__func__);

        HW_ID = Read_HW_ID(); //read HW/SKU ID

        pr_debug("HW_ID :%d,\n", HW_ID);

        if (camera_power_1p2_en < 0) {
        ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_POWER_1P2_EN,
							 GPIOF_DIR_OUT, 0);
        if (ret < 0){
            printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
            return ret;
        }
        camera_power_1p2_en = ret;
	}

	if (camera_power_1p8_en < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_POWER_1P8_EN,
							 GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P8_EN);
			return ret;
		}
		camera_power_1p8_en = ret;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_RESET,
							 GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_ISP_RESET);
			return ret;
		}
		camera_reset = ret;
	}

	if (camera_suspend < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_SUSPEND,
							GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
			return ret;
		}
		camera_suspend = ret;
	}

	pr_debug("<< Power flag:%d\n", flag);
	if (flag){

                if(HW_ID_SR1 == HW_ID || HW_ID_SR1_SKU_3_4 == HW_ID){
			//For switch camera
			gpio_set_value(111, 0);
			printk("<< SUBCAM_MCLK_EN_N:%d\n", 0);
			msleep(1);
                }

		//pull low reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			pr_debug("<<< camera_reset = 0\n");
			msleep(1);
		}

		//turn on DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 1);
			pr_debug("<<< DVDD 1.2V = 1\n");
			msleep(1);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;

			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				pr_debug(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}

			pr_debug("<<< VDD_SEN VDD_HOST 1.8V = 1 vprog1\n");
			msleep(10);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (camera_power_1p8_en >= 0){
			gpio_set_value(camera_power_1p8_en, 1);
			pr_debug("<<< DVDD 1.8V = 1\n");
			msleep(1);
		}

		//turn on VCM power 2.85V for ISP 2.8
		if (!camera_vemmc_on) {
			camera_vemmc_on = 1;

			reg_err = regulator_enable(vemmc_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vemmc\n");
				return reg_err;
			}

			pr_debug("<<< VCM 2.85V = 1 vemmc for ISP 2.8 : mt9m114\n");
			msleep(10);
		}

		//turn on MCLK
		mt9m114_flisclk_ctrl(sd, 1);
		msleep(1); //need wait 16 clk cycle

		/*Need change load code flow here +++*/
		//Pull high suspend to load fw from SPI / Pull low suspend to load fw from host
                switch (HW_ID){
                        case HW_ID_SR1:
                        case HW_ID_SR1_SKU_3_4:
                                if (camera_suspend >= 0){
                                            gpio_set_value(camera_suspend, 1);
                                            pr_debug("<<< suspend = 1, load fw from rom\n");
                                }

                                //Reset control
                                if (camera_reset >= 0){
                                            gpio_set_value(camera_reset, 1);
                                            pr_debug("<<< reset = 1\n");
                                            msleep(6); //wait 6ms
                                }
                                break;

                        case HW_ID_PRE_ER:
                        case HW_ID_ER:
                        case HW_ID_PR:
                        case HW_ID_MP:
                                if (camera_suspend >= 0){
                                            gpio_set_value(camera_suspend, 0);
                                            pr_debug("<<< suspend = 0, load fw from host\n");
                                }

                                lnw_gpio_set_alt(SPI_1_CLK, LNW_GPIO);

                                gpio_request(SPI_1_CLK, NULL);
                                gpio_direction_output(SPI_1_CLK, 1);
                                msleep(6); //wait 6ms

                                //Reset control
                                if (camera_reset >= 0){
                                            gpio_set_value(camera_reset, 1);
                                            pr_debug("<<< reset = 1\n");
                                            msleep(6); //wait 6ms
                                }

                                gpio_direction_output(SPI_1_CLK, 0);
                                gpio_free(SPI_1_CLK);

                                lnw_gpio_set_alt(SPI_1_CLK, LNW_ALT_1);
                                lnw_gpio_set_alt(SPI_1_SS3, LNW_ALT_1);
                                lnw_gpio_set_alt(SPI_1_SDO, LNW_ALT_1);
                                break;
                }

		//Pull low suspend
		if (camera_suspend >= 0){
			gpio_set_value(camera_suspend, 0);
			pr_debug("<<< suspend = 0\n");
		}

		msleep(10); //delay time for first i2c command
	}else{
		//pull low reset
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			pr_debug("<<< reset = 0\n");
			gpio_free(camera_reset);
			camera_reset = -1;
		}

		//turn off MCLK
		mt9m114_flisclk_ctrl(sd, 0);

		//turn off VCM power 2.85V for ISP 2.8
		if (camera_vemmc_on) {
			camera_vemmc_on = 0;

			reg_err = regulator_disable(vemmc_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vemmc\n");
				return reg_err;
			}

			pr_debug("<<< VCM 2.85V = 0 : mt9m1141\n");
			msleep(10);
		}

		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_power_1p8_en >= 0){
			gpio_set_value(camera_power_1p8_en, 0);
			pr_debug("<<< DVDD 1.8V = 0\n");
			gpio_free(camera_power_1p8_en);
			camera_power_1p8_en = -1;
		}
		msleep(1);

		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;

			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}

			pr_debug("<<< VDD_SEN VDD_HOST 1.8V = 0 vprog1\n");
			msleep(10);
		}

		//turn off DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 0);
			pr_debug("<<< DVDD 1.2V = 0\n");
			gpio_free(camera_power_1p2_en);
			camera_power_1p2_en = -1;
		}
		msleep(1); //need 0.1ms delay

		//release suspend gpio
		if (camera_suspend >= 0){
			pr_debug("<<< Release camera_suspend pin:%d\n", camera_suspend);
			gpio_free(camera_suspend);
			camera_suspend = -1;
		}
                // ASUS_BSP +++ fix gpio direction output warning
                lnw_gpio_set_alt(SPI_1_SS3, LNW_GPIO);
                lnw_gpio_set_alt(SPI_1_SDO, LNW_GPIO);
                gpio_request(SPI_1_SS3, NULL);
                gpio_request(SPI_1_SDO, NULL);

                gpio_direction_output(SPI_1_SS3, 0);
                gpio_direction_output(SPI_1_SDO, 0);
                gpio_free(SPI_1_SS3);
                gpio_free(SPI_1_SDO);
                // ASUS_BSP --- fix gpio direction output warning
	}

	return 0;
}

static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
        // ASUS_BSP +++ get raw data
        return camera_sensor_csi_2(sd, ATOMISP_CAMERA_PORT_PRIMARY, 4,
               ATOMISP_INPUT_FORMAT_YUV422_8, -1,
               ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr,
               flag);
        // ASUS_BSP --- get raw data
}

static int mt9m114_platform_init(struct i2c_client *client)
{
	int ret;

	pr_debug("%s: ++\n", __func__);

	//VPROG1 for VDD_HOST and VDD_SEN, 1.8V
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 set failed\n");
		regulator_put(vprog1_reg);
	}

	/*
	//VPROG2 for AVDD, 2.8V
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "vprog2 failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog2 set failed\n");
		regulator_put(vprog2_reg);
	}*/

	vemmc_reg = regulator_get(&client->dev, "vemmc2");
	if (IS_ERR(vemmc_reg)) {
		dev_err(&client->dev, "vemmc2 failed\n");
		return PTR_ERR(vemmc_reg);
	}

	return ret;
}

static int mt9m114_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	//regulator_put(vprog2_reg);
	regulator_put(vemmc_reg);
}

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	 = mt9m114_gpio_ctrl,
	.flisclk_ctrl	 = mt9m114_flisclk_ctrl,
	.power_ctrl	 = mt9m114_power_ctrl,
	.csi_cfg	 = mt9m114_csi_configure,
	.platform_init   = mt9m114_platform_init,
	.platform_deinit = mt9m114_platform_deinit,
};

void *mt9m114_platform_data(void *info)
{
    pr_debug("%s\n", __func__);
	camera_power_1p2_en = -1;
	camera_power_1p8_en = -1;
	camera_reset = -1;
	camera_suspend = -1;
	return &mt9m114_sensor_platform_data;
}
