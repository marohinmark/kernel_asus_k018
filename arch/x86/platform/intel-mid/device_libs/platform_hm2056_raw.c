/*
 * platform_hm2056_raw.c: hm2056_raw platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
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
#include "platform_hm2056_raw.h"
#include "platform_mt9e013.h"

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;
static int camera_sensor_1_8v;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
/*
 * MFLD PR2 secondary camera sensor - HM2056_RAW platform data
 */
static int hm2056_raw_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	/* disable by Wesley_kao, modify power sequence
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(177, GP_CAMERA_0_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = 177;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(60);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			gpio_free(camera_reset);
			camera_reset = -1;
			msleep(10);
		}
	}
	*/
	return 0;
}

static int hm2056_raw_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0); //Wesley, from OSC_CLK_CAM1 to OSC_CLK_CAM0 for 170
}

static int hm2056_raw_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err, ret;

	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(161, GP_CAMERA_0_PDWN,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_0_PDWN);
			return ret;
		}
		camera_power_down = 161;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(177, GP_CAMERA_0_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = 177;
	}

	if (flag) {

		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			msleep(10);
		}

		//turn off power down
		if (camera_power_down >= 0){
			gpio_set_value(camera_power_down, 0);
			printk("<<< PWDN = 0\n");
			msleep(10);
		}

		//turn on I2C 1.8V
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 1;
				printk("<<< I2C 1.8V = 1\n");
			}
			else{
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return ret;
			}
			msleep(10);
		}

		//turn on I2C 2.8V
		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 1;
				printk("<<< I2C 2.8V = 1\n");
			}
			else{
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return ret;
			}
			msleep(10);
		}

		hm2056_raw_flisclk_ctrl(sd, 1);
		msleep(10);

		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 1);
			msleep(10);
		}


	} else {


                //trun off camera reset pin
                if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			gpio_free(camera_reset);
			camera_reset = -1;
			msleep(10);
		}
		//turn off I2C 2.8V
		if (camera_vprog2_on) {
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 0;
				printk("<<< I2C 2.8V = 0\n");
			}
			else {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return ret;
			}
			msleep(10);
		}

		//turn off I2C 1.8V
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 0;
				printk("<<< I2C 1.8V = 0\n");
			}
			else {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return ret;
			}
			msleep(100);
		}

		//turn off PWDN
		if (camera_power_down >= 0){
			gpio_set_value(camera_power_down, 0);
			printk("<<< PWDN = 0\n");
			gpio_free(camera_power_down);
			camera_power_down = -1;
			msleep(60);
		}

		hm2056_raw_flisclk_ctrl(sd, 0);
		msleep(10);
	}

	return 0;
}

static int hm2056_raw_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_gbrg, flag);

//	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
//		-1, 0, flag);

//	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
//			ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);	
}

static int hm2056_raw_platform_init(struct i2c_client *client)
{
	int ret = 0;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get 1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage 1 set failed\n");
		regulator_put(vprog1_reg);
	}

	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "regulator_get 2 failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage 2 set failed\n");
		regulator_put(vprog2_reg);
	}

	return ret;
}

static int hm2056_raw_platform_deinit(void)
{
	regulator_put(vprog1_reg);
        regulator_put(vprog2_reg);
	return 0;
}

static struct camera_sensor_platform_data hm2056_raw_sensor_platform_data = {
	.gpio_ctrl	= hm2056_raw_gpio_ctrl,
	.flisclk_ctrl	= hm2056_raw_flisclk_ctrl,
	.power_ctrl	= hm2056_raw_power_ctrl,
	.csi_cfg	= hm2056_raw_csi_configure,
	.platform_init = hm2056_raw_platform_init,
	.platform_deinit = hm2056_raw_platform_deinit,
};

void *hm2056_raw_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_sensor_1_8v = -1;

	return &hm2056_raw_sensor_platform_data;
}

