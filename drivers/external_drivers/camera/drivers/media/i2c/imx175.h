/*
 * Support for imx175 Camera Sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __IMX175_H__
#define __IMX175_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
//FW_BSP+++
#include <linux/wakelock.h>
//FW_BSP---


#define V4L2_IDENT_IMX175 0001 //Peter, what is this?

#define MSG_LEN_OFFSET		2
#define MAX_FMTS		1

//parameter define+++
#define SENSOR_ISO_AUTO		0
#define SENSOR_ISO_50		1
#define SENSOR_ISO_100		2
#define SENSOR_ISO_200		3
#define SENSOR_ISO_400		4
#define SENSOR_ISO_800		5
#define SENSOR_ISO_1600         6

#define SENSOR_FOCUS_MODE_AUTO		0
#define SENSOR_FOCUS_MODE_MACRO		1
#define SENSOR_FOCUS_MODE_INFINITY	2
#define SENSOR_FOCUS_MODE_FIXED		3
#define SENSOR_FOCUS_MODE_TOUCH		4
#define SENSOR_FOCUS_MODE_MANUAL	5
#define SENSOR_FOCUS_MODE_FACE		6
#define SENSOR_FOCUS_MODE_CONTINUOUS	7

#define SENSOR_ROI_TRIGGER_AF 0
#define SENSOR_ROI_TRIGGER_AE_AF 1
#define SENSOR_ROI_TRIGGER_STOP 2

#define SENSOR_WDR_OFF  0
#define SENSOR_WDR_ON  1

#define SENSOR_EIS_OFF  0 //ASUS_BSP, Camera5.0
#define SENSOR_EIS_ON  1 //ASUS_BSP, Camera5.0

#define SENSOR_MOVING_OFF  0
#define SENSOR_MOVING_ON  1

#define SENSOR_AE_AWB_AF_UNLOCK  0
#define SENSOR_AE_AWB_UNLOCK  1
#define SENSOR_AE_LOCK  2
#define SENSOR_AWB_LOCK  3
#define SENSOR_AE_UNLOCK  4
#define SENSOR_AWB_UNLOCK  5
#define SENSOR_AE_AWB_LOCK  6

//parameter define---

//i2c r/w +++
#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

struct sensor_reg {
	u16 addr;
	u16 val;
};
//i2c r/w --

//TODO: need to modify
#define IMX175_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define IMX175_FOCAL_LENGTH_DEM	100
#define IMX175_F_NUMBER_DEFAULT_NUM	24
#define IMX175_F_NUMBER_DEM	10

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define IMX175_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define IMX175_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define IMX175_F_NUMBER_RANGE 0x180a180a


//FW_BSP++
enum iCatch_fw_update_status{
	ICATCH_FW_NO_CMD,
	ICATCH_FW_IS_BURNING,
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;

enum iCatch_flash_type{
	ICATCH_FLASH_TYPE_ST,
	ICATCH_FLASH_TYPE_SST,
};
static enum iCatch_flash_type flash_type = ICATCH_FLASH_TYPE_ST;

//FW_BSP--




//TODO: add more resolution later
/* Supported resolutions */
enum {
	IMX175_RES_QCIF,
        IMX175_RES_QVGA,
        IMX175_RES_CIF,
	IMX175_RES_VGA,
	IMX175_RES_720P,
	IMX175_RES_960P,
	IMX175_RES_1080P,
        IMX175_RES_3M,
	IMX175_RES_5M,
        IMX175_RES_8M, //ASUS_BSP++
};

#define IMX175_RES_8M_SIZE_H		3264  //ASUS_BSP++
#define IMX175_RES_8M_SIZE_V		2448  //ASUS_BSP++
#define IMX175_RES_5M_SIZE_H		2592
#define IMX175_RES_5M_SIZE_V		1944
#define IMX175_RES_3M_SIZE_H            2080
#define IMX175_RES_3M_SIZE_V            1560
#define IMX175_RES_1080P_SIZE_H		1920
#define IMX175_RES_1080P_SIZE_V		1080
#define IMX175_RES_960P_SIZE_H		1280
#define IMX175_RES_960P_SIZE_V		960
#define IMX175_RES_720P_SIZE_H		1280
#define IMX175_RES_720P_SIZE_V		720
#define IMX175_RES_480P_SIZE_H		768
#define IMX175_RES_480P_SIZE_V		480
#define IMX175_RES_VGA_SIZE_H		640
#define IMX175_RES_VGA_SIZE_V		480
#define IMX175_RES_QVGA_SIZE_H		320
#define IMX175_RES_QVGA_SIZE_V		240
#define IMX175_RES_CIF_SIZE_H           352
#define IMX175_RES_CIF_SIZE_V           288
#define IMX175_RES_QCIF_SIZE_H		176
#define IMX175_RES_QCIF_SIZE_V		144

struct imx175_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	unsigned int res;
	int last_run_mode;

	struct mutex input_lock; /* serialize sensor's ioctl */
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *run_mode;
//FW_BSP++
	struct wake_lock *fw_update_wakelock;
//FW_BSP--

	struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct imx175_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct imx175_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	int row_time;
	bool used;
	struct regval_list *regs;
};

struct imx175_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/*
 * Modes supported by the imx175 driver.
 * Please, keep them in ascending order.
 */
static struct imx175_res_struct imx175_res[] = {
	{
	.desc	= "QCIF",
	.res	= IMX175_RES_QCIF,
	.width	= 176,
	.height	= 144,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
        {
        .desc   = "QVGA",
        .res    = IMX175_RES_QVGA,
        .width  = 320,
        .height = 240,
        .fps    = 30,
        .used   = 0,
        .regs   = NULL,
        .skip_frames = 0,
        },
        {
        .desc   = "CIF",
        .res    = IMX175_RES_CIF,
        .width  = 352,
        .height = 288,
        .fps    = 30,
        .used   = 0,
        .regs   = NULL,
        .skip_frames = 0,
        },
	{
	.desc	= "VGA",
	.res	= IMX175_RES_VGA,
	.width	= 640,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
/*
	{
	.desc	= "480p",
	.res	= IMX175_RES_480P,
	.width	= 768,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
*/
	{
	.desc	= "720p",
	.res	= IMX175_RES_720P,
	.width	= 1280,
	.height	= 720,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "960P",
	.res	= IMX175_RES_960P,
	.width	= 1280,
	.height	= 960,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "1080P",
	.res	= IMX175_RES_1080P,
	.width	= 1920,
	.height	= 1080,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
        {
        .desc   = "3M",
        .res    = IMX175_RES_3M,
        .width  = 2080,
        .height = 1560,
        .fps    = 30,
        .used   = 0,
        .regs   = NULL,
        .skip_frames = 0,
        },
	{
	.desc	= "5M",
	.res	= IMX175_RES_5M,
	.width	= 2592,
	.height	= 1944,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "8M",  //ASUS_BSP++
	.res	= IMX175_RES_8M,
	.width	= 3264,
	.height	= 2448,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
};
#define N_RES (ARRAY_SIZE(imx175_res))

static const struct i2c_device_id imx175_id[] = {
	{"imx175", 0},
	{}
};

#endif
