/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

static unsigned int HW_ID = 0xFF;
static unsigned int SKU_ID = 0xFF;

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

#define IS_BYT (INTEL_MID_BOARD(1, PHONE, BYT) || \
	INTEL_MID_BOARD(1, TABLET, BYT))

/* MFLD iCDK camera sensor GPIOs */

extern int intel_scu_ipc_msic_vemmc1(int); //VCM power

#define GP_CAMERA_ISP_POWER_1P2_EN	"ISP_1P2_EN" //iCatch 1.2V powwer enable pin, GPIO:
#define GP_CAMERA_ISP_POWER_1P8_EN	"ISP_1P8_EN" //iCatch 1.8V powwer enable pin, GPIO:
#define GP_CAMERA_ISP_RESET		    "ISP_RST_N" //iCatch reset pin, GPIO:
#define GP_CAMERA_ISP_SUSPEND		"ISP_SUSPEND" //iCatch suspend pin, GPIO:
#define GP_CAMERA_ISP_INT		    "ISP_INT" //iCatch interrupt pin, GPIO:

// ASUS_BSP +++ fix gpio direction output warning
#define SPI_1_CLK 23
#define SPI_1_SS3 19
#define SPI_1_SDO 21
// ASUS_BSP --- fix gpio direction output warning

#if 0
/* Obsolete pin, maybe used by old MFLD iCDK */
#define GP_CAMERA_0_POWER_DOWN          "cam0_vcm_2p8"
/* Camera VDD 1.8V Switch */
#define GP_CAMERA_1P8			"camera_on_n"
/* Camera0 Standby pin control */
#define GP_CAMERA_0_STANDBY		"camera_0_power"
#define GP_CAMERA_1_POWER_DOWN          "camera_1_power"
#define GP_CAMERA_0_RESET               "camera_0_reset"
#define GP_CAMERA_1_RESET               "camera_1_reset"
#endif
#ifdef CONFIG_TF103CG
//Wesley++
#define GP_CAMERA_VGA_RESET		"FCAM_RST_N"
#define GP_CAMERA_1_PDWN		"FCAM_PD"

#define GP_CAMERA_0_RESET		"RCAM_RST_N"
#define GP_CAMERA_0_PDWN		"RCAM_PD"
//Wesley--

#define GP_CAMERA_1_RESET		"RCAM_RST_N"
#define GP_CAMERA_2_8V 			"CAM_2P8_EN"
#endif
extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
// ASUS_BSP +++ get raw data
extern int camera_sensor_csi_2(struct v4l2_subdev *sd, u32 port,
                        u32 lanes, u32 format, u32 bayer_order,
                        u32 raw_format, u32 raw_bayer_order, int flag);
// ASUS_BSP --- get raw data
extern void intel_register_i2c_camera_device(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
				__attribute__((weak));

/*
 * FIXME! This PMIC power access workaround for BTY
 * since currently no VRF for BTY
 */
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2

enum camera_pmic_pin {
	CAMERA_1P8V,
	CAMERA_2P8V,
	CAMERA_POWER_NUM,
};

struct vprog_status {
	unsigned int user;
};

int camera_set_pmic_power(enum camera_pmic_pin pin, bool flag);
#endif
#endif
