#ifndef _ELAN_I2C_ASUS_H
#define _ELAN_I2C_ASUS_H

#include "asusdec.h"

//Shouchung modify, disable debug message in MP image
#define ELAN_DEBUG			0
//Shouchung end
#if ELAN_DEBUG
#define ELAN_INFO(format, arg...)	\
	printk(KERN_INFO "elan_i2c_asus: [%s] " format , __FUNCTION__ , ## arg)
#else
#define ELAN_INFO(format, arg...)
#endif

#define ELAN_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "elan_i2c_asus: [%s] " format , __FUNCTION__ , ## arg)

#define ELAN_ERR(format, arg...)	\
	printk(KERN_ERR "elan_i2c_asus: [%s] " format , __FUNCTION__ , ## arg)

#define ELANTOUCHPAD		727
#define NEED_GET_INFO		0
#define SET_REPORT_MODE		1		// 0: standard mode, 1: absolute mode

#define REAL_RESO_X		1280
#define REAL_RESO_Y		800

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH			2
#define ETP_MAX_FINGERS			5
#define ETP_HID_DESC_LENGTH		30
#define ETP_REPORT_DESC_LENGTH	79
#define ETP_REPORT_ID_OFFSET	2
//Shouchung modify, the standard report length is 7 not 6
#define ETP_STD_REPORT_LENGTH	7
//Shouchung end
#define ETP_ABS_REPORT_LENGTH	30
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_FINGER_DATA_LEN		5

#define ETP_STD_REPORT_ID	0x01
#define ETP_ABS_REPORT_ID	0x5d

#define REG_HID_DESC		0x0001
#define REG_REPORT_DESC		0x0002
#define REG_INPUT_REPORT	0x0003
#define REG_OUTPUT_REPORT	0x0004
#define REG_COMMAND			0x0005
#define REG_DATA			0x0006
#define REG_HID_VERSION		0x0100
#define REG_FW_VERSION		0x0102		//Not exist in spec
#define REG_XY_TRACE_NUM	0x0105
#define REG_X_AXIS_MAX		0x0106
#define REG_Y_AXIS_MAX		0x0107
#define REG_DPI				0x0108		//Not resolution
#define REG_MK_VALUE		0x0109
#define REG_TP_DRIVER_CONTROL	0x0300

#define CMD_RESET		0x0100
#define CMD_WAKE_UP		0x0800
#define CMD_SLEEP		0x0801
#define CMD_STD_MODE	0x0000
#define CMD_ABS_MODE	0x0001

#define LEFT_BTN_CHECK		0x01
#define RIGHT_BTN_CHECK		0x02

int elan_i2c_enable(struct i2c_client *client);
int elan_i2c_disable(struct i2c_client *client);
void elan_i2c_report_data(struct elan_i2c_data *data, u8 *packet);
int elan_i2c_check_packet(u8 *packet);
int elan_i2c_input_dev_create(struct elan_i2c_data *data);
int elan_i2c_initialize(struct i2c_client *client);

#endif
