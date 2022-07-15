#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/input/mt.h>

#include "elan_i2c_asus.h"

static int elan_i2c_read_reg(struct i2c_client *client, u16 reg,
					u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;
	int tries = 5;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	do{
		ret = i2c_transfer(client->adapter, msgs, 2);
		msleep(20);
		if (ret != 2)
			ELAN_ERR("wrong length ret: %d, msg_num: %d\n");
		else
			break;
		tries--;
		ELAN_ERR("retrying elan_i2c_read_reg:%d (%d)\n", reg, tries);
	} while (tries > 0);

	return ret != 2 ? -EIO : 0;
}

static int elan_i2c_write_reg(struct i2c_client *client, u16 reg,
					u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	msleep(20);
	if (ret < 0)
		return ret;

	return ret != 1 ? -EIO : 0;
}

int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_reg(client, REG_COMMAND, CMD_RESET);
}

int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_HID_DESC, val,
					ETP_HID_DESC_LENGTH);
}

int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_REPORT_DESC, val,
					ETP_REPORT_DESC_LENGTH);
}

int elan_i2c_set_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg(client, REG_TP_DRIVER_CONTROL,
					CMD_ABS_MODE);
}

int elan_i2c_set_standard_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg(client, REG_TP_DRIVER_CONTROL,
					CMD_STD_MODE);
}

int elan_i2c_enable(struct i2c_client *client)
{
	return elan_i2c_write_reg(client, REG_COMMAND, CMD_WAKE_UP);
}

int elan_i2c_disable(struct i2c_client *client)
{
	return elan_i2c_write_reg(client, REG_COMMAND, CMD_SLEEP);
}

int elan_i2c_get_x_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_X_AXIS_MAX, val,
					ETP_INF_LENGTH);
}

int elan_i2c_get_y_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_Y_AXIS_MAX, val,
					ETP_INF_LENGTH);
}

//Shouchung modify, for HIT team short term solution
void elan_i2c_report_absolute(struct elan_i2c_data *data, u8 *packet)
{
	struct input_dev *input = data->input;
	u8 *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	bool button_click;
	bool finger_on[ETP_MAX_FINGERS];
	int pos_x[ETP_MAX_FINGERS], pos_y[ETP_MAX_FINGERS];
	int area_x[ETP_MAX_FINGERS], area_y[ETP_MAX_FINGERS];
	int pressure[ETP_MAX_FINGERS];
	int i, finger_num, finger_normal_index, finger_button_index;
	int normal_area;

	finger_num = 0;
	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on[i] = (packet[3] >> (3 + i)) & 0x01;
		if (finger_on[i]) {
			finger_num++;
		}
	}

	button_click = ((packet[3] & LEFT_BTN_CHECK) == 1);

	normal_area = REAL_RESO_Y * 5 / 6;
	finger_normal_index = -1;
	finger_button_index = -1;
	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		input_mt_slot(input, i);
		if (finger_on[i]) {
			pos_x[i] = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
			pos_x[i] = (pos_x[i] * REAL_RESO_X) / data->max_x;
			pos_y[i] = data->max_y - (((finger_data[0] & 0x0f) << 8) |
					finger_data[2]);
			pos_y[i] = (pos_y[i] * REAL_RESO_Y) / data->max_y;
			area_x[i] = finger_data[3] & 0x0f;
			area_y[i] = finger_data[3] >> 4;
			pressure[i] = finger_data[4];

			finger_data += ETP_FINGER_DATA_LEN;

			if ((finger_num == 1) || (pos_y[i] < normal_area)) {
				input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
				if ((finger_num == 1 && !button_click) || (finger_num > 1)) {
					input_report_abs(input, ABS_MT_POSITION_X, pos_x[i]);
					input_report_abs(input, ABS_MT_POSITION_Y, pos_y[i]);
				}
				input_report_abs(input, ABS_MT_PRESSURE, pressure[i]);
				input_report_abs(input, ABS_MT_TOUCH_MAJOR,
						max(area_x[i], area_y[i]));
				if (finger_normal_index == -1)
					finger_normal_index = i;
			} else {
				input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
				finger_button_index = i;
			}
		} else {
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		}
	}

	if (button_click && ((finger_button_index != -1) || (finger_num == 1))) {
		if (finger_button_index == -1)
			finger_button_index = finger_normal_index;
		if (pos_x[finger_button_index] < (REAL_RESO_X / 2)) {
			input_report_key(input, BTN_LEFT, true);
			data->btn_left = 1;
		} else {
			input_report_key(input, BTN_RIGHT, true);
			data->btn_right = 1;
		}
	} else if (!button_click) {
		if (data->btn_left == 1) {
			input_report_key(input, BTN_LEFT, false);
			data->btn_left = 0;
		} else if (data->btn_right == 1) {
			input_report_key(input, BTN_RIGHT, false);
			data->btn_right = 0;
		}
	}

	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
	ELAN_INFO(
			"report data pos_x = %d, pos_y = %d, finger_num = %d, " \
			"btn_left = %d, btn_right = %d\n",
			pos_x[finger_normal_index], pos_y[finger_normal_index],
			finger_num, data->btn_left, data->btn_right);
}
//Shouchung end

void elan_i2c_report_standard(struct elan_i2c_data *data, u8 *packet)
{
	struct input_dev *input = data->input;
	u8 *finger_data = &packet[2];
	int left_btn, right_btn, delta_x, delta_y;

	left_btn = (finger_data[1] & LEFT_BTN_CHECK) ? 1 : 0;
	right_btn = (finger_data[1] & RIGHT_BTN_CHECK) ? 1 : 0;
	delta_x =
		(finger_data[2] > 0x80) ? (finger_data[2] - 0xff) : finger_data[2];
	delta_y =
		(finger_data[3] > 0x80) ? (finger_data[3] - 0xff) : finger_data[3];

	input_report_rel(input, REL_X, delta_x);
	input_report_rel(input, REL_Y, delta_y);
	input_report_key(input, BTN_LEFT, left_btn);
	input_report_key(input, BTN_RIGHT, right_btn);
	input_sync(input);
	ELAN_INFO(
			"report data delta_x = %d, delta_y = %d, " \
			"left_btn = %d, right_btn = %d\n",
			delta_x, delta_y, left_btn, right_btn);
}

//Shouchung modify, let the relative mode can work if initialize failed
//in MP image
void elan_i2c_report_data(struct elan_i2c_data *data, u8 *packet)
{
#if ELAN_DEBUG
	#if SET_REPORT_MODE
		elan_i2c_report_absolute(data, packet);
	#else
		elan_i2c_report_standard(data, packet);
	#endif
#else
	if (elan_i2c_check_packet(packet) == 1)
		elan_i2c_report_absolute(data, packet);
	else
		elan_i2c_report_standard(data, packet);
#endif
}

int elan_i2c_check_packet(u8 *packet)
{
	u16 length = le16_to_cpu(packet[0]);
	u8 report_id = packet[ETP_REPORT_ID_OFFSET];

	ELAN_INFO("check received packet length = %x(%x), report_id = %x(%x)\n",
			length, ETP_ABS_REPORT_LENGTH, report_id, ETP_ABS_REPORT_ID);
	if (length == ETP_ABS_REPORT_LENGTH && report_id == ETP_ABS_REPORT_ID) {
		ELAN_INFO("the received packet is absolute mode\n");
		return 1;
	}

	ELAN_INFO("check received packet length = %x(%x), report_id = %x(%x)\n",
			length, ETP_STD_REPORT_LENGTH, report_id, ETP_STD_REPORT_ID);
	if (length == ETP_STD_REPORT_LENGTH && report_id == ETP_STD_REPORT_ID) {
		ELAN_INFO("the received packet is standard mode\n");
		return 2;
	}

	ELAN_INFO("the received packet is error\n");
	return -1;
}
//Shouchung end

int elan_i2c_input_dev_create(struct elan_i2c_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input;
	u8 val[ETP_INF_LENGTH];
	int ret;

	//Shouchung add, check the input device is created or not
	if (data->input != NULL)
		return 0;
	//Shouchung end

	input = input_allocate_device();
	if (!input) {
		ELAN_ERR("input device allocation failed\n");
		return -ENOMEM;
	}

	input->name = "Elan I2C Touchpad";
	input->id.bustype = BUS_I2C;

	elan_i2c_get_x_max(client, val);
	data->max_x = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_y_max(client, val);
	data->max_y = (0x0f & val[1]) << 8 | val[0];

	set_bit(INPUT_PROP_POINTER, input->propbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_TOOL_FINGER, input->keybit);
	set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
	set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
	input_set_abs_params(input, ABS_X, 0, REAL_RESO_X, 0, 0);
    input_set_abs_params(input, ABS_Y, 0, REAL_RESO_Y, 0, 0);
    input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);
    input_abs_set_res(input, ABS_X, REAL_RESO_X);
	input_abs_set_res(input, ABS_Y, REAL_RESO_Y);

	input_mt_init_slots(input, ETP_MAX_FINGERS, INPUT_MT_POINTER |
			INPUT_MT_DROP_UNUSED);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, REAL_RESO_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, REAL_RESO_Y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, REAL_RESO_X);
	input_abs_set_res(input, ABS_MT_POSITION_Y, REAL_RESO_Y);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);

    set_bit(EV_REL, input->evbit);
	set_bit(REL_X, input->relbit);
    set_bit(REL_Y, input->relbit);

	set_bit(BTN_LEFT, input->keybit);
	set_bit(BTN_RIGHT, input->keybit);
	set_bit(EV_KEY, input->evbit);
    set_bit(EV_SYN, input->evbit);
    input_set_capability(input, EV_KEY, KEY_WAKEUP);

	ELAN_NOTICE(
		"Elan I2C Trackpad Information:\n" \
		"    Max ABS X,Y:   %d,%d\n",
		data->max_x, data->max_y);

	ret = input_register_device(input);
	if (ret) {
		ELAN_ERR("input device register failed, %d\n", ret);
		input_free_device(input);
		return ret;
	}
	
	data->input = input;

	return 0;
}

int elan_i2c_initialize(struct i2c_client *client)
{
#if NEED_GET_INFO
	u8 val[ETP_REPORT_DESC_LENGTH];
	int i;
#endif
	int rc;

	ELAN_NOTICE("elan_i2c_initialize start\n");

	rc = elan_i2c_reset(client);
	if (rc < 0) {
		ELAN_ERR("device reset failed\n");
		return -1;
	}

#if NEED_GET_INFO
	rc = elan_i2c_get_desc(client, val);
	if (rc < 0) {
		ELAN_ERR("get device descriptor failed\n");
		return -1;
	}
	for (i = 0; i < ETP_HID_DESC_LENGTH; i += 2) {
		ELAN_NOTICE("Device Descriptor [%d][%d] = %x, %x\n", i, i + 1,
				val[i], val[i + 1]);
	}

	rc = elan_i2c_get_report_desc(client, val);
	if (rc < 0) {
		ELAN_ERR("get report descriptor failed\n");
		return -1;
	}
#endif

#if SET_REPORT_MODE
	rc = elan_i2c_set_absolute_mode(client);
	if (rc < 0) {
		ELAN_ERR("switch to absolute mode failed\n");
		return -1;
	}
#else
	rc = elan_i2c_set_standard_mode(client);
	if (rc < 0) {
		ELAN_ERR("switch to standard mode failed\n");
		return -1;
	}
#endif

	rc = elan_i2c_enable(client);
	if (rc < 0) {
		ELAN_ERR("device wake up failed\n");
		return -1;
	}

	ELAN_NOTICE("elan_i2c_initialize end\n");

	return 0;
}
