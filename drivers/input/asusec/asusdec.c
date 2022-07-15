/*
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <asm/gpio.h>
#include <linux/statfs.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/acpi.h>
#include <linux/spinlock.h>
#include <linux/acpi_gpio.h>
#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>

#include "asusdec.h"
#include "elan_i2c_asus.h"

#define GPIO1P3 0x3E

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */
static void asusdec_send_ec_req(void);
static int asusdec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asusdec_resume(struct i2c_client *client);
static int asusdec_open(struct inode *inode, struct file *flip);
static int asusdec_release(struct inode *inode, struct file *flip);
static long asusdec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static void asusdec_switch_apower_state(int state);
static void asusdec_win_shutdown(void);
static void asusdec_enter_factory_mode(void);
static void asusdec_enter_normal_mode(void);

static int asusdec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int asusdec_remove(struct i2c_client *client);

static int asusdec_set_wakeup_cmd(void);
static void asusdec_reset_dock(void);
static void asusdec_smi(void);
/*Power on/off TP/USB of dock ec start*/
int asusdec_susb_control(int arg);
/*Power on/off TP/USB of dock ec end*/

/*
* extern variable
*/
extern unsigned int factory_mode;

/*
 * global variable
 */
char* switch_value[]={"0", "10", "11", "12"}; //0: no dock, 1:mobile dock, 2:audio dock, 3: audio st

enum firmware_update_type {
	UPDATE_BYTE_MODE = 0,
	UPDATE_BLOCK_MODE,
};

/*Chris porting TF103CG start*/
#define	HARDWARE_ID_SR	0x0
extern int Read_HW_ID(void);
extern int entry_mode;
static int asusdec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH,
			ASUSDEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSDEC_KEY_AUTOBRIGHT,
			KEY_CAMERA, -9, -10, -11,
			-12, -13, -14, -15,
			KEY_WWW, ASUSDEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE,
			KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP,
			ASUSDEC_KEYPAD_READINGMODE};

static unsigned int asusdec_dock_in_gpio = 111;
static unsigned int asusdec_dock_in_gpio_irq = 0;
static unsigned int asusdec_apwake_gpio = 72;
static unsigned int asusdec_apwake_gpio_irq = 0;
static unsigned int asusdec_dock_power = 42;
static unsigned int asusdec_hall_sensor_gpio = 0;
static unsigned int asusdec_dock_5v_enable = 114;
static unsigned int dock_ec_pwr_en = 77;
/*Chris porting TF103CG end*/

static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data
int reg_addr = -1;
static int fu_type = 0;
static int fu_block_mode = 0;
static struct i2c_client kb_client;
static struct i2c_client tp_client;
static struct i2c_client intr_client;
struct i2c_client dockram_client;

static struct class *asusdec_class;
static struct device *asusdec_device ;
static struct asusdec_chip *ec_chip;
static struct elan_i2c_data *elan_data;

struct cdev *asusdec_cdev ;
static dev_t asusdec_dev ;
static int asusdec_major = 0 ;
static int asusdec_minor = 0 ;

static struct workqueue_struct *asusdec_wq;
struct delayed_work asusdec_stress_work;

/*
 * functions definition
 */

void ec_irq_disable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ec_chip->irq_lock, irqflags);
	printk("ec_irq_disable flag = %d\n", ec_chip->apwake_disabled);
	if (!ec_chip->apwake_disabled)
	{
		ec_chip->apwake_disabled = 1;
		disable_irq_nosync(gpio_to_irq(asusdec_apwake_gpio));
//		disable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
	}
	spin_unlock_irqrestore(&ec_chip->irq_lock, irqflags);
}


void ec_irq_enable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ec_chip->irq_lock, irqflags);
	printk("ec_irq_enable flag = %d\n", ec_chip->apwake_disabled);
	if (ec_chip->apwake_disabled) 
	{
		enable_irq(gpio_to_irq(asusdec_apwake_gpio));
//		enable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
		ec_chip->apwake_disabled = 0;
	}
	spin_unlock_irqrestore(&ec_chip->irq_lock, irqflags);
}

static int asusdec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSDEC_MAGIC_NUM){
		ASUSDEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSDEC_MAGIC_NUM){
		ASUSDEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to write data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asusdec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSDEC_I2C_ERR_TOLERANCE){
		ec_irq_disable();
		disable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
		ASUSDEC_ERR("Disable pad apwake\n");
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}
	ASUSDEC_I2C_DATA(ec_chip->i2c_data, ec_chip->i2c_err_count);
	return ret;
}

static int asusdec_intr_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->intr_i2c_data);
	if (ret < 0) {
		ASUSDEC_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}

/*Change read type HID over i2c start*/
static int asus_keyboard_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u16 len)
{
       struct i2c_msg msgs[2];
       u8 buf[2];
       int ret;

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

       ret = i2c_transfer(client->adapter, msgs, 2);
       if (ret < 0)
               return ret;

       return ret != 2 ? -EIO : 0;
}
/*Change read type HID over i2c end*/

static int BuffDataSize(void)
{
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out){
        return (in - out);
    } else {
        return ((EC_BUFF_LEN - out) + in);
    }
}

static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1)){
        ASUSDEC_ERR("Error: EC work-buf overflow \n");
        return;
    }

    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN){
        buff_in_ptr = 0;
    }
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0){
        c = (char) ec_to_host_buffer[buff_out_ptr];
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN){
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t dock_ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;
	
	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);

    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
        count--;
        i++;
		f_counter = 0;
		total_buf++;
    }

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i;
    }

    return ret;
}

static ssize_t dock_ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;
    int i;

    if (h2ec_count > 0)
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN)
    {
        return -EINVAL; /* data size is too big */
    }
	
    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        ASUSDEC_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    switch (fu_type){
    case UPDATE_BYTE_MODE:
    for (i = 0; i < count ; i++){
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
	break;
	case UPDATE_BLOCK_MODE:
    for (i = 0; i < count ; i++){
     	ec_chip->i2c_fu_data[i] = host_to_ec_buffer[i];
    }
    i2c_smbus_write_block_data(&dockram_client, 0x41, count, ec_chip->i2c_fu_data);
//    msleep(50);
    break;
	
	default:
    break;
    }
	
    h2ec_count = 0;
    return count;

}

//***********************SYSFS DOCK**************************************************************

/*Chris fix low power mode support power time sequence start*/
static ssize_t asusdec_boot_power_on_store(struct device *class,struct device_attribute *attr,char *buf)
{
	ASUSDEC_NOTICE("Enter boot_power_on\n");
	//ec_chip->boot_flag = 1;
	//schedule_work(&ec_chip->asusdec_dock_init_work);
}
/*Chris fix low power mode support power time sequence end*/

static ssize_t asusdec_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

/*Chris porting TF103CG dock ec start*/
static ssize_t asusdec_irq_data(struct device *class,struct device_attribute *attr,char *buf)
{
	//return sprintf(buf, "%d\n", ec_chip->status);
	asusdec_intr_i2c_read_data(&intr_client);
	ASUSDEC_NOTICE("0x%x 0x%x 0x%x 0x%x\n", ec_chip->intr_i2c_data[0],
	ec_chip->intr_i2c_data[1], ec_chip->intr_i2c_data[2], ec_chip->intr_i2c_data[3]);
}
/*Chris porting TF103CG dock ec end*/

static ssize_t asusdec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_version_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0)
		return sprintf(buf, "fail to get pad ec control-flag info\n");
	else{
		sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
		strcpy(buf, temp_buf);
		for (i = 1; i < 9; i++){
			sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
			strcat(buf, temp_buf);
		}
		return strlen(buf);
	}
}

static ssize_t asusdec_dock_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
        int i = 0;
        char temp_buf[64];
        int ret_val = 0;
        ret_val = asusdec_dockram_read_data(0x23);
        if (ret_val < 0)
            return sprintf(buf, "fail to get control-flag info\n");
        else{
            sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
            strcpy(buf, temp_buf);
            for (i = 1; i < 9; i++){
                sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
                strcat(buf, temp_buf);
              }
            return strlen(buf);
         }
        return sprintf(buf, "fail to get control-flag info\n");
}

static ssize_t asusdec_send_ec_req_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_send_ec_req();
	return sprintf(buf, "EC_REQ is sent\n");
}

static ssize_t asusdec_enter_factory_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_enter_factory_mode();
	return sprintf(buf, "Entering factory mode\n");
}

static ssize_t asusdec_enter_normal_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_enter_normal_mode();
	return sprintf(buf, "Entering normal mode\n");
}

static ssize_t asusdec_win_shutdown_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asusdec_win_shutdown();
	return sprintf(buf, "Win shutdown\n");
}
static ssize_t asusdec_cmd_data_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int buf_len = strlen(buf);
	int data_len = (buf_len -1)/2;
	char chr[2], data_num[data_len];
	int i=0, j=0, idx=0, ret, data_cnt;
	int cmd;
	chr[2] = '\0';

	if (ec_chip->ec_in_s3){
		asusdec_send_ec_req();
		msleep(200);
	}
	memset(&ec_chip->i2c_dm_data, 0, 32);
	printk("buf_len=%d, data_len=%d \n",buf_len, data_len);
	if(!(buf_len&0x01) || !data_len){
		return -1;
	} else if(buf_len==3){
		reg_addr = (u8) simple_strtoul (buf,NULL,16);
		return EAGAIN;
	}
	for(i=0;i<buf_len-1;i++){
		chr[j] = *(buf+i);
		if(j==1){
			if (i == 1) {
				cmd = (int) simple_strtoul (chr,NULL,16);
			} else
				data_num[idx++] = (u8) simple_strtoul (chr,NULL,16);
		}
		j++;
		if(j>1){
			j=0;
		}
	}
	data_num[idx] = '\0';
	data_cnt = data_len - 1;

	if(data_cnt > 32) {
		printk("Input data count is over length\n");
		return -1;
	}
	memcpy(&ec_chip->i2c_dm_data[1], data_num, data_cnt);
	ec_chip->i2c_dm_data[0] = data_len - 1;

	for(i=0; i<data_len; i++){
		if (i ==0) printk("I2c cmd=0x%x\n", cmd);
		printk("I2c_dm_data[%d]=0x%x\n",i, ec_chip->i2c_dm_data[i]);
	}
	ret = asusdec_dockram_write_data(cmd, data_len);
	if(ret <0)
		ASUSDEC_NOTICE("Fail to write data\n");
	return count;
}

static ssize_t asusdec_return_data_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i, ret_val;
	char temp_buf[64];

	if (reg_addr != -1) {
		if (ec_chip->ec_in_s3){
			asusdec_send_ec_req();
			msleep(200);
		}
		printk("Smbus read EC command=0x%02x\n", reg_addr);
		ret_val = asusdec_dockram_read_data(reg_addr);
		reg_addr = -1;

		if (ret_val < 0)
			return sprintf(buf, "Fail to read ec data\n");
		else{
			if (ec_chip->i2c_dm_data[0]> 32)
				return sprintf(buf, "EC return data length error\n");
			for (i = 1; i <= ec_chip->i2c_dm_data[0] ; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}
	return 0;
}

static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asusdec_status_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asusdec_info_show,NULL);
static DEVICE_ATTR(ec_version, S_IWUSR | S_IRUGO, asusdec_version_show,NULL);
static DEVICE_ATTR(ec_control_flag, S_IWUSR | S_IRUGO, asusdec_control_flag_show,NULL);
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asusdec_dock_control_flag_show,NULL);
static DEVICE_ATTR(ec_request, S_IWUSR | S_IRUGO, asusdec_send_ec_req_show,NULL);
static DEVICE_ATTR(ec_factory_mode, S_IWUSR | S_IRUGO, asusdec_enter_factory_mode_show,NULL);
static DEVICE_ATTR(ec_normal_mode, S_IWUSR | S_IRUGO, asusdec_enter_normal_mode_show,NULL);
static DEVICE_ATTR(ec_win_shutdown, S_IWUSR | S_IRUGO, asusdec_win_shutdown_show,NULL);
static DEVICE_ATTR(ec_cmd_data_send, S_IWUSR | S_IRUGO, NULL, asusdec_cmd_data_store);
static DEVICE_ATTR(ec_data_read, S_IWUSR | S_IRUGO,  asusdec_return_data_show, NULL);
/*Chris porting TF103CG dock ec start*/
static DEVICE_ATTR(ec_irq_data_read, S_IWUSR | S_IRUGO,  asusdec_irq_data, NULL);
/*Chris porting TF103CG dock ec end*/
/*Chris fix low power mode support power time sequence start*/
static DEVICE_ATTR(boot_power_on, S_IWUSR | S_IWGRP, NULL, asusdec_boot_power_on_store);
/*Chris fix low power mode support power time sequence end*/

static struct attribute *asusdec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_ec_control_flag.attr,
	&dev_attr_ec_dock_control_flag.attr,
	&dev_attr_ec_request.attr,
	&dev_attr_ec_factory_mode.attr,
	&dev_attr_ec_normal_mode.attr,
	&dev_attr_ec_win_shutdown.attr,
	&dev_attr_ec_cmd_data_send.attr,
	&dev_attr_ec_data_read.attr,
	/*Chris porting TF103CG dock ec start*/
	&dev_attr_ec_irq_data_read.attr,
	/*Chris porting TF103CG dock ec end*/
	/*Chris fix low power mode support power time sequence start*/
	&dev_attr_boot_power_on.attr,
	/*Chris fix low power mode support power time sequence end*/
NULL
};

static const struct attribute_group asusdec_smbus_group = {
	.attrs = asusdec_smbus_attributes,
};

static void asusdec_kb_init(struct i2c_client *client){
         kb_client.adapter = client->adapter;
         kb_client.addr = 0x16;
         kb_client.detected = client->detected;
         kb_client.dev = client->dev;
         kb_client.driver = client->driver;
         kb_client.flags = client->flags;
         strcpy(kb_client.name,client->name);
}

static void asusdec_tp_init(struct i2c_client *client){
	tp_client.adapter = client->adapter;
	tp_client.addr = 0x15;
	tp_client.detected = client->detected;
	tp_client.dev = client->dev;
	tp_client.driver = client->driver;
	tp_client.flags = client->flags;
	strcpy(tp_client.name,client->name);
}

static void asusdec_intr_init(struct i2c_client *client){
	intr_client.adapter = client->adapter;
	intr_client.addr = 0x19;
	intr_client.detected = client->detected;
	intr_client.dev = client->dev;
	intr_client.driver = client->driver;
	intr_client.flags = client->flags;
	strcpy(intr_client.name,client->name);
}

static void asusdec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x1b;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
	ec_chip->ec_ram_init = ASUSDEC_MAGIC_NUM;
}

static int asusdec_i2c_test(struct i2c_client *client){
	return asusdec_i2c_write_data(client, 0x0000);
}

static int asusdec_set_wakeup_cmd(void){
	int ret_val = 0;
	ASUSDEC_NOTICE("send command \n");
	if (ec_chip->dock_in){
		ret_val = asusdec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			asusdec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->dec_wakeup){
				ec_chip->i2c_dm_data[1] = 0x00;
				ec_chip->i2c_dm_data[2] = 0x00;
				ec_chip->i2c_dm_data[3] = 0x00;
				ec_chip->i2c_dm_data[4] = 0x00;
				ec_chip->i2c_dm_data[5] = 0x80;
			} else {
				ec_chip->i2c_dm_data[1] = 0x80;
			}
			asusdec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static void asusdec_reset_dock(void){
//	ec_chip->dock_init = 0;
	ASUSDEC_NOTICE("send EC_Request \n");
	/*Chris porting TF103CG dock ec start*/
	gpio_set_value(asusdec_dock_5v_enable, 0);
	msleep(20);
	gpio_set_value(asusdec_dock_5v_enable, 1);
	/*Chris porting TF103CG dock ec end*/
}

static void asusdec_additonal_porting(void){

	int ret_val = 0;
	int i = 0;

	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0){
		ASUSDEC_ERR("fail to get control flag\n");
		return;
	}
	ASUSDEC_INFO("EC RAM 1:%2x %2x %2x %2x %2x %2x %2x %2x %2x \n",ec_chip->i2c_dm_data[0],ec_chip->i2c_dm_data[1],ec_chip->i2c_dm_data[2],ec_chip->i2c_dm_data[3]
	,ec_chip->i2c_dm_data[4],ec_chip->i2c_dm_data[5],ec_chip->i2c_dm_data[6],ec_chip->i2c_dm_data[7],ec_chip->i2c_dm_data[8]);
	
	ec_chip->i2c_dm_data[0] = 0x08;
	ec_chip->i2c_dm_data[1] = 0x20;
	ec_chip->i2c_dm_data[2] = 0;
	ec_chip->i2c_dm_data[3] = 0;
	ec_chip->i2c_dm_data[4] = 0;
	ec_chip->i2c_dm_data[5] = 0;
	ec_chip->i2c_dm_data[6] = 0x20;
	ec_chip->i2c_dm_data[7] = 0;
	ec_chip->i2c_dm_data[8] = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("asusdec_SUSB_DOWN set to 0 fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("asusdec_SUSB_DOWN set to 0 ok\n");
			break;
		}
	}
	ret_val = asusdec_dockram_read_data(0x0A);
	if (ret_val < 0){
		ASUSDEC_ERR("fail to get control flag\n");
		return;
	}
	ASUSDEC_INFO("EC RAM 2:%2x %2x %2x %2x %2x %2x %2x %2x %2x \n",ec_chip->i2c_dm_data[0],ec_chip->i2c_dm_data[1],ec_chip->i2c_dm_data[2],ec_chip->i2c_dm_data[3]
	,ec_chip->i2c_dm_data[4],ec_chip->i2c_dm_data[5],ec_chip->i2c_dm_data[6],ec_chip->i2c_dm_data[7],ec_chip->i2c_dm_data[8]);
}

static int asusdec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i = 0;

	ec_chip->op_mode = 0;

	for ( i = 0; i < 10; i++){
		ret_val = asusdec_i2c_test(client);
		if (ret_val < 0)
			msleep(300);
		else
			break;
	}

	if(ret_val < 0){
		ASUSDEC_INFO("asusdec_i2c_test error %x\n", ret_val);
		goto fail_to_access_ec;
	}

	if (asusdec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);

	if (asusdec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (asusdec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	ASUSDEC_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asusdec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	ASUSDEC_NOTICE("PCBA Version: %s\n", ec_chip->ec_pcba);

	asusdec_additonal_porting();
//#if FACTORY_MODE
//	if(factory_mode == 2)
//		asusdec_enter_factory_mode();
//	else
//		asusdec_enter_normal_mode();
//#else
//		asusdec_enter_normal_mode();
//#endif

	ec_chip->status = 1;
fail_to_access_ec:
	return 0;

}

static irqreturn_t asusdec_interrupt_handler(int irq, void *dev_id){
	int gpio = asusdec_apwake_gpio;
	int ret_val = 0;
	
	ASUSDEC_INFO("irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
	if(irq == asusdec_dock_in_gpio_irq){
		schedule_work(&ec_chip->asusdec_dock_init_work);
		printk("==Receive the dock in irq==\n");
	}
	else if (irq == asusdec_apwake_gpio_irq){
		ASUSDEC_NOTICE("asusdec_apwake_gpio_irq before call irq disable\n");
		ec_irq_disable();
		ASUSDEC_NOTICE("asusdec_apwake_gpio_irq after call irq disable\n");
		ASUSDEC_NOTICE("print value gpio=%d irq=%d ec_chip->op_mode=%d\n",gpio ,irq ,ec_chip->op_mode);
		if (ec_chip->op_mode == 1){
			ASUSDEC_NOTICE("into asusdec_fw_update_work");
			schedule_work(&ec_chip->asusdec_fw_update_work);
		} else if (ec_chip->op_mode == 0){
			schedule_work(&ec_chip->asusdec_work);
		}

	}
	return IRQ_HANDLED;
}

/*Chris porting TF103CG dock ec start*/
static void asusdec_kp_sci(void){
	int ec_signal = ec_chip->intr_i2c_data[2];
	ASUSDEC_INFO("asusdec_kp_sci ec_chip->dock_status=%d\n",ec_chip->dock_status);
	if(ec_chip->dock_status == 0){
	return;
	}

	ec_chip->keypad_data.input_keycode = asusdec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		ASUSDEC_INFO("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);

		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);
	}else{
		ASUSDEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
	}
}

static int asusdec_kp_consumer_key_mapping(int x)
{
	switch (x){
		case ASUSDEC_KEYPAD_CONSUMER_BRIGHTNESSDOWN:
			return KEY_BRIGHTNESSDOWN;
		case ASUSDEC_KEYPAD_CONSUMER_BRIGHTNESSUP:
			return KEY_BRIGHTNESSUP;
		case ASUSDEC_KEYPAD_CONSUMER_EXPLORER:
			return KEY_WWW;
		case ASUSDEC_KEYPAD_CONSUMER_PREVIOUSTRACK:
			return KEY_PREVIOUSSONG;
		case ASUSDEC_KEYPAD_CONSUMER_PLAYPAUSE:
			return KEY_PLAYPAUSE;
		case ASUSDEC_KEYPAD_CONSUMER_NEXTTRACK:
			return KEY_NEXTSONG;
		case ASUSDEC_KEYPAD_CONSUMER_MUTE:
			return KEY_MUTE;
		case ASUSDEC_KEYPAD_CONSUMER_VOLUMEDOWN:
			return KEY_VOLUMEDOWN;
		case ASUSDEC_KEYPAD_CONSUMER_VOLUMEUP:
			return KEY_VOLUMEUP;
		case ASUSDEC_KEYPAD_SUSPEND:
			return KEY_SLEEP;
		default:
			printk("No consumer mapping string\n");
			return -1;
	}
}
/*Chris porting TF103CG dock ec end*/

static int asusdec_kp_key_mapping(int x)
{
	switch (x){
		case ASUSDEC_KEYPAD_ESC:
			return KEY_BACK;

		case ASUSDEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSDEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSDEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSDEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSDEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSDEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSDEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSDEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSDEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSDEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSDEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSDEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSDEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSDEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSDEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSDEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSDEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSDEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSDEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSDEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSDEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSDEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSDEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSDEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSDEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSDEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSDEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSDEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSDEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSDEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSDEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSDEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSDEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSDEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSDEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSDEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSDEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSDEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSDEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSDEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSDEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSDEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSDEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSDEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSDEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSDEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSDEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSDEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSDEC_KEYPAD_HOME:
			return KEY_HOME;

		case ASUSDEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSDEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSDEC_KEYPAD_END:
			return KEY_END;

		case ASUSDEC_KEYPAD_SCRLK:
			return KEY_SCROLLLOCK;

		case ASUSDEC_KEYPAD_NUMLK:
                        return KEY_NUMLOCK;	
	
		case ASUSDEC_KEYPAD_TPONOFF:
			return KEY_F2;

		case ASUSDEC_KEYPAD_MUTE:
                        return KEY_MUTE;
 
                case ASUSDEC_KEYPAD_VOLUMEDOWN:
                        return KEY_VOLUMEDOWN;
 
                case ASUSDEC_KEYPAD_VOLUMEUP:
                        return KEY_VOLUMEUP;

		case ASUSDEC_KEYPAD_DELETE:
			return KEY_DELETE;

		case ASUSDEC_KEYPAD_BRIGHTNESSDOWN:
			return KEY_BRIGHTNESSDOWN;

		case ASUSDEC_KEYPAD_BRIGHTNESSUP:
			return KEY_BRIGHTNESSUP;

		case ASUSDEC_KEYPAD_FLYMODE:
			return KEY_F22;

		case ASUSDEC_KEYPAD_SUSPEND:
			return KEY_SLEEP;

		case ASUSDEC_KEYPAD_PAUSE:
			return KEY_PAUSE;

		case ASUSDEC_KEYPAD_PRINTSCREEN:
			return KEY_PRINT;

		case ASUSDEC_KEYPAD_INSERT:
			return KEY_INSERT;
		/*Chris 0812 end*/
		/*Chris mark start
		//--- JP keys
		case ASUSDEC_YEN:
			return KEY_YEN;

		case ASUSDEC_RO:
			return KEY_RO;

		case ASUSDEC_MUHENKAN:
			return KEY_MUHENKAN;

		case ASUSDEC_HENKAN:
			return KEY_HENKAN;

		case ASUSDEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;
		Chris mark end*/
		//--- UK keys
		case ASUSDEC_EUROPE_2:
			return KEY_102ND;
		default:
			printk("No mapping string\n");
			return -1;
	}
}

static void asusdec_kb_report_work_function(struct work_struct *dat)
{
        int gpio = asusdec_apwake_gpio;
        int irq = gpio_to_irq(gpio);
        int ret_val = 0;
        int i = 0;
	int j = 0;
	int the_same_key = 0;
        int scancode = 0;
        
        memset(&ec_chip->i2c_kb_data, 0, 32);
	/*Change read type HID over i2c start*/        
        //ret_val = i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 11, ec_chip->i2c_kb_data);
	ret_val = asus_keyboard_i2c_read(&kb_client, 0x73, ec_chip->i2c_kb_data,11);
	/*Change read type HID over i2c end*/
        ec_irq_enable();

		ASUSDEC_INFO("kb irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
	if(ec_chip->dock_status == 0){
                return;
        }

        if(ec_chip->i2c_kb_data[0] == 0 && ec_chip->i2c_kb_data[1] == 0){//not press key
                return;
        }

        ASUSDEC_NOTICE("key code[1] : 0x%x\n",ec_chip->i2c_kb_data[0]);
        ASUSDEC_NOTICE("key code[2] : 0x%x\n",ec_chip->i2c_kb_data[1]);
        ASUSDEC_NOTICE("key code[3] : 0x%x\n",ec_chip->i2c_kb_data[2]);
        ASUSDEC_NOTICE("key code[4] : 0x%x\n",ec_chip->i2c_kb_data[3]);
        ASUSDEC_NOTICE("key code[5] : 0x%x\n",ec_chip->i2c_kb_data[4]);
        ASUSDEC_NOTICE("key code[6] : 0x%x\n",ec_chip->i2c_kb_data[5]);
	
        ec_chip->keypad_data.extend = 0;
		/*
		look kbc interface
		i2c_kb_data[2] => KBC Staus Flag
		i2c_kb_data[3] => start data
		.....
		*/
	/*Chris porting TF103CG dock ec start*/
	if( (ec_chip->i2c_kb_data[0] == 0x5 && ec_chip->i2c_kb_data[2] == 0x13) ||
	    (ec_chip->i2c_kb_data[0] == 0x4 && ec_chip->i2c_kb_data[2] == 0x14) )
	{
		ASUSDEC_NOTICE("consumer key\n");
		if(ec_chip->i2c_kb_data[3])
			input_report_key(ec_chip->indev, asusdec_kp_consumer_key_mapping(ec_chip->i2c_kb_data[3]), 1);
		else
			input_report_key(ec_chip->indev, asusdec_kp_consumer_key_mapping(ec_chip->i2c_old_kb_data[3]), 0);
	}
	else{
	/*Chris porting TF103CG dock ec end*/
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
                input_report_key(ec_chip->indev, KEY_LEFTCTRL, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
                input_report_key(ec_chip->indev, KEY_LEFTCTRL, 0);
        }
	
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
                input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
                input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 0);
        }

        if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
                input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
                input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 0);
        }

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
                input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
                input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 0);
        }
       
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTALT){
                input_report_key(ec_chip->indev, KEY_LEFTALT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTALT){
                input_report_key(ec_chip->indev, KEY_LEFTALT, 0);
        }

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
                input_report_key(ec_chip->indev, KEY_RIGHTALT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
                input_report_key(ec_chip->indev, KEY_RIGHTALT, 0);
        }
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
                input_report_key(ec_chip->indev, KEY_HOMEPAGE, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
                input_report_key(ec_chip->indev, KEY_HOMEPAGE, 0);
        }
	/*Chris porting TF103CG dock ec start*/	
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
		input_report_key(ec_chip->indev, KEY_SEARCH, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
		input_report_key(ec_chip->indev, KEY_SEARCH, 0);
	}
	}
	/*Chris porting TF103CG dock ec end*/
	for(i = 0;i < 6;i++)//normal keys
        {
                if(ec_chip->i2c_kb_data[i+5] > 0){//press key
                        ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_kb_data[i+5]);
                        ec_chip->keypad_data.value = 1;
                        ASUSDEC_INFO("keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
                        input_report_key(ec_chip->indev,
                               ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
                }else if(ec_chip->i2c_kb_data[i+5] == 0){
                       break;
                }else{
                       ASUSDEC_INFO("Unknown scancode = 0x%x\n", scancode);
                }
        }
	for(i = 0;i < 6;i++)
        {
                if(ec_chip->i2c_old_kb_data[i+5] > 0){
                        for(j = 0;j < 6;j++)//check key break
                        {
                                if(ec_chip->i2c_kb_data[j+5] == ec_chip->i2c_old_kb_data[i+5]){
                                       the_same_key = 1;
                                       break;
                                 }
                                 else
                                       the_same_key = 0;
                        }
                        if(the_same_key == 0){
                                ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_old_kb_data[i+5]);
                                input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
                        }
                }else{
                        break;
                }
        }
	for(i = 0;i < 8;i++)
        {
               ec_chip->i2c_old_kb_data[i+3] = ec_chip->i2c_kb_data[i+3];
        }
        input_sync(ec_chip->indev);
	/*Chris porting TF103CG dock ec start*/
#if 1
	if(ec_chip->suspend_state){
		printk("jjt wake system\n");
		input_report_key(ec_chip->indev, KEY_WAKEUP, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, KEY_WAKEUP, 0);
		input_sync(ec_chip->indev);
	}
#endif
	/*Chris porting TF103CG dock ec end*/
}

static void asusdec_tp_report_work_function(struct work_struct *dat) {
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	//Shouchung modify, for new touchpad driver
	u8 packet[ETP_ABS_REPORT_LENGTH];
	//Shouchung end
	int retval;
	int i = 0;

	//Shouchung modify, for new touchpad driver
	retval = i2c_master_recv(&tp_client, packet, ETP_ABS_REPORT_LENGTH);
	ec_irq_enable();
	ASUSDEC_INFO("tp irq = %d, GPIO = %d, state = %d\n", irq, gpio, gpio_get_value(gpio));
	//Shouchung modify, add condition to check if the touchpad initialized completed or not
	//Shouchung modify, modify the check packet condition
	if ((retval > 0) && (elan_i2c_check_packet(packet) > 0) &&
			(ec_chip->touchpad_member != -1)) {
	//Shouchung end
		elan_i2c_report_data(ec_chip->private, packet);
	} else {
		printk("package data: ");
		for (i = 0; i < ETP_ABS_REPORT_LENGTH; i++)
			printk("0x%x ", packet[i]);
		printk("\n");
	}
	//Shouchung end
}

//Shouchung modify, solve the touchpad control will let initialize failed problem
static int asusdec_tp_control(int arg) {
	int ret_val = 0;

	if (ec_chip->touchpad_member == -1) {
		if (arg == ASUSDEC_TP_ON) {
			ec_chip->tp_control_enable = 1;
			schedule_work(&ec_chip->asusdec_touchpad_init_work);
		} else if (arg == ASUSDEC_TP_OFF) {
			ec_chip->tp_control_enable = 0;
			schedule_work(&ec_chip->asusdec_touchpad_init_work);
		} else {
			ret_val = -ENOTTY;
		}
	} else {
		if (arg == ASUSDEC_TP_ON) {
			if (ec_chip->tp_enable == 0) {
				elan_i2c_enable(&tp_client);
				ec_chip->tp_enable = 1;
				ec_chip->tp_control_enable = 1;
			}
		} else if (arg == ASUSDEC_TP_OFF) {
			if (ec_chip->tp_enable == 1) {
				elan_i2c_disable(&tp_client);
				ec_chip->tp_enable = 0;
				ec_chip->tp_control_enable = 0;
			}
		} else {
			ret_val = -ENOTTY;
		}
	}

	return ret_val;
}
//Shouchung end

static int asusdec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0;
	u8 value = 0;
	/*Chris porting TF103CG dock ec start*/
	int gpio = asusdec_dock_5v_enable;
	const char* label = "asusdec_request";
	printk("======in asusdec_irq_ec_request======\n");

	rc = gpio_request(gpio, label);

	printk("RC = %d\n",rc);
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}
	
	rc = gpio_direction_output(gpio, 1) ;
	
	printk("RC = %d\n",rc);
	if (rc) {
		ASUSDEC_ERR("gpio_direction_output failed for output %d\n", gpio);
		goto err_output_exit;
	}
	
	printk("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	printk("GPIO = %d , state = %d\n", gpio, gpio_get_value_cansleep(gpio));
	
	printk("=======End in asusdec_irq_ec_request========\n");
	return 0 ;
	
err_output_exit:
	gpio_free(gpio);
err_exit:
	return rc;
}

static int asusdec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_dock_in_gpio;
	//set dock in irq for irq handler
	unsigned irq =asusdec_dock_in_gpio_irq=gpio_to_irq(asusdec_dock_in_gpio);
	unsigned dock_pwr_gpio = asusdec_dock_power;
	const char* label = "asusdec_dock_insert";

	printk("In %s\n",__FUNCTION__);
	
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);	
		goto err_gpio_dock_in;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));


	//request dock power
	rc = gpio_request(dock_pwr_gpio,"DOCK_PWR_EN");
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", dock_pwr_gpio);
		goto err_gpio_dock_pwr_gpio;
	}

	rc = request_irq(irq, asusdec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	
	enable_irq_wake(irq);

	ASUSDEC_INFO("==Dock in irq = %d, rc = %d\n==", irq, rc);
	
	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(dock_pwr_gpio);
err_gpio_direction_input_failed:
err_gpio_dock_pwr_gpio:
	gpio_free(gpio);
err_gpio_dock_in:
	return rc;
}

static int asusdec_irq_ec_apwake(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_apwake_gpio;
	int irq = asusdec_apwake_gpio_irq = gpio_to_irq(gpio);
	const char* label = "asusdec_apwake" ;
	
	printk("===asusdec_irq_ec_apwake===\n");

	
	rc = gpio_request(gpio, label);
	
	if (rc) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	
	if (rc) {
		ASUSDEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	
	ASUSDEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	/*Chris porting TF103CG dock ec start*/
	if( Read_HW_ID() == HARDWARE_ID_SR )
	{
		rc = request_irq(irq, asusdec_interrupt_handler,/*IRQF_TRIGGER_RISING|*/IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
		ASUSDEC_INFO("Read_HW_ID() = HARDWARE_ID_SR, IRQF_TRIGGER_FALLING\n");
	}
	else
	{
		ASUSDEC_INFO("Read_HW_ID() != HARDWARE_ID_SR, IRQF_TRIGGER_LOW\n");
		rc = request_irq(irq, asusdec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	}
	/*Chris porting TF103CG dock ec end*/
	ec_irq_disable();
	if (rc < 0) {
		ASUSDEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	/*Debug power to dock cause timing condition start*/
	ec_chip->request_apwake_irq = APWAKE_REQUEST;
	/*Debug power to dock cause timing condition end*/
	printk("==request_irq(irq, asusdec_interrupt_handler!!!==");
	ASUSDEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_input_gpio_failed :
	return rc;
}

static void asusdec_enter_s3_timer(unsigned long data){ 
	ASUSDEC_NOTICE("enter_s3_timer\n");
	schedule_work(&ec_chip->asusdec_enter_s3_work);
}

static void asusdec_send_ec_req(void){
	int rc =0;
	u8 value =0;
	ASUSDEC_NOTICE("send EC_Request\n");
	//Shouchung add to solve the touch pad issue when dock in
	//Shouchung end
	/*Chris porting TF103CG dock ec start*/
	gpio_set_value(asusdec_dock_5v_enable, 0);
	ASUSDEC_INFO("jjt 1 GPIO = %d , state = %d\n", asusdec_dock_5v_enable, gpio_get_value(asusdec_dock_5v_enable));
	msleep(DELAY_TIME_MS);
	gpio_set_value(asusdec_dock_5v_enable, 1);
	ASUSDEC_INFO("jjt 2 GPIO = %d , state = %d\n", asusdec_dock_5v_enable, gpio_get_value(asusdec_dock_5v_enable));
	
	//Shouchung add to solve the touch pad issue when dock in
	msleep(100);
	/*Chris porting TF103CG dock ec end*/
	//Shouchung end
}

static void asusdec_smi(void){
	if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_HANDSHAKING){
		ASUSDEC_NOTICE("ASUSDEC_SMI_HANDSHAKING\n");
		if(ec_chip->status == 0){
			asusdec_chip_init(ec_chip->client);
		}
		ec_chip->ec_in_s3 = 0;
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_RESET){
		ASUSDEC_NOTICE("ASUSDEC_SMI_RESET\n");
		schedule_work(&ec_chip->asusdec_dock_init_work);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SMI_WAKE){
		ASUSDEC_NOTICE("ASUSDEC_SMI_WAKE\n");
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_S5){
		ASUSDEC_NOTICE("APOWER_POWEROFF\n");
		asusdec_switch_apower_state(APOWER_POWEROFF);
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_NOTIFY_SHUTDOWN){
		ASUSDEC_NOTICE("APOWER_NOTIFY_SHUTDOWN\n");
		asusdec_switch_apower_state(APOWER_NOTIFY_SHUTDOWN);
	} else if (ec_chip->intr_i2c_data[2] == APOWER_SMI_RESUME){
		ASUSDEC_NOTICE("APOWER_SMI_RESUME\n");
		asusdec_switch_apower_state(APOWER_RESUME);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_EC_WAKEUP){        
		ASUSDEC_NOTICE("ASUSDEC_SxI_EC_WAKEUP \n");
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_BOOTBLOCK_RESET){ 
		ASUSDEC_NOTICE("ASUSDEC_SxI_BOOTBLOCK_RESET \n");
		schedule_work(&ec_chip->asusdec_dock_init_work);
	}else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_WATCHDOG_RESET){
		ASUSDEC_NOTICE("ASUSDEC_SxI_WATCHDOG_RESET \n");
		schedule_work(&ec_chip->asusdec_dock_init_work);
	}else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_ADAPTER_CHANGE){
		ASUSDEC_NOTICE("ASUSDEC_SxI_ADAPTER_CHANGE \n");
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_DOCK_INSERT){    
		ASUSDEC_NOTICE("ASUSDEC_SxI_DOCK_INSERT\n");
		schedule_work(&ec_chip->asusdec_dock_init_work);
	}else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_DOCK_REMOVE){     
		ASUSDEC_NOTICE("ASUSDEC_SxI_DOCK_REMOVE\n");
		schedule_work(&ec_chip->asusdec_dock_init_work);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_PAD_BL_CHANGE){ 
		ASUSDEC_NOTICE("ASUSDEC_SxI_PAD_BL_CHANGE \n");
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_HID_Status_Changed){
		ASUSDEC_NOTICE("ASUSDEC_SxI_HID_Status_Changed \n");
		schedule_work(&ec_chip->asusdec_touchpad_init_work);
	} else if (ec_chip->intr_i2c_data[2] == ASUSDEC_SxI_HID_WakeUp){      
		ASUSDEC_NOTICE("ASUSDEC_SxI_HID_WakeUp \n");
	}

}
static void asusdec_enter_s3_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	if (ec_chip->op_mode){
		ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		mutex_unlock(&ec_chip->state_change_lock);
		return ;
	}

	ec_chip->ec_in_s3 = 1;
	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Send s3 command fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asusdec_stresstest_work_function(struct work_struct *dat)
{
	asusdec_i2c_read_data(ec_chip->client);
	queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
}

static void asusdec_dock_status_report(void){
	ASUSDEC_NOTICE("dock_in = %d, ec_chip->dock_type = %d \n", ec_chip->dock_in, ec_chip->dock_type);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_type]);
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);
	for ( i = 0; i < 255; i++)
		set_bit(i,dev->keybit);
	//Shouchung add for touch pad
	set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
	set_bit(REL_Y, dev->relbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);
	set_bit(EV_SYN, dev->evbit);
	//Shouchung end
}

static int asusdec_input_device_create(struct i2c_client *client){
	int err = 0;
	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		ASUSDEC_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->indev->name = "asuspec";
	ec_chip->indev->phys = "/dev/input/asuspec";
	asusdec_keypad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		ASUSDEC_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;
}

static void asusdec_dock_init_work_function(struct work_struct *dat)
{
	int gpio = asusdec_dock_in_gpio;
	int gpio_state = 0;
	int ret_val;
	/*Chris fix low power mode support power time sequence start*/
	printk("entry_mode = %d\n",entry_mode);
	if ( entry_mode == 4 )
	return ;
	/*Chris fix low power mode support power time sequence end*/
	wake_lock(&ec_chip->wake_lock_init);

	gpio_state = gpio_get_value(gpio);

	printk("dock in gpio state = %d\n",gpio_state);
	if (gpio_state){
		ASUSDEC_NOTICE("No dock detected\n");
		/*Debug power to dock cause timing condition start*/
		if(ec_chip->request_apwake_irq)
			asusdec_irq_ec_apwake(ec_chip->client);
		/*Debug power to dock cause timing condition end*/
		ec_irq_disable();
		ec_chip->status = 0;
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->dock_status = 0;
//		ec_chip->dock_init = 0;
		ec_chip->dock_type = DOCK_UNKNOWN;
		ec_chip->touchpad_member = -1;
//		memset(ec_chip->dec_model_name, 0, 32);
//		memset(ec_chip->dec_version, 0, 32);
		/*Chris start*/
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}
		/*Chris end*/
		if (ec_chip->private->input){
			input_unregister_device(ec_chip->private->input);
			ec_chip->private->input = NULL;
		}
		asusdec_dock_status_report();
		//if(gpio_get_value(asusdec_dock_power) == 1) 
		gpio_direction_output(asusdec_dock_power, 0) ;
		gpio_direction_output(dock_ec_pwr_en , 0);
		ASUSDEC_NOTICE("asusdec_dock_power=%d, dock_ec_pwr_en=%d\n",gpio_get_value(asusdec_dock_power), gpio_get_value(dock_ec_pwr_en));
	}else{
		ASUSDEC_NOTICE("Dock-in detected\n");
		ASUSDEC_NOTICE("gpio_get_value(asusdec_dock_power)=%d\n",gpio_get_value(asusdec_dock_power));
		//dock_pwr_en start
		gpio_direction_output(dock_ec_pwr_en , 1);
		gpio_direction_output(asusdec_dock_power, 1) ;
		msleep(200);
		/*Debug power to dock cause timing condition start*/
		if(ec_chip->request_apwake_irq)
			asusdec_irq_ec_apwake(ec_chip->client);
		/*Debug power to dock cause timing condition end*/
		ASUSDEC_NOTICE("asusdec_dock_power=%d, dock_ec_pwr_en=%d\n",gpio_get_value(asusdec_dock_power), gpio_get_value(dock_ec_pwr_en));
		//dock_pwr_en end
		//asusdec_send_ec_req();
		//msleep(500);
		if(ec_chip->status == 0)
		asusdec_chip_init(ec_chip->client);
		/*Chris start*/
		if (&kb_client != NULL){
			asusdec_input_device_create(&kb_client);
			printk("asusdec_input_device_create in\n");
		}
		printk("asusdec_input_device_create out\n");
		/*Power on/off TP/USB of dock ec start*/
		ret_val = asusdec_susb_control(ASUSDEC_USBPOWER_ON);
		/*Power on/off TP/USB of dock ec end*/
		memset(&ec_chip->i2c_kb_data, 0, 32);
		ec_chip->i2c_kb_data[0] = 0x00;
		ec_chip->i2c_kb_data[1] = 0x00;
		ec_chip->i2c_kb_data[2] = 0x08;
		i2c_smbus_write_i2c_block_data(&kb_client, 0x75, 3, ec_chip->i2c_kb_data);
		msleep(50);//FIXME:will use retry
		/*Change read type HID over i2c start*/
		//i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 11, ec_chip->i2c_kb_data);
		ret_val = asus_keyboard_i2c_read(&kb_client, 0x73, ec_chip->i2c_kb_data,11);
		/*Change read type HID over i2c end*/
		/*Chris end*/
		msleep(50);//FIXME:use retry
		schedule_work(&ec_chip->asusdec_touchpad_init_work);
		ec_chip->dock_type = MOBILE_DOCK;
		ec_chip->dock_in = 1;
		ec_chip->init_success = 1;
		ec_chip->dock_status = 1;
		asusdec_dock_status_report();
		ec_irq_enable();
	}
exit:
	ASUSDEC_NOTICE("exit! \n");
	wake_unlock(&ec_chip->wake_lock_init);
	return ;

fail_to_access_ec:
	if (gpio_state){
		ASUSDEC_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		ASUSDEC_NOTICE("Need EC FW update\n");
	}
	goto exit;
}

static void asusdec_touchpad_init_work_function(struct work_struct *dat)
{
	int ret_val = 0;

	/*read the Host Control Flags*/
	ret_val = asusdec_dockram_read_data(0x0A);
	//Shouchung modify, modify the condition to let touchpad initial when HID status = 1
	if ((ret_val >= 0) && (ec_chip->i2c_dm_data[3] & ASUSDEC_HID_STATUS)) {
	//Shouchung end
		//Shouchung modify, solve the touchpad control will let initialize failed problem
		if (ec_chip->touchpad_member == ELANTOUCHPAD) return;
		if(!elan_i2c_initialize(&tp_client)){
			ec_chip->private->client = &tp_client;
			ret_val = elan_i2c_input_dev_create(ec_chip->private);
				if (ret_val < 0) ASUSDEC_NOTICE("fail to creat input for touchpad\n");
			//Shouchung modify, move the initialize completed flag after input device creation
			ec_chip->touchpad_member = ELANTOUCHPAD;
			//Shouchung end
			if (ec_chip->tp_control_enable == 0) {
				elan_i2c_disable(&tp_client);
				ec_chip->tp_enable = 0;
			} else {
				ec_chip->tp_enable = 1;
			}
		} else {
			ASUSDEC_NOTICE("fail to init touchpad\n");
		}
	} else {
		ASUSDEC_NOTICE("HID is not ready yet\n");
	}
	//Shouchung end
	return ;
}

static void asusdec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int i = 0;
	int ret = 0;

	mutex_lock(&ec_chip->lock);

        switch (fu_type){
        case UPDATE_BYTE_MODE:
    	    smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
    	    ec_irq_enable();
    	    BuffPush(smbus_data);
            break;
        case UPDATE_BLOCK_MODE:
    	    ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x6a, 8/*32*/, ec_chip->i2c_fu_data);
    	    if (ret < 0)
    	    	ASUSDEC_ERR("fu work Fail to read data, status %02x\n", ret);
    	    for (i = 0; i < ec_chip->i2c_fu_data[0] + 2 ; i++){
    	    	BuffPush(ec_chip->i2c_fu_data[i]);//FIXME:only need push fa
    	    }
            ec_irq_enable();
            break;

        default:
            break;
        }
	mutex_unlock(&ec_chip->lock);
}

static void asusdec_work_function(struct work_struct *dat)
{
	int gpio = asusdec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;
	ret_val = asusdec_intr_i2c_read_data(&intr_client);
	ASUSDEC_NOTICE("=======0x%x 0x%x 0x%x 0x%x=======ap_wake =%d\n", ec_chip->intr_i2c_data[0],
                 ec_chip->intr_i2c_data[1], ec_chip->intr_i2c_data[2], ec_chip->intr_i2c_data[3], gpio_get_value(asusdec_apwake_gpio));
	if (ret_val < 0){
		printk("asusdec_work_function ret_val < 0!!\n");
		ec_irq_enable();
		return ;
	}	
	if((ec_chip->intr_i2c_data[0] == 0x3)&&(ec_chip->intr_i2c_data[1] == 0xc1)&&(ec_chip->intr_i2c_data[2] == ASUSDEC_KEY_MASK)){
		/*Chris porting TF103CG dock ec start*/
		if(ec_chip->intr_i2c_data[3] == 0x11 || ec_chip->intr_i2c_data[3] == 0x13
		|| ec_chip->intr_i2c_data[3] == 0x14)
		{
		/*Chris porting TF103CG dock ec end*/
			schedule_work(&ec_chip->asusdec_kb_report_work);
		}else if(ec_chip->intr_i2c_data[3] == 0x1){
			schedule_work(&ec_chip->asusdec_tp_report_work);
		}
		return ;
	}
	ec_irq_enable();
		if (ec_chip->intr_i2c_data[1] & ASUSDEC_SMI_MASK){
			asusdec_smi();
			ASUSDEC_INFO("asusdec_smi irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
			return ;
		}
		if(ec_chip->intr_i2c_data[1] == ASUSDEC_SCI_MASK){
			ASUSDEC_INFO("ec_chip->i2c_data[1] == ASUSDEC_SCI_MASK\n");
			if(ec_chip->i2c_data[2] >= 0 && ec_chip->i2c_data[2] < 25)
				asusdec_kp_sci();
			ASUSDEC_INFO("asusdec_sci irq = %d GPIO = %d , state = %d\n", irq, gpio, gpio_get_value(gpio));
			return;
		}
}

static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", APOWER_SDEV_NAME);
}

static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->apower_state);
}

static int asusdec_open(struct inode *inode, struct file *flip){
	ASUSDEC_NOTICE("\n");
	return 0;
}
static int asusdec_release(struct inode *inode, struct file *flip){
	ASUSDEC_NOTICE("\n");
	return 0;
}
static long asuspec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char name_buf[64];
	int length = 0;
	char *envp[3];
	int env_offset = 0;

	if (_IOC_TYPE(cmd) != ASUSDEC_IOC_MAGIC)
	 return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSDEC_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd) {
		case ASUSDEC_POLLING_DATA:
			if (arg == ASUSDEC_IOCTL_HEAVY){
				ASUSDEC_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSDEC_IOCTL_NORMAL){
				ASUSDEC_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(asusdec_wq, &asusdec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSDEC_IOCTL_END){
				ASUSDEC_NOTICE("polling end\n");
		    	cancel_delayed_work_sync(&asusdec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSDEC_FW_UPDATE:
			ASUSDEC_NOTICE("ASUSDEC_FW_UPDATE\n");
			mutex_lock(&ec_chip->state_change_lock);
			//asusdec_send_ec_req();
			msleep(200);
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			h2ec_count = 0;
			memset(host_to_ec_buffer, 0, EC_BUFF_LEN);
			memset(ec_to_host_buffer, 0, EC_BUFF_LEN);
			memset(&ec_chip->i2c_dm_data, 0, 32);
			ec_chip->status = 0;
			ec_chip->op_mode = 1;
			//Shouchung add, solve the touchpad can not initialize after firmware update
			ec_chip->touchpad_member = -1;
			//Shouchung end
			wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			msleep(2400);
			switch(arg){
			case 0:
				ASUSDEC_ERR("ASUSDEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				schedule_work(&ec_chip->asusdec_dock_init_work);
				/*mars*/
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				return -ENOTTY;
				break;
			case 1:
				if (ec_chip->dock_in){
				ASUSDEC_NOTICE("ASUSDEC_FW_UPDATE use block mode\n");
				fu_block_mode = 1;
				fu_type = UPDATE_BLOCK_MODE;
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				msleep(2500);
				} else {
					ASUSDEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;

			default:
				ASUSDEC_ERR("error fu type!\n");
				break;
			}
			msleep(1000);
			mutex_unlock(&ec_chip->state_change_lock);
			break;
		case ASUSDEC_INIT:
			ASUSDEC_NOTICE("ASUSDEC_INIT\n");
			msleep(500);
			ec_chip->status = 0;
			ec_chip->op_mode = 0;
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			switch(fu_type){
			case UPDATE_BYTE_MODE:
			case UPDATE_BLOCK_MODE:
				schedule_work(&ec_chip->asusdec_dock_init_work);
				/*mars*/
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				ASUSDEC_NOTICE("ASUSDEC_INIT - EC version: %s\n", ec_chip->ec_version);
				length = strlen(ec_chip->ec_version);
				ec_chip->ec_version[length] = NULL;
				snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
				envp[env_offset++] = name_buf;
				envp[env_offset] = NULL;
				kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
				break;
			default:
				ASUSDEC_ERR("ASUSDEC_INIT unknow case!\n");
				break;
			}
			msleep(2500);
			break;
		case ASUSDEC_TP_CONTROL:
		       ASUSDEC_NOTICE("ASUSDEC_TP_CONTROL\n");
                     if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
                         //err = asusdec_tp_control(arg);
                         //err = asusdec_tp_control2(arg);
                         asusdec_tp_control(arg);//jj
                         return err;
                        }
		       else
			    return -ENOTTY;
		case ASUSDEC_EC_WAKEUP:
			msleep(500);
			ASUSDEC_NOTICE("ASUSDEC_EC_WAKEUP, arg = %s \n", arg?"ASUSDEC_EC_ON":"ASUSDEC_EC_OFF");
			if (arg == ASUSDEC_EC_OFF){
				ec_chip->dec_wakeup = 0;
				ASUSDEC_NOTICE("Set EC shutdown when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}
			else if (arg == ASUSDEC_EC_ON){
				ec_chip->dec_wakeup = 1;
				ASUSDEC_NOTICE("Keep EC active when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}else{
				ASUSDEC_ERR("Unknown argument");
				return -ENOTTY;
			}
		case ASUSDEC_FW_DUMMY:
			ASUSDEC_NOTICE("ASUSDEC_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;

			switch(fu_type){
			case UPDATE_BYTE_MODE:
				ASUSDEC_ERR("dont support byte mode\n");
				break;
			case UPDATE_BLOCK_MODE:
				if (ec_chip->dock_in){
				ASUSDEC_NOTICE("ASUSDEC_dock FW_UPDATE use block mode\n");
				fu_block_mode = 1;
				ASUSDEC_NOTICE("ASUSDEC_FW_DUMMY pad block mode\n");
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				msleep(500);
				} else {
					ASUSDEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			default:
				ASUSDEC_ERR("error fu type!\n");
				break;
			}
			if(arg == 1){
			}
			else{
				ASUSDEC_ERR("ASUSDEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				schedule_work(&ec_chip->asusdec_dock_init_work);
				/*mars*/
				switch_set_state(&ec_chip->dock_sdev, !ec_chip->dock_sdev.state);
				msleep(2500);
				return -ENOTTY;
			}
			break;
		case ASUSDEC_WIN_SHUTDOWN:
			ASUSDEC_NOTICE("ASUSDEC_WIN_SHUTDOWN\n", arg);
			asusdec_win_shutdown();
			break;
		default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return 0;
}

static void asusdec_switch_apower_state(int state){
	ec_chip->apower_state = state;
	
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->apower_state = APOWER_IDLE;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
}

static void asusdec_win_shutdown(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asusdec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x40;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Win shutdown command fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("Win shutdown\n");
			break;
		}
	}
}

static void asusdec_enter_factory_mode(void){

	ASUSDEC_NOTICE("Entering factory mode\n");
	asusdec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
	asusdec_dockram_write_data(0x0A,9);
}

static void asusdec_enter_normal_mode(void){

	int ret_val = 0;
	int i = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSDEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSDEC_ERR("Entering normal mode fail\n");
			msleep(100);
		}
		else {
			ASUSDEC_NOTICE("Entering normal mode\n");
			break;
		}
	}
}

static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", switch_value[ec_chip->dock_type]);
}

struct file_operations asusdec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asuspec_ioctl,
	.open = asusdec_open,
	.write = dock_ec_write,
	.read = dock_ec_read,
	.release = asusdec_release,
};

static const struct i2c_device_id asusdec_id[] = {
	{"asusdec", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, asusdec_id);

static struct i2c_driver asusdec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asusdec",
		.owner = THIS_MODULE,
	},
	.probe	 = asusdec_probe,
	.remove	 = asusdec_remove,
	.suspend = asusdec_suspend,
	.resume = asusdec_resume,
	.id_table = asusdec_id,
};

static int asusdec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	int rc =0;
	u8 value =0;
	u32 testvalue = 0;
	
	ASUSDEC_INFO("asusdec probe!!!\n");
	
	asusdec_apwake_gpio_irq = gpio_to_irq(asusdec_apwake_gpio);
	int dock_pwr_gpio = asusdec_dock_power;
	
	printk("GPIO asusdec_dock_in_gpio = %d\n", asusdec_dock_in_gpio);
	printk("GPIO asusdec_apwake_gpio = %d\n", asusdec_apwake_gpio);
	printk("GPIO asusdec_apwake_gpio_irq = %d\n", asusdec_apwake_gpio_irq);
	printk("GPIO asusdec_dock_power = %d\n", asusdec_dock_power);

	asusdec_irq_dock_in(client);

	ASUSDEC_INFO("GPIO = %d , state = %d\n", dock_pwr_gpio, gpio_get_value(dock_pwr_gpio));	

	/*Chris porting TF103CG dock ec start*/	
	err = gpio_request(dock_ec_pwr_en,"DOCK_EC_PWR_EN");
	gpio_direction_output(asusdec_dock_power, 0) ;
	gpio_direction_output(dock_ec_pwr_en, 0) ;
	if (err) {
		ASUSDEC_ERR("gpio_request failed for input %d\n", dock_pwr_gpio);
		goto fail_gpio_request;
	}
	ASUSDEC_INFO("GPIO = %d , state = %d\n", dock_pwr_gpio, gpio_get_value(dock_pwr_gpio));
	/*Chris porting TF103CG dock ec end*/
	
	err = sysfs_create_group(&client->dev.kobj, &asusdec_smbus_group);
	if (err) {
		ASUSDEC_ERR("Unable to create the sysfs\n");
		goto fail_sysfs_create;
	}

	ec_chip = kzalloc(sizeof (struct asusdec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSDEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto fail_allocate_ec_chip;
	}
	ec_chip->private = kzalloc(sizeof(struct elan_i2c_data), GFP_KERNEL);
	if (!ec_chip->private) {
		ASUSDEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto fail_allocate_private;
	}

	i2c_set_clientdata(client, ec_chip);
	
#if 1
	device_init_wakeup(&client->dev, 1);
#endif
	
	ec_chip->client = client;
	ec_chip->client->driver = &asusdec_driver;
	ec_chip->client->flags = 1;
	init_timer(&ec_chip->asusdec_timer);
	asusdec_kb_init(client);
	asusdec_tp_init(client);
	asusdec_intr_init(client);
	ec_chip->asusdec_timer.function = asusdec_enter_s3_timer;
	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asusdec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusdec_wake_init");
	mutex_init(&ec_chip->lock);
	spin_lock_init(&ec_chip->irq_lock);
	mutex_init(&ec_chip->state_change_lock);
	//Shouchung add to solve the touch pad issue when dock in
	mutex_init(&ec_chip->tp_lock);
	//Shouchung end
	mutex_init(&ec_chip->dock_init_lock);
	ec_chip->indev = NULL;
	ec_chip->lid_indev = NULL;
	ec_chip->private->input = NULL;
	ec_chip->dock_status = 0;
//	ec_chip->dock_init = 0;	
	ec_chip->ec_ram_init = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;
	ec_chip->dock_type = DOCK_UNKNOWN;
	/*Power on/off TP/USB of dock ec start*/
	ec_chip->suspend_state = 0;
	/*Power on/off TP/USB of dock ec end*/
//	ec_chip->kb_and_ps2_enable = 0;
	//Shouchung add, solve the touchpad control will let initialize failed problem
	ec_chip->touchpad_member = -1;
	ec_chip->tp_control_enable = 1;
	//Shouchung end
	//Shouchung modify for enable touchpad when dock-in
	ec_chip->tp_enable = 0;
	//Shouchung end
	/*Chris fix low power mode support power time sequence start*/
	ec_chip->boot_flag = 0;
	/*Chris fix low power mode support power time sequence end*/
	/*Debug power to dock cause timing condition start*/
	ec_chip->request_apwake_irq = APWAKE_NOT_REQUEST;
	/*Debug power to dock cause timing condition end*/
	asusdec_dockram_init(client);
	err = cdev_add(asusdec_cdev,asusdec_dev,1) ;
	if(err)
	{
		ASUSDEC_ERR("cdev_add failed (err: %d)", err);
		goto fail_cdev_add;
	}

	ec_chip->apower_sdev.name = APOWER_SDEV_NAME;
	ec_chip->apower_sdev.print_name = apower_switch_name;
	ec_chip->apower_sdev.print_state = apower_switch_state;
	ec_chip->apower_state = 0;
	if(switch_dev_register(&ec_chip->apower_sdev) < 0){
		ASUSDEC_ERR("switch_dev_register for apower failed!\n");
		goto fail_register_apower_sdev;
	}
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = asusdec_switch_name;
	ec_chip->dock_sdev.print_state = asusdec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		ASUSDEC_ERR("switch_dev_register for dock failed!\n");
		goto fail_register_dock_sdev;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);

	//create work queue for all work
	asusdec_wq = create_singlethread_workqueue("asusdec_wq");
	if(!asusdec_wq)
	{
		ASUSDEC_ERR("Unable to create workqueue\n");
		goto fail_create_wq;
	}
	
	INIT_WORK(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
	INIT_WORK(&ec_chip->asusdec_touchpad_init_work, asusdec_touchpad_init_work_function);
	INIT_WORK(&ec_chip->asusdec_work, asusdec_work_function);
	printk("before init work function!!!\n");
	asusdec_irq_ec_request(client);
	//asusdec_irq_ec_apwake(client);
	printk("end init work function!!!\n");

	INIT_WORK(&ec_chip->asusdec_fw_update_work, asusdec_fw_update_work_function);
	INIT_WORK(&ec_chip->asusdec_enter_s3_work, asusdec_enter_s3_work_function);
	INIT_DELAYED_WORK(&asusdec_stress_work, asusdec_stresstest_work_function);
	INIT_WORK(&ec_chip->asusdec_kb_report_work, asusdec_kb_report_work_function);
	INIT_WORK(&ec_chip->asusdec_tp_report_work, asusdec_tp_report_work_function);
	
	schedule_work(&ec_chip->asusdec_dock_init_work);

	return 0;

fail_create_wq:
	switch_dev_unregister(&ec_chip->dock_sdev);
fail_register_dock_sdev:
	switch_dev_unregister(&ec_chip->apower_sdev);
fail_register_apower_sdev:
	cdev_del(asusdec_cdev);
fail_cdev_add:
	kfree(ec_chip->private);
fail_allocate_private:
	kfree(ec_chip);
fail_allocate_ec_chip:
	sysfs_remove_group(&client->dev.kobj, &asusdec_smbus_group);
fail_sysfs_create:
	gpio_free(dock_ec_pwr_en);
fail_gpio_request:
	return err;
}

/*Power on/off TP/USB of dock ec start*/
int asusdec_susb_control(int arg)
{
	int ret_val = 0;
	memset(&ec_chip->i2c_dm_data, 0, 32);
	ec_chip->i2c_dm_data[0] = 8;
	if (arg == ASUSDEC_USBPOWER_OFF)
	{
		//SUSB-Down = 1, Device-on = 0
		ec_chip->i2c_dm_data[2] = 0x20; //Device-on = 0
		ec_chip->i2c_dm_data[3] = 0x40; //USB_5V_EN = 0
		ec_chip->i2c_dm_data[5] = 0x22; //Host in S3 = 1; SUSB-Down = 1

		ASUSDEC_NOTICE("Trun Off USB Power\n");
		ret_val = 0;
	}
	else if(arg == ASUSDEC_USBPOWER_ON)
	{
		//SUSB-Down = 0, Device-on = 1
		ec_chip->i2c_dm_data[1] = 0x20; //SUSB-Down = 0
		ec_chip->i2c_dm_data[6] = 0x20; //Device-on = 1
		ec_chip->i2c_dm_data[7] = 0x40; //USB_5V_EN = 1
		ASUSDEC_NOTICE("Trun ON USB Power\n");
		ret_val = 0;
	}
	else
		ret_val = -ENOTTY;
	asusdec_dockram_write_data(0x0A,9);
	return ret_val;
}
/*Power on/off TP/USB of dock ec end*/

static int asusdec_remove(struct i2c_client *client)
{
	struct asusdec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	destroy_workqueue(asusdec_wq);
	switch_dev_unregister(&ec_chip->dock_sdev);
	switch_dev_unregister(&ec_chip->apower_sdev);
	cdev_del(asusdec_cdev);
	//Shouchung add, unregister touchpad's input device when remove driver
	if (ec_chip->private->input != NULL) {
		input_unregister_device(ec_chip->private->input);
		ec_chip->private->input = NULL;
	}
	//Shouchung end
	kfree(ec_chip->private);
	input_unregister_device(chip->indev);
	sysfs_remove_group(&client->dev.kobj, &asusdec_smbus_group);
	gpio_free(asusdec_dock_power);
	kfree(chip);
	return 0;
}

static int asusdec_suspend(struct i2c_client *client, pm_message_t mesg){
	int ret_val;
	printk("asusdec_suspend+++\n");
	if (ec_chip->dock_in && (ec_chip->suspend_state == 0))
	{
		flush_workqueue(asusdec_wq);
		ec_chip->suspend_state = 1;
		ec_chip->init_success = 0;
		ec_chip->touchpad_member = -1;
		ec_chip->ec_in_s3 = 1;
		if (ec_chip->op_mode)
			ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		else
			elan_i2c_disable(&tp_client);
		ret_val = asusdec_susb_control(ASUSDEC_USBPOWER_OFF);
		if (ret_val < 0)
			ASUSDEC_NOTICE("Turn Off USB Power Failed\n");		
	}
	
	/*if (device_may_wakeup(&client->dev)){
		printk("jjt dec suspend");
		ec_irq_enable();
		enable_irq_wake(gpio_to_irq(asusdec_apwake_gpio));
	}*/
	printk("asusdec_suspend---\n");
	return 0;
}

static int asusdec_resume(struct i2c_client *client){
	int ret_val;
	printk("asusdec_resume+++\n");
	if (ec_chip->dock_in && ec_chip->suspend_state && gpio_get_value(asusdec_apwake_gpio)) 
	{
		ec_chip->dock_type = DOCK_UNKNOWN;
		//asusdec_reset_dock();
		msleep(70);
		ec_chip->suspend_state = 0;
		ec_chip->init_success = 0;
		ec_chip->ec_in_s3 = 0;
		ec_chip->touchpad_member = -1;
		ret_val = asusdec_susb_control(ASUSDEC_USBPOWER_ON);
		if (ret_val < 0)
			ASUSDEC_NOTICE("Turn ON USB Power Failed\n");
		if (ec_chip->op_mode)
			ASUSDEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		else
			schedule_work(&ec_chip->asusdec_dock_init_work);
		ec_chip->i2c_err_count = 0;
	}
	printk("asusdec_resume-\n");
	return 0;
}

static int __init asusdec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####\n", __func__);

	if (asusdec_major) {
		asusdec_dev = MKDEV(asusdec_major, asusdec_minor);
		err_code = register_chrdev_region(asusdec_dev, 1, "asuspec");
		if (err_code)
		{
			ASUSDEC_ERR("Unable to register chrdev\n");
			goto fail_register_chrdev;
		}
	} else {
		err_code = alloc_chrdev_region(&asusdec_dev, asusdec_minor, 1,"asuspec");
		if (err_code)
		{
			ASUSDEC_ERR("Unable to alloc chrdev\n");
			goto fail_register_chrdev;
		}
		asusdec_major = MAJOR(asusdec_dev);
	}

	ASUSDEC_NOTICE("cdev_alloc\n") ;
	asusdec_cdev = cdev_alloc() ;
	asusdec_cdev->owner = THIS_MODULE ;
	asusdec_cdev->ops = &asusdec_fops ;

	err_code=i2c_add_driver(&asusdec_driver);
	if(err_code){
		ASUSDEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asusdec_class = class_create(THIS_MODULE, "asuspec");
	if(asusdec_class <= 0){
		ASUSDEC_ERR("asusdec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asusdec_device = device_create(asusdec_class, NULL, MKDEV(asusdec_major, asusdec_minor), NULL, "asuspec" );
	if(asusdec_device <= 0){
		ASUSDEC_ERR("asusdec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	ASUSDEC_INFO("return value %d\n", err_code) ;
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;

device_create_fail :
	class_destroy(asusdec_class) ;
class_create_fail :
	i2c_del_driver(&asusdec_driver);
i2c_add_driver_fail :
	unregister_chrdev_region(MKDEV(asusdec_major, asusdec_minor), 1);
	printk(KERN_INFO "%s- #####\n", __func__);
fail_register_chrdev:
	return err_code;
}

static void __exit asusdec_exit(void)
{
	device_destroy(asusdec_class,MKDEV(asusdec_major, asusdec_minor)) ;
	class_destroy(asusdec_class) ;
	i2c_del_driver(&asusdec_driver);
	unregister_chrdev_region(asusdec_dev, 1);
}

module_init(asusdec_init);
module_exit(asusdec_exit);
