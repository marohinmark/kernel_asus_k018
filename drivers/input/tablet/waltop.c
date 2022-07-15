/* 
 * Source for : Waltop ASIC5 pen touch controller.
 * drivers/input/tablet/waltop_I2C.c
 * 
 * Copyright (C) 2008-2013	Waltop International Corp. <waltopRD@waltop.com.tw>
 * 
 * History:
 * Copyright (c) 2011	Martin Chen <MartinChen@waltop.com.tw>
 * Copyright (c) 2012	Taylor Chuang <chuang.pochieh@gmail.com>
 * Copyright (c) 2012	Herman Han <HermanHan@waltop.com>
 * * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */

#include <linux/unistd.h>  
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/platform_data/waltop_device.h>
#include <linux/earlysuspend.h>
#include <linux/switch.h>
#include <linux/HWVersion.h>
#include <linux/time.h>
#include <linux/lnw_gpio.h>

/*****************************************************************************
 * MACRO definitions and structure
 ****************************************************************************/
//#define PEN_ASIC_X6_VERSION	// now X7 version, this is for X6 version
#define REMAP_TO_LCD_SIZE     // mapping to screen resolution
#define IAP_FWUPDATE			// firmware update code include
#define ANDROID_DIGITIZER_SYS_FS // support sysfs for path(/sys/android_digitizer)
#define I2C_CHECK_CODE	1

#ifdef IAP_FWUPDATE
#include <linux/wakelock.h>
#include <linux/wait.h>
#include <linux/sysfs.h>
//
#define FW_IODATA_SIZE		8	// 8 bytes data
#define FW_IOBUFFER_SIZE	16	// 10 bytes I2C packet
#define FW_BLOCK_SIZE		128	// 128 bytes buffer for user ap
#define FW_RESET_DELAY_A2B	30	/* Delay 30 ms from A to B */
#define FW_RESET_DELAY_B2C	50	/* Delay 50 ms from B to C */
#define FW_RESET_DELAY_CP	5	/* Delay 5 ms after C */
#define USE_WAKELOCK

#endif

#ifdef	REMAP_TO_LCD_SIZE
/* Screen characteristics */
#define LCD_SCREEN_MAX_X		   1920
#define LCD_SCREEN_MAX_Y		   1080
#endif

#define CONFIG_HAS_WALTOP_EARLYSUSPEND
#define IC_RESET_LOW        0
#define IC_RESET_HIGH       1

#define KLOG_NAME               "[Waltop]"
#define PEN_DELAY HZ/10
#define NAME_PEN_INSERT "pen_insert"

static unsigned int HW_ID = 0xFF;
extern int Read_HW_ID(void);

/*****************************************************************************
 * MACRO definitions and structure add for update firmware
 ****************************************************************************/
#define UF_FWPAGESIZE		128     // write 128 bytes each times
#define UF_FWIAPSIZE		0x1000L	// skip IAP part when compare f/w data
#define UF_FWVERSION_LOCATION		4249	// fw version location
//
#define UF_CMD_SIZE 		4       // command is 4 bytes
#define UF_ID_ENTERISP		0x00
#define UF_ID_ISPWRITE		0x01
#define UF_ID_ISPREAD		0x02
//
#define FW_VERSION_DIFFERENCE_95U_140U	4

/*****************************************************************************
 * MACRO definitions and structure add for update firmware
 ****************************************************************************/
#define FIRMWARE_UPDATE_WITH_HEADER 1

#ifdef FIRMWARE_UPDATE_WITH_HEADER
static int firmware_update_header(struct i2c_client *client, const unsigned char *firmware, unsigned int page_number);
#endif

struct waltop_I2C
{
	struct i2c_client	*client;
	struct input_dev	*input;
	struct work_struct	work;
	struct timer_list	timer;
    struct waltop_platform_data    *pdata;
    
	char	phys[32];
	atomic_t irq_enabled;
	// Minimun value of X,Y,P are 0
	__u16	x_max;      // X maximun value
	__u16	y_max;      // Y maximun value
	__u16	p_max;      // Pressure maximun value
	__u16	p_minTipOn;	// minimun tip on pressure
	__u16	fw_version;	// 110 means 1.10

	__u8	pkt_data[16];// packets data buffer

	// Ensures that only one function can specify the Device Mode at a time
	struct mutex mutex;	// reentrant protection for struct
	unsigned int delaytime;
	unsigned int hand_state;
	unsigned int second_pen_state;
	int pdct_irq;
	struct early_suspend early_suspend;
	struct switch_dev pen_switch;
	struct workqueue_struct *pen_det_wq;
	struct delayed_work pen_det_work;
	struct switch_dev digitizer_switch;
};



/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/
void waltop_I2C_worker(struct work_struct *work);
static irqreturn_t waltop_I2C_irq(int irq, void *handle);

static int waltop_I2C_read(struct waltop_I2C *tp);

static int waltop_I2C_remove(struct i2c_client *client);
static int waltop_I2C_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_HAS_WALTOP_EARLYSUSPEND
static void waltop_early_suspend(struct early_suspend *h);
static void waltop_late_resume(struct early_suspend *h);
#endif



/*****************************************************************************
 * Global Variables
 ****************************************************************************/
static struct workqueue_struct *waltop_I2C_wq;

static const struct i2c_device_id waltop_I2C_idtable[] = {
	{ "waltop", 0 },
	{ }
};

static struct i2c_driver waltop_I2C_driver = {
	.driver = {
		.name	= "waltop",
		.owner	= THIS_MODULE,
	},
	.id_table	= waltop_I2C_idtable,
	.probe		= waltop_I2C_probe,
	.remove	=  waltop_I2C_remove,
#ifndef CONFIG_HAS_WALTOP_EARLYSUSPEND
	.suspend = waltop_suspend,
	.resume = waltop_resume,
#endif

};

#ifdef IAP_FWUPDATE
static DECLARE_WAIT_QUEUE_HEAD(iap_wait_queue_head);
#ifdef USE_WAKELOCK
struct wake_lock iap_wake_lock;
struct wake_lock pen_detect_wake_lock;
#endif
static int wait_queue_flag=0;
static int m_loop_write_flag=0;
static int m_request_count=0;	// request bytes counter
static int m_iap_fw_updating=0;	// 1 means updating
static int m_iapIsReadCmd=0;
static int m_iapPageCount=0;
static int m_fw_cmdAckValue=0;	// fw command ack value
static char iap_fw_status[64];	// fw update status string
#endif

static int m_pen_detected;
static int m_pen_insert_state;

static int m_dominant_hand_state = 1; // default, right hand = 1, left hand = 0;
static int m_second_pen_state = 0; // 0 means not support
static __u16 waltop_fw_version;
static struct kobject *android_digitizer_kobj = NULL;
static struct waltop_I2C *private_tp = NULL;

#ifdef I2C_CHECK_CODE
static int m_i2c_check_mode=0;	// default is 0, 1 for i2c_check mode
static int m_i2c_check_time=600;	// default is 60 sec
static int m_i2c_check_errorCount=0;	// Error counter
static int m_i2c_check_totalCount=0;	// Total count
static int m_i2c_check_FailRateInt=0;	// default is 2 percent
static int m_i2c_check_FailRateFrac=200;// 200 means 2 percent
static int m_i2c_check_repeatTime=2;	// default is repeat 2 times
static int m_i2c_check_ContinueErrorCount=0;	// Continue Error counter
static int m_i2c_check_IsReapeatErrorCount=0;	// cycling between 0 and m_i2c_check_repeatTime
static struct timespec m_i2c_check_beginTime;
static struct timespec m_i2c_check_currentTime;
static int m_i2c_count_mode=0;
#endif

static int I2C_write_func(struct waltop_I2C *tp, unsigned char *write_buf, int write_count)
{
	int ret = -1;
    struct i2c_msg msg;

	/*if( write_count <= 4 ) {
		printk(KERN_INFO "FW CMD=0x%02x, 0x%02x, 0x%02x, 0x%02x\n", 
           write_buf[0], write_buf[1], write_buf[2], write_buf[3]);
	}*/
	msg.addr = tp->client->addr;
    msg.flags = 0; //Write
	msg.len = write_count;
	msg.buf = write_buf;
	
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = write_count;
 	}
	else { // negative is error
		printk(KERN_ERR "%s %s failed?-:%d\n", KLOG_NAME,__FUNCTION__, __LINE__);
		printk(KERN_ERR "%s FW CMD=0x%02x, 0x%02x, 0x%02x, 0x%02x\n", KLOG_NAME,
           write_buf[0], write_buf[1], write_buf[2], write_buf[3]);
		ret = -EINVAL;
	}
	return ret;
}

/*****************************************************************************
 * suspend and resume functions
 ****************************************************************************/

void force_release_pos(struct i2c_client *client) //TO-DO
{
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	struct input_dev *inp = tp->input;

	input_report_key(inp, BTN_TOOL_PEN, 0);
	input_report_abs(inp, ABS_PRESSURE, 0);
	input_report_key(inp, BTN_TOUCH, 0);
	input_report_key(inp, BTN_STYLUS, 0);
	input_report_key(inp, BTN_STYLUS2, 0);

	input_sync(tp->input);
}

static int waltop_set_reset_state(struct i2c_client *client, int state)
{
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	if(gpio_get_value(private_tp->pdata->gpio_reset)==state)
		return 0;
	if (state==IC_RESET_HIGH) {
		gpio_direction_output(tp->pdata->gpio_reset, 1);
		mdelay(50);
		printk(KERN_INFO "%s %s set chip reset HIGH\n",KLOG_NAME,__func__);
	} else {
		gpio_direction_output(tp->pdata->gpio_reset, 0);
		mdelay(30);
		printk(KERN_INFO "%s %s set chip reset LOW\n",KLOG_NAME,__func__);
	}

	return 0;
}

static int waltop_check_ic_state(struct i2c_client *client)
{
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	struct i2c_msg msg;
	int ret = -1;
	unsigned char buf[1];

	//sent COMMAND 0x55
	buf[0] = 0x55;
	msg.addr = tp->client->addr;
	msg.flags = 0; //Write
	msg.len = 1;
	msg.buf = (unsigned char *)buf;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);

	return ret;
}

static int waltop_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct timeval start, end;
	do_gettimeofday(&start);
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	int rc = 0;
	printk(KERN_INFO "%s Enter %s \n",KLOG_NAME,__func__);
	force_release_pos(client); //TO-DO
	if(m_iap_fw_updating == 0){
		disable_irq(client->irq);
		rc = cancel_work_sync(&tp->work);
		if (rc)
			enable_irq(client->irq);

	    rc = waltop_set_reset_state(client, IC_RESET_LOW);
	}

	do_gettimeofday(&end);
	printk("PM-Digitizer-Suspend : spent %ld msecs\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));
	return 0;
}

static int waltop_resume(struct i2c_client *client)
{
	struct timeval start, end;
	do_gettimeofday(&start);
	struct waltop_I2C *tp = i2c_get_clientdata(client);
	int rc = 0, retry=5, ret = 0;
	unsigned char left_command[1] = {0x2D};
	printk(KERN_INFO "%s Enter %s \n",KLOG_NAME,__func__);
	if(m_iap_fw_updating == 0){
		if(m_pen_insert_state==1&&m_second_pen_state==0)
			goto dominant_hand_setting; //skip the turn on step
		do{
	    	rc = waltop_set_reset_state(client, IC_RESET_HIGH);
			rc = waltop_check_ic_state(client);
			if(rc==1)
				break;
		}while(--retry);
	}
dominant_hand_setting:
	if(m_dominant_hand_state == 0){
		ret=I2C_write_func(tp, left_command, 1);
		if(ret <0)
			goto failed;
		m_dominant_hand_state = 0;
		printk(KERN_INFO "%s dominant hand status change to %d.\n",KLOG_NAME, m_dominant_hand_state);

		}
failed:
    enable_irq(client->irq);

	do_gettimeofday(&end);
	printk("PM-Digitizer-Resume : spent %ld msecs\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));
	return 0;
}


#ifdef CONFIG_HAS_WALTOP_EARLYSUSPEND
static void waltop_early_suspend(struct early_suspend *h)
{
	struct waltop_I2C *tp;
	tp = container_of(h, struct waltop_I2C, early_suspend);
	waltop_suspend(tp->client, PMSG_SUSPEND);
}

static void waltop_late_resume(struct early_suspend *h)
{
	struct waltop_I2C *tp;
	tp = container_of(h, struct waltop_I2C, early_suspend);
	waltop_resume(tp->client);
}
#endif

static void do_reset_pen(void)
{
	// Reset it for sync
	mutex_lock(&private_tp->mutex);

	gpio_direction_output(private_tp->pdata->gpio_reset, 0);
	mdelay(20);
	gpio_direction_output(private_tp->pdata->gpio_reset, 1);
	m_i2c_check_mode = 0;
	m_i2c_check_time = 60;

	mutex_unlock(&private_tp->mutex);
}

/*****************************************************************************
 * I2C read functions
 ****************************************************************************/
static int waltop_I2C_read(struct waltop_I2C *tp)
{
	int ret = -1;
    struct i2c_msg msg;

//  2012/12, Martin, This is only for X6, 
#ifdef PEN_ASIC_X6_VERSION
	unsigned char buf[1];
  	//20120614 Herman : sent COMMAND 0x4F to Pen >>>>>
	buf[0] = 0x4F;
  	//20120614 split to two messages for X6
    msg.addr = tp->client->addr;
    msg.flags = 0; //Write
    msg.len = 1;
    msg.buf = (unsigned char *)buf;

	ret = i2c_transfer(tp->client->adapter, &msg, 1);
#endif
//
	msg.addr = tp->client->addr;
    msg.flags = I2C_M_RD; //Read
	msg.len = 8;
	msg.buf = tp->pkt_data;
	
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = 8;
    }else if(ret == -ETIMEDOUT){ // I2C timeout , reset the chip
        do_reset_pen();
    }
 	else {// some error
		printk(KERN_ERR "%s failed?-%s:%d , Error code = %d \n", __FUNCTION__, __FILE__, __LINE__,ret);
	}
	return ret;
}

static int waltop_I2C_readDeviceInfo(struct waltop_I2C *tp)
{
	__u8 sum = 0;
	int i, ret = -1;
    struct i2c_msg msg;
	unsigned char buf[1];

  	//sent COMMAND 0x2A to Pen >>>>>
	buf[0] = 0x2A;
    msg.addr = tp->client->addr;
    msg.flags = 0; //Write
    msg.len = 1;
    msg.buf = (unsigned char *)buf;
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1)	// 1 msg sent OK
 	{
		// Delay 1 ms, wait for f/w device data ready
		mdelay(1);
		//read back device information
		msg.addr = tp->client->addr;
		msg.flags = I2C_M_RD; //Read
		msg.len = 9;
		msg.buf = tp->pkt_data;
	
		ret = i2c_transfer(tp->client->adapter, &msg, 1);

		if( ret == 1) {	// 1 msg sent OK
			// Check checksum
			for(i=1; i<8; i++) // D1 to D7
				sum = sum + tp->pkt_data[i];
			if( sum == tp->pkt_data[8] )
				ret = 9;
			else {
				ret = -2;
				printk(KERN_ERR "%s Checksum error!-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
			}
		}
		else {// some error
			printk(KERN_ERR "%s failed?-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
		}
	}
	return ret;
}

#ifdef IAP_FWUPDATE
/*****************************************************************************
 * Firmware update related finctions
 ****************************************************************************/
static int I2C_read_func(struct waltop_I2C *tp, unsigned char *read_buf, int read_count)
{
	int ret = -1;
    struct i2c_msg msg;

	msg.addr = tp->client->addr;
    msg.flags = I2C_M_RD; //Read
	msg.len = read_count;
	msg.buf = read_buf;
	
	ret = i2c_transfer(tp->client->adapter, &msg, 1);
 	if( ret == 1) {	// 1 msg sent OK
		ret = read_count;
 	}
 	else {// some error
		printk(KERN_ERR "%s failed?-%s:%d\n", __FUNCTION__, __FILE__, __LINE__);
	}
	return ret;
}

static int Read_Ack(struct waltop_I2C *tp)
{
	int ret = -1;
	unsigned char tmp_buf[10];
	
	tmp_buf[0]=0;
	tmp_buf[1]=0;
	ret = I2C_read_func(tp, tmp_buf, 2);
	//printk(KERN_INFO "FW Ack Value=%x,%x\n", tmp_buf[0], tmp_buf[1]);
	if( ret == 2) { // read count as we request
		ret = (tmp_buf[0] << 8 | tmp_buf[1]);
	}
    //printk(KERN_INFO "FW Ack Value=%d", ret);
	return ret;
}

static int Write_Data(struct waltop_I2C *tp, unsigned char *tx_buf, int count)
{
	int i;
	unsigned char checkSum;
	unsigned char out_buffer[FW_IOBUFFER_SIZE];

	checkSum = 0;
	if( count>FW_IODATA_SIZE )
		count = FW_IODATA_SIZE;
    for (i=0; i<count; i++)
	{ 
		out_buffer[i] = tx_buf[i];
		checkSum += tx_buf[i];
		//printk("%x,",tx_buf[i]);
	}

	//printk("\n");
	out_buffer[i] = checkSum;
	out_buffer[i+1] = checkSum;
	return I2C_write_func(tp, out_buffer, count+2);
}

static ssize_t fwdata_read(struct file *filp, struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{

	struct device *dev = container_of(kobj, struct device, kobj);
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	unsigned char ACKDataOK[2] = {0x92, 0xE1};
	unsigned char ACKPageOK[2] = {0x92, 0xE2};
	unsigned char rx_buf[FW_BLOCK_SIZE+4];
	int i, ret=0, retCount=count;

	if(count>FW_BLOCK_SIZE) {
		printk(KERN_ERR "read size over buffer size!\n");
		return -1;
	}

	strcpy(iap_fw_status, "reading");
	m_loop_write_flag = 1;

	for(i=0; i<(count-FW_IODATA_SIZE); i=i+FW_IODATA_SIZE)
	{
		ret=I2C_read_func(tp, &rx_buf[i], FW_IODATA_SIZE);
		if(FW_IODATA_SIZE==ret) {
			wait_queue_flag = 0;
			ret=I2C_write_func(tp, ACKDataOK, 2);
			// wait for fw INT
			wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
		}
		else {
			retCount = 0x92E0;
			strcpy(iap_fw_status, "error");
			break;
		}
	}
	if(retCount<0x9200) // send read page OK and return data
	{
		//last read
		ret=I2C_read_func(tp, &rx_buf[count-FW_IODATA_SIZE], FW_IODATA_SIZE);
		if(FW_IODATA_SIZE==ret) {
			wait_queue_flag = 0;
			m_iapPageCount--;
			// fwupdate will send final ack code at last read, so this is only for every page
			if( m_iapPageCount>0 ) {
				ret=I2C_write_func(tp, ACKPageOK, 2);
				// wait for fw INT from f/w
				wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
			}
			memcpy(buf, rx_buf, count);
		}
		else {
			retCount = 0x92E0;
			strcpy(iap_fw_status, "error");
		}
	}
	wait_queue_flag = 0;
	m_loop_write_flag = 0;
	return retCount;
}

static ssize_t fwdata_write(struct file *filp, struct kobject *kobj,
			    struct bin_attribute *bin_attr,
			    char *buf, loff_t off, size_t count)
{

    //printk(KERN_INFO "request count size=%d\n", count);
    struct device *dev = container_of(kobj, struct device, kobj);
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	unsigned char tx_buf[FW_BLOCK_SIZE+4];
	int i, ret=0, retCount=count;
    
    if(count>FW_BLOCK_SIZE) {
		printk(KERN_ERR "write size over buffer size!\n");
		return -1;
	}
	memcpy(tx_buf, buf, count);

	m_fw_cmdAckValue = 0;
	wait_queue_flag = 0;
	m_loop_write_flag = 1;

	// 2012/12/18, Martin check
	if((count==4)&&(tx_buf[0]==0x84)) //make sure it is fw IAP command
	{
		m_iapIsReadCmd=0;
		ret=I2C_write_func(tp, tx_buf, count);
		// wait for fw ACK
		wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
		wait_queue_flag = 0;
		if(strcmp(iap_fw_status, "error") == 0) {
			retCount = m_fw_cmdAckValue;	// return the error code
		}
		else {
			if(tx_buf[1]==0x02) {	// Start IAP Read
				m_iapIsReadCmd = 1;
				m_iapPageCount = tx_buf[2]*4;
				// wait for fw INT then back
				wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
				wait_queue_flag = 0;
			}
		}
	}else if(count == 2 && tx_buf[0] == 0x92){
	    dev_info(&tp->client->dev, "Write the finish message 0x%02X,0x%02X\n", tx_buf[0], tx_buf[1]);
	    ret=I2C_write_func(tp, tx_buf, count);
	}else // Write data
	{
		strcpy(iap_fw_status, "waiting");
		m_request_count = count;
		for(i=0; i<count; i=i+FW_IODATA_SIZE)
		{
			//m_request_count -= FW_IODATA_SIZE;
			ret=Write_Data(tp, &(tx_buf[i]), FW_IODATA_SIZE);
			// wait for fw ACK
			wait_event_interruptible(iap_wait_queue_head, wait_queue_flag!=0);
			wait_queue_flag = 0;
            m_request_count -= FW_IODATA_SIZE;
			//if(strcmp(iap_fw_status, "error") == 0 || strcmp(iap_fw_status, "finish") == 0) {
            if(m_fw_cmdAckValue == 0x9200 || m_fw_cmdAckValue == 0x92E0 || m_fw_cmdAckValue == 0x92EF) { 
				retCount = m_fw_cmdAckValue;	// return the error code 0x9200, 0x92E0, 0x92EF
				break;
			}
		}
	}
	m_loop_write_flag = 0;
	return retCount;
}

static void waltop_enter_IAP(struct waltop_I2C *tp){
	//Pulse pattern to enter IAP mode
	mutex_lock(&tp->mutex);
		disable_irq(tp->client->irq);
	gpio_direction_output(tp->pdata->gpio_int, 0);
	udelay(5);
	/* Reset Pen, LOW for 30 ms, then HIGH */
	gpio_direction_output(tp->pdata->gpio_reset, 0);
	mdelay(FW_RESET_DELAY_A2B);
	gpio_direction_output(tp->pdata->gpio_reset, 1);
	mdelay(FW_RESET_DELAY_B2C);
	/* Set IRQ pin to High, re-enable IRQ */
	/* set SCL=HIGH */
	gpio_direction_input(tp->pdata->gpio_int);
	udelay(5);
	if (tp->client->irq != 0)
	    enable_irq(tp->client->irq);
	/* wait for ready */
	mdelay(FW_RESET_DELAY_CP);
	mutex_unlock(&tp->mutex);
}

static void waltop_exit_IAP(struct waltop_I2C *tp){

    mutex_lock(&tp->mutex);
	gpio_direction_output(tp->pdata->gpio_reset, 0);
	mdelay(30);
	gpio_direction_output(tp->pdata->gpio_reset, 1);
	mdelay(50);

	m_iap_fw_updating = 0;
	m_iapIsReadCmd = 0;
	m_iapPageCount = 0;
#ifdef USE_WAKELOCK
	wake_unlock(&iap_wake_lock);
#endif
	strcpy(iap_fw_status, "exitIAP");
	mutex_unlock(&tp->mutex);
}

static struct bin_attribute waltop_I2C_fwdata_attributes = {
	.attr = {
		.name = "fwdata",
		.mode = S_IRUGO|S_IWGRP|S_IWUSR, //change this to super user only when release
	},
	.size = 0,	// 0 means no limit, but not over 4KB
	.read = fwdata_read,
	.write = fwdata_write,
};
#endif

#ifdef FIRMWARE_UPDATE_WITH_HEADER
static unsigned char digitizer_firmware[] = {
 #include "waltop_fw_data.b"
};


static int check_fw_version(struct waltop_I2C *tp){
	int fwversion = digitizer_firmware[UF_FWVERSION_LOCATION]&0x7F;
	printk(KERN_INFO "%s check fw.bin version = %d ", KLOG_NAME ,fwversion);
	HW_ID = Read_HW_ID();

	if(tp->fw_version >= fwversion)
		goto NO_NEED_TO_UPDATE_FIRMWARE;

	switch(HW_ID){
		case HW_ID_EV:
		case HW_ID_SR1:
		case HW_ID_SR1_SKU_3_4:
		case HW_ID_PRE_ER:
		case HW_ID_ER:
                        goto NO_NEED_TO_UPDATE_FIRMWARE;
			break;
		case HW_ID_PR:
		case HW_ID_MP:
			if(fwversion > FW_VERSION_DIFFERENCE_95U_140U)
				return 1;
			break;
		deafult:
			goto NO_NEED_TO_UPDATE_FIRMWARE;
	}

NO_NEED_TO_UPDATE_FIRMWARE:
	return 0;
}

static int update_fw_information(struct waltop_I2C *tp){
	int ret;
	ret = waltop_I2C_readDeviceInfo(tp);
	if( ret>0 ) {
		tp->x_max = ((tp->pkt_data[1] << 8) | tp->pkt_data[2]);
		tp->y_max = ((tp->pkt_data[3] << 8) | tp->pkt_data[4]);
		tp->p_max = ((tp->pkt_data[6]&0x80)>>7)|((tp->pkt_data[7]&0x80)>>6);
		tp->p_max = ((tp->p_max << 8) | (tp->pkt_data[5]));
		tp->fw_version = ((tp->pkt_data[6]&0x7F)*100) + (tp->pkt_data[7]&0x7F);
        printk(KERN_ERR "waltop_I2C_readDeviceInfo() - firmware version : %d , Max Pressure : %d \n", tp->fw_version ,tp->p_max);
	}
	else {
		tp->x_max = tp->pdata->x_max ? tp->pdata->x_max : WALTOP_MAX_X;
		tp->y_max = tp->pdata->y_max ? tp->pdata->y_max : WALTOP_MAX_Y;
		tp->p_max = tp->pdata->p_max ? tp->pdata->p_max : WALTOP_MAX_P;
		tp->fw_version = 0; // default
        printk(KERN_ERR "waltop_I2C_readDeviceInfo() ERROR - firmware version : %d , Max Pressure : %d \n", tp->fw_version ,tp->p_max);
	}
	tp->p_minTipOn = tp->pdata->p_minTipOn ? tp->pdata->p_minTipOn : 0;
	waltop_fw_version = tp->fw_version;

	return ret;
}

static int firmware_update_header(struct i2c_client *client, const unsigned char *firmware, unsigned int pages_number){
    int ret, i,j,retCount =0;
    int retry_times = 3, write_times;

	u8 cmdEnterISPMode[UF_CMD_SIZE] = {0x84, 0x00, 0x10, 0x14};
	u8 cmdISPWriteStart[UF_CMD_SIZE]= {0x84, 0x01, 0x40 ,0x45};
    u8 cmdISPWriteStartWithVerify[UF_CMD_SIZE]= {0x84, 0x11, 0x40 ,0x55};
	u8 cmdISPReadStart[UF_CMD_SIZE] = {0x84, 0x02, 0x40 ,0x46};

    unsigned char *cursor;
    struct waltop_I2C *tp = i2c_get_clientdata(client);

    if(tp == NULL) 
        return -1;

    printk(KERN_INFO "%s Start firmware update!\n",KLOG_NAME);
	printk(KERN_INFO "%s FW pages_number = %d \n", KLOG_NAME,pages_number);
    wake_lock(&iap_wake_lock);
    m_iap_fw_updating = 1;
    waltop_enter_IAP(tp);

	//memcpy(tx_buf,cmdISPWriteStartWithVerify , UF_CMD_SIZE);

	m_fw_cmdAckValue = 0;
	wait_queue_flag = 0;
	m_loop_write_flag = 1;

	//Write IAP command
	m_iapIsReadCmd=0;
	ret=I2C_write_func(tp, cmdEnterISPMode, UF_CMD_SIZE);
	// wait for fw ACK
	if(ret<0)
		goto update_failed;
	ret = wait_event_interruptible_timeout(iap_wait_queue_head, wait_queue_flag!=0,HZ/20);
	wait_queue_flag = 0;
	if(strcmp(iap_fw_status, "error") == 0||ret==0) {
		retCount = m_fw_cmdAckValue;	// return the error code
		goto update_failed;
	}
	udelay(50);

	//Write write-command
	ret=I2C_write_func(tp, cmdISPWriteStart, UF_CMD_SIZE);
	if(ret<0)
		goto update_failed;
	ret = wait_event_interruptible_timeout(iap_wait_queue_head, wait_queue_flag!=0,HZ/20);
	wait_queue_flag = 0;
	if(strcmp(iap_fw_status, "error") == 0||ret==0) {
		retCount = m_fw_cmdAckValue;	// return the error code
		goto update_failed;
	}
	udelay(50);

	//Write Data
	cursor = firmware;
	for(i=0; i<pages_number;i++) {
		strcpy(iap_fw_status, "waiting");
		m_request_count = FW_IODATA_SIZE;
		for(j=0; j<UF_FWPAGESIZE; j=j+FW_IODATA_SIZE)
		{
			ret=Write_Data(tp, &(cursor[j]), FW_IODATA_SIZE);
			if(ret<0)
				goto update_failed;
			// wait for fw ACK
			ret = wait_event_interruptible_timeout(iap_wait_queue_head, wait_queue_flag!=0,HZ/20);
			wait_queue_flag = 0;
	        m_request_count -= FW_IODATA_SIZE;
	        if(m_fw_cmdAckValue == 0x9200 || m_fw_cmdAckValue == 0x92E0||ret==0) {
				retCount = m_fw_cmdAckValue;	// return the error code 0x9200, 0x92E0
				goto update_failed;
			}else if(m_fw_cmdAckValue == 0x92EF){ // Write End
				retCount = m_fw_cmdAckValue;
				printk(KERN_INFO "%s FW ACK = %x , Write Finished" ,KLOG_NAME,retCount);
			}
		}
		cursor += UF_FWPAGESIZE;
	}
	m_loop_write_flag = 0;

	mdelay(20);
update_success:
	waltop_exit_IAP(tp);
	mdelay(20);
	if(update_fw_information(tp)<0)
		goto update_failed;
	printk(KERN_INFO "%s Firmware Update Success !!" ,KLOG_NAME );
	return retCount;

update_failed:
	waltop_exit_IAP(tp);
	printk(KERN_INFO "%s Firmware Update Failed !! ERROR code = %d , ret = %d" ,KLOG_NAME ,retCount,ret);
	return retCount;


}



#endif

/*************************************************************************
 * SYSFS related functions
 ************************************************************************/
static ssize_t waltop_show_irq_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", atomic_read(&tp->irq_enabled));
}

static ssize_t waltop_irq_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
	if (atomic_cmpxchg(&tp->irq_enabled, 1, 0))
	{
		dev_dbg(dev, "%s() - PEN IRQ %u has been DISABLED.\n", __func__, tp->client->irq);
		disable_irq(tp->client->irq);
	}
	else
	{
		atomic_set(&tp->irq_enabled, 1);
		dev_dbg(dev, "%s() - PEN IRQ %u has been ENABLED.\n", __func__, tp->client->irq);
		enable_irq(tp->client->irq);
	}
               	
    return size;
}
static DEVICE_ATTR(irq_enable,		S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,	waltop_show_irq_status,		waltop_irq_enable);

static ssize_t waltop_show_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
    mutex_lock(&tp->mutex);
	disable_irq(tp->client->irq);
	gpio_direction_output(tp->pdata->gpio_reset, 0);
	mdelay(20);
	gpio_direction_output(tp->pdata->gpio_reset, 1);
#ifdef I2C_CHECK_CODE
	m_i2c_check_mode = 0;
#endif
	// enable irq again
	if (tp->client->irq != 0)
	{
		irq_set_irq_type(tp->client->irq, IRQF_TRIGGER_FALLING);
		enable_irq(tp->client->irq);
	}
	mutex_unlock(&tp->mutex);

    return snprintf(buf, PAGE_SIZE, "Reset finished.\n");
}

static ssize_t waltop_show_read_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
    mutex_lock(&tp->mutex);
	waltop_I2C_read(tp);
	mutex_unlock(&tp->mutex);
	
	return snprintf(buf, PAGE_SIZE, "%x, %x, %x, %x,   %x, %x, %x, %x,   %x, %x, %x\n",
		tp->pkt_data[0], tp->pkt_data[1], tp->pkt_data[2], tp->pkt_data[3],
		tp->pkt_data[4], tp->pkt_data[5], tp->pkt_data[6], tp->pkt_data[7],
		tp->pkt_data[8], tp->pkt_data[9], tp->pkt_data[10]);
}

static ssize_t waltop_store_write_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	
	__u8 tx_buf[4] = {0};
	int buf_tmp[4] = {0};
	int ret = 0;
	
    mutex_lock(&tp->mutex);
    sscanf(buf, "%x%x%x", &buf_tmp[0], &buf_tmp[1], &buf_tmp[2]);
    printk(KERN_INFO "%x, %x, %x\n", buf_tmp[0], buf_tmp[1], buf_tmp[2]);
	tx_buf[0] = (__u8) buf_tmp[0];
	tx_buf[1] = (__u8) buf_tmp[1];
	tx_buf[2] = (__u8) buf_tmp[2];

	ret = i2c_master_send(tp->client, tx_buf, 3);
	if( ret<0 ) { // negative is error
		printk(KERN_ERR "i2c_write failed?-%s:%d\n", __FILE__, __LINE__);
		ret = -EINVAL;
	} else if (ret==0){
		printk(KERN_ERR "i2c_write Success  -%s:%d\n", __FILE__, __LINE__);
    }
	mutex_unlock(&tp->mutex);
	
	return size;
}


static ssize_t waltop_store_delaytime(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
		struct waltop_I2C *tp = dev_get_drvdata(dev);

		sscanf(buf, "%d", &tp->delaytime);
		printk("Set the delay time %d\n", tp->delaytime);
		return size;
}


static ssize_t waltop_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
	int ret;
	ret = waltop_I2C_readDeviceInfo(tp);
	if( ret>0 ) {
		tp->fw_version = ((tp->pkt_data[6]&0x7F)*100) + (tp->pkt_data[7]&0x7F);
	}

    return sprintf(buf, "%u\n", tp->fw_version);
}

static ssize_t waltop_digitizer_switch_name(struct switch_dev *sdev, char *buf)
{
	int count = 0;
	count += sprintf(buf+count, "Waltop:Version_0x0");
	count += sprintf(buf+count, "%u\n", waltop_fw_version);
       return count;
}

static ssize_t print_pen_insert_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", m_pen_insert_state);
}

#ifdef IAP_FWUPDATE
static ssize_t waltop_show_enter_IAP(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);
    mutex_lock(&tp->mutex);
    
	m_iap_fw_updating = 1;
    //disable_irq_nosync(INT_GPIO_BASE + TEGRA_GPIO_PH4);
#ifdef USE_WAKELOCK
	wake_lock(&iap_wake_lock);
#endif
	strcpy(iap_fw_status, "enterIAP");
	// Enter IAP
	// MUX -Add change your CPU'mux to GPIO here if needed
    // GPIO6 = Low, SDA = HIGH, SCL = LOW
#ifdef PEN_GPIO_FW_UPDATE
	gpio_set_value(PEN_GPIO_FW_UPDATE, 0);
#endif
	/* Disable IRQ, set IRQ pin to Low */	
    disable_irq(tp->client->irq);
    gpio_direction_output(tp->pdata->gpio_int, 0);	
    udelay(5);	
    /* Reset Pen, LOW for 30 ms, then HIGH */	
    gpio_direction_output(tp->pdata->gpio_reset, 0);	
    mdelay(FW_RESET_DELAY_A2B);	
    gpio_direction_output(tp->pdata->gpio_reset, 1);	
    mdelay(FW_RESET_DELAY_B2C);    
    /* Set IRQ pin to High, re-enable IRQ */	
    /* set SCL=HIGH */	
    gpio_direction_input(tp->pdata->gpio_int);	
    udelay(5);	
    if (tp->client->irq != 0)		
        enable_irq(tp->client->irq);    
    /* wait for ready */    
    mdelay(FW_RESET_DELAY_CP);
	mutex_unlock(&tp->mutex);

    return snprintf(buf, PAGE_SIZE, "Enter firmware update mode.\n");
}

static ssize_t waltop_show_exit_IAP(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct waltop_I2C *tp = dev_get_drvdata(dev);

    mutex_lock(&tp->mutex);
    
	// GPIO6 = Low, SDA = HIGH, SCL = HIGH
#ifdef PEN_GPIO_FW_UPDATE
	//gpio_set_value(PEN_GPIO_FW_UPDATE, 0);
#endif
	//gpio_direction_output(TEGRA_GPIO_PC4, 1);	// set SDA High
	//gpio_direction_output(TEGRA_GPIO_PC5, 1);	// set SCL High
	// Reset Pen, LOW for 20 ms, then HIGH
	gpio_direction_output(tp->pdata->gpio_reset, 0);
	mdelay(30);
	gpio_direction_output(tp->pdata->gpio_reset, 1);
	mdelay(50);
    
	// MUX -Add change your CPU'mux to I2C here if needed
	m_iap_fw_updating = 0;
	m_iapIsReadCmd = 0;
	m_iapPageCount = 0;
#ifdef USE_WAKELOCK
	wake_unlock(&iap_wake_lock);
#endif
	strcpy(iap_fw_status, "exitIAP");
    //enable_irq(INT_GPIO_BASE + TEGRA_GPIO_PH4);
	mutex_unlock(&tp->mutex);

    return snprintf(buf, PAGE_SIZE, "Enter normal mode.\n");
}
#endif

//Shouchung add ATD tool function
static ssize_t waltop_get_digitizer_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -1;
	int test_count = 5;
	int i ,value;
	struct i2c_msg msg;
	mutex_lock(&private_tp->mutex);
	for (i = 0; i < test_count; i++) {
		ret = waltop_I2C_readDeviceInfo(private_tp);
		if(ret < 0) {
			value = gpio_get_value(private_tp->pdata->gpio_reset) ? 1 :0;
			printk(KERN_INFO "%s status check=%d , reset = %d , count = %d",KLOG_NAME,ret,value,i);
			break;
		}
		msleep(50);
	}
	mutex_unlock(&private_tp->mutex);

	return sprintf(buf, "%d\n", (ret < 0) ? 0 : 1);
}
//Shouchung end

static ssize_t waltop_show_dominant_hand_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", m_dominant_hand_state);
}

static ssize_t waltop_set_dominant_hand(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 left_command[1] = {0x2D};
	u8 right_command[1] = {0x2C};
	int ret = 0;
	sscanf(buf, "%d", &private_tp->hand_state);
	m_dominant_hand_state = private_tp->hand_state;
	//left_hand
	if (m_dominant_hand_state == 0){
             ret = I2C_write_func(private_tp, left_command, 1);
		if(ret < 0)
			goto failed;
		printk(KERN_INFO "%s dominant hand status change to %d.\n",KLOG_NAME, m_dominant_hand_state);
		return size;
	}
	//right_hand
	if (m_dominant_hand_state == 1)
	{
		ret = I2C_write_func(private_tp, right_command, 1);
		if(ret < 0)
			goto failed;
		printk(KERN_INFO "%s dominant hand status change to %d.\n",KLOG_NAME, m_dominant_hand_state);
		return size;
	}
failed:
    return size;
}

static ssize_t waltop_show_second_pen_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", m_second_pen_state);
}

static ssize_t waltop_set_second_pen(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d", &private_tp->second_pen_state);
	m_second_pen_state = private_tp->second_pen_state;
	if(m_second_pen_state==0){
		if(m_pen_insert_state==0)
			waltop_set_reset_state(private_tp->client,IC_RESET_HIGH);
		else
			waltop_set_reset_state(private_tp->client,IC_RESET_LOW);
	}else{
		waltop_set_reset_state(private_tp->client,IC_RESET_HIGH);
	}
	return size;
}

#ifdef I2C_CHECK_CODE
static ssize_t waltop_store_enable_data_count(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//struct waltop_I2C *tp = dev_get_drvdata(dev);
	m_i2c_check_errorCount = 0;
	m_i2c_check_totalCount = 0;
	m_i2c_check_ContinueErrorCount = 0;
	m_i2c_check_IsReapeatErrorCount = 0;
    m_i2c_check_beginTime = current_kernel_time();
	sscanf(buf, "%d ", &m_i2c_count_mode);
	return size;
}

static ssize_t waltop_store_checki2c_start(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = -1;
	unsigned char CheckCmd[2] = {0x38, 0xC7};
	//struct waltop_I2C *tp = dev_get_drvdata(dev);
    struct i2c_msg msg;
    int	tmpInt=0, tmpFrac=0;

	sscanf(buf, "%d %d.%d %d", &m_i2c_check_time, &tmpInt,
		&tmpFrac, &m_i2c_check_repeatTime);
	m_i2c_check_FailRateInt = tmpInt/100;
	m_i2c_check_FailRateFrac = (tmpInt-m_i2c_check_FailRateInt)*10+tmpFrac;

	m_i2c_check_errorCount = 0;
	m_i2c_check_totalCount = 0;
	m_i2c_check_ContinueErrorCount = 0;
	m_i2c_check_IsReapeatErrorCount = 0;

	// Send command to f/w to start check process
	// cmd + time in seconds + checksum
	msg.addr = private_tp->client->addr;
	msg.flags = 0; //Write
	msg.len = 2;
	msg.buf = CheckCmd;

	ret = i2c_transfer(private_tp->client->adapter, &msg, 1);
	if( ret == 1 ) {	// 1 msg sent OK
		m_i2c_check_beginTime = current_kernel_time();
		m_i2c_check_mode = 1;
		printk("i2c_check_start - OK , %d sec \n",m_i2c_check_time);
	}
	else { // negative is error
		printk(KERN_ERR "%s %s failed?-:%d\n", KLOG_NAME,__FUNCTION__, __LINE__);
		printk("i2c_check_start - Error \n");
		m_i2c_check_errorCount++;
	}
	return size;
}

void waltop_GetPercentage(long molecular, long denominator, int *resultInt, long *resultFrac)
{
	long theInt=0, theFrac=0, tmpMolecular=0;

	if( molecular>0 ) {
		tmpMolecular = molecular;
		if( tmpMolecular>denominator ) {
			theInt = tmpMolecular/denominator;
			tmpMolecular = tmpMolecular - denominator;
		}
		theFrac = (tmpMolecular*100000)/denominator;
	}
	*resultInt = (int)theInt;
	// output frac 2176 means 0.02176
	*resultFrac = theFrac;
}

static ssize_t waltop_show_checki2c_report(struct device *dev, struct device_attribute *attr, char *buf)
{
	long td_sec, td_ns;
	int result=0;
	int errorRateInt=0, contRateInt=0, tint=0;
	long errorRateFrac=0, contRateFrac=0;

	//m_i2c_check_currentTime = current_kernel_time();
	td_sec = m_i2c_check_currentTime.tv_sec - m_i2c_check_beginTime.tv_sec;
	td_ns = (m_i2c_check_currentTime.tv_nsec - m_i2c_check_beginTime.tv_nsec)/1000000;	// in ms
	if( td_ns < 0 ) {
		td_sec--;
		td_ns += 1000;
	}

	waltop_GetPercentage(m_i2c_check_errorCount, m_i2c_check_totalCount, &errorRateInt, &errorRateFrac);
	waltop_GetPercentage(m_i2c_check_ContinueErrorCount, m_i2c_check_totalCount, &contRateInt, &contRateFrac);
	// Shift to percentage
	errorRateInt = errorRateInt*100+errorRateFrac/1000;
	tint = errorRateFrac/1000;
	errorRateFrac = errorRateFrac-tint*1000;
	contRateInt = contRateInt*100+contRateFrac/1000;
	tint = contRateFrac/1000;
	contRateFrac = contRateFrac-tint*1000;

	// format as timeStamp, totalCount, errorCount, ContinueErrorCount, errorRate, continueErrorRate
	return sprintf(buf, "%ld.%ld, %ld, %ld, %ld, %d.%03ld, %d.%03ld\n", td_sec, td_ns, m_i2c_check_totalCount, m_i2c_check_errorCount,
		m_i2c_check_ContinueErrorCount, errorRateInt, errorRateFrac, contRateInt, contRateFrac);
}

static ssize_t waltop_show_checki2c_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int result =0;
    int errorRateInt=0, errorRateFrac=0,tint=0;
    int input =0,output=0;
	if(m_i2c_check_errorCount>0) {
		waltop_GetPercentage(m_i2c_check_errorCount, m_i2c_check_totalCount, &errorRateInt, &errorRateFrac);
        errorRateInt = errorRateInt*100+errorRateFrac/1000;
        tint = errorRateFrac/1000;
        errorRateFrac = errorRateFrac-tint*1000;
        input =(int) 1000*m_i2c_check_FailRateInt+100*m_i2c_check_FailRateFrac;
        output = (int)1000*errorRateInt+errorRateFrac;
		if(output>input)
			result = 1;
	}
	return sprintf(buf, "%d\n" ,result);
}

static void reset_i2c(void){

	gpio_request(25, "scl");
	gpio_request(24, "sda");
	lnw_gpio_set_alt(25, LNW_GPIO);
	lnw_gpio_set_alt(24, LNW_GPIO);
	usleep_range(10, 10);
	gpio_direction_input(25);
	gpio_direction_input(24);
	lnw_gpio_set_alt(25, LNW_ALT_1);
	lnw_gpio_set_alt(24, LNW_ALT_1);
	usleep_range(10, 10);
	gpio_free(25);
	gpio_free(24);

}
#endif

// SYSFS : Device Attributes 
static DEVICE_ATTR(reset,			S_IRUGO,	waltop_show_reset,			NULL);
static DEVICE_ATTR(read_data,		S_IRUGO,	waltop_show_read_data,		NULL);
static DEVICE_ATTR(write_command,	S_IWGRP|S_IWUSR,	NULL,		waltop_store_write_cmd);
static DEVICE_ATTR(fwversion,		S_IRUGO,	waltop_show_fw_version,		NULL);
#ifdef IAP_FWUPDATE
static DEVICE_ATTR(fwupdate_entry,	S_IRUGO,	waltop_show_enter_IAP,		NULL);
static DEVICE_ATTR(fwupdate_exit,	S_IRUGO,	waltop_show_exit_IAP,		NULL);
#endif
#ifdef I2C_CHECK_CODE
static DEVICE_ATTR(checki2c_start,	S_IWGRP|S_IWUSR,	NULL,		waltop_store_checki2c_start);
static DEVICE_ATTR(checki2c_data,	S_IRUGO,	waltop_show_checki2c_report,		NULL);
static DEVICE_ATTR(checki2c_status,	S_IRUGO,	waltop_show_checki2c_status,		NULL);
static DEVICE_ATTR(checki2c_count,	S_IWGRP|S_IWUSR,	NULL,		waltop_store_enable_data_count);
#endif
static DEVICE_ATTR(write_delaytime,	S_IWGRP|S_IWUSR,	NULL,		waltop_store_delaytime);
//Shouchung add ATD tool function
static DEVICE_ATTR(digitizer_status,	S_IRUGO,	waltop_get_digitizer_status,		NULL);
//Shouchung end
static DEVICE_ATTR(dominant_hand,	S_IRUGO|S_IWGRP|S_IWUSR,	waltop_show_dominant_hand_status,		waltop_set_dominant_hand);
static DEVICE_ATTR(second_pen,		S_IRUGO|S_IWGRP|S_IWUSR,	waltop_show_second_pen_status,		waltop_set_second_pen);

static struct attribute *waltop_I2C_attributes[] = {
	&dev_attr_reset.attr,
	&dev_attr_read_data.attr,
	&dev_attr_write_command.attr,
    &dev_attr_fwversion.attr,
#ifdef IAP_FWUPDATE
	&dev_attr_fwupdate_entry.attr,
	&dev_attr_fwupdate_exit.attr,
#endif
#ifdef I2C_CHECK_CODE
    &dev_attr_checki2c_start.attr,
    &dev_attr_checki2c_data.attr,
    &dev_attr_checki2c_status.attr,
    &dev_attr_checki2c_count.attr,
#endif
    &dev_attr_write_delaytime.attr,
    &dev_attr_digitizer_status.attr, //Shouchung add ATD tool function
    &dev_attr_dominant_hand.attr,
    &dev_attr_second_pen.attr,
	NULL
};

static struct attribute_group waltop_I2C_attribute_group = {
    .attrs = waltop_I2C_attributes
};

static int waltop_digitizer_sysfs_init(void)
{
    int ret = 0;
    android_digitizer_kobj = kobject_create_and_add("android_digitizer", NULL);
    if (android_digitizer_kobj == NULL)
    {
        printk(KERN_ERR "%s Subsystem register failed\n", KLOG_NAME);
        ret = -ENOMEM;
        return ret;
    }
    ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_digitizer_status.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file digitizer_status failed\n",KLOG_NAME);
        return ret;
    }
    ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_dominant_hand.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file dominant_hand failed\n",KLOG_NAME);
        return ret;
    }
	ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_second_pen.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file second_pen failed\n",KLOG_NAME);
        return ret;
    }
	ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_checki2c_data.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file checki2c_data failed\n",KLOG_NAME);
        return ret;
    }
	ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_checki2c_start.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file checki2c_start failed\n",KLOG_NAME);
        return ret;
    }
	ret = sysfs_create_file(android_digitizer_kobj, &dev_attr_checki2c_status.attr);
    if (ret)
    {
        printk(KERN_ERR "%s Create file checki2c_status failed\n",KLOG_NAME);
        return ret;
    }
    return 0 ;
}

static void waltop_digitizer_sysfs_deinit(void)
{
    sysfs_remove_file(android_digitizer_kobj, &dev_attr_digitizer_status.attr);
    sysfs_remove_file(android_digitizer_kobj, &dev_attr_dominant_hand.attr);
	sysfs_remove_file(android_digitizer_kobj, &dev_attr_second_pen.attr);
	sysfs_remove_file(android_digitizer_kobj, &dev_attr_checki2c_data.attr);
	sysfs_remove_file(android_digitizer_kobj, &dev_attr_checki2c_start.attr);
	sysfs_remove_file(android_digitizer_kobj, &dev_attr_checki2c_status.attr);
    kobject_del(android_digitizer_kobj);
}

/*****************************************************************************
 * Interrupt and Workqueue related finctions
 ****************************************************************************/
void waltop_I2C_worker(struct work_struct *work)
{
	struct waltop_I2C *tp = container_of(work, struct waltop_I2C, work);
	struct input_dev *inp = tp->input;
	unsigned int x, y, ps, dv;
	int btn_up, btn_low, in_range, tip;
	__u16 sum;
	//printk("%s: enter...\n", __func__);
#ifdef IAP_FWUPDATE
	int ret = 0;

	if(m_iap_fw_updating)
	{
		if(m_iapIsReadCmd==0)
		{
			ret = Read_Ack(tp);
	   
			//printk("m_request_count = %d\n", m_request_count);
			m_fw_cmdAckValue = ret;
			if(ret == 0x9200 || ret == 0x92E0 || ret == 0x92EF)
			{
				if(ret == 0x92EF)
					strcpy(iap_fw_status, "finish");
				else{
					strcpy(iap_fw_status, "error");
					printk(KERN_ERR "%s FW Update Failed , ACK = %d\n",KLOG_NAME, ret);
				}
			#ifdef USE_WAKELOCK
				wake_unlock(&iap_wake_lock);
			#endif
			}
			else if(m_request_count == 0) {		
				strcpy(iap_fw_status, "continue");
			}
		}
		if(m_loop_write_flag) {
			wait_queue_flag = 1;
			wake_up_interruptible(&iap_wait_queue_head);
		}
		goto i2cReadErr_out;
	}
#endif
#ifdef I2C_CHECK_CODE
	if( m_i2c_check_mode==1 ) {
		m_i2c_check_currentTime = current_kernel_time();
		// do I2C read
		ret = waltop_I2C_read(tp);
		if( ret == -EAGAIN ){ // the error we concern
			m_i2c_check_errorCount++;
			m_i2c_check_IsReapeatErrorCount++;
			if(m_i2c_check_IsReapeatErrorCount>=m_i2c_check_repeatTime) {
				m_i2c_check_IsReapeatErrorCount=0;
				m_i2c_check_ContinueErrorCount++;
			}
			reset_i2c();
			udelay(200);
		}
		else {
			if( m_i2c_check_IsReapeatErrorCount > 0 )
				m_i2c_check_IsReapeatErrorCount = 0;
		}
		// add total count
		m_i2c_check_totalCount++;
		// Reset when timeout
		if( (m_i2c_check_currentTime.tv_sec - m_i2c_check_beginTime.tv_sec)>m_i2c_check_time ) {
			do_reset_pen();
		}
		goto report_point;
	} else if (m_i2c_count_mode==1){
        m_i2c_check_currentTime = current_kernel_time();
		// do I2C read
		ret = waltop_I2C_read(tp);
		if( ret == -EAGAIN ){ // the error we concern
			m_i2c_check_errorCount++;
			m_i2c_check_IsReapeatErrorCount++;
			if(m_i2c_check_IsReapeatErrorCount>=m_i2c_check_repeatTime) {
				m_i2c_check_IsReapeatErrorCount=0;
				m_i2c_check_ContinueErrorCount++;
			}
		}
		else {
			if( m_i2c_check_IsReapeatErrorCount > 0 )
				m_i2c_check_IsReapeatErrorCount = 0;
		}
		// add total count
		m_i2c_check_totalCount++;
		// Reset when timeout
		goto report_point;
    }
#endif

	/* do I2C read */
	ret = waltop_I2C_read(tp);
	if(ret == -EREMOTEIO){
		mdelay(1);
		ret = waltop_I2C_read(tp);//re-try again
	}

	if( ret < 0 ){ // some error
		goto i2cReadErr_out;
	} else { //checksum
		int Result = 0;
		sum = tp->pkt_data[1] + tp->pkt_data[2] + tp->pkt_data[3] + tp->pkt_data[4] + tp->pkt_data[5] + tp->pkt_data[6];
		Result = (sum & 0xff) ^ tp->pkt_data[7];
		if(Result > 0)
			goto i2cReadErr_out;
	}
	
	// Log read packet data from firmware for debug
	//printk(KERN_INFO "%x, %x, %x, %x,   %x, %x, %x, %x,   %x, %x, %x\n",
	//tp->pkt_data[0], tp->pkt_data[1], tp->pkt_data[2], tp->pkt_data[3],
	//tp->pkt_data[4], tp->pkt_data[5], tp->pkt_data[6], tp->pkt_data[7],
	//tp->pkt_data[8], tp->pkt_data[9], tp->pkt_data[10]);
		
	// do input_sync() here
	// ...
report_point:
	in_range = tp->pkt_data[6]&0x20;
	//printk(KERN_INFO "in_range=0x%x", in_range);

	/* report BTN_TOOL_PEN event depend on in range */
	input_report_key(inp, BTN_TOOL_PEN, in_range>0 ? 1:0);

	if( in_range )
	{
		x = ((tp->pkt_data[1] << 8) | tp->pkt_data[2]);
		y = ((tp->pkt_data[3] << 8) | tp->pkt_data[4]);
		ps = (((tp->pkt_data[6] & 0x03) << 8 ) | tp->pkt_data[5]);
		dv = (tp->pkt_data[6]&0x40);
	
		tip = (tp->pkt_data[6]&0x04);
		btn_low = (tp->pkt_data[6]&0x08);
		btn_up = (tp->pkt_data[6]&0x10);

		//printk(KERN_INFO "x=%d, y=%d, pressure=%d, in_range=0x%x, tip=0x%x, btn_low=0x%x, btn_up=%x , dv = %x\n",
		//x, y, ps, in_range, tip, btn_low, btn_up,dv);

		// <<<< 2012/11 mirror direction if x or y is opposite >>>>
		//x = tp->x_max - x;
		y = tp->y_max - y;
		//printk(KERN_INFO "x2=%d, y2=%d\n",x, y);

#ifdef REMAP_TO_LCD_SIZE
		// <<<< 2012/11 scale the resolution here if Android don't do it >>>>
		//x = x * LCD_SCREEN_MAX_X / (tp->x_max);
		//y = y * LCD_SCREEN_MAX_Y / (tp->y_max);
		// or
		x = x * LCD_SCREEN_MAX_Y / (tp->x_max);
		y = y * LCD_SCREEN_MAX_X / (tp->y_max);
#endif
		// Use standard single touch event
		// Report X, Y Value, <<<< 2012/11 swap x,y here if need >>>>
		//input_report_abs(inp, ABS_X, x);
		//input_report_abs(inp, ABS_Y, y);
		input_report_abs(inp, ABS_X, y);
		input_report_abs(inp, ABS_Y, x);

		// Report pressure and Tip as Down/Up
		if( dv && (ps > tp->p_minTipOn) ) {
			input_report_abs(inp, ABS_PRESSURE, ps);
			input_report_key(inp, BTN_TOUCH, 1);
		}
		else {
			input_report_abs(inp, ABS_PRESSURE, 0);
			input_report_key(inp, BTN_TOUCH, 0);
		}
		// Report side buttons on Pen
		input_report_key(inp, BTN_STYLUS, btn_low);
		input_report_key(inp, BTN_STYLUS2, btn_up);

	} else{
#ifdef I2C_CHECK_CODE
		if( (m_i2c_check_mode==0)&&(tp->pkt_data[6]&0x80) ) { // f/w in i2c_check_mode
			// Reset it for sync
			do_reset_pen();
		}
#endif
		input_report_abs(inp, ABS_PRESSURE, 0);
		input_report_key(inp, BTN_TOUCH, 0);
		input_report_key(inp, BTN_STYLUS, 0);
		input_report_key(inp, BTN_STYLUS2, 0);
        //printk(KERN_INFO "in_range=0x%x\n", in_range);
    }

	input_sync(inp);
	
i2cReadErr_out:
	if (tp->client->irq != 0) {
		enable_irq(tp->client->irq);
	}
	return;
}

static irqreturn_t waltop_I2C_irq(int irq, void *handle)
{
	struct waltop_I2C *tp = (struct waltop_I2C *) handle;

	dev_dbg(&(tp->client->dev), "%s : got irq!\n",__func__);

	//printk(KERN_INFO "waltop_I2C_irq in irq : %s\n", dev_name(&tp->client->dev));

	/* disable other irq until this irq is finished */
	disable_irq_nosync(tp->client->irq);
		
	/* schedule workqueue */
	queue_work(waltop_I2C_wq, &tp->work);

	return IRQ_HANDLED;
} 
 

/*****************************************************************************
 * Probe and Initialization functions
 ****************************************************************************/
static int waltop_I2C_init_sysfile(struct i2c_client *client, struct waltop_I2C *tp)
{
	int ret = 0;
	
	ret = sysfs_create_group(&client->dev.kobj, &waltop_I2C_attribute_group);
	if (ret) {
		dev_err(&(client->dev), "%s() - ERROR: sysfs_create_group() failed: %d\n", __func__, ret);
	}
	else {
		dev_err(&(client->dev), "%s() - sysfs_create_group() succeeded.\n", __func__);
	}

	ret = device_create_file(&tp->client->dev, &dev_attr_irq_enable);
	if (ret < 0) {
		dev_err(&(client->dev), "%s() - ERROR: File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto error_dev_create_file;
	}
#ifdef IAP_FWUPDATE
	ret = sysfs_create_bin_file(&client->dev.kobj, &waltop_I2C_fwdata_attributes);
	if (ret < 0) {
		dev_err(&(client->dev), "%s() - ERROR: Binary file attributes creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto error_sysfs_create_bin_file;
	}
#ifdef USE_WAKELOCK
	wake_lock_init(&iap_wake_lock, WAKE_LOCK_SUSPEND,"PenIAP_WakeLock");
    wake_lock_init(&pen_detect_wake_lock, WAKE_LOCK_SUSPEND,"PenDetect_WakeLock");
#endif
	return 0;

error_sysfs_create_bin_file:
	device_remove_file(&tp->client->dev, &dev_attr_irq_enable);
#else
	return 0;
#endif

error_dev_create_file:
	sysfs_remove_group(&client->dev.kobj, &waltop_I2C_attribute_group);
	
	return ret;
}

static int waltop_init_gpio(struct i2c_client *client,struct waltop_I2C *tp)
{
	int ret = 0;

    dev_dbg(&(client->dev),"%s() - waltop init gpio\n", __func__);

    ret = gpio_request(tp->pdata->gpio_reset, "EM_RST_N");
	if (ret) {
		dev_err(&(client->dev),"%s : failed to request gpio %d\n", __func__,
				tp->pdata->gpio_reset);
		goto err0;
	}
	gpio_direction_output(tp->pdata->gpio_reset, 1);

	ret = gpio_request(tp->pdata->gpio_int, "EM_INT");
	if (ret) {
		dev_err(&(client->dev),"%s : failed to request gpio %d\n", __func__,
				tp->pdata->gpio_int);
		goto err1;
	}
	gpio_direction_input(tp->pdata->gpio_int);

	ret = gpio_request(tp->pdata->gpio_pdct, "EM_PDCT");
	if (ret) {
		dev_err(&(client->dev),"%s : failed to request gpio %d\n", __func__,
				tp->pdata->gpio_pdct);
		goto err2;
	}
	gpio_direction_input(tp->pdata->gpio_pdct);

    gpio_direction_output(tp->pdata->gpio_reset, 0);
    mdelay(30);
    gpio_direction_output(tp->pdata->gpio_reset, 1);
    mdelay(50);
	
    goto out;

err2:
err1:

err0:

out:
	return ret;
}

void pen_detect_work(struct work_struct *work)
{
	struct waltop_I2C *tp = container_of(work, struct waltop_I2C,
					pen_det_work.work);

	if(wake_lock_active(&pen_detect_wake_lock)){
		wake_unlock(&pen_detect_wake_lock);
	}
	wake_lock_timeout(&pen_detect_wake_lock,300);
	mdelay(250);
	if(gpio_get_value(tp->pdata->gpio_pdct)!=m_pen_detected){
		force_release_pos(tp->client);
		if(m_second_pen_state==1)
			goto switch_state_change; // skip turn off chip step if second_pen is set.
	    disable_irq(tp->client->irq);
		if (gpio_get_value(tp->pdata->gpio_pdct)) {
	        gpio_direction_output(tp->pdata->gpio_reset, 1);
	        mdelay(50);
		} else {
		    gpio_direction_output(tp->pdata->gpio_reset, 0);
	        mdelay(30);
		}
	    enable_irq(tp->client->irq);
switch_state_change:
		m_pen_detected = gpio_get_value(tp->pdata->gpio_pdct);
		m_pen_insert_state = m_pen_detected ? 0 : 1 ;
		switch_set_state(&tp->pen_switch, m_pen_insert_state);
		printk(KERN_INFO "%s %s : Detected %d , Second Pen = %d" ,KLOG_NAME,__func__,m_pen_insert_state,m_second_pen_state);
	}else{
		printk(KERN_ERR "%s %s : Detected ignore" ,KLOG_NAME,__func__);
	}
}

static irqreturn_t waltop_pen_pdct(int irq, void *data)
{
    struct waltop_I2C *tp = (struct waltop_I2C *)data;
	queue_delayed_work(tp->pen_det_wq, &tp->pen_det_work, PEN_DELAY);

	return IRQ_HANDLED;
}

static int waltop_I2C_initialize(struct i2c_client *client, struct waltop_I2C *tp, struct waltop_platform_data *pfdata)
{
	struct input_dev *input_device;
	int ret = 0;

	/* create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device)
	{
		ret = -ENOMEM;
		dev_err(&(client->dev), "%s() - ERROR: Could not allocate input device.\n", __func__);
		goto error_free_device;
	}

	tp->client = client;
	tp->input = input_device;

	// 2013/01/30, Martin add device information
	ret = waltop_I2C_readDeviceInfo(tp);
	if( ret>0 ) { 
		tp->x_max = ((tp->pkt_data[1] << 8) | tp->pkt_data[2]);
		tp->y_max = ((tp->pkt_data[3] << 8) | tp->pkt_data[4]);
		tp->p_max = ((tp->pkt_data[6]&0x80)>>7)|((tp->pkt_data[7]&0x80)>>6);
		tp->p_max = ((tp->p_max << 8) | (tp->pkt_data[5]));
		tp->fw_version = ((tp->pkt_data[6]&0x7F)*100) + (tp->pkt_data[7]&0x7F);
        printk(KERN_ERR "waltop_I2C_readDeviceInfo() - firmware version : %d , Max Pressure : %d \n", tp->fw_version ,tp->p_max);
	}
	else {
		tp->x_max = pfdata->x_max ? pfdata->x_max : WALTOP_MAX_X;
		tp->y_max = pfdata->y_max ? pfdata->y_max : WALTOP_MAX_Y;
		tp->p_max = pfdata->p_max ? pfdata->p_max : WALTOP_MAX_P;
		tp->fw_version = 0; // default
        printk(KERN_ERR "waltop_I2C_readDeviceInfo() ERROR - firmware version : %d , Max Pressure : %d \n", tp->fw_version ,tp->p_max);
	}
	tp->p_minTipOn = pfdata->p_minTipOn ? pfdata->p_minTipOn : 0;
	waltop_fw_version =  tp->fw_version;
    if(tp->p_max==0)
        tp->p_max = WALTOP_MAX_P;
    /* Prepare worker structure prior to set up the timer/ISR */
	INIT_WORK(&tp->work, waltop_I2C_worker);
	atomic_set(&tp->irq_enabled, 1);
	
	/* set input device information */
	snprintf(tp->phys, sizeof(tp->phys), "%s/input0", dev_name(&client->dev));

	input_device->phys = tp->phys;
	input_device->dev.parent = &client->dev;
	input_device->name = "waltop";
	input_device->id.bustype = BUS_I2C;

	/* for example, read product name and version from f/w to fill into id information */
	input_device->id.vendor  = 0x172f;
	input_device->id.product = 0x0100;	// Module code xxxx
	input_device->id.version = tp->fw_version;	// should this be fw_version?

	/* set wakeup but disabled */
	/* for example, suspend after timeout, because we are disabled */
	//waltop_I2C_disable(tp);

	input_set_drvdata(input_device, tp);

	// Use standard single touch event
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_SYN, input_device->evbit);
	
	set_bit(ABS_X, input_device->absbit);
    set_bit(ABS_Y, input_device->absbit);
    set_bit(ABS_PRESSURE, input_device->absbit);
	
	set_bit(BTN_TOOL_PEN, input_device->keybit);
	set_bit(BTN_TOUCH, input_device->keybit);	
	set_bit(BTN_STYLUS, input_device->keybit);
	set_bit(BTN_STYLUS2, input_device->keybit);


	// Set ABS_X, ABS_Y as module's resolution
#ifdef REMAP_TO_LCD_SIZE
	// <<<< 2012/11 scaling the resolution here if Android don't do it >>>>
	input_set_abs_params(input_device, ABS_X, 0, LCD_SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, LCD_SCREEN_MAX_Y, 0, 0);
#else
	input_set_abs_params(input_device, ABS_X, 0, tp->x_max, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, tp->y_max, 0, 0);
	// or
#endif
	input_set_abs_params(input_device, ABS_PRESSURE, 0, tp->p_max, 0, 0);

	ret = input_register_device(input_device);
	if (0 != ret) {
		printk(KERN_ERR "input_register_device() - Error:%s:%d\n", __FILE__, __LINE__);
		goto error_free_device;
	}
		
	/* interrupt setup */
	if (tp->client->irq)
	{		
		/* enable irq */
		ret = request_irq(tp->client->irq, waltop_I2C_irq, IRQF_TRIGGER_FALLING, input_device->name, tp);
		if (ret)
		{
			dev_err(&(client->dev), "%s() - ERROR: Could not request IRQ: %d\n", __func__, ret);
			goto error_free_irq;
		}
	}

       /* digitizer register switch class */
	tp->digitizer_switch.name = "digitizer";
	tp->digitizer_switch.print_name = waltop_digitizer_switch_name;
	ret = switch_dev_register(&tp->digitizer_switch);
	if(ret < 0){
		printk (KERN_ERR "%s : Could not register switch device, ret = %d\n", KLOG_NAME,ret);
		}
	switch_set_state(&tp->digitizer_switch, 0);

	/* pen_insert register switch class */
	tp->pen_det_wq = create_singlethread_workqueue("pen_det_wq");
	INIT_DELAYED_WORK(&tp->pen_det_work, pen_detect_work);
	tp->pen_switch.name = NAME_PEN_INSERT;
	tp->pen_switch.print_state = print_pen_insert_state;
	ret = switch_dev_register(&tp->pen_switch);
	if (ret < 0) {
		printk (KERN_ERR "%s : Could not register switch device, ret = %d\n", KLOG_NAME,ret);
		m_pen_detected = 0;
	} else
		m_pen_detected = gpio_get_value(tp->pdata->gpio_pdct);

	/* Pen_detect interrupt setup */

	ret = request_threaded_irq(tp->pdct_irq, NULL, waltop_pen_pdct,
					IRQF_TRIGGER_RISING
					| IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"waltop_pdct", tp);

	if (ret < 0) {
	    dev_err(&(client->dev),"%s: failed to request thread for pdct irq gpio\n", __func__);
		m_pen_detected = 0;
	}else{
		m_pen_detected = gpio_get_value(tp->pdata->gpio_pdct);
	}

	m_pen_insert_state = m_pen_detected ? 0 :1;
	switch_set_state(&tp->pen_switch, m_pen_insert_state);

#ifdef ANDROID_DIGITIZER_SYS_FS
    // register sysfs node for path(/sys/android_digitizer)
    waltop_digitizer_sysfs_init();
#endif

	// Create SYSFS related file
	ret = waltop_I2C_init_sysfile(client, tp);
	if (ret < 0) {
		goto error_free_irq;
	}

	i2c_set_clientdata(client, tp);
	goto succeed;
    
error_free_irq:
	free_irq(tp->client->irq, tp);
error_free_device:
	if (input_device)
	{
		input_free_device(input_device);
		tp->input = NULL;
	}
	kfree(tp);
succeed:
	return ret;
}

static int waltop_I2C_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct waltop_I2C *tp;
	int err , ret=-1;

	//struct waltop_platform_data *pfdata = dev->platform_data;
	tp = kzalloc(sizeof(struct waltop_I2C), GFP_KERNEL);
	tp->pdata = client->dev.platform_data;

	printk(KERN_ERR "waltop_I2C_probe-%s:%d\n", __FILE__, __LINE__);
	mdelay(5);
		
	// check platform data
	if (!tp->pdata) {
		printk(KERN_ERR "no platform data?-%s:%d\n", __FILE__, __LINE__);
		err = -EINVAL;
		goto err_out;
	}
	else {
		printk(KERN_INFO "platform data?-%s:%d\n", __FILE__, __LINE__);
		printk(KERN_INFO "xmax=%d, ymax =%d\n", tp->pdata->x_max, tp->pdata->y_max);
	}

	// check functionality
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

    //tp = kzalloc(sizeof(struct waltop_I2C), GFP_KERNEL);
    if (NULL == tp)
    {
        dev_err(&(client->dev), "%s() - ERROR: Could not allocate %d bytes of kernel memory for ft5x06 struct.\n", __func__, sizeof(struct waltop_I2C));
        err = -ENOMEM;
        goto error_devinit0;
    } 
    
    err = waltop_init_gpio(client,tp);	
    if (0 > err) {
		dev_err(&(client->dev),"%s: failed to initialize gpio\n", __func__);
		goto err_out;
	}
    
    client->irq = gpio_to_irq(tp->pdata->gpio_int);
    if (client->irq < 0) {
		dev_err(&(client->dev),"%s: failed to enable irq gpio\n", __func__);
		goto err_out;
	}

    tp->pdct_irq = gpio_to_irq(tp->pdata->gpio_pdct);
    if (tp->pdct_irq < 0) {
		dev_err(&(client->dev),"%s: failed to enable pdct irq gpio\n", __func__);
		goto err_out;
	}

	err = irq_set_irq_wake(tp->pdct_irq, 1);
	if (err  < 0) {
        dev_err(&(client->dev), "%s() - ERROR: waltop_I2C tp->pdct_irq enable failed.\n", __func__);
		goto err_out;
	}
    
	err = enable_irq_wake(tp->pdct_irq);
	if (err  < 0) {
        dev_err(&(client->dev), "%s() - ERROR: waltop_I2C tp->pdct_irq enable failed.\n", __func__);
		goto err_out;
	}
	
    // Need to initialize the SYSFS mutex before creating the SYSFS entries in waltop_I2C_initialize().
    mutex_init(&tp->mutex);
    err = waltop_I2C_initialize(client, tp, tp->pdata);
    if (0 > err)
    {
        dev_err(&(client->dev), "%s() - ERROR: waltop_I2C could not be initialized.\n", __func__);
        goto error_mutex_destroy;
    }

    private_tp = tp;

#ifdef CONFIG_HAS_WALTOP_EARLYSUSPEND
	tp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	tp->early_suspend.suspend = waltop_early_suspend;
	tp->early_suspend.resume = waltop_late_resume;
	register_early_suspend(&tp->early_suspend);
#endif

	if(check_fw_version(tp)){
		ret = firmware_update_header(client, digitizer_firmware, sizeof(digitizer_firmware)/UF_FWPAGESIZE);(client);
		if (0 > ret)
		{
		    dev_err(&(client->dev), "%s - ERROR , firmware update failed\n", KLOG_NAME);

		}
	} else {
		printk(KERN_INFO "%s No need to update Firmware", KLOG_NAME);
	}
	goto succeed;

error_mutex_destroy:
    mutex_destroy(&tp->mutex);        
error_devinit0:    	
err_out:
succeed:
	return err;	
}


static int waltop_I2C_remove(struct i2c_client *client)
{
	struct waltop_I2C *tp = i2c_get_clientdata(client);

#ifdef ANDROID_DIGITIZER_SYS_FS
    waltop_digitizer_sysfs_deinit();
#endif
	
	dev_info(&(client->dev), "%s() - Driver is unregistering.\n", __func__);

	device_remove_file(&tp->client->dev, &dev_attr_irq_enable);
    if(tp->client->irq != 0)
    {
		free_irq(client->irq, tp);
	}
	input_unregister_device(tp->input);	

    mutex_lock(&tp->mutex);
    /* Remove the SYSFS entries */
#ifdef IAP_FWUPDATE
    sysfs_remove_bin_file(&client->dev.kobj, &waltop_I2C_fwdata_attributes);
#ifdef USE_WAKELOCK
	wake_unlock(&iap_wake_lock);
	wake_lock_destroy(&iap_wake_lock);
	wake_unlock(&pen_detect_wake_lock);
	wake_lock_destroy(&pen_detect_wake_lock);
#endif
#endif
    sysfs_remove_group(&client->dev.kobj, &waltop_I2C_attribute_group);
    mutex_unlock(&tp->mutex);
    mutex_destroy(&tp->mutex);
    
	kfree(tp);
	
    dev_info(&(client->dev), "%s() - Driver unregistration is complete.\n", __func__);
    
	return 0;
}

static int __init waltop_I2C_init(void)
{
	int ret = 0;

    printk(KERN_INFO "%s() - Waltop I2C Pen Driver (Built %s @ %s)\n", __func__, __DATE__, __TIME__);

    waltop_I2C_wq = create_singlethread_workqueue("waltop_I2C_wq");
    if (NULL == waltop_I2C_wq)
    {
        printk(KERN_ERR "%s() - ERROR: Could not create the Work Queue due to insufficient memory.\n", __func__);
        ret = -ENOMEM;
    }
    else
    {
        ret = i2c_add_driver(&waltop_I2C_driver);
        
        printk(KERN_ERR "waltop_I2C_init-%s:%d\n", __FILE__, __LINE__);
		mdelay(5);
    }

    return ret;
}

static void __exit waltop_I2C_exit(void)
{
	if (waltop_I2C_wq)
    {
        destroy_workqueue(waltop_I2C_wq);
    }

    return i2c_del_driver(&waltop_I2C_driver);
}


module_init(waltop_I2C_init);
module_exit(waltop_I2C_exit); 

MODULE_AUTHOR("Waltop");
MODULE_DESCRIPTION("Waltop I2C pen driver");
MODULE_LICENSE("GPL");
