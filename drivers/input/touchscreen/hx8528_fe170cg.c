/* Himax Android Driver Sample Code Ver 1.9
 *
 * Copyright (C) 2012 Himax Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/usb/penwell_otg.h>
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/HWVersion.h>
/******************** Himax: include .h file ********************/
#include <linux/hx8528_fe170cg.h>

/******************** Himax: include .h file ********************/

//#include "hx8528_fe170cg_cfg.h"

/******************** Himax: Function Define ********************/
#define HX_TP_FW_UPDATE 			//Support Firmware update
#define HX_TP_SYS_FS					//Support Debug Tool
#define HX_TP_SYS_FLASH_DUMP	//Support Flash dump function
#define HX_TP_COORDINATE_DUMP	//Support Coordinate dump function
#define HX_TP_SYS_HITOUCH
#define HX_RST_PIN_FUNC
#define HX_FW_UPDATE_BY_I_FILE
//Select one, one time.
//#define HX_85XX_A_SERIES_PWON
//#define HX_85XX_B_SERIES_PWON
//#define HX_85XX_C_SERIES_PWON
#define HX_85XX_D_SERIES_PWON

#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"
/******************** Himax: Function Define ********************/

/******************** Himax: define and inital setting********************/
/********** 1. About fundamental define and setting **********/
#define HX_TP_MAX_FINGER             10		// Support MAX points
#define HX_TOUCH_INFO_VKEY           22		// Virtual Key address at HX_TOUCH_INFO_SIZE
#define HX_TOUCH_INFO_ID_1_INFO      53
#define HX_TOUCH_INFO_ID_2_INFO      54
#define HX_TOUCH_INFO_POINT_CNT      52     //Point Count address at Himax Touch Controller reported data
//#define DEFAUULT_X_RES               600		// Support X resolution
//#define DEFAUULT_Y_RES               1024		// Support Y resolution
#define HX_KEY_MAX_COUNT             4      //max virtual keys
#define HX_TP_X_CH_NUM               30         //set TP X channel for debug data
#define HX_TP_Y_CH_NUM               18         //set TP Y channel for debug data
#define HX_KEY_COUNT                 4      //number of virtual keys
//#define HX_LOADIN_CONFIG									// Support Common FW,load in config

#ifdef HX_RST_PIN_FUNC
#define ENABLE_CHIP_RESET_MACHINE
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE
#define HX_TP_SYS_RESET										// Support Sys : HW Reset function			,default is open
//#define ENABLE_CHIP_STATUS_MONITOR			// Support Polling ic status            ,default is close
#endif

/******************** Himax: Firmware bin Checksum ***********************/
//Select one,one time
//#define HX_TP_BIN_CHECKSUM_SW
//#define HX_TP_BIN_CHECKSUM_HW
#define HX_TP_BIN_CHECKSUM_CRC

//array for vritaul key value
#define HX_VKEY_0   KEY_MENU
#define HX_VKEY_1   KEY_BACK
#define HX_VKEY_2   KEY_HOMEPAGE
#define HX_VKEY_3   104
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

// Himax TP COMMANDS -> Do not modify the below definition
#define HX_CMD_NOP                   0x00   /* no operation */
#define HX_CMD_SETMICROOFF           0x35   /* set micro on */
#define HX_CMD_SETROMRDY             0x36   /* set flash ready */
#define HX_CMD_TSSLPIN               0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT              0x81   /* set sleep out */
#define HX_CMD_TSSOFF                0x82   /* sense off */
#define HX_CMD_TSSON                 0x83   /* sense on */
#define HX_CMD_ROE                   0x85   /* read one event */
#define HX_CMD_RAE                   0x86   /* read all events */
#define HX_CMD_RLE                   0x87   /* read latest event */
#define HX_CMD_CLRES                 0x88   /* clear event stack */
#define HX_CMD_TSSWRESET             0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD   /* set cache function */
#define HX_CMD_SETIDLE               0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3   /* set idle delay */

#define HX_CMD_SELFTEST_BUFFER       0x8D   /* Self-test return buffer */

#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F
#define HX_CMD_B9                    0xB9
#define HX_CMD_76                    0x76

#define DEFAULT_RETRY_CNT            3		// For I2C R/W
#define CONFIG_HAS_EARLYSUSPEND

//Himax Button function Enable/Disable
//#define HX_EN_BUTTON
//#define Gesture_Detemination

//Himax RST PIN FUNCTION

#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1
#define USB_SHIFT 0
#define AC_SHIFT 1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)

/***********Himax Test Define*********/
//#define HX_CHECK_I2C_PATH
//#define HX_READ_FW_VERSION
//#define HX_ISR_ENTER_CHECK
//#define HX_ENABLE_EDGE_TRIGGER


//nick_hsiao:id of TP
#define TP_WINTEK	0
#define TP_OFILM 2
//#define TP_TRULY_OGS	2

#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;	//0-->close ; 1--> far away
static u8 last_tpd_proximity_detect = 1;	//0-->close ; 1--> far away
#define PS_INIT_POLL_MAX_COUNT 10
static int himax_ps_irq_data_count = 0;
static int himax_ps_poll_time = 200; // ms , 5hz
static int himax_ps_min_poll_time = 200; // ms , 5hz
static int tpd_enable_ps(int);
#endif

#define TOUCH_DC "1"
#define TOUCH_IIR "2"
#define TOUCH_BANK "3"

extern int entry_mode;
extern int Read_HW_ID(void);
static int HW_ID;

struct himax_ts_data
{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct workqueue_struct *himax_wq;
    struct work_struct work;
    struct work_struct handshaking_work;
#ifdef HX_TP_SYS_FLASH_DUMP
    struct workqueue_struct *flash_wq;
    struct work_struct flash_work;
#endif
    int (*power)(int on);
    struct early_suspend early_suspend;
    int intr_gpio;
    // Firmware Information
    int fw_ver;
    int fw_id;
    int x_resolution;
    int y_resolution;
    // For Firmare Update
    struct miscdevice firmware;
    struct attribute_group attrs;
    int status;
    struct switch_dev touch_sdev;
    int abs_x_max;
    int abs_y_max;
    int rst_gpio;
    int tp_pwr_gpio;
    struct regulator *vdd;

// Wakelock Protect start
    struct wake_lock wake_lock;
// Wakelock Protect end

// Mutexlock Protect Start
    struct mutex mutex_lock;
// Mutexlock Protect End

// ESD Workaround start
    int init_success;
    int retry_time;

#ifdef ENABLE_CHIP_RESET_MACHINE
    struct delayed_work himax_chip_reset_work;
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
    struct delayed_work himax_chip_monitor;
    int running_status;
#endif
// ESD Workaround end

};

/************************* ESD Workaround Start **********************/
#define ESD_WORKAROUND 1

#ifdef ESD_WORKAROUND
static u8 	reset_activate = 1;
u8 			ESD_COUNTER = 0;
int 		ESD_COUNTER_SETTING = 3;
void ESD_HW_REST(void);
#endif

uint8_t 	IC_STATUS_CHECK = 0xAA;

static int himax_hang_shaking(void);
int himax_touch_cable_status(int status);
/************************* ESD Workaround End ************************/

static struct himax_ts_data *private_ts = NULL;
static struct semaphore pSem;
static int tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY;

struct i2c_client *touch_i2c = NULL;
static int himax_debug_flag = 1;

static uint8_t rFE96_setting[8] = { 0x02, 0x58, 0x17, 0x10, 0x35, 0x07, 0x37, 0x0C}; //add by TM for init self_test FE96
static int self_test_delay_time = 3; //add by TM for init self_test delay time(s)
static bool diag_dump_to_gauge = false;

static ssize_t himax_self_test_setting(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);
static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf);
static int himax_chip_self_test(void);
static int himax_ts_poweron(struct himax_ts_data *ts_modify);
static ssize_t himax_diag_dump_func(const char *buf);
static ssize_t himax_chip_raw_data_store_func(char *buf, char *PreFilePath);
#ifdef HX_LOADIN_CONFIG
static int himax_config_flow();
#endif

/********** 1. About fundamental define and setting **********/

/********** 2. About Firmware update define and setting **********/
// Himax: Set FW and CFG Flash Address
#ifdef HX_85XX_A_SERIES_PWON //20
#endif

#ifdef HX_85XX_B_SERIES_PWON //26
#define FW_VER_MAJ_FLASH_ADDR		133			//0x0085
#define FW_VER_MAJ_FLASH_LENG		1
#define FW_VER_MIN_FLASH_ADDR		728			//0x02D8
#define FW_VER_MIN_FLASH_LENG		1
#define CFG_VER_MAJ_FLASH_ADDR		692			//0x02B4
#define CFG_VER_MAJ_FLASH_LENG		3
#define CFG_VER_MIN_FLASH_ADDR		704			//0x02C0
#define CFG_VER_MIN_FLASH_LENG		3
#endif

#ifdef HX_85XX_C_SERIES_PWON //31
#endif

#ifdef HX_85XX_D_SERIES_PWON //28 D0
#define FW_VER_MAJ_FLASH_ADDR       133         //0x0085
#define FW_VER_MAJ_FLASH_LENG       1
#define FW_VER_MIN_FLASH_ADDR       134         //0x0086
#define FW_VER_MIN_FLASH_LENG       1
#define CFG_VER_MAJ_FLASH_ADDR      160         //0x00A0
#define CFG_VER_MAJ_FLASH_LENG      12
#define CFG_VER_MIN_FLASH_ADDR      172         //0x00AC
#define CFG_VER_MIN_FLASH_LENG      12
#endif

static u16 FW_VER_MAJ_buff[FW_VER_MAJ_FLASH_LENG];
static u16 FW_VER_MIN_buff[FW_VER_MIN_FLASH_LENG];
static u16 CFG_VER_MAJ_buff[CFG_VER_MAJ_FLASH_LENG];
static u16 CFG_VER_MIN_buff[CFG_VER_MIN_FLASH_LENG];

static unsigned char FW_VER_MAJ_FLASH_buff[FW_VER_MAJ_FLASH_LENG*4];
static unsigned char FW_VER_MIN_FLASH_buff[FW_VER_MIN_FLASH_LENG*4];
static unsigned char CFG_VER_MAJ_FLASH_buff[CFG_VER_MAJ_FLASH_LENG*4];
static unsigned char CFG_VER_MIN_FLASH_buff[CFG_VER_MIN_FLASH_LENG*4];

// For Firmware Update
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
//static int work_lock=0x00;

//for virtual key
//static int tpd_key = 0;
//static int tpd_key_old = 0xFF;

static int hx_point_num = 0;
static int p_point_num = 0xFFFF;
static int tpd_key = 0;
static int tpd_key_old = 0xFF;
static int Dist_Cal_EX = 0xFFFF;
static int Dist_Cal_Now = 0xFFFF;
static int ZoomInCnt = 0;
static int ZoomOutCnt = 0;

//----[HX_FW_UPDATE_BY_I_FILE]--------------------------------------------------------------------------start
#ifdef HX_FW_UPDATE_BY_I_FILE
static struct task_struct		*i_update_firmware_tsk;
static bool i_Needupdate = true;
static unsigned char i_isTP_Updated = 0;
static unsigned char* i_CTPM_FW;
static unsigned char i_CTPM_FW_Wintek_ER[]=
{
#include "hx8528_fw_fe170cg_wintek_ER.h"
};
static unsigned char i_CTPM_FW_Wintek_MP[]=
{
#include "hx8528_fw_fe170cg_wintek_MP.h"
};
static unsigned char i_CTPM_FW_Ofilm_ER[]=
{
#include "hx8528_fw_fe170cg_ofilm_ER.h"
};
static unsigned char i_CTPM_FW_Ofilm_MP[]=
{
#include "hx8528_fw_fe170cg_ofilm_MP.h"
};
#endif
//----[HX_FW_UPDATE_BY_I_FILE]----------------------------------------------------------------------------end

#ifdef HX_TP_SYS_HITOUCH
static int	hitouch_command			= 0;
static bool hitouch_is_connect	= false;
#endif

unsigned char SFR_3u_1[16][2] = {{0x18,0x06},{0x18,0x16},{0x18,0x26},{0x18,0x36},{0x18,0x46},
    {0x18,0x56},{0x18,0x66},{0x18,0x76},{0x18,0x86},{0x18,0x96},
    {0x18,0xA6},{0x18,0xB6},{0x18,0xC6},{0x18,0xD6},{0x18,0xE6},
    {0x18,0xF6}
};

unsigned char SFR_6u_1[16][2] = {{0x98,0x04},{0x98,0x14},{0x98,0x24},{0x98,0x34},{0x98,0x44},
    {0x98,0x54},{0x98,0x64},{0x98,0x74},{0x98,0x84},{0x98,0x94},
    {0x98,0xA4},{0x98,0xB4},{0x98,0xC4},{0x98,0xD4},{0x98,0xE4},
    {0x98,0xF4}
};
/********** 2. About Firmware update define and setting **********/

// Himax Feature Support

#define ABS_MT_POSITION         0x2a    /* Group a set of X and Y */
#define ABS_MT_AMPLITUDE        0x2b    /* Group a set of Z and W */

/********** 3. About Debug tool define and setting **********/
//#define RAW_DATA_LENGTH_PER_PKT      (128 - HX_TOUCH_INFO_SIZE)
#define DEFAULT_X_CHANNEL            HX_TP_X_CH_NUM   /* face the TS, x-axis */
#define DEFAULT_Y_CHANNEL            HX_TP_Y_CH_NUM   /* face the TS, y-axis */

#ifdef HX_TP_SYS_FS
static uint8_t *getMutualBuffer(void);
static void setMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDebugLevel(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);
static int himax_touch_sysfs_init(void);
static void himax_touch_sysfs_deinit(void);
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *getFlashBuffer(void);
static void setFlashBuffer(void);
static uint8_t getFlashCommand(void);
static uint8_t getFlashDumpComplete(void);
static uint8_t getFlashDumpFail(void);
static uint8_t getFlashDumpProgress(void);
static uint8_t getFlashReadStep(void);
static uint8_t getSysOperation(void);
static uint8_t getFlashDumpSector(void);
static uint8_t getFlashDumpPage(void);
static bool	   getFlashDumpGoing(void);
static void setFlashCommand(uint8_t command);
static void setFlashReadStep(uint8_t step);
static void setFlashDumpComplete(uint8_t complete);
static void setFlashDumpFail(uint8_t fail);
static void setFlashDumpProgress(uint8_t progress);
static void setSysOperation(uint8_t operation);
static void setFlashDumpSector(uint8_t sector);
static void setFlashDumpPage(uint8_t page);
static void setFlashDumpGoing(bool going);
#endif
static struct kobject *android_touch_kobj = NULL;

static uint8_t	register_command			= 0;
static uint8_t	multi_register_command		= 0;
static uint8_t	multi_register[8]			= {0x00};
static uint8_t	multi_cfg_bank[8]			= {0x00};
static uint8_t	multi_value[1024]			= {0x00};
static bool		config_bank_reg				= false;

static uint8_t	debug_log_level			= 0;
static bool		fw_update_complete		= false;
static bool		irq_enable				= false;
static int		handshaking_result		= 0;
static unsigned char debug_level_cmd	= 0;
static uint8_t x_channel = DEFAULT_X_CHANNEL; /* x asix, when you face the top of TS */
static uint8_t y_channel = DEFAULT_Y_CHANNEL; /* y asix, when you face the top of TS  */
static uint8_t *diag_mutual = NULL;
static uint8_t diag_command = 0;
static uint8_t diag_coor[128];// = {0xFF};
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *flash_buffer = NULL;
static uint8_t flash_command = 0;
static uint8_t flash_read_step = 0;
static uint8_t flash_progress = 0;
static uint8_t flash_dump_complete = 0;
static uint8_t flash_dump_fail = 0;
static uint8_t sys_operation = 0;
static uint8_t flash_dump_sector		= 0;
static uint8_t flash_dump_page			= 0;
static bool    flash_dump_going			= false;
#endif
#ifdef HX_EN_BUTTON
static uint8_t diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL+HX_KEY_COUNT] = {0};
#else
static uint8_t diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL] = {0};
#endif
#ifdef HX_TP_COORDINATE_DUMP
static uint8_t coordinate_dump_enable = 0;
static uint8_t coordinate_dump_file_create = 0;
struct file *fn;
#endif
#endif
/********** 3. About Debug tool define and setting **********/

static unsigned int TP_ID = TP_WINTEK;//0xFF; KK porting
extern int Read_TP_ID(void);

static bool isTouchSuspend = false;
static bool isUSBSuspendNotify = false;
struct wake_lock wakelock_detect_cable;
//extern unsigned int query_cable_status(void); KK porting

static char handshaking_fail_result[200];
static char handshaking_fail_result_tmp[200];
static int handshaking_fail_result_count = 0;

/******************** Himax: I2C R/W function ********************/
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry;
    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &command,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        }
    };

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 2) == 2)
            break;
        msleep(10);
    }
    if (retry == toRetry)
    {
        printk(KERN_INFO "[TP] %s: i2c_read_block retry over %d\n", __func__,
               toRetry);
        return -EIO;
    }
    return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry, loop_i;
    uint8_t *buf = kzalloc(sizeof(uint8_t)*(length+1), GFP_KERNEL);

    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    buf[0] = command;
    for (loop_i = 0; loop_i < length; loop_i++)
        buf[loop_i + 1] = data[loop_i];

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry)
    {
        printk(KERN_ERR "[TP] %s: i2c_write_block retry over %d\n", __func__,
               toRetry);
        kfree(buf);
        return -EIO;
    }
    kfree(buf);
    return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
    return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry, loop_i;
    uint8_t *buf = kzalloc(sizeof(uint8_t)*length, GFP_KERNEL);

    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = buf,
        }
    };

    for (loop_i = 0; loop_i < length; loop_i++)
        buf[loop_i] = data[loop_i];

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry)
    {
        printk(KERN_ERR "[TP] %s: i2c_write_block retry over %d\n", __func__,
               toRetry);
        kfree(buf);
        return -EIO;
    }
    kfree(buf);
    return 0;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
    int retry;
    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        }
    };

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }
    if (retry == toRetry)
    {
        printk(KERN_INFO "[TP] %s: i2c_read_block retry over %d\n", __func__,
               toRetry);
        return -EIO;
    }
    return 0;
}
/******************** Himax: I2C R/W function ********************/

/******************** Himax: ESD Workaround function start ***************/
#ifdef ESD_WORKAROUND
void ESD_HW_REST(void)
{
    reset_activate = 1;
    ESD_COUNTER = 0;

    printk("Himax TP: ESD - Reset\n");

    //Wakelock Protect start
    wake_lock(&private_ts->wake_lock);
    //Wakelock Protect end

    //Mutexlock Protect Start
    mutex_lock(&private_ts->mutex_lock);
    //Mutexlock Protect End

    gpio_set_value(private_ts->rst_gpio, 0);
    msleep(30);
    gpio_set_value(private_ts->rst_gpio, 1);
    msleep(30);

    //Mutexlock Protect Start
    mutex_unlock(&private_ts->mutex_lock);
    //Mutexlock Protec End

    himax_ts_poweron(private_ts);

    //Wakelock Protect start
    wake_unlock(&private_ts->wake_lock);
    //Wakelock Protect end


    //himax_touch_cable_status(cable_status);

    if( tpd_proximity_flag == 1)
    {
        tpd_enable_ps(1);
    }
    /*//Joe Test ++
    if(gpio_get_value(private_ts->intr_gpio) == 0)
    {
        printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
        enable_irq(private_ts->client->irq);
    }
    //Joe Test --*/
}
#endif

int himax_hang_shaking(void)    //0:Running, 1:Stop, 2:I2C Fail
{
    int ret, result;
    uint8_t hw_reset_check[1];
    uint8_t hw_reset_check_2[1];
    uint8_t buf0[2];

    struct timeval start;
    struct tm broken;
    do_gettimeofday(&start);
    time_to_tm(start.tv_sec, 0, &broken);
    if(handshaking_fail_result_count > 129)
    {
        sprintf(handshaking_fail_result_tmp,"%s",handshaking_fail_result+26);
        strcpy(handshaking_fail_result,handshaking_fail_result_tmp);
        handshaking_fail_result_count=handshaking_fail_result_count-26;
    }

    //Mutexlock Protect Start
    mutex_lock(&private_ts->mutex_lock);
    //Mutexlock Protect End

    //Write 0x92
    buf0[0] = 0x92;
    if(IC_STATUS_CHECK == 0xAA)
    {
        buf0[1] = 0xAA;
        IC_STATUS_CHECK = 0x55;
    }
    else
    {
        buf0[1] = 0x55;
        IC_STATUS_CHECK = 0xAA;
    }

    ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",__LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    msleep(15); //Must more than 1 frame

    buf0[0] = 0x92;
    buf0[1] = 0x00;
    ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",__LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    msleep(2);

    ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check, 1, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
        goto work_func_send_i2c_msg_fail;
    }
    //printk("[Himax]: ESD 0xDA - 0x%x.\n", hw_reset_check[0]);

    if(hw_reset_check[0] == 0x00)
    {
        result = 1; //MCU Stop
    }
    else if((IC_STATUS_CHECK != hw_reset_check[0]))
    {
        msleep(2);
        ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
        if(ret < 0)
        {
            printk(KERN_ERR "[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
            goto work_func_send_i2c_msg_fail;
        }
        //printk("[Himax]: ESD check 2 0xDA - 0x%x.\n", hw_reset_check_2[0]);

        if(hw_reset_check[0] == hw_reset_check_2[0])
        {
            result = 1; //MCU Stop
        }
        else
        {
            result = 0; //MCU Running
        }
    }
    else
    {
        result = 0; //MCU Running
    }

    //clear the 0xDA
    buf0[0] = 0xD1;
    buf0[1] = 0x00;
    ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[Himax]:write 0xDA failed line: %d \n",__LINE__);
        goto work_func_send_i2c_msg_fail;
    }

    //Mutexlock Protect Start
    mutex_unlock(&private_ts->mutex_lock);
    //Mutexlock Protect End

    if(result == 1)
    {
        handshaking_fail_result_count += sprintf(handshaking_fail_result + handshaking_fail_result_count, " %02d:%02d:%02d:%03d -%s", broken.tm_hour, broken.tm_min, broken.tm_sec, start.tv_usec/1000," MCU Stop \n");
    }

    return result;

work_func_send_i2c_msg_fail:
    //Mutexlock Protect Start
    mutex_unlock(&private_ts->mutex_lock);
    //Mutexlock Protect End
    if(result == 2)
    {
        handshaking_fail_result_count += sprintf(handshaking_fail_result + handshaking_fail_result_count, " %02d:%02d:%02d:%03d -%s", broken.tm_hour, broken.tm_min, broken.tm_sec, start.tv_usec/1000," I2C Fail \n");
    }
    return 2;
}

void touch_callback_himax_hang_shaking()
{
    //reject callback before probing successfully
    if(private_ts == NULL || private_ts->init_success != 1 || entry_mode != 1)
    {
        return;
    }

    int ret = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail
    if(ret == 2)
    {
#ifdef ENABLE_CHIP_RESET_MACHINE
        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
        printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
    }
    if(ret == 1)
    {
        printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
#ifdef ENABLE_CHIP_RESET_MACHINE
        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
    }
    else
    {
        printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__);
    }
}

EXPORT_SYMBOL(touch_callback_himax_hang_shaking);

#ifdef ENABLE_CHIP_RESET_MACHINE
static void himax_chip_reset_function(struct work_struct *dat)
{
    printk("[Himax]:himax_chip_reset_function ++ \n");

    //if(private_ts->retry_time <= 10)
    {
        //Wakelock Protect start
        wake_lock(&private_ts->wake_lock);
        //Wakelock Protect end

        //Mutexlock Protect Start
        mutex_lock(&private_ts->mutex_lock);
        //Mutexlock Protect End

#ifdef ESD_WORKAROUND
        reset_activate = 1;
#endif

        gpio_set_value(private_ts->rst_gpio, 0);
        msleep(30);
        gpio_set_value(private_ts->rst_gpio, 1);
        msleep(30);

        //Mutexlock Protect Start
        mutex_unlock(&private_ts->mutex_lock);
        //Mutexlock Protect End

        himax_ts_poweron(private_ts);

        //Wakelock Protect start
        wake_unlock(&private_ts->wake_lock);
        //Wakelock Protect end

        if (hx_point_num != 0 && tpd_key == 0xFF)
        {
            input_report_key(private_ts->input_dev, BTN_TOUCH, 0);  // touch up
            input_mt_sync(private_ts->input_dev);
            input_sync(private_ts->input_dev);
            printk(KERN_INFO "[himax] %s: set touch up!\n", __func__);
        }

        //Joe Test ++
        if(gpio_get_value(private_ts->intr_gpio) == 0)
        {
            printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
            enable_irq(private_ts->client->irq);
        }
        //Joe Test --
    }
    //private_ts->retry_time ++;
    printk("[Himax]:himax_chip_reset_function retry_time =%d --\n",private_ts->retry_time);
}
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
static int himax_chip_monitor_function(struct work_struct *dat) //for ESD solution
{
    int ret;

    //printk(KERN_INFO "[Himax] running_status = %d, suspend_state =%d\n", himax_chip->running_status, himax_chip->suspend_state);
    //printk("[Himax]%s: IRQ =%x\n", __func__,gpio_get_value(himax_chip->intr_gpio)); //Joe Test

    if(private_ts->running_status == 0)//&& himax_chip->suspend_state == 0)
    {
        //printk(KERN_INFO "[Himax] %s \n", __func__);

        //Joe Test ++
        if(gpio_get_value(private_ts->intr_gpio) == 0)
        {
            printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
            enable_irq(private_ts->client->irq);
        }
        //Joe Test --

        ret = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail
        if(ret == 2)
        {
#ifdef ENABLE_CHIP_RESET_MACHINE
            queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
            printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
        }
        if(ret == 1)
        {
            printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
            private_ts->retry_time = 0;
            //queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
            //Do HW_RESET??
            ESD_HW_REST();
        }
        //else
        //printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__);

        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 10*HZ);
    }

    return 0;
}
#endif


/******************** Himax: ESD Workaround function end *****************/

/******************** Himax: Firmware update function ********************/
#ifdef HX_TP_FW_UPDATE
int himax_ManualMode(int enter)
{
    uint8_t cmd[2];
    cmd[0] = enter;
    if( i2c_himax_write(touch_i2c, 0x42 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

int himax_FlashMode(int enter)
{
    uint8_t cmd[2];
    cmd[0] = enter;
    if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

void himax_HW_reset(void)
{
#ifdef HX_RST_PIN_FUNC
#ifdef ESD_WORKAROUND
    //ESD Workaround Start
    reset_activate = 1;
    //ESD Workaround End
#endif
    gpio_set_value(private_ts->rst_gpio, 0);
    msleep(100);
    gpio_set_value(private_ts->rst_gpio, 1);
    msleep(100);
#endif
}


int himax_lock_flash(void)
{
    uint8_t cmd[5];

    /* lock sequence start */
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x03;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x7D;
    cmd[3] = 0x03;
    if( i2c_himax_write(touch_i2c, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if( i2c_himax_write(touch_i2c, 0x4A ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(50);
    return 0;
    /* lock sequence stop */
}

int himax_unlock_flash(void)
{
    uint8_t cmd[5];

    /* unlock sequence start */
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x03;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x3D;
    cmd[3] = 0x03;
    if( i2c_himax_write(touch_i2c, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if( i2c_himax_write(touch_i2c, 0x4A ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(50);

    return 0;
    /* unlock sequence stop */
}

int himax_modifyIref(void)
{
    //int readLen;
    unsigned char i;
    uint8_t cmd[5];
    uint8_t Iref[2] = {0x00,0x00};

    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x08;
    if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    mdelay(5);
    for(i=0; i<16; i++)
    {
        if(cmd[1]==SFR_3u_1[i][0]&&cmd[2]==SFR_3u_1[i][1])
        {
            Iref[0]= SFR_6u_1[i][0];
            Iref[1]= SFR_6u_1[i][1];
        }
    }

    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = Iref[0];
    cmd[1] = Iref[1];
    cmd[2] = 0x27;
    cmd[3] = 0x27;
    if( i2c_himax_write(touch_i2c, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if( i2c_himax_write(touch_i2c, 0x4A ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    return 1;
}

//nick_hsiao:summation of FW_VER_MAJ_buff and FW_VER_MIN_buff
u32 fw_version_int()
{
    u32 version = 0;
    int i = 0;
    for(i=0; i<FW_VER_MAJ_FLASH_LENG; i++)
    {
        if(i != 0)
            version = version << 8;
        version += FW_VER_MAJ_buff[i];
    }
    for(i=0; i<FW_VER_MIN_FLASH_LENG; i++)
    {
        version = version << 8;
        version += FW_VER_MIN_buff[i];
    }
    return version;
}

/*u8 print_full_version(char *buf, int count)
{
	int i;
	count += sprintf(buf + count, "%2.2X,",FW_VER_MAJ_buff[0]);
	count += sprintf(buf + count, "-");
	count += sprintf(buf + count, "%2.2X,",FW_VER_MIN_buff[0]);
	count += sprintf(buf + count, "-");
	return count;
}*/

//nick_hsiao:return 0:success/1:fail
u8 himax_read_FW_ver()
{
    //nick_hsiao:if have queried,no need to query again
    if(fw_version_int() > 0)
        return 0;

    u16 fw_ver_maj_start_addr;
    u16 fw_ver_maj_end_addr;
    u16 fw_ver_maj_addr;
    u16 fw_ver_maj_length;

    u16 fw_ver_min_start_addr;
    u16 fw_ver_min_end_addr;
    u16 fw_ver_min_addr;
    u16 fw_ver_min_length;

    u16 cfg_ver_maj_start_addr;
    u16 cfg_ver_maj_end_addr;
    u16 cfg_ver_maj_addr;
    u16 cfg_ver_maj_length;

    u16 cfg_ver_min_start_addr;
    u16 cfg_ver_min_end_addr;
    u16 cfg_ver_min_addr;
    u16 cfg_ver_min_length;

    uint8_t cmd[3];
    u16 i = 0;
    u16 j = 0;
    u16 k = 0;

    fw_ver_maj_start_addr 	= FW_VER_MAJ_FLASH_ADDR / 4;															// start addr = 133 / 4 = 33
    fw_ver_maj_length				= FW_VER_MAJ_FLASH_LENG;																	// length = 1
    fw_ver_maj_end_addr 		= (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length ) / 4 + 1;		// end addr = 134 / 4 = 33
    fw_ver_maj_addr 				= FW_VER_MAJ_FLASH_ADDR % 4;															// 133 mod 4 = 1

    fw_ver_min_start_addr   = FW_VER_MIN_FLASH_ADDR / 4;															// start addr = 134 / 4 = 33
    fw_ver_min_length       = FW_VER_MIN_FLASH_LENG;																	// length = 1
    fw_ver_min_end_addr     = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length ) / 4 + 1;		// end addr = 135 / 4 = 33
    fw_ver_min_addr         = FW_VER_MIN_FLASH_ADDR % 4;															// 134 mod 4 = 2

    cfg_ver_maj_start_addr  = CFG_VER_MAJ_FLASH_ADDR / 4;															// start addr = 160 / 4 = 40
    cfg_ver_maj_length      = CFG_VER_MAJ_FLASH_LENG;																	// length = 12
    cfg_ver_maj_end_addr    = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length ) / 4 + 1;	// end addr = (160 + 12) / 4 = 43
    cfg_ver_maj_addr        = CFG_VER_MAJ_FLASH_ADDR % 4;															// 160 mod 4 = 0

    cfg_ver_min_start_addr  = CFG_VER_MIN_FLASH_ADDR / 4;															// start addr = 172 / 4 = 43
    cfg_ver_min_length      = CFG_VER_MIN_FLASH_LENG;																	// length = 12
    cfg_ver_min_end_addr    = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length ) / 4 + 1;	// end addr = (172 + 12) / 4 = 46
    cfg_ver_min_addr        = CFG_VER_MIN_FLASH_ADDR % 4;													// 172 mod 4 = 0

    disable_irq(private_ts->client->irq);

    //hw reset
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset();
#endif

    //Sleep out
    if( i2c_himax_write(touch_i2c, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        goto firmware_read_fail;
    }
    mdelay(120);

    if( i2c_himax_write(touch_i2c, 0x82 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 82 fail.\n",__func__);
        goto firmware_read_fail;
    }
    msleep(100);

    //Enter flash mode
    himax_FlashMode(1);

    //Read Flash Start
    //FW Version MAJ
    i = fw_ver_maj_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 33 mod 32 	= 1
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 33 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 33 / 1024 	= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if(i == fw_ver_maj_start_addr) //first page
        {
            j = 0;
            for( k = fw_ver_maj_addr; k < 4 && j < fw_ver_maj_length; k++)
            {
                FW_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for( k = 0; k < 4 && j < fw_ver_maj_length; k++)
            {
                FW_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < fw_ver_maj_end_addr);


    //FW Version MIN
    i = fw_ver_min_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 33 mod 32 	= 1
        cmd[1] = (i >> 5) & 0x1F;		//page		= 33 / 32			= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector	= 33 / 1024		= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if(i == fw_ver_min_start_addr) //first page
        {
            j = 0;
            for(k = fw_ver_min_addr; k < 4 && j < fw_ver_min_length; k++)
            {
                FW_VER_MIN_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < fw_ver_min_length; k++)
            {
                FW_VER_MIN_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < fw_ver_min_end_addr);


    //CFG Version MAJ
    i = cfg_ver_maj_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 40 mod 32 	= 8
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 40 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 40 / 1024 	= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if(i == cfg_ver_maj_start_addr) //first page
        {
            j = 0;
            for( k = cfg_ver_maj_addr; k < 4 && j < cfg_ver_maj_length; k++)
            {
                CFG_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < cfg_ver_maj_length; k++)
            {
                CFG_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < cfg_ver_maj_end_addr);



    //CFG Version MIN
    i = cfg_ver_min_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 43 mod 32 	= 11
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 43 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 43 / 1024		= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if(i == cfg_ver_min_start_addr) //first page
        {
            j = 0;
            for(k = cfg_ver_min_addr; k < 4 && j < cfg_ver_min_length; k++)
            {
                CFG_VER_MIN_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < cfg_ver_min_length; k++)
            {
                CFG_VER_MIN_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < cfg_ver_min_end_addr);

    //Exit flash mode
    himax_FlashMode(0);

    printk("FW_VER_MAJ_buff : %d \n",FW_VER_MAJ_buff[0]);
    printk("FW_VER_MIN_buff : %d \n",FW_VER_MIN_buff[0]);
    printk("CFG_VER_MAJ_buff = ");
    for(i=0; i < CFG_VER_MAJ_FLASH_LENG; i++)
    {
        printk("0x%2.2X",CFG_VER_MAJ_buff[i]);
        if(i < CFG_VER_MAJ_FLASH_LENG-1)
            printk(" ");
        else
            printk("\n");
    }
    printk("CFG_VER_MIN_buff = ");
    for(i=0; i < CFG_VER_MIN_FLASH_LENG; i++)
    {
        printk("0x%2.2X",CFG_VER_MIN_buff[i]);
        if(i < CFG_VER_MIN_FLASH_LENG-1)
            printk(" ");
        else
            printk("\n");
    }
    //HW reset and power on.
#ifdef ENABLE_CHIP_RESET_MACHINE
    if(private_ts->init_success)
    {
        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
        //himax_chip_reset_function(NULL);
    }
#endif
    enable_irq(private_ts->client->irq);
    return 0;

firmware_read_fail:
    memset(FW_VER_MAJ_buff, 0x00, FW_VER_MAJ_FLASH_LENG);
    memset(FW_VER_MIN_buff, 0x00, FW_VER_MIN_FLASH_LENG);
    memset(CFG_VER_MAJ_buff, 0x00, CFG_VER_MAJ_FLASH_LENG);
    memset(CFG_VER_MIN_buff, 0x00, CFG_VER_MIN_FLASH_LENG);
    enable_irq(private_ts->client->irq);
    //HW reset and power on.
#ifdef ENABLE_CHIP_RESET_MACHINE
    if(private_ts->init_success)
    {
        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
        //himax_chip_reset_function(NULL);
    }
#endif
    enable_irq(private_ts->client->irq);
    return 1;
}

#ifdef HX_TP_SYS_HITOUCH
static ssize_t himax_hitouch_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret = 0;

    if(hitouch_command == 0)
    {
        ret += sprintf(buf + ret, "Driver Version:2.0 \n");
    }

    return ret;
}

//-----------------------------------------------------------------------------------
//himax_hitouch_store
//command 0 : Get Driver Version
//command 1 : Hitouch Connect
//command 2 : Hitouch Disconnect
//-----------------------------------------------------------------------------------
static ssize_t himax_hitouch_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    if(buf[0] == '0')
    {
        hitouch_command = 0;
    }
    else if(buf[0] == '1')
    {
        hitouch_is_connect = true;
        printk("[HIMAX TP MSG] hitouch_is_connect = true\n");
    }
    else if(buf[0] == '2')
    {
        hitouch_is_connect = false;
        printk("[HIMAX TP MSG] hitouch_is_connect = false\n");
    }
    return count;
}

static DEVICE_ATTR(hitouch, (S_IWUSR|S_IRUGO|S_IWGRP),himax_hitouch_show, himax_hitouch_store);
#endif

#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    //----[ENABLE_CHIP_RESET_MACHINE]-------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
    if(private_ts->init_success)
    {
        queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
    }
#endif
    //----[ENABLE_CHIP_RESET_MACHINE]--------------------------------------------------------------------end

    return count;
}

static DEVICE_ATTR(reset, (S_IWUSR|S_IWGRP),NULL, himax_reset_set);
#endif


static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)//, int address, int RST)
{
#ifdef HX_TP_BIN_CHECKSUM_SW
    u16 checksum = 0;
    uint8_t cmd[5], last_byte;
    int FileLength, i, readLen, k, lastLength;

    FileLength = fullLength - 2;
    memset(cmd, 0x00, sizeof(cmd));

    //himax_HW_reset(RST);

    //if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &cmd[0]))< 0)
    //return 0;

    //mdelay(120);
    //printk("himax_marked, Sleep out: %d\n", __LINE__);
    //himax_unlock_flash();

    himax_FlashMode(1);

    FileLength = (FileLength + 3) / 4;
    for (i = 0; i < FileLength; i++)
    {
        last_byte = 0;
        readLen = 0;

        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1)
            last_byte = 1;
        cmd[1] = (i >> 5) & 0x1F;
        cmd[2] = (i >> 10) & 0x1F;
        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }

        if (i < (FileLength - 1))
        {
            checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
            if (i == 0)
                printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
        }
        else
        {
            printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (not last): %d\n", __func__, checksum);

            lastLength = (((fullLength - 2) % 4) > 0)?((fullLength - 2) % 4):4;

            for (k = 0; k < lastLength; k++)
                checksum += cmd[k];
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (final): %d\n", __func__, checksum);

            //Check Success
            if (ImageBuffer[fullLength - 1] == (u8)(0xFF & (checksum >> 8)) && ImageBuffer[fullLength - 2] == (u8)(0xFF & checksum))
            {
                himax_FlashMode(0);
                return 1;
            }
            else //Check Fail
            {
                himax_FlashMode(0);
                return 0;
            }
        }
    }
#endif

#ifdef  HX_TP_BIN_CHECKSUM_HW
    u32 checksum = 0;
    uint8_t cmd[5], last_byte;
    int FileLength, i, readLen, k, lastLength;

    FileLength = fullLength;
    memset(cmd, 0x00, sizeof(cmd));

    //himax_HW_reset(RST);

    //if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &cmd[0]))< 0)
    //return 0;

    //mdelay(120);
    //printk("himax_marked, Sleep out: %d\n", __LINE__);
    //himax_unlock_flash();

    himax_FlashMode(1);

    FileLength = (FileLength + 3) / 4;
    for (i = 0; i < FileLength; i++)
    {
        last_byte = 0;
        readLen = 0;

        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1)
            last_byte = 1;
        cmd[1] = (i >> 5) & 0x1F;
        cmd[2] = (i >> 10) & 0x1F;
        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }

        if (i < (FileLength - 1))
        {
            checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
            if (i == 0)
                printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
        }
        else
        {
            printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (not last): %d\n", __func__, checksum);

            lastLength = ((fullLength % 4) > 0)?(fullLength % 4):4;

            for (k = 0; k < lastLength; k++)
                checksum += cmd[k];
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (final): %d\n", __func__, checksum);


            //Enable HW Checksum function.
            cmd[0] = 0x01;
            if( i2c_himax_write(touch_i2c, 0xE5 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            //Must sleep 5 ms.
            msleep(30);

            //Get HW Checksum.
            if( i2c_himax_read(touch_i2c, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return -1;
            }

            //Compare the checksum.
            if( cmd[0] == (u8)(0xFFFF & (checksum >> 24)) &&
                    cmd[1] == (u8)(0xFFFF & (checksum >> 16)) &&
                    cmd[2] == (u8)(0xFFFF & (checksum >> 8)) &&
                    cmd[3] == (u8)(0xFFFF & checksum ))
            {
                himax_FlashMode(0);
                return 1;
            }
            else
            {
                himax_FlashMode(0);
                return 0;
            }
        }
    }
#endif

#ifdef HX_TP_BIN_CHECKSUM_CRC
    uint8_t cmd[5];

    himax_HW_reset();
    msleep(120);
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0]))< 0)
        return 0;
    msleep(120);

    //Set Flash Clock Rate
    if( i2c_himax_read(touch_i2c, 0x7F, cmd, 5, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return -1;
    }
    cmd[3] = 0x02;

    if( i2c_himax_write(touch_i2c, 0x7F ,&cmd[0], 5, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    //Enable Flash
    himax_FlashMode(1);

    //Select CRC Mode
    cmd[0] = 0x05;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    if( i2c_himax_write(touch_i2c, 0xD2 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    //Enable CRC Function
    cmd[0] = 0x01;
    if( i2c_himax_write(touch_i2c, 0xE5 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    //Must delay 30 ms
    msleep(50);

    //Read HW CRC
    if( i2c_himax_read(touch_i2c, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return -1;
    }

    printk("Himax AD : %d,%d,%d,%d \n",cmd[0],cmd[1],cmd[2],cmd[3]);


    if( cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 )
    {
        himax_FlashMode(0);
        return 1;
    }
    else
    {
        himax_FlashMode(0);
        return 0;
    }
#endif

    return 0;
}

//return 1:Success, 0:Fail
//----[HX_FW_UPDATE_BY_I_FILE]--------------------------------------------------------------------------start
#ifdef HX_FW_UPDATE_BY_I_FILE
//int fts_ctpm_fw_upgrade_with_i_file(const struct firmware *fw)
int fts_ctpm_fw_upgrade_with_i_file()
{
    unsigned char* ImageBuffer =i_CTPM_FW;
    int fullFileLength = sizeof(i_CTPM_FW_Wintek_ER);

    int i, j;
    uint8_t cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;

    //Try 3 Times
    for (j = 0; j < 3; j++)
    {
#ifdef HX_TP_BIN_CHECKSUM_CRC
        FileLength = fullFileLength;
#else
        FileLength = fullFileLength - 2;
#endif

#ifdef HX_RST_PIN_FUNC
        himax_HW_reset();
#endif

        if( i2c_himax_write(touch_i2c, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        mdelay(120);

        himax_unlock_flash();  //ok

        cmd[0] = 0x05;
        cmd[1] = 0x00;
        cmd[2] = 0x02;
        if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_write(touch_i2c, 0x4F ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }
        mdelay(50);

        himax_ManualMode(1);                                                 //ok
        himax_FlashMode(1);                                                     //ok

        FileLength = (FileLength + 3) / 4;
        for (i = 0, prePage = 0; i < FileLength; i++)
        {
            last_byte = 0;

            cmd[0] = i & 0x1F;
            if (cmd[0] == 0x1F || i == FileLength - 1)
            {
                last_byte = 1;
            }
            cmd[1] = (i >> 5) & 0x1F;
            cmd[2] = (i >> 10) & 0x1F;
            if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (prePage != cmd[1] || i == 0)
            {
                prePage = cmd[1];

                cmd[0] = 0x01;
                cmd[1] = 0x09;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x0D;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x09;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
            }

            memcpy(&cmd[0], &ImageBuffer[4*i], 4);//Paul
            if( i2c_himax_write(touch_i2c, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;
            cmd[1] = 0x0D;//cmd[2] = 0x02;
            if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;
            cmd[1] = 0x09;//cmd[2] = 0x02;
            if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (last_byte == 1)
            {
                cmd[0] = 0x01;
                cmd[1] = 0x01;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x05;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x01;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x00;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                mdelay(10);
                if (i == (FileLength - 1))
                {
                    himax_FlashMode(0);
                    himax_ManualMode(0);
                    checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);

                    himax_lock_flash();

                    if (checksumResult) //Success
                    {
                        return 1;
                    }
                    else //Fail
                    {
                        return 0;
                    }
                }
            }
        }
    }
    return 0;
}
static bool i_Check_FW_Version()
{

    u16 fw_ver_maj_start_addr;
    u16 fw_ver_maj_end_addr;
    u16 fw_ver_maj_addr;
    u16 fw_ver_maj_length;

    u16 fw_ver_min_start_addr;
    u16 fw_ver_min_end_addr;
    u16 fw_ver_min_addr;
    u16 fw_ver_min_length;

    u16 cfg_ver_maj_start_addr;
    u16 cfg_ver_maj_end_addr;
    u16 cfg_ver_maj_addr;
    u16 cfg_ver_maj_length;

    u16 cfg_ver_min_start_addr;
    u16 cfg_ver_min_end_addr;
    u16 cfg_ver_min_addr;
    u16 cfg_ver_min_length;

    uint8_t cmd[3];
    u16 i = 0;
    u16 j = 0;
    u16 k = 0;

    fw_ver_maj_start_addr 	= FW_VER_MAJ_FLASH_ADDR / 4;															// start addr = 133 / 4 = 33
    fw_ver_maj_length				= FW_VER_MAJ_FLASH_LENG;																	// length = 1
    fw_ver_maj_end_addr 		= (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length ) / 4 + 1;		// end addr = 134 / 4 = 33
    fw_ver_maj_addr 				= FW_VER_MAJ_FLASH_ADDR % 4;

    fw_ver_min_start_addr   = FW_VER_MIN_FLASH_ADDR / 4;															// start addr = 134 / 4 = 33
    fw_ver_min_length       = FW_VER_MIN_FLASH_LENG;																	// length = 1
    fw_ver_min_end_addr     = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length ) / 4 + 1;		// end addr = 135 / 4 = 33
    fw_ver_min_addr         = FW_VER_MIN_FLASH_ADDR % 4;															// 134 mod 4 = 2

    cfg_ver_maj_start_addr  = CFG_VER_MAJ_FLASH_ADDR / 4;															// start addr = 160 / 4 = 40
    cfg_ver_maj_length      = CFG_VER_MAJ_FLASH_LENG;																	// length = 12
    cfg_ver_maj_end_addr    = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length ) / 4 + 1;	// end addr = (160 + 12) / 4 = 43
    cfg_ver_maj_addr        = CFG_VER_MAJ_FLASH_ADDR % 4;															// 160 mod 4 = 0

    cfg_ver_min_start_addr  = CFG_VER_MIN_FLASH_ADDR / 4;															// start addr = 172 / 4 = 43
    cfg_ver_min_length      = CFG_VER_MIN_FLASH_LENG;																	// length = 12
    cfg_ver_min_end_addr    = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length ) / 4 + 1;	// end addr = (172 + 12) / 4 = 46
    cfg_ver_min_addr        = CFG_VER_MIN_FLASH_ADDR % 4;

    //Sleep out
    if( i2c_himax_write(touch_i2c, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        goto  firmware_read_fail;
    }
    mdelay(120);

    if( i2c_himax_write(touch_i2c, 0x82 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 82 fail.\n",__func__);
        goto  firmware_read_fail;
    }
    msleep(100);

    //Enter flash mode
    himax_FlashMode(1);

    //Read Flash Start
    //FW Version MAJ
    i = fw_ver_maj_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 33 mod 32 	= 1
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 33 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 33 / 1024 	= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if(i == fw_ver_maj_start_addr) //first page
        {
            j = 0;
            for( k = fw_ver_maj_addr; k < 4 && j < fw_ver_maj_length; k++)
            {
                FW_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for( k = 0; k < 4 && j < fw_ver_maj_length; k++)
            {
                FW_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < fw_ver_maj_end_addr);

    //FW Version MIN
    i = fw_ver_min_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column	= 33 mod 32		= 1
        cmd[1] = (i >> 5) & 0x1F;		//page		= 33 / 32			= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector	= 33 / 1024		= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if(i == fw_ver_min_start_addr) //first page
        {
            j = 0;
            for(k = fw_ver_min_addr; k < 4 && j < fw_ver_min_length; k++)
            {
                FW_VER_MIN_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < fw_ver_min_length; k++)
            {
                FW_VER_MIN_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < fw_ver_min_end_addr);


    //CFG Version MAJ
    i = cfg_ver_maj_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 40 mod 32 	= 8
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 40 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 40 / 1024 	= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if(i == cfg_ver_maj_start_addr) //first page
        {
            j = 0;
            for( k = cfg_ver_maj_addr; k < 4 && j < cfg_ver_maj_length; k++)
            {
                CFG_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < cfg_ver_maj_length; k++)
            {
                CFG_VER_MAJ_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < cfg_ver_maj_end_addr);



    //CFG Version MIN
    i = cfg_ver_min_start_addr;
    do
    {
        cmd[0] = i & 0x1F;					//column 	= 43 mod 32 	= 11
        cmd[1] = (i >> 5) & 0x1F;		//page 		= 43 / 32 		= 1
        cmd[2] = (i >> 10) & 0x1F;	//sector 	= 43 / 1024		= 0

        if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_write(touch_i2c, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if( i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            goto  firmware_read_fail;
        }

        if(i == cfg_ver_min_start_addr) //first page
        {
            j = 0;
            for(k = cfg_ver_min_addr; k < 4 && j < cfg_ver_min_length; k++)
            {
                CFG_VER_MIN_buff[j++] = cmd[k];
            }
        }
        else //other page
        {
            for(k = 0; k < 4 && j < cfg_ver_min_length; k++)
            {
                CFG_VER_MIN_buff[j++] = cmd[k];
            }
        }
        i++;
    }
    while(i < cfg_ver_min_end_addr);

    printk("FW_VER_MAJ_buff : %d \n",FW_VER_MAJ_buff[0]);
    printk("FW_VER_MIN_buff : %d \n",FW_VER_MIN_buff[0]);
    printk("CFG_VER_MAJ_buff : ");
    for(k = 0; k < cfg_ver_maj_length; k++)
    {
        printk("%d",CFG_VER_MAJ_buff[k]);
        if(k != cfg_ver_maj_length-1)
            printk(",");
    }
    printk("\nCFG_VER_MIN_buff : ");
    for(k = 0; k < cfg_ver_min_length; k++)
    {
        printk("%d",CFG_VER_MIN_buff[k]);
        if(k != cfg_ver_min_length-1)
            printk(",");
    }
    printk("\n");
    return true;
firmware_read_fail:
    memset(FW_VER_MAJ_buff, 0x00, FW_VER_MAJ_FLASH_LENG);
    memset(FW_VER_MIN_buff, 0x00, FW_VER_MIN_FLASH_LENG);
    memset(CFG_VER_MAJ_buff, 0x00, CFG_VER_MAJ_FLASH_LENG);
    memset(CFG_VER_MIN_buff, 0x00, CFG_VER_MIN_FLASH_LENG);
    return false;
}


static bool compareVersion(char *fw)
{
    int i;
    /*compare FW maj version */
    u32 thisVersion = 0;  //fw_version_int();
    u32 nextVersion = 0;
    for(i=0; i<FW_VER_MAJ_FLASH_LENG; i++)
    {
        if(i != 0)
        {
            nextVersion = nextVersion << 8;
            thisVersion = thisVersion << 8;
        }
        thisVersion += FW_VER_MAJ_buff[i];
        nextVersion += fw[FW_VER_MAJ_FLASH_ADDR+i];
    }
    printk("[himax]FW MAJ, thisversion:%d,nextversion:%d\n",thisVersion,nextVersion);
    if(nextVersion > thisVersion)
        return true;

    /*compare FW min version */
    thisVersion = 0;
    nextVersion = 0;
    for(i=0; i<FW_VER_MIN_FLASH_LENG; i++)
    {
        if(i != 0)
        {
            nextVersion = nextVersion << 8;
            thisVersion = thisVersion << 8;
        }
        thisVersion += FW_VER_MIN_buff[i];
        nextVersion += fw[FW_VER_MIN_FLASH_ADDR+i];
    }
    printk("[himax]FW MIN, thisversion:%d,nextversion:%d\n",thisVersion,nextVersion);
    if(nextVersion > thisVersion)
        return true;

    /*compare TP ID version */
    thisVersion = 0;
    nextVersion = 0;
    thisVersion += CFG_VER_MAJ_buff[CFG_VER_MAJ_FLASH_LENG-1];
    nextVersion += fw[CFG_VER_MAJ_FLASH_ADDR+CFG_VER_MAJ_FLASH_LENG-1];
    printk("[himax]TP ID, thisversion:%d,nextversion:%d\n",thisVersion,nextVersion);
    if(nextVersion != thisVersion)
        return true;

    /*compare CFG min version */
    thisVersion = 0;
    nextVersion = 0;
    thisVersion += CFG_VER_MIN_buff[CFG_VER_MIN_FLASH_LENG-1];
    nextVersion += fw[CFG_VER_MIN_FLASH_ADDR+CFG_VER_MIN_FLASH_LENG-1];
    printk("[himax]CFG MIN, thisversion:%d,nextversion:%d\n",thisVersion,nextVersion);
    if(nextVersion > thisVersion)
        return true;

    return false;
}

static void i_update_func()
{
    printk("[Himax]:i_update_func\n");
    unsigned char* ImageBuffer = i_CTPM_FW;
    int fullFileLength = sizeof(i_CTPM_FW_Wintek_ER); //Paul Check
    printk("__ding__ start update\n");
    int i;
    if(compareVersion(i_CTPM_FW) || himax_calculateChecksum(ImageBuffer, fullFileLength) == 0)
    {
        printk("NH:start to update\n");
        if(fts_ctpm_fw_upgrade_with_i_file() == 0)
        {
            printk(KERN_ERR "TP upgrade error, line: %d\n", __LINE__);
            memset(FW_VER_MAJ_buff, 0x00, FW_VER_MAJ_FLASH_LENG);
            memset(FW_VER_MIN_buff, 0x00, FW_VER_MIN_FLASH_LENG);
            memset(CFG_VER_MAJ_buff, 0x00, CFG_VER_MAJ_FLASH_LENG);
            memset(CFG_VER_MIN_buff, 0x00, CFG_VER_MIN_FLASH_LENG);

        }
        else
        {
            printk(KERN_ERR "TP upgrade OK, line: %d\n", __LINE__);
            int i;
            for(i=0; i<FW_VER_MAJ_FLASH_LENG; i++)
                FW_VER_MAJ_buff[i] = i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR+i];
            for(i=0; i<FW_VER_MIN_FLASH_LENG; i++)
                FW_VER_MIN_buff[i] = i_CTPM_FW[FW_VER_MIN_FLASH_ADDR+i];
            for(i=0; i<CFG_VER_MAJ_FLASH_LENG; i++)
                CFG_VER_MAJ_buff[i] = i_CTPM_FW[CFG_VER_MAJ_FLASH_ADDR+i];
            for(i=0; i<CFG_VER_MIN_FLASH_LENG; i++)
                CFG_VER_MIN_buff[i] = i_CTPM_FW[CFG_VER_MIN_FLASH_ADDR+i];
        }
    }
}
#endif

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
    unsigned char* ImageBuffer = fw;//CTPM_FW;
    int fullFileLength = len;//sizeof(CTPM_FW); //Paul Check
    int i, j;
    uint8_t cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;

    /*printk("[TP] FW_VER_MAJ_FLASH_ADDR:%d\n",fw[FW_VER_MAJ_FLASH_ADDR]);
    printk("[TP] FW_VER_MIN_FLASH_ADDR:%d\n",fw[FW_VER_MIN_FLASH_ADDR]);*/

    //Try 3 Times
    for (j = 0; j < 3; j++)
    {
#ifdef HX_TP_BIN_CHECKSUM_CRC
        FileLength = fullFileLength;
#else
        FileLength = fullFileLength - 2;
#endif

#ifdef HX_RST_PIN_FUNC
        himax_HW_reset();
#endif

        if( i2c_himax_write(touch_i2c, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        mdelay(120);

        himax_unlock_flash();  //ok

        cmd[0] = 0x05;
        cmd[1] = 0x00;
        cmd[2] = 0x02;
        if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if( i2c_himax_write(touch_i2c, 0x4F ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }
        mdelay(50);

        himax_ManualMode(1); //ok
        himax_FlashMode(1);  //ok

        FileLength = (FileLength + 3) / 4;
        for (i = 0, prePage = 0; i < FileLength; i++)
        {
            last_byte = 0;
            cmd[0] = i & 0x1F;
            if (cmd[0] == 0x1F || i == FileLength - 1)
                last_byte = 1;
            cmd[1] = (i >> 5) & 0x1F;
            cmd[2] = (i >> 10) & 0x1F;
            if( i2c_himax_write(touch_i2c, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (prePage != cmd[1] || i == 0)
            {
                prePage = cmd[1];
                cmd[0] = 0x01;
                cmd[1] = 0x09;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x0D;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x09;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
            }

            memcpy(&cmd[0], &ImageBuffer[4*i], 4);//Paul
            if( i2c_himax_write(touch_i2c, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;
            cmd[1] = 0x0D;//cmd[2] = 0x02;
            if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;
            cmd[1] = 0x09;//cmd[2] = 0x02;
            if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
            {
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (last_byte == 1)
            {
                cmd[0] = 0x01;
                cmd[1] = 0x01;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x05;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x01;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;
                cmd[1] = 0x00;//cmd[2] = 0x02;
                if( i2c_himax_write(touch_i2c, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
                {
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                mdelay(10);
                if (i == (FileLength - 1))
                {
                    himax_FlashMode(0);
                    himax_ManualMode(0);
                    checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);//, address, RST);
                    //himax_ManualMode(0);
                    himax_lock_flash();

                    if (checksumResult) //Success
                    {
                        himax_HW_reset();
                        int i = 0;
                        //nick_hsiao,assign fw number since upgrade FW success
                        for(i=0; i<FW_VER_MAJ_FLASH_LENG; i++)
                            FW_VER_MAJ_buff[i] = fw[FW_VER_MAJ_FLASH_ADDR+i];
                        for(i=0; i<FW_VER_MIN_FLASH_LENG; i++)
                            FW_VER_MIN_buff[i] = fw[FW_VER_MIN_FLASH_ADDR+i];
                        for(i=0; i<CFG_VER_MAJ_FLASH_LENG; i++)
                            CFG_VER_MAJ_buff[i] = fw[CFG_VER_MAJ_FLASH_ADDR+i];
                        for(i=0; i<CFG_VER_MIN_FLASH_LENG; i++)
                            CFG_VER_MIN_buff[i] = fw[CFG_VER_MIN_FLASH_ADDR+i];
                        return 1;
                    }
                    else if (/*j == 4 && */!checksumResult) //Fail
                    {
                        himax_HW_reset();
                        //nick_hsiao:clear FW number if upgrade fail
                        memset(FW_VER_MAJ_buff, 0x00, FW_VER_MAJ_FLASH_LENG);
                        memset(FW_VER_MIN_buff, 0x00, FW_VER_MIN_FLASH_LENG);
                        memset(CFG_VER_MAJ_buff, 0x00, CFG_VER_MAJ_FLASH_LENG);
                        memset(CFG_VER_MIN_buff, 0x00, CFG_VER_MIN_FLASH_LENG);
                        return 0;
                    }
                    else //Retry
                    {
                        himax_FlashMode(0);
                        himax_ManualMode(0);
                    }
                }
            }
        }
    }
    return 0;
}
#endif
/******************** Himax: Firmware update function ********************/

/******************** Himax: Debug Tool function ********************/
#ifdef HX_TP_SYS_FS

#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *getFlashBuffer(void)
{
    return flash_buffer;
}

static void setFlashBuffer(void)
{
    int i=0;
    flash_buffer = kzalloc(32768*sizeof(uint8_t), GFP_KERNEL);
    for(i=0; i<32768; i++)
        flash_buffer[i] = 0x00;
}

static uint8_t getFlashCommand(void)
{
    return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
    return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
    return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
    return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
    return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
    return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
    return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
    return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
    return flash_dump_going;
}

static void setSysOperation(uint8_t operation)
{
    sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
    flash_progress = progress;
    printk("TPPPP setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
    flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
    flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
    flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
    flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
    flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
    flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
    flash_dump_going = going;
}
#endif

static uint8_t *getMutualBuffer(void)
{
    return diag_mutual;
}

static void setMutualBuffer(void)
{
    diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

static uint8_t *getSelfBuffer(void)
{
    return &diag_self[0];
}

static uint8_t getDebugLevel(void)
{
    return debug_log_level;
}

static uint8_t getDiagCommand(void)
{
    return diag_command;
}

static uint8_t getXChannel(void)
{
    return x_channel;
}

static uint8_t getYChannel(void)
{
    return y_channel;
}

static void setXChannel(uint8_t x)
{
    x_channel = x;
}

static void setYChannel(uint8_t y)
{
    y_channel = y;
}

#ifdef TPD_PROXIMITY
static int tpd_get_ps_value(void)
{
    return tpd_proximity_detect;
}
static int tpd_enable_ps(int enable)
{
    int ret = 0;
    char data[1] = {0};
    int err;
    if(enable)	//Proximity On
    {
        printk("tpd_enable_ps enable \n");
        data[0] =0x01;
        err = i2c_himax_write(touch_i2c, 0x92, &data[0], 1, 3);
        if(err < 0)
        {
            //tpd_proximity_flag = 1;
            printk("tpd_enable_ps enable error value : %d\n",err);
            ret = 1;
        }
        else
            tpd_proximity_flag = 1;
    }
    else	//Proximity off
    {
        printk("tpd_enable_ps disable \n");
        data[0] =0x00;
        err = i2c_himax_write(touch_i2c, 0x92, &data[0], 1, 3);
        if(err < 0)
        {
            printk("tpd_enable_ps disable error value : %d\n",err);
            //tpd_proximity_flag = 0;
            ret = 1;
        }
        else
            tpd_proximity_flag = 0;
    }
    msleep(1);

    return ret;
}
#endif

#ifdef TPD_PROXIMITY
static ssize_t himax_show_ps_object_distance(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int object_distance = (tpd_proximity_detect == 0) ? 0 : 5;
    return sprintf(buf, "%d\n", object_distance);
}
static DEVICE_ATTR(ps_object_distance, (S_IRUGO),himax_show_ps_object_distance, NULL);

static ssize_t himax_ps_status(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", tpd_proximity_flag);
}

static ssize_t himax_ps_enable_disable_poll_event(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count)
{
    long enable;
    printk("himax : %s\n",__func__);
    if (strict_strtol(buf, 10, &enable))
        return -EINVAL;
    if ((enable != 1) && (enable != 0))
        return -EINVAL;

    if(enable == 1)
    {
        printk("himax : himax ps poll enable\n");
        tpd_enable_ps(1);
        himax_ps_irq_data_count = PS_INIT_POLL_MAX_COUNT;//+++ add 10 count for init polling
        /*himax_ps_flag_pollin = true;
        queue_delayed_work(himax_ps_poll_work_queue, &himax_ps_report_poll_event_work, msecs_to_jiffies(0));*/
    }
    else
    {
        printk("himax : himax ps poll disable\n");
        tpd_enable_ps(0);
        //cancel_delayed_work_sync(&himax_ps_report_poll_event_work);
    }
    return strnlen(buf, count);
}

static DEVICE_ATTR(ps_status,(S_IWUSR|S_IRUGO|S_IWGRP),himax_ps_status,himax_ps_enable_disable_poll_event);

static ssize_t himax_ps_set_delay(struct device *dev, struct device_attribute *attr,
                                  const char *buf , size_t count)
{
    long temp;
    int delay_time;
    if (strict_strtol(buf, 10, &temp))
        return -EINVAL;
    delay_time = (int)temp;
    printk("himax : himax_ps_set_delay delay_time = %d\n",delay_time);
    if(delay_time<himax_ps_min_poll_time)
    {
        himax_ps_poll_time = himax_ps_min_poll_time;
    }
    else
    {
        himax_ps_poll_time = delay_time;
    }
    printk("himax : himax_ps_poll_time = %d\n",himax_ps_poll_time);
    return strnlen(buf, count);
}

static ssize_t himax_ps_get_delay(struct device *dev ,
                                  struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", himax_ps_poll_time);
}

static DEVICE_ATTR(ps_poll_time,(S_IWUSR|S_IRUGO|S_IWGRP),himax_ps_get_delay,himax_ps_set_delay);

static ssize_t himax_ps_show_irq_data_count(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", himax_ps_irq_data_count);
}

static DEVICE_ATTR(ps_has_data,(S_IRUGO),himax_ps_show_irq_data_count,NULL);

#endif

static ssize_t himax_register_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int base = 0;
    int tmp = 0;
    uint16_t loop_i,loop_j;
    uint8_t data[128];
    uint8_t outData[5];

    memset(outData, 0x00, sizeof(outData));
    memset(data, 0x00, sizeof(data));

    printk(KERN_INFO "Himax multi_register_command = %d \n",multi_register_command);

    if(multi_register_command == 1)
    {
        base = 0;

        for(loop_i = 0; loop_i < 6; loop_i++)
        {
            if(multi_register[loop_i] != 0x00)
            {
                if(multi_cfg_bank[loop_i] == 1) //config bank register
                {
                    outData[0] = 0x15;
                    i2c_himax_write(touch_i2c, 0xE1 ,&outData[0], 1, DEFAULT_RETRY_CNT);
                    msleep(10);

                    outData[0] = 0x00;
                    outData[1] = multi_register[loop_i];
                    i2c_himax_write(touch_i2c, 0xD8 ,&outData[0], 2, DEFAULT_RETRY_CNT);
                    msleep(10);

                    i2c_himax_read(touch_i2c, 0x5A, data, 128, DEFAULT_RETRY_CNT);

                    outData[0] = 0x00;
                    i2c_himax_write(touch_i2c, 0xE1 ,&outData[0], 1, DEFAULT_RETRY_CNT);

                    for(loop_j=0; loop_j<128; loop_j++)
                    {
                        multi_value[base++] = data[loop_j];
                    }
                }
                else //normal register
                {
                    i2c_himax_read(touch_i2c, multi_register[loop_i], data, 128, DEFAULT_RETRY_CNT);

                    for(loop_j=0; loop_j<128; loop_j++)
                    {
                        multi_value[base++] = data[loop_j];
                    }
                }
            }
        }

        base = 0;
        for(loop_i = 0; loop_i < 6; loop_i++)
        {
            if(multi_register[loop_i] != 0x00)
            {
                if(multi_cfg_bank[loop_i] == 1)
                {
                    ret += sprintf(buf + ret, "Register: FE(%x)\n", multi_register[loop_i]);
                }
                else
                {
                    ret += sprintf(buf + ret, "Register: %x\n", multi_register[loop_i]);
                }

                for (loop_j = 0; loop_j < 128; loop_j++)
                {
                    ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
                    if ((loop_j % 16) == 15)
                    {
                        ret += sprintf(buf + ret, "\n");
                    }
                }
            }
        }
        return ret;
    }

    if(config_bank_reg)
    {
        printk(KERN_INFO "[TP] %s: register_command = FE(%x)\n", __func__, register_command);

        //Config bank register read flow.
        outData[0] = 0x15;
        i2c_himax_write(touch_i2c, 0xE1,&outData[0], 1, DEFAULT_RETRY_CNT);

        msleep(10);

        outData[0] = 0x00;
        outData[1] = register_command;
        i2c_himax_write(touch_i2c, 0xD8,&outData[0], 2, DEFAULT_RETRY_CNT);

        msleep(10);

        i2c_himax_read(touch_i2c, 0x5A, data, 128, DEFAULT_RETRY_CNT);

        msleep(10);

        outData[0] = 0x00;
        i2c_himax_write(touch_i2c, 0xE1,&outData[0], 1, DEFAULT_RETRY_CNT);
    }
    else
    {
        if (i2c_himax_read(touch_i2c, register_command, data, 128, DEFAULT_RETRY_CNT) < 0)
        {
            return ret;
        }
    }

    if(config_bank_reg)
    {
        ret += sprintf(buf, "command: FE(%x)\n", register_command);
    }
    else
    {
        ret += sprintf(buf, "command: %x\n", register_command);
    }

    for (loop_i = 0; loop_i < 128; loop_i++)
    {
        ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
        if ((loop_i % 16) == 15)
        {
            ret += sprintf(buf + ret, "\n");
        }
    }
    ret += sprintf(buf + ret, "\n");
    return ret;
}

static ssize_t himax_register_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6], length = 0;
    unsigned long result		= 0;
    uint8_t veriLen					= 0;
    uint8_t loop_i					= 0;
    uint8_t loop_j					= 0;
    uint16_t base					= 5;
    uint8_t write_da[128];
    uint8_t outData[5];

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
    memset(outData, 0x0, sizeof(outData));

    printk("himax %s \n",buf);

    if( buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':')
    {
        memset(multi_register, 0x00, sizeof(multi_register));
        memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
        memset(multi_value, 0x00, sizeof(multi_value));

        printk("himax multi register enter\n");

        multi_register_command = 1;

        base		= 2;
        loop_i	= 0;

        while(true)
        {
            if(buf[base] == '\n')
            {
                break;
            }

            if(loop_i >= 6 )
            {
                break;
            }

            if(buf[base] == ':' && buf[base+1] == 'x' && buf[base+2] == 'F' && buf[base+3] == 'E' && buf[base+4] != ':')
            {
                memcpy(buf_tmp, buf + base + 4, 2);
                if (!strict_strtoul(buf_tmp, 16, &result))
                {
                    multi_register[loop_i] = result;
                    multi_cfg_bank[loop_i++] = 1;
                }
                base += 6;
            }
            else
            {
                memcpy(buf_tmp, buf + base + 2, 2);
                if (!strict_strtoul(buf_tmp, 16, &result))
                {
                    multi_register[loop_i] = result;
                    multi_cfg_bank[loop_i++] = 0;
                }
                base += 4;
            }
        }

        printk(KERN_INFO "========================== \n");
        for(loop_i = 0; loop_i < 6; loop_i++)
        {
            printk(KERN_INFO "%d,%d:",multi_register[loop_i],multi_cfg_bank[loop_i]);
        }
        printk(KERN_INFO "\n");
    }
    else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':')
    {
        multi_register_command = 0;

        if (buf[2] == 'x')
        {
            if(buf[3] == 'F' && buf[4] == 'E') //Config bank register
            {
                config_bank_reg = true;

                memcpy(buf_tmp, buf + 5, 2);
                if (!strict_strtoul(buf_tmp, 16, &result))
                {
                    register_command = result;
                }
                base = 7;

                printk(KERN_INFO "CMD: FE(%x)\n", register_command);
            }
            else
            {
                config_bank_reg = false;

                memcpy(buf_tmp, buf + 3, 2);
                if (!strict_strtoul(buf_tmp, 16, &result))
                {
                    register_command = result;
                }
                base = 5;
                printk(KERN_INFO "CMD: %x\n", register_command);
            }

            for (loop_i = 0; loop_i < 128; loop_i++)
            {
                if (buf[base] == '\n')
                {
                    if (buf[0] == 'w')
                    {
                        if(config_bank_reg)
                        {
                            outData[0] = 0x15;
                            i2c_himax_write(touch_i2c, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);

                            msleep(10);

                            outData[0] = 0x00;
                            outData[1] = register_command;
                            i2c_himax_write(touch_i2c, 0xD8, &outData[0], 2, DEFAULT_RETRY_CNT);

                            msleep(10);
                            i2c_himax_write(touch_i2c, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);

                            msleep(10);

                            outData[0] = 0x00;
                            i2c_himax_write(touch_i2c, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);

                            printk(KERN_INFO "CMD: FE(%x), %x, %d\n", register_command,write_da[0], length);
                        }
                        else
                        {
                            i2c_himax_write(touch_i2c, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
                            printk(KERN_INFO "CMD: %x, %x, %d\n", register_command,write_da[0], length);
                        }
                    }

                    printk(KERN_INFO "\n");
                    return count;
                }
                if (buf[base + 1] == 'x')
                {
                    buf_tmp[4] = '\n';
                    buf_tmp[5] = '\0';
                    memcpy(buf_tmp, buf + base + 2, 2);
                    if (!strict_strtoul(buf_tmp, 16, &result))
                    {
                        write_da[loop_i] = result;
                    }
                    length++;
                }
                base += 4;
            }
        }
    }
    return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO|S_IWGRP),
                   himax_register_show, himax_register_store);

//add by josh setting self_test config
/*
* time : t:x12 (12s)
* fe96 : w:x96:x03:x02......
*/
static ssize_t himax_self_test_setting(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;
    static uint8_t himax_command = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
    if (buf[0] == 't' && buf[1] == ':' && buf[2] == 'x')
    {
        if(buf[3] > 47 && buf[3] < 58 && buf[4] > 47 && buf[4] < 58)
        {
            self_test_delay_time = ( buf[3] - 48 ) * 10 + buf[4] - 48;
            printk(KERN_INFO "self_test_delay_time: %d", self_test_delay_time);
        }
        return count;
    }

    if (buf[0] == 's')
    {
        diag_dump_to_gauge = true;
        if (buf[2] == 'w' && buf[3] == ':')
        {
            if (buf[4] == 'x')
            {
                uint8_t loop_i;
                uint16_t base = 7;
                memcpy(buf_tmp, buf + 3, 2);
                if (!strict_strtoul(buf_tmp, 16, &result))
                    himax_command = result;
                for (loop_i = 0; loop_i < 100; loop_i++)
                {
                    if (buf[base] == '\n')
                    {
                        if (buf[2] == 'w')
                            printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
                                   write_da[0], length);
                        for (veriLen = 0; veriLen < length; veriLen++)
                        {
                            printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));
                            rFE96_setting[veriLen] = *((&write_da[0])+veriLen);
                            printk(KERN_INFO "rFE96_setting[%d] : %x \n",veriLen ,rFE96_setting[veriLen]);
                        }

                        printk(KERN_INFO "\n");
                        return count;
                    }
                    if (buf[base + 1] == 'x')
                    {
                        buf_tmp[4] = '\n';
                        buf_tmp[5] = '\0';
                        memcpy(buf_tmp, buf + base + 2, 2);
                        if (!strict_strtoul(buf_tmp, 16, &result))
                            write_da[loop_i] = result;
                        length++;
                    }
                    base += 4;
                }
            }
        }
        return count;
    }

    if (buf[0] == 'w' && buf[1] == ':')
    {
        if (buf[2] == 'x')
        {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                himax_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++)
            {
                if (buf[base] == '\n')
                {
                    if (buf[0] == 'w')

                        printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
                               write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++)
                    {
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));
                        rFE96_setting[veriLen] = *((&write_da[0])+veriLen);
                        printk(KERN_INFO "rFE96_setting[%d] : %x \n",veriLen ,rFE96_setting[veriLen]);
                    }

                    printk(KERN_INFO "\n");
                    return count;
                }
                if (buf[base + 1] == 'x')
                {
                    buf_tmp[4] = '\n';
                    buf_tmp[5] = '\0';
                    memcpy(buf_tmp, buf + base + 2, 2);
                    if (!strict_strtoul(buf_tmp, 16, &result))
                        write_da[loop_i] = result;
                    length++;
                }
                base += 4;
            }
        }
    }

    return count;
}


static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val=0x00;
    val = himax_chip_self_test();
    char *tmp;
    if(diag_dump_to_gauge == true)
    {
        //touch dc
        himax_diag_dump_func(TOUCH_DC);
        msleep(2000);
        himax_chip_raw_data_store_func(buf, "/gauge/");
        //touch iir
        /*himax_diag_dump_func(TOUCH_IIR);
        msleep(2000);
        himax_chip_raw_data_store_func(buf, "/gauge/");*/
        //touch bank
        himax_diag_dump_func(TOUCH_BANK);
        msleep(2000);
        himax_chip_raw_data_store_func(buf, "/gauge/");
        diag_dump_to_gauge = false;
    }

    if(val == 0)
    {
        return sprintf(buf, "PASS\n");
    }
    else
    {
        return sprintf(buf, "FAIL\n");
    }
}

static int himax_chip_self_test(void)
{
    uint8_t cmdbuf[11];
    int ret = 0;
    uint8_t valuebuf[16];
    int i=0, pf_value=0x00;


    //reset IC
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset();
#endif

    himax_ts_poweron(private_ts);

    //Step 0 : sensor off
    i2c_himax_write(private_ts->client, 0x82,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);

    //Sleep In
    i2c_himax_write(private_ts->client, 0x80,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);

    //Step 1 : Close Re-Calibration FE02
    //-->Read 0xFE02
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x02; //FE02
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    msleep(100);

    printk("[Himax]:0xFE02_0 = 0x%x\n",valuebuf[0]);
    printk("[Himax]:0xFE02_1 = 0x%x\n",valuebuf[1]);

    valuebuf[0] = valuebuf[1] & 0xFD; // close re-calibration  , shift first byte of config bank register read issue.

    printk("[Himax]:0xFE02_valuebuf = 0x%x\n",valuebuf[0]);

    //-->Write 0xFE02
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x02; //FE02
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = valuebuf[0];
    i2c_himax_write(private_ts->client, 0x40,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    msleep(100);
    //0xFE02 Read Back

    //-->Read 0xFE02
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x02; //FE02
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(100);

    printk("[Himax]:0xFE02_0_back = 0x%x\n",valuebuf[0]);
    printk("[Himax]:0xFE02_1_back = 0x%x\n",valuebuf[1]);

    //Step 2 : Close Flash-Reload
    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE3,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    msleep(100);

    i2c_himax_read(private_ts->client, 0xE3, valuebuf, 1, DEFAULT_RETRY_CNT);

    printk("[Himax]:0xE3_back = 0x%x\n",valuebuf[0]);

    //Step 4 : Write self_test parameter to FE96~FE9D
    //-->Write FE96~FE9D
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x96; //FE96
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    //-->Modify the initial value of self_test.
    cmdbuf[0] = rFE96_setting[0];
    cmdbuf[1] = rFE96_setting[1];
    cmdbuf[2] = rFE96_setting[2];
    cmdbuf[3] = rFE96_setting[3];
    cmdbuf[4] = rFE96_setting[4];
    cmdbuf[5] = rFE96_setting[5];
    cmdbuf[6] = rFE96_setting[6];
    cmdbuf[7] = rFE96_setting[7];
    i2c_himax_write(private_ts->client, 0x40,&cmdbuf[0], 8, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    msleep(100);

    //Read back
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x96; //FE96
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    for(i=1; i<16; i++)
    {
        printk("[Himax]:0xFE96 buff_back[%d] = 0x%x\n",i,valuebuf[i]);
    }

    msleep(100);

    //Step 5 : Enter self_test mode
    cmdbuf[0] = 0x16;
    i2c_himax_write(private_ts->client, 0x91,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    i2c_himax_read(private_ts->client, 0x91, valuebuf, 1, DEFAULT_RETRY_CNT);

    printk("[Himax]:0xE3_back = 0x%x\n",valuebuf[0]);
    msleep(10);

    //Step 6 : Sensor On
    i2c_himax_write(private_ts->client, 0x83,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
    msleep(120);

    //Sleep Out
    i2c_himax_write(private_ts->client, 0x81,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);

    mdelay(self_test_delay_time * 1000);

    //Step 7 : Sensor Off
    i2c_himax_write(private_ts->client, 0x82,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);

    msleep(120);

    //Step 8 : Get self_test result
    cmdbuf[0] = 0x15;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    cmdbuf[1] = 0x96; //FE96
    i2c_himax_write(private_ts->client, 0xD8,&cmdbuf[0], 2, DEFAULT_RETRY_CNT);
    msleep(10);

    i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
    msleep(10);

    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0xE1,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    //Final : Leave self_test mode
    cmdbuf[0] = 0x00;
    i2c_himax_write(private_ts->client, 0x91,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);

    if(valuebuf[1]==0xAA) //get the self_test result , shift first byte for config bank read issue.
    {
        printk("[Himax]: self-test pass\n");
        pf_value = 0x0;
    }
    else
    {
        printk("[Himax]: self-test fail\n");
        pf_value = 0x1;
        for(i=1; i<16; i++)
        {
            printk("[Himax]:0xFE96 buff[%d] = 0x%x\n",i,valuebuf[i]);
        }
    }

    i2c_himax_read(private_ts->client, 0xF6, valuebuf, 10, DEFAULT_RETRY_CNT);
    printk("[Himax]:F6[9] = 0x%x\n",valuebuf[9]);
    msleep(10);

    //HW reset and power on again.
    //----[HX_RST_PIN_FUNC]-----------------------------------------------------------------------------start
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset();
#endif
    //----[HX_RST_PIN_FUNC]-------------------------------------------------------------------------------end

    himax_ts_poweron(private_ts);

    return pf_value;
}


static DEVICE_ATTR(tp_self_test, (S_IWUSR|S_IRUGO|S_IWGRP), himax_chip_self_test_function, himax_self_test_setting);

#ifdef HX_TP_SYS_FLASH_DUMP
static ssize_t himax_flash_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int loop_i;
    uint8_t local_flash_read_step=0;
    uint8_t local_flash_complete = 0;
    uint8_t local_flash_progress = 0;
    uint8_t local_flash_command = 0;
    uint8_t local_flash_fail = 0;

    local_flash_complete = getFlashDumpComplete();
    local_flash_progress = getFlashDumpProgress();
    local_flash_command = getFlashCommand();
    local_flash_fail = getFlashDumpFail();

    printk("TPPPP flash_progress = %d \n",local_flash_progress);

    if(local_flash_fail)
    {
        ret += sprintf(buf+ret, "FlashStart:Fail \n");
        ret += sprintf(buf + ret, "FlashEnd");
        ret += sprintf(buf + ret, "\n");
        return ret;
    }

    if(!local_flash_complete)
    {
        ret += sprintf(buf+ret, "FlashStart:Ongoing:0x%2.2x \n",flash_progress);
        ret += sprintf(buf + ret, "FlashEnd");
        ret += sprintf(buf + ret, "\n");
        return ret;
    }

    if(local_flash_command == 1 && local_flash_complete)
    {
        ret += sprintf(buf+ret, "FlashStart:Complete \n");
        ret += sprintf(buf + ret, "FlashEnd");
        ret += sprintf(buf + ret, "\n");
        return ret;
    }

    if(local_flash_command == 3 && local_flash_complete)
    {
        ret += sprintf(buf+ret, "FlashStart: \n");
        for(loop_i = 0; loop_i < 128; loop_i++)
        {
            ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
            if((loop_i % 16) == 15)
            {
                ret += sprintf(buf + ret, "\n");
            }
        }
        ret += sprintf(buf + ret, "FlashEnd");
        ret += sprintf(buf + ret, "\n");
        return ret;
    }

    //flash command == 0 , report the data
    local_flash_read_step = getFlashReadStep();

    ret += sprintf(buf+ret, "FlashStart:%2.2x \n",local_flash_read_step);

    for (loop_i = 0; loop_i < 1024; loop_i++)
    {
        ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);

        if ((loop_i % 16) == 15)
        {
            ret += sprintf(buf + ret, "\n");
        }
    }

    ret += sprintf(buf + ret, "FlashEnd");
    ret += sprintf(buf + ret, "\n");
    return ret;
}

//-----------------------------------------------------------------------------------
//himax_flash_store
//
//command 0 : Read the page by step number
//command 1 : driver start to dump flash data, save it to mem
//command 2 : driver start to dump flash data, save it to sdcard/Flash_Dump.bin
//
//-----------------------------------------------------------------------------------
static ssize_t himax_flash_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6];
    unsigned long result = 0;
    uint8_t loop_i = 0;
    int base = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));

    printk(KERN_INFO "[TP] %s: buf[0] = %s\n", __func__, buf);

    if(getSysOperation() == 1)
    {
        printk(KERN_INFO "[TP] SYS is busy , return!\n", __func__);
        return count;
    }

    if(buf[0] == '0')
    {
        setFlashCommand(0);
        if(buf[1] == ':' && buf[2] == 'x')
        {
            memcpy(buf_tmp, buf + 3, 2);
            printk(KERN_INFO "[TP] %s: read_Step = %s\n", __func__, buf_tmp);
            if (!strict_strtoul(buf_tmp, 16, &result))
            {
                printk(KERN_INFO "[TP] %s: read_Step = %d\n", __func__, result);
                setFlashReadStep(result);
            }
        }
    }
    else if(buf[0] == '1')
    {
        setSysOperation(1);
        setFlashCommand(1);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);
        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    }
    else if(buf[0] == '2')
    {
        setSysOperation(1);
        setFlashCommand(2);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);

        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    }
    else if(buf[0] == '3')
    {
        setSysOperation(1);
        setFlashCommand(3);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);

        memcpy(buf_tmp, buf + 3, 2);
        if (!strict_strtoul(buf_tmp, 16, &result))
        {
            setFlashDumpSector(result);
        }

        memcpy(buf_tmp, buf + 7, 2);
        if (!strict_strtoul(buf_tmp, 16, &result))
        {
            setFlashDumpPage(result);
        }

        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    }
    else if(buf[0] == '4')
    {
        printk(KERN_INFO "[TP] %s: command 4 enter.\n", __func__);
        setSysOperation(1);
        setFlashCommand(4);
        setFlashDumpProgress(0);
        setFlashDumpComplete(0);
        setFlashDumpFail(0);

        memcpy(buf_tmp, buf + 3, 2);
        if (!strict_strtoul(buf_tmp, 16, &result))
        {
            setFlashDumpSector(result);
        }
        else
        {
            printk(KERN_INFO "[TP] %s: command 4 , sector error.\n", __func__);
            return count;
        }

        memcpy(buf_tmp, buf + 7, 2);
        if (!strict_strtoul(buf_tmp, 16, &result))
        {
            setFlashDumpPage(result);
        }
        else
        {
            printk(KERN_INFO "[TP] %s: command 4 , page error.\n", __func__);
            return count;
        }

        base = 11;

        printk(KERN_INFO "=========Himax flash page buffer start=========\n");
        for(loop_i=0; loop_i<128; loop_i++)
        {
            memcpy(buf_tmp, buf + base, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
            {
                flash_buffer[loop_i] = result;
                printk(" %d ",flash_buffer[loop_i]);
                if(loop_i % 16 == 15)
                {
                    printk("\n");
                }
            }
            base += 3;
        }
        printk(KERN_INFO "=========Himax flash page buffer end=========\n");

        queue_work(private_ts->flash_wq, &private_ts->flash_work);
    }
    return count;
}

static DEVICE_ATTR(flash_dump, (S_IWUSR|S_IRUGO|S_IWGRP),himax_flash_show, himax_flash_store);
#endif

static ssize_t touch_vendor_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;

    sprintf(buf, "%s_%s\n", FW_VER_MAJ_FLASH_buff, FW_VER_MIN_FLASH_buff);
    ret = strlen(buf) + 1;

    return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, touch_vendor_show, NULL);

static ssize_t himax_debug_level_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    size_t count = 0;
    int i = 0;

    if(debug_level_cmd == 't')
    {
        if(fw_update_complete)
        {
            count += sprintf(buf, "FW Update Complete \n");
        }
        else
        {
            count += sprintf(buf, "FW Update Fail \n");
        }
    }
    else if(debug_level_cmd == 'i')
    {
        if(irq_enable)
        {
            count += sprintf(buf, "IRQ is enable\n");
        }
        else
        {
            count += sprintf(buf, "IRQ is disable\n");
        }
    }
    else if(debug_level_cmd == 'h')
    {
        if(handshaking_result == 0)
        {
            count += sprintf(buf, "Handshaking Result = %d (MCU Running)\n",handshaking_result);
        }
        else if(handshaking_result == 1)
        {
            count += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",handshaking_result);
        }
        else if(handshaking_result == 2)
        {
            count += sprintf(buf, "Handshaking Result = %d (I2C Error)\n",handshaking_result);
        }
        else
        {
            count += sprintf(buf, "Handshaking Result = error \n");
        }
    }
    else if(debug_level_cmd == 'l')
    {
        count += sprintf(buf, "Handshaking Fail Result List : \n%s",handshaking_fail_result);
    }
    else if(debug_level_cmd == 'v')
    {
        count += sprintf(buf + count, "FW_VER_MAJ_buff = ");
        count += sprintf(buf + count, "0x%2.2X \n",FW_VER_MAJ_buff[0]);

        count += sprintf(buf + count, "FW_VER_MIN_buff = ");
        count += sprintf(buf + count, "0x%2.2X \n",FW_VER_MIN_buff[0]);

        count += sprintf(buf + count, "CFG_VER_MAJ_buff = ");
        for(i=0; i < CFG_VER_MAJ_FLASH_LENG; i++)
        {
            count += sprintf(buf + count,  "0x%2.2X",CFG_VER_MAJ_buff[i]);
            if(i < CFG_VER_MAJ_FLASH_LENG-1)
                count += sprintf(buf + count,  " ");
            else
                count += sprintf(buf + count,  "\n");
        }

        count += sprintf(buf + count, "CFG_VER_MIN_buff = ");
        for(i=0; i < CFG_VER_MIN_FLASH_LENG; i++)
        {
            count += sprintf(buf + count,  "0x%2.2X",CFG_VER_MIN_buff[i]);
            if(i < CFG_VER_MIN_FLASH_LENG-1)
                count += sprintf(buf + count,  " ");
            else
                count += sprintf(buf + count,  "\n");
        }

        count += sprintf(buf + count, "TP = %d\n",TP_ID);
    }
    else if(debug_level_cmd == 'n')
    {
        u32 version = fw_version_int();
        count += sprintf(buf + count, "%x\n", version);
    }
    else
    {
        count += sprintf(buf, "%d\n", debug_log_level);
    }
    return count;
}

static unsigned char upgrade_fw[32*1024];

static ssize_t himax_debug_level_dump(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef HX_TP_FW_UPDATE
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    char fileName[128];
#endif

    //nick_hsiao:disable irq while starting update
    //disable_irq(&private_ts->client->irq);

    if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
    {
        debug_log_level = buf[0] - '0';
    }

    if (buf[0] == 'i') //irq
    {
        debug_level_cmd = buf[0];

        if( buf[2] == '1') //enable irq
        {
            enable_irq(private_ts->client->irq);
            irq_enable = true;
        }
        else if(buf[2] == '0') //disable irq
        {
            disable_irq(private_ts->client->irq);
            irq_enable = false;
        }
        else
        {
            printk(KERN_ERR "[TP] %s: debug_level command = 'i' , parameter error.\n", __func__);
        }
        return count;
    }

    if( buf[0] == 'h') //handshaking
    {
        debug_level_cmd = buf[0];

        disable_irq(private_ts->client->irq);

        handshaking_result = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail

        enable_irq(private_ts->client->irq);

        return count;
    }

    if( buf[0] == 'l') //handshaking fail result log
    {
        debug_level_cmd = buf[0];
        return count;
    }

    if( buf[0] == 'v') //firmware version
    {
        debug_level_cmd = buf[0];
        if(himax_read_FW_ver() != 0)
        {
            count += sprintf(buf+count, "get FW version fail\n");
        }
        return count;
    }

    if( buf[0] == 'c')
    {
#ifdef HX_LOADIN_CONFIG
        himax_config_flow();
#endif
        return count;
    }

#ifdef TPD_PROXIMITY
    if( buf[0] == 'p')
    {
        if( buf[2] == '1') //p sensor on
        {
            if(tpd_enable_ps(1) > 0)
                printk("tpd_enable_ps enable fail\n");
            else
                printk("tpd_enable_ps enable sucess\n");
        }
        else if(buf[2] == '0') //p sensor off
        {
            tpd_proximity_detect = 1;
            if(tpd_enable_ps(0) > 0)
                printk("tpd_enable_ps disable fail\n");
            else
                printk("tpd_enable_ps disable sucess\n");
        }
        return count;
    }
#endif

    //nick_hsiao:read FW version for hex output
    if( buf[0] == 'n') //firmware version
    {
        debug_level_cmd = buf[0];
        if(himax_read_FW_ver() != 0)
        {
            count += sprintf(buf+count, "get FW version fail\n");
        }
        return count;
    }

    //ESD Workaround Start
#ifdef ENABLE_CHIP_STATUS_MONITOR
    cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif
    //ESD Workaround End

    //Wakelock Protect start
    wake_lock(&private_ts->wake_lock);
    //Wakelock Protect end

    //Mutexlock Protect Start
    mutex_lock(&private_ts->mutex_lock);
    //Mutexlock Protect End


#ifdef HX_TP_FW_UPDATE
    if(buf[0] == 't')
    {
        debug_level_cmd = buf[0];
        memset(fileName, 0, 128);
        // parse the file name
        snprintf(fileName, count-2, "%s", &buf[2]);
        printk(KERN_INFO "[TP] %s: upgrade from file(%s) start!\n", __func__, fileName);
        // open file
        filp = filp_open(fileName, O_RDONLY, 0);
        if(IS_ERR(filp))
        {
            printk(KERN_ERR "[TP] %s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        // read the latest firmware binary file
        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0)
        {
            printk(KERN_ERR "[TP] %s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk(KERN_INFO "[TP] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

        if(result > 0)
        {
            // start to upgrade
            disable_irq(private_ts->client->irq);
            fw_update_complete = false;
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
            {
                printk(KERN_INFO "[TP] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
                count += sprintf(buf, "TP upgrade error\n");
                fw_update_complete = false;
            }
            else
            {
                printk(KERN_INFO "[TP] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
                count += sprintf(buf, "TP upgrade pass\n");
                fw_update_complete = true;
            }
            enable_irq(private_ts->client->irq);
            goto firmware_upgrade_done;
            //return count;
        }
    }

    //printk("himax touch test_fw: %d, buf[0]: %s, upgrade_fw_len: %d\n", test_fw, buf[0], upgrade_fw_len);

    /*#ifdef HX_FW_UPDATE_BY_I_FILE
        if(buf[0] == 'f')
        {
            printk(KERN_INFO "[TP] %s: upgrade firmware from kernel image start!\n", __func__);
            if (i_isTP_Updated == 0)
            {
                printk("himax touch isTP_Updated: %d\n", i_isTP_Updated);
                if(1)// (himax_read_FW_ver() == 0)
                {
                    disable_irq(private_ts->client->irq);
                    printk("himax touch firmware upgrade: %d\n", i_isTP_Updated);
                    if(fts_ctpm_fw_upgrade_with_i_file() == 0)
                    {
                        printk("himax_marked TP upgrade error, line: %d\n", __LINE__);
                        fw_update_complete = false;
                    }
                    else
                    {
                        printk("himax_marked TP upgrade OK, line: %d\n", __LINE__);
                        fw_update_complete = true;
                    }
                    enable_irq(private_ts->client->irq);
                    //i_isTP_Updated = 1;
                    goto firmware_upgrade_done;
                }
            }
        }
    #endif*/
#endif

firmware_upgrade_done:
    //ESD Workaround Start

    //Mutexlock Protect Start
    mutex_unlock(&private_ts->mutex_lock);
    //Mutexlock Protect End

#ifdef ENABLE_CHIP_RESET_MACHINE
    queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
    //todo himax_chip->tp_firmware_upgrade_proceed = 0;
    //todo himax_chip->suspend_state = 0;
    //todo enable_irq(himax_chip->irq);

    //Wakelock Protect start
    wake_unlock(&private_ts->wake_lock);
    //Wakelock Protect end

#ifdef ENABLE_CHIP_STATUS_MONITOR
    queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 10*HZ);
#endif
    //ESD Workaround End

    //nick_hsiao:enable irq while finishing update
    enable_irq(&private_ts->client->irq);
    return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO|S_IWGRP),
                   himax_debug_level_show, himax_debug_level_dump);


static ssize_t himax_diag_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    size_t count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;

    mutual_num = x_channel * y_channel;
    self_num = x_channel + y_channel; //don't add KEY_COUNT

    width = x_channel;
    count += sprintf(buf + count, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);

    // start to show out the raw data in adb shell
    if (diag_command >= 1 && diag_command <= 6)
    {
        if (diag_command <= 3)
        {
            for (loop_i = 0; loop_i < mutual_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                {
                    count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
                }
            }
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < width; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }

#ifdef HX_EN_BUTTON
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < HX_KEY_COUNT; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL+loop_i]);
            }
#endif

        }
        else if (diag_command > 4)
        {
            for (loop_i = 0; loop_i < self_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
        else
        {
            for (loop_i = 0; loop_i < mutual_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
        count += sprintf(buf + count, "ChannelEnd");
        count += sprintf(buf + count, "\n");
    }
    else if (diag_command == 7)
    {
        for (loop_i = 0; loop_i < 128 ; loop_i++)
        {
            if((loop_i % 16) == 0)
            {
                count += sprintf(buf + count, "LineStart:");
            }

            count += sprintf(buf + count, "%4d", diag_coor[loop_i]);
            if((loop_i % 16) == 15)
            {
                count += sprintf(buf + count, "\n");
            }
        }
    }

    return count;
}

static ssize_t himax_diag_dump_func(const char *buf)
{
    const uint8_t command_ec_128_raw_flag = 0x01;
    const uint8_t command_ec_24_normal_flag = 0x00;
#ifndef HX_85XX_D_SERIES_PWON
    const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
#else
    const uint8_t command_ec_128_raw_baseline_flag = 0x02;
    const uint8_t command_ec_128_raw_bank_flag = 0x03;
#endif
    uint8_t command_91h[2] = {0x91, 0x00};
    uint8_t command_82h[1] = {0x82};
    uint8_t command_F3h[2] = {0xF3, 0x00};
    uint8_t command_83h[1] = {0x83};
    uint8_t receive[1];

    if (buf[0] == '1')
    {
        command_91h[1] = command_ec_128_raw_baseline_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else if (buf[0] == '2')
    {
        command_91h[1] = command_ec_128_raw_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else if (buf[0] == '3')
    {
#ifndef HX_85XX_D_SERIES_PWON
        i2c_himax_write(touch_i2c, command_82h[0] ,&command_82h[0], 0, DEFAULT_RETRY_CNT);
        msleep(50);

        i2c_himax_read(touch_i2c, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT);

        command_F3h[1] = (receive[0] | 0x80);

        i2c_himax_write(touch_i2c, command_F3h[0] ,&command_F3h[1], 1, DEFAULT_RETRY_CNT);

        command_91h[1] = command_ec_128_raw_baseline_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);

        i2c_himax_write(touch_i2c, command_83h[0] ,&command_83h[0], 0, DEFAULT_RETRY_CNT);
        msleep(50);
#else
        command_91h[1] = command_ec_128_raw_bank_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
#endif
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else if (buf[0] == '7')
    {
        diag_command = buf[0] - '0';
    }
#ifdef HX_TP_COORDINATE_DUMP
    else if (buf[0] == '8')
    {
        diag_command = buf[0] - '0';
    }
    else if (buf[0] == '9')
    {
        coordinate_dump_enable = 0;
        coordinate_dump_file_create = 0;
        diag_command = buf[0] - '0';
    }
#endif
    else
    {
#ifndef HX_85XX_D_SERIES_PWON
        i2c_himax_write(touch_i2c, command_82h[0] ,&command_82h[0], 0, DEFAULT_RETRY_CNT);
        msleep(50);

        command_91h[1] = command_ec_24_normal_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);

        i2c_himax_read(touch_i2c, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT);
        command_F3h[1] = (receive[0] & 0x7F);
        i2c_himax_write(touch_i2c, command_F3h[0] ,&command_F3h[1], 1, DEFAULT_RETRY_CNT);

        i2c_himax_write(touch_i2c, command_83h[0] ,&command_83h[0], 0, DEFAULT_RETRY_CNT);
#else
        command_91h[1] = command_ec_24_normal_flag;
        i2c_himax_write(touch_i2c, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
#endif
        diag_command = 0;
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }



    /*
        if (buf[0] == '1' || buf[0] == '3' || buf[0] == '5') {
        	  // start to dump the DC value
            new_command[1] = command_ec_128_raw_baseline_flag;
            i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1, &new_command[1]);
            diag_command = buf[0] - '0';
        } else if (buf[0] == '2' || buf[0] == '4' || buf[0] == '6') {
            // start to dump the IIR value
            new_command[1] = command_ec_128_raw_flag;
            i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1, &new_command[1]);
            diag_command = buf[0] - '0';
        } else {
            // stop for dumping the raw-count
            new_command[1] = command_ec_24_normal_flag;
            i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1, &new_command[1]);
            diag_command = 0;
        }
    */
}

static ssize_t himax_diag_dump(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    himax_diag_dump_func(buf);
    return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO|S_IWGRP),
                   himax_diag_show, himax_diag_dump);

static ssize_t himax_chip_raw_data_store_func(char *buf, char *PreFilePath)
{
    size_t count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;
    char filePath[50];
    struct file* filp = NULL;
    mm_segment_t oldfs;

    mutual_num = x_channel * y_channel;
    self_num = x_channel + y_channel;
    width = x_channel;
    strcpy(filePath, PreFilePath);
    if(diag_command == 1)
    {
        strcat(filePath,"/touch_dc.txt");
        filp = filp_open(filePath, O_RDWR|O_CREAT,S_IRUSR);
        if(IS_ERR(filp))
        {
            printk(KERN_ERR "[Himax] %s: open %s failed\n", __func__, filePath);
            return 0;
        }
        oldfs = get_fs();
        set_fs(get_ds());
    }
    else if(diag_command == 2)
    {
        strcat(filePath,"/touch_iir.txt");
        filp = filp_open(filePath, O_RDWR|O_CREAT,S_IRUSR);
        if(IS_ERR(filp))
        {
            printk(KERN_ERR "[Himax] %s: open %s failed\n", __func__, filePath);
            return 0;
        }
        oldfs = get_fs();
        set_fs(get_ds());
    }
    else if(diag_command == 3)
    {
        strcat(filePath,"/touch_bank.txt");
        filp = filp_open(filePath, O_RDWR|O_CREAT,S_IRUSR);
        if(IS_ERR(filp))
        {
            printk(KERN_ERR "[Himax] %s: open %s failed\n", __func__, filePath);
            return 0;
        }
        oldfs = get_fs();
        set_fs(get_ds());
    }
    count += sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);
    if (diag_command >= 1 && diag_command <= 6)
    {
        if (diag_command < 4)
        {
            for (loop_i = 0; loop_i < mutual_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);

                if ((loop_i % width) == (width - 1))
                {
                    count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
                }
            }
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < width; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                {
                    count += sprintf(buf + count, "\n");
                }
            }

#ifdef HX_EN_BUTTON
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < HX_KEY_COUNT; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL+loop_i]);
            }
#endif
        }
        else if (diag_command > 4)
        {
            for (loop_i = 0; loop_i < self_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
        else
        {
            for (loop_i = 0; loop_i < mutual_num; loop_i++)
            {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
    }
    if(diag_command >= 1 && diag_command <= 3)
    {
        filp->f_op->write(filp, buf, count, &filp->f_pos);
        set_fs(oldfs);
        filp_close(filp, NULL);
    }
    return count;
}
static ssize_t himax_chip_raw_data_store(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return himax_chip_raw_data_store_func(buf,"/data/local");
}

static DEVICE_ATTR(tp_output_raw_data, (S_IWUSR|S_IRUGO|S_IWGRP), himax_chip_raw_data_store, himax_diag_dump);

//Shouchung add ATD tool function
static ssize_t himax_get_touch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t buf0[3];
    int ret = -1;
    int test_count = 5;
    int i;

    buf0[0] = 0xE9;
    buf0[1] = 0x01;
    buf0[2] = 0x01;

    for (i = 0; i < test_count; i++)
    {
        ret = i2c_himax_master_write(private_ts->client, buf0, 3, DEFAULT_RETRY_CNT);
        if(ret < 0)
        {
            break;
        }
        msleep(50);
    }

    return sprintf(buf, "%d\n", (ret < 0) ? 0 : 1);
}
static DEVICE_ATTR(touch_status, (S_IRUGO), himax_get_touch_status, NULL);
//Shouchung end


static ssize_t himax_set_touch_switch(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(buf[0] == '0')
    {
        //set touch off
        disable_irq(private_ts->client->irq);
        irq_enable = false;
        printk("himax_touch_off\n");
    }
    else if(buf[0] == '1')
    {
        //set touch on
        enable_irq(private_ts->client->irq);
        irq_enable = true;
        printk("himax_touch_on\n");
    }
}
static DEVICE_ATTR(touch_switch, (S_IWGRP|S_IWUSR),NULL, himax_set_touch_switch);

static ssize_t himax_get_touch_panel_id(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", TP_ID);
}
static DEVICE_ATTR(tp_id, (S_IRUGO),himax_get_touch_panel_id, NULL);

static struct attribute *himax_attr[] =
{
    &dev_attr_register.attr,
    &dev_attr_vendor.attr,
    &dev_attr_debug_level.attr,
    &dev_attr_diag.attr,
    &dev_attr_tp_self_test.attr,
    &dev_attr_tp_output_raw_data.attr,
    &dev_attr_touch_status.attr, //Shouchung add ATD tool function
    &dev_attr_touch_switch.attr, //TsuanMin add touch switch for pen
    &dev_attr_tp_id.attr, //TsuanMin add touch panel id
#ifdef TPD_PROXIMITY
    &dev_attr_ps_object_distance.attr,
    &dev_attr_ps_status.attr,
    &dev_attr_ps_poll_time.attr,
    &dev_attr_ps_has_data.attr,
#endif
    NULL
};

static int himax_touch_sysfs_init(void)
{
    int ret;
    android_touch_kobj = kobject_create_and_add("android_touch", NULL);
    if (android_touch_kobj == NULL)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: subsystem_register failed\n");
        ret = -ENOMEM;
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: create_file debug_level failed\n");
        return ret;
    }
    register_command = 0;
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: create_file register failed\n");
        return ret;
    }

    ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
        return ret;
    }

    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_self_test failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_output_raw_data failed\n");
        return ret;
    }

#ifdef HX_TP_SYS_FLASH_DUMP
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
        return ret;
    }
#endif

#ifdef HX_TP_SYS_HITOUCH
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_hitouch.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
        return ret;
    }
#endif

#ifdef HX_TP_SYS_RESET
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
    if (ret)
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
        return ret;
    }
#endif

    //Shouchung add ATD tool function
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_status failed\n");
        return ret;
    }
    //Shouchung end

    //TsuanMin add touch switch for pen
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_switch.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_switch failed\n");
        return ret;
    }
    //TsuanMin end

    //TsuanMin add touch panel id
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_id.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_id failed\n");
        return ret;
    }
    //TsuanMin end

#ifdef TPD_PROXIMITY
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_ps_object_distance.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_ps_object_distance failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_ps_status.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_ps_status failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_ps_poll_time.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_ps_poll_time failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_ps_has_data.attr);
    if (ret)
    {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_ps_has_data failed\n");
        return ret;
    }
#endif

    return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
    sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);

    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr);
    //Shouchung add ATD tool function
    sysfs_remove_file(android_touch_kobj, &dev_attr_touch_status.attr);
    //Shouchung end
    sysfs_remove_file(android_touch_kobj, &dev_attr_touch_switch.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_id.attr);
#ifdef HX_TP_SYS_RESET
    sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
#endif
#ifdef TPD_PROXIMITY
    sysfs_remove_file(android_touch_kobj, &dev_attr_ps_object_distance.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_ps_status.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_ps_poll_time.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_ps_has_data.attr);
#endif

    kobject_del(android_touch_kobj);
}
#endif
/******************** Himax: Debug Tool function ********************/

/******************** Himax: Fundamental function ********************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
static void himax_ts_flash_work_func(struct work_struct *work)
{
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

    uint8_t page_tmp[128];
    uint8_t x59_tmp[4] = {0,0,0,0};
    int i=0, j=0, k=0, l=0,/* j_limit = 0,*/ buffer_ptr = 0, flash_end_count = 0;
    uint8_t local_flash_command = 0;
    uint8_t sector = 0;
    uint8_t page = 0;

    uint8_t x81_command[2] = {0x81,0x00};
    uint8_t x82_command[2] = {0x82,0x00};
    uint8_t x83_command[2] = {0x83,0x00};
    uint8_t x42_command[2] = {0x42,0x00};
    uint8_t x43_command[4] = {0x43,0x00,0x00,0x00};
    uint8_t x44_command[4] = {0x44,0x00,0x00,0x00};
    uint8_t x45_command[5] = {0x45,0x00,0x00,0x00,0x00};
    uint8_t x46_command[2] = {0x46,0x00};
    uint8_t x4A_command[2] = {0x4A,0x00};
    uint8_t x4D_command[2] = {0x4D,0x00};
    /*uint8_t x59_command[2] = {0x59,0x00};*/

    disable_irq(ts->client->irq);
    setFlashDumpGoing(true);

    sector = getFlashDumpSector();
    page = getFlashDumpPage();

    local_flash_command = getFlashCommand();

    if( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 )//sleep out
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 81 fail.\n",__func__);
        goto Flash_Dump_i2c_transfer_error;
    }
    msleep(120);

    if( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 )
    {
        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 82 fail.\n",__func__);
        goto Flash_Dump_i2c_transfer_error;
    }
    msleep(100);

    printk(KERN_INFO "[TP] %s: local_flash_command = %d enter.\n", __func__,local_flash_command);
    printk(KERN_INFO "[TP] %s: flash buffer start.\n", __func__);
    for(i=0; i<128; i++)
    {
        printk(KERN_INFO " %2.2x ",flash_buffer[i]);
        if((i%16) == 15)
        {
            printk("\n");
        }
    }
    printk(KERN_INFO "[TP] %s: flash buffer end.\n", __func__);

    if(local_flash_command == 1 || local_flash_command == 2)
    {
        x43_command[1] = 0x01;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
        {
            goto Flash_Dump_i2c_transfer_error;
            return;
        }
        msleep(100);

        for( i=0 ; i<8 ; i++)
        {
            for(j=0 ; j<32 ; j++)
            {
                printk("TPPPP Step 2 i=%d , j=%d %s\n",i,j,__func__);
                //read page start
                for(k=0; k<128; k++)
                {
                    page_tmp[k] = 0x00;
                }
                for(k=0; k<32; k++)
                {
                    x44_command[1] = k;
                    x44_command[2] = j;
                    x44_command[3] = i;
                    if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
                    {
                        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }

                    if( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
                    {
                        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 46 fail.\n",__func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }
                    //msleep(2);
                    if( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
                    {
                        printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 59 fail.\n",__func__);
                        goto Flash_Dump_i2c_transfer_error;
                    }
                    //msleep(2);
                    for(l=0; l<4; l++)
                    {
                        page_tmp[k*4+l] = x59_tmp[l];
                    }
                    //msleep(10);
                }
                //read page end

                for(k=0; k<128; k++)
                {
                    flash_buffer[buffer_ptr++] = page_tmp[k];
                    /*
                    if(page_tmp[k] == 0xFF)
                    {
                    	flash_end_count ++;
                    	if(flash_end_count == 32)
                    	{
                    		flash_end_count = 0;
                    		buffer_ptr = buffer_ptr -32;
                    		goto FLASH_END;
                    	}
                    }
                    else
                    {
                    	flash_end_count = 0;
                    }*/
                }
                setFlashDumpProgress(i*32 + j);
            }
        }
    }
    else if(local_flash_command == 3)
    {
        x43_command[1] = 0x01;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);

        for(i=0; i<128; i++)
        {
            page_tmp[i] = 0x00;
        }

        for(i=0; i<32; i++)
        {
            x44_command[1] = i;
            x44_command[2] = page;
            x44_command[3] = sector;

            if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }

            if( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 46 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            //msleep(2);
            if( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 59 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            //msleep(2);
            for(j=0; j<4; j++)
            {
                page_tmp[i*4+j] = x59_tmp[j];
            }
            //msleep(10);
        }
        //read page end
        for(i=0; i<128; i++)
        {
            flash_buffer[buffer_ptr++] = page_tmp[i];
        }
    }
    else if(local_flash_command == 4)
    {
        //page write flow.
        printk(KERN_INFO "[TP] %s: local_flash_command = 4, enter.\n", __func__);

        //-----------------------------------------------------------------------------------------------
        // unlock flash
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        x43_command[3] = 0x06;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x44_command[1] = 0x03;
        x44_command[2] = 0x00;
        x44_command[3] = 0x00;
        if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x45_command[1] = 0x00;
        x45_command[2] = 0x00;
        x45_command[3] = 0x3D;
        x45_command[4] = 0x03;
        if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        if( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4A fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(50);

        //-----------------------------------------------------------------------------------------------
        // page erase
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        x43_command[3] = 0x02;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x44_command[1] = 0x00;
        x44_command[2] = page;
        x44_command[3] = sector;
        if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        if( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4D fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);

        //-----------------------------------------------------------------------------------------------
        // enter manual mode
        //-----------------------------------------------------------------------------------------------
        x42_command[1] = 0x01;
        if( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 42 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(100);

        //-----------------------------------------------------------------------------------------------
        // flash enable
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        //-----------------------------------------------------------------------------------------------
        // set flash address
        //-----------------------------------------------------------------------------------------------
        x44_command[1] = 0x00;
        x44_command[2] = page;
        x44_command[3] = sector;
        if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        //-----------------------------------------------------------------------------------------------
        // manual mode command : 47 to latch the flash address when page address change.
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x09;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x43_command[1] = 0x01;
        x43_command[2] = 0x0D;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x43_command[1] = 0x01;
        x43_command[2] = 0x09;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        for(i=0; i<32; i++)
        {
            printk(KERN_INFO "himax :i=%d \n",i);
            x44_command[1] = i;
            x44_command[2] = page;
            x44_command[3] = sector;
            if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);

            x45_command[1] = flash_buffer[i*4 + 0];
            x45_command[2] = flash_buffer[i*4 + 1];
            x45_command[3] = flash_buffer[i*4 + 2];
            x45_command[4] = flash_buffer[i*4 + 3];
            if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);

            //-----------------------------------------------------------------------------------------------
            // manual mode command : 48 ,data will be written into flash buffer
            //-----------------------------------------------------------------------------------------------
            x43_command[1] = 0x01;
            x43_command[2] = 0x0D;
            if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);

            x43_command[1] = 0x01;
            x43_command[2] = 0x09;
            if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
            {
                printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
                goto Flash_Dump_i2c_transfer_error;
            }
            msleep(10);
        }

        //-----------------------------------------------------------------------------------------------
        // manual mode command : 49 ,program data from flash buffer to this page
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x01;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x43_command[1] = 0x01;
        x43_command[2] = 0x05;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x43_command[1] = 0x01;
        x43_command[2] = 0x01;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        //-----------------------------------------------------------------------------------------------
        // flash disable
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x00;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        //-----------------------------------------------------------------------------------------------
        // leave manual mode
        //-----------------------------------------------------------------------------------------------
        x42_command[1] = 0x00;
        if( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        //-----------------------------------------------------------------------------------------------
        // lock flash
        //-----------------------------------------------------------------------------------------------
        x43_command[1] = 0x01;
        x43_command[2] = 0x00;
        x43_command[3] = 0x06;
        if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x44_command[1] = 0x03;
        x44_command[2] = 0x00;
        x44_command[3] = 0x00;
        if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        x45_command[1] = 0x00;
        x45_command[2] = 0x00;
        x45_command[3] = 0x7D;
        x45_command[4] = 0x03;
        if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }
        msleep(10);

        if( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
        {
            printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4D fail.\n",__func__);
            goto Flash_Dump_i2c_transfer_error;
        }

        msleep(50);

        buffer_ptr = 128;
        printk(KERN_INFO "Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
    }

FLASH_END:

    printk("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

    printk(" buffer_ptr = %d \n",buffer_ptr);

    for (i = 0; i < buffer_ptr; i++)
    {
        printk("%2.2X ", flash_buffer[i]);

        if ((i % 16) == 15)
        {
            printk("\n");
        }
    }
    printk("End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    i2c_himax_master_write(ts->client, x43_command, 1, 3);
    msleep(50);

    if(local_flash_command == 2)
    {
        struct file *fn;

        fn = filp_open(FLASH_DUMP_FILE,O_CREAT | O_WRONLY ,0);
        if(!IS_ERR(fn))
        {
            fn->f_op->write(fn,flash_buffer,buffer_ptr*sizeof(uint8_t),&fn->f_pos);
            filp_close(fn,NULL);
        }
    }

    enable_irq(ts->client->irq);
    setFlashDumpGoing(false);

    setFlashDumpComplete(1);
    setSysOperation(0);
    return;

Flash_Dump_i2c_transfer_error:
    enable_irq(ts->client->irq);
    setFlashDumpGoing(false);
    setFlashDumpComplete(0);
    setFlashDumpFail(1);
    setSysOperation(0);
    return;
}
#endif

static void himax_ts_work_func(struct work_struct *work)
{
#ifdef HX_TP_SYS_FS
    uint8_t *mutual_data;
    uint8_t *self_data;
    int mul_num, self_num;
    int index = 0;
#endif

    int ret, i;
    unsigned int x=0, y=0, area=0, press=0;
    const unsigned int x_res = private_ts->abs_x_max;
    const unsigned int y_res = private_ts->abs_y_max;
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
    uint8_t diag_cmd;
    unsigned char check_sum_cal = 0;
    struct i2c_msg msg[2];
    uint8_t start_reg;
    uint8_t buf[128] = {0};
    int RawDataLen = 0;
    unsigned int temp_x[HX_TP_MAX_FINGER], temp_y[HX_TP_MAX_FINGER];
#ifdef HX_TP_COORDINATE_DUMP
    char coordinate_char[15+(HX_TP_MAX_FINGER+5)*2*5+2];
    struct timeval t;
    struct tm broken;
#endif
    //Bizzy added for common RawData
    int raw_cnt_max = HX_TP_MAX_FINGER/4;
    int raw_cnt_rmd = HX_TP_MAX_FINGER%4;
    int hx_touch_info_size;
    if(raw_cnt_rmd != 0x00)
    {
#ifdef HX_85XX_D_SERIES_PWON
        RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+3)*4) - 1;
#else
        RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+3)*4);
#endif
        hx_touch_info_size = (HX_TP_MAX_FINGER+raw_cnt_max+2)*4;
    }
    else
    {
#ifdef HX_85XX_D_SERIES_PWON
        RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+2)*4) - 1;
#else
        RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+2)*4);
#endif
        hx_touch_info_size = (HX_TP_MAX_FINGER+raw_cnt_max+1)*4;
    }

    //ESD Workaround Start
#ifdef ENABLE_CHIP_STATUS_MONITOR
    ts->running_status = 1;
    cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif
    //ESD Workaround End

    start_reg = HX_CMD_RAE;
    msg[0].addr = ts->client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &start_reg;

    msg[1].addr = ts->client->addr;
    msg[1].flags = I2C_M_RD;

    if(diag_command)
    {
#ifdef HX_TP_SYS_FS
#ifdef HX_85XX_D_SERIES_PWON
        msg[1].len =  128;//hx_touch_info_size + RawDataLen + 4 + 1;	//4: RawData Header
#else
        msg[1].len =  128;//hx_touch_info_size + RawDataLen + 4;	//4: RawData Header
#endif
#else
        msg[1].len =  hx_touch_info_size;
#endif
    }
    else
    {
        msg[1].len =  hx_touch_info_size;
    }

    msg[1].buf = buf;


    //#ifdef HX_ISR_ENTER_CHECK
    //printk(KERN_ERR "******Himax Touch Controller Trigger ISR test\n");
    //#endif

    //Mutexlock Protect Start
    mutex_lock(&ts->mutex_lock);
    //Mutexlock Protect End

    // Read out all events from touch IC
    ret = i2c_transfer(ts->client->adapter, msg, 2);
    if (ret < 0)
    {
        //printk(KERN_INFO "[Himax]:%s:i2c_transfer fail.\n", __func__);
        memset(buf, 0xff , 128);

        //ESD Workaround Start
#ifdef ENABLE_CHIP_RESET_MACHINE

        //Mutexlock Protect Start
        mutex_unlock(&ts->mutex_lock);
        //Mutexlock Protect End

        enable_irq(ts->client->irq);
        goto work_func_send_i2c_msg_fail;
#endif
        //ESD Workaround End
    }

    //ESD Workaround Start
#ifdef ESD_WORKAROUND
    for(i = 0; i < hx_touch_info_size; i++)
    {
        if(buf[i] == 0x00)
        {
            check_sum_cal = 1;
        }
        else if(buf[i] == 0xED)
        {
            check_sum_cal = 2;
        }
        else
        {
            check_sum_cal = 0;
            i = hx_touch_info_size;
        }
    }

    diag_cmd = getDiagCommand();
    if(check_sum_cal != 0 && reset_activate == 0 && diag_cmd == 0)  //ESD Check
    {
        //Mutexlock Protect Start
        mutex_unlock(&ts->mutex_lock);
        //Mutexlock Protect End
        ret = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail
        enable_irq(ts->client->irq);

        if(ret == 2)
        {
            goto work_func_send_i2c_msg_fail;
        }

        if((ret == 1) && (check_sum_cal == 1))
        {
            printk("[Himax]: ESD event checked - ALL Zero.\n");
            ESD_HW_REST();
        }
        else if(check_sum_cal == 2)
        {
            printk("[Himax]: ESD event checked - ALL 0xED.\n");
            ESD_HW_REST();
        }

#ifdef ENABLE_CHIP_STATUS_MONITOR
        ts->running_status = 0;
        queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
        return;
    }
    else if(reset_activate)
    {
        reset_activate = 0;
        //printk(KERN_INFO "[Himax]:%s: Back from ESD reset, ready to serve.\n", __func__);
        //Mutexlock Protect Start
        mutex_unlock(&ts->mutex_lock);
        //Mutexlock Protect End
        enable_irq(ts->client->irq);

#ifdef ENABLE_CHIP_STATUS_MONITOR
        ts->running_status = 0;
        queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
        return;
    }
#endif
    //ESD Workaround End

    for(i = 0; i < hx_touch_info_size; i++)
    {
        check_sum_cal += buf[i];
    }

    // check the checksum of received i2c packet
    if ((check_sum_cal != 0x00) || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0 )!= 0xF0)
    {
        //printk(KERN_INFO "check_sum_cal: 0x%02X\n", check_sum_cal);

        //Mutexlock Protect Start
        mutex_unlock(&ts->mutex_lock);
        //Mutexlock Protect End

        enable_irq(ts->client->irq);

        //ESD Workaround Start
#ifdef ESD_WORKAROUND
        ESD_COUNTER++;
        //printk("[Himax]: ESD event checked - check_sum_cal, ESD_COUNTER = %d.\n", ESD_COUNTER);
        if(ESD_COUNTER > ESD_COUNTER_SETTING)
        {
            ESD_HW_REST();
        }
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
        ts->running_status = 0;
        queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
        //ESD Workaround End
        return;
    }

#ifdef HX_TP_SYS_FS
    // Print out the raw data by printk
    if (getDebugLevel() & 0x1)
    {
        //printk(KERN_INFO "[TP]%s: raw data:\n", __func__);
        for (i = 0; i < 128; i=i+8)
        {
            //printk(KERN_INFO "%d: 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X \n", i, buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
        }
    }

    // Print out the raw data by sysfs node
    diag_cmd = getDiagCommand();
    if (diag_cmd >= 1 && diag_cmd <= 6)
    {
        //printk(KERN_INFO "[TP]%s: Diag Raw Data:\n", __func__);

#ifdef HX_85XX_D_SERIES_PWON
        //Check 128th byte CRC
        for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++)
        {
            check_sum_cal += buf[i];
        }
        if (check_sum_cal % 0x100 != 0)
        {
            goto bypass_checksum_failed_packet;
        }
#endif

        mutual_data = getMutualBuffer();
        self_data = getSelfBuffer();
        // initiallize the block number of mutual and self
        mul_num = getXChannel() * getYChannel();

#ifdef HX_EN_BUTTON
        self_num = getXChannel() + getYChannel() + HX_KEY_COUNT;
#else
        self_num = getXChannel() + getYChannel();
#endif

        //Himax: Check Raw-Data Header
        if(buf[hx_touch_info_size] == buf[hx_touch_info_size+1] && buf[hx_touch_info_size+1] == buf[hx_touch_info_size+2]
                && buf[hx_touch_info_size+2] == buf[hx_touch_info_size+3] && buf[hx_touch_info_size] > 0)
        {
            index = (buf[hx_touch_info_size] - 1) * RawDataLen;
            //printk("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
            for (i = 0; i < RawDataLen; i++)
            {
#ifdef HX_85XX_D_SERIES_PWON
                if ((index+i) < mul_num)
#else
                if (index < mul_num)
#endif
                {
                    //mutual
                    mutual_data[index + i] = buf[i + hx_touch_info_size+4];	//4: RawData Header
                }
                else
                {
                    //self
#ifdef HX_85XX_D_SERIES_PWON
                    if ((i+index) >= (self_num+mul_num))
#else
                    if (i >= self_num)
#endif
                        break;

#ifdef HX_85XX_D_SERIES_PWON
                    self_data[i+index-mul_num] = buf[i + hx_touch_info_size+4];	//4: RawData Header
#else
                    self_data[i] = buf[i + hx_touch_info_size+4];	//4: RawData Header
#endif
                }
            }
        }
        else
        {
            //printk(KERN_INFO "[TP]%s: header format is wrong!\n", __func__);
        }
    }
    else if(diag_cmd == 7)
    {
        memcpy(&(diag_coor[0]), &buf[0], 128);
    }
#ifdef HX_TP_COORDINATE_DUMP
    else if( (diag_cmd == 8) && (coordinate_dump_file_create == 0) )
    {
        //printk(KERN_INFO "[TP]%s: diag = 8\n", __func__);
        coordinate_dump_enable = 1;
        coordinate_dump_file_create = 1;
    }
    else if(diag_cmd == 9)
    {
        //printk(KERN_INFO "[TP]%s: diag = 9\n", __func__);
        coordinate_dump_enable = 0;
        coordinate_dump_file_create = 0;
        filp_close(fn,NULL);
    }

    if(coordinate_dump_file_create == 1)
    {
        sys_unlink("/sdcard/Coordinate_Dump.csv");
        fn = filp_open("/sdcard/Coordinate_Dump.csv",O_CREAT | O_WRONLY | O_APPEND ,0666);
        if(fn == NULL)
        {
            printk(KERN_INFO "[TP]%s: coordinate_dump_file_create error\n", __func__);
            coordinate_dump_enable = 0;
            filp_close(fn,NULL);
        }
        coordinate_dump_file_create = 2;
    }

    for(i=0; i<(15 + (HX_TP_MAX_FINGER+5)*2*5); i++)
    {
        coordinate_char[i] = 0x20;
    }
    coordinate_char[15 + (HX_TP_MAX_FINGER+5)*2*5] = 0xD;
    coordinate_char[15 + (HX_TP_MAX_FINGER+5)*2*5 + 1] = 0xA;
#endif
#endif

#ifdef HX_85XX_D_SERIES_PWON
bypass_checksum_failed_packet:
#endif

#ifdef HX_EN_BUTTON
    tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
    if(tpd_key == 0x0F)
    {
        tpd_key = 0xFF;
    }
    //printk("TPD BT:  %x\r\n", tpd_key);
#else
    tpd_key = 0xFF;
#endif

    p_point_num = hx_point_num;

    if(buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
    {
        hx_point_num = 0;
    }
    else
    {
        hx_point_num= buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
    }

    // Touch Point information
    if(hx_point_num != 0 && tpd_key == 0xFF)
    {
        // parse the point information
        for(i=0; i<HX_TP_MAX_FINGER; i++)
        {
            //if(buf[4*i] != 0xFF)
            //{
            // x and y axis
            x = buf[4 * i + 1] | (buf[4 * i] << 8) ;
            y = buf[4 * i + 3] | (buf[4 * i + 2] << 8);

            temp_x[i] = x;
            temp_y[i] = y;

#ifdef TPD_PROXIMITY
            if (tpd_proximity_flag == 1)
            {
                if(buf[4 * i] == 0xFE && buf[4 * i + 1] == 0xFE && buf[4 * i + 2] == 0xFE && buf[4 * i + 3] == 0xFE)
                {
                    tpd_proximity_detect = 0;	//Proxi	Detect
                    //printk("fake p sensor near\n");
                    break;
                }
            }
#endif

            if((x <= x_res) && (y <= y_res))
            {
                // caculate the pressure and area
                press = buf[4*HX_TP_MAX_FINGER+i];
                area = press;
                if(area > 31)
                {
                    area = (area >> 3);
                }
                if(area == 0)
                    continue;
                // kernel call for report point area, pressure and x-y axis
                input_report_key(ts->input_dev, BTN_TOUCH, 1);             // touch down
                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);     //ID of touched point
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, area); //Finger Size
                input_report_abs(ts->input_dev, ABS_MT_PRESSURE, press);   // Pressure
                if(HW_ID == PROJ_FE170CG_HWID_SR)//for SR
                {
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_res - x);     // X axis
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_res - y);     // Y axis
                }
                else
                {
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);     // X axis
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);     // Y axis
                }

                input_mt_sync(ts->input_dev);

                //if (unlikely(himax_debug_flag))
                //{
                //printk(KERN_INFO "[himax] %s: x = %d, y = %d, area = %d, press = %d!", __func__, x, y, area, press);
                //}
#ifdef HX_TP_COORDINATE_DUMP
                if(coordinate_dump_enable == 1)
                {
                    do_gettimeofday(&t);
                    time_to_tm(t.tv_sec, 0, &broken);

                    sprintf(&coordinate_char[0], "%2d:%2d:%2d:%3d,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);

                    sprintf(&coordinate_char[15 + (i*2)*5], "%4d,", x);
                    sprintf(&coordinate_char[15 + (i*2)*5 + 5], "%4d,", y);
                }
#endif
            }

            //}
            /*else
            {
                temp_x[i] = 0xFFFF;
                temp_y[i] = 0xFFFF;
                input_mt_sync(ts->input_dev);
            }*/
        }
        input_sync(ts->input_dev);
        //ESD Workaround Start
#ifdef ESD_WORKAROUND
        ESD_COUNTER = 0;
#endif
        //ESD Workaround End
    }
    else if(hx_point_num == 0 && tpd_key != 0xFF)
    {
        temp_x[0] = 0xFFFF;
        temp_y[0] = 0xFFFF;
        temp_x[1] = 0xFFFF;
        temp_y[1] = 0xFFFF;

        if( tpd_key == 1)
        {
            input_report_key(ts->input_dev, tpd_keys_local[0], 1);
            input_sync(ts->input_dev);
            //printk("Press BT1*** \r\n");
        }
        if( tpd_key == 2)
        {
            input_report_key(ts->input_dev, tpd_keys_local[1], 1);
            input_sync(ts->input_dev);
            //printk("Press BT2*** \r\n");
        }
        if( tpd_key == 3)
        {
            input_report_key(ts->input_dev, tpd_keys_local[2], 1);
            input_sync(ts->input_dev);
            //printk("Press BT3*** \r\n");
        }
        if( tpd_key == 4)
        {
            input_report_key(ts->input_dev, tpd_keys_local[3], 1);
            input_sync(ts->input_dev);
            //printk("Press BT4*** \r\n");
        }
        //ESD Workaround Start
#ifdef ESD_WORKAROUND
        ESD_COUNTER = 0;
#endif
        //ESD Workaround End
    }
    else if(hx_point_num == 0 && tpd_key == 0xFF)
    {
        temp_x[0] = 0xFFFF;
        temp_y[0] = 0xFFFF;
        temp_x[1] = 0xFFFF;
        temp_y[1] = 0xFFFF;

        if (tpd_key_old != 0xFF)
        {
            input_report_key(ts->input_dev, tpd_keys_local[tpd_key_old-1], 0);
            input_sync(ts->input_dev);
        }
        else
        {
#ifdef TPD_PROXIMITY
            if (tpd_proximity_flag == 1)
            {
                tpd_proximity_detect = 1;	//No Proxi Detect
                //printk("fake p sensor far\n");
            }
#endif
            // leave event
            input_report_key(ts->input_dev, BTN_TOUCH, 0);  // touch up
            input_mt_sync(ts->input_dev);
            input_sync(ts->input_dev);
            if (unlikely(himax_debug_flag))
            {
                //printk(KERN_INFO "[himax] %s: touch up!", __func__);
            }
#ifdef HX_TP_COORDINATE_DUMP
            if(coordinate_dump_enable == 1)
            {
                do_gettimeofday(&t);
                time_to_tm(t.tv_sec, 0, &broken);

                sprintf(&coordinate_char[0], "%2d:%2d:%2d:%3d,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);
                sprintf(&coordinate_char[15], "Touch up!", 0);
            }
#endif
        }
        //ESD Workaround Start
#ifdef ESD_WORKAROUND
        ESD_COUNTER = 0;
#endif
        //ESD Workaround End
    }
#ifdef HX_TP_COORDINATE_DUMP
    if(coordinate_dump_enable == 1)
    {
        fn->f_op->write(fn,&coordinate_char[0],15 + (HX_TP_MAX_FINGER+5)*2*sizeof(char)*5 + 2,&fn->f_pos);
    }
#endif

#ifdef Gesture_Detemination

    if (hx_point_num == 2 && p_point_num == 2 && temp_x[0] != 0xFFFF && temp_y[0] != 0xFFFF
            && temp_x[1] != 0xFFFF && temp_y[1] != 0xFFFF)
    {
        if (temp_x[0] > temp_x[1] && temp_y[0] > temp_y[1])
        {
            Dist_Cal_Now = (temp_x[0] - temp_x[1]) + (temp_y[0] - temp_y[1]);
        }
        else if (temp_x[0] > temp_x[1] && temp_y[1] > temp_y[0])
        {
            Dist_Cal_Now = (temp_x[0] - temp_x[1]) + (temp_y[1] - temp_y[0]);
        }
        else if (temp_x[1] > temp_x[0] && temp_y[0] > temp_y[1])
        {
            Dist_Cal_Now = (temp_x[1] - temp_x[0]) + (temp_y[0] - temp_y[1]);
        }
        else if (temp_x[1] > temp_x[0] && temp_y[1] > temp_y[0])
        {
            Dist_Cal_Now = (temp_x[1] - temp_x[0]) + (temp_y[1] - temp_y[0]);
        }

        if (Dist_Cal_Now - Dist_Cal_EX > 10)
        {
            ZoomInCnt++;
        }
        else if (Dist_Cal_EX - Dist_Cal_Now > 10)
        {
            ZoomOutCnt++;
        }

        //EX_x[0]= x[0];
        //EX_y[0]= y[0];
        //EX_x[1]= x[1];
        //EX_y[1]= y[1];

        if (ZoomInCnt > 2)
        {
            ZoomInFlag = 1;
            ZoomOutFlag = 0;
        }
        else if (ZoomOutCnt > 2)
        {
            ZoomOutFlag =1;
            ZoomInFlag = 0;
        }
        Dist_Cal_EX = Dist_Cal_Now;
    }
    else if (hx_point_num == 2 && p_point_num != 2 && temp_x[0] != 0xFFFF && temp_y[0] != 0xFFFF
             && temp_x[1] != 0xFFFF && temp_y[1] != 0xFFFF)
    {
        Dist_Cal_EX = 0;
    }
    else
    {
        Dist_Cal_EX = 0xFFFF;
        Dist_Cal_Now = 0xFFFF;
        ZoomInCnt = 0;
        ZoomOutCnt = 0;
        p_point_num = 0xFFFF;
    }

#endif

#if 0
    if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff && tpd_key == 0xFF)
    {
        // leave event
        input_report_key(ts->input_dev, BTN_TOUCH, 0);  // touch up
        input_mt_sync(ts->input_dev);
        input_sync(ts->input_dev);
        if (unlikely(himax_debug_flag))
        {
            printk(KERN_INFO "[himax] %s: touch up!", __func__);
        }
    }
    else
    {
        // parse the point information
        for(i=0; i<HX_TP_MAX_FINGER; i++)
        {
            if(buf[4*i] != 0xFF)
            {
                // x and y axis
                y = buf[4 * i + 1] | (buf[4 * i] << 8) ;
                x = buf[4 * i + 3] | (buf[4 * i + 2] << 8);

                if((x <= x_res) && (y <= y_res))
                {
                    // caculate the pressure and area
                    press = buf[4*HX_TP_MAX_FINGER+i];
                    area = press;
                    if(area > 31)
                    {
                        area = (area >> 3);
                    }

                    // kernel call for report point area, pressure and x-y axis
                    input_report_key(ts->input_dev, BTN_TOUCH, 1);             // touch down
                    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);     //ID of touched point
                    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, area); //Finger Size
                    input_report_abs(ts->input_dev, ABS_MT_PRESSURE, press);   // Pressure
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);     // X axis
                    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);     // Y axis

                    input_mt_sync(ts->input_dev);

                    if (unlikely(himax_debug_flag))
                    {
                        printk(KERN_INFO "[himax] %s: x = %d, y = %d, area = %d, press = %d!", __func__, x, y, area, press);
                    }
                }
            }
            else
            {
                input_mt_sync(ts->input_dev);
            }
        }
        input_sync(ts->input_dev);
    }
#endif
    tpd_key_old = tpd_key;

    //Mutexlock Protect Start
    mutex_unlock(&ts->mutex_lock);
    //Mutexlock Protect End

    enable_irq(ts->client->irq);

    //ESD Workaround Start
#ifdef ENABLE_CHIP_STATUS_MONITOR
    ts->running_status = 0;
    queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
    //ESD Workaround End

    return;

    //ESD Workaround Start
work_func_send_i2c_msg_fail:
    printk(KERN_ERR "[Himax]:work_func_send_i2c_msg_fail: %d \n",__LINE__);

#ifdef ENABLE_CHIP_RESET_MACHINE
    if(private_ts->init_success)
    {
        queue_delayed_work(ts->himax_wq, &ts->himax_chip_reset_work, 0);
    }
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
    ts->running_status = 0;
    queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif

    return;
    //ESD Workaround End
}

static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
    struct himax_ts_data *ts = dev_id;
    struct i2c_client *client = ts->client;

    //printk("Himax interrupt handler enter\n");

    //dev_dbg(&client->dev, "[himax] %s\n", __func__);
    disable_irq_nosync(ts->client->irq);
    queue_work(ts->himax_wq, &ts->work);

    return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);
    int err = 0;


    printk("Himax debug %s client irq = %d\n",__func__,client->irq);

#ifndef HX_ENABLE_EDGE_TRIGGER
    // set to level-triger
    err = request_irq(client->irq, himax_ts_irq_handler,
                      IRQF_TRIGGER_LOW, "ts_int", ts);
#else
    // set to edge-triger
    err = request_irq(client->irq, himax_ts_irq_handler,
                      IRQF_TRIGGER_FALLING, client->name, ts);
#endif


    if (err)
        dev_err(&client->dev, "[himax] %s: request_irq %d err=%d failed\n",
                __func__, client->irq, err);
    else
        printk("[himax]%s request_irq ok \r\n",__func__);


    return err;
}

#ifdef HX_LOADIN_CONFIG
static int himax_config_flow()
{
    char data[4];
    uint8_t buf0[4];
    int i2c_CheckSum = 0;
    unsigned long i = 0;

    data[0] = 0xE3;
    data[1] = 0x00;	//reload disable
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c0[0],sizeof(c0),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c1[0],sizeof(c1),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c2[0],sizeof(c2),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c3[0],sizeof(c3),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c4[0],sizeof(c4),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c5[0],sizeof(c5),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c6[0],sizeof(c6),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c7[0],sizeof(c7),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c8[0],sizeof(c8),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c9[0],sizeof(c9),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c10[0],sizeof(c10),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c11[0],sizeof(c11),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c12[0],sizeof(c12),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c13[0],sizeof(c13),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c14[0],sizeof(c14),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c15[0],sizeof(c15),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c16[0],sizeof(c16),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c17[0],sizeof(c17),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c18[0],sizeof(c18),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c19[0],sizeof(c19),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c20[0],sizeof(c20),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c21[0],sizeof(c21),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c22[0],sizeof(c22),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c23[0],sizeof(c23),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c24[0],sizeof(c24),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c25[0],sizeof(c25),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c26[0],sizeof(c26),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c27[0],sizeof(c27),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c28[0],sizeof(c28),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c29[0],sizeof(c29),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c30[0],sizeof(c30),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c31[0],sizeof(c31),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c32[0],sizeof(c32),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c33[0],sizeof(c33),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c34[0],sizeof(c34),3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c35[0],sizeof(c35),3))<0)
    {
        goto HimaxErr;
    }

    //i2c check sum start.
    data[0] = 0xAB;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xAB;
    data[1] = 0x01;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    //------------------------------------------------------------------config bank PART c36 START
#ifdef HX_ENABLE_EDGE_TRIGGER
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x00;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c36_edge[0],sizeof(c36_edge),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c36_edge) ; i++ )
    {
        i2c_CheckSum += c36_edge[i];
    }
    printk("Himax i2c_checksum_36_size = %d \n",sizeof(c36_edge));
    printk("Himax i2c_checksum_36 = %d \n",i2c_CheckSum);

    i2c_CheckSum += 0x2AF;

    printk("Himax i2c_checksum_36 = %d \n",i2c_CheckSum);
#else
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x00;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c36_level[0],sizeof(c36_level),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c36_level) ; i++ )
    {
        i2c_CheckSum += c36_level[i];
    }
    printk("Himax i2c_checksum_36_size = %d \n",sizeof(c36_level));
    printk("Himax i2c_checksum_36 = %d \n",i2c_CheckSum);

    i2c_CheckSum += 0x2AF;

    printk("Himax i2c_checksum_36 = %d \n",i2c_CheckSum);
#endif
    //------------------------------------------------------------------config bank PART c36 END


    //------------------------------------------------------------------config bank PART c37 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x1E;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c37[0],sizeof(c37),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c37) ; i++ )
    {
        i2c_CheckSum += c37[i];
    }
    printk("Himax i2c_checksum_37_size = %d \n",sizeof(c37));
    printk("Himax i2c_checksum_37 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x2CD;
    printk("Himax i2c_checksum_37 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c37 END

    //------------------------------------------------------------------config bank PART c38 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x3C;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c38[0],sizeof(c38),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c38) ; i++ )
    {
        i2c_CheckSum += c38[i];
    }
    printk("Himax i2c_checksum_38_size = %d \n",sizeof(c38));
    printk("Himax i2c_checksum_38 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x2EB;
    printk("Himax i2c_checksum_38 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c38 END

    //------------------------------------------------------------------config bank PART c39 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x4C;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c39[0],sizeof(c39),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c39) ; i++ )
    {
        i2c_CheckSum += c39[i];
    }
    printk("Himax i2c_checksum_39_size = %d \n",sizeof(c39));
    printk("Himax i2c_checksum_39 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x2FB;
    printk("Himax i2c_checksum_39 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c39 END

    //------------------------------------------------------------------config bank PART c40 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x64;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c40[0],sizeof(c40),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c40) ; i++ )
    {
        i2c_CheckSum += c40[i];
    }
    printk("Himax i2c_checksum_40_size = %d \n",sizeof(c40));
    printk("Himax i2c_checksum_40 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x313;
    printk("Himax i2c_checksum_40 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c40 END

    //------------------------------------------------------------------config bank PART c41 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x7A;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c41[0],sizeof(c41),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c41) ; i++ )
    {
        i2c_CheckSum += c41[i];
    }
    printk("Himax i2c_checksum_41_size = %d \n",sizeof(c41));
    printk("Himax i2c_checksum_41 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x329;
    printk("Himax i2c_checksum_41 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c41 END

    //------------------------------------------------------------------config bank PART c42 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x96;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c42[0],sizeof(c42),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c42) ; i++ )
    {
        i2c_CheckSum += c42[i];
    }
    printk("Himax i2c_checksum_42_size = %d \n",sizeof(c42));
    printk("Himax i2c_checksum_42 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x345;
    printk("Himax i2c_checksum_42 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c42 END

    //------------------------------------------------------------------config bank PART c43_1 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0x9E;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c43_1[0],sizeof(c43_1),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c43_1) ; i++ )
    {
        i2c_CheckSum += c43_1[i];
    }
    printk("Himax i2c_checksum_43_1_size = %d \n",sizeof(c43_1));
    printk("Himax i2c_checksum_43_1 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x34D;
    printk("Himax i2c_checksum_43_1 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c43_1 END

    //------------------------------------------------------------------config bank PART c43_2 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0xBD;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c43_2[0],sizeof(c43_2),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c43_2) ; i++ )
    {
        i2c_CheckSum += c43_2[i];
    }
    printk("Himax i2c_checksum_43_2_size = %d \n",sizeof(c43_2));
    printk("Himax i2c_checksum_43_2 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x36C;
    printk("Himax i2c_checksum_43_2 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c43_2 END

    //------------------------------------------------------------------config bank PART c44_1 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0xDA;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c44_1[0],sizeof(c44_1),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c44_1) ; i++ )
    {
        i2c_CheckSum += c44_1[i];
    }
    printk("Himax i2c_checksum_44_1_size = %d \n",sizeof(c44_1));
    printk("Himax i2c_checksum_44_1 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x389;
    printk("Himax i2c_checksum_44_1 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c44_1 END

    //------------------------------------------------------------------config bank PART c44_2 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0xF9;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c44_2[0],sizeof(c44_2),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c44_2) ; i++ )
    {
        i2c_CheckSum += c44_2[i];
    }
    printk("Himax i2c_checksum_44_2_size = %d \n",sizeof(c44_2));
    printk("Himax i2c_checksum_44_2 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x3A8;
    printk("Himax i2c_checksum_44_2 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c44_2 END

    //------------------------------------------------------------------config bank PART c45 START
    data[0] = 0xE1;
    data[1] = 0x15;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xD8;
    data[1] = 0x00;
    data[2] = 0xFE;	//Start addr
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    if((i2c_himax_master_write(touch_i2c, &c45[0],sizeof(c45),3))<0)
    {
        goto HimaxErr;
    }

    data[0] = 0xE1;
    data[1] = 0x00;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    for( i=0 ; i<sizeof(c45) ; i++ )
    {
        i2c_CheckSum += c45[i];
    }
    printk("Himax i2c_checksum_45_size = %d \n",sizeof(c45));
    printk("Himax i2c_checksum_45 = %d \n",i2c_CheckSum);
    i2c_CheckSum += 0x3AD;
    printk("Himax i2c_checksum_45 = %d \n",i2c_CheckSum);
    //------------------------------------------------------------------config bank PART c45 END

    data[0] = 0xAB;
    data[1] = 0x10;
    if((i2c_himax_master_write(touch_i2c, &data[0],2,3))<0)
    {
        goto HimaxErr;
    }

    i2c_CheckSum += 0xAB;
    i2c_CheckSum += 0x10;

    printk("Himax i2c_checksum_Final = %d \n",i2c_CheckSum);

    data[0] = 0xAC;
    data[1] = i2c_CheckSum & 0xFF;
    data[2] = (i2c_CheckSum >> 8) & 0xFF;
    if((i2c_himax_master_write(touch_i2c, &data[0],3,3))<0)
    {
        goto HimaxErr;
    }

    printk("Himax i2c_checksum_AC = %d , %d \n",data[1],data[2]);

    int ret = i2c_himax_read(touch_i2c, 0xAB, buf0, 2, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
        goto HimaxErr;
    }

    if(buf0[0] == 0x18)
    {
        return -1;
    }

    if(buf0[0] == 0x10)
    {
        return 1;
    }

    return 1;

HimaxErr:
    return -1;
}
#endif
static int himax_ts_poweron(struct himax_ts_data *ts_modify)
{
    uint8_t buf0[11];
    int ret = 0;
    int config_fail_retry = 0;

    //Wakelock Protect Start
    wake_lock(&ts_modify->wake_lock);
    //Wakelock Protect End

    //Mutexlock Protect Start
    mutex_lock(&ts_modify->mutex_lock);
    //Mutexlock Protect End

#ifdef HX_85XX_C_SERIES_PWON
    buf0[0] = HX_CMD_MANUALMODE;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETMICROOFF;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETROMRDY;
    buf0[1] = 0x0F;
    buf0[2] = 0x53;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SET_CACHE_FUN;
    buf0[1] = 0x05;
    buf0[2] = 0x03;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_B9;
    buf0[1] = 0x01;
    buf0[2] = 0x2D;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = 0xE3;
    buf0[1] = 0x00;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_TSSON;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

    buf0[0] = HX_CMD_TSSLPOUT;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

#else
#ifdef HX_85XX_A_SERIES_PWON

    buf0[0] = HX_CMD_MANUALMODE;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETMICROOFF;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETROMRDY;
    buf0[1] = 0x0F;
    buf0[2] = 0x53;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SET_CACHE_FUN;
    buf0[1] = 0x06;
    buf0[2] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_76;
    buf0[1] = 0x01;
    buf0[2] = 0x2D;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_TSSON;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

    buf0[0] = HX_CMD_TSSLPOUT;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

#else
#ifdef HX_85XX_B_SERIES_PWON
    buf0[0] = HX_CMD_MANUALMODE;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETMICROOFF;
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETROMRDY;
    buf0[1] = 0x0F;
    buf0[2] = 0x53;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SET_CACHE_FUN;
    buf0[1] = 0x06;
    buf0[2] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_76;
    buf0[1] = 0x01;
    buf0[2] = 0x2D;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_TSSON;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

    buf0[0] = HX_CMD_TSSLPOUT;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

#else
#ifdef HX_85XX_D_SERIES_PWON

    buf0[0] = HX_CMD_MANUALMODE;	//0x42
    buf0[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SETROMRDY;	//0x36
    buf0[1] = 0x0F;
    buf0[2] = 0x53;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_SET_CACHE_FUN;	//0xDD
    buf0[1] = 0x05;
    buf0[2] = 0x03;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_B9;	//setCVDD
    buf0[1] = 0x01;
    buf0[2] = 0x36;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

    buf0[0] = HX_CMD_CB;
    buf0[1] = 0x01;
    buf0[2] = 0xF5;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    udelay(100);

#ifdef HX_LOADIN_CONFIG
    //load config
    printk("Himax start load config.\n");
    config_fail_retry = 0;
    while(true)
    {
        if(himax_config_flow() == -1)
        {
            config_fail_retry++;
            if(config_fail_retry == 3)
            {
                printk("himax_config_flow retry fail.\n");
                goto send_i2c_msg_fail;
            }
            printk("Himax config retry = %d \n",config_fail_retry);
        }
        else
        {
            break;
        }
    }
    printk("Himax end load config.\n");
    msleep(100); //100ms
#endif

    buf0[0] = HX_CMD_TSSON;
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms

    buf0[0] = HX_CMD_TSSLPOUT;	//0x81
    ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(120); //120ms
    //------------------------------------------

#endif
#endif
#endif
#endif

    //Mutexlock Protect Start
    mutex_unlock(&ts_modify->mutex_lock);
    //Mutexlock Protect End

    //Wakelock Protect Start
    wake_unlock(&ts_modify->wake_lock);
    //Wakelock Protect End

    return ret;

    //ESD Workaround Start
send_i2c_msg_fail:
    printk(KERN_ERR "[Himax]:send_i2c_msg_failline: %d \n",__LINE__);
    //todo mutex_unlock(&himax_chip->mutex_lock);

    //Mutexlock Protect Start
    mutex_unlock(&ts_modify->mutex_lock);
    //Mutexlock Protect End

    //Wakelock Protect Start
    wake_unlock(&ts_modify->wake_lock);
    //Wakelock Protect End

#ifdef ENABLE_CHIP_RESET_MACHINE
    if(ts_modify->init_success)
    {
        queue_delayed_work(ts_modify->himax_wq, &ts_modify->himax_chip_reset_work, 0);
    }
#endif
    return -1;
    //ESD Workaround End
}

static int himax_i2c_test_function(struct himax_ts_data *ts_modify)
{
    uint8_t buf0[5];
    int ret = 0;

    buf0[0] = 0xE9;
    buf0[1] = 0x01;
    buf0[2] = 0x01;

    while(1)
    {
        ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//sleep out
        if(ret < 0)
        {
            printk(KERN_ERR "*****HIMAX TEST: i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
        }
        else
        {
            printk(KERN_ERR "*****HIMAX TEST: OK addr = 0x%x\n",ts_modify->client->addr);
        }
        mdelay(200);
    }
    return ret;
}

int himax_touch_cable_status(int status)
{
    //reject callback before probing successfully
    if(private_ts == NULL || private_ts->init_success != 1 || entry_mode != 1)
    {
        return 0;
    }

    uint8_t buf0[2] = {0};
    int ret = 0;

    printk("[Himax] %s: cable_status=%d, init_success=%d.\n", __func__, status,private_ts->init_success);

    if(status == 0x02) //non usb
    {
        buf0[0] = 0x90;
        buf0[1] = 0x00;
        ret = i2c_himax_master_write(touch_i2c, buf0, 2, DEFAULT_RETRY_CNT);
    }
    else if((status == 0x00) || (status == 0x01)) //usb plug in
    {
        buf0[0] = 0x90;
        buf0[1] = 0x01;
        ret = i2c_himax_master_write(touch_i2c, buf0, 2, DEFAULT_RETRY_CNT);
    }

    return 0;
}
EXPORT_SYMBOL(himax_touch_cable_status);

static ssize_t himax_touch_switch_name(struct switch_dev *sdev, char *buf)
{
    himax_read_FW_ver();
    int count = 0;
    count += sprintf(buf + count, "Himax:MAJ");
    int i;
    for(i=0; i<FW_VER_MAJ_FLASH_LENG; i++)
    {
        count += sprintf(buf + count, "-0x%2.2X",FW_VER_MAJ_buff[i]);
    }
    count += sprintf(buf + count, ":MIN");
    for(i=0; i<FW_VER_MIN_FLASH_LENG; i++)
    {
        count += sprintf(buf + count, "-0x%2.2X",FW_VER_MIN_buff[i]);
    }
    /*count += sprintf(buf + count, ":CFG MAJ-");
    for(i=0; i < CFG_VER_MAJ_FLASH_LENG; i++)
    {
        count += sprintf(buf + count,  "0x%2.2X",CFG_VER_MAJ_buff[i]);
        if(i < CFG_VER_MAJ_FLASH_LENG-1)
            count += sprintf(buf + count,  " ");
    }*/
    count += sprintf(buf + count, ":CFG");
    count += sprintf(buf + count,  "-0x%2.2X",CFG_VER_MIN_buff[CFG_VER_MIN_FLASH_LENG-1]);
    count += sprintf(buf + count, ":TP-%d\n",TP_ID);
    return count;
}
#ifdef HX_LOADIN_CONFIG
void tp_config_setup()
{
    TP_ID = Read_TP_ID();
    switch (TP_ID)
    {
        printk("Himax:TP id:%d\n",TP_ID);
    case TP_WINTEK_OGS_65:
        //Bizzy added for 65ohm
        c30[1] = 0x19;	//VR1_M
        c39[16] = 0x19;	//DeThreshold
        c39[21] = 0x0F;	//Water Dethreshold

        c39[19] = 0xAA;	//WET
        c39[24] = 0x40;	//Water Threshold
        c39[14] = 0x87;	//IIR Comp.

        c10[10] = 0x22;	//Bank
        c11[9] = 0x22;	//Bank
        c11[10] = 0x22;	//Bank

        c36_edge[9] = 0x73; //LGD IIR
        c36_level[9] = 0x73; //LGD IIR

        //c37[11] = 0x83; //Self LRG Channel

    case TP_WINTEK_OGS_100:
        /*c7[9]=0x22;
        c7[10]=0x22;
        c8[9]=0x22;
        c40[17] = 0x03;*///invert x/y

        //Bizzy added for wintek singal 20130730
        c29[5] = 0x17;
        c30[3] = 0x18;

        //Bizzy added for wintek reverse
        c12[9] = 0x22;
        c12[10] = 0x22;
        c13[9] = 0x22;
        c40[17] = 0x01;

        c43_1[1] = 0x00;
        c43_1[2] = 0xFF;
        c43_1[3] = 0x24;
        c43_1[4] = 0x11;
        c43_1[5] = 0x01;
        c43_1[6] = 0xFF;
        c43_1[7] = 0x23;
        c43_1[8] = 0x10;
        c43_1[9] = 0x02;
        c43_1[10] = 0xFF;
        c43_1[11] = 0x1E;
        c43_1[12] = 0x0F;
        c43_1[13] = 0x03;
        c43_1[14] = 0xFF;
        c43_1[15] = 0x1D;
        c43_1[16] = 0xFF;
        c43_1[17] = 0x04;
        c43_1[18] = 0xFF;
        c43_1[19] = 0x1C;
        c43_1[20] = 0xFF;
        c43_1[21] = 0x05;
        c43_1[22] = 0x2A;
        c43_1[23] = 0x1B;
        c43_1[24] = 0xFF;
        c43_1[25] = 0x06;
        c43_1[26] = 0x29;
        c43_1[27] = 0x1A;
        c43_1[28] = 0xFF;
        c43_1[29] = 0x07;
        c43_1[30] = 0x28;
        c43_1[31] = 0x19; //start 0x9E,size 32

        c43_2[1] = 0xFF;
        c43_2[2] = 0x08;
        c43_2[3] = 0x27;
        c43_2[4] = 0x18;
        c43_2[5] = 0xFF;
        c43_2[6] = 0x09;
        c43_2[7] = 0x26;
        c43_2[8] = 0x17;
        c43_2[9] = 0xFF;
        c43_2[10] = 0x0A;
        c43_2[11] = 0x25;
        c43_2[12] = 0x16;
        c43_2[13] = 0xFF;
        c43_2[14] = 0x0B;
        c43_2[15] = 0x22;
        c43_2[16] = 0x15;
        c43_2[17] = 0xFF;
        c43_2[18] = 0x0C;
        c43_2[19] = 0x21;
        c43_2[20] = 0x14;
        c43_2[21] = 0xFF;
        c43_2[22] = 0x0D;
        c43_2[23] = 0x20;
        c43_2[24] = 0x13;
        c43_2[25] = 0xFF;
        c43_2[26] = 0x0E;
        c43_2[27] = 0x1F;
        c43_2[28] = 0x12;
        c43_2[29] = 0xFF; //start 0xBD,size 29

        c31[0] = 0xC9;
        c31[1] = 0x00;
        c31[2] = 0x00;
        c31[3] = 0x00;
        c31[4] = 0x30;
        c31[5] = 0x2E;
        c31[6] = 0x2C;
        c31[7] = 0x2A;
        c31[8] = 0x28;
        c31[9] = 0x26;
        c31[10] = 0x24;
        c31[11] = 0x22;
        c31[12] = 0x1E;
        c31[13] = 0x1C;
        c31[14] = 0x20;
        c31[15] = 0x1A;
        c31[16] = 0x18;
        c31[17] = 0x16;
        c31[18] = 0x2F;
        c31[19] = 0x2D;
        c31[20] = 0x2B;
        c31[21] = 0x29;
        c31[22] = 0x27;
        c31[23] = 0x25;
        c31[24] = 0x23;
        c31[25] = 0x21;
        c31[26] = 0x1D;
        c31[27] = 0x1B;
        c31[28] = 0x1F;
        c31[29] = 0x19;
        c31[30] = 0x17;
        c31[31] = 0x15;
        c31[32] = 0x00;
        c31[33] = 0x00;
        c31[34] = 0x00;

        if(TP_ID == TP_WINTEK_OGS_65)
            break;

        c39[14] = 0xD9;	//IIR Comp.
        c36_edge[9] = 0x64; //LGD IIR
        c36_level[9] = 0x64; //LGD IIR

        break;
        /*case TP_TRULY_OGS:*/
    }
}
#endif

static int cable_status_notify(struct notifier_block *self, unsigned long action, void *dev)
{
    if (isTouchSuspend)
    {
        printk(KERN_INFO "Touch is suspend but USB still notify !!!\n", __func__);
        wake_lock(&wakelock_detect_cable);
        isUSBSuspendNotify = true;
        return NOTIFY_OK;
    }
    switch (action)
    {
    case CHRG_UNKNOWN:
        printk(KERN_INFO "%s CHRG_UNKNOWN !!!\n", __func__);
        himax_touch_cable_status(2);
        break;
    case CHRG_SDP:
        printk(KERN_INFO "%s CHRG_SDP !!!\n", __func__);
        himax_touch_cable_status(0);
        break;
        /*case CHRG_CDP:*/
    case CHRG_DCP:
        printk(KERN_INFO "%s CHRG_DCP !!!\n", __func__);
        himax_touch_cable_status(0);
        break;
        /*case CHRG_ACA:
        case CHRG_SE1:
        case CHRG_MHL:*/
    }
    return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier =
{
    .notifier_call = cable_status_notify,
};

//extern int cable_status_register_client(struct notifier_block *nb); KK porting
//extern int cable_status_unregister_client(struct notifier_block *nb); KK porting

void loadingTouchFW_with_header_file()
{
    switch(HW_ID)
    {
    case PROJ_FE170CG_HWID_SR:
        break;
    case PROJ_FE170CG_HWID_ER1:
    case PROJ_FE170CG_HWID_ER2:
        if(TP_ID == TP_WINTEK )
        {
            i_CTPM_FW = i_CTPM_FW_Wintek_ER;
        }
        else
        {
            i_CTPM_FW = i_CTPM_FW_Ofilm_ER;
        }
        break;
    case PROJ_FE170CG_HWID_MP:
        if(TP_ID == TP_WINTEK )
        {
            i_CTPM_FW = i_CTPM_FW_Wintek_MP;
        }
        else
        {
            i_CTPM_FW = i_CTPM_FW_Ofilm_MP;
        }
        break;
    }
}

static int himax_ts_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    printk("[himax] kernel boot mode:%d\n",entry_mode);
    //only do probe in the MOS mode
    if(entry_mode != 1)
    {
        return -1;
    }

    int err = 0, i;
    struct himax_i2c_platform_data *pdata;

#ifdef HX_LOADIN_CONFIG
    tp_config_setup();
#endif

    printk("Himax test i2c functionality\n");
    // check i2c capability
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "[himax] %s: i2c check functionality error\n", __func__);
        err = -ENODEV;
        goto err_check_functionality_failed;
    }

    printk("Himax test i2c funtionality ok\n");

    private_ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
    if (private_ts == NULL)
    {
        printk(KERN_ERR "[himax] %s: allocate himax_ts_data failed\n", __func__);
        err = -ENOMEM;
        goto err_alloc_data_failed;
    }
    private_ts->init_success = 0;

    printk("Himax debug ts_data allocate ok\n");

#ifdef  HX_TP_SYS_FLASH_DUMP
    private_ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
    if (!private_ts->flash_wq)
    {
        printk(KERN_ERR "[himax] %s: create flash workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_wq_failed;
    }
#endif

    private_ts->himax_wq = create_singlethread_workqueue("himax_wq");
    if (!private_ts->himax_wq)
    {
        printk(KERN_ERR "[himax] %s: create workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_wq_failed;
    }

    printk("Himax work queue create ok\n");

    //ESD Workaround Start
#ifdef ENABLE_CHIP_RESET_MACHINE
    INIT_DELAYED_WORK(&private_ts->himax_chip_reset_work, himax_chip_reset_function);
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
    INIT_DELAYED_WORK(&private_ts->himax_chip_monitor, himax_chip_monitor_function); //for ESD solution
#endif
    //ESD Workaround End

    printk("Himax ESD ok\n");
#ifdef HX_TP_SYS_FLASH_DUMP
    INIT_WORK(&private_ts->flash_work, himax_ts_flash_work_func);
#endif
    printk("Himax debug flash dump ok\n");

    //register interrupt service routine
    INIT_WORK(&private_ts->work, himax_ts_work_func);
    INIT_WORK(&private_ts->handshaking_work, touch_callback_himax_hang_shaking);
    private_ts->client = client;
    i2c_set_clientdata(client, private_ts);
    pdata = client->dev.platform_data;

    client->irq = gpio_to_irq(pdata->intr_gpio);

    printk("Himax debug init work ok\n");

    //ESD Workaround Start
    private_ts->init_success = 0;
    private_ts->retry_time = 0;

#ifdef ENABLE_CHIP_STATUS_MONITOR
    private_ts->running_status = 0;
#endif
    //ESD Workaround End

    // register GPIO pin for interrupt and reset
    /* This depend on the hardware design.
     * If there is no GPIO pin, this can be ignored.
     */
    if (likely(pdata != NULL))
    {
        private_ts->intr_gpio = pdata->intr_gpio;
        private_ts->rst_gpio = pdata->rst_gpio;
        private_ts->tp_pwr_gpio = pdata->tp_pwr_gpio;
    }

#ifdef ESD_WORKAROUND
    reset_activate = 1;
#endif

    gpio_request(private_ts->rst_gpio,"TS_RST_N");
    gpio_request(private_ts->intr_gpio,"TS_INT_N");
    gpio_direction_input(private_ts->intr_gpio);
    gpio_request(private_ts->tp_pwr_gpio,"TP_PWR_EN");
    printk("Himax ts_rst = %d, ts_int = %d, tp_pwr_gpio = %d\n",private_ts->rst_gpio,private_ts->intr_gpio,private_ts->tp_pwr_gpio);
    printk("Himax debug power gpio 1\n");
    gpio_direction_output(private_ts->tp_pwr_gpio, 1);
    printk("Himax debug power gpio 1 .ok\n");
    msleep(50);

    gpio_set_value(private_ts->rst_gpio, 1);

    sema_init(&pSem, 1);

    private_ts->status = 1; // set I2C status is OK;

    //Mutexlock Protect Start
    mutex_init(&private_ts->mutex_lock);
    //Mutexlock Protect End

    //Wakelock Protect Start
    wake_lock_init(&private_ts->wake_lock, WAKE_LOCK_SUSPEND, "himax_touch_wake_lock");
    //Wakelock Protect End

    printk("Himax debug allocate input device\n");

    private_ts->input_dev = input_allocate_device();

    if (private_ts->input_dev == NULL)
    {
        err = -ENOMEM;
        dev_err(&client->dev, "[himax] Failed to allocate input device\n");
        goto err_input_dev_alloc_failed;
    }

    // initiallize the touch panel information of the android system
    /*
     * This includes the maximum x-y axix and the report information such as
     * area, pressure and x-y point.
     *
     */
    private_ts->input_dev->name = "himax-touchscreen";  ////
    private_ts->abs_x_max = pdata->abs_x_max;
    private_ts->abs_y_max = pdata->abs_y_max;
    dev_info(&client->dev, "[himax] Max X=%d, Max Y=%d\n", private_ts->abs_x_max, private_ts->abs_y_max);

#ifdef HX_EN_BUTTON
    //initial KEY setting
    for(i=0; i<HX_KEY_COUNT; i++)
    {
        set_bit(tpd_keys_local[i], private_ts->input_dev->keybit);
    }
#endif

    __set_bit(EV_KEY, private_ts->input_dev->evbit);
    __set_bit(EV_ABS, private_ts->input_dev->evbit);
    __set_bit(BTN_TOUCH, private_ts->input_dev->keybit);

    input_set_abs_params(private_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
    input_set_abs_params(private_ts->input_dev, ABS_MT_POSITION_X, 0, private_ts->abs_x_max, 0, 0);
    input_set_abs_params(private_ts->input_dev, ABS_MT_POSITION_Y, 0, private_ts->abs_y_max, 0, 0);
    input_set_abs_params(private_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0); //Finger Size
    input_set_abs_params(private_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 31, 0, 0); //Touch Size
    input_set_abs_params(private_ts->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);

    err = input_register_device(private_ts->input_dev);
    if (err)
    {
        dev_err(&client->dev,"[himax]%s: unable to register %s input device\n",__func__, private_ts->input_dev->name);
        goto err_input_register_device_failed;
    }

    touch_i2c = client;

    //Himax I2C path check function
#ifdef HX_CHECK_I2C_PATH
    himax_i2c_test_function(private_ts);
#endif

    printk("himax probe step1 \n");

    printk("himax probe step2 \n");

#ifdef HX_TP_SYS_FS
    // Himax: register sysfs node for reading raw data
    // initiallize the x and y channel number
    setXChannel(HX_TP_X_CH_NUM); // X channel
    setYChannel(HX_TP_Y_CH_NUM); // Y channel

    printk("himax probe step3 \n");

#ifdef HX_TP_SYS_FLASH_DUMP
    setSysOperation(0);
    setFlashBuffer();
#endif

    printk("himax probe step4 \n");

    // allocate buffer to receive the raw data
    setMutualBuffer();
    if (getMutualBuffer() == NULL)
    {
        printk(KERN_ERR "[TP] %s: mutual buffer allocate fail failed\n", __func__);
        return -1;
    }
    printk("himax probe step5 \n");

    //Shouchung remove, for ATD function
    // register sysfs node for debug APK such as raw-count and fw-upgrade
    //himax_touch_sysfs_init();
    //Shouchung end

    printk("himax probe step6 \n");

    //ts->attrs.attrs = himax_attr;
    //err = sysfs_create_group(&client->dev.kobj, &ts->attrs);
    //if (err) {
    //    dev_err(&client->dev, "[TP] %s: Not able to create the sysfs\n", __func__);
    //}
#endif

    printk("himax probe step7 \n");

    // register the style of interrupt triger
    himax_ts_register_interrupt(private_ts->client);

    printk("himax probe step8 \n");

#ifdef CONFIG_HAS_EARLYSUSPEND
    private_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    private_ts->early_suspend.suspend = himax_ts_early_suspend;
    private_ts->early_suspend.resume = himax_ts_late_resume;
    register_early_suspend(&private_ts->early_suspend);
#endif

    printk("himax probe step9 \n");

    /* Register Switch file */
    private_ts->touch_sdev.name = "touch";
    private_ts->touch_sdev.print_name = himax_touch_switch_name;
    if(switch_dev_register(&private_ts->touch_sdev) < 0)
    {
        dev_info(&client->dev, "switch_dev_register for dock failed!\n");
    }
    switch_set_state(&private_ts->touch_sdev, 0);


    //Shouchung add ATD tool function
#ifdef HX_TP_SYS_FS
    // register sysfs node for debug APK such as raw-count and fw-upgrade
    himax_touch_sysfs_init();

    private_ts->attrs.attrs = himax_attr;
    err = sysfs_create_group(&client->dev.kobj, &private_ts->attrs);
    if (err)
    {
        dev_err(&client->dev, "[TP] %s: Not able to create the sysfs\n", __func__);
    }
#endif
    //Shouchung end

    printk("himax probe step10 \n");

    //ESD Workaround Start
    private_ts->init_success = 1;
    private_ts->retry_time = 0;

#ifdef ENABLE_CHIP_STATUS_MONITOR
    queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 60*HZ);   //for ESD solution
#endif

    printk("himax probe step11 \n");

    // Himax: touch screen power on sequence
    /*
     * Please follow the power sequence of each touch IC
     */

    printk("Himax debug Start do power on \n");

    i_Check_FW_Version();
#ifdef HX_RST_PIN_FUNC
    himax_HW_reset();
#endif
    msleep(50);

    TP_ID = Read_TP_ID();
    HW_ID = Read_HW_ID();

    loadingTouchFW_with_header_file();

    if(i_Needupdate && HW_ID != PROJ_FE170CG_HWID_SR)
    {
        i_update_func();
    }

#ifdef HX_RST_PIN_FUNC
    himax_HW_reset();
#endif
    msleep(50);
    himax_ts_poweron(private_ts);

    if (gpio_get_value(private_ts->intr_gpio) == 0)
    {
        printk(KERN_INFO "[himax]%s: handle missed interrupt\n", __func__);
        himax_ts_irq_handler(client->irq, private_ts);
    }
    /*else
    {
        enable_irq(client->irq);
    }*/

    //ESD Workaround End

    dev_info(&client->dev, "[himax] Start touchscreen %s in interrupt mode\n",private_ts->input_dev->name);

    //cable_status_register_client(&cable_status_notifier); KK porting
    wake_lock_init(&wakelock_detect_cable, WAKE_LOCK_SUSPEND, "cable_wakelock");

    /*if(i_Needupdate)
    {
        request_firmware_nowait(THIS_MODULE,
                                FW_ACTION_HOTPLUG,
                                "touch/himax/fe170cg_fw.bin",
                                &private_ts->client->dev,
                                GFP_KERNEL,
                                NULL,
                                i_update_func);
    }*/
    return 0;

err_input_register_device_failed:
    if (private_ts->input_dev)
    {
        input_free_device(private_ts->input_dev);
    }
err_input_dev_alloc_failed:

//Mutexlock Protect Start
    mutex_destroy(&private_ts->mutex_lock);
//Mutexlock Protect End

//Wakelock Protect Start
    wake_lock_destroy(&private_ts->wake_lock);
//Wakelock Protect End

//ESD Workaround Start
#ifdef ENABLE_CHIP_RESET_MACHINE
    cancel_delayed_work(&private_ts->himax_chip_reset_work);
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
    cancel_delayed_work(&private_ts->himax_chip_monitor);
#endif
//ESD Workaround End

    if (private_ts->himax_wq)
    {
        destroy_workqueue(private_ts->himax_wq);
    }

err_create_wq_failed:
    kfree(private_ts);

err_alloc_data_failed:
err_check_functionality_failed:
    printk("himax probe err_check_functionality_failed \n");
//ESD Workaround Start
    private_ts->init_success = 0;
//ESD Workaround End
    return err;
}

static int himax_ts_remove(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);

#ifdef HX_TP_SYS_FS
    himax_touch_sysfs_deinit();
#endif

    unregister_early_suspend(&ts->early_suspend);
    free_irq(client->irq, ts);

    //Mutexlock Protect Start
    mutex_destroy(&ts->mutex_lock);
    //Mutexlock Protect End

    if (ts->himax_wq)
        destroy_workqueue(ts->himax_wq);
    input_unregister_device(ts->input_dev);

    //Wakelock Protect Start
    wake_lock_destroy(&ts->wake_lock);
    //Wakelock Protect End

    kfree(ts);

    //cable_status_unregister_client(&cable_status_notifier); KK porting

    return 0;
}

static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{

#ifdef TPD_PROXIMITY
    if(tpd_proximity_detect == 0 && tpd_proximity_flag == 1)
    {
        //Wakelock Protect Start
        wake_lock(&private_ts->wake_lock);
        //Wakelock Protect End
        return 0;
    }
#endif

    struct timeval start, end;
    do_gettimeofday(&start);
    struct himax_ts_data *ts_modify = i2c_get_clientdata(client);
    uint8_t buf[2] = {0};
    int ret = 0;

    printk(KERN_INFO "[himax] %s: TS suspend\n", __func__);

    isTouchSuspend = true;
#ifdef HX_TP_SYS_HITOUCH
    if(hitouch_is_connect)
    {
        printk(KERN_INFO "[himax] %s: Hitouch connect, reject suspend\n",__func__);
        return 0;
    }
#endif

    //Wakelock Protect Start
    wake_lock(&ts_modify->wake_lock);
    //Wakelock Protect End

    //Mutexlock Protect Start
    mutex_lock(&ts_modify->mutex_lock);
    //Mutexlock Protect End

    buf[0] = HX_CMD_TSSOFF;
    ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
    }
    msleep(120);

    buf[0] = HX_CMD_TSSLPIN;
    ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
    }
    msleep(120);

    buf[0] = HX_CMD_SETDEEPSTB;
    buf[1] = 0x01;
    ret = i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
    }
    msleep(120);

    //Mutexlock Protect Start
    mutex_unlock(&ts_modify->mutex_lock);
    //Mutexlock Protect End

    //Wakelock Protect Start
    wake_unlock(&ts_modify->wake_lock);
    //Wakelock Protect End

    //ESD Workaround Start
#ifdef ENABLE_CHIP_STATUS_MONITOR
    ts_modify->running_status = 1;
    cancel_delayed_work_sync(&ts_modify->himax_chip_monitor);
#endif
    //ESD Workaround End

    disable_irq(client->irq);

    ret = cancel_work_sync(&ts_modify->work);
    if (ret)
        enable_irq(client->irq);
    if (hx_point_num != 0 && tpd_key == 0xFF)
    {
        input_report_key(private_ts->input_dev, BTN_TOUCH, 0);  // touch up
        input_mt_sync(private_ts->input_dev);
        input_sync(private_ts->input_dev);
        printk(KERN_INFO "[himax] %s: set touch up!\n", __func__);
    }
    do_gettimeofday(&end);
    printk("PM-Touch-Suspend : spent %ld msecs\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));
    return 0;
}

static int himax_ts_resume(struct i2c_client *client)
{

#ifdef TPD_PROXIMITY
    if(tpd_proximity_flag == 1 && isTouchSuspend == false)
    {
        //Wakelock Protect Start
        wake_unlock(&private_ts->wake_lock);
        //Wakelock Protect End
        return 0;
    }
#endif

    struct timeval start, end;
    struct himax_ts_data *ts_modify = i2c_get_clientdata(touch_i2c);
    uint8_t buf[3] = {0};
    int ret = 0;
    //Wakelock Protect Start
    wake_lock(&ts_modify->wake_lock);
    //Wakelock Protect End
    do_gettimeofday(&start);

    printk(KERN_INFO "[himax] %s: TS resume\n", __func__);

    buf[0] = 0xF6;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    ret = i2c_himax_master_write(ts_modify->client, buf, 4, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_SETDEEPSTB;	//0xD7
    buf[1] = 0x00;
    ret = i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_MANUALMODE;	//0x42
    buf[1] = 0x02;
    ret = i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT);//Reload Disable
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_SETROMRDY;	//0x36
    buf[1] = 0x0F;
    buf[2] = 0x53;
    ret = i2c_himax_master_write(ts_modify->client, buf, 3, DEFAULT_RETRY_CNT);//enable flash
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_SET_CACHE_FUN;	//0xDD
    buf[1] = 0x05;
    buf[2] = 0x03;
    ret = i2c_himax_master_write(ts_modify->client, buf, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_B9;	//0xB9
    buf[1] = 0x01;
    buf[2] = 0x36;
    ret = i2c_himax_master_write(ts_modify->client, buf, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_CB;	//0xCB
    buf[1] = 0x01;
    buf[2] = 0xF5;
    ret = i2c_himax_master_write(ts_modify->client, buf, 3, DEFAULT_RETRY_CNT);
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(5);

    buf[0] = HX_CMD_TSSON;	//0x83
    ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }
    msleep(50);

    buf[0] = HX_CMD_TSSLPOUT;	//0x81
    ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);//sense on
    if(ret < 0)
    {
        printk(KERN_ERR "[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
        goto send_i2c_msg_fail;
    }

    //himax_ts_poweron(ts_modify);

    //ESD Workaround Start

    msleep(50);

#ifdef ESD_WORKAROUND_RESUME
    msleep(150);
    ret = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail
    if(ret == 2)
    {
        printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
#ifdef ENABLE_CHIP_RESET_MACHINE
        himax_chip_reset_function(NULL);
#endif
    }
    if(ret == 1)
    {
        printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
#ifdef ENABLE_CHIP_RESET_MACHINE
        himax_chip_reset_function(NULL);
#endif
        //Do HW_RESET??
        //#ifdef ESD_WORKAROUND
        //ESD_HW_REST();
        //#endif
    }
    else
    {
        printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__);
    }
    //ESD Workaround End
#endif

    enable_irq(touch_i2c->irq);

    //ESD Workaround Start
#ifdef ENABLE_CHIP_STATUS_MONITOR
    queue_delayed_work(ts_modify->himax_wq, &ts_modify->himax_chip_monitor, 10*HZ); //for ESD solution
#endif

#ifdef ESD_WORKAROUND
    ESD_COUNTER = 0;
#endif
    //ESD Workaround End

    isTouchSuspend = false;
    if (isUSBSuspendNotify)
    {
        isUSBSuspendNotify = false;
        //cable_status_notify( NULL, query_cable_status(), &ts_modify->client->dev); KK porting
        wake_unlock(&wakelock_detect_cable);
    }
    do_gettimeofday(&end);
    printk("PM-Touch-Resume : spent %ld msecs\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));
    //Wakelock Protect Start
    wake_unlock(&ts_modify->wake_lock);
    //Wakelock Protect End

    return;
send_i2c_msg_fail:
    himax_chip_reset_function(NULL);

    //Wakelock Protect Start
    wake_unlock(&ts_modify->wake_lock);
    //Wakelock Protect End

    return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
    struct himax_ts_data *ts;
    ts = container_of(h, struct himax_ts_data, early_suspend);
    himax_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
    struct himax_ts_data *ts;
    ts = container_of(h, struct himax_ts_data, early_suspend);
    himax_ts_resume(ts->client);
}
#endif

void himax_touch_shutdown(struct i2c_client *client)
{
    printk("[himax] touch shutdown\n");
    if(private_ts == NULL || private_ts->init_success != 1 || entry_mode != 1)
    {
        return;
    }
    gpio_set_value(private_ts->rst_gpio, 0);
}

static const struct i2c_device_id himax_ts_id[] =
{
    { HIMAX_TS_NAME, 0 },
    { }
};

static struct i2c_driver himax_ts_driver =
{
    .probe		= himax_ts_probe,
    .remove		= himax_ts_remove,
    .shutdown           = himax_touch_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend	= himax_ts_suspend,
    .resume		= himax_ts_resume,
#endif
    .id_table    = himax_ts_id,
    .driver        = {
        .name = HIMAX_TS_NAME,
    },
};

static int himax_ts_init(void)
{
    printk(KERN_INFO "[himax_] %s\n", __func__);
    return i2c_add_driver(&himax_ts_driver);
}

static void __exit himax_ts_exit(void)
{
    i2c_del_driver(&himax_ts_driver);
    return;
}
/******************** Himax: Fundamental function ********************/

module_init(himax_ts_init);
module_exit(himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");


