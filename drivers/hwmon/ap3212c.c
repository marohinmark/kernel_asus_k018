#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
//#include <asm/mach-types.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>
#include <asm/intel-mid.h>//for get gpio by name

/** i2c cmd **/
#define CMD_SYS_CONFIG 0x00
#define CMD_INT_STATUS 0x01
#define CMD_INT_CLEAR 0x02
#define CMD_IR_DATA_LOW 0x0A
#define CMD_IR_DATA_HIGH 0x0B
#define CMD_ALS_DATA_LOW 0x0C
#define CMD_ALS_DATA_HIGH 0x0D
#define CMD_PS_DATA_LOW 0x0E
#define CMD_PS_DATA_HIGH 0x0F

#define CMD_ALS_CONFIG 0x10
#define CMD_ALS_CAL 0x19 //no use
#define CMD_ALS_LOW_THR_1 0x1A //no use
#define CMD_ALS_LOW_THR_2 0x1B //no use
#define CMD_ALS_HIGH_THR_1 0x1C //no use
#define CMD_ALS_HIGH_THR_2 0x1D //no use

#define CMD_PS_CONFIG 0x20
#define CMD_PS_LED_DRIVE 0x21
#define CMD_PS_INT_FORM 0x22
#define CMD_PS_MEAN_TIME 0x23
#define CMD_PS_LED_WAIT_TIME 0x24
#define CMD_PS_CAL_LOW 0x28
#define CMD_PS_CAL_HIGH 0x29
#define CMD_PS_LOW_THR_1 0x2A
#define CMD_PS_LOW_THR_2 0x2B
#define CMD_PS_HIGH_THR_1 0x2C
#define CMD_PS_HIGH_THR_2 0x2D

/** i2c cmd mask **/
#define CMD_SYS_CONFIG_PS_MASK 0x02
#define CMD_SYS_CONFIG_ALS_MASK 0x01

#define CMD_INT_STATUS_PS_MASK 0x02

#define CMD_PS_DATA_LOW_MASK 0x0F
#define CMD_PS_DATA_HIGH_MASK 0x3F
#define CMD_PS_DATA_LOW_OBJECT_MASK 0x80

#define CMD_IR_DATA_LOW_MASK 0x03
#define CMD_IR_DATA_HIGH_MASK 0xFF
#define CMD_IR_DATA_LOW_OVERFLOW_MASK 0x80

/** i2c cmd some config val **/
#define CMD_SYS_CONFIG_ALL_OFF_VAL 0x00
#define CMD_SYS_CONFIG_ALS_ON_VAL 0x01
#define CMD_SYS_CONFIG_PS_ON_VAL 0x02
#define CMD_SYS_CONFIG_ALL_ON_VAL 0x03

#define CMD_INT_CLEAR_SET_AUTO_VAL 0x00

#define CMD_ALS_CONFIG_DEFAULT_VAL 0x20 //range 4095 lux , 1 conversion time

#define CMD_PS_CONFIG_DEFALUT_VAL 0x41 // 4T , gain 1 , 2 conversion time
#define CMD_PS_LED_DRIVE_DEFAULT_VAL 0x13 // 1 pulse , led ratio 100%
#define CMD_PS_INT_MODE_DEFAULT_VAL 0x01 // hysteresis type
#define CMD_PS_MEAN_TIME_DEFAULT_VAL 0x00 // 12.5ms
#define CMD_PS_LED_WAIT_TIME_DEFAULT_VAL 0x00 // no wait

/** defines **/
#define CMD_SIZE 26

/** variables **/
static u8 ap3212c_cmds[CMD_SIZE] =
    {0x00,0x01,0x02,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
     0x10,0x19,0x1a,0x1b,0x1c,0x1d,
     0x20,0x21,0x22,0x23,0x24,0x28,0x29,0x2a,0x2b,0x2c,0x2d};

static int ap3212c_i2c_retry_count = 3;
static int ap3212c_als_poll_time = 200; // ms , 5hz
static int ap3212c_als_min_poll_time = 200; // ms , 5hz
static int ap3212c_ps_poll_time = 200; // ms , 5hz
static int ap3212c_ps_min_poll_time = 200; // ms , 5hz
struct delayed_work ap3212c_als_report_poll_event_work;
struct delayed_work ap3212c_ps_report_poll_event_work;
static struct workqueue_struct *ap3212c_als_poll_work_queue;
static struct workqueue_struct *ap3212c_ps_poll_work_queue;
static wait_queue_head_t ap3212c_als_poll_wait_queue_head_t;
static wait_queue_head_t ap3212c_ps_poll_wait_queue_head_t;
static bool ap3212c_als_flag_pollin = true;
static bool ap3212c_ps_flag_pollin = true;
static struct miscdevice ap3212c_als_misc_dev;
static struct miscdevice ap3212c_ps_misc_dev;

/** functions **/
static int ap3212c_i2c_read(struct i2c_client *client,uint8_t cmd);
static int ap3212c_i2c_write(struct i2c_client *client,uint8_t cmd,uint8_t data);
static int ap3212c_als_get_adc(struct i2c_client *client);
static int ap3212c_als_get_lux(struct i2c_client *client);
static int ap3212c_als_get_default_lux(struct i2c_client *client);
static int ap3212c_als_on_default_conf(struct i2c_client *client);
static int ap3212c_als_off_default_conf(struct i2c_client *client);
static int ap3212c_als_update_calibration(void);
static int ap3212c_ps_get_ambient_ir(struct i2c_client *client);
static int ap3212c_ps_get_ambient_ir_overflow(struct i2c_client *client);
static int ap3212c_ps_get_adc(struct i2c_client *client);
static int ap3212c_ps_get_object_distance(struct i2c_client *client);
static int ap3212c_ps_get_object_far(struct i2c_client *client);
static int ap3212c_ps_default_config(struct i2c_client *client);
static int ap3212c_ps_cal_config(struct i2c_client *client);
static int ap3212c_ps_cal_clean(struct i2c_client *client);
static int ap3212c_ps_threshold_config(struct i2c_client *client);
static int ap3212c_ps_on_default_conf(struct i2c_client *client);
static int ap3212c_ps_off_default_conf(struct i2c_client *client);
static int ap3212c_ps_update_calibration(void);
static irqreturn_t ap3212c_irq_handle_fn(int irq, void *saved_data);
static int ap3212c_ps_update_threshold_high_calibration(void);
static int ap3212c_ps_update_threshold_low_calibration(void);

/** calibration **/
#define CAL_ALS_PATH "/factory/lightsensor/ALS_Config.ini"
static bool ap3212c_als_calibration_updated_flag = false;
static int ap3212c_als_calibratino_base_lux = 1000;
static int ap3212c_als_calibration_value = 1000;
static int ap3212c_als_calibration_default_value = 1000;

static bool ap3212c_ps_xtalk_calibration_updated_flag = false;
#define CAL_PS_XTALK_PATH "/factory/proximitysensor/PS_Config_Xtalk.ini"
static int ap3212c_ps_xtalk_value = 0;
static int ap3212c_ps_xtalk_default_value = 0;

static bool ap3212c_ps_thr_calibration_updated_flag = false;
#define CAL_PS_THR_H_PATH "/factory/proximitysensor/PS_Config_Threshold_High.ini"
#define CAL_PS_THR_H_SHIFT 0
static int ap3212c_ps_high_threshold_distance = 3;//cm
static int ap3212c_ps_high_threshold = 500;
static int ap3212c_ps_high_threshold_default = 500;

#define CAL_PS_THR_L_PATH "/factory/proximitysensor/PS_Config_Threshold_Low.ini"
#define CAL_PS_THR_L_SHIFT 0
static int ap3212c_ps_low_threshold_distance = 5;//cm
static int ap3212c_ps_low_threshold = 450;
static int ap3212c_ps_low_threshold_default = 450;

#define PS_INIT_POLL_MAX_COUNT 10
static int ap3212c_ps_init_poll_count = 0;
static int ap3212c_ps_irq_data_count = 0;

struct ap3212c_data {
    struct i2c_client *client;
    struct mutex lock;
};

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

//TODO after firmware addr corrected , remove this two read/write function
static int ap3212c_i2c_smbus_read_byte_data(struct i2c_client *client,uint8_t cmd){
    int ret = 0;
    int i;
    int i2c_err = 0;
    int i2c_retry_count = 3;
    uint8_t readData[1];
    uint8_t cmdData[1]={cmd};
    struct i2c_msg msg[] = {
        {
         .addr = 0x1c,
         .flags = 0,
         .len = 1,
         .buf = cmdData,
         },
        {
         .addr = 0x1c,
         .flags = I2C_M_RD,
         .len = 1,
         .buf = readData,
         },
    };
    for (i = 0;i < i2c_retry_count; i++){
        i2c_err = i2c_transfer(client->adapter, msg, 2);
        //printk("ap3212c : %s , i2c transfer ret = %d\n",__func__,i2c_err);
        if(i2c_err >= 0){
            break;
        }
        printk("ap3212c : %s , readData retry again! \n",__func__);
        msleep(10);
    }
    if (i >= i2c_retry_count) {
        printk("ap3212c : %s,i2c read fail , over retry count\n",__func__);
        return -EIO;
    }
    return readData[0];
}

static int ap3212c_i2c_smbus_write_byte_data(struct i2c_client *client,uint8_t cmd,uint8_t write_data){
    int ret = 0;
    int i;
    int i2c_err = 0;
    int i2c_retry_count = 3;
    uint8_t writeData[2]={cmd,write_data};
    struct i2c_msg msg[] = {
        {
         .addr = 0x1c,
         .flags = 0,
         .len = 2,
         .buf = writeData,
         },
    };
    for (i = 0;i < i2c_retry_count; i++){
        i2c_err = i2c_transfer(client->adapter, msg, 1);
        //printk("ap3212c : %s , i2c transfer ret = %d\n",__func__,i2c_err);
        if(i2c_err >= 0){
            break;
        }
        msleep(10);
    }

    if (i >= i2c_retry_count) {
        printk("ap3212c : %s,i2c write fail , over retry count\n",__func__);
        return -EIO;
    }
    return ret;
}

static int ap3212c_i2c_read(struct i2c_client *client,uint8_t cmd){
    struct ap3212c_data *data = i2c_get_clientdata(client);
    int read_data;
    int i;
    mutex_lock(&data->lock);
    for(i=0;i<ap3212c_i2c_retry_count;i++){
        read_data = ap3212c_i2c_smbus_read_byte_data(client, cmd);
        if(read_data>=0){
            //break for loop
            i=ap3212c_i2c_retry_count;
        }else{
            printk("ap3212c %s : i2c read fail , cmd = %d , err code = %d , retry time = %d\n",__func__,cmd,read_data,i);
        }
    }
    mutex_unlock(&data->lock);
    if(read_data<0){
        printk("ap3212c %s : i2c read fail ,cmd = %d, err code = %d \n",__func__,cmd,read_data);
    }
    return read_data;
}

static int ap3212c_i2c_write(struct i2c_client *client,uint8_t cmd,uint8_t write_data){
    struct ap3212c_data *data = i2c_get_clientdata(client);
    int ret = 0;
    int i = 0;
    mutex_lock(&data->lock);
    for(i=0;i<ap3212c_i2c_retry_count;i++){
        ret = ap3212c_i2c_smbus_write_byte_data(client, cmd, write_data);
        if(ret>=0){
            //break for loop
            i=ap3212c_i2c_retry_count;
        }else{
            printk("ap3212c %s : i2c write fail , cmd = %d , write_data = %d, err code = %d , retry time = %d\n",__func__,cmd,write_data,ret,i);
        }
    }
    mutex_unlock(&data->lock);
    if(ret<0){
        printk("ap3212c %s : i2c write fail ,cmd = %d , write_data = %d, err code = %d \n",__func__,cmd,write_data,ret);
    }
    return ret;
}

static int ap3212c_als_get_adc(struct i2c_client *client){
    u8 low,high;
    u16 all;
    int err = 0;
    if(!ap3212c_als_calibration_updated_flag){
        err = ap3212c_als_update_calibration();
        if(err){
            printk("ap3212c : %s , ap3212c_als_update_calibration fail !\n",__func__);
        }else{
            printk("ap3212c : %s , ap3212c_als_update_calibration OK !\n",__func__);
        }
        ap3212c_als_calibration_updated_flag = true;
    }

    low = ap3212c_i2c_read(client,CMD_ALS_DATA_LOW);
    if(low<0){
        printk("ap3212c %s : i2c read low fail , low = %d \n",__func__,low);
        return -1;
    }

    high = ap3212c_i2c_read(client,CMD_ALS_DATA_HIGH);
    if(high<0){
        printk("ap3212c %s : i2c read high fail , high = %d \n",__func__,high);
        return -1;
    }

    all = (u16)(high<<8|low);
    return all;
}

static int ap3212c_als_get_lux(struct i2c_client *client){
    int adc;

    adc = ap3212c_als_get_adc(client);

    if(adc>=0){
        return (u32)( ( adc*ap3212c_als_calibratino_base_lux )/ap3212c_als_calibration_value );
    }else{
        return adc;
    }
}

static int ap3212c_als_get_default_lux(struct i2c_client *client){
    int adc;

    adc = ap3212c_als_get_adc(client);

    if(adc>=0){
        return (u32)( ( adc*ap3212c_als_calibratino_base_lux )/ap3212c_als_calibration_default_value );
    }else{
        return adc;
    }
}

static int ap3212c_als_on_default_conf(struct i2c_client *client){
    //TODO (als on/off) (ps on/off) need mutex !!!!
    int err=0;
    int ret;
    u8 ps_state;

    //get ps power state
    ps_state = ap3212c_i2c_read(client,CMD_SYS_CONFIG);
    if(ps_state<0){
        printk("ap3212c %s : can't read ps state , ps_state = %d \n",__func__,ps_state);
    }
    ps_state = (ps_state & CMD_SYS_CONFIG_PS_MASK);

    //als power on
    if(ps_state>0){
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALL_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : all power on fail \n",__func__);
        }
    }else{
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALS_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : als power on fail \n",__func__);
        }
    }

    //als config
    ret = ap3212c_i2c_write(client,CMD_ALS_CONFIG,CMD_ALS_CONFIG_DEFAULT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : als power on fail \n",__func__);
    }

    return err;
}

static int ap3212c_als_off_default_conf(struct i2c_client *client){
    //TODO (als on/off) (ps on/off) need mutex !!!!
    int err=0;
    int ret;
    u8 ps_state;

    //get ps power state
    ps_state = ap3212c_i2c_read(client,CMD_SYS_CONFIG);
    if(ps_state<0){
        printk("ap3212c %s : can't read ps state , ps_state = %d \n",__func__,ps_state);
    }
    ps_state = (ps_state & CMD_SYS_CONFIG_PS_MASK);

    //als power off
    if(ps_state>0){
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_PS_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : ps power on fail \n",__func__);
        }
    }else{
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALL_OFF_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : all power off fail \n",__func__);
        }
    }

    return err;
}

static int ap3212c_als_update_calibration()
{
    char buf[256];
    int calibration_value = 0;
    struct file *fp = NULL;
    mm_segment_t oldfs;
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(CAL_ALS_PATH, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            ap3212c_als_calibration_value = calibration_value;
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
        return 0;
    }else{
        return -1;
    }
}

static int ap3212c_ps_get_ambient_ir(struct i2c_client *client){
    u8 low,high;
    u16 all;

    low = ap3212c_i2c_read(client,CMD_IR_DATA_LOW);
    if(low<0){
        printk("ap3212c %s : i2c read low fail , low = %d \n",__func__,low);
        return -1;
    }
    low = (low & CMD_IR_DATA_LOW_MASK);

    high = ap3212c_i2c_read(client,CMD_IR_DATA_HIGH);
    if(high<0){
        printk("ap3212c %s : i2c read high fail , high = %d \n",__func__,high);
        return -1;
    }
    high = (high & CMD_IR_DATA_HIGH_MASK);

    //low is 2 bits
    all = (u16)(high<<2|low);
    return all;
}

static int ap3212c_ps_get_ambient_ir_overflow(struct i2c_client *client){
    u8 low;
    u8 ir_overflow_flag;
    low = ap3212c_i2c_read(client,CMD_IR_DATA_LOW);
    if(low<0){
        printk("ap3212c %s : i2c read low fail , low = %d \n",__func__,low);
        return -1;
    }
    ir_overflow_flag = (low & CMD_IR_DATA_LOW_OVERFLOW_MASK);
    if(ir_overflow_flag>0){
        return 1;
    }else{
        return 0;
    }
}

static int ap3212c_ps_get_adc(struct i2c_client *client){
    u8 low,high;
    u16 all;

    low = ap3212c_i2c_read(client,CMD_PS_DATA_LOW);
    if(low<0){
        printk("ap3212c %s : i2c read low fail , low = %d \n",__func__,low);
        return -1;
    }
    low = (low & CMD_PS_DATA_LOW_MASK);

    high = ap3212c_i2c_read(client,CMD_PS_DATA_HIGH);
    if(high<0){
        printk("ap3212c %s : i2c read high fail , high = %d \n",__func__,high);
        return -1;
    }
    high = (high & CMD_PS_DATA_HIGH_MASK);

    //low is 4 bits
    all = (u16)(high<<4|low);
    return all;
}

static int ap3212c_ps_get_object_distance(struct i2c_client *client){
    //TODO 1. find the threshold low/high
    //TODO 2. object distance (cm) can be got by ps adc ?

    // near/far solution
    int ret = 0;
    int fixed_distance = 5;//5cm
    bool xtalk_load_success = false;
    if(!ap3212c_ps_xtalk_calibration_updated_flag){
        ret = ap3212c_ps_update_calibration();
        if(ret<0){
            printk("ap3212c , ps udate calibration fail\n");
        }else{
            xtalk_load_success = true;
        }
        if(xtalk_load_success){
            ret = ap3212c_ps_cal_config(client);
            if(ret<0){
                printk("ap3212c , ps cal config set fail\n");
            }
        }
        ap3212c_ps_xtalk_calibration_updated_flag = true;
    }

    if( (!ap3212c_ps_thr_calibration_updated_flag) && xtalk_load_success){
        ret = ap3212c_ps_update_threshold_high_calibration();
        if(ret<0){
            printk("ap3212c , ps udate threshold high calibration fail\n");
        }
        ret = ap3212c_ps_update_threshold_low_calibration();
        if(ret<0){
            printk("ap3212c , ps udate threshold low calibration fail\n");
        }
        if(ap3212c_ps_high_threshold <= ap3212c_ps_low_threshold){
            printk("ap3212c , ap3212c_ps_high_threshold(%d) <= ap3212c_ps_low_threshold(%d) \n",ap3212c_ps_high_threshold,ap3212c_ps_low_threshold);
        }
        ret = ap3212c_ps_threshold_config(client);
        if(ret<0){
            printk("ap3212c , ps cal threshold config set fail\n");
        }
        ap3212c_ps_thr_calibration_updated_flag = true;
    }

    //+++ for PS INT auto clear
    ret = ap3212c_ps_get_adc(client);
    //---
    ret = ap3212c_ps_get_object_far(client);
    if(ap3212c_ps_irq_data_count>0){
        ap3212c_ps_irq_data_count--;
    }
    if(ret==1){
        //far
        printk("ap3212c : object far \n");
        return fixed_distance;
    }else if(ret==0){
        //near
        printk("ap3212c : object near \n");
        return 0;
    }else{
        //error
        return -1;
    }
}

static int ap3212c_ps_get_object_far(struct i2c_client *client){
    u8 low;
    u8 object_flag;
    low = ap3212c_i2c_read(client,CMD_PS_DATA_LOW);
    if(low<0){
        printk("ap3212c %s : i2c read low fail , low = %d \n",__func__,low);
        return -1;
    }
    object_flag = (low & CMD_PS_DATA_LOW_OBJECT_MASK);
    if(object_flag>0){
        return 0;
    }else{
        return 1;
    }
}

static int ap3212c_ps_default_config(struct i2c_client *client){
    int err=0;
    int ret;
    ret = ap3212c_i2c_write(client,CMD_PS_CONFIG,CMD_PS_CONFIG_DEFALUT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps config fail \n",__func__);
    }
    ret = ap3212c_i2c_write(client,CMD_PS_LED_DRIVE,CMD_PS_LED_DRIVE_DEFAULT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps led drive fail \n",__func__);
    }
    ret = ap3212c_i2c_write(client,CMD_PS_INT_FORM,CMD_PS_INT_MODE_DEFAULT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps int form set fail \n",__func__);
    }
    ret = ap3212c_i2c_write(client,CMD_PS_MEAN_TIME,CMD_PS_MEAN_TIME_DEFAULT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps mean time set fail \n",__func__);
    }
    ret = ap3212c_i2c_write(client,CMD_PS_LED_WAIT_TIME,CMD_PS_LED_WAIT_TIME_DEFAULT_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps led wait time set fail \n",__func__);
    }
    ret = ap3212c_i2c_write(client,CMD_INT_CLEAR,CMD_INT_CLEAR_SET_AUTO_VAL);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps int clear manager config auto set fail \n",__func__);
    }
    return err;
}

static int ap3212c_ps_cal_clean(struct i2c_client *client){
    int err=0;
    int ret;
    u8 low,high;
    low = 0;
    high = 0;
    printk("ap3212c %s :low =%d,high=%d \n",__func__,low,high);
    ret = ap3212c_i2c_write(client,CMD_PS_CAL_LOW,low);
    if(ret<0){
        printk("ap3212c %s : ps write cal low byte fail \n",__func__);
        return -1;
    }
    ret = ap3212c_i2c_write(client,CMD_PS_CAL_HIGH,high);
    if(ret<0){
        printk("ap3212c %s : ps write cal high byte fail \n",__func__);
        return -1;
    }
    return err;
}


static int ap3212c_ps_cal_config(struct i2c_client *client){
    int err=0;
    int ret;
    u8 low,high;
    printk("ap3212c %s : ap3212c_ps_xtalk_value = %d\n",__func__ ,ap3212c_ps_xtalk_value);
    low = (u8)(ap3212c_ps_xtalk_value&1);//one bit mask
    high = (u8)(ap3212c_ps_xtalk_value>>1);
    printk("ap3212c %s :low =%d,high=%d \n",__func__,low,high);
    ret = ap3212c_i2c_write(client,CMD_PS_CAL_LOW,low);
    if(ret<0){
        printk("ap3212c %s : ps write cal low byte fail \n",__func__);
        return -1;
    }
    ret = ap3212c_i2c_write(client,CMD_PS_CAL_HIGH,high);
    if(ret<0){
        printk("ap3212c %s : ps write cal high byte fail \n",__func__);
        return -1;
    }
    return err;
}
static int ap3212c_ps_threshold_config(struct i2c_client *client){
    int err=0;
    int ret;
    u8 low,high;
    int low_threshold = ap3212c_ps_low_threshold;
    int high_threshold = ap3212c_ps_high_threshold;
    int delta_threshold = 0;
    if(high_threshold <= low_threshold){
        printk("ap3212c %s : ps update threshold fail , th_low(%d) > th_high(%d) \n",__func__,low_threshold,high_threshold);
        // use default threshold
        low_threshold = ap3212c_ps_low_threshold_default;
        high_threshold = ap3212c_ps_high_threshold_default;
    }

    //+++ get a 4cm threshold
    printk("ap3212c %s : ps old threshold low = %d , high = %d\n",__func__,low_threshold,high_threshold);
    delta_threshold = high_threshold -low_threshold;
    if(delta_threshold > 0){
        printk("ap3212c %s : ps create 4cm threshold \n",__func__);
        high_threshold = low_threshold + (delta_threshold/2);
    }
    printk("ap3212c %s : ps new threshold low = %d , high = %d\n",__func__,low_threshold,high_threshold);
    //---

    //config low threshold
    printk("ap3212c %s : ps config low threshold = %d\n",__func__,low_threshold);
    low = (u8)( low_threshold&2 );//two bits mask
    high = (u8)( low_threshold>>2 );
    printk("ap3212c %s : low threshold low=%d,high=%d \n",__func__,low,high);
    ret = ap3212c_i2c_write(client,CMD_PS_LOW_THR_1,low);
    if(ret<0){
        printk("ap3212c %s : ps write threshold low byte 1 fail \n",__func__);
        return -1;
    }
    ret = ap3212c_i2c_write(client,CMD_PS_LOW_THR_2,high);
    if(ret<0){
        printk("ap3212c %s : ps write threshold low byte 2 fail \n",__func__);
        return -1;
    }

    //config high threshold
    printk("ap3212c %s : ps config high threshold = %d\n",__func__,high_threshold);
    low = (u8)( high_threshold&2 );//two bits mask
    high = (u8)( high_threshold>>2 );
    printk("ap3212c %s : high threshold low=%d,high=%d \n",__func__,low,high);
    ret = ap3212c_i2c_write(client,CMD_PS_HIGH_THR_1,low);
    if(ret<0){
        printk("ap3212c %s : ps write threshold high byte 1 fail \n",__func__);
        return -1;
    }
    ret = ap3212c_i2c_write(client,CMD_PS_HIGH_THR_2,high);
    if(ret<0){
        printk("ap3212c %s : ps write threshold high byte 2 fail \n",__func__);
        return -1;
    }

    return err;
}

static int ap3212c_ps_on_default_conf(struct i2c_client *client){
    //TODO (als on/off) (ps on/off) need mutex !!!!
    int err=0;
    int ret;
    u8 als_state;

    //get als power state
    als_state = ap3212c_i2c_read(client,CMD_SYS_CONFIG);
    if(als_state<0){
        printk("ap3212c %s : can't read als state , als_state = %d \n",__func__,als_state);
    }
    als_state = (als_state & CMD_SYS_CONFIG_ALS_MASK);

    //ps power on
    if(als_state>0){
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALL_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : all power on fail \n",__func__);
        }
    }else{
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_PS_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : ps power on fail \n",__func__);
        }
    }

    //ps config
    ret = ap3212c_ps_default_config(client);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps config fail\n",__func__);
    }

    //ps set calibration
    ret = ap3212c_ps_cal_config(client);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps cal config fail\n",__func__);
    }
    //ps set threshold
    ret = ap3212c_ps_threshold_config(client);
    if(ret<0){
        err = -1;
        printk("ap3212c %s : ps threshold config fail\n",__func__);
    }
    return err;
}

static int ap3212c_ps_off_default_conf(struct i2c_client *client){
    //TODO (als on/off) (ps on/off) need mutex !!!!
    int err=0;
    int ret;
    u8 als_state;

    //get als power state
    als_state = ap3212c_i2c_read(client,CMD_SYS_CONFIG);
    if(als_state<0){
        printk("ap3212c %s : can't read als state , als_state = %d \n",__func__,als_state);
        //can't get als power state , assume als is on
        als_state = 1;
    }else{
        als_state = (als_state & CMD_SYS_CONFIG_ALS_MASK);
    }

    //ps power off
    if(als_state>0){
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALS_ON_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : als power on fail \n",__func__);
        }
    }else{
        ret = ap3212c_i2c_write(client,CMD_SYS_CONFIG,CMD_SYS_CONFIG_ALL_OFF_VAL);
        if(ret<0){
            err = -1;
            printk("ap3212c %s : all power on fail \n",__func__);
        }
    }
    return err;
}
static int ap3212c_ps_update_calibration(void){
    char buf[256];
    int calibration_value = 0;
    struct file *fp = NULL;
    mm_segment_t oldfs;
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(CAL_PS_XTALK_PATH, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value >= 0){
            printk("ap3212c %s : update xtalk cal = %d \n",__func__,calibration_value);
            ap3212c_ps_xtalk_value = calibration_value;
        }else{
            printk("ap3212c %s : wrong xtalk cal = %d , use default cal \n",__func__,calibration_value);
            ap3212c_ps_xtalk_value = ap3212c_ps_xtalk_default_value;
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
        return 0;
    }else{
        printk("ap3212c %s : update xtalk cal fail , use default cal \n",__func__);
        ap3212c_ps_xtalk_value = ap3212c_ps_xtalk_default_value;
        return -1;
    }
}

static int ap3212c_ps_update_threshold_high_calibration(void){
    char buf[256];
    int calibration_value = 0;
    struct file *fp = NULL;
    mm_segment_t oldfs;
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(CAL_PS_THR_H_PATH, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            printk("ap3212c %s : factory thr high cal = %d \n",__func__,calibration_value);
            ap3212c_ps_high_threshold = calibration_value+CAL_PS_THR_H_SHIFT;
            printk("ap3212c %s : update thr high cal = %d \n",__func__,ap3212c_ps_high_threshold);
        }else{
            printk("ap3212c %s : wrong thr high cal = %d , use default cal \n",__func__,calibration_value);
            ap3212c_ps_high_threshold = ap3212c_ps_high_threshold_default;
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
        return 0;
    }else{
        ap3212c_ps_high_threshold = ap3212c_ps_high_threshold_default;
        printk("ap3212c %s : update thr high cal fail , use default cal \n",__func__);
        return -1;
    }
}

static int ap3212c_ps_update_threshold_low_calibration(void){
    char buf[256];
    int calibration_value = 0;
    struct file *fp = NULL;
    mm_segment_t oldfs;
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(CAL_PS_THR_L_PATH, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            printk("ap3212c %s : factory thr low cal = %d \n",__func__,calibration_value);
            ap3212c_ps_low_threshold = calibration_value+CAL_PS_THR_L_SHIFT;
            printk("ap3212c %s : update thr low cal = %d \n",__func__,ap3212c_ps_low_threshold);
        }else{
            printk("ap3212c %s : wrong thr low cal = %d , use default cal \n",__func__,calibration_value);
            ap3212c_ps_low_threshold = ap3212c_ps_low_threshold_default;
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
        return 0;
    }else{
        ap3212c_ps_low_threshold = ap3212c_ps_low_threshold_default;
        printk("ap3212c %s : update thr low cal fail , use default cal \n",__func__);
        return -1;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

static ssize_t ap3212c_show_als_adc(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int adc = ap3212c_als_get_adc(client);
    return sprintf(buf, "%d\n", adc);
}

static ssize_t ap3212c_show_als_lux(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int lux = ap3212c_als_get_lux(client);
    return sprintf(buf, "%d\n", lux);
}

static ssize_t ap3212c_show_ps_adc(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int adc = ap3212c_ps_get_adc(client);
    return sprintf(buf, "%d\n", adc);
}

static ssize_t ap3212c_show_ps_ir(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int ir = ap3212c_ps_get_ambient_ir(client);
    return sprintf(buf, "%d\n", ir);
}

static ssize_t ap3212c_show_ps_ir_overflow(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int ir_overflow = ap3212c_ps_get_ambient_ir_overflow(client);
    return sprintf(buf, "%d\n", ir_overflow);
}

static ssize_t ap3212c_show_ps_object_far(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int object_far = ap3212c_ps_get_object_far(client);
    return sprintf(buf, "%d\n", object_far);
}

static ssize_t ap3212c_show_ps_object_distance(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int object_distance = ap3212c_ps_get_object_distance(client);
    return sprintf(buf, "%d\n", object_distance);
}

static ssize_t ap3212c_als_status(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int write_i2c_err = 0;
    int read_i2c_err = 0;
    int i;
    int retry_time = 5;

    for(i=0;i<retry_time;i++){
        write_i2c_err = ap3212c_als_on_default_conf(client);
        if( write_i2c_err<0 ){
            printk("ap3212c , check als write i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    for(i=0;i<retry_time;i++){
        read_i2c_err = ap3212c_als_get_adc(client);
        if(read_i2c_err<0){
            printk("ap3212c , check als read i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    if(write_i2c_err<0 || read_i2c_err<0){
        return sprintf(buf, "%d\n", 0);
    }else{
        return sprintf(buf, "%d\n", 1);
    }
}

static ssize_t ap3212c_ps_status(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int write_i2c_err = 0;
    int read_i2c_err = 0;
    int i;
    int retry_time = 5;

    for(i=0;i<retry_time;i++){
        write_i2c_err = ap3212c_ps_on_default_conf(client);
        if( write_i2c_err<0 ){
            printk("ap3212c , check ps write i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    for(i=0;i<retry_time;i++){
        read_i2c_err = ap3212c_ps_get_adc(client);
        if(read_i2c_err<0){
            printk("ap3212c , check ps read i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    if(write_i2c_err<0 || read_i2c_err<0){
        return sprintf(buf, "%d\n", 0);
    }else{
        return sprintf(buf, "%d\n", 1);
    }
}

static ssize_t ap3212c_als_enable_disable_poll_event(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long enable;
    printk("ap3212c : %s\n",__func__);
    if (strict_strtol(buf, 10, &enable))
        return -EINVAL;
    if ((enable != 1) && (enable != 0))
        return -EINVAL;

    if(enable == 1){
        printk("ap3212c : ap3212c als poll enable\n");
        printk("ap3212c : ap3212c first event no wait \n");
        ap3212c_als_flag_pollin = true;
        queue_delayed_work(ap3212c_als_poll_work_queue, &ap3212c_als_report_poll_event_work, 0);
    }else{
        printk("ap3212c : ap3212c als poll disable\n");
        cancel_delayed_work_sync(&ap3212c_als_report_poll_event_work);
    }
    return strnlen(buf, count);
}

static ssize_t ap3212c_ps_enable_disable_poll_event(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long enable;
    printk("ap3212c : %s\n",__func__);
    if (strict_strtol(buf, 10, &enable))
        return -EINVAL;
    if ((enable != 1) && (enable != 0))
        return -EINVAL;

    if(enable == 1){
        printk("ap3212c : ap3212c ps poll enable\n");
        ap3212c_ps_init_poll_count = 0;
        ap3212c_ps_irq_data_count = PS_INIT_POLL_MAX_COUNT;//+++ add 10 count for init polling
        ap3212c_ps_flag_pollin = true;
        queue_delayed_work(ap3212c_ps_poll_work_queue, &ap3212c_ps_report_poll_event_work, msecs_to_jiffies(0));
    }else{
        printk("ap3212c : ap3212c ps poll disable\n");
        cancel_delayed_work_sync(&ap3212c_ps_report_poll_event_work);
    }
    return strnlen(buf, count);
}

static ssize_t ap3212c_als_set_delay(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long temp;
    int delay_time;
    if (strict_strtol(buf, 10, &temp))
        return -EINVAL;
    delay_time = (int)temp;
    printk("ap3212c : ap3212c_als_set_delay delay_time = %d\n",delay_time);
    if(delay_time<ap3212c_als_min_poll_time){
        ap3212c_als_poll_time = ap3212c_als_min_poll_time;
    }else{
        ap3212c_als_poll_time = delay_time;
    }
    printk("ap3212c : ap3212c_als_poll_time = %d\n",ap3212c_als_poll_time);
    return strnlen(buf, count);
}

static ssize_t ap3212c_als_get_delay(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ap3212c_als_poll_time);
}

static ssize_t ap3212c_ps_set_delay(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long temp;
    int delay_time;
    if (strict_strtol(buf, 10, &temp))
        return -EINVAL;
    delay_time = (int)temp;
    printk("ap3212c : ap3212c_ps_set_delay delay_time = %d\n",delay_time);
    if(delay_time<ap3212c_ps_min_poll_time){
        ap3212c_ps_poll_time = ap3212c_ps_min_poll_time;
    }else{
        ap3212c_ps_poll_time = delay_time;
    }
    printk("ap3212c : ap3212c_ps_poll_time = %d\n",ap3212c_ps_poll_time);
    return strnlen(buf, count);
}

static ssize_t ap3212c_ps_get_delay(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ap3212c_ps_poll_time);
}

static ssize_t ap3212c_als_calibration_update(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    int err = ap3212c_als_update_calibration();
    return sprintf(buf, "%d\n", err);
}

static ssize_t ap3212c_ps_calibration_update(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err = 0;
    int ret;
    ret = ap3212c_ps_update_calibration();
    if(ret<0){
        err = -1;
        printk("ap3212c , ps udate calibration fail\n");
    }
    ret = ap3212c_ps_cal_config(client);
    if(ret<0){
        err = -1;
        printk("ap3212c , ps cal config set fail\n");
    }
    return sprintf(buf, "%d\n", err);
}

static ssize_t ap3212c_show_als_default_lux(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int lux = ap3212c_als_get_default_lux(client);
    return sprintf(buf, "%d\n", lux);
}

static ssize_t ap3212c_show_ps_cal_clean(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err = ap3212c_ps_cal_clean(client);
    return sprintf(buf, "%d\n", err);
}

static ssize_t ap3212c_ps_threshold_update(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int err = 0;
    int ret;
    ret = ap3212c_ps_update_threshold_high_calibration();
    if(ret<0){
        err = -1;
        printk("ap3212c , ps udate threshold high calibration fail\n");
    }
    ret = ap3212c_ps_update_threshold_low_calibration();
    if(ret<0){
        err = -1;
        printk("ap3212c , ps udate threshold low calibration fail\n");
    }
    if(ap3212c_ps_high_threshold <= ap3212c_ps_low_threshold){
        err = -1;
        printk("ap3212c , ap3212c_ps_high_threshold(%d) <= ap3212c_ps_low_threshold(%d) \n",ap3212c_ps_high_threshold,ap3212c_ps_low_threshold);
    }
    ret = ap3212c_ps_threshold_config(client);
    if(ret<0){
        err = -1;
        printk("ap3212c , ps cal threshold config set fail\n");
    }
    return sprintf(buf, "%d\n", err);
}

static ssize_t ap3212c_ps_show_irq_data_count(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    return sprintf(buf, "%d\n", ap3212c_ps_irq_data_count);
}

static SENSOR_DEVICE_ATTR(als_adc,0644,ap3212c_show_als_adc,NULL,0);
static SENSOR_DEVICE_ATTR(als_lux,0644,ap3212c_show_als_lux,NULL,1);
static SENSOR_DEVICE_ATTR(ps_adc,0644,ap3212c_show_ps_adc,NULL,2);
static SENSOR_DEVICE_ATTR(ps_ir,0644,ap3212c_show_ps_ir,NULL,3);
static SENSOR_DEVICE_ATTR(ps_ir_overflow,0644,ap3212c_show_ps_ir_overflow,NULL,4);
static SENSOR_DEVICE_ATTR(ps_object_far,0644,ap3212c_show_ps_object_far,NULL,5);
static SENSOR_DEVICE_ATTR(ps_object_distance,0644,ap3212c_show_ps_object_distance,NULL,6);
static SENSOR_DEVICE_ATTR(als_status,0666,ap3212c_als_status,ap3212c_als_enable_disable_poll_event,7);
static SENSOR_DEVICE_ATTR(ps_status,0666,ap3212c_ps_status,ap3212c_ps_enable_disable_poll_event,8);
static SENSOR_DEVICE_ATTR(als_calibration_update,0644,ap3212c_als_calibration_update,NULL,9);
static SENSOR_DEVICE_ATTR(als_default_lux,0644,ap3212c_show_als_default_lux,NULL,10);
static SENSOR_DEVICE_ATTR(als_poll_time,0666,ap3212c_als_get_delay,ap3212c_als_set_delay,11);
static SENSOR_DEVICE_ATTR(ps_poll_time,0666,ap3212c_ps_get_delay,ap3212c_ps_set_delay,12);
static SENSOR_DEVICE_ATTR(ps_calibration_update,0644,ap3212c_ps_calibration_update,NULL,13);
static SENSOR_DEVICE_ATTR(ps_cal_clean,0644,ap3212c_show_ps_cal_clean,NULL,14);
static SENSOR_DEVICE_ATTR(ps_threshold_update,0644,ap3212c_ps_threshold_update,NULL,15);
static SENSOR_DEVICE_ATTR(ps_has_data,0644,ap3212c_ps_show_irq_data_count,NULL,16);
//TODO
//DEVICE ATTR dump regs
//DEVICE ATTR debug_reg ( directly read/write )

static struct attribute *ap3212c_attributes[] = {
    &sensor_dev_attr_als_adc.dev_attr.attr,
    &sensor_dev_attr_als_lux.dev_attr.attr,
    &sensor_dev_attr_ps_adc.dev_attr.attr,
    &sensor_dev_attr_ps_ir.dev_attr.attr,
    &sensor_dev_attr_ps_ir_overflow.dev_attr.attr,
    &sensor_dev_attr_ps_object_far.dev_attr.attr,
    &sensor_dev_attr_ps_object_distance.dev_attr.attr,
    &sensor_dev_attr_als_status.dev_attr.attr,
    &sensor_dev_attr_ps_status.dev_attr.attr,
    &sensor_dev_attr_als_calibration_update.dev_attr.attr,
    &sensor_dev_attr_als_default_lux.dev_attr.attr,
    &sensor_dev_attr_als_poll_time.dev_attr.attr,
    &sensor_dev_attr_ps_poll_time.dev_attr.attr,
    &sensor_dev_attr_ps_calibration_update.dev_attr.attr,
    &sensor_dev_attr_ps_cal_clean.dev_attr.attr,
    &sensor_dev_attr_ps_threshold_update.dev_attr.attr,
    &sensor_dev_attr_ps_has_data.dev_attr.attr,
    NULL
};

static const struct attribute_group ap3212c_attr_group = {
    .attrs = ap3212c_attributes,
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

static irqreturn_t ap3212c_irq_handle_fn(int irq, void *saved_data)
{
    //TODO check irq by prox
//    struct ap3212c_data *data = saved_data;
//    int int_status = 0;
//    int ps_int_status = 0;
//    printk("ap3212c irq test3\n");
//    int_status = ap3212c_i2c_read(data->client,CMD_INT_STATUS);
//    ps_int_status = (int_status & CMD_INT_STATUS_PS_MASK);
//    if(ps_int_status > 0){
//    //handle wake up for prox
//    }
    ap3212c_ps_irq_data_count++;
    printk("ap3212c : irq data count = %d\n",ap3212c_ps_irq_data_count);
    ap3212c_ps_flag_pollin = true;
    wake_up_interruptible(&ap3212c_ps_poll_wait_queue_head_t);
    return IRQ_HANDLED;
}

static void  ap3212c_als_report_poll_event(struct work_struct * work)
{
    //wake up device poll wait queue
    ap3212c_als_flag_pollin = true;
    wake_up_interruptible(&ap3212c_als_poll_wait_queue_head_t);
    //printk("ap3212c : %s\n", __func__);
    //add next work for polling
    queue_delayed_work(ap3212c_als_poll_work_queue, &ap3212c_als_report_poll_event_work, msecs_to_jiffies(ap3212c_als_poll_time));
}

static void  ap3212c_ps_report_poll_event(struct work_struct * work)
{
    //wake up device poll wait queue
    ap3212c_ps_flag_pollin = true;
    wake_up_interruptible(&ap3212c_ps_poll_wait_queue_head_t);
    //printk("ap3212c : %s\n", __func__);
    //add next work for polling
    if(ap3212c_ps_init_poll_count < PS_INIT_POLL_MAX_COUNT){
        queue_delayed_work(ap3212c_ps_poll_work_queue, &ap3212c_ps_report_poll_event_work, msecs_to_jiffies(ap3212c_ps_poll_time));
        ap3212c_ps_init_poll_count++;
        printk("ap3212c : ps poll count = %d (max count 10)\n",ap3212c_ps_init_poll_count);
    }
}

int ap3212c_als_open(struct inode *inode, struct file *filp)
{
    printk("ap3212c : %s\n", __func__);
    return 0;
}

int ap3212c_als_release(struct inode *inode, struct file *filp)
{
    printk("ap3212c : %s\n", __func__);
    return 0;
}

static unsigned int ap3212c_als_poll(struct file *filp, poll_table *wait){
    unsigned int mask = 0;
    poll_wait(filp, &ap3212c_als_poll_wait_queue_head_t, wait);
    if(ap3212c_als_flag_pollin==true){
        mask |= POLLIN;
        ap3212c_als_flag_pollin=false;
    }
    //printk("ap3212c : %s\n", __func__);
    return mask;
}

struct file_operations ap3212c_als_fops = {
    .owner = THIS_MODULE,
    .open = ap3212c_als_open,
    .release = ap3212c_als_release,
    .poll = ap3212c_als_poll,
//    .unlocked_ioctl = ap3212c_als_ioctl,
};

int ap3212c_ps_open(struct inode *inode, struct file *filp)
{
    printk("ap3212c : %s\n", __func__);
    return 0;
}

int ap3212c_ps_release(struct inode *inode, struct file *filp)
{
    printk("ap3212c : %s\n", __func__);
    return 0;
}

static unsigned int ap3212c_ps_poll(struct file *filp, poll_table *wait){
    unsigned int mask = 0;
    poll_wait(filp, &ap3212c_ps_poll_wait_queue_head_t, wait);
    if(ap3212c_ps_flag_pollin==true){
        mask |= POLLIN;
        ap3212c_ps_flag_pollin=false;
    }
    //printk("ap3212c : %s\n", __func__);
    return mask;
}

struct file_operations ap3212c_ps_fops = {
    .owner = THIS_MODULE,
    .open = ap3212c_ps_open,
    .release = ap3212c_ps_release,
    .poll = ap3212c_ps_poll,
//    .unlocked_ioctl = ap3212c_ps_ioctl,
};

static int ap3212c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    struct ap3212c_data *data;
    int err = 0;

    printk("ap3212c_probe+\n");

    data = kzalloc(sizeof(struct ap3212c_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    i2c_set_clientdata(client, data);
    mutex_init(&data->lock);

    /* ap3212c init */
    //TODO if probe init fail , need a later init or other enable method
//    err = ap3212c_als_off_default_conf(client);
//    if(err){
//        printk("ap3212c : %s , ap3212c_als_off_default_conf fail !\n",__func__);
//    }
    err = ap3212c_als_on_default_conf(client);
    if(err){
        printk("ap3212c : %s , ap3212c_als_on_default_conf fail !\n",__func__);
    }
//    err = ap3212c_ps_off_default_conf(client);
//    if(err){
//        printk("ap3212c : %s , ap3212c_ps_off_default_conf fail !\n",__func__);
//    }
    err = ap3212c_ps_on_default_conf(client);
    if(err){
        printk("ap3212c : %s , ap3212c_ps_on_default_conf fail !\n",__func__);
    }

    /* register sysfs hooks */
    err = sysfs_create_group(&client->dev.kobj, &ap3212c_attr_group);
    if (err){
        printk("ap3212c : init sysfs fail\n");
        goto exit_kfree;
    }

    /* poll event work queue*/
    //als
    ap3212c_als_poll_work_queue = create_singlethread_workqueue("lightsensor_poll_wq");
    if(!ap3212c_als_poll_work_queue){
        printk("ap3212c : unable to create als poll workqueue\n");
        goto remove_sysfs_group;
    }
    INIT_DELAYED_WORK(&ap3212c_als_report_poll_event_work, ap3212c_als_report_poll_event);
    //ps
    ap3212c_ps_poll_work_queue = create_singlethread_workqueue("proximitysensor_poll_wq");
    if(!ap3212c_ps_poll_work_queue){
        printk("ap3212c : unable to create ps poll workqueue\n");
        goto remove_als_work_queue;
    }
    INIT_DELAYED_WORK(&ap3212c_ps_report_poll_event_work, ap3212c_ps_report_poll_event);

    /* misc dev */
    //als
    ap3212c_als_misc_dev.minor = MISC_DYNAMIC_MINOR;
    ap3212c_als_misc_dev.name = "lightsensor";
    ap3212c_als_misc_dev.fops  = &ap3212c_als_fops;
    init_waitqueue_head(&ap3212c_als_poll_wait_queue_head_t);
    err = misc_register(&ap3212c_als_misc_dev);
    if(err){
        printk("ap3212c : register als misc dev fail\n");
        goto remove_ps_work_queue;
    }
    //ps
    ap3212c_ps_misc_dev.minor = MISC_DYNAMIC_MINOR;
    ap3212c_ps_misc_dev.name = "proximitysensor";
    ap3212c_ps_misc_dev.fops  = &ap3212c_ps_fops;
    init_waitqueue_head(&ap3212c_ps_poll_wait_queue_head_t);
    err = misc_register(&ap3212c_ps_misc_dev);
    if(err){
        printk("ap3212c : register ps misc dev fail\n");
        goto remove_als_misc_dev;
    }

    //ps irq
    int irq_gpio = get_gpio_by_name("ALS_INT#");

    printk("ap3212c : ALS_INT# = %d\n",irq_gpio);
    printk("ap3212c : test 5\n");
    err = gpio_request(irq_gpio,"ALS_INT#");
    if(err){
        printk("ap3212c , request gpio %d ALS_INT# fail!!!\n",irq_gpio);
    }
    err = gpio_direction_input(irq_gpio);
    if(err){
        printk("ap3212c , gpio_direction_input gpio %d ALS_INT# fail!!!\n",irq_gpio);
    }
    err=request_irq( gpio_to_irq(irq_gpio) , ap3212c_irq_handle_fn,
          IRQF_TRIGGER_FALLING | IRQF_DISABLED , "ap3212c", data);
    if(err){
        printk("ap3212c : request irq fail , err = %d\n",err);
    }

    printk("ap3212c : %s , probe ok!\n",__func__);
    printk("ap3212c_probe-\n");
    return 0;

//remove_ps_misc_dev:
//    misc_deregister(&ap3212c_ps_misc_dev);
remove_als_misc_dev:
    misc_deregister(&ap3212c_als_misc_dev);
remove_ps_work_queue:
    destroy_workqueue(ap3212c_ps_poll_work_queue);
remove_als_work_queue:
    destroy_workqueue(ap3212c_als_poll_work_queue);
remove_sysfs_group:
    sysfs_remove_group(&client->dev.kobj, &ap3212c_attr_group);
exit_kfree:
    kfree(data);

    printk("ap3212c : %s , probe fail!\n",__func__);
    printk("ap3212c_probe-\n");
    return err;
}

static int ap3212c_remove(struct i2c_client *client)
{
    misc_deregister(&ap3212c_ps_misc_dev);
    misc_deregister(&ap3212c_als_misc_dev);
    destroy_workqueue(ap3212c_ps_poll_work_queue);
    destroy_workqueue(ap3212c_als_poll_work_queue);
    sysfs_remove_group(&client->dev.kobj, &ap3212c_attr_group);
    kfree(i2c_get_clientdata(client));
    printk("ap3212c : remove successed\n");
    return 0;
}

#ifdef CONFIG_PM
static int ap3212c_suspend(struct device *dev)
{
    int err = 0;
    struct i2c_client *client = to_i2c_client(dev);
    printk("ap3212c_suspend+\n");
    err = ap3212c_als_off_default_conf(client);
    if(err){
        printk("ap3212c : als power off fail \n");
    }
    err = ap3212c_ps_off_default_conf(client);
    if(err){
        printk("ap3212c : ps power off fail \n");
    }
    printk("ap3212c_suspend-\n");
    return 0;// DO NOT return err , cause system fail
}

static int ap3212c_resume(struct device *dev)
{
    int err=0;
    struct i2c_client *client = to_i2c_client(dev);
    printk("ap3212c_resume+\n");

    err = ap3212c_als_on_default_conf(client);
    if(err){
        printk("ap3212c : als power oo fail \n");
    }

    err = ap3212c_ps_on_default_conf(client);
    if(err){
        printk("ap3212c : als power on fail \n");
    }

    printk("ap3212c_resume-\n");
    return 0;// DO NOT return err , cause system fail
}

static const struct dev_pm_ops ap3212c_pm_ops = {
    .suspend = ap3212c_suspend,
    .resume  = ap3212c_resume,
};
#define ap3212c_PM_OPS (&ap3212c_pm_ops)

#else
#define ap3212c_PM_OPS NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id ap3212c_id[] = {
    { "ap3212c", 0 },
    {}
};
static struct i2c_driver ap3212c_driver = {
    .driver = {
        .name    = "ap3212c",
        .owner    = THIS_MODULE,
        .pm = ap3212c_PM_OPS,
    },
    .probe    = ap3212c_probe,
    .remove    = ap3212c_remove,
    .id_table = ap3212c_id,
};

static int __init ap3212c_init(void)
{
    int ret=0;
    printk("%s+\n", __func__);
    ret = i2c_add_driver(&ap3212c_driver);
    printk("%s-\n", __func__);
    return ret;
}

static void __exit ap3212c_exit(void)
{
    i2c_del_driver(&ap3212c_driver);
}

MODULE_AUTHOR("asus");
MODULE_DESCRIPTION("ap3212c driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1");

module_init(ap3212c_init);
module_exit(ap3212c_exit);
