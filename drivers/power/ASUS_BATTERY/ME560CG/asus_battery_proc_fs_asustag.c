/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include "asus_battery.h"
#include "bq27520_battery_core.h"
#include <linux/power_supply.h>

#define PROCFS_BATTERY 		"battery_status"

extern struct mutex batt_info_mutex;
extern struct battery_info_reply batt_info;
//Carlisle add for ATD gague IC status
extern struct battery_dev_info bq27520_dev_info;

#ifdef ME560CG_ENG_BUILD

//-->Carlisle add for ATD querying gauge IC status
int check_gaugeIC_status_read(struct seq_file *m, void *p) {
    struct i2c_client *client = NULL;
    int len; 
    int ret;
    int flags=0;
    int gauge_status=0;
    
    client = bq27520_dev_info.i2c; 

    ret = bq27520_send_subcmd(client, &flags, BQ27520_SUBCMD_CTNL_STATUS);
    printk("+++Carl ret= %d++\n",ret);
    if (ret==0) {

        printk("+++Carl Success++++\n");
	    gauge_status=1;
	    len = seq_printf(m, "%d\n", gauge_status);
        
    }
    else{
	    
	    dev_err(&client->dev, "Send subcommand error %d.\n", ret); 
        printk("+++Carl Error++++\n");
        len = seq_printf(m, "%d\n", gauge_status);
	}
    

    return len;
}        
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
int check_charge_status_read(struct seq_file *m, void *p) {
	int res = 0;
	int len = 0;
	
    
    struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);
        
    if(tmp_batt_info.status==POWER_SUPPLY_STATUS_CHARGING){
       res = 1;
       len = seq_printf(m, "%d\n", res);    
    }
    else
     len = seq_printf(m, "%d\n", res);   
        
    return len;
}        
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
int check_gauge_fw_status_read(struct seq_file *m, void *p) {
	int len = 0;
	int ret = 0;
	int fw_cfg_status = 0;
	
	fw_cfg_status = bq27520_asus_battery_dev_read_fw_cfg_version();
	
	if(fw_cfg_status == LATEST_FW_CFG_VERSION){
		len = 1;
		ret = seq_printf(m, "%d\n", len);
    }	
	else
	 ret = seq_printf(m, "%d\n", len);    
    
    return ret;
}
//<--Carlisle add for ATD querying gauge IC fw flash identify end

//-->Carlisle add for ATD querying gauge IC status
static int gaugeIC_status_open(struct inode *inode, struct file *file) {
	return single_open(file, check_gaugeIC_status_read, NULL);
}

static const struct file_operations gaugeIC_status_ops = {
	.open		= gaugeIC_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};
int check_gaugeIC_status(void) {
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("gaugeIC_status", 0666, NULL, &gaugeIC_status_ops);
    if (!entry) {
        BAT_DBG_E("Unable to create gaugeIC_status\n");
        return -EINVAL;
    }
    
    return 0;
}        
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
static int charge_status_open(struct inode *inode, struct file *file) {
	return single_open(file, check_charge_status_read, NULL);
}

static const struct file_operations charge_status_ops = {
	.open		= charge_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};
int check_charge_status(void) {
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("charge_status", 0666, NULL, &charge_status_ops);
    if (!entry) {
        BAT_DBG_E("Unable to create charge_status\n");
        return -EINVAL;
    }
    
    return 0;
}
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
static int gauge_fw_status_open(struct inode *inode, struct file *file) {
	return single_open(file, check_gauge_fw_status_read, NULL);
}

static const struct file_operations gauge_fw_status_ops = {
	.open		= gauge_fw_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};
int check_gauge_fw_status(void) {
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("gauge_fw_status", 0666, NULL, &gauge_fw_status_ops);
    if (!entry) {
        BAT_DBG_E("Unable to create gauge_fw_status\n");
        return -EINVAL;
    }
    
    return 0;
}
//<--Carlisle add for ATD querying gauge IC fw flash identify end

//asus_eng_charging_limit     ------START
int asus_charging_toggle_write(struct file *file, const char *buffer, size_t count, loff_t *data) {
    struct battery_info_reply tmp_batt_info;
    bool eng_charging_limit = true;

    BAT_DBG(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (buffer[0] == '0') {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
    }
    else if (buffer[0] == '1') {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
    }

    tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}

static const struct file_operations asus_eng_charging_limit_ops = {
	.read		= seq_read,
        .write          = asus_charging_toggle_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};
//asus_eng_charging_limit      ------END

int init_asus_charging_toggle(void) {
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("asus_eng_charging_limit", 0666, NULL, &asus_eng_charging_limit_ops);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }
    return 0;
}

#endif

int asus_battery_register_proc_fs_test(void) {
        int ret = 0;
#ifdef ME560CG_ENG_BUILD
        ret = init_asus_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_asus_charging_toggle\n");
                goto proc_fail;
        }
//-->Carlisle add for ATD querying gauge IC status
        ret = check_gaugeIC_status();
        if (ret) {
                BAT_DBG_E("Unable to create proc check_gaugeIC_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
        ret = check_charge_status();
        if (ret) {
                BAT_DBG_E("Unable to create proc check_charge_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
		ret = check_gauge_fw_status();
		if (ret) {
                BAT_DBG_E("Unable to create proc check_gauge_fw_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying gauge IC fw flash identify end
proc_fail:
#endif
        return ret;
}
