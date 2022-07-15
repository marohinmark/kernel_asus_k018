/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "uPILogger.h"

extern struct battery_info_reply batt_info;

static int bq27520_proc_info_dump_read(struct seq_file *m, void *p) {
        int len, local_len;
        static int bq_batt_percentage = 0;
        static int bq_batt_volt = 0;
        static int bq_batt_current = 0;
        static int bq_batt_temp = 0;
        static int bq_batt_remaining_capacity = 0;
        static int bq_batt_full_charge_capacity = 0;

        len = local_len = 0;

        bq_batt_full_charge_capacity = bq27520_asus_battery_dev_read_full_charge_capacity();

        bq_batt_remaining_capacity = bq27520_asus_battery_dev_read_remaining_capacity();
        bq_batt_percentage         = bq27520_asus_battery_dev_read_percentage();
        bq_batt_volt               = bq27520_asus_battery_dev_read_volt();
        bq_batt_current            = bq27520_asus_battery_dev_read_current();
        bq_batt_temp               = bq27520_asus_battery_dev_read_temp();

        if (bq_batt_current >= 0) bq_batt_current  = bq_batt_current  - 0x10000;

        seq_printf(m,"LMD(mAh): %d\n", bq_batt_full_charge_capacity);
        seq_printf(m,"NAC(mAh): %d\n", bq_batt_remaining_capacity);
        seq_printf(m,"RSOC: %d\n", bq_batt_percentage);
        seq_printf(m,"USOC: %d\n", batt_info.percentage);
        seq_printf(m,"voltage(mV): %d\n", bq_batt_volt);
        seq_printf(m,"average_current(mA): %d\n", bq_batt_current);
        seq_printf(m,"temp: %d\n", bq_batt_temp);

        return len;

}

static int proc_bq27520_test_info_dump_open(struct inode *inode, struct file *file) {
	return single_open(file, bq27520_proc_info_dump_read, NULL);
}

static const struct file_operations proc_bq27520_test_info_dump_ops = {
	.open		= proc_bq27520_test_info_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};

int bq27520_register_upilogger_proc_fs(void) {
        struct proc_dir_entry *entry=NULL;

        entry = proc_create("bq27520_test_info_dump", 0666, NULL, &proc_bq27520_test_info_dump_ops);
        if (!entry) {
            printk("[%s]Unable to create bq27520_test_info_dump \n", __FUNCTION__);
            return -EINVAL;
        }

        return 0;
}

