/*
 * platform_kxtj2.c: Accel platform data initialization file
 *
 * (C) Copyright 2013
 * Author : ASUS
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_kxtj2_fe170cg.h"
#include <linux/input/kxtj2_fe170cg.h>

void *kxtj2_fe170cg_platform_data(void *info)
{
    static struct kionix_accel_platform_data kxtj2_pdata;

    kxtj2_pdata.min_interval = 5;
    kxtj2_pdata.poll_interval = 200;

    kxtj2_pdata.accel_direction = 8;
    kxtj2_pdata.accel_irq_use_drdy = 1;

    kxtj2_pdata.accel_res = KIONIX_ACCEL_RES_12BIT;
    kxtj2_pdata.accel_g_range = KIONIX_ACCEL_G_2G;

    kxtj2_pdata.int_gpio = get_gpio_by_name("accel_int");

    return &kxtj2_pdata;
}
