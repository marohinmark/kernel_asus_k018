/* 
 * Header file for : Waltop ASIC5 pen touch controller.
 * drivers/input/tablet/waltop_I2C.c
 * 
 * Copyright (C) 2008-2013	Waltop International Corp. <waltopRD@waltop.com.tw>
 * 
 * History:
 * Copyright (c) 2011	Martin Chen <MartinChen@waltop.com.tw>
 * Copyright (c) 2012	Taylor Chuang <chuang.pochieh@gmail.com>
 * Copyright (c) 2012	Herman Han <HermanHan@waltop.com>
 * 
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
 
#ifndef __Waltop_I2C_H__
#define __Waltop_I2C_H__


/* Tablet characteristics */
#define WALTOP_MAX_X            5200    // min value is 0
#define WALTOP_MAX_Y            3000    // min value is 0
#define WALTOP_MAX_P		   	511    // min value is 0
#define WALTOP_MIN_P_TIPON      0       // min tips on pressure value

/* Pen related GPIO */
#define PEN_GPIO_IRQ			175
#define PEN_GPIO_RESET			173
//#define PEN_GPIO_PWR_EN			1	// Undefine this for always enable
//#define PEN_GPIO_FW_UPDATE		3	// Undefine this for always set low

/* Pen I2C pin number, MB's pin  */
#define PEN_GPIO_SDA_PIN		77
#define PEN_GPIO_SCL_PIN		78

struct waltop_platform_data
{
	__u16	x_max;
	__u16	y_max;
	__u16	p_max;
	__u16	p_minTipOn;
	int gpio_reset;
	int gpio_int;
    int gpio_pdct;
};


#endif /* __Waltop_I2C_H__ */
