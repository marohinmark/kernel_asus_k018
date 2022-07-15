/*
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * */


#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>

#include <linux/spi/spi.h>
#include <linux/freezer.h>

#include "spca700xa.h"
#include "app_i2c_lib_icatch.h"


#define SPCA700XA_SPI_BITS_PER_WORD	8
#define SPCA700XA_CMD_NORMAL		0x55


struct spca700xa *isp;


struct spca700xa_spi_rd {
	struct spi_transfer	spi_xfer;
	u32			spi_tx;
	u32			spi_rx;
};

struct spca700xa {
	struct spi_device	*spi;
	struct spi_message  spi_read_msg;
	struct mutex		mutex;
	spinlock_t		lock;
};


static int spca700xa_cmd(struct spca700xa *isp, UINT8 *cmd, UINT32 length)
{
	UINT8 *tx = cmd;
	struct spi_transfer xfer = {
		.tx_buf		= tx,
		.len		= length,
		.bits_per_word	= 8,
	};
	struct spi_message msg;
	int error;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	error = spi_sync(isp->spi, &msg);
	if (error) {
		printk("%s: error\n", __func__);
		dev_err(&isp->spi->dev, "%s: failed, command: %x, error: %d\n",
			__func__, cmd, error);
		return error;
	}

	return 0;
}


static int spca700xa_cmd_byte(struct spca700xa *isp, u8 cmd)
{
	u8 tx = cmd;
	struct spi_transfer xfer = {
		.tx_buf		= &tx,
		.len		= 1,
		.bits_per_word	= 8,
	};
	struct spi_message msg;
	int error;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	error = spi_sync(isp->spi, &msg);
	if (error) {
		printk("%s: error\n", __func__);
		dev_err(&isp->spi->dev, "%s: failed, command: %x, error: %d\n",
			__func__, cmd, error);
		return error;
	}

	return 0;
}


static int spca700xa_write(struct spca700xa *isp, u8 reg, u16 value)
{
	u32 tx = ((reg) << 16) | value;
	struct spi_transfer xfer = {
		.tx_buf		= &tx,
		.len		= 1,
		.bits_per_word	= 8,
	};
	struct spi_message msg;
	int error;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	error = spi_sync(isp->spi, &msg);
	if (error) {
		dev_err(&isp->spi->dev,
			"%s: failed, register: %x, value: %x, error: %d\n",
			__func__, reg, value, error);
		return error;
	}

	return 0;
}

static void spca700xa_setup_read(struct spca700xa_spi_rd *rd, UINT32 ulTransByteCnt, bool last)
{
	memset(rd, 0, sizeof(*rd));

	rd->spi_xfer.tx_buf	   = &rd->spi_tx;
	rd->spi_xfer.rx_buf	   = &rd->spi_rx;
	rd->spi_xfer.len	   = ulTransByteCnt;
	rd->spi_xfer.bits_per_word = 8;
	rd->spi_xfer.cs_change	   = !last;
}

static int spca700xa_read(struct spca700xa *isp, UINT8 *value, UINT32 ulTransByteCnt)
{
        //struct spca700xa_spi_rd spi_rd;
	struct spi_message msg;
	int error;
        printk("In %s:\n", __func__);
        u8 *rx = value;

        //ASUS_BSP+++, for calibration
        struct spi_transfer xfer = {
               .rx_buf = rx,
               .len = ulTransByteCnt,
               .bits_per_word = 8,
        };
        //ASUS_BSP---, for calibration

	spi_message_init(&msg);
        spi_message_add_tail(&xfer, &msg);//ASUS_BSP+++, for calibration
        //spi_message_add_tail(&spi_rd.spi_xfer, &msg);

	error = spi_sync(isp->spi, &msg);
        printk("In spca700xa_read, Error is %d:\n", error);
        if (error) { //ASUS_BSP+++, for calibration
                printk("%s: error\n", __func__);
                dev_err(&isp->spi->dev, "%s: failed, command: %x, error: %d\n",
                __func__, value, error);
                return error;
        }

	return 0;
}


int spca700xa_SPI_write(UINT8 *ucStartAddr, UINT32 ulTransByteCnt)
{
	int i;

//	int size;
//	UINT8 pattern[5];

//	pattern[0]=0x55;
//	pattern[1]=0x02;
//	pattern[2]=0x03;
//	pattern[3]=0x04;
//	pattern[4]=0x05;

//	size=sizeof(pattern);

//	printk("%s: size=%d\n", __func__, size);
	printk("%s: ulTransByteCnt = %d\n", __func__, ulTransByteCnt);

	spca700xa_cmd(isp, ucStartAddr, ulTransByteCnt);
/*
	for (i=0;i<ulTransByteCnt;i++){
		spca700xa_cmd_byte(isp, ucStartAddr[i]);
	}
*/
/*
	for (i=0; i<10; i++) {
		spca700xa_cmd_byte(isp, pattern[0]);
	}
*/



	return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_write);

int spca700xa_SPI_read(UINT8 *ucStartAddr, UINT32 ulTransByteCnt)
{
	int i;
        printk("In %s: ulTransByteCnt = %d, FW Address: 0x%x\n", __func__, ulTransByteCnt, ucStartAddr);
	spca700xa_read(isp, ucStartAddr, ulTransByteCnt);
/*
	for (i = 0x1390; i < 0x13b0; i++) {
		printk("%s: FW Address for read [%d]: %x\n", __func__, i, ucStartAddr[i]);
	}
*/
	return 0;
}

EXPORT_SYMBOL(spca700xa_SPI_read);

static int spca700xa_probe(struct spi_device *spi)
{
	int error,i;


	printk("%s:\n", __func__);
	/* Set up SPI*/
	spi->bits_per_word = SPCA700XA_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	error = spi_setup(spi);
	if (error < 0) {
		dev_err(&spi->dev, "%s: SPI setup error %d\n",
			__func__, error);
		return error;
	}else{
		printk("Wesley SET SPI MODE OK");
	}
	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	isp->spi = spi;

	spi_set_drvdata(spi, isp);

	return 0;
}

static int spca700xa_remove(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_PM

static int spca700xa_suspend(struct spi_device *spi, pm_message_t state)
{
	return 0;
}

static int spca700xa_resume(struct spi_device *spi)
{
	return 0;
}

#else
#define spca700xa_suspend NULL
#define spca700xa_resume  NULL
#endif

static struct spi_driver spca700xa_driver = {
	.driver = {
		.name		= "spca700xa",
		.owner		= THIS_MODULE,
	},

	.probe		= spca700xa_probe,
	.remove		= spca700xa_remove,
	.suspend	= spca700xa_suspend,
	.resume		= spca700xa_resume,
};

static int __init spca700xa_init(void)
{
	return spi_register_driver(&spca700xa_driver);
}
module_init(spca700xa_init);

static void __exit spca700xa_exit(void)
{
	spi_unregister_driver(&spca700xa_driver);
}
module_exit(spca700xa_exit);

MODULE_DESCRIPTION("SPCA700XA driver");
MODULE_AUTHOR("ASUS SW3");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spca700xa");
