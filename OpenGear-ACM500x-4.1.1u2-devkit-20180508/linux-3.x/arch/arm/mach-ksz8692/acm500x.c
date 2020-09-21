/****************************************************************************/

/*
 *	acm500x.c -- support code for the OpenGear/ACM500X boards
 *
 * 	(C) Copyright 2009,  Greg Ungerer <greg.ungerer@opengear.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/****************************************************************************/

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <mach/platform.h>

static struct spi_board_info acm500x_spi_board_info[] __initdata = {
	{
		.modalias	= "lm70",
		.mode		= SPI_MODE_0 | SPI_3WIRE,
		.max_speed_hz	= 1000000,
		.bus_num	= 0,
		.chip_select	= 1,
	},	
};

static struct i2c_board_info acm500x_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("m41t11", 0x68)
	},
	{
		I2C_BOARD_INFO("pca9534", 0x38)
	},
	{	
		I2C_BOARD_INFO("pca9534", 0x39)
	}
	
};


static int __init acm500x_init(void)
{
	spi_register_board_info(acm500x_spi_board_info, ARRAY_SIZE(acm500x_spi_board_info));
	i2c_register_board_info(0, acm500x_i2c_board_info, ARRAY_SIZE(acm500x_i2c_board_info));
	KS8692_WRITE_REG(KS8692_DDR_MEM_CFG, 0x00010198);
	/* Enable Panic on oops, and reboot on panic */
	panic_on_oops = 1;
	panic_timeout = 10;
	return 0;
}

arch_initcall(acm500x_init);

/* USB Reset stuff */

/* 
 * On newer ACM500x PCAs, the USB power enable pins are hooked up to Pin 6 and 7 on the 1st
 * PCA9534 ioexpander. The support below controls pin 7, which is the one that the cell modem
 * riser card is connected to. Reset support is exposed via sysfs in the same way as the ACM5504-5
 * models
 */

/* Function prototypes exported by the PCA9534 driver */
void pca9534_setoutpin(u32 idx, u32 pin, u32 v);
void pca9534_setdir(u32 idx, u32 pin, u32 d);

#define CELLMODEM_POWER_PIN 7
/* 
 * This function is called to power the usb port back up 
 */
static void usb_reset_work(struct work_struct *work) {
	(void)work;

	pr_info("Reset finished\n");
	/* Reset it back to Hi-Z Low */
	pca9534_setdir(0, CELLMODEM_POWER_PIN, 1);
	pca9534_setoutpin(0, CELLMODEM_POWER_PIN, 0);
}

/* Because the power enable pin is hooked up to an i2c ioexpander, we can't use
 * jiffy timers, as they run in the interrupt context, and i2c operations need to sleep.
 * Instead, we use a delayed work queue
 */
static DECLARE_DELAYED_WORK(usb_reset_wq, usb_reset_work);
 
static void acm500x_usb_reset_handler(int l) {
	pr_info("Resetting cellmodem power\n");
	/* Set the pin as output, and pull it high for the reset period */
	pca9534_setdir(0, CELLMODEM_POWER_PIN, 0);
	pca9534_setoutpin(0, CELLMODEM_POWER_PIN, 1);
	schedule_delayed_work(&usb_reset_wq, msecs_to_jiffies(l * 1000)); 
}

/* USB Sysfs functions */
static ssize_t attr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (strcmp(attr->attr.name, "cell_reset") == 0) {
		return sprintf(buf, "0\n");
	}
	return -ENOENT;
}

static ssize_t attr_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *instr,
	size_t bytes) 
{
	unsigned long l;
	if (strcmp(attr->attr.name, "cell_reset") == 0) {
		int tmp = strict_strtoul(instr, 0, &l);
		if (tmp)
			return tmp;

		if (l > 0) {
			acm500x_usb_reset_handler(l);			
		}
	} else {
		bytes = -ENOENT;
	}
	
	return bytes;
}

static struct kobj_attribute cell_reset = 
	__ATTR(cell_reset, 0600, attr_show, attr_store);


static struct attribute *cell_attributes[] = {
	&cell_reset.attr,
	NULL
};

static struct attribute_group cell_attribute_group = {
	.name = "cellctl",
	.attrs = cell_attributes,
};

static int __init cellctl_init(void)
{
	/* Set up the sysfs group for modem control */
	return sysfs_create_group(kernel_kobj, &cell_attribute_group);
}

late_initcall(cellctl_init);

