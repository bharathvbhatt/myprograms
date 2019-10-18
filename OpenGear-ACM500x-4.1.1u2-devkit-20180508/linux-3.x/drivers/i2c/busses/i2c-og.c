/*
 * Bitbanging I2C bus driver using the GPIO API with fixes for OG separate
 * RX/TX
 *
 * Copyright (C) 2010 Opengear Inc.
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-og-gpio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <mach/regs-gpio.h>

static volatile unsigned int *gpdatap = (volatile unsigned int *) (KS8695_GPIO_VA + KS8695_IOPD);
static volatile unsigned int *gpmodep = (volatile unsigned int *) (KS8695_GPIO_VA + KS8695_IOPM);

static void i2c_og_setdir(int pin, int state) {
	if (state) {
		/* Set the initial state to HIGH */ 
		*gpdatap |= (1 << pin);
		*gpmodep |= (1 << pin);
	} else {
		*gpmodep &= ~(1 << pin);
	}
}

static void i2c_og_setpin(int pin, int val) {
	if (val) {
		*gpdatap |= (1 << pin);
	} else {
		*gpdatap &= ~(1 << pin);
	}
}

static int i2c_og_getpin(int pin) {
	return (*gpdatap & (1 << pin)) ? 1 : 0;
}

static void i2c_og_gpio_setsda(void *data, int state) 
{
	struct i2c_og_gpio_platform_data *pdata = data;
	i2c_og_setpin(pdata->sdatx_pin, state);
}

static void i2c_og_gpio_setscl(void *data, int state)
{
	struct i2c_og_gpio_platform_data *pdata = data;
	i2c_og_setpin(pdata->scl_pin, state);
}

static int i2c_og_gpio_getsda(void *data)
{
	struct i2c_og_gpio_platform_data *pdata = data;
	return i2c_og_getpin(pdata->sdarx_pin);
}

static int i2c_og_gpio_getscl(void *data)
{
	struct i2c_og_gpio_platform_data *pdata = data;
	return i2c_og_getpin(pdata->scl_pin);
}

static int i2c_og_gpio_probe(struct platform_device *pdev)
{
	struct i2c_og_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;
	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
		goto err_alloc_bit_data;

	i2c_og_setdir(pdata->sdatx_pin, 1);
	bit_data->setsda = i2c_og_gpio_setsda;

	i2c_og_setdir(pdata->sdarx_pin, 0);
	bit_data->getsda = i2c_og_gpio_getsda;

	i2c_og_setdir(pdata->scl_pin, 1);
	bit_data->setscl = i2c_og_gpio_setscl;
	bit_data->getscl = i2c_og_gpio_getscl;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else
		bit_data->udelay = 5;			/* 100 kHz */

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-og-gpio%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev, "using pins %u (SDA_TX) %u (SDA_RX) and %u (SCL)\n",
		 pdata->sdatx_pin, pdata->sdarx_pin, pdata->scl_pin);

	return 0;

err_add_bus:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	return ret;
}

static int i2c_og_gpio_remove(struct platform_device *pdev)
{
	struct i2c_og_gpio_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

static struct platform_driver i2c_og_gpio_driver = {
	.driver		= {
		.name	= "i2c-og-gpio",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_og_gpio_probe,
	.remove		= i2c_og_gpio_remove,
};

static int __init i2c_og_gpio_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_og_gpio_driver);
	if (ret)
		printk(KERN_ERR "i2c-og-gpio: probe failed: %d\n", ret);

	return ret;
}
module_init(i2c_og_gpio_init);

static void __exit i2c_og_gpio_exit(void)
{
	platform_driver_unregister(&i2c_og_gpio_driver);
}
module_exit(i2c_og_gpio_exit);

MODULE_AUTHOR("Ken Wilson <ken.wilson@opengear.com>");
MODULE_DESCRIPTION("OG Specific bitbanging I2C driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-og-gpio");
