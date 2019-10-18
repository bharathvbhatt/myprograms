/*
 * i2c-gpio interface to platform code
 *
 * Copyright (C) 2010 Opengear Inc.
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_I2C_OG_GPIO_H
#define _LINUX_I2C_OG_GPIO_H

/**
 * struct i2c_og_gpio_platform_data - Platform-dependent data for i2c-gpio
 * @sdatx_pin: GPIO pin ID to use for SDA_TX
 * @sdarx_pin: GPIO pin ID to use for SDA_RX
 * @scl_pin: GPIO pin ID to use for SCL
 * @udelay: signal toggle delay. SCL frequency is (500 / udelay) kHz
 * @timeout: clock stretching timeout in jiffies. If the slave keeps
 *	SCL low for longer than this, the transfer will time out.
 */
struct i2c_og_gpio_platform_data {
	unsigned int	sdatx_pin;
	unsigned int	sdarx_pin;
	unsigned int	scl_pin;
	int		udelay;
	int		timeout;
};

#endif /* _LINUX_I2C_OG_GPIO_H */
