/****************************************************************************/

/*
 *	acm550x.c -- support code for the OpenGear/ACM550X boards
 *	(C) Copyright 2011,  Ken Wilson <ken.wilson@opengear.com>
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
#include <asm/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <mach/platform.h>
#include <linux/serial_8250.h>
#include <mach/ks8692_utils.h>

/*
 * On the ACM550x models, we have the following i2c devices
 * i2c_addr | chip	| description
 * 0x68     | M41T11  	| RTC
 * 0x48     | LM75      | Temp Sensor
 * 0x20	    | PCA9534	| IO Expander  
 */
static struct i2c_board_info __initdata acm550x_i2c_devices[] = {
	{
		I2C_BOARD_INFO("m41t11", 0x68),
	},
	{
		I2C_BOARD_INFO("lm75", 0x48),
	},
	{
		I2C_BOARD_INFO("pca9534", 0x38),
	},	
};

/* External IO Mapped UARTS */
/* Single UART for modem */
#define MODEM_UART_OFFSET 	0x10000
#define MODEM_UART_IO_SIZE 	0x0FFFF
#define MODEM_UART_INTERRUPT	(LOW_IRQS + KS8692_INT_EXT_INT2)
/* Quad UART for ports 5-8 */
#define QUAD_UART_1_OFFSET	0x20000
#define QUAD_UART_2_OFFSET	0x24000
#define QUAD_UART_3_OFFSET 	0x28000
#define QUAD_UART_4_OFFSET 	0x2C000
#define UARTS_TOTAL_SIZE	0x1FFFF
#define QUAD_UART_INTERRUPT	(LOW_IRQS + KS8692_INT_EXT_INT1)
#define UART_CLK (115200 * 16 * 2) // 3.686 MHz xtal for both chips
static struct resource og_extio_uart_resources[] = {
	{
		.start		= KS8692_EXTIO_BASE + MODEM_UART_OFFSET,
		.end		= KS8692_EXTIO_BASE + UARTS_TOTAL_SIZE,
		.flags		= IORESOURCE_MEM
	},
};

static struct plat_serial8250_port og_extio_uart_data[] = {
	{
		.mapbase	= KS8692_EXTIO_BASE + QUAD_UART_1_OFFSET,
		.membase	= (char*)( KS8692_EXTIO_VIRT_BASE + QUAD_UART_1_OFFSET),
		.irq		= QUAD_UART_INTERRUPT,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ,
		.iotype		= UPIO_MEM,	
		.regshift	= 2,
		.uartclk	= UART_CLK,
	},
	{
		.mapbase	= KS8692_EXTIO_BASE + QUAD_UART_2_OFFSET,
		.membase	= (char*)( KS8692_EXTIO_VIRT_BASE + QUAD_UART_2_OFFSET),
		.irq		= QUAD_UART_INTERRUPT,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ,
		.iotype		= UPIO_MEM,	
		.regshift	= 2,
		.uartclk	= UART_CLK,
	},
	{
		.mapbase	= KS8692_EXTIO_BASE + QUAD_UART_3_OFFSET,
		.membase	= (char*)( KS8692_EXTIO_VIRT_BASE + QUAD_UART_3_OFFSET),
		.irq		= QUAD_UART_INTERRUPT,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ,
		.iotype		= UPIO_MEM,	
		.regshift	= 2,
		.uartclk	= UART_CLK,
	},
	{
		.mapbase	= KS8692_EXTIO_BASE + QUAD_UART_4_OFFSET,
		.membase	= (char*)( KS8692_EXTIO_VIRT_BASE + QUAD_UART_4_OFFSET),
		.irq		= QUAD_UART_INTERRUPT,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ,
		.iotype		= UPIO_MEM,	
		.regshift	= 2,
		.uartclk	= UART_CLK,
	},
	{
		.mapbase	= KS8692_EXTIO_BASE + MODEM_UART_OFFSET,
		.membase	= (char*)( KS8692_EXTIO_VIRT_BASE + MODEM_UART_OFFSET),
		.irq		= MODEM_UART_INTERRUPT,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,	
		.regshift	= 2,
		.uartclk	= UART_CLK,
	},
	{ },
};

static struct platform_device og_extio_uart = {
	.name			= "serial8250",
	.id			= 0,
	.dev.platform_data	= og_extio_uart_data,
	.num_resources		= 1,
	.resource		= og_extio_uart_resources
};

/* 
 * IO Latch Control (LEDS, SIM card selection, RS422/485)
 *
 * bit range	| Devices
 * 0:5		| LED 1 -> 6
 * 6		| SIM Select
 * 7		| 3G PCIe Reset
 * 8:15		| 422/485 Ports 1:8 
 */

#define LED_BITMASK		0x3F
#define SIM_SELECT_BIT		0x40
#define CELL_PCIE_RESET		0x80
#define RS485_BITMASK		0xFF00
#define LATCH_VIO_ADDR		0xF0000000

static u16 io_latch;

#define SET_LATCH()	__raw_writew(io_latch, KS8692_EXTIO_VIRT_BASE)

void acm550x_set_leds(unsigned long bits) {
	bits &= LED_BITMASK;
	io_latch &= ~LED_BITMASK;
	io_latch |= bits;
	SET_LATCH();
}
 
void acm550x_set_sim(int sim) {
	u32 bits = 0;
	
	if (sim == 1) {
		bits = SIM_SELECT_BIT;
	}
	
	io_latch &= ~SIM_SELECT_BIT;
	io_latch |= bits;
	SET_LATCH();
}

int acm550x_get_sim_idx(void) {
	if (io_latch & SIM_SELECT_BIT)
		return 1;
	
	return 0;
}

static void acm550x_cell_reset_handler(unsigned long data) {
	static DEFINE_TIMER(acm550x_cell_reset_timer, acm550x_cell_reset_handler, 0, 1);
	/* This is a single shot timer that pulls the pcie reset pin high, then low again */

	if (data == 0) {
		io_latch |= CELL_PCIE_RESET;
		mod_timer(&acm550x_cell_reset_timer, jiffies + msecs_to_jiffies(100));
	} else {
		io_latch &= ~CELL_PCIE_RESET;
	}
	SET_LATCH();
	if (data == 0) {
		printk(KERN_ERR "Resetting 3G modem\n");
	}
}

/* Port num from 1 to 8 */
void acm550x_set_rs485(int port_num, int value) {
	io_latch &= ~(value << (port_num + 7));
	io_latch |= (value << (port_num + 7));
	SET_LATCH();
}

EXPORT_SYMBOL(acm550x_set_rs485);

/* 
 * RS232 Modem reset functions
 */

static void acm550x_modem_reset_handler(unsigned long data) {
	static DEFINE_TIMER(acm550x_modem_reset_timer, acm550x_modem_reset_handler, 0, 1);
	/* This is a single shot timer that pulls the modem reset pin low, then high again */

	if (data == 0) {
		/* Set GPIO 19 Low */
		u32 reg = KS8692_READ_REG(KS8692_GPIO_DATA);
		reg &= ~(1 << 19); 
		KS8692_WRITE_REG(KS8692_GPIO_DATA, reg);
		mod_timer(&acm550x_modem_reset_timer, jiffies + msecs_to_jiffies(400));
	} else {
		/* Reset it back to High */
		u32 reg = KS8692_READ_REG(KS8692_GPIO_DATA);
		reg |= (1 << 19); 
		KS8692_WRITE_REG(KS8692_GPIO_DATA, reg);
	}
	
	if (data == 0) {
		printk(KERN_ERR "Resetting internal modem\n");
	}
}

#define ERASE_BUTTON_INT (LOW_IRQS + KS8692_INT_EXT_INT3)

static int __init acm550x_init(void)
{

	u32 v;
	/* Initialise the IO Latch data to all off */
	io_latch = 0;
	/* Switch off the top 2 DDR clocks as they're not being used */
	KS8692_WRITE_REG(KS8692_DDR_MEM_CFG, 0x00030198);
	/* Switch off other un-used peripherals (PCI/SDIO/I2S) */
	v = KS8692_READ_REG(KS8692_FEATURE_INTER_CFG);
	v |= 0x2044;
	KS8692_WRITE_REG(KS8692_FEATURE_INTER_CFG, v); 	
	/* Make MDC/MDIO only clock during transactions */
	v = KS8692_READ_REG(KS8692_STA_CONF);
	v |= 0x20000;
	KS8692_WRITE_REG(KS8692_STA_CONF, v); 	
		
	/* Map in the address spaces for devices on the following chip selects:
	 * CS	| Device		| Address (width)	| Region Size 	
	 * 0	| 422/485 Latches	| 0x18000000 (16)	|	0xFFFF (64 K <- 2 latches (min address space))
	 * 1	| Modem 16550		| 0x18010000 (8)	|	0xFFFF (64 K  <- 1 UART ( min address space))
	 * 2	| Quad 16550		| 0x18020000 (8)	|	0xFFFF (64 K <- 2 UART (min address space))
	 */

	v = 0x000924; // Start at 0 (in ext addr range) end at 0xFFFF, slow timings.
	KS8692_WRITE_REG(KS8692_IO_CTRL0, v);
	v = 0x401924;	// Start at 10000 (in ext addr range) end at 0x1FFFF, slow timings.
	KS8692_WRITE_REG(KS8692_IO_CTRL1, v);
	v = 0x802924;	// Start at 20000 (in ext addr range) end at 0x2FFFF, slow timings.
	KS8692_WRITE_REG(KS8692_IO_CTRL2, v);
	v = 0xFFD00070; // Move flash to 0x19000000, and give it 48 meg of range (we wont use that)
	KS8692_WRITE_REG(KS8692_MEM_CTRL0, v);
	v = 0x30160001; // Switch on the ext io mappings, giving 16 bit databus for CS 0, 8 bit for 1 and 2
	KS8692_WRITE_REG(KS8692_MEM_GENERAL, v);
	/* FIXME: Speed up timing once we've tested */
	/* Setup the interrupts for the uarts */

	ks8692_util_enable_interrupt(MODEM_UART_INTERRUPT, 1);
	ks8692_util_enable_interrupt(QUAD_UART_INTERRUPT, 1);
	 
	/* Register the modem and quad uarts */
	platform_device_register(&og_extio_uart);
	/* Register the I2C devices */
	i2c_register_board_info(0, acm550x_i2c_devices,
				ARRAY_SIZE(acm550x_i2c_devices));
	
	/* Setup the interrupt for the erase button */
	ks8692_util_enable_interrupt(ERASE_BUTTON_INT, 1);
	irq_set_irq_type(ERASE_BUTTON_INT, IRQ_TYPE_EDGE_FALLING);

	/* Set GPIO 19 to high */
	u32 reg = KS8692_READ_REG(KS8692_GPIO_DATA);
	reg |= (1 << 19);
	KS8692_WRITE_REG(KS8692_GPIO_DATA, reg);
	
	/* Use GPIO 19 as Output (Modem Reset) */
	reg = KS8692_READ_REG(KS8692_GPIO_MODE);
	reg |= (1 << 19);
	KS8692_WRITE_REG(KS8692_GPIO_MODE, reg);
	
	/* Enable Panic on oops, and reboot on panic */
	panic_on_oops = 1;
	panic_timeout = 10;
	return 0;
}

arch_initcall(acm550x_init);

/*
 * We need a nice way of switching which SIM we use from userspace,
 * and for resetting the cellular modem
 * We expose sysfs attributes for this.
 */
static ssize_t attr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (strcmp(attr->attr.name, "sim_idx") == 0) {
		unsigned long sim = acm550x_get_sim_idx();
		return sprintf(buf, "%lu\n", sim); 
	} else if (strcmp(attr->attr.name, "cell_reset") == 0) {
		return sprintf(buf, "0\n");
	} else if (strcmp(attr->attr.name, "modem_reset") == 0) {
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
	if (strcmp(attr->attr.name, "sim_idx") == 0) {
		int tmp = strict_strtoul(instr, 0, &l);
		if (tmp)
			return tmp;

		acm550x_set_sim((l > 0 ? 1: 0)); 
	} else if (strcmp(attr->attr.name, "cell_reset") == 0) {
		int tmp = strict_strtoul(instr, 0, &l);
		if (tmp)
			return tmp;

		if (l == 1) {
			acm550x_cell_reset_handler(0);			
		}
	} else if (strcmp(attr->attr.name, "modem_reset") == 0) {
		int tmp = strict_strtoul(instr, 0, &l);
		if (tmp)
			return tmp;

		if (l == 1) {
			acm550x_modem_reset_handler(0);			
		}
	} else {
		bytes = -ENOENT;
	}
	
	return bytes;
}

static struct kobj_attribute sim_idx = 
	__ATTR(sim_idx, 0600, attr_show, attr_store);

static struct kobj_attribute cell_reset = 
	__ATTR(cell_reset, 0600, attr_show, attr_store);


static struct attribute *cell_attributes[] = {
	&sim_idx.attr,
	&cell_reset.attr,
	NULL
};

static struct attribute_group cell_attribute_group = {
	.name = "cellctl",
	.attrs = cell_attributes,
};

static int __init cellctl_init(void)
{
	/* Set up the sysfs group for cell control */
	return sysfs_create_group(kernel_kobj, &cell_attribute_group);
}

late_initcall(cellctl_init);


/* Modem Sysfs functions */
static struct kobj_attribute modem_reset = 
	__ATTR(modem_reset, 0600, attr_show, attr_store);


static struct attribute *modem_attributes[] = {
	&modem_reset.attr,
	NULL
};

static struct attribute_group modem_attribute_group = {
	.name = "modemctl",
	.attrs = modem_attributes,
};

static int __init modemctl_init(void)
{
	/* Set up the sysfs group for modem control */
	return sysfs_create_group(kernel_kobj, &modem_attribute_group);
}

late_initcall(modemctl_init);


