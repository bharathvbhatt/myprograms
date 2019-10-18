/*
 * board-og.c -- support for the OpenGear KS8695 based boards.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/usb/sl811.h>
#include <asm/setup.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/devices.h>
#include <mach/regs-mem.h>
#include <mach/regs-gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-og-gpio.h>
#include <mach/gpio-ks8695.h>
#include "generic.h"


#ifdef CONFIG_MTD_SNAPARM_BANK
/*
 * Go ahead and disable all the flash and chip select regions here.
 * The MTD map driver can re-map as required for bank selection.
 */
void __init ks8695_early_mem_init(void)
{
	u32 v;

	printk("NOTICE: remapping memory and chip-select regions\n");

	v = __raw_readl(KS8695_MEM_VA + KS8695_ERGCON);
	v &= ~0x000f000f; 
	__raw_writel(v, KS8695_MEM_VA + KS8695_ERGCON);

	v = __raw_readl(KS8695_MEM_VA + KS8695_SDCON0);
	v = (v & ~0xffc00000) | 0xdfc00000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_SDCON0);
}

/*
 * Complete hack to modify the 48M memory size supplied in the boot
 * args by the boot loader to be 56M. We could just modify the memory
 * size in kernel, but the users are less confused when they see that
 * the command line actually has 56M on it.
 */
void __init ks8695_fix_command_line(char *from)
{
	int i;

	for (i = 0; (i < COMMAND_LINE_SIZE-7); i++, from++) {
		if (*from == '\0')
			break;
		if (memcmp(from, "mem=48M", 7) == 0) {
			printk("NOTICE: changing command line RAM size to 56MB\n");
			from[4] = '5';
			from[5] = '6';
		}
	}
}
#endif /* CONFIG_MTD_SNAPARM_BANK */

#ifdef CONFIG_PCI
static int og_pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
#ifdef CONFIG_MACH_IM4004
	if (slot == 8)
		return KS8695_IRQ_EXTERN1;
#endif
	return KS8695_IRQ_EXTERN0;
}

static struct ks8695_pci_cfg __initdata og_pci = {
	.mode		= KS8695_MODE_PCI,
	.map_irq	= og_pci_map_irq,
};

static void __init og_register_pci(void)
{
	/* Initialise the GPIO lines for interrupt mode */
	ks8695_gpio_interrupt(KS8695_GPIO_0, IRQ_TYPE_LEVEL_LOW);

#ifdef CONFIG_MACH_IM4004
	/* Cardbus Slot */
	ks8695_gpio_interrupt(KS8695_GPIO_1, IRQ_TYPE_LEVEL_LOW);
#endif

#ifdef CONFIG_PCI
	ks8695_init_pci(&og_pci);
#endif
}

/*
 * The PCI bus reset is driven by a dedicated GPIO line. Toggle it here
 * and bring the PCI bus out of reset.
 */
static void __init og_pci_bus_reset(void)
{
#define	GPIOMODE (KS8695_GPIO_VA + KS8695_IOPM)
#define	GPIODATA (KS8695_GPIO_VA + KS8695_IOPD)
#if defined(CONFIG_MACH_CM4008) || defined(CONFIG_MACH_CM41xx)
#define	GPIO_PCIRESET	0x2
#endif
#if defined(CONFIG_MACH_IM4004)
#define	GPIO_PCIRESET	0x4
#endif
#if defined(CONFIG_MACH_IM42xx)
#define	GPIO_PCIRESET	0x10
#endif

        /* Enable PCI reset line as output */
	__raw_writel(__raw_readl(GPIOMODE) | GPIO_PCIRESET, GPIOMODE);
	__raw_writel(__raw_readl(GPIODATA) & ~GPIO_PCIRESET, GPIODATA);
        mdelay(100);
	__raw_writel(__raw_readl(GPIODATA) | GPIO_PCIRESET, GPIODATA);
        mdelay(100);
}

#ifdef CONFIG_MACH_IM4004
void im4004_cardbus_fixup(struct pci_dev *dev)
{
	static int once = 0;
	u32 scr, mfr, ba;
	u16 bcr;
	u8 dc;

	if (once)
		return;
	once = 1;

	printk(KERN_WARNING "PCI1510: controller fixup...\n");

	/* Cardbus slot into reset state */
	bcr = 0x0340;
	pci_write_config_word(dev, 0x3e, bcr);

	/* Enable MFUNC as interrupt source for slot */
	scr = 0x08449060;
	pci_write_config_dword(dev, 0x80, scr);

	mfr = 0x00001002;
	pci_write_config_dword(dev, 0x8c, mfr);

	/* Serialized interrupts (must be done after 8C) */
	dc = 0x66;
	pci_write_config_byte(dev, 0x92, dc);

	ba = 0;
	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &ba);
#if 0
	if (ba) {
		writel(0, ba + 0x4);
		writel(0, ba + 0x10);
	}
#endif
}
#endif /* IM4004 */
#endif /* CONFIG_PCI */


#ifdef S8250_PHYS

static struct resource og_uart_resources[] = {
	{
		.start		= S8250_VIRT,
		.end		= S8250_VIRT + S8250_SIZE,
		.flags		= IORESOURCE_MEM
	},
};

static struct plat_serial8250_port og_uart_data[] = {
	{
		.mapbase	= S8250_VIRT,
		.membase	= (char*) S8250_VIRT,
		.irq		= S8250_IRQ,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM_KS8695,	
		.regshift	= 2,
		.uartclk	= 115200 * 16,
	},
	{ },
};

static struct platform_device og_uart = {
	.name			= "serial8250",
	.id			= 0,
	.dev.platform_data	= og_uart_data,
	.num_resources		= 1,
	.resource		= og_uart_resources
};

#endif /* S8250_PHYS */


#ifdef CONFIG_MACH_IM42xx
/* 
 * Platform setup for the GPIO driven i2c driver
 */
static struct i2c_og_gpio_platform_data im42xx_i2c_device_platdata = {
	.sdatx_pin      = 6,
	.sdarx_pin		= 7,
	.scl_pin        = 8,
	.udelay         = 40,
};

static struct platform_device im42xx_i2c_device = {
	.name           = "i2c-og-gpio",
	.id             = -1,
	.num_resources  = 0,
	.resource       = NULL,
	.dev            = {
		.platform_data  = &im42xx_i2c_device_platdata,
	},
};

/*
 * Setup for the RTC 
 */
static struct i2c_board_info im42xx_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("m41t11", 0x68)
	}
};
/*
 * Platform setup for the SL811 (non-PCI) USB host device.
 */
static struct resource og_sl811_hcd_resources[] = {
        {
                .start = 0x03fe0100,
                .end = 0x03fe0100,
                .flags = IORESOURCE_MEM,
        }, {
                .start = 0x03fe0104,
                .end = 0x03fe0104,
                .flags = IORESOURCE_MEM,
        }, {
                .start = 3,
                .end = 3,
                .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
        },
};

static struct sl811_platform_data og_sl811_priv = {
        .potpg = 10,
        .power = 250,       /* == 500mA */
};

static struct platform_device og_sl811_hcd_device = {
        .name = "sl811-hcd",
        .id = 0,
        .dev = {
                .platform_data = &og_sl811_priv,
        },
        .num_resources = ARRAY_SIZE(og_sl811_hcd_resources),
        .resource = og_sl811_hcd_resources,
};

/*
 * Map in the extcs2 region. The stand alone 16c550 and the sl811 USB
 * controller both live of this region. Also enable their interrupt lines.
 */
static void __init og_im42xx_init(void)
{
	u32 v;

#ifndef CONFIG_TL_REBRAND
	printk("IM42XX: mapping external peripherals\n");
#else
	printk("B096: mapping external peripherals\n");
#endif

	/* Init the first bank of flash - the uart driver may need to do some reads here */
	v = __raw_readl(KS8695_MEM_VA + KS8695_ROMCON0);
	v = (v & 0x00000fff) | 0xeff80000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ROMCON0);

	v = __raw_readl(KS8695_MEM_VA + KS8695_ROMCON1);
	v = (v & 0x00000fff) | 0xeff80000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ROMCON1);

	v = __raw_readl(KS8695_MEM_VA + KS8695_EXTACON0);
	v = (v & 0x00000fff) | 0xeff80000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_EXTACON0);

	v = __raw_readl(KS8695_MEM_VA + KS8695_EXTACON1);
	v = (v & 0x00000fff) | 0xeff80000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_EXTACON1);
	
	/* Map the UART and USB Controller */ 
	__raw_writel(0xffbfe000, KS8695_MEM_VA + KS8695_EXTACON2);

	v = __raw_readl(KS8695_MEM_VA + KS8695_ERGCON);
	v = (v & 0xffcfffff) | 0x00300000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ERGCON);


	
	/* External IRQ1 for the USB host controller */
	ks8695_gpio_interrupt(KS8695_GPIO_1, IRQ_TYPE_LEVEL_HIGH);

	/* External IRQ2 for the UART */
	ks8695_gpio_interrupt(KS8695_GPIO_2, IRQ_TYPE_LEVEL_HIGH);
	/* Register the RTC */
	i2c_register_board_info(0, im42xx_i2c_board_info, ARRAY_SIZE(im42xx_i2c_board_info));
}
#endif /* CONFIG_MACH_IM42xx */

static struct platform_device *og_devices[] __initdata = {
#ifdef S8250_PHYS
	&og_uart,
#endif
#ifdef CONFIG_MACH_IM42xx
	&og_sl811_hcd_device,
	&im42xx_i2c_device,
#endif
};

/*
 * We can adjust the boot loader chip-select timings to improve
 * performace a bit. The big win is in winding the TMULT setting back.
 * We don't seem to need to play much with the fine detail of the
 * cycle shape on the EXT settings. We are not changing the mapping
 * address regions here either, only the timing values.
 */
static void __init og_adjust_cs_timing(void)
{
	u32 v;

	/* Set TMULT to 1 from the slow default setting of 3 */
	v = __raw_readl(KS8695_MEM_VA + KS8695_ERGCON);
	v = (v & 0xcfffffff) | 0x10000000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ERGCON);

	v = __raw_readl(KS8695_MEM_VA + KS8695_ROMCON0);
	v = (v & 0xfffff000) | 0x00000070;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ROMCON0);

	v = __raw_readl(KS8695_MEM_VA + KS8695_ROMCON1);
	v = (v & 0xfffff000) | 0x00000070;
	__raw_writel(v, KS8695_MEM_VA + KS8695_ROMCON1);

	v = __raw_readl(KS8695_MEM_VA + KS8695_EXTACON0);
	v = (v & 0xfffff000) | 0x00000000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_EXTACON0);

	v = __raw_readl(KS8695_MEM_VA + KS8695_EXTACON1);
	v = (v & 0xfffff000) | 0x00000000;
	__raw_writel(v, KS8695_MEM_VA + KS8695_EXTACON1);
}

static void __init og_init(void)
{
#ifdef CONFIG_PCI
	og_pci_bus_reset();
	og_register_pci();
#endif
#ifdef CONFIG_MACH_CM4002
	ks8695_gpio_interrupt(KS8695_GPIO_1, IRQ_TYPE_LEVEL_HIGH);
#endif
	og_adjust_cs_timing();
#ifdef CONFIG_MACH_IM42xx
	og_im42xx_init();
#endif
#ifdef S8250_PHYS
	platform_add_devices(og_devices, ARRAY_SIZE(og_devices));
#endif
	/* Enable Panic on Oops and reboot on Oops */
	panic_on_oops = 1;
	panic_timeout = 10;

	/* Add devices */
#ifdef CONFIG_MACH_IM4004
	ks8695_add_device_wan();	/* eth0 = WAN */
	ks8695_add_device_lan();	/* eth1 = LAN */

#else
	/* Reversed on IM42xx and CM41xx */
	ks8695_add_device_lan();	/* eth0 = LAN */
	ks8695_add_device_wan();	/* eth1 = WAN */
#endif
}

extern  void __init ks8695_timer_init(void);
#ifdef CONFIG_MACH_CM4002
MACHINE_START(CM4002, "OpenGear/CM4002")
	/* OpenGear Inc. */
	.atag_offset	= 0x100,
	.map_io		= ks8695_map_io,
	.init_irq	= ks8695_init_irq,
	.init_machine	= og_init,
	.init_time	= ks8695_timer_init,
	.restart        = ks8695_restart,
MACHINE_END
#endif

#ifdef CONFIG_MACH_CM4008
MACHINE_START(CM4008, "OpenGear/CM4008")
	/* OpenGear Inc. */
	.atag_offset	= 0x100,
	.map_io		= ks8695_map_io,
	.init_irq	= ks8695_init_irq,
	.init_machine	= og_init,
	.init_time	= ks8695_timer_init,
	.restart        = ks8695_restart,
MACHINE_END
#endif

#ifdef CONFIG_MACH_CM41xx
MACHINE_START(CM41XX, "OpenGear/CM41xx")
	/* OpenGear Inc. */
	.atag_offset	= 0x100,
	.map_io		= ks8695_map_io,
	.init_irq	= ks8695_init_irq,
	.init_machine	= og_init,
	.init_time	= ks8695_timer_init,
	.restart        = ks8695_restart,
MACHINE_END
#endif

#ifdef CONFIG_MACH_IM42xx
#ifndef CONFIG_TL_REBRAND
MACHINE_START(IM42XX, "OpenGear/IM42xx")
#else
MACHINE_START(IM42XX, "TrippLite/B096")
#endif
	/* OpenGear Inc. */
	.atag_offset	= 0x100,
	.map_io		= ks8695_map_io,
	.init_irq	= ks8695_init_irq,
	.init_machine	= og_init,
	.init_time	= ks8695_timer_init,
	.restart        = ks8695_restart,
MACHINE_END
#endif

#ifdef CONFIG_MACH_IM4004
MACHINE_START(IM4004, "OpenGear/IM4004")
	/* OpenGear Inc. */
	.atag_offset	= 0x100,
	.map_io		= ks8695_map_io,
	.init_irq	= ks8695_init_irq,
	.init_machine	= og_init,
	.init_time	= ks8695_timer_init,
	.restart        = ks8695_restart,
MACHINE_END
#endif

