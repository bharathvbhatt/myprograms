/*
 * arch/arm/mach-kirkwood/im72xx-setup.c
 *
 * Opengear IM72xx Setup
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
//#include <linux/spi/orion_spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/ledman.h>
#include <linux/cfax128_plat.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <mach/kirkwood.h>
#include <mach/im72xx.h>
#include "common.h"
#include "im72xx-309063.h"
#include "mpp.h"

#include <linux/platform_data/mmc-mvsdio.h>
#include <plat/pcie.h>

static struct mtd_partition im72xx_nor_parts[] = {
	{
		.name = "u-boot",
		.offset = 0,
		.size = SZ_256K
	}, {
		.name = "u-boot config block",
		.offset = MTDPART_OFS_NXTBLK ,
		.size = SZ_128K
	}, {
		.name = "config",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_2M
	}, {
		.name = "image #1",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_16M + SZ_4M,
	}, {
		.name = "image #2",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_16M + SZ_4M,
	}, {
		.name = "all",
		.offset = 0,
		.size = MTDPART_SIZ_FULL
	},
};

const struct flash_platform_data im72xx_flash = {
	.type		= "n25q512a1",
	.name		= "spi_flash",
	.parts		= im72xx_nor_parts,
	.nr_parts	= ARRAY_SIZE(im72xx_nor_parts),
};

struct spi_board_info __initdata im72xx_spi_slave_info[] = {
	{
		.modalias	= "n25q512a1",
		.platform_data	= &im72xx_flash,
		.irq		= -1,
		.max_speed_hz	= 50000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};

static int __init im72xx_register_flash(void)
{
	if (machine_is_im72xx()) {
		spi_register_board_info(im72xx_spi_slave_info,
					ARRAY_SIZE(im72xx_spi_slave_info));
		kirkwood_spi_init();
	}
	return 0;
}

static struct mvsdio_platform_data im72xx_mvsdio_data = {
	.gpio_card_detect	= -1,
};

static struct gpio_keys_button im72xx_keypad_buttons[] = {
	{
		.code		= KEY_UP,
		.gpio		= GPIO_KEY_4,
		.desc		= "Keypad Up",
		.active_low	= 1,
	},
	{
		.code		= KEY_DOWN,
		.gpio		= GPIO_KEY_3,
		.desc		= "Keypad Down",
		.active_low	= 1,
	},
	{
		.code		= KEY_ENTER,
		.gpio		= GPIO_KEY_2,
		.desc		= "Keypad OK",
		.active_low	= 1,
	},
	{
		.code		= KEY_BACKSPACE,
		.gpio		= GPIO_KEY_1,
		.desc		= "Keypad Back",
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data im72xx_keypad_button_data = {
	.buttons	= im72xx_keypad_buttons,
	.nbuttons	= ARRAY_SIZE(im72xx_keypad_buttons),
	.poll_interval  = 100
};

static struct platform_device im72xx_keypad_button_device = {
	.name		= "gpio-keys-polled",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &im72xx_keypad_button_data,
	}
};

static int im72xx_tca6424_setup(struct i2c_client *client,
			       unsigned gpio_base, unsigned ngpio,
			       void *context)
{
	int n;
	/* Initialise all our GPIOs to output first, then we'll re-set them up with correct direction*/
	for (n = 0; n < ngpio; n++) {
		gpio_request_one(gpio_base + n, 0, "N/C - Output");
		gpio_free(gpio_base + n); 
	}

	for (n = 0; n < ARRAY_SIZE(im72xx_tca6424_gpio_settings); ++n) {
		if (im72xx_tca6424_gpio_settings[n].flags == IM72XX_GPIO_FLAG_NC) 
			continue;

		gpio_request_one(gpio_base + n, 
				(im72xx_tca6424_gpio_settings[n].flags | IM72XX_GPIO_FLAG_INVERTED ? GPIOF_DIR_IN : 0),
					im72xx_tca6424_gpio_settings[n].name);
		if (im72xx_tca6424_gpio_settings[n].flags & IM72XX_GPIO_FLAG_INPUT) {
				gpio_direction_input(gpio_base + n);
		} 
		if (im72xx_tca6424_gpio_settings[n].flags & IM72XX_GPIO_FLAG_OUTPUT) {	
				gpio_direction_output(gpio_base + n, 0);
		}
		if (im72xx_tca6424_gpio_settings[n].flags & IM72XX_GPIO_FLAG_OUTPUT_HIGH) {	
				gpio_direction_output(gpio_base + n, 1);
		}
	}

	/* Export the SFP Selector as output only */
	gpio_export(GPIO_I2C_SFP_SELECT, 0);
	
#ifdef CONFIG_LEDMAN
	{
		extern void ledman_leds_initialised(void);
		ledman_leds_initialised(); 
	}
#endif
	/* Register the keypad now too */
	platform_device_register(&im72xx_keypad_button_device);
	return 0;
}

static int im72xx_tca6424_sfp_setup(struct i2c_client *client,
			       unsigned base, unsigned ngpio,
			       void *context) {
	struct gpio gpios[] = {
		{ base +  0, GPIOF_DIR_IN,	"TX_FAULT_1" },
		{ base +  1, 0,			"TX_DISABLE_1" },
		{ base +  2, GPIOF_DIR_IN,	"MODULE_PRESENT_1" },
		{ base +  3, 0,			"RATE_SELECT_1" },
		{ base +  4, GPIOF_DIR_IN,	"LOSS_SIG_1" },
		{ base +  5, GPIOF_DIR_IN,	"READY_1" },
		{ base +  8, GPIOF_DIR_IN,	"TX_FAULT_2" },
		{ base +  9, 0,			"TX_DISABLE_2" },
		{ base + 10, GPIOF_DIR_IN,	"MODULE_PRESENT_2" },
		{ base + 11, 0,			"RATE_SELECT_2" },
		{ base + 12, GPIOF_DIR_IN,	"LOSS_SIG_2" },
		{ base + 13, GPIOF_DIR_IN,	"READY_2" },
	};
	int ret;

	ret = gpio_request_array(gpios, ARRAY_SIZE(gpios));
	if (ret)
		return ret;
	gpio_free_array(gpios, ARRAY_SIZE(gpios));
	return ret;
}

static struct pca953x_platform_data im72xx_i2c_tca6424_platdata[] = {
	{
		.gpio_base	= IM72XX_I2C_GPIO_BASE_23,
        .irq_base   = -1,
		.setup		= im72xx_tca6424_sfp_setup,
	},
	{
		.gpio_base	= IM72XX_I2C_GPIO_BASE_22,
		.irq_base	= -1, 
		.setup		= im72xx_tca6424_setup,
	}
};

static struct i2c_board_info __initdata im72xx_i2c_bus0[] __initdata = {
	{
		I2C_BOARD_INFO("tca6424", 0x23),
		.platform_data = &im72xx_i2c_tca6424_platdata[0],
	},
	{
		I2C_BOARD_INFO("tca6424", 0x22),
		.platform_data = &im72xx_i2c_tca6424_platdata[1],
	}
};

subsys_initcall(im72xx_register_flash);

static struct mv643xx_eth_platform_data im72xx_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0x0),
};

static struct mv643xx_eth_platform_data im72xx_ge01_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0x1),
};


static struct cfax128_platform_data im72xx_lcd_data = {
	.ctrl_chip = {
		.a0 = 11,
		.e = 49,
		.rw = 48,
		.cs1 = 46,
		.data_bus = {38, 39, 40, 41, 42, 43, 44, 45},
	},
};

static struct platform_device im72xx_lcd_device = {
	.name 	= "cfax128fb",
	.id 	=  -1,
	.dev 	= {
			.platform_data = &im72xx_lcd_data,
	},
	.num_resources = 0,
};

static unsigned int im72xx_mpp_config[] __initdata = {
		MPP0_SPI_SCn,
		MPP1_SPI_MOSI,
		MPP2_SPI_SCK,
		MPP3_SPI_MISO,
		MPP4_UART0_RXD,
		MPP5_UART0_TXD,
		MPP6_SYSRST_OUTn,
		MPP7_GPO,	/* Watchdog Twiddle */
		MPP8_TW0_SDA,
		MPP9_TW0_SCK,
		MPP10_GPO, /* LCD Reset */
		MPP11_GPIO, /* LCD A0 */
		MPP12_SD_CLK,
		MPP13_SD_CMD,
		MPP14_SD_D0,
		MPP15_SD_D1,
		MPP16_SD_D2,
		MPP17_SD_D3,
		MPP18_GPO, /* USB A 5V Enable */
		MPP19_GPO, /* USB B 5V Enable */
		MPP20_GE1_TXD0,
		MPP21_GE1_TXD1,
		MPP22_GE1_TXD2,
		MPP23_GE1_TXD3,
		MPP24_GE1_RXD0,
		MPP25_GE1_RXD1,
		MPP26_GE1_RXD2,
		MPP27_GE1_RXD3,
		MPP28_GPIO, /* Card 2 3.3v En */
		MPP29_GPIO, /* Card 1 3.3v En */
		MPP30_GE1_RXCTL,
		MPP31_GE1_RXCLK,
		MPP32_GE1_TCLKOUT,
		MPP33_GE1_TXCTL,
		MPP34_GPIO, /* PEX INT A */
		MPP35_GPIO, /* IO Expander INT */
		MPP36_TW1_SDA, /* TWI DATA */
		MPP37_TW1_SCK, /* TWI Clock */
		MPP38_GPIO,
		MPP39_GPIO,
		MPP40_GPIO,
		MPP41_GPIO,
		MPP42_GPIO,
		MPP43_GPIO,
		MPP44_GPIO,
		MPP45_GPIO,
		MPP46_GPIO, 	/* M_RLED */
		MPP47_GPIO,	/* M_GLED */
		MPP48_GPIO,	/* B_RLED */
		MPP49_GPO,	/* B_GLED */
		0
};

#ifdef CONFIG_SATA_MV
/* if you enable the driver assume you want it to work */
static struct mv_sata_platform_data im72xx_sata_data = {
	.n_ports	= 1,
};
#endif

#if 0
static void im72xx_junction_temp_checker(unsigned long data)
{
	static DEFINE_TIMER(im72xx_junction_temp_timer, im72xx_junction_temp_checker, 0, 0);
	u32 reg;
	/*
	 * This routine is called every 5 seconds to report the junction temperature
	 */	

	reg = readl(KIRKWOOD_REGS_VIRT_BASE | 0x10078);
	{	
		u32 temp = ((reg >> 10) & 0x1FF);
		temp = ((322 - temp) * 1000) / 1362;
		printk(KERN_ERR "CPU Junction Temp %d deg C\n", temp);
	}
	mod_timer(&im72xx_junction_temp_timer, jiffies + (5 * HZ));
}
#endif

static void __init im72xx_gpio_output(int pin, char *name, int val)
{
	if (gpio_request(pin, name) != 0 ||
			gpio_direction_output(pin, val) != 0)
		printk(KERN_ERR "im72xx: can't set up output GPIO %d (%s)\n",
				pin, name);
}

static void __init im72xx_gpio_input(int pin, char *name)
{
	if (gpio_request(pin, name) != 0 || gpio_direction_input(pin) != 0)
		printk(KERN_ERR "im72xx: can't set up input GPIO %d (%s)\n", pin, name);
}

static void __init im72xx_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();
	kirkwood_mpp_conf(im72xx_mpp_config);
	kirkwood_uart0_init();
	
	im72xx_gpio_output(7,  "Watchdog", 0);
	im72xx_gpio_output(18, "USB A 5V Enable", 1);
	im72xx_gpio_output(19, "USB B 5V Enable", 1);
	im72xx_gpio_output(28, "Card 2 3.3v Enable", 1);
	im72xx_gpio_output(29, "Card 1 3.3v Enable", 1);
	im72xx_gpio_input (34, "PEX INT A");
	im72xx_gpio_input (35, "IO Expander INT");
	im72xx_gpio_output(47, "PCIe Reset", 1);

	/* No interrupts for the io expanders */
	im72xx_i2c_bus0[1].irq = -1;
	im72xx_i2c_bus0[0].irq = -1;

	kirkwood_pcie_init(KW_PCIE0 | KW_PCIE1);	
	kirkwood_i2c_init();
	
	/* Initialise the i2c peripherals */
	i2c_register_board_info(0, im72xx_i2c_bus0,
				ARRAY_SIZE(im72xx_i2c_bus0));
       /* Initialise SDIO */
       kirkwood_sdio_init(&im72xx_mvsdio_data);
	/* Initialise LCD */
	platform_device_register(&im72xx_lcd_device);

	kirkwood_ehci_init();

#ifdef CONFIG_SATA_MV
	/* if you enable the driver assume you want it to work */
	kirkwood_sata_init(&im72xx_sata_data);
#endif

	kirkwood_ge00_init(&im72xx_ge00_data);
	kirkwood_ge01_init(&im72xx_ge01_data);

	/* turn on PHY polling for port 0 */
	writel(readl(KIRKWOOD_REGS_VIRT_BASE + 0x720b0) | 0x2,
			KIRKWOOD_REGS_VIRT_BASE + 0x720b0);
	/* turn on PHY polling for port 1 */
	writel(readl(KIRKWOOD_REGS_VIRT_BASE + 0x760b0) | 0x2,
			KIRKWOOD_REGS_VIRT_BASE + 0x760b0);

#if 0
	im72xx_junction_temp_checker(0);
#endif
}

static void __init im72xx_init_late(void)
{
	/* Look for and set up the 16S+24E sub-board */
	board_309063_setup();
}

#if 0
static int __init im72xx_pci_init(void)
{
	if (machine_is_im72xx()) {
		orion_pcie_reset((void __iomem *)PCIE_VIRT_BASE);
		orion_pcie_reset((void __iomem *)PCIE1_VIRT_BASE);
		kirkwood_pcie_init(KW_PCIE0 | KW_PCIE1);	
	}
	return 0;
}
//subsys_initcall(im72xx_pci_init);
#endif

MACHINE_START(IM72XX, "Opengear IM72xx")
	/* Maintainer: Ken Wilson <ken.wilson@opengear.com> */
	.atag_offset  = 0x00000100,
	.init_machine = im72xx_init,
	.map_io       = kirkwood_map_io,
	.init_early   = kirkwood_init_early,
	.init_irq     = kirkwood_init_irq,
	.init_time   = &kirkwood_timer_init,
	.init_late    = im72xx_init_late,
	.restart      = kirkwood_restart,
MACHINE_END
