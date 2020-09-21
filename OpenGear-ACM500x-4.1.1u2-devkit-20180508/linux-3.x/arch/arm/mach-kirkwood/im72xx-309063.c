
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/mdio-gpio.h>


#include "im72xx-309063.h"

/* The 16S+24E sub-board */
struct board_309063 {
	struct pci_dev *igb;
	struct pci_dev *xr;
	struct gpio_chip *xrgpio;
};

static struct mdio_gpio_platform_data board_309063_xr_mdio_data;

static struct platform_device board_309063_xr_mdio_device = {
	.name           = "mdio-gpio",
	.id             = 0,
	.dev.platform_data = &board_309063_xr_mdio_data
};

static int __init board_309063_init_mdio(struct board_309063 *board)
{
	struct platform_device *dev = &board_309063_xr_mdio_device;
	struct mdio_gpio_platform_data *mdio_data = dev_get_platdata(&dev->dev);
	int ret;

	/* Create a virtual mdio bus on the Exar's MPIO[8:10]  */
	mdio_data->mdc = board->xrgpio->base + 8;
	mdio_data->mdio= board->xrgpio->base + 10;
	mdio_data->mdo = board->xrgpio->base + 9;
	ret = platform_device_register(dev);
	if (ret) {
		printk(KERN_ERR "309063: cannot register mdio bus, %d", ret);
	}

	return ret;
}

/* Visit each PCI device to finds the 309063's i210 and xr */
static int __init board_309063_match_pci(struct pci_dev *dev, void *userdata)
{
	struct board_309063 *board = userdata;

	if (dev->vendor == 0x8086 && dev->device == 0x1537) {
		printk(KERN_DEBUG "309063: found i210 at %s\n", pci_name(dev));
		board->igb = dev;
	}
	if (dev->vendor == 0x13a8 && dev->device == 0x8358) {
		printk(KERN_DEBUG "309063: found xr at %s\n", pci_name(dev));
		board->xr = dev;
	}
	return 0;
}

/* Visit each GPIO controller to find the one on the xr */
static int __init board_309063_match_xrgpio(struct gpio_chip *chip, void *data)
{
	struct board_309063 *board = data;

	if (chip->dev == &board->xr->dev) {
		printk(KERN_INFO "309063: found xr gpios at %s\n", chip->label);
		return 1;
	}
	return 0;
}

static int __init board_309063_probe(struct board_309063 *board)
{
	struct pci_bus *bus;

	/* Scan PCI devices to find igb and xr */
	list_for_each_entry(bus, &pci_root_buses, node)
		pci_walk_bus(bus, board_309063_match_pci, board);
	if (!board->igb || !board->xr)
		return -ENODEV;

	/* Scan GPIO controllers */
	board->xrgpio = gpiochip_find(board, board_309063_match_xrgpio);
	if (!board->xrgpio) {
		printk(KERN_WARNING "309063: no gpiochip for dev=%p (%s)\n",
		    &board->xr->dev, dev_name(&board->xr->dev));
		return -ENODEV;
	}

	printk(KERN_INFO "309063: found 16S+24E board\n");
	return 0;
}

void __init board_309063_setup(void)
{
	struct board_309063 board;

	memset(&board, 0, sizeof board);

	if (board_309063_probe(&board) == 0) {
		board_309063_init_mdio(&board);
	}
}
