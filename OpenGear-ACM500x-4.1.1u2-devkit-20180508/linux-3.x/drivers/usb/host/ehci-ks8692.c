/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2000-2004 David Brownell <dbrownell@users.sourceforge.net>
 *
 * Bus Glue for Micrel Pegasus KSZ8692 EHCI
 *
 * Based on "ohci-ks8692.c" by Matt Porter <mporter@kernel.crashing.org>
 *
 * Modified for Micrel Pegasus KSZ8692 EHCI
 *  by David Choi <david.choi@micrel.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>
#include "ks8692.h"

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static void ks8692_start_ehc(struct platform_device *dev)
{

        u32 uReg;
#ifndef ks8692_write
#define ks8692_write    au_writel
#endif
#ifndef ks8692_read
#define ks8692_read     au_readl
#endif
	pr_debug(__FILE__ ": starting KS8692 EHCI USB Controller\n");

        //feature enable: reset to zero is to enable a feature
        uReg = ks8692_read(KS8692_FEATURE_INTER_CFG);              //

        ks8692_write(KS8692_FEATURE_INTER_CFG,(uReg & ~0x01) );        //bit0:host enable

        //Interrupt mode
        uReg = ks8692_read(KS8692_INT_CONTL1);             //
        ks8692_write(KS8692_INT_CONTL1,uReg & ~INT_USB_HOST_EHCI );  //IRQ mode

        //Interrupt enable
        uReg = ks8692_read(KS8692_INT_ENABLE1);            //
        ks8692_write(KS8692_INT_ENABLE1,uReg | INT_USB_HOST_EHCI );  //INT Enable

        //Interrupt status
        uReg = ks8692_read(KS8692_INT_STATUS1);            //
        ks8692_write(KS8692_INT_STATUS1,uReg & ~( INT_USB_HOST_EHCI ) ); //Clear INT

        //Interrupt priority
        uReg = ks8692_read(KS8692_INT_USB_PRIORITY);               //
        ks8692_write(KS8692_INT_USB_PRIORITY,uReg & ~0xfff );       //lowest priority

}

static void ks8692_stop_ehc(struct platform_device *dev)
{
	pr_debug(__FILE__ ": stopping KS8692 EHCI USB Controller\n");

}

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_ehci_ks8692_probe - initialize KS8692-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_ehci_ks8692_probe(const struct hc_driver *driver,
			  struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug("ehci_ks8692 resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
	}

	hcd = usb_create_hcd(driver, &dev->dev, "KS8692");
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("KS8692 EHCI:request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	//hcd->regs = VIO(hcd->rsrc_start);

	if (!hcd->regs) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err2;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0))
	ehci->regs = hcd->regs + HC_LENGTH(echi, ehci_readl(ehci, &ehci->caps->hc_capbase));
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
	ehci->regs = hcd->regs + HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
#else
	ehci->regs = hcd->regs + HC_LENGTH(READL(&ehci->caps->hc_capbase));
#endif

	/* cache this readonly data; minimize chip reads */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
#else
	ehci->hcs_params = READL(&ehci->caps->hcs_params);
#endif

	/* ehci_hcd_init(hcd_to_ehci(hcd)); */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
	retval = usb_add_hcd(hcd, dev->resource[1].start, IRQF_DISABLED);
#else
	retval = usb_add_hcd(hcd, dev->resource[1].start, SA_INTERRUPT );
#endif
	if (retval == 0)
	{
		ks8692_start_ehc(dev);
		return retval;
	}

	ks8692_stop_ehc(dev);
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_ehci_hcd_ks8692_remove - shutdown processing for KS8692-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_ks8692_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_ehci_ks8692_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	ks8692_stop_ehc(dev);
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_ks8692_hc_driver = {
	.description = hcd_name,
	.product_desc = KS8692_SOC_EHCI_NAME,
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
	.shutdown = ehci_shutdown,
#endif

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.hub_suspend = ehci_hub_suspend,
	.hub_resume = ehci_hub_resume,
#endif
};

/*-------------------------------------------------------------------------*/

static int ehci_hcd_ks8692_drv_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("In ehci_hcd_ks8692_drv_probe\n");

	if (usb_disabled())
		return -ENODEV;

	ret = usb_ehci_ks8692_probe(&ehci_ks8692_hc_driver, pdev);
	return ret;
}

static int ehci_hcd_ks8692_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_ehci_ks8692_remove(hcd, pdev);
	return 0;
}

 /*TBD*/
/*static int ehci_hcd_ks8692_drv_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	return 0;
}
static int ehci_hcd_ks8692_drv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	return 0;
}
*/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
MODULE_ALIAS("pegasus-ehci");
#endif

static struct platform_driver ehci_hcd_ks8692_driver = {
	.probe = ehci_hcd_ks8692_drv_probe,
	.remove = ehci_hcd_ks8692_drv_remove,
        /*.suspend      = ehci_hcd_ks8692_drv_suspend, */
        /*.resume       = ehci_hcd_ks8692_drv_resume, */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
	.shutdown = usb_hcd_platform_shutdown,
#endif
	.driver = {

/*
    [name] field here MUST matches with that of ks8692_device_ehci in
    linux/arch/mach-ks8692/devs.h. Otherwise this device will never
    be served.
*/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
                .name   = "pegasus-ehci",
#else
		.name	= "ks8692-ehci",
#endif
		.owner	= THIS_MODULE,
	},
};


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21))
static int __init ehci_hcd_ks8692_init(void)
{
	pr_debug(DRIVER_INFO " (KS8692)\n");

	return platform_driver_register(&ehci_hcd_ks8692_driver);
}

static void __exit ehci_hcd_ks8692_cleanup(void)
{
	platform_driver_unregister(&ehci_hcd_ks8692_driver);
}

module_init(ehci_hcd_ks8692_init);
module_exit(ehci_hcd_ks8692_cleanup);
#endif
