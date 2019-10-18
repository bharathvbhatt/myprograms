/*
 *    Filename: cfag12864bfb.c
 *     Version: 0.1.0
 * Description: cfag12864b LCD framebuffer driver
 *     License: GPLv2
 *     Depends: cfag12864b
 *
 *      Author: Copyright (C) Miguel Ojeda Sandonis
 *        Date: 2006-10-31
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/nt7534.h>
#include <linux/cfax128.h>
#include <linux/cfax128_plat.h>

#define bit(n) (((unsigned char)1)<<(n))
#define CFAX128FB_NAME "cfax128fb"

/* Frame buffer structures */

static struct fb_fix_screeninfo cfax128fb_fix = {
	.id = "cfax12864",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_MONO10,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = CFAX128_WIDTH / 8,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo cfax128fb_var = {
	.xres = CFAX128_WIDTH,
	.yres = CFAX128_HEIGHT,
	.xres_virtual = CFAX128_WIDTH,
	.yres_virtual = CFAX128_HEIGHT,
	.bits_per_pixel = 1,
	.red = { 0, 1, 0 },
      	.green = { 0, 1, 0 },
      	.blue = { 0, 1, 0 },
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.vmode = FB_VMODE_NONINTERLACED,
};

static int cfax128fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return vm_insert_page(vma, vma->vm_start,
		virt_to_page(cfax128_buffer));
}

static struct fb_ops cfax128fb_ops = {
	.owner = THIS_MODULE,
	.fb_read = fb_sys_read,
	.fb_write = fb_sys_write,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
	.fb_mmap = cfax128fb_mmap,
};

static unsigned int cfax128_start_line = 2;

static unsigned int cfax128_rate = CONFIG_CFAX128_RATE;

unsigned int cfax128_getrate(void)
{
	return cfax128_rate;
}

static void cfax128_clear(void)
{
	int i, j;
	for (i = 0; i <= CFAX128_PAGES; i++) {
		nt7534_page(i);
		nt7534_column_address(0);
		for (j = 0; j < 132; j++) {
			nt7534_write_data(0x0);
		}
	}
	nt7534_page(8);
	nt7534_column_address(0);;
	for (j = 0; j < 132; j++) {
		nt7534_write_data(0x01);
	}
	
}


unsigned char *cfax128_buffer;
static unsigned char *cfax128_cache;
static DEFINE_MUTEX(cfax128_mutex);
static unsigned char cfax128_updating;
static void cfax128_update(struct work_struct *delayed_work);
static struct workqueue_struct *cfax128_workqueue;
static DECLARE_DELAYED_WORK(cfax128_work, cfax128_update);

static void cfax128_queue(void)
{
	queue_delayed_work(cfax128_workqueue, &cfax128_work,
		HZ / cfax128_rate);
}

unsigned char cfax128_enable(void)
{
	unsigned char ret;

	mutex_lock(&cfax128_mutex);

	if (!cfax128_updating) {
		cfax128_updating = 1;
		cfax128_queue();
		ret = 0;
	} else
		ret = 1;

	mutex_unlock(&cfax128_mutex);

	return ret;
}

void cfax128_disable(void)
{
	mutex_lock(&cfax128_mutex);

	if (cfax128_updating) {
		cfax128_updating = 0;
		cancel_delayed_work(&cfax128_work);
		flush_workqueue(cfax128_workqueue);
	}

	mutex_unlock(&cfax128_mutex);
}

unsigned char cfax128_isenabled(void)
{
	return cfax128_updating;
}

static void cfax128_update(struct work_struct *work)
{
	unsigned char c;
	unsigned short j, k, b;

	if (memcmp(cfax128_cache, cfax128_buffer, CFAX128_SIZE)) {
		for (j = 0; j < CFAX128_PAGES; j++) {
			nt7534_page(j);
			nt7534_column_address(4);
			for (k = 0; k < CFAX128_ADDRESSES; k++) {
				for (c = 0, b = 0; b < 8; b++) {
					if (cfax128_buffer
						[k / 8 + (j * 8 + b) *
						CFAX128_WIDTH / 8]
						& bit(k % 8))
						c |= bit(b);
				}
					
				nt7534_write_data(c);
			}

		}
		

		memcpy(cfax128_cache, cfax128_buffer, CFAX128_SIZE);
	}

	if (cfax128_updating)
		cfax128_queue();
}

static int cfax128fb_probe(struct platform_device *device)
{
	int ret = -EINVAL;
	struct cfax128_platform_data *pdata;
	struct fb_info *info;

	BUILD_BUG_ON(PAGE_SIZE < CFAX128_SIZE);
	
	/* Get the platform data for the display */
	pdata = device->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "Can't get platform data :(");
		return ret;
	}

	nt7534_init(&pdata->ctrl_chip);
	cfax128_buffer = (unsigned char *) get_zeroed_page(GFP_KERNEL);
	if (cfax128_buffer == NULL) {
		printk(KERN_ERR CFAX128FB_NAME ": ERROR: "
			"can't get a free page\n");
		ret = -ENOMEM;
		goto none;
	}

	cfax128_cache = kmalloc(sizeof(unsigned char) *
		CFAX128_SIZE, GFP_KERNEL);
	if (cfax128_cache == NULL) {
		printk(KERN_ERR CFAX128FB_NAME ": ERROR: "
			"can't alloc cache buffer (%i bytes)\n",
			CFAX128_SIZE);
		ret = -ENOMEM;
		goto bufferalloced;
	}

	cfax128_workqueue = create_singlethread_workqueue(CFAX128FB_NAME);
	if (cfax128_workqueue == NULL)
		goto cachealloced;

	nt7534_reset();
	udelay(100);	
	/* Do the initialisation commands for the device */
	nt7534_display_state(true);
	nt7534_write_control(0xA4);
	nt7534_write_control(0xA6);
	nt7534_write_control(0xA1);  	// Flip X
	nt7534_write_control(0xC8); 	// flip Y
	nt7534_write_control(0xA3); 	// LCD Bias (A2 A3)
	nt7534_write_control(0x2F); 	// VF
	nt7534_write_control(0x24);	// Resistor (20 - 27). This acts like a "range" contol on the contrast.
	nt7534_write_control(0x81);
	nt7534_write_control(27 & 0x3F); // Default Contrast
	nt7534_write_control(0xAD);
	// Duty
	nt7534_start_line(cfax128_start_line);
	nt7534_page(0);
	nt7534_column_address(0);
	cfax128_clear();
	

 	info = framebuffer_alloc(0, &device->dev);

	if (!info)
		goto cachealloced;

	info->screen_base = (char __iomem *) cfax128_buffer;
	info->screen_size = CFAX128_SIZE;
	info->fbops = &cfax128fb_ops;
	info->fix = cfax128fb_fix;
	info->var = cfax128fb_var;
	info->pseudo_palette = NULL;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	if (register_framebuffer(info) < 0)
		goto fballoced;

	cfax128_enable();
	pdata->info = info;

	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
		info->fix.id);


	return 0;
fballoced:
	framebuffer_release(info);

cachealloced:
	kfree(cfax128_cache);

bufferalloced:
	free_page((unsigned long) cfax128_buffer);

none:
	return ret;
}

static int cfax128fb_remove(struct platform_device *device)
{
	struct cfax128_platform_data *pdata = device->dev.platform_data;

	if (pdata->info) {
		unregister_framebuffer(pdata->info);
		framebuffer_release(pdata->info);
	}
	
	cfax128_disable();
	destroy_workqueue(cfax128_workqueue);
	kfree(cfax128_cache);
	free_page((unsigned long) cfax128_buffer);

	return 0;
}


static struct platform_driver cfax128fb_driver = {
	.probe = cfax128fb_probe,
	.remove = cfax128fb_remove,
	.driver = {
		.name = CFAX128FB_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init cfax128fb_init (void) {
	int rc = platform_driver_register(&cfax128fb_driver);
	return rc;
}

static void __exit cfax128fb_exit (void) {
	platform_driver_unregister(&cfax128fb_driver);
}

module_param(cfax128_start_line, uint, S_IRUGO);
module_init(cfax128fb_init);
module_exit(cfax128fb_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ken Wilson");
MODULE_DESCRIPTION("CFAX128 FB Driver");
MODULE_ALIAS("platform:cfax128fb");
MODULE_PARM_DESC(cfax128_start_line,
	"Display start line");

