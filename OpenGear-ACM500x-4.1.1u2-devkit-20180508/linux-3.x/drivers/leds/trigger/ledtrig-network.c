/*
 * Network Activity Trigger
 *
 * Copyright 2014 Opengear Inc.
 *
 * Author: Ken Wilson <ken.wilson@opengear.com>
 *
 * Based on Richard Purdie's ledtrig-ide-disk.c 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>

#define BLINK_DELAY 30


DEFINE_LED_TRIGGER(ledtrig_network);

static unsigned long network_blink_delay = BLINK_DELAY;

void ledtrig_network_activity(void)
{
	led_trigger_blink_oneshot(ledtrig_network,
				  &network_blink_delay, &network_blink_delay, 0);
}
EXPORT_SYMBOL(ledtrig_network_activity);

static int __init ledtrig_network_init(void)
{
	led_trigger_register_simple("network", &ledtrig_network);
	return 0;
}

static void __exit ledtrig_network_exit(void)
{
	led_trigger_unregister_simple(ledtrig_network);
}

module_init(ledtrig_network_init);
module_exit(ledtrig_network_exit);

MODULE_AUTHOR("Ken Wilson <ken.wilson@opengear.com>");
MODULE_DESCRIPTION("Network Activity Trigger");
MODULE_LICENSE("GPL");
