/*
 * arch/arm/mach-ks8695/generic.h
 *
 * Copyright (C) 2006 Ben Dooks <ben@simtec.co.uk>
 * Copyright (C) 2006 Simtec Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
*/

extern __init void ks8695_map_io(void);
extern __init void ks8695_init_irq(void);
extern void ks8695_restart(char, const char *);
extern struct sys_timer ks8695_timer;

/*
 * Direct connect serial ports (non-PCI that is) on some boards.
 */
#ifdef CONFIG_MACH_CM4002
#define	S8250_PHYS	0x03800000
#define	S8250_VIRT	0xf4000000
#define	S8250_SIZE	0x00010000
#define	S8250_IRQ	3
#endif /* CONFIG_MACH_CM4002 */

#ifdef CONFIG_MACH_IM42xx
#define	S8250_PHYS	0x03fe0000
#define	S8250_VIRT	0xf4000000
#define	S8250_SIZE	0x00010000
#define	S8250_IRQ	4
#endif /* CONFIG_MACH_IM42xx */
