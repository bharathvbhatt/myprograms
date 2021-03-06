/*
 * arch/arm/mach-ks8695/include/mach/uncompress.h
 *
 * Copyright (C) 2006 Ben Dooks <ben@simtec.co.uk>
 * Copyright (C) 2006 Simtec Electronics
 *
 * KS8695 - Kernel uncompressor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <linux/io.h>
#include <asm/mach-types.h>
#include <mach/regs-uart.h>

static void putc(char c)
{
	if (!machine_is_lite300() && !machine_is_sg310() &&
	    !machine_is_im42xx() && !machine_is_im4004() && 
	    !machine_is_cm41xx()) {
		while (!(__raw_readl(KS8695_UART_PA + KS8695_URLS) & URLS_URTHRE))
			barrier();
		__raw_writel(c, KS8695_UART_PA + KS8695_URTH);
	}
}

static inline void flush(void)
{
	if (!machine_is_lite300() && !machine_is_sg310() &&
	    !machine_is_im42xx() && !machine_is_im4004() && 
	    !machine_is_cm41xx()) {
		while (!(__raw_readl(KS8695_UART_PA + KS8695_URLS) & URLS_URTE))
			barrier();
	}
}

#define arch_decomp_setup()

#endif
