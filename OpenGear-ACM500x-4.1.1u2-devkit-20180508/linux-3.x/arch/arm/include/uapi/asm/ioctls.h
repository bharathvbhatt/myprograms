#ifndef __ASM_ARM_IOCTLS_H
#define __ASM_ARM_IOCTLS_H

#define FIOQSIZE	0x545E
#define TIOCRS232	0x5201	/* OpenGear RS232/RS422 setting ioctls */
#define TIOCRS422	0x5202
#define TIOCRS485	0x5203
#define TIOCRS485e	0x5204
#define TIOCRS422x	0x5205
#define TIOCPINOUTX1	0x5301 /* OpenGear RS232 Pinout switching ioctls */
#define TIOCPINOUTX2	0x5302

#include <asm-generic/ioctls.h>

#endif
