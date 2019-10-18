
/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.


********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/
#ifndef _MVOSABSTRACT_H
#define _MVOSABSTRACT_H
/* Catchall OS Abstraction Header */
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/pci.h>

extern void mv_early_printk(char *fmt, ...);

#define MV_ASM              __asm__ __volatile__
#define INLINE              inline
#define _INIT				__init
#define MV_TRC_REC	        TRC_REC
#define mvOsPrintf          printk
#define mvOsEarlyPrintf	    mv_early_printk
#define mvOsOutput          printk
#define mvOsSPrintf         sprintf
#define mvOsSNPrintf        snprintf
#define mvOsMalloc(_size_)  kmalloc(_size_, GFP_ATOMIC)
#define mvOsFree            kfree
#define mvOsMemcpy          memcpy
#define mvOsMemset          memset
#define mvOsSleep(_mils_)   mdelay(_mils_)
#define mvOsTaskLock()
#define mvOsTaskUnlock()
#define strtol              simple_strtoul
#define mvOsDelay(x)        mdelay(x)
#define mvOsUDelay(x)       udelay(x)
#define mvCopyFromOs        copy_from_user
#define mvCopyToOs          copy_to_user
#define mvOsWarning()       WARN_ON(1)
#define mvOsGetTicks()      jiffies
#define mvOsGetTicksFreq()  HZ

#define MV_ERROR		    (-1)
#define MV_OK			    (0)	/* Operation succeeded                   */
#define MV_FAIL			    (1)	/* Operation failed                      */
#define MV_BAD_VALUE        (2)	/* Illegal value (general)               */
#define MV_OUT_OF_RANGE     (3)	/* The value is out of range             */
#define MV_BAD_PARAM        (4)	/* Illegal parameter in function called  */
#define MV_BAD_PTR          (5)	/* Illegal pointer value                 */
#define MV_BAD_SIZE         (6)	/* Illegal size                          */
#define MV_BAD_STATE        (7)	/* Illegal state of state machine        */
#define MV_SET_ERROR        (8)	/* Set operation failed                  */
#define MV_GET_ERROR        (9)	/* Get operation failed                  */
#define MV_CREATE_ERROR     (10)	/* Fail while creating an item           */
#define MV_NOT_FOUND        (11)	/* Item not found                        */
#define MV_NO_MORE          (12)	/* No more items found                   */
#define MV_NO_SUCH          (13)	/* No such item                          */
#define MV_TIMEOUT          (14)	/* Time Out                              */
#define MV_NO_CHANGE        (15)	/* Parameter(s) is already in this value */
#define MV_NOT_SUPPORTED    (16)	/* This request is not support           */
#define MV_NOT_IMPLEMENTED  (17)	/* Request supported but not implemented */
#define MV_NOT_INITIALIZED  (18)	/* The item is not initialized           */
#define MV_NO_RESOURCE      (19)	/* Resource not available (memory ...)   */
#define MV_FULL             (20)	/* Item is full (Queue or table etc...)  */
#define MV_EMPTY            (21)	/* Item is empty (Queue or table etc...) */
#define MV_INIT_ERROR       (22)	/* Error occured while INIT process      */
#define MV_HW_ERROR         (23)	/* Hardware error                        */
#define MV_TX_ERROR         (24)	/* Transmit operation not succeeded      */
#define MV_RX_ERROR         (25)	/* Recieve operation not succeeded       */
#define MV_NOT_READY	    (26)	/* The other side is not ready yet       */
#define MV_ALREADY_EXIST    (27)	/* Tried to create existing item         */
#define MV_OUT_OF_CPU_MEM   (28)	/* Cpu memory allocation failed.         */
#define MV_NOT_STARTED      (29)	/* Not started yet                       */
#define MV_BUSY             (30)	/* Item is busy.                         */
#define MV_TERMINATE        (31)	/* Item terminates it's work.            */
#define MV_NOT_ALIGNED      (32)	/* Wrong alignment                       */
#define MV_NOT_ALLOWED      (33)	/* Operation NOT allowed                 */
#define MV_WRITE_PROTECT    (34)	/* Write protected                       */
#define MV_DROPPED          (35)	/* Packet dropped                        */
#define MV_STOLEN           (36)	/* Packet stolen */
#define MV_CONTINUE         (37)        /* Continue */
#define MV_RETRY		    (38)	/* Operation failed need retry           */

#define MV_INVALID  (int)(-1)

#define MV_FALSE	0
#define MV_TRUE     (!(MV_FALSE))

#ifndef NULL
#define NULL ((void *)0)
#endif

/* typedefs */

typedef char MV_8;
typedef unsigned char MV_U8;

typedef int MV_32;
typedef unsigned int MV_U32;

typedef short MV_16;
typedef unsigned short MV_U16;

#ifdef MV_PPC64
typedef long MV_64;
typedef unsigned long MV_U64;
#else
typedef long long MV_64;
typedef unsigned long long MV_U64;
#endif

typedef long MV_LONG;		/* 32/64 */
typedef unsigned long MV_ULONG;	/* 32/64 */

typedef int MV_STATUS;
typedef int MV_BOOL;
typedef void MV_VOID;
typedef float MV_FLOAT;

typedef int (*MV_FUNCPTR) (void);	/* ptr to function returning int   */
typedef void (*MV_VOIDFUNCPTR) (void);	/* ptr to function returning void  */
typedef double (*MV_DBLFUNCPTR) (void);	/* ptr to function returning double */
typedef float (*MV_FLTFUNCPTR) (void);	/* ptr to function returning float */

typedef MV_U32 MV_KHZ;
typedef MV_U32 MV_MHZ;
typedef MV_U32 MV_HZ;

/* This enumerator describes the set of commands that can be applied on   	*/
/* an engine (e.g. IDMA, XOR). Appling a comman depends on the current   	*/
/* status (see MV_STATE enumerator)                      					*/
/* Start can be applied only when status is IDLE                         */
/* Stop can be applied only when status is IDLE, ACTIVE or PAUSED        */
/* Pause can be applied only when status is ACTIVE                          */
/* Restart can be applied only when status is PAUSED                        */
typedef enum _mvCommand {
	MV_START,		/* Start     */
	MV_STOP,		/* Stop     */
	MV_PAUSE,		/* Pause    */
	MV_RESTART		/* Restart  */
} MV_COMMAND;

/* This enumerator describes the set of state conditions.					*/
/* Moving from one state to other is stricted.   							*/
typedef enum _mvState {
	MV_IDLE,
	MV_ACTIVE,
	MV_PAUSED,
	MV_UNDEFINED_STATE
} MV_STATE;

typedef enum {
	ETH_MAC_SPEED_10M,
	ETH_MAC_SPEED_100M,
	ETH_MAC_SPEED_1000M,
	ETH_MAC_SPEED_AUTO

} MV_ETH_MAC_SPEED;

/* This structure describes address space window. Window base can be        */
/* 64 bit, window size up to 4GB                                            */
typedef struct _mvAddrWin {
	MV_U32 baseLow;		/* 32bit base low       */
	MV_U32 baseHigh;	/* 32bit base high      */
	MV_U64 size;		/* 64bit size           */
} MV_ADDR_WIN;

/* This binary enumerator describes protection attribute status             */
typedef enum _mvProtRight {
	ALLOWED,		/* Protection attribute allowed                         */
	FORBIDDEN		/* Protection attribute forbidden                       */
} MV_PROT_RIGHT;

/* Unified struct for Rx and Tx packet operations. The user is required to 	*/
/* be familier only with Tx/Rx descriptor command status.               	*/
typedef struct _bufInfo {
	MV_U32 cmdSts;		/* Tx/Rx command status                                     */
	MV_U16 byteCnt;		/* Size of valid data in the buffer     */
	MV_U16 bufSize;		/* Total size of the buffer             */
	MV_U8 *pBuff;		/* Pointer to Buffer                    */
	MV_U8 *pData;		/* Pointer to data in the Buffer        */
	MV_U32 userInfo1;	/* Tx/Rx attached user information 1    */
	MV_U32 userInfo2;	/* Tx/Rx attached user information 2    */
	struct _bufInfo *pNextBufInfo;	/* Next buffer in packet            */
} BUF_INFO;

/* This structure contains information describing one of buffers
 * (fragments) they are built Ethernet packet.
 */
typedef struct {
	MV_U8 *bufVirtPtr;
	MV_ULONG bufPhysAddr;
	MV_U32 bufSize;
	MV_U32 dataSize;
	MV_U32 memHandle;
	MV_32 bufAddrShift;
} MV_BUF_INFO;

/* This structure contains information describing Ethernet packet.
 * The packet can be divided for few buffers (fragments)
 */
typedef struct {
	MV_ULONG osInfo;
	MV_BUF_INFO *pFrags;
	MV_U32 status;
	MV_U16 pktSize;
	MV_U16 numFrags;
	MV_U32 ownerId;
	MV_U32 fragIP;
	MV_U32 txq;
} MV_PKT_INFO;

/* This structure describes SoC units address decode window	*/
typedef struct {
	MV_ADDR_WIN addrWin;	/* An address window */
	MV_BOOL enable;		/* Address decode window is enabled/disabled    */
	MV_U8 attrib;		/* chip select attributes */
	MV_U8 targetId;		/* Target Id of this MV_TARGET */
} MV_UNIT_WIN_INFO;

/* This structure describes access rights for Access protection windows     */
/* that can be found in IDMA, XOR, Ethernet and MPSC units.                 */
/* Note that the permission enumerator coresponds to its register format.   */
/* For example, Read only premission is presented as "1" in register field. */
typedef enum _mvAccessRights {
	NO_ACCESS_ALLOWED = 0,	/* No access allowed            */
	READ_ONLY = 1,		/* Read only permission         */
	ACC_RESERVED = 2,	/* Reserved access right                */
	FULL_ACCESS = 3,	/* Read and Write permission    */
	MAX_ACC_RIGHTS
} MV_ACCESS_RIGHTS;

typedef struct _mvDecRegs {
	MV_U32 baseReg;
	MV_U32 baseRegHigh;
	MV_U32 ctrlReg;
} MV_DEC_REGS;

/* The golden ration: an arbitrary value */
#define MV_JHASH_GOLDEN_RATIO           0x9e3779b9

#define MV_JHASH_MIX(a, b, c)        \
{                                   \
    a -= b; a -= c; a ^= (c>>13);   \
    b -= c; b -= a; b ^= (a<<8);    \
    c -= a; c -= b; c ^= (b>>13);   \
    a -= b; a -= c; a ^= (c>>12);   \
    b -= c; b -= a; b ^= (a<<16);   \
    c -= a; c -= b; c ^= (b>>5);    \
    a -= b; a -= c; a ^= (c>>3);    \
    b -= c; b -= a; b ^= (a<<10);   \
    c -= a; c -= b; c ^= (b>>15);   \
}

#ifdef MV_VXWORKS
static __inline__ MV_U32 mv_jhash_3words(MV_U32 a, MV_U32 b, MV_U32 c, MV_U32 initval)
#else
static inline MV_U32 mv_jhash_3words(MV_U32 a, MV_U32 b, MV_U32 c, MV_U32 initval)

#endif
{
	a += MV_JHASH_GOLDEN_RATIO;
	b += MV_JHASH_GOLDEN_RATIO;
	c += initval;
	MV_JHASH_MIX(a, b, c);

	return c;
}


/* Swap tool */

/* 16bit nibble swap. For example 0x1234 -> 0x2143                          */
#define MV_NIBBLE_SWAP_16BIT(X)        (((X&0xf) << 4) |     \
					((X&0xf0) >> 4) |    \
					((X&0xf00) << 4) |   \
					((X&0xf000) >> 4))

/* 32bit nibble swap. For example 0x12345678 -> 0x21436587                  */
#define MV_NIBBLE_SWAP_32BIT(X)		(((X&0xf) << 4) |       \
					((X&0xf0) >> 4) |      \
					((X&0xf00) << 4) |     \
					((X&0xf000) >> 4) |    \
					((X&0xf0000) << 4) |   \
					((X&0xf00000) >> 4) |  \
					((X&0xf000000) << 4) | \
					((X&0xf0000000) >> 4))

/* 16bit byte swap. For example 0x1234->0x3412                             */
#define MV_BYTE_SWAP_16BIT(X) ((((X)&0xff)<<8) | (((X)&0xff00)>>8))

/* 32bit byte swap. For example 0x12345678->0x78563412                    */
#define MV_BYTE_SWAP_32BIT(X)  ((((X)&0xff)<<24) |                       \
				(((X)&0xff00)<<8) |                      \
				(((X)&0xff0000)>>8) |                    \
				(((X)&0xff000000)>>24))

/* 64bit byte swap. For example 0x11223344.55667788 -> 0x88776655.44332211  */
#define MV_BYTE_SWAP_64BIT(X) ((l64) ((((X)&0xffULL)<<56) |             \
				      (((X)&0xff00ULL)<<40) |           \
				      (((X)&0xff0000ULL)<<24) |         \
				      (((X)&0xff000000ULL)<<8) |        \
				      (((X)&0xff00000000ULL)>>8) |      \
				      (((X)&0xff0000000000ULL)>>24) |   \
				      (((X)&0xff000000000000ULL)>>40) | \
				      (((X)&0xff00000000000000ULL)>>56)))

/* Endianess macros.                                                        */
#if defined(MV_CPU_LE)
#define MV_16BIT_LE(X)  (X)
#define MV_32BIT_LE(X)  (X)
#define MV_64BIT_LE(X)  (X)
#define MV_16BIT_BE(X)  MV_BYTE_SWAP_16BIT(X)
#define MV_32BIT_BE(X)  MV_BYTE_SWAP_32BIT(X)
#define MV_64BIT_BE(X)  MV_BYTE_SWAP_64BIT(X)
#elif defined(MV_CPU_BE)
#define MV_16BIT_LE(X)  MV_BYTE_SWAP_16BIT(X)
#define MV_32BIT_LE(X)  MV_BYTE_SWAP_32BIT(X)
#define MV_64BIT_LE(X)  MV_BYTE_SWAP_64BIT(X)
#define MV_16BIT_BE(X)  (X)
#define MV_32BIT_BE(X)  (X)
#define MV_64BIT_BE(X)  (X)
#else
#error "CPU endianess isn't defined!\n"
#endif

/* Bit field definitions */
#define NO_BIT      0x00000000

/* avoid redefinition of bits */
#ifndef BIT0

#define BIT0        0x00000001
#define BIT1        0x00000002
#define BIT2        0x00000004
#define BIT3        0x00000008
#define BIT4        0x00000010
#define BIT5        0x00000020
#define BIT6        0x00000040
#define BIT7        0x00000080
#define BIT8        0x00000100
#define BIT9        0x00000200
#define BIT10       0x00000400
#define BIT11       0x00000800
#define BIT12       0x00001000
#define BIT13       0x00002000
#define BIT14       0x00004000
#define BIT15       0x00008000
#define BIT16       0x00010000
#define BIT17       0x00020000
#define BIT18       0x00040000
#define BIT19       0x00080000
#define BIT20       0x00100000
#define BIT21       0x00200000
#define BIT22       0x00400000
#define BIT23       0x00800000
#define BIT24       0x01000000
#define BIT25       0x02000000
#define BIT26       0x04000000
#define BIT27       0x08000000
#define BIT28       0x10000000
#define BIT29       0x20000000
#define BIT30       0x40000000
#define BIT31       0x80000000

#endif /* BIT0 */
/* Handy sizes */
#define _1K         0x00000400
#define _2K         0x00000800
#define _4K         0x00001000
#define _8K         0x00002000
#define _16K        0x00004000
#define _32K        0x00008000
#define _64K        0x00010000
#define _128K       0x00020000
#define _256K       0x00040000
#define _512K       0x00080000

#define _1M         0x00100000
#define _2M         0x00200000
#define _4M         0x00400000
#define _8M         0x00800000
#define _16M        0x01000000
#define _32M        0x02000000
#define _64M        0x04000000
#define _128M       0x08000000
#define _256M       0x10000000
#define _512M       0x20000000

#define _1G         0x40000000
#define _2G         0x80000000

/* Tclock and Sys clock define */
#define _100MHz     100000000
#define _125MHz     125000000
#define _133MHz     133333334
#define _150MHz     150000000
#define _160MHz     160000000
#define _166MHz     166666667
#define _175MHz     175000000
#define _178MHz     178000000
#define _183MHz     183333334
#define _187MHz     187000000
#define _192MHz     192000000
#define _194MHz     194000000
#define _200MHz     200000000
#define _233MHz     233333334
#define _250MHz     250000000
#define _266MHz     266666667
#define _300MHz     300000000

/* Supported clocks */
#define MV_BOARD_TCLK_100MHZ	100000000
#define MV_BOARD_TCLK_125MHZ	125000000
#define MV_BOARD_TCLK_133MHZ	133333333
#define MV_BOARD_TCLK_150MHZ	150000000
#define MV_BOARD_TCLK_166MHZ	166666667
#define MV_BOARD_TCLK_200MHZ	200000000
#define MV_BOARD_TCLK_250MHZ	250000000

#define MV_BOARD_SYSCLK_100MHZ	100000000
#define MV_BOARD_SYSCLK_125MHZ	125000000
#define MV_BOARD_SYSCLK_133MHZ	133333333
#define MV_BOARD_SYSCLK_150MHZ	150000000
#define MV_BOARD_SYSCLK_166MHZ	166666667
#define MV_BOARD_SYSCLK_200MHZ	200000000
#define MV_BOARD_SYSCLK_233MHZ	233333333
#define MV_BOARD_SYSCLK_250MHZ	250000000
#define MV_BOARD_SYSCLK_267MHZ	266666667
#define MV_BOARD_SYSCLK_300MHZ	300000000
#define MV_BOARD_SYSCLK_333MHZ	333333334
#define MV_BOARD_SYSCLK_400MHZ	400000000

#define MV_BOARD_REFCLK_25MHZ	 25000000

/* For better address window table readability */
#define EN			MV_TRUE
#define DIS			MV_FALSE
#define N_A			-1	/* Not applicable */

/* Cache configuration options for memory (DRAM, SRAM, ... ) */

/* Memory uncached, HW or SW cache coherency is not needed */
#define MV_UNCACHED             0
/* Memory cached, HW cache coherency supported in WriteThrough mode */
#define MV_CACHE_COHER_HW_WT    1
/* Memory cached, HW cache coherency supported in WriteBack mode */
#define MV_CACHE_COHER_HW_WB    2
/* Memory cached, No HW cache coherency, Cache coherency must be in SW */
#define MV_CACHE_COHER_SW       3

/* Macro for testing aligment. Positive if number is NOT aligned   */
#define MV_IS_NOT_ALIGN(number, align)      ((number) & ((align) - 1))

/* Macro for alignment up. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0340   */
#define MV_ALIGN_UP(number, align)                                          \
(((number) & ((align) - 1)) ? (((number) + (align)) & ~((align)-1)) : (number))

/* Macro for alignment down. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0320 */
#define MV_ALIGN_DOWN(number, align) ((number) & ~((align)-1))

/* This macro returns absolute value                                        */
#define MV_ABS(number)  (((int)(number) < 0) ? -(int)(number) : (int)(number))

/* Bit fields manipulation macros                                           */

/* An integer word which its 'x' bit is set                                 */
#define MV_BIT_MASK(bitNum)         (1 << (bitNum))

/* Checks wheter bit 'x' in integer word is set                             */
#define MV_BIT_CHECK(word, bitNum)  ((word) & MV_BIT_MASK(bitNum))

/* Clear (reset) bit 'x' in integer word (RMW - Read-Modify-Write)          */
#define MV_BIT_CLEAR(word, bitNum)  ((word) &= ~(MV_BIT_MASK(bitNum)))

/* Set bit 'x' in integer word (RMW)                                        */
#define MV_BIT_SET(word, bitNum)    ((word) |= MV_BIT_MASK(bitNum))

/* Invert bit 'x' in integer word (RMW)                                     */
#define MV_BIT_INV(word, bitNum)    ((word) ^= MV_BIT_MASK(bitNum))

/* Get the min between 'a' or 'b'                                           */
#define MV_MIN(a, b)    (((a) < (b)) ? (a) : (b))

/* Get the max between 'a' or 'b'                                           */
#define MV_MAX(a, b)    (((a) < (b)) ? (b) : (a))

#define mvOsDivide(num, div)	\
({				\
	int i = 0, rem = (num);	\
	while (rem >= (div)) {	\
		rem -= (div);	\
		i++;		\
	}			\
	(i);			\
})

#define mvOsReminder(num, div)	\
({				\
	int rem = (num);	\
	while (rem >= (div))	\
		rem -= (div);	\
	(rem);			\
})

#define MV_MACQUAD_FMT "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x"

#define MV_MACQUAD(addr) \
	((unsigned char *)addr)[0], \
	((unsigned char *)addr)[1], \
	((unsigned char *)addr)[2], \
	((unsigned char *)addr)[3], \
	((unsigned char *)addr)[4], \
	((unsigned char *)addr)[5]

#define MV_IPQUAD_FMT         "%u.%u.%u.%u"
#define MV_IPQUAD(ip)         ip[0], ip[1], ip[2], ip[3]

#define MV_IP_QUAD(ipAddr)    ((ipAddr >> 24) & 0xFF), ((ipAddr >> 16) & 0xFF), \
				((ipAddr >> 8) & 0xFF), ((ipAddr >> 0) & 0xFF)

#define MV_IP6_FMT		"%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x"
#define MV_IP6_ARG(L3)		L3[0], L3[1], L3[2], L3[3],	\
				L3[4], L3[5], L3[6], L3[7],	\
				L3[8], L3[9], L3[10], L3[11],	\
				L3[12], L3[13], L3[14], L3[15]

#define MV_IS_POWER_OF_2(num) ((num != 0) && ((num & (num - 1)) == 0))

#define MV_GET_BIT(word, bitNum) (((word) & (1 << (bitNum))) >> (bitNum))

#define MV_SET_BIT(word, bitNum, bitVal) (((word) & ~(1 << (bitNum))) | (bitVal << bitNum))

#define MV_ARRAY_SIZE(a)                    ((sizeof(a)) / (sizeof(a[0])))

/* REG BASE */
#define INTER_REGS_BASE 0xfec00000  /* Virt Address Base */
#define MV_CESA_MAX_BUF_SIZE 1600
#define MV_CESA_TDMA_REGS_BASE(chanNum) (0x90000 + (chanNum * 0x2000))
#define MV_CESA_REGS_BASE(chanNum) (0x9D000 + (chanNum * 0x2000))

#ifdef MV_NDEBUG
#define mvOsAssert(cond)
#else
#define mvOsAssert(cond) { do { if (!(cond)) { BUG(); } } while (0); }
#endif /* MV_NDEBUG */

#define mvOsIoVirtToPhy(pDev, pVirtAddr)        virt_to_dma((pDev), (pVirtAddr))
/*    pci_map_single((pDev), (pVirtAddr), 0, PCI_DMA_BIDIRECTIONAL) */

#define mvOsIoVirtToPhys(pDev, pVirtAddr)       virt_to_dma((pDev), (pVirtAddr))

#define mvOsCacheFlushInv(pDev, p, size)                            \
	pci_map_single((pDev), (p), (size), PCI_DMA_BIDIRECTIONAL)
/*
 * This was omitted because as of 2.6.35 the Biderection performs only
 * flush for outer cache because of the speculative issues.
 * This should be replaced by Flush then invalidate
 *
#define mvOsCacheClear(pDev, p, size )                              \
	pci_map_single((pDev), (p), (size), PCI_DMA_BIDIRECTIONAL)
*/

#define mvOsCacheFlush(pDev, p, size)                              \
	pci_map_single((pDev), (p), (size), PCI_DMA_TODEVICE)

#define mvOsCacheInvalidate(pDev, p, size)                          \
	pci_map_single((pDev), (p), (size), PCI_DMA_FROMDEVICE)

#define mvOsCacheUnmap(pDev, phys, size)                          \
	pci_unmap_single((pDev), (dma_addr_t)(phys), (size), PCI_DMA_FROMDEVICE)

#define CPU_PHY_MEM(x)              (MV_U32)x
#define CPU_MEMIO_CACHED_ADDR(x)    (void *)x
#define CPU_MEMIO_UNCACHED_ADDR(x)  (void *)x


/* CPU architecture dependent 32, 16, 8 bit read/write IO addresses */
#define MV_MEMIO32_WRITE(addr, data)    \
	((*((volatile unsigned int *)(addr))) = ((unsigned int)(data)))

#define MV_MEMIO32_READ(addr)           \
	((*((volatile unsigned int *)(addr))))

#define MV_MEMIO16_WRITE(addr, data)    \
	((*((volatile unsigned short *)(addr))) = ((unsigned short)(data)))

#define MV_MEMIO16_READ(addr)           \
	((*((volatile unsigned short *)(addr))))

#define MV_MEMIO8_WRITE(addr, data)     \
	((*((volatile unsigned char *)(addr))) = ((unsigned char)(data)))

#define MV_MEMIO8_READ(addr)            \
	((*((volatile unsigned char *)(addr))))


/* No Fast Swap implementation (in assembler) for ARM */
#define MV_32BIT_LE_FAST(val)            MV_32BIT_LE(val)
#define MV_16BIT_LE_FAST(val)            MV_16BIT_LE(val)
#define MV_32BIT_BE_FAST(val)            MV_32BIT_BE(val)
#define MV_16BIT_BE_FAST(val)            MV_16BIT_BE(val)

/* 32 and 16 bit read/write in big/little endian mode */

/* 16bit write in little endian mode */
#define MV_MEMIO_LE16_WRITE(addr, data) \
	MV_MEMIO16_WRITE(addr, MV_16BIT_LE_FAST(data))

/* 16bit read in little endian mode */
static inline MV_U16 MV_MEMIO_LE16_READ(MV_U32 addr)
{
	MV_U16 data;

	data = (MV_U16)MV_MEMIO16_READ(addr);

	return (MV_U16)MV_16BIT_LE_FAST(data);
}

/* 32bit write in little endian mode */
#define MV_MEMIO_LE32_WRITE(addr, data) \
	MV_MEMIO32_WRITE(addr, MV_32BIT_LE_FAST(data))

/* 32bit read in little endian mode */
static inline MV_U32 MV_MEMIO_LE32_READ(MV_U32 addr)
{
	MV_U32 data;

	data = (MV_U32)MV_MEMIO32_READ(addr);

	return (MV_U32)MV_32BIT_LE_FAST(data);
}

static inline void mvOsBCopy(char *srcAddr, char *dstAddr, int byteCount)
{
	while (byteCount != 0) {
		*dstAddr = *srcAddr;
		dstAddr++;
		srcAddr++;
		byteCount--;
	}
}

static INLINE MV_U64 mvOsDivMod64(MV_U64 divided, MV_U64 divisor, MV_U64 *modulu)
{
	MV_U64  division = 0;

	if (divisor == 1)
		return divided;

	while (divided >= divisor) {
		division++;
		divided -= divisor;
	}
	if (modulu != NULL)
		*modulu = divided;

	return division;
}

#if defined(MV_BRIDGE_SYNC_REORDER)
extern MV_U32 *mvUncachedParam;

static inline void mvOsBridgeReorderWA(void)
{
	volatile MV_U32 val = 0;

	val = mvUncachedParam[0];
}
#endif


/* Flash APIs */
#define MV_FL_8_READ            MV_MEMIO8_READ
#define MV_FL_16_READ           MV_MEMIO_LE16_READ
#define MV_FL_32_READ           MV_MEMIO_LE32_READ
#define MV_FL_8_DATA_READ       MV_MEMIO8_READ
#define MV_FL_16_DATA_READ      MV_MEMIO16_READ
#define MV_FL_32_DATA_READ      MV_MEMIO32_READ
#define MV_FL_8_WRITE           MV_MEMIO8_WRITE
#define MV_FL_16_WRITE          MV_MEMIO_LE16_WRITE
#define MV_FL_32_WRITE          MV_MEMIO_LE32_WRITE
#define MV_FL_8_DATA_WRITE      MV_MEMIO8_WRITE
#define MV_FL_16_DATA_WRITE     MV_MEMIO16_WRITE
#define MV_FL_32_DATA_WRITE     MV_MEMIO32_WRITE


/* CPU cache information */
#define CPU_I_CACHE_LINE_SIZE   32    /* 2do: replace 32 with linux core macro */
#define CPU_D_CACHE_LINE_SIZE   32    /* 2do: replace 32 with linux core macro */

#if defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_4413)
#define	 DSBWA_4413(x)	dmb() 		/* replaced dsb() for optimization */
#else
#define  DSBWA_4413(x)
#endif

#if defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_4611)
#define	 DSBWA_4611(x)	dmb()		/* replaced dsb() for optimization */
#else
#define  DSBWA_4611(x)
#endif

#if defined(CONFIG_AURORA_IO_CACHE_COHERENCY)
#define mvOsCacheLineFlushInv(handle, addr)
#define mvOsCacheLineInv(handle, addr)
#define mvOsCacheLineFlush(handle, addr)
#define mvOsCacheMultiLineFlush(handle, addr, size)
#define mvOsCacheMultiLineInv(handle, addr, size)
#define mvOsCacheMultiLineFlushInv(handle, addr, size)
#define mvOsCacheIoSync()	dma_io_sync()
#else
#define mvOsCacheIoSync()	/* Not needed in s/w cache coherency (SWCC) */
/*************************************/
/* FLUSH & INVALIDATE single D$ line */
/*************************************/
#if defined(CONFIG_L2_CACHE_ENABLE) || defined(CONFIG_CACHE_FEROCEON_L2)
#define mvOsCacheLineFlushInv(handle, addr)                     \
{                                                               \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 1, %0, c15, c10, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 0, r0, c7, c10, 4");		\
}
#elif defined(CONFIG_CACHE_AURORA_L2)
#define mvOsCacheLineFlushInv(handle, addr)                     \
{                                                               \
	DSBWA_4611(addr);						 \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));  /* Clean and Inv D$ by MVA to PoC */ \
	writel(__virt_to_phys((int)(((int)addr) & ~0x1f)), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x7F0/*L2_FLUSH_PA*/)); \
	writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/)); \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr));  /* DSB */ \
}
#else
#define mvOsCacheLineFlushInv(handle, addr)                     \
{                                                               \
	DSBWA_4611(addr);						 \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); \
}
#endif

/*****************************/
/* INVALIDATE single D$ line */
/*****************************/
#if defined(CONFIG_L2_CACHE_ENABLE) || defined(CONFIG_CACHE_FEROCEON_L2)
#define mvOsCacheLineInv(handle, addr)                          \
{                                                               \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c6, 1" : : "r" (addr)); \
	__asm__ __volatile__ ("mcr p15, 1, %0, c15, c11, 1" : : "r" (addr)); \
}
#elif defined(CONFIG_CACHE_AURORA_L2)
#define mvOsCacheLineInv(handle, addr)                          \
{                                                               \
	DSBWA_4413(addr);								\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c6, 1" : : "r" (addr));   /* Invalidate D$ by MVA to PoC */ \
	writel(__virt_to_phys(((int)addr) & ~0x1f), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x770/*L2_INVALIDATE_PA*/)); \
	writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/)); \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr));  /* DSB */ \
}
#else
#define mvOsCacheLineInv(handle, addr)                          \
{                                                               \
	DSBWA_4413(addr);							\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c6, 1" : : "r" (addr)); \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); \
}
#endif

/************************/
/* FLUSH single D$ line */
/************************/
#if defined(CONFIG_L2_CACHE_ENABLE) || defined(CONFIG_CACHE_FEROCEON_L2)
#if defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6043) || defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6124)
#  define mvOsCacheLineInvalidateDcacheMVA(addr) \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr))
#else
#  define mvOsCacheLineInvalidateDcacheMVA(addr) \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 1" : : "r" (addr))
#endif
#define mvOsCacheLineFlush(handle, addr)                     \
{                                                               \
	mvOsCacheLineInvalidateDcacheMVA(addr); \
	__asm__ __volatile__ ("mcr p15, 1, %0, c15, c9, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 0, r0, c7, c10, 4");          \
 }
 #elif defined(CONFIG_CACHE_AURORA_L2) && !defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6043) && !defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6124)

 #define mvOsCacheLineFlush(handle, addr)                     \
 {                                                               \
   DSBWA_4611(addr);                                             \
        __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 1" : : "r" (addr)); /* Clean D$ line by MVA to PoC */ \
   writel(__virt_to_phys(((int)addr) & ~0x1f), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x7B0/*L2_CLEAN_PA*/)); \
   writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/)); \
   __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); /* DSB */ \
 }

#elif defined(CONFIG_CACHE_AURORA_L2) && (defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6043) || defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6124))
 #define mvOsCacheLineFlush(handle, addr)                     \
 {                                                               \
   DSBWA_4611(addr);						 \
        __asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));\
   writel(__virt_to_phys(((int)addr) & ~0x1f), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x7B0/*L2_CLEAN_PA*/)); \
   writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/)); \
   __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); /* DSB */ \
 }
#elif defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6043) || defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6124)
#define mvOsCacheLineFlush(handle, addr)                     \
{                                                               \
	DSBWA_4611(addr);					 \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); \
}
#else
#define mvOsCacheLineFlush(handle, addr)                     \
{                                                               \
	DSBWA_4611(addr);					 \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 1" : : "r" (addr));\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); \
}
#endif

#define MV_OS_CACHE_MULTI_THRESH	256

/* Flush multiple cache lines using mvOsCacheLineFlush to improve performance.              */
/* addr is the pointer to start the flush operation from. It will be aligned to             */
/* the beginning of the cache line automatically and the size will be adjusted accordingly. */
static inline void mvOsCacheMultiLineFlush(void *handle, void *addr, int size)
{
	if (size <= MV_OS_CACHE_MULTI_THRESH) {
		int shift = ((MV_ULONG)(addr) & (CPU_D_CACHE_LINE_SIZE - 1));
		if (shift) {
			addr -= shift; /* align address back to the beginning of a cache line */
			size += shift;
		}
#if defined(CONFIG_CACHE_AURORA_L2)
		DSBWA_4611(addr);
		while (size > 0) {
#if defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6043) || defined(CONFIG_SHEEVA_ERRATA_ARM_CPU_6124)
			__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));
#else
			__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 1" : : "r" (addr)); /* Clean D$ line by MVA to PoC */
#endif
			writel(__virt_to_phys(((int)addr) & ~0x1f), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x7B0/*L2_CLEAN_PA*/));
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
		writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/));
		__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr)); /* DSB */
#else
		while (size > 0) {
			mvOsCacheLineFlush(handle, addr);
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
#endif /* CONFIG_CACHE_AURORA_L2 */
	} else
		pci_map_single(handle, addr, size, PCI_DMA_TODEVICE);
}

/* Invalidate multiple cache lines using mvOsCacheLineInv to improve performance.           */
/* addr is the pointer to start the invalidate operation from. It will be aligned to        */
/* the beginning of the cache line automatically and the size will be adjusted accordingly. */
/* IMPORTANT: this function assumes the invalidate operation on partial lines does not      */
/* interfere with the data written there.                                                   */
/* DO NOT USE this function unless you are certain of this!                                 */
static inline void mvOsCacheMultiLineInv(void *handle, void *addr, int size)
{
	if (size <= MV_OS_CACHE_MULTI_THRESH) {
		int shift = ((MV_ULONG)(addr) & (CPU_D_CACHE_LINE_SIZE - 1));
		if (shift) {
			addr -= shift; /* align address back to the beginning of a cache line */
			size += shift;
		}
#if defined(CONFIG_CACHE_AURORA_L2)
		DSBWA_4413(addr);
		while (size > 0) {
			__asm__ __volatile__ ("mcr p15, 0, %0, c7, c6, 1" : : "r" (addr));   /* Invalidate D$ by MVA to PoC */
			writel(__virt_to_phys(((int)addr) & ~0x1f), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x770/*L2_INVALIDATE_PA*/));
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
		writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/));
		__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr));  /* DSB */
#else
		while (size > 0) {
			mvOsCacheLineInv(handle, addr);
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
#endif  /* CONFIG_CACHE_AURORA_L2 */
	} else
		pci_map_single(handle, addr, size, PCI_DMA_FROMDEVICE);
}

/* Flush and invalidate multiple cache lines using mvOsCacheLineFlushInv to improve performance. */
/* addr is the pointer to start the flush and invalidate operation from. It will be aligned to   */
/* the beginning of the cache line automatically and the size will be adjusted accordingly.      */
static inline void mvOsCacheMultiLineFlushInv(void *handle, void *addr, int size)
{
	if (size <= MV_OS_CACHE_MULTI_THRESH) {
		int shift = ((MV_ULONG)(addr) & (CPU_D_CACHE_LINE_SIZE - 1));
		if (shift) {
			addr -= shift; /* align address back to the beginning of a cache line */
			size += shift;
		}
#if defined(CONFIG_CACHE_AURORA_L2)
		DSBWA_4611(addr);
		while (size > 0) {
			__asm__ __volatile__ ("mcr p15, 0, %0, c7, c14, 1" : : "r" (addr));  /* Clean and Inv D$ by MVA to PoC */
			writel(__virt_to_phys((int)(((int)addr) & ~0x1f)), (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x7F0/*L2_FLUSH_PA*/));
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
		writel(0x0, (INTER_REGS_BASE + MV_AURORA_L2_REGS_OFFSET + 0x700/*L2_SYNC*/));
		__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (addr));  /* DSB */
#else
		while (size > 0) {
			mvOsCacheLineFlushInv(handle, addr);
			size -= CPU_D_CACHE_LINE_SIZE;
			addr += CPU_D_CACHE_LINE_SIZE;
		}
#endif  /* CONFIG_CACHE_AURORA_L2 */
	} else
		pci_map_single(handle, addr, size, PCI_DMA_BIDIRECTIONAL);
}
#endif /* CONFIG_AURORA_IO_CACHE_COHERENCY */

static inline void mvOsPrefetch(const void *ptr)
{
	__asm__ __volatile__(
		"pld\t%0"
		:
		: "o" (*(char *)ptr)
		: "cc");
}


/* Flush CPU pipe */
#define CPU_PIPE_FLUSH





/* register manipulations  */

/******************************************************************************
* This debug function enable the write of each register that u-boot access to
* to an array in the DRAM, the function record only MV_REG_WRITE access.
* The function could not be operate when booting from flash.
* In order to print the array we use the printreg command.
******************************************************************************/
/* #define REG_DEBUG */
#if defined(REG_DEBUG)
extern int reg_arry[2048][2];
extern int reg_arry_index;
#endif

/* Marvell controller register read/write macros */
#define MV_REG_VALUE(offset)          \
	(MV_MEMIO32_READ((INTER_REGS_BASE | (offset))))

/* PPv2 specific reg read/write */
#define MV_PP2_CPU0_REG_READ(offset)             \
	(MV_MEMIO_LE32_READ(PP2_CPU0_VIRT_BASE | (offset & 0xffff)))
#define MV_PP2_CPU0_REG_WRITE(offset, val)    \
	MV_MEMIO_LE32_WRITE((PP2_CPU0_VIRT_BASE | (offset & 0xffff)), (val))

#define MV_PP2_CPU1_REG_READ(offset)             \
	(MV_MEMIO_LE32_READ(PP2_CPU1_VIRT_BASE | (offset & 0xffff)))
#define MV_PP2_CPU1_REG_WRITE(offset, val)    \
	MV_MEMIO_LE32_WRITE((PP2_CPU1_VIRT_BASE | (offset & 0xffff)), (val))

#ifdef CONFIG_SMP
#define MV_PP2_REG_READ(offset)	\
	((smp_processor_id() == 0) ? MV_PP2_CPU0_REG_READ(offset) : MV_PP2_CPU1_REG_READ(offset))

#define MV_PP2_REG_WRITE(offset, val)	\
	((smp_processor_id() == 0) ? MV_PP2_CPU0_REG_WRITE(offset, val) : MV_PP2_CPU1_REG_WRITE(offset, val))
#else
#define MV_PP2_REG_READ(offset)	\
	MV_PP2_CPU0_REG_READ(offset)

#define MV_PP2_REG_WRITE(offset, val)	\
	MV_PP2_CPU0_REG_WRITE(offset, val)
#endif

#define MV_REG_READ(offset)             \
	(MV_MEMIO_LE32_READ(INTER_REGS_BASE | (offset)))

#if defined(REG_DEBUG)
#define MV_REG_WRITE(offset, val)    \
	MV_MEMIO_LE32_WRITE((INTER_REGS_BASE | (offset)), (val)); \
	{ \
		reg_arry[reg_arry_index][0] = (INTER_REGS_BASE | (offset));\
		reg_arry[reg_arry_index][1] = (val);\
		reg_arry_index++;\
	}
#else
#define MV_REG_WRITE(offset, val)    \
	MV_MEMIO_LE32_WRITE((INTER_REGS_BASE | (offset)), (val))
#endif

#define MV_REG_BYTE_READ(offset)        \
	(MV_MEMIO8_READ((INTER_REGS_BASE | (offset))))

#if defined(REG_DEBUG)
#define MV_REG_BYTE_WRITE(offset, val)  \
	MV_MEMIO8_WRITE((INTER_REGS_BASE | (offset)), (val)); \
	{ \
		reg_arry[reg_arry_index][0] = (INTER_REGS_BASE | (offset));\
		reg_arry[reg_arry_index][1] = (val);\
		reg_arry_index++;\
	}
#else
#define MV_REG_BYTE_WRITE(offset, val)  \
	MV_MEMIO8_WRITE((INTER_REGS_BASE | (offset)), (val))
#endif

#if defined(REG_DEBUG)
#define MV_REG_BIT_SET(offset, bitMask)                 \
	(MV_MEMIO32_WRITE((INTER_REGS_BASE | (offset)), \
	(MV_MEMIO32_READ(INTER_REGS_BASE | (offset)) | \
	MV_32BIT_LE_FAST(bitMask)))); \
	{ \
		reg_arry[reg_arry_index][0] = (INTER_REGS_BASE | (offset));\
		reg_arry[reg_arry_index][1] = (MV_MEMIO32_READ(INTER_REGS_BASE | (offset)));\
		reg_arry_index++;\
	}
#else
#define MV_REG_BIT_SET(offset, bitMask)                 \
	(MV_MEMIO32_WRITE((INTER_REGS_BASE | (offset)), \
	(MV_MEMIO32_READ(INTER_REGS_BASE | (offset)) | \
	MV_32BIT_LE_FAST(bitMask))))
#endif

#if defined(REG_DEBUG)
#define MV_REG_BIT_RESET(offset, bitMask)                \
	(MV_MEMIO32_WRITE((INTER_REGS_BASE | (offset)), \
	(MV_MEMIO32_READ(INTER_REGS_BASE | (offset)) & \
	MV_32BIT_LE_FAST(~bitMask)))); \
	{ \
		reg_arry[reg_arry_index][0] = (INTER_REGS_BASE | (offset));\
		reg_arry[reg_arry_index][1] = (MV_MEMIO32_READ(INTER_REGS_BASE | (offset)));\
		reg_arry_index++;\
	}
#else
#define MV_REG_BIT_RESET(offset, bitMask)                \
	(MV_MEMIO32_WRITE((INTER_REGS_BASE | (offset)), \
	(MV_MEMIO32_READ(INTER_REGS_BASE | (offset)) & \
	MV_32BIT_LE_FAST(~bitMask))))
#endif

/* Assembly functions */

/*
** MV_ASM_READ_CPU_EXTRA_FEATURES
** Read Marvell extra features register.
*/
#define MV_ASM_READ_EXTRA_FEATURES(x) __asm__ volatile("mrc  p15, 1, %0, c15, c1, 0" : "=r" (x));

/*
** MV_ASM_WAIT_FOR_INTERRUPT
** Wait for interrupt.
*/
#define MV_ASM_WAIT_FOR_INTERRUPT      __asm__ volatile("mcr  p15, 0, r0, c7, c0, 4");


/* ARM architecture APIs */
MV_U32  mvOsCpuRevGet(MV_VOID);
MV_U32  mvOsCpuPartGet(MV_VOID);
MV_U32  mvOsCpuArchGet(MV_VOID);
MV_U32  mvOsCpuVarGet(MV_VOID);
MV_U32  mvOsCpuAsciiGet(MV_VOID);
MV_U32  mvOsCpuThumbEEGet(MV_VOID);

/*  Other APIs  */
void *mvOsIoCachedMalloc(void *osHandle, MV_U32 size, MV_ULONG *pPhyAddr, MV_U32 *memHandle);
void *mvOsIoUncachedMalloc(void *osHandle, MV_U32 size, MV_ULONG *pPhyAddr, MV_U32 *memHandle);
void mvOsIoUncachedFree(void *osHandle, MV_U32 size, MV_ULONG phyAddr, void *pVirtAddr, MV_U32 memHandle);
void mvOsIoCachedFree(void *osHandle, MV_U32 size, MV_ULONG phyAddr, void *pVirtAddr, MV_U32 memHandle);
int  mvOsRand(void);

#endif /* _MVOSABSTRACT_H */
