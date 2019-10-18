/*
 * ASIX AX8817X based USB 2.0 Ethernet Devices
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
 * Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
 * Copyright (c) 2002-2003 TiVo Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

//#define	DEBUG			// error path messages, extra info
//#define	VERBOSE			// more; success messages

#include <linux/version.h>
#ifdef	CONFIG_USB_DEBUG
#   define DEBUG
#endif
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>

#include "ax88172a.h"
#include "ax88172a_ioctl.h"

#define DRV_VERSION	"1.2.0"

static char version[] =
KERN_INFO "ASIX AX88172A USB To Ethernet Adapter:v" DRV_VERSION 
	" " __TIME__ " " __DATE__ "\n"
KERN_INFO "    http://www.asix.com.tw";

#define RX_FORCE_ALIGN
#undef  RX_FORCE_ALIGN

#define RX_CTL_MFB_SIZE		AX_RX_CTL_MFB_2K
#define RX_URB_SIZE		(2048 << (RX_CTL_MFB_SIZE >> 8))

/*
 * Nineteen USB 1.1 max size bulk transactions per frame (ms), max.
 * Several dozen bytes of IPv4 data can fit in two such transactions.
 * One maximum size Ethernet packet takes twenty four of them.
 * For high speed, each frame comfortably fits almost 36 max size
 * Ethernet packets (so queues should be bigger).
 *
 * REVISIT qlens should be members of 'struct usbnet'; the goal is to
 * let the USB host controller be busy for 5msec or more before an irq
 * is required, under load.  Jumbograms change the equation.
 */
#define RX_MAX_QUEUE_MEMORY (60 * 1518)
#define	RX_QLEN(ax_local) (((ax_local)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/RX_URB_SIZE) : 4)
#define	TX_QLEN(ax_local) (((ax_local)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/ \
			((ax_local)->net->mtu + \
			(ax_local)->net->hard_header_len)) : 4)

// throttle rx/tx briefly after some faults, so khubd might disconnect()
// us (it polls at HZ/4 usually) before we report too many false errors.
#define THROTTLE_JIFFIES	(HZ/8)

#define UNLINK_TIMEOUT_MS	3

static int ax88172a_link_reset(struct ax_private *ax_local);
void ax88172a_defer_kevent (struct ax_private *ax_local, int work);
static void ax88172a_defer_bh(struct ax_private *ax_local, struct sk_buff *skb,
			struct sk_buff_head *list);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,14)
static void ax88172a_rx_submit (struct ax_private *ax_local, struct urb *urb,
			gfp_t flags);
#else
static void ax88172a_rx_submit (struct ax_private *ax_local, struct urb *urb,
			int flags);
#endif
static void ax88172a_skb_return (struct ax_private *ax_local,
			struct sk_buff *skb);

static const char driver_name [] = "ax88172a";

/* use ethtool to change the level for any given device */
static int msg_level = -1;
module_param (msg_level, int, 0);
MODULE_PARM_DESC (msg_level, "Override default message level");

static int ax8817x_read_cmd(struct ax_private *ax_local, u8 cmd, u16 value,
			    u16 index, u16 size, void *data)
{
	return usb_control_msg(
		ax_local->udev,
		usb_rcvctrlpipe(ax_local->udev, 0),
		cmd,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		data,
		size,
		USB_CTRL_GET_TIMEOUT);
}

static int ax8817x_write_cmd(struct ax_private *ax_local, u8 cmd, u16 value,
			     u16 index, u16 size, void *data)
{
	return usb_control_msg(
		ax_local->udev,
		usb_sndctrlpipe(ax_local->udev, 0),
		cmd,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		data,
		size,
		USB_CTRL_SET_TIMEOUT);
}

static int ax8817x_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct ax_private *ax_local = netdev_priv(netdev);
	u16 res;
	u8 buf[1];

	ax8817x_write_cmd (ax_local, AX_CMD_SET_SW_MII, 0, 0, 0, &buf);
	ax8817x_read_cmd (ax_local, AX_CMD_READ_MII_REG, phy_id,
				(__u16)loc, 2, (u16 *)&res);
	ax8817x_write_cmd (ax_local, AX_CMD_SET_HW_MII, 0, 0, 0, &buf);

	return res & 0xffff;
}

/* same as above, but converts resulting value to cpu byte order */
static int ax8817x_mdio_read_le(struct net_device *netdev, int phy_id, int loc)
{
	return le16_to_cpu (ax8817x_mdio_read (netdev, phy_id, loc));
}

static void
ax8817x_mdio_write(struct net_device *netdev, int phy_id, int loc, int val)
{
	struct ax_private *ax_local = netdev_priv(netdev);
	u16 res = val;
	u8 buf[1];

	ax8817x_write_cmd(ax_local, AX_CMD_SET_SW_MII, 0, 0, 0, &buf);
	ax8817x_write_cmd(ax_local, AX_CMD_WRITE_MII_REG, phy_id,
				(__u16)loc, 2, (u16 *)&res);
	ax8817x_write_cmd(ax_local, AX_CMD_SET_HW_MII, 0, 0, 0, &buf);
}

/* same as above, but converts new value to le16 byte order before writing */
static void
ax8817x_mdio_write_le(struct net_device *netdev, int phy_id, int loc, int val)
{
	ax8817x_mdio_write( netdev, phy_id, loc, cpu_to_le16(val) );
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void ax8817x_async_cmd_callback(struct urb *urb, struct pt_regs *regs)
#else
static void ax8817x_async_cmd_callback(struct urb *urb)
#endif
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;

	if (urb->status < 0)
		printk(KERN_ERR "ax8817x_async_cmd_callback() failed with %d",
			urb->status);

	kfree(req);
	usb_free_urb(urb);
}

static int ax8817x_set_mac_addr (struct net_device *net, void *p)
{
	struct sockaddr *addr = p;
	struct ax_private *ax_local = netdev_priv(net);

	memcpy (net->dev_addr, addr->sa_data, ETH_ALEN);

	/* Set the MAC address */
	return ax8817x_write_cmd (ax_local, AX88772_CMD_WRITE_NODE_ID,
			0, 0, ETH_ALEN, net->dev_addr);

}

static void
ax8817x_write_cmd_async(struct ax_private *ax_local, u8 cmd, u16 value,
				u16 index, u16 size, void *data)
{
	struct usb_ctrlrequest *req;
	int status;
	struct urb *urb;

	if ((urb = usb_alloc_urb(0, GFP_ATOMIC)) == NULL) {
		devdbg(ax_local, "Error allocating URB in write_cmd_async!");
		return;
	}

	if ((req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC)) == NULL) {
		deverr(ax_local, "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, ax_local->udev,
			     usb_sndctrlpipe(ax_local->udev, 0),
			     (void *)req, data, size,
			     ax8817x_async_cmd_callback, req);

	if((status = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		deverr(ax_local,
			"Error submitting the control message: status=%d",
			status);
		kfree(req);
		usb_free_urb(urb);
	}
}

static int
ax88172a_write_cbw_cmd (struct ax_private *ax_local, u8 device, u8 reg_addr,
			u8 dir)
{
	int ret;
	int actual_length;

	memset (ax_local->cbw, 0, sizeof (*(ax_local->cbw)));

	ax_local->cbw->sig = cpu_to_le16(CBW_SIG);
	ax_local->cbw->device_code = device;
	ax_local->cbw->reg_addr = reg_addr;
	ax_local->cbw->tag = 0;
	ax_local->cbw->dir = dir;

	ret = usb_bulk_msg (
		ax_local->udev,
		ax_local->ser_out,
		ax_local->cbw,
		sizeof (*(ax_local->cbw)),
		&actual_length,
		USB_CTRL_SET_TIMEOUT * 10);

	ax_local->ser_out_cnt++;

	if (!ret)
		return actual_length;
	else
		return (0);
}

static struct ax88172a_csw_data *
ax88172a_read_csw_data (struct ax_private *ax_local)
{
	int ret;
	int actual_length;

	ret = usb_bulk_msg (
		ax_local->udev,
		ax_local->ser_in,
		ax_local->csw,
		sizeof (*(ax_local->csw)),
		&actual_length,
		USB_CTRL_GET_TIMEOUT);

	ax_local->ser_in_cnt++;

	if (!ret)
		return ax_local->csw;
	else
		return NULL;
}

static int
ax88172a_write_ser_data (struct ax_private *ax_local, void *buf, int len)
{
	int ret;
	int actual_length;

	ret = usb_bulk_msg (
		ax_local->udev,
		ax_local->ser_out,
		buf,
		len,
		&actual_length,
		USB_CTRL_SET_TIMEOUT);

	ax_local->ser_out_cnt++;

	if (!ret) {
		return actual_length;
	}
	else
		return (0);
}

static int
ax88172a_read_ser_data (struct ax_private *ax_local, void *buf, int len)
{
	int ret;
	int actual_length;

	ret = usb_bulk_msg (
		ax_local->udev,
		ax_local->ser_in,
		buf,
		len,
		&actual_length,
		USB_CTRL_GET_TIMEOUT);

	ax_local->ser_in_cnt++;

	if (!ret)
		return actual_length;
	else
		return (0);
}

static int
ax88172a_issue_serial_cmd (struct ax_private *ax_local, u8 device, u8 cmd,
				u8 dir, u8 *buf, int len)
{
	struct ax88172a_csw_data *csw;

	/* Send CBW command */
	if (ax88172a_write_cbw_cmd (ax_local, device, cmd, dir) == 0) {
		printk("Failed to write the CBW command\n");
		return -1;
	}

	if (dir == CBW_DIR_OUT) {
		if (ax88172a_write_ser_data (ax_local, buf, len) == 0) {
			printk("Failed to write the CBW data\n");
			return -1;
		}
	} else {
		if (ax88172a_read_ser_data (ax_local, buf, len) == 0) {
			printk("Failed to write the CBW data\n");
			return -1;
		}
	}

	/* Read CSW data */
	csw = ax88172a_read_csw_data (ax_local);
	if (!csw) {
		printk("Failed to read the CSW data\n");
		return -1;
	}

	if (csw->status) {
		printk ("Dump CSW data\n");
		printk ("csw->sig = 0x%04x\n", csw->sig);
		printk ("csw->device_code = 0x%02x\n", csw->device_code);
		printk ("csw->reg_addr = 0x%02x\n", csw->reg_addr);
		printk ("csw->tag = 0x%02x\n", csw->tag);
		printk ("csw->status = 0x%02x\n", csw->status);
		return -1;
	}

	return 0;
}

static int
ax88172a_spi_check_complete (struct ax_private *ax_local)
{
	u8 buf;

	/* Waiting for transfer complete */
	do {
		/* Read SPIISR to clear STCF. */
		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
			AX_CBW_CMD_SPI_ISR, CBW_DIR_IN, &buf, 1)) {
				return -1;
		}
	} while (!(buf & 1));

	return 0;
}

static int
ax88172a_spi_set_config (struct ax_private *ax_local, struct ioctl_spi_cfg *cfg)
{
	u8 buf[5];

	buf[0] = cfg->SPICR;	/* SPICR value */
	buf[1] = cfg->SPIBRR;	/* SPIBRR value */
	buf[2] = cfg->SPISSR;	/* SPISSR value, always set this to 0xFE */
	buf[3] = cfg->SPIIER;	/* SPIIER value */
	buf[4] = cfg->SPISCR;	/* SPISCR value, always set this to 0x01 */

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_CR, CBW_DIR_OUT, buf, 5)) {
			return -1;
	}

	return 0;
}

static int
ax88172a_spi_read_status (struct ax_private *ax_local)
{
	u8 buf[6];

	/* Write 0x00, 0x00, 0x00, and 0x00 to SPITBR. */
	/* Write 0x8F to SPIMCR. */
	memset (buf, 0, 6);
	buf[0] = 0x8F;

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
			return -1;
	}

	/* Read SPIRBR, repeat step 1 and 4 until byte 0 of SPIRBR is 0x01 */
	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_RBR, CBW_DIR_IN, buf, 4)) {
			return -1;
	}

	return buf[0];
}

static int
ax88172a_spi_read_cmd (struct ax_private *ax_local, struct ioctl_spi_cmd *cmd
			, u8 __user *user_cmd, u8 __user *user_data)
{
	u8 buf[6];
	int left;
	int status;

	/* Waiting for slave device ready */
	while (1) {
		status = ax88172a_spi_read_status (ax_local);
		if (status == 0x01)
			break;
		else if (status < 0)	/* command failed */
			return status;
	}

	/* Write Read command and length to slave device*/
	/* Write address 0x00, 0x00, and 0x00 to slave deivce. */
	memset (buf, 0, 6);
	buf[0] = 0x9F;
	buf[4] = 0x4b;	/* 0x4X : read command, 0xXb length */

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
			return -1;
	}

	/* Waiting for transfer complete */
	status = ax88172a_spi_check_complete (ax_local);
	if (status < 0)
		return status;

	/* Waiting for slave device ready */
	while (1) {
		status = ax88172a_spi_read_status (ax_local);
		if (status == 0x01)
			break;
		else if (status < 0)	/* command failed */
			return status;
	}

	/* sned command 0x10 to start */
	/* Write user_cmd to slave device. */
	memset (buf, 0, 6);
	buf[0] = 0x9F;	/* GO_BSY */
	buf[4] = 0x10;

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
			return -1;
	}

	/* Waiting for transfer complete */
	status = ax88172a_spi_check_complete (ax_local);
	if (status < 0)
		return status;

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_RBR, CBW_DIR_IN, buf, 4)) {
			return -1;
	}

	left = 12;
	while (left > 0) {

		/* sned command 0x10 to start */
		/* Write user_cmd to slave device. */
		memset (buf, 0, 6);
		if (left == 1)
			buf[0] = 0x87;	/* GO_BSY */
		else
			buf[0] = 0xDF;	/* GO_BSY | LL */

		/* needs to issue read command at first time */
		if (left == 12)
			buf[4] = 0x10;
		else
			buf[4] = 0x00;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
			AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
				return -1;
		}
	
		/* Waiting for transfer complete */
		status = ax88172a_spi_check_complete (ax_local);
		if (status < 0)
			return status;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
			AX_CBW_CMD_SPI_RBR, CBW_DIR_IN, buf, 4)) {
				return -1;
		}

		if (left == 12) {
			left -= 3;
		} else if (left == 1) {
			left -= 1;
		} else {
			left -= 4;
		}
	}

	return 0;
}

static int
ax88172a_spi_write_cmd (struct ax_private *ax_local, struct ioctl_spi_cmd *cmd
			, u8 __user *user_cmd, u8 __user *user_data)
{
	u8 buf[6];
	int left, status;
	int length;
	int loop;
	u8 i;

	/* Waiting for slave device ready */
	while (1) {
		status = ax88172a_spi_read_status (ax_local);
		if (status == 0x01)
			break;
		else if (status < 0)	/* command failed */
			return status;
	}

	length = (user_cmd[0] & 0x1f) + 1;

	/* Write user_cmd to slave device. */
	buf[0] = 0xDF;	/* GO_BSY | LL */
	for (i = 0; i < 4; i++)
		buf[4 - i] = user_cmd[i];

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
		AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
			return -1;
	}

	/* Waiting for transfer complete */
	status = ax88172a_spi_check_complete (ax_local);
	if (status < 0)
		return status;

	loop = (length + 3) / 4;
	left = length;

	for (loop = 0; loop < ((length + 3) / 4); loop++) {

		if (left <= 4) {
			buf[0] = 0x80 | (left * 8 - 1);
			for (i = 0; i < left; i++)
				buf[left - i] = user_data[loop * 4 + i];

		} else {
			buf[0] = 0xDF;	/* GO_BSY | LL */
			for (i = 0; i < 4; i++)
				buf[4 - i] = user_data[loop * 4 + i];
		}

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_SPI,
			AX_CBW_CMD_SPI_TBR, CBW_DIR_OUT, buf, 5)) {
				return -1;
		}

		/* Waiting for transfer complete */
		status = ax88172a_spi_check_complete (ax_local);
		if (status < 0)
			return status;

		left -= 4;
	}

	return 0;
}

static int
ax88172a_i2c_set_config (struct ax_private *ax_local, struct ioctl_i2c_cfg *cfg)
{
	u8 buf[6];

	buf[0] = (cfg->MSS << 7) | (cfg->SIE << 6) | (cfg->TE << 3)
			| (cfg->SD << 2) | (cfg->I2CEN << 1) | cfg->MIE;
	buf[1] = (u8)(cfg->Clock & 0xFF);
	buf[2] = (u8)((cfg->Clock >> 8) & 0xFF);
	buf[3] = (u8)(cfg->SDA & 0xFF);
	buf[4] = (u8)((cfg->SDA >> 8) & 0xFF);
	buf[5] = 0;

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
		AX_CBW_CMD_I2C_CR, CBW_DIR_OUT, buf, 6)) {
			return -1;
	}

	return 0;
}

static int ax88172a_i2c_check_complete (struct ax_private *ax_local, u8 *buf)
{

	/* Waiting for command complation */
	do {

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
					AX_CBW_CMD_I2C_SR, CBW_DIR_IN,
					buf, 6)) {
			return -1;
		}

	} while ((buf[3] & 0x82));

	return 0;
}

static int
ax88172a_i2c_write_cmd (struct ax_private *ax_local, struct ioctl_i2c_cmd *cmd,
		u8 __user *user_cmd, u8 __user *user_data)
{
	u8 buf[6];

	if (cmd->Direction) {	/* Write operation */

		/* phase 1 */
		memset (buf, 0, 6);

		/* Write one bit left shift of slave address to I2CTR. */
		buf[1] = (u8)(cmd->Address << 1);

		/* Set STA, WR, and MG bits to 1. */
		buf[0] = AX_I2C_CMD_STA | AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		/* phase 2 */
		memset (buf, 0, 6);

		/* Write the one-byte data to I2CTR. */
		buf[1] = user_cmd[0];

		/* Set WR, and MG bits to I2CCR */
		buf[0] = AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		/* phase 3 */
		memset (buf, 0, 6);

		/* Write the one-byte data to I2CTR. */
		buf[1] = user_data[0];

		/* Set STO, WR, and MG bits to I2CCR */
		buf[0] = AX_I2C_CMD_STO | AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

	} else {		/* Read operation */

		/* phase 1 */
		memset (buf, 0, 6);

		/* Write (one bit left shift of slave address) to I2CTR */
		buf[1] = (u8)(cmd->Address << 1);

		/* Set STA, WR, and MG bits to 1 */
		buf[0] = AX_I2C_CMD_STA | AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		/* phase 2 */
		memset (buf, 0, 6);

		/* Write offset of the slave device to read to I2CTR.*/
		buf[1] = user_cmd[0];

		/* Set WR, and MG bits to 1. */
		buf[0] = AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		/* phase 3 */
		memset (buf, 0, 6);
		/* Write (one bit left shift of slave address) and read operation to I2CTR */
		buf[1] = ((u8)(cmd->Address << 1) | 1);

		/* Set STA, WR, and MG bits to 1. */
		buf[0] = AX_I2C_CMD_STA | AX_I2C_CMD_WR | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		/* phase 4 */
		memset (buf, 0, 6);
		/* Set STO, RD, and MG bits to 1. */
		buf[0] = AX_I2C_CMD_STO | AX_I2C_CMD_RD | AX_I2C_CMD_MG;

		if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_I2C,
				AX_CBW_CMD_I2C_CMD, CBW_DIR_OUT, buf, 6)) {
			return -1;
		}

		if (ax88172a_i2c_check_complete (ax_local, buf))
			return -1;

		user_data[0] = buf[1];
	}

	//printk ("offset 0x%02x, data = 0x%02x\n", user_cmd[0], user_data[0]);

	return 0;
}

static int
ax88172a_uart_set_config (struct ax_private *ax_local,
				struct ioctl_uart_cfg *cfg)
{
	u8 buf[6];

	memset (buf, 0, 6);

	buf[0] = cfg->DLLR;
	buf[1] = cfg->DLHR;
	buf[2] = cfg->LCR;
	buf[3] = cfg->IER;
	buf[4] = cfg->FCR;
	buf[5] = cfg->Reserved;

	if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_UART,
			AX_CBW_CMD_UART_CR, CBW_DIR_OUT, buf, 6)) {
		return -1;
	}

	return 0;

}

static int
ax88172a_uart_send (struct ax_private *ax_local, u32 send_len, u8 *data_buf)
{
	u8 buf[6];
	u32 loop, i;
	u16 timeout;
	u8 left;

	do {
		loop = (send_len / 16);
		left = (send_len % 16);

		for (i = 0; i < loop; i++) {

			if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_UART,
					AX_CBW_CMD_UART_TX, CBW_DIR_OUT,
					(data_buf + i * 16), 16)) {
				return -1;
			}

			/* Waiting for tx fifo empty */
			timeout = 0;
			do {
				if (ax88172a_issue_serial_cmd (ax_local,
						CBW_DEV_UART,
						AX_CBW_CMD_UART_SR, CBW_DIR_IN,
						buf, 6)) {
					return -1;
				}

			} while (!(buf[1] & 0x20) && (++timeout < 0xFFFF));
		}

		if (left) {

			if (ax88172a_issue_serial_cmd (ax_local, CBW_DEV_UART,
					AX_CBW_CMD_UART_TX, CBW_DIR_OUT,
					&data_buf[send_len - left], left)) {
				return -1;
			}
		}

	} while (0);

	return 0;

}

static int
ax88172a_uart_rcv (struct ax_private *ax_local, u8 *data_buf)
{
	struct ax88172a_csw_data *csw;
	u8 buf[6];
	u8 length = 0;

	do {
		/* Send CBW command */
		if (ax88172a_write_cbw_cmd (ax_local, CBW_DEV_UART,
			AX_CBW_CMD_UART_SR, CBW_DIR_IN) == 0) {
			printk("Failed to write the CBW command\n");
			break;
		}
	
		/* Read CBW data */
		if (ax88172a_read_ser_data (ax_local, buf, 6) == 0) {
			printk("Failed to read the CBW data\n");
			break;
		}
	
		/* Read CSW data */
		csw = ax88172a_read_csw_data (ax_local);
		if (!csw) {
			printk("Failed to read the CSW data\n");
			break;
		}

		/* Got data in rx fifo */
		if ((buf[1] & 0x01) || (buf[1] & 0x02)) {

			/* Send CBW command */
			if (ax88172a_write_cbw_cmd (ax_local, CBW_DEV_UART,
				AX_CBW_CMD_UART_RX, CBW_DIR_IN) == 0) {
				printk("Failed to write the CBW command\n");
				break;
			}

			/* Read CBW data */
			length = ax88172a_read_ser_data (ax_local, data_buf, 16);
			if (!length) {
				printk("Failed to read the CBW data\n");
				break;
			}

			/* Read CSW data */
			csw = ax88172a_read_csw_data (ax_local);
			if (!csw) {
				printk("Failed to read the CSW data\n");
				break;
			}
		}

	} while (0);

	return length;
}

static int ax8817x_rx_fixup(struct ax_private *ax_local, struct sk_buff *skb)
{
	u8  *head;
	u32  header;
	char *packet;
	struct sk_buff *ax_skb;
	u16 size;

	head = (u8 *) skb->data;
	memcpy(&header, head, sizeof(header));
	le32_to_cpus(&header);

	packet = head + sizeof(header);

	skb_pull(skb, 4);

	while (skb->len > 0) {
		if ((short)(header & 0x0000ffff) !=
		    ~((short)((header & 0xffff0000) >> 16))) {
			deverr(ax_local,"header length data is error");
		}
		/* get the packet length */
		size = (u16) (header & 0x0000ffff);

#ifndef RX_FORCE_ALIGN
		if ((skb->len) - ((size + 1) & 0xfffe) == 0) {
			return 2;
		}
#endif
		if (size > ETH_FRAME_LEN) {
			deverr(ax_local,"invalid rx length %d", size);
			return 0;
		}

#ifndef RX_FORCE_ALIGN
		ax_skb = skb_clone(skb, GFP_ATOMIC);
		if (ax_skb) {
			ax_skb->len = size;
			ax_skb->data = packet;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
			ax_skb->tail = packet + size;
#else
			skb_set_tail_pointer (ax_skb, size);
#endif
			ax88172a_skb_return(ax_local, ax_skb);
		} else {
			return 0;
		}
#else
		ax_skb = alloc_skb (size + NET_IP_ALIGN, GFP_ATOMIC);
		if (ax_skb) {
			skb_reserve (ax_skb, NET_IP_ALIGN);
			memcpy (skb_put (ax_skb, size), packet, size);
			ax88172a_skb_return(ax_local, ax_skb);
		} else {
			deverr(ax_local,
			       "Cannot alloc new skb for the incoming packet");
			return 0;
		}
#endif

		skb_pull(skb, (size + 1) & 0xfffe);

		if (skb->len == 0)
			break;

		head = (u8 *) skb->data;
		memcpy(&header, head, sizeof(header));
		le32_to_cpus(&header);

		packet = head + sizeof(header);
		skb_pull(skb, 4);
	}

	if (skb->len < 0) {
		deverr(ax_local,"invalid rx length %d", skb->len);
		return 0;
	}

	return 1;
}

static struct sk_buff *ax8817x_tx_fixup(struct ax_private *ax_local,
						struct sk_buff *skb)
{
	int padlen;
	int headroom = skb_headroom(skb);
	int tailroom = skb_tailroom(skb);
	u32 packet_len;
	u32 padbytes = 0xffff0000;

	padlen = ((skb->len + 4) % ax_local->maxpacket) ? 0 : 4;

	if ((!skb_cloned(skb))
	    && ((headroom + tailroom) >= (4 + padlen))) {
		if ((headroom < 4) || (tailroom < padlen)) {
			skb->data = memmove(skb->head + 4, skb->data, skb->len);
			skb->tail = skb->data + skb->len;
		}
	} else {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, 4, padlen, GFP_ATOMIC);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb) {
			return NULL;
		}
	}

	skb_push(skb, 4);
	packet_len = (((skb->len - 4) ^ 0x0000ffff) << 16) + (skb->len - 4);
	memcpy(skb->data, &packet_len, sizeof(packet_len));

	if ((skb->len % ax_local->maxpacket) == 0) {
		memcpy( skb->tail, &padbytes, sizeof(padbytes));
		skb_put(skb, sizeof(padbytes));
	}

	return skb;
}

// unlink pending rx/tx; completion handlers do all other cleanup

static int unlink_urbs (struct ax_private *ax_local, struct sk_buff_head *q)
{
	unsigned long		flags;
	struct sk_buff		*skb, *skbnext;
	int			count = 0;

	spin_lock_irqsave (&q->lock, flags);
	for (skb = q->next; skb != (struct sk_buff *) q; skb = skbnext) {
		struct skb_data		*entry;
		struct urb		*urb;
		int			retval;

		entry = (struct skb_data *) skb->cb;
		urb = entry->urb;
		skbnext = skb->next;

		// during some PM-driven resume scenarios,
		// these (async) unlinks complete immediately
		retval = usb_unlink_urb (urb);
		if (retval != -EINPROGRESS && retval != 0)
			devdbg (ax_local, "unlink urb err, %d", retval);
		else
			count++;
	}
	spin_unlock_irqrestore (&q->lock, flags);
	return count;
}

/* Passes this packet up the stack, updating its accounting.
 * Some link protocols batch packets, so their rx_fixup paths
 * can return clones as well as just modify the original skb.
 */
static void
ax88172a_skb_return (struct ax_private *ax_local, struct sk_buff *skb)
{
	int	status;

	skb->dev = ax_local->net;
	skb->protocol = eth_type_trans (skb, ax_local->net);
	ax_local->stats.rx_packets++;
	ax_local->stats.rx_bytes += skb->len;

	if (netif_msg_rx_status (ax_local))
		devdbg (ax_local, "< rx, len %zu, type 0x%x",
			skb->len + sizeof (struct ethhdr), skb->protocol);
	memset (skb->cb, 0, sizeof (struct skb_data));
	status = netif_rx (skb);
	if (status != NET_RX_SUCCESS && netif_msg_rx_err (ax_local))
		deverr (ax_local, "netif_rx status %d", status);

}

#ifdef RX_FORCE_ALIGN
static inline void ax88172a_rx_process (struct ax_private *ax_local,
			struct sk_buff *skb)
{
	if (!ax8817x_rx_fixup (ax_local, skb)) {
		ax_local->stats.rx_errors++;
	}

	skb_queue_tail (&ax_local->done, skb);
}
#else
static inline void ax88172a_rx_process (struct ax_private *ax_local,
			struct sk_buff *skb)
{
	if (!ax8817x_rx_fixup (ax_local, skb))
		goto error;
	// else network stack removes extra byte if we forced a short packet

	if (skb->len)
		ax88172a_skb_return (ax_local, skb);
	else {
		if (netif_msg_rx_err (ax_local))
			devdbg (ax_local, "drop");
error:
		ax_local->stats.rx_errors++;
		skb_queue_tail (&ax_local->done, skb);
	}
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void ax88172a_rx_complete (struct urb *urb, struct pt_regs *regs)
#else
static void ax88172a_rx_complete (struct urb *urb)
#endif
{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct ax_private	*ax_local = entry->ax_local;
	int			urb_status = urb->status;

	skb_put (skb, urb->actual_length);
	entry->state = rx_done;
	entry->urb = NULL;

	switch (urb_status) {
	/* success */
	case 0:
		if (skb->len < ax_local->net->hard_header_len) {
			entry->state = rx_cleanup;
		}

		break;

	/* stalls need manual reset. this is rare ... except that
	 * when going through USB 2.0 TTs, unplug appears this way.
	 * we avoid the highspeed version of the ETIMEOUT/EILSEQ
	 * storm, recovering as needed.
	 */
	case -EPIPE:
		ax_local->stats.rx_errors++;
		ax88172a_defer_kevent (ax_local, EVENT_RX_HALT);
		// FALLTHROUGH

	/* software-driven interface shutdown */
	case -ECONNRESET:		/* async unlink */
	case -ESHUTDOWN:		/* hardware gone */
		if (netif_msg_ifdown (ax_local))
			devdbg (ax_local, "rx shutdown, code %d", urb_status);
		goto block;

	/* we get controller i/o faults during khubd disconnect() delays.
	 * throttle down resubmits, to avoid log floods; just temporarily,
	 * so we still recover when the fault isn't a khubd delay.
	 */
	case -EPROTO:
	case -ETIME:
	case -EILSEQ:
		ax_local->stats.rx_errors++;
		if (!timer_pending (&ax_local->delay)) {
			mod_timer (&ax_local->delay,
				   jiffies + THROTTLE_JIFFIES);
			if (netif_msg_link (ax_local))
				devdbg (ax_local, "rx throttle %d", urb_status);
		}
block:
		entry->state = rx_cleanup;
		entry->urb = urb;
		urb = NULL;
		break;

	/* data overrun ... flush fifo? */
	case -EOVERFLOW:
		ax_local->stats.rx_over_errors++;
		// FALLTHROUGH

	default:
		entry->state = rx_cleanup;
		ax_local->stats.rx_errors++;
		if (netif_msg_rx_err (ax_local))
			devdbg (ax_local, "rx status %d", urb_status);
		break;
	}

	ax88172a_defer_bh(ax_local, skb, &ax_local->rxq);

	if (urb) {
		if (netif_running (ax_local->net)
			&& !test_bit (EVENT_RX_HALT, &ax_local->flags)) {
			ax88172a_rx_submit (ax_local, urb, GFP_ATOMIC);
			return;
		}
		usb_free_urb (urb);
	}
	if (netif_msg_rx_err (ax_local))
		devdbg (ax_local, "no read resubmitted");

}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,14)
static void
ax88172a_rx_submit (struct ax_private *ax_local, struct urb *urb, gfp_t flags)
#else
static void
ax88172a_rx_submit (struct ax_private *ax_local, struct urb *urb, int flags)
#endif
{
	struct sk_buff		*skb;
	struct skb_data		*entry;
	int			retval = 0;
	unsigned long		lockflags;
	size_t			size = RX_URB_SIZE;

#ifndef RX_FORCE_ALIGN
	if ((skb = alloc_skb (size + NET_IP_ALIGN, flags)) == NULL) {
#else
	if ((skb = alloc_skb (size, flags)) == NULL) {
#endif
		if (netif_msg_rx_err (ax_local))
			deverr (ax_local, "no rx skb");
		ax88172a_defer_kevent (ax_local, EVENT_RX_MEMORY);
		usb_free_urb (urb);

		return;
	}

#ifndef RX_FORCE_ALIGN
	skb_reserve (skb, NET_IP_ALIGN);
#endif

	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->ax_local = ax_local;
	entry->state = rx_start;
	entry->length = 0;

	usb_fill_bulk_urb (urb, ax_local->udev, ax_local->pkt_in,
		skb->data, size, ax88172a_rx_complete, skb);

	spin_lock_irqsave (&ax_local->rxq.lock, lockflags);

	if (netif_running (ax_local->net)
			&& netif_device_present (ax_local->net)
			&& !test_bit (EVENT_RX_HALT, &ax_local->flags)) {
		switch (retval = usb_submit_urb (urb, GFP_ATOMIC)) {
		case -EPIPE:
			ax88172a_defer_kevent (ax_local, EVENT_RX_HALT);
			break;
		case -ENOMEM:
			ax88172a_defer_kevent (ax_local, EVENT_RX_MEMORY);
			break;
		case -ENODEV:
			if (netif_msg_ifdown (ax_local))
				devdbg (ax_local, "device gone");
			netif_device_detach (ax_local->net);
			break;
		default:
			if (netif_msg_rx_err (ax_local))
				devdbg (ax_local, "rx submit, %d", retval);
			tasklet_schedule (&ax_local->bh);
			break;
		case 0:
			__skb_queue_tail (&ax_local->rxq, skb);
		}
	} else {
		if (netif_msg_ifdown (ax_local))
			devdbg (ax_local, "rx: stopped");
		retval = -ENOLINK;
	}
	spin_unlock_irqrestore (&ax_local->rxq.lock, lockflags);
	if (retval) {
		dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	}

}

/* some LK 2.4 HCDs oopsed if we freed or resubmitted urbs from
 * completion callbacks.  2.5 should have fixed those bugs...
 */

static void
ax88172a_defer_bh(struct ax_private *ax_local, struct sk_buff *skb,
			struct sk_buff_head *list)
{
	unsigned long		flags;

	spin_lock_irqsave(&list->lock, flags);
	__skb_unlink(skb, list);
	spin_unlock(&list->lock);
	spin_lock(&ax_local->done.lock);
	__skb_queue_tail(&ax_local->done, skb);
	if (ax_local->done.qlen == 1)
		tasklet_schedule(&ax_local->bh);
	spin_unlock_irqrestore(&ax_local->done.lock, flags);
}

// tasklet (work deferred from completions, in_irq) or timer
static void ax88172a_bh (unsigned long param)
{
	struct ax_private	*ax_local = (struct ax_private *) param;
	struct sk_buff		*skb;
	struct skb_data		*entry;

	while ((skb = skb_dequeue (&ax_local->done))) {
		entry = (struct skb_data *) skb->cb;
		switch (entry->state) {
		case rx_done:
			entry->state = rx_cleanup;
			ax88172a_rx_process (ax_local, skb);
			continue;
		case tx_done:
		case rx_cleanup:
			usb_free_urb (entry->urb);
			dev_kfree_skb (skb);
			continue;
		default:
			devdbg (ax_local, "bogus skb state %d", entry->state);
		}
	}

	// waiting for all pending urbs to complete?
	if (ax_local->wait) {
		if ((ax_local->txq.qlen + ax_local->rxq.qlen +
			ax_local->done.qlen) == 0) {
			wake_up (ax_local->wait);
		}

	// or are we maybe short a few urbs?
	} else if (netif_running (ax_local->net)
			&& netif_device_present (ax_local->net)
			&& !timer_pending (&ax_local->delay)
			&& !test_bit (EVENT_RX_HALT, &ax_local->flags)) {
		int	temp = ax_local->rxq.qlen;
		int	qlen = RX_QLEN (ax_local);

		if (temp < qlen) {
			struct urb	*urb;
			int		i;

			// don't refill the queue all at once
			for (i = 0; i < 10 && ax_local->rxq.qlen < qlen; i++) {
				urb = usb_alloc_urb (0, GFP_ATOMIC);
				if (urb != NULL) {
					ax88172a_rx_submit (ax_local, urb,
						GFP_ATOMIC);
				}
			}
			if (temp != ax_local->rxq.qlen &&
				netif_msg_link (ax_local))
				devdbg (ax_local, "rxqlen %d --> %d",
						temp, ax_local->rxq.qlen);
			if (ax_local->rxq.qlen < qlen)
				tasklet_schedule (&ax_local->bh);
		}
		if (ax_local->txq.qlen < TX_QLEN (ax_local))
			netif_wake_queue (ax_local->net);
	}
}

/* some work can't be done in tasklets, so we use keventd
 *
 * NOTE:  annoying asymmetry:  if it's active, schedule_work() fails,
 * but tasklet_schedule() doesn't.  hope the failure is rare.
 */
void ax88172a_defer_kevent (struct ax_private *ax_local, int work)
{
	set_bit (work, &ax_local->flags);
	if (!schedule_work (&ax_local->kevent))
		deverr (ax_local, "kevent %d may have been dropped", work);
	else
		devdbg (ax_local, "kevent %d scheduled", work);
}

/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void kevent (void *data)
#else
static void kevent (struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	struct ax_private	*ax_local = data;
#else
	struct ax_private	*ax_local =
			container_of(work, struct ax_private, kevent);
#endif
	int			status;

	/* usb_clear_halt() needs a thread context */
	if (test_bit (EVENT_TX_HALT, &ax_local->flags)) {
		unlink_urbs (ax_local, &ax_local->txq);
		status = usb_clear_halt (ax_local->udev, ax_local->pkt_out);
		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_tx_err (ax_local))
				deverr (ax_local,
					"can't clear tx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_TX_HALT, &ax_local->flags);
			if (status != -ESHUTDOWN)
				netif_wake_queue (ax_local->net);
		}
	}

	if (test_bit (EVENT_RX_HALT, &ax_local->flags)) {
		unlink_urbs (ax_local, &ax_local->rxq);
		status = usb_clear_halt (ax_local->udev, ax_local->pkt_in);
		if (status < 0
				&& status != -EPIPE
				&& status != -ESHUTDOWN) {
			if (netif_msg_rx_err (ax_local))
				deverr (ax_local, 
					"can't clear rx halt, status %d",
					status);
		} else {
			clear_bit (EVENT_RX_HALT, &ax_local->flags);
			tasklet_schedule (&ax_local->bh);
		}
	}

	/* tasklet could resubmit itself forever if memory is tight */
	if (test_bit (EVENT_RX_MEMORY, &ax_local->flags)) {
		struct urb	*urb = NULL;

		if (netif_running (ax_local->net))
			urb = usb_alloc_urb (0, GFP_KERNEL);
		else
			clear_bit (EVENT_RX_MEMORY, &ax_local->flags);
		if (urb != NULL) {
			clear_bit (EVENT_RX_MEMORY, &ax_local->flags);
			ax88172a_rx_submit (ax_local, urb, GFP_KERNEL);
			tasklet_schedule (&ax_local->bh);
		}
	}

	if (test_bit (EVENT_LINK_RESET, &ax_local->flags)) {
		struct driver_info	*info = ax_local->driver_info;
		int			retval = 0;

		clear_bit (EVENT_LINK_RESET, &ax_local->flags);
		if((retval = ax88172a_link_reset(ax_local)) < 0) {
			devinfo(ax_local, "link reset failed (%d) ax88172a"
				"usb-%s-%s, %s", retval,
				ax_local->udev->bus->bus_name,
				ax_local->udev->devpath,
				info->description);
		}
	}

	if (ax_local->flags)
		devdbg (ax_local, "kevent done, flags = 0x%lx",
			ax_local->flags);
}

static int ax88172a_ioctl (struct net_device *net, struct ifreq *rq, int cmd)
{
	struct ax_private *ax_local = netdev_priv(net);
	union ioctl_cmd ioctl;
	union ioctl_cmd __user *ioctl_rq = (union ioctl_cmd *)rq->ifr_ifru.ifru_data;
	int ret = 0;
	unsigned long i;
	u8 __user *user_cmd = NULL;
	u8 __user *user_data = NULL;

	switch (cmd) {
	case IOCTL_SIGNATURE:
		if ( copy_to_user (rq->ifr_data, ASIX_GID, sizeof (ASIX_GID)) )
			ret = -EFAULT;
		break;

	case IOCTL_SPI_SET_CONFIGURATION:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_spi_set_config (ax_local, &ioctl.spi_cfg);

		break;

	case IOCTL_SPI_SEND_WRITE_COMMAND:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		if ((!ioctl.spi_cmd.OpCodeBufferOffsetlength) ||
			(!ioctl.spi_cmd.DataBufferLength))
			break;

		user_cmd = kmalloc (ioctl.spi_cmd.OpCodeBufferOffsetlength,
					GFP_KERNEL);
		if (!user_cmd) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_cmd, ioctl.spi_cmd.OpCodeBufferOffset,
					ioctl.spi_cmd.OpCodeBufferOffsetlength) ) {
			ret = -EFAULT;
			break;
		}

		user_data = kmalloc (ioctl.spi_cmd.DataBufferLength, GFP_KERNEL);
		if (!user_data) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_data, ioctl.spi_cmd.DataBufferOffset,
					ioctl.spi_cmd.DataBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_spi_write_cmd (ax_local, &ioctl.spi_cmd
			, user_cmd, user_data);

		break;

	case IOCTL_SPI_SEND_READ_COMMAND:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		if ((!ioctl.spi_cmd.OpCodeBufferOffsetlength) ||
			(!ioctl.spi_cmd.DataBufferLength))
			break;

		user_cmd = kmalloc (ioctl.spi_cmd.OpCodeBufferOffsetlength,
					GFP_KERNEL);
		if (!user_cmd) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_cmd, ioctl.spi_cmd.OpCodeBufferOffset,
					ioctl.spi_cmd.OpCodeBufferOffsetlength) ) {
			ret = -EFAULT;
			break;
		}

		user_data = kmalloc (ioctl.spi_cmd.DataBufferLength, GFP_KERNEL);
		if (!user_data) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_data, ioctl.spi_cmd.DataBufferOffset,
					ioctl.spi_cmd.DataBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_spi_read_cmd (ax_local, &ioctl.spi_cmd
			, user_cmd, user_data);

		if ( copy_to_user (ioctl.spi_cmd.DataBufferOffset, user_data,
			ioctl.spi_cmd.DataBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		if ( copy_to_user (ioctl_rq, &ioctl, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

	break;

	case IOCTL_I2C_SET_CONFIGURATION:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_i2c_set_config (ax_local, &ioctl.i2c_cfg);

		break;

	case IOCTL_I2C_SEND_COMMAND:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		if (!ioctl.i2c_cmd.CmdBufferLength)
			break;

		user_cmd = kmalloc (ioctl.i2c_cmd.CmdBufferLength, GFP_KERNEL);
		if (!user_cmd) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_cmd, ioctl.i2c_cmd.CmdBufferOffset,
					ioctl.i2c_cmd.CmdBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		if (!ioctl.i2c_cmd.DataBufferLength)
			break;

		user_data = kmalloc (ioctl.i2c_cmd.DataBufferLength, GFP_KERNEL);
		if (!user_data) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_data, ioctl.i2c_cmd.DataBufferOffset,
					ioctl.i2c_cmd.DataBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_i2c_write_cmd (ax_local, &ioctl.i2c_cmd,
					user_cmd, user_data);

		if ( copy_to_user (ioctl.i2c_cmd.DataBufferOffset, user_data,
					ioctl.i2c_cmd.DataBufferLength) ) {
			ret = -EFAULT;
			break;
		}

		if ( copy_to_user (ioctl_rq, &ioctl, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		break;

	case IOCTL_UART_SET_CONFIGURATION:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		ax88172a_uart_set_config (ax_local, &ioctl.uart_cfg);
		break;

	case IOCTL_UART_RECEIVE:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		user_data = kmalloc (16, GFP_KERNEL);

		if (!user_data) {
			ret = -ENOMEM;
			break;
		}

		ioctl.uart_data.data_len = ax88172a_uart_rcv (ax_local, user_data);

		if ( copy_to_user (ioctl.uart_data.Data, user_data, 16) ) {
			ret = -EFAULT;
			break;
		}

		if ( copy_to_user (ioctl_rq, &ioctl, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		break;

	case IOCTL_UART_SEND:

		if ( copy_from_user (&ioctl, ioctl_rq, sizeof (ioctl)) ) {
			ret = -EFAULT;
			break;
		}

		if (!ioctl.uart_data.data_len)
			break;

		user_data = kmalloc (ioctl.uart_data.data_len, GFP_KERNEL);
		if (!user_data) {
			ret = -ENOMEM;
			break;
		}

		if ( copy_from_user (user_data, ioctl.uart_data.Data,
					ioctl.uart_data.data_len) ) {
			ret = -EFAULT;
			break;
		}

		{
			u32 loop_cnt = ioctl.uart_data.data_len / 16;
			u8 left = ioctl.uart_data.data_len % 16;

			for (i = 0; i < loop_cnt; i++) {
				ax88172a_uart_send (ax_local, 16, user_data);
				user_data += 16;
			}

			if (left) {
				ax88172a_uart_send (ax_local, left, user_data);
			}	

		}

		break;

 	default:
 		ret = generic_mii_ioctl(&ax_local->mii, if_mii(rq), cmd, NULL);
 	}

	if (user_cmd)
		kfree (user_cmd);
	if (user_data)
		kfree (user_data);

 	return ret;
}

static void ax88172a_set_multicast(struct net_device *net)
{
	struct ax_private	*ax_local = netdev_priv(net);
	u16			rx_ctl = AX_RX_CTL_START | AX_RX_CTL_AB |
					 RX_CTL_MFB_SIZE;
	int mc_count;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	mc_count = net->mc_count;
#else
	mc_count = netdev_mc_count (net);
#endif

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= AX_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI || mc_count > AX_MAX_MCAST) {
		rx_ctl |= AX_RX_CTL_AMALL;
	} else if (mc_count == 0) {
		/* just broadcast and directed */
	} else {

		u32 crc_bits;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
		struct dev_mc_list *mc_list = net->mc_list;
		int i;

		memset(ax_local->multi_filter, 0, AX_MCAST_FILTER_SIZE);
		/* Build the multicast hash filter. */
		for (i = 0; i < net->mc_count; i++) {
			crc_bits = ether_crc (ETH_ALEN,
					      mc_list->dmi_addr) >> 26;
			ax_local->multi_filter[crc_bits >> 3] |=
							1 << (crc_bits & 7);
			mc_list = mc_list->next;
		}
#else
		struct netdev_hw_addr *ha;
		memset(ax_local->multi_filter, 0, AX_MCAST_FILTER_SIZE);

		netdev_for_each_mc_addr (ha, net) {
			crc_bits = ether_crc (ETH_ALEN, ha->addr) >> 26;
			ax_local->multi_filter[crc_bits >> 3] |=
							1 << (crc_bits & 7);
		}
#endif

		ax8817x_write_cmd_async(ax_local, AX_CMD_WRITE_MULTI_FILTER,
					0, 0, AX_MCAST_FILTER_SIZE,
					ax_local->multi_filter);

		rx_ctl |= AX_RX_CTL_AM;
	}

	ax8817x_write_cmd_async(ax_local, AX_CMD_WRITE_RX_CTL, rx_ctl,
				0, 0, NULL);
}

static struct net_device_stats *ax88172a_get_stats (struct net_device *net)
{
	struct ax_private	*ax_local = netdev_priv(net);
	return &ax_local->stats;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void ax88172a_tx_complete (struct urb *urb, struct pt_regs *regs)
#else
static void ax88172a_tx_complete (struct urb *urb)
#endif
{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct ax_private	*ax_local = entry->ax_local;

	if (urb->status == 0) {
		ax_local->stats.tx_packets++;
		ax_local->stats.tx_bytes += entry->length;
	} else {
		ax_local->stats.tx_errors++;

		switch (urb->status) {
		case -EPIPE:
			ax88172a_defer_kevent (ax_local, EVENT_TX_HALT);
			break;

		/* software-driven interface shutdown */
		case -ECONNRESET:		// async unlink
		case -ESHUTDOWN:		// hardware gone
			break;

		// like rx, tx gets controller i/o faults during khubd delays
		// and so it uses the same throttling mechanism.
		case -EPROTO:
		case -ETIME:
		case -EILSEQ:
			if (!timer_pending (&ax_local->delay)) {
				mod_timer (&ax_local->delay,
					jiffies + THROTTLE_JIFFIES);
				if (netif_msg_link (ax_local))
					devdbg (ax_local, "tx throttle %d",
							urb->status);
			}

			netif_stop_queue (ax_local->net);
			break;
		default:
			if (netif_msg_tx_err (ax_local))
				devdbg (ax_local, "tx err %d",
					entry->urb->status);
			break;
		}
	}

	urb->dev = NULL;
	entry->state = tx_done;
	ax88172a_defer_bh (ax_local, skb, &ax_local->txq);
}

static int ax88172a_start_xmit (struct sk_buff *skb, struct net_device *net)
{
	struct ax_private	*ax_local = netdev_priv(net);
	int			length;
	int			retval = NET_XMIT_SUCCESS;
	struct urb		*urb = NULL;
	struct skb_data		*entry;
	unsigned long		flags;

	skb = ax8817x_tx_fixup (ax_local, skb);

	if (!skb) {
		if (netif_msg_tx_err (ax_local))
			devdbg (ax_local, "can't tx_fixup skb");
		goto drop;
	}

	length = skb->len;

	if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
		if (netif_msg_tx_err (ax_local))
			devdbg (ax_local, "no urb");
		goto drop;
	}

	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->ax_local = ax_local;
	entry->state = tx_start;
	entry->length = length;

	usb_fill_bulk_urb (urb, ax_local->udev, ax_local->pkt_out,
			   skb->data, skb->len, ax88172a_tx_complete, skb);

	/* don't assume the hardware handles USB_ZERO_PACKET
	 * NOTE:  strictly conforming cdc-ether devices should expect
	 * the ZLP here, but ignore the one-byte packet.
	 */
	if ((length % ax_local->maxpacket) == 0) {
		urb->transfer_buffer_length++;
		if (skb_tailroom(skb)) {
			skb->data[skb->len] = 0;
			__skb_put(skb, 1);
		}
	}

	spin_lock_irqsave (&ax_local->txq.lock, flags);
	switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
	case -EPIPE:
		netif_stop_queue (net);
		ax88172a_defer_kevent (ax_local, EVENT_TX_HALT);
		break;
	default:
		if (netif_msg_tx_err (ax_local))
			devdbg (ax_local, "tx: submit urb err %d", retval);
		break;
	case 0:
		net->trans_start = jiffies;
		__skb_queue_tail (&ax_local->txq, skb);
		if (ax_local->txq.qlen >= TX_QLEN (ax_local))
			netif_stop_queue (net);
	}
	spin_unlock_irqrestore (&ax_local->txq.lock, flags);

	if (retval) {
		if (netif_msg_tx_err (ax_local))
			devdbg (ax_local, "drop, code %d", retval);
drop:
		retval = NET_XMIT_SUCCESS;
		ax_local->stats.tx_dropped++;
		if (skb)
			dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	} else if (netif_msg_tx_queued (ax_local)) {
		devdbg (ax_local, "> tx, len %d, type 0x%x",
			length, skb->protocol);
	}

	return retval;
}

static int ax88172a_open (struct net_device *net)
{
	struct ax_private	*ax_local = netdev_priv(net);
	int			retval;

/*--------------------------------------------------
* 	if (ax_local->serial) {
* 
* 		ax_local->cbw = kmalloc (sizeof (struct ax88172a_cbw_data), GFP_KERNEL);
* 		ax_local->csw = kmalloc (sizeof (struct ax88172a_csw_data), GFP_KERNEL);
* 	
* 		if (!ax_local->cbw || !ax_local->csw)
* 			return -ENOMEM;
* 	
* 		memset (ax_local->cbw, 0, sizeof (struct ax88172a_cbw_data));
* 		memset (ax_local->csw, 0, sizeof (struct ax88172a_csw_data));
* 	}
*--------------------------------------------------*/

	/* start any status interrupt transfer */
	retval = usb_submit_urb (ax_local->interrupt, GFP_KERNEL);
	if (retval < 0) {
		if (netif_msg_ifup (ax_local))
			deverr (ax_local, "intr submit %d", retval);
		return retval;
	}

	netif_start_queue (net);

	// delay posting reads until we're fully open
	tasklet_schedule (&ax_local->bh);

	return retval;
}

static int ax88172a_stop (struct net_device *net)
{
	struct ax_private	*ax_local = netdev_priv(net);
	int			temp;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK (unlink_wakeup);
#else
	DECLARE_WAIT_QUEUE_HEAD (unlink_wakeup);
#endif
	DECLARE_WAITQUEUE (wait, current);

	netif_stop_queue (net);

	if (netif_msg_ifdown (ax_local))
		devinfo (ax_local, "stop stats: rx/tx %ld/%ld, errs %ld/%ld",
			ax_local->stats.rx_packets, ax_local->stats.tx_packets,
			ax_local->stats.rx_errors, ax_local->stats.tx_errors
			);

	// ensure there are no more active urbs
	add_wait_queue (&unlink_wakeup, &wait);
	ax_local->wait = &unlink_wakeup;
	temp = unlink_urbs (ax_local, &ax_local->txq) +
		unlink_urbs (ax_local, &ax_local->rxq);

	// maybe wait for deletions to finish.
	while (!skb_queue_empty(&ax_local->rxq)
			&& !skb_queue_empty(&ax_local->txq)
			&& !skb_queue_empty(&ax_local->done)) {
		msleep(UNLINK_TIMEOUT_MS);
		if (netif_msg_ifdown (ax_local))
			devdbg (ax_local, "waited for %d urb completions",
				temp);
	}
	ax_local->wait = NULL;
	remove_wait_queue (&unlink_wakeup, &wait);

	usb_kill_urb(ax_local->interrupt);

/*--------------------------------------------------
* 	if (ax_local->serial) {
* 
* 		kfree (ax_local->csw);
* 		kfree (ax_local->cbw);
* 
* 		usb_free_urb (ax_local->serial);
* 	}
*--------------------------------------------------*/

	/* deferred work (task, timer, softirq) must also stop.
	 * can't flush_scheduled_work() until we drop rtnl (later),
	 * else workers could deadlock; so make workers a NOP.
	 */
	ax_local->flags = 0;
	del_timer_sync (&ax_local->delay);
	tasklet_kill (&ax_local->bh);

	return 0;
}

static int ax88172a_link_reset(struct ax_private *ax_local)
{
	u16 bmcr;
	u16 mode;

	mode = AX88772_MEDIUM_DEFAULT;

	bmcr = ax8817x_mdio_read_le(ax_local->net, ax_local->mii.phy_id,
				    MII_BMCR);
	if (!(bmcr & BMCR_SPEED100))
		mode &= ~AX88772_MEDIUM_100MB;

	if (!(bmcr & BMCR_FULLDPLX))
		mode &= ~AX88772_MEDIUM_FULL_DUPLEX;

	ax8817x_write_cmd(ax_local, AX_CMD_WRITE_MEDIUM_MODE, mode, 0, 0,
			  NULL);

	return 0;
}

static void ax88172a_status(struct ax_private *ax_local, struct urb *urb)
{
	struct ax88172_int_data *event;
	int link;

	if (urb->actual_length < 8)
		return;

	if ((ax_local->OpMode == OP_MODE_DPHY) ||
	    (ax_local->OpMode == OP_MODE_PHY))
		return;

	event = urb->transfer_buffer;

	if (ax_local->Vendor == VENDOR_DIMOTO)
		link = event->link & 0x02 ? 1 : 0;
	else
		link = event->link & 0x01;

	link = 0x01;

	if (netif_carrier_ok(ax_local->net) != link) {

		if (link) {
			netif_carrier_on(ax_local->net);

			/*
			 * Do not reset the medium mode when link fixed
			 * at 100 full
			 */
			if (!ax_local->FixAt100F) {
				ax88172a_defer_kevent (ax_local,
					EVENT_LINK_RESET);
			}

		} else
			netif_carrier_off(ax_local->net);
		devwarn(ax_local, "ax88172a - Link Status is: %d", link);

	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void ax88172a_intr_complete (struct urb *urb, struct pt_regs *regs)
#else
static void ax88172a_intr_complete (struct urb *urb)
#endif
{
	struct ax_private	*ax_local = urb->context;
	int			status = urb->status;

	switch (status) {
	/* success */
	case 0:
		ax88172a_status (ax_local, urb);
		break;

	/* software-driven interface shutdown */
	case -ENOENT:		/* urb killed */
	case -ESHUTDOWN:	/* hardware gone */
		if (netif_msg_ifdown (ax_local))
			devdbg (ax_local, "intr shutdown, code %d", status);
		return;

	/* NOTE:  not throttling like RX/TX, since this endpoint
	 * already polls infrequently
	 */
	default:
		devdbg (ax_local, "intr status %d", status);
		break;
	}

	if (!netif_running (ax_local->net))
		return;

	memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status != 0 && netif_msg_timer (ax_local))
		deverr(ax_local, "intr resubmit --> %d", status);
}

static int
ax88172a_init_status (struct ax_private *ax_local, struct usb_interface *intf)
{
	char		*buf = NULL;
	unsigned	pipe = 0;
	unsigned	maxp;
	unsigned	period;

	pipe = usb_rcvintpipe (ax_local->udev,
			       ax_local->status->desc.bEndpointAddress &
			       USB_ENDPOINT_NUMBER_MASK);

	maxp = usb_maxpacket (ax_local->udev, pipe, 0);

	/* avoid 1 msec chatter:  min 8 msec poll rate */
	period = max ((int) ax_local->status->desc.bInterval,
		(ax_local->udev->speed == USB_SPEED_HIGH) ? 7 : 3);

	buf = kmalloc (maxp, GFP_KERNEL);
	if (buf) {
		ax_local->interrupt = usb_alloc_urb (0, GFP_KERNEL);
		if (!ax_local->interrupt) {
			kfree (buf);
			return -ENOMEM;
		} else {
			usb_fill_int_urb(ax_local->interrupt, ax_local->udev,
					 pipe, buf, maxp,
					 ax88172a_intr_complete,
					 ax_local, period);

			dev_dbg(&intf->dev,
				"status ep%din, %d bytes period %d\n",
				usb_pipeendpoint(pipe), maxp, period);
		}
	}
	return 0;
}

static int 
ax88172a_get_endpoints(struct ax_private *ax_local, struct usb_interface *intf)
{
	struct usb_host_interface	*alt;
	struct usb_host_endpoint	*in_1 = NULL, *out_1 = NULL;
	struct usb_host_endpoint	*in_2 = NULL, *out_2 = NULL;
	struct usb_host_endpoint	*status = NULL;
	unsigned			ep;

	alt = intf->altsetting;

	/* take the first altsetting with in-bulk + out-bulk;
	 * remember any status endpoint, just in case;
	 * ignore other endpoints and altsetttings.
	 */
	for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
		struct usb_host_endpoint	*e;
		int				intr = 0;

		e = alt->endpoint + ep;
		switch (e->desc.bmAttributes) {
		case USB_ENDPOINT_XFER_INT:
			if (!(e->desc.bEndpointAddress & USB_DIR_IN))
				continue;
			intr = 1;
			/* FALLTHROUGH */
		case USB_ENDPOINT_XFER_BULK:
			break;
		default:
			continue;
		}

		if (e->desc.bEndpointAddress & USB_DIR_IN) {

			if (!intr && (!in_1 || !in_2)) {

				if (!in_1)
					in_1 = e;
				else
					in_2 = e;

			} else if (intr && !status) {
				status = e;
			}

		} else {

			if (!out_1 || !out_2) {

				if (!out_1)
					out_1 = e;
				else
					out_2 = e;

			}
		}
	}

	if (!alt || !in_1 || !out_1)
		return -EINVAL;

	ax_local->pkt_in = usb_rcvbulkpipe (ax_local->udev,
					    in_1->desc.bEndpointAddress &
					    USB_ENDPOINT_NUMBER_MASK);

	ax_local->pkt_out = usb_sndbulkpipe (ax_local->udev,
					     out_1->desc.bEndpointAddress &
					     USB_ENDPOINT_NUMBER_MASK);

/*--------------------------------------------------
* 	if (in_2 && out_2) {
* 		ax_local->ser_in = usb_rcvbulkpipe (ax_local->udev,
* 					in_2->desc.bEndpointAddress &
* 					USB_ENDPOINT_NUMBER_MASK);
* 		ax_local->ser_out = usb_sndbulkpipe (ax_local->udev,
* 					out_2->desc.bEndpointAddress &
* 					USB_ENDPOINT_NUMBER_MASK);
* 	}
*--------------------------------------------------*/

	ax_local->status = status;

	return 0;
}

#ifndef ADVERTISE_PAUSE_CAP
#define ADVERTISE_PAUSE_CAP	0x400
#endif

static int
ax88172a_bind(struct ax_private *ax_local, struct usb_interface *intf)
{
	int ret;
	void *buf;
	u16 EepromData;

	ax88172a_get_endpoints (ax_local, intf);

	buf = kmalloc(6, GFP_KERNEL);
	if(!buf) {
		devdbg (ax_local, "Cannot allocate memory for buffer");
		ret = -ENOMEM;
		goto out1;
	}

	/* reload eeprom data */
	if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_WRITE_GPIOS,
			AXGPIOS_RSE, 0, 0, buf)) < 0)
		goto out2;

	msleep(5);

	if ((ret = ax8817x_read_cmd(ax_local, AX_CMD_READ_EEPROM,
			 0x0017, 0, 2, (void *)(&EepromData))) < 0) {
		devdbg(ax_local, "read SROM address 17h failed: %d", ret);
		goto out2;
	}

	ax_local->EEPromData = le16_to_cpu (EepromData);

	if (ax_local->EEPromData == 0xFFFF) {
		ax_local->Vendor = VENDOR_ASIX;
		ax_local->OpMode = OP_MODE_MAC;
	} else {
		ax_local->Vendor = ax_local->EEPromData & VENDOR_MASK;
		ax_local->OpMode = ax_local->EEPromData & OP_MODE_MASK;
	}

	/* Initialize MII structure */
	ax_local->mii.dev = ax_local->net;
	ax_local->mii.mdio_read = ax8817x_mdio_read;
	ax_local->mii.mdio_write = ax8817x_mdio_write;
	ax_local->mii.phy_id_mask = 0xff;
	ax_local->mii.reg_num_mask = 0xff;

	/* Get the PHY id */
	if ((ret = ax8817x_read_cmd(ax_local, AX_CMD_READ_PHY_ID,
			0, 0, 2, buf)) < 0) {
		devdbg (ax_local, "Error reading PHY ID: %02x", ret);
		goto out2;
	} else if (ret < 2) {
		/* this should always return 2 bytes */
		devdbg (ax_local, "AX_CMD_READ_PHY_ID returned less than"
			"2 bytes: ret=%02x", ret);
		ret = -EIO;
		goto out2;
	}

	/* AX88172A runs in MAC mode and uses internal PHY */
	if ((ax_local->OpMode == OP_MODE_MAC) &&
		(ax_local->Vendor == VENDOR_ASIX)) {

		ax_local->mii.phy_id = *((u8 *)buf + 1);

		if(ax_local->mii.phy_id != 0x10) {
			devdbg (ax_local, "Got wrong PHY ID: %02x",
				ax_local->mii.phy_id);
			ret = -EIO;
			goto out2;
		}

		/*
		* set the embedded Ethernet PHY in power-up
		* mode and operating state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_IPRL, 0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}
	
		/*
		* set the embedded Ethernet PHY in power-down
		* mode and operating state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_IPPD | AX_SWRESET_IPRL,
				0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}
	
		msleep(10);
	
		/*
		* set the embedded Ethernet PHY in power-up mode
		* and operating state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_IPRL, 0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}
	
		msleep(60);
	
		/* 
		* set the embedded Ethernet PHY in power-up mode
		* and reset state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_CLEAR, 0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}
	
		/*
		* set the embedded Ethernet PHY in power-up mode
		* and operating state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_IPRL, 0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}

		ax8817x_mdio_write_le(ax_local->net, ax_local->mii.phy_id,
					MII_BMCR, BMCR_RESET);
		ax8817x_mdio_write_le(ax_local->net, ax_local->mii.phy_id,
				MII_ADVERTISE, (ADVERTISE_ALL | ADVERTISE_CSMA
				| ADVERTISE_PAUSE_CAP));

		mii_nway_restart(&ax_local->mii);

		/* select the external fiber PHY */
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_PHY_SELECT,
				AX_SS_EN | AX_SS_MII,
				0, 0, buf)) < 0) {
			devdbg (ax_local, "Select External MII failed: %d", ret);
		}

		ax_local->mii.phy_id = 0x08;
		ax_local->FixAt100F = 1;

		/* set the uplink port to 100MB FD */
		ax8817x_mdio_write_le(ax_local->net,
					ax_local->mii.phy_id, MII_BMCR,
					(BMCR_SPEED100 | BMCR_FULLDPLX));

	} else if (ax_local->Vendor == VENDOR_DIMOTO) {

		/* Uses external fiber PHY */
		ax_local->mii.phy_id = *((u8 *)buf) & 0x1F;

		/* select the external fiber PHY */
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_PHY_SELECT,
				AX_SWRESET_BZ | AX_SWRESET_PRTE,
				0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}

		/* The Fiber must fixed at speed 100, full duplex */
		ax8817x_mdio_write_le(ax_local->net,
					ax_local->mii.phy_id, MII_BMCR,
					(BMCR_SPEED100 | BMCR_FULLDPLX));

		ax_local->FixAt100F = 1;

	} else if (ax_local->OpMode == OP_MODE_PHY) {

		ax_local->mii.phy_id = *((u8 *)buf + 1);

		if(ax_local->mii.phy_id != 0x10) {
			devdbg (ax_local, "Got wrong PHY ID: %02x",
				ax_local->mii.phy_id);
			ret = -EIO;
			goto out2;
		}

		/*
		* set the embedded Ethernet PHY in power-up
		* mode and operating state.
		*/
		if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_RESET,
				AX_SWRESET_IPRL, 0, 0, buf)) < 0) {
			devdbg (ax_local, "Select PHY #1 failed: %d", ret);
			goto out2;
		}

		if ((ret = ax8817x_read_cmd(ax_local, AX_CMD_SW_PHY_STATUS,
					0, 0, 1, buf)) < 0) {
			devdbg (ax_local, "Failed to read MAC address: %d", ret);
			goto out2;
		}

		if (*((u8 *)buf) & AX_SS_RevMII) {

			if ((ret = ax8817x_write_cmd (ax_local, AX_CMD_SW_PHY_SELECT,
					(AX_SS_RevMII | AX_SS_EN), 0, 0, buf)) < 0) {
				devdbg (ax_local, "Select PHY #1 failed: %d", ret);
				goto out2;
			}

		} else if (*((u8 *)buf) & AX_SS_RevRMII) {

			if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_SW_PHY_SELECT,
					(AX_SS_RevRMII | AX_SS_EN), 0, 0, buf)) < 0) {
				devdbg (ax_local, "Select PHY #1 failed: %d", ret);
				goto out2;
			}

		} else {
			devdbg (ax_local, "Cannot idenfy PHY operation mode");
			goto out2;
		}

		/* Set the embedded Ethernet PHY to the power-saving mode */
		ax8817x_mdio_write_le(ax_local->net, ax_local->mii.phy_id,
					0x08, 0x3900);

		ax_local->FixAt100F = 1;

	} else if (ax_local->OpMode == OP_MODE_DPHY) {
		ax_local->FixAt100F = 1;
	}

	/* stop MAC operation */
	if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_WRITE_RX_CTL,
			AX_RX_CTL_STOP, 0, 0, buf)) < 0) {
		devdbg (ax_local, "Reset RX_CTL failed: %d", ret);
		goto out2;
	}

	/* Get the MAC address */
	memset(buf, 0, ETH_ALEN);
	if ((ret = ax8817x_read_cmd(ax_local, AX_CMD_READ_NODE_ID,
				0, 0, ETH_ALEN, buf)) < 0) {
		devdbg (ax_local, "Failed to read MAC address: %d", ret);
		goto out2;
	}
	memcpy(ax_local->net->dev_addr, buf, ETH_ALEN);

	if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_WRITE_MEDIUM_MODE,
				AX88772_MEDIUM_DEFAULT, 0, 0, buf)) < 0) {
		devdbg (ax_local, "Write medium mode register: %d", ret);
		goto out2;
	}

	if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_WRITE_IPG0,
				AX88772A_IPG0_DEFAULT | AX88772A_IPG1_DEFAULT << 8,
				AX88772A_IPG2_DEFAULT, 0, buf)) < 0) {
		devdbg (ax_local, "Write IPG,IPG1,IPG2 failed: %d", ret);
		goto out2;
	}

	/* Set RX_CTL to default values with 2k buffer, and enable cactus */
	if ((ret = ax8817x_write_cmd(ax_local, AX_CMD_WRITE_RX_CTL,
			(AX_RX_CTL_START | AX_RX_CTL_AB | RX_CTL_MFB_SIZE),
			0, 0, buf)) < 0) {
		devdbg (ax_local, "Reset RX_CTL failed: %d", ret);
		goto out2;
	}

	if ((ax_local->OpMode == OP_MODE_DPHY) && (ax_local->OpMode == OP_MODE_PHY))
		netif_carrier_on (ax_local->net);

out2:
	kfree(buf);
out1:
	return ret;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static const struct net_device_ops ax88172a__netdev_ops = {
	.ndo_get_stats =		ax88172a_get_stats,
	.ndo_start_xmit =		ax88172a_start_xmit,
	.ndo_open =			ax88172a_open,
	.ndo_stop =			ax88172a_stop,
	.ndo_set_rx_mode = 		ax88172a_set_multicast,
	.ndo_do_ioctl =			ax88172a_ioctl,
	.ndo_set_mac_address =		ax8817x_set_mac_addr
};
#endif

int
ax88172a_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct net_device		*net;
	struct ax_private		*ax_local;
	struct usb_host_interface	*interface;
	struct driver_info		*info;
	struct usb_device		*xdev;
	int				status;
	const char			*name;

	printk (version);

	name = udev->dev.driver->name;
	info = (struct driver_info *) prod->driver_info;

	if (!info) {
		dev_dbg (&udev->dev, "blacklisted by %s\n", name);
		return -ENODEV;
	}
	xdev = interface_to_usbdev (udev);
	interface = udev->cur_altsetting;

	usb_get_dev (xdev);

	status = -ENOMEM;

	// set up our own records
	net = alloc_etherdev(sizeof(*ax_local));
	if (!net) {
		pr_debug ("can't kmalloc dev");
		goto out;
	}

	ax_local = netdev_priv(net);
	ax_local->udev = xdev;
	ax_local->intf = udev;
	ax_local->driver_info = info;
	ax_local->driver_name = name;
	ax_local->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);

	skb_queue_head_init (&ax_local->rxq);
	skb_queue_head_init (&ax_local->txq);
	skb_queue_head_init (&ax_local->done);

	ax_local->net = net;

	ax_local->bh.func = ax88172a_bh;
	ax_local->bh.data = (unsigned long) ax_local;

	ax_local->delay.function = ax88172a_bh;
	ax_local->delay.data = (unsigned long) ax_local;
	init_timer (&ax_local->delay);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK (&ax_local->kevent, kevent, ax_local);
#else
	INIT_WORK (&ax_local->kevent, kevent);
#endif 

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	net->get_stats =		ax88172a_get_stats;
	net->hard_start_xmit =		ax88172a_start_xmit;
	net->open =			ax88172a_open;
	net->stop =			ax88172a_stop;
	net->set_multicast_list = 	ax88172a_set_multicast;
	net->do_ioctl =			ax88172a_ioctl;
#else
	net->netdev_ops = &ax88172a__netdev_ops;
#endif

	status = ax88172a_bind (ax_local, udev);
	if (status < 0)
		goto out1;

	status = ax88172a_init_status (ax_local, udev);
	if (status < 0)
		goto out1;

/*--------------------------------------------------
* 	if (ax_local->ser_in && ax_local->ser_out)
* 		if (!(ax_local->serial = usb_alloc_urb (0, GFP_KERNEL)))
* 			goto out1;
*--------------------------------------------------*/

	ax_local->maxpacket = usb_maxpacket (ax_local->udev, ax_local->pkt_out, 1);

	SET_NETDEV_DEV(net, &udev->dev);

	status = register_netdev (net);
	if (status)
		goto out1;

	if (netif_msg_probe (ax_local))
		devinfo (ax_local, "register '%s' at usb-%s-%s, "
			"%02x:%02x:%02x:%02x:%02x:%02x",
			ax_local->driver_info->description,
			xdev->bus->bus_name, xdev->devpath,
			net->dev_addr [0], net->dev_addr [1],
			net->dev_addr [2], net->dev_addr [3],
			net->dev_addr [4], net->dev_addr [5]);

	// ok, it's ready to go.
	usb_set_intfdata (udev, net);

	// start as if the link is up
	netif_device_attach (net);

	return 0;

out1:
	free_netdev(net);
out:
	usb_put_dev(xdev);
	return status;
}

void ax88172a_disconnect (struct usb_interface *intf)
{
	struct usb_device	*xdev;
	struct net_device	*net;
	struct ax_private	*ax_local;

	net = usb_get_intfdata(intf);
	ax_local = netdev_priv(net);

	usb_set_intfdata(intf, NULL);
	if (!net)
		return;

	xdev = interface_to_usbdev (intf);

	if (netif_msg_probe (ax_local))
		devinfo (ax_local, "unregister '%s' usb-%s-%s",
			ax_local->driver_info->description,
			xdev->bus->bus_name, xdev->devpath);

	unregister_netdev (net);

	free_netdev(net);
	usb_put_dev (xdev);

}

static const struct driver_info ax88172a_info = {
	.description = "ASIX AX88172A USB2.0 to Ethernet Adapter",
};

static const struct driver_info dimoto_info = {
	.description = "DiMoto DM-USB USB2.0 to POF Adapter",
};

static const struct usb_device_id products [] = {
{
	// ASIX AX88172A 10/100
        USB_DEVICE (0x0b95, 0x172A),
	.driver_info =	(unsigned long) &ax88172a_info,
},
{
	// ASIX AX88772A 10/100
        USB_DEVICE (0x0b95, 0x772A),
	.driver_info =	(unsigned long) &ax88172a_info,
},
{
	// Dimoto
        USB_DEVICE (0x1E51, 0x1000),
	.driver_info =	(unsigned long) &dimoto_info,
},
	{ },		// END
};
MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver ax88172a_driver = {
	.name =		"ax88172a",
	.id_table =	products,
	.probe =	ax88172a_probe,
	.disconnect =	ax88172a_disconnect,
};

static int __init ax88172a_init(void)
{
 	return usb_register(&ax88172a_driver);
}
module_init(ax88172a_init);

static void __exit ax88172a_exit(void)
{
 	usb_deregister(&ax88172a_driver);
}
module_exit(ax88172a_exit);

//MODULE_AUTHOR("David Hollis");
MODULE_DESCRIPTION("ASIX AX88172A based USB 2.0 Ethernet Devices");
MODULE_LICENSE("GPL");

