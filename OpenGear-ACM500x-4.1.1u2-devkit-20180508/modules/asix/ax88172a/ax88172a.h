
#define AX_MONITOR_MODE			0x01
#define AX_MONITOR_LINK			0x02
#define AX_MONITOR_MAGIC		0x04
#define AX_MONITOR_HSFS			0x10

/* AX88172 Medium Status Register values */
#define AX_MEDIUM_FULL_DUPLEX		0x02
#define AX_MEDIUM_TX_ABORT_ALLOW	0x04
#define AX_MEDIUM_FLOW_CONTROL_EN	0x10
#define AX_MCAST_FILTER_SIZE		8
#define AX_MAX_MCAST			64

#define AX_EEPROM_LEN			0x40

#define AX_SWRESET_CLEAR		0x00
#define AX_SWRESET_RR			0x01
#define AX_SWRESET_RT			0x02
#define AX_SWRESET_PRTE			0x04
#define AX_SWRESET_PRL			0x08
#define AX_SWRESET_BZ			0x10
#define AX_SWRESET_IPRL			0x20
#define AX_SWRESET_IPPD			0x40

#define AX88772_IPG0_DEFAULT		0x15
#define AX88772_IPG1_DEFAULT		0x0c
#define AX88772_IPG2_DEFAULT		0x12

#define AX88772A_IPG0_DEFAULT		0x15
#define AX88772A_IPG1_DEFAULT		0x16
#define AX88772A_IPG2_DEFAULT		0x1A

#define AX88772_MEDIUM_FULL_DUPLEX	0x0002
#define AX88772_MEDIUM_RESERVED		0x0004
#define AX88772_MEDIUM_RX_FC_ENABLE	0x0010
#define AX88772_MEDIUM_TX_FC_ENABLE	0x0020
#define AX88772_MEDIUM_PAUSE_FORMAT	0x0080
#define AX88772_MEDIUM_RX_ENABLE	0x0100
#define AX88772_MEDIUM_100MB		0x0200
#define AX88772_MEDIUM_DEFAULT	\
	(AX88772_MEDIUM_FULL_DUPLEX | AX88772_MEDIUM_RX_FC_ENABLE | \
	 AX88772_MEDIUM_TX_FC_ENABLE | AX88772_MEDIUM_100MB | \
	 AX88772_MEDIUM_RESERVED | AX88772_MEDIUM_RX_ENABLE )

#define AX_CMD_SET_SW_MII		0x06
#define AX_CMD_READ_MII_REG		0x07
#define AX_CMD_WRITE_MII_REG		0x08
#define AX_CMD_READ_STM_REG		0x09
#define AX_CMD_SET_HW_MII		0x0a
#define AX_CMD_READ_EEPROM		0x0b
#define AX_CMD_WRITE_EEPROM		0x0c
#define AX_CMD_WRITE_EEPROM_EN		0x0d
#define AX_CMD_WRITE_EEPROM_DIS		0x0e
#define AX_CMD_READ_RX_CTL		0x0f
#define AX_CMD_WRITE_RX_CTL		0x10
#define AX_CMD_READ_IPG012		0x11
#define AX_CMD_WRITE_IPG0		0x12
#define AX_CMD_WRITE_IPG1		0x13
#define AX_CMD_READ_NODE_ID		0x13
#define AX88772_CMD_WRITE_NODE_ID       0x14
#define AX_CMD_WRITE_IPG2		0x14
#define AX_CMD_WRITE_MULTI_FILTER	0x16
//#define AX_CMD_READ_NODE_ID		0x17
#define AX_CMD_READ_PHY_ID		0x19
#define AX_CMD_WRITE_MEDIUM_MODE	0x1b
#define AX_CMD_READ_MONITOR_MODE	0x1c
#define AX_CMD_WRITE_MONITOR_MODE	0x1d
#define AX_CMD_WRITE_GPIOS		0x1f
#define AX_CMD_SW_RESET 		0x20
#define AX_CMD_SW_PHY_STATUS		0x21
#define AX_CMD_SW_PHY_SELECT		0x22

#define REG_LENGTH			2

#define PHY_ID_MASK			0x1f

#define AX_RX_CTL_MFB_16K	0x0300	/* Maximum Frame size 16384 bytes */
#define AX_RX_CTL_MFB_8K	0x0200	/* Maximum Frame size 8192 bytes */
#define AX_RX_CTL_MFB_4K	0x0100	/* Maximum Frame size 4096 bytes */
#define AX_RX_CTL_MFB_2K	0x0000	/* Maximum Frame size 2048 bytes */
#define AX_RX_CTL_START		0x0080	/* Ethernet MAC start */
#define AX_RX_CTL_AP		0x0020	/* Accept physcial address from Multicast array */
#define AX_RX_CTL_AM		0x0010	
#define AX_RX_CTL_AB		0x0008	/* Accetp Brocadcast frames*/
#define AX_RX_CTL_SEP		0x0004	/* Save error packets */	
#define AX_RX_CTL_AMALL		0x0002	/* Accetp all multicast frames */
#define AX_RX_CTL_PRO		0x0001	/* Promiscuous Mode */
#define AX_RX_CTL_STOP		0x0000	/* Stop MAC */

#define AX_MONITOR_MODE 		0x01
#define AX_MONITOR_LINK 		0x02
#define AX_MONITOR_MAGIC		0x04
#define AX_MONITOR_HSFS 		0x10

#define AX_MCAST_FILTER_SIZE		8
#define AX_MAX_MCAST			64
#define AX_INTERRUPT_BUFSIZE		8

#define AX_EEPROM_LEN			0x40
#define AX_EEPROM_MAGIC 		0xdeadbeef
#define EEPROMMASK			0x7f

#define AX_SWRESET_CLEAR		0x00
#define AX_SWRESET_RR			0x01
#define AX_SWRESET_RT			0x02
#define AX_SWRESET_PRTE 		0x04
#define AX_SWRESET_PRL			0x08
#define AX_SWRESET_BZ			0x10
#define AX_SWRESET_IPRL 		0x20
#define AX_SWRESET_IPPD 		0x40

#define AX_SS_MII			(0x01 << 2)
#define AX_SS_RevMII			(0x10 << 2)
#define AX_SS_RevRMII			(0x11 << 2)
#define AX_SS_EN			(0x01 << 4)

#define AX88772_IPG0_DEFAULT		0x15
#define AX88772_IPG1_DEFAULT		0x0c
#define AX88772_IPG2_DEFAULT		0x12

/* GPIO REGISTER */
#define AXGPIOS_GPO0EN			0X01
#define AXGPIOS_GPO0			0X02
#define AXGPIOS_GPO1EN			0X04
#define AXGPIOS_GPO1			0X08
#define AXGPIOS_GPO2EN			0X10
#define AXGPIOS_GPO2			0X20
#define AXGPIOS_RSE			0X80

#define AX_CBW_CMD_SPI_CR		0x00
#define AX_CBW_CMD_SPI_ISR		0x01
#define AX_CBW_CMD_SPI_TBR		0x02
#define AX_CBW_CMD_SPI_RBR		0x03
#define AX_CBW_CMD_SPI_STRBR		0x04

#define AX_CBW_CMD_I2C_CR		0x00
#define AX_CBW_CMD_I2C_SR		0x01
#define AX_CBW_CMD_I2C_CMD		0x02
#	define AX_I2C_CMD_STA		0x80
#	define AX_I2C_CMD_STO		0x40
#	define AX_I2C_CMD_RD		0x20
#	define AX_I2C_CMD_WR		0x10
#	define AX_I2C_CMD_MG		0x08
#	define AX_I2C_CMD_SG		0x02
#	define AX_I2C_CMD_RLS		0x01

#define AX_CBW_CMD_UART_CR		0x00
#define AX_CBW_CMD_UART_SR		0x01
#define AX_CBW_CMD_UART_RX		0x02
#define AX_CBW_CMD_UART_TX		0x03

/* interface from the device/framing level "minidriver" to core */
struct driver_info {
	char		*description;

	/* for new devices, use the descriptor-reading code instead */
	int		in;		/* rx endpoint */
	int		out;		/* tx endpoint */

	unsigned long	data;		/* Misc driver specific data */
};

/* interface from usbnet core to each USB networking link we handle */
struct ax_private {
	/* housekeeping */
	struct usb_device	*udev;
	struct usb_interface	*intf;
	struct driver_info	*driver_info;
	const char		*driver_name;
	void			*driver_priv;
	wait_queue_head_t	*wait;

	/* i/o info: pipes etc */
	unsigned		pkt_in, pkt_out;
	unsigned		ser_in, ser_out;
	unsigned long		ser_in_cnt, ser_out_cnt;
	struct usb_host_endpoint *status;
	unsigned		maxpacket;

	/* protocol/interface state */
	struct net_device	*net;
	struct net_device_stats	stats;
	int			msg_enable;
	struct mii_if_info	mii;

	/* various kinds of pending driver work */
	struct sk_buff_head	rxq;
	struct sk_buff_head	txq;
	struct sk_buff_head	done;
	struct urb		*interrupt;

	struct timer_list	delay;
	struct tasklet_struct	bh;
	struct work_struct	kevent;
	unsigned long		flags;
#		define EVENT_TX_HALT		0
#		define EVENT_RX_HALT		1
#		define EVENT_RX_MEMORY		2
#		define EVENT_LINK_RESET		3

	u8			multi_filter[AX_MCAST_FILTER_SIZE];
	u16			EEPromData;
	u16			Vendor;
#		define VENDOR_ASIX		(0 << 6)
#		define VENDOR_DIMOTO		(1 << 6)
#		define VENDOR_MASK		0x03C0

	u16			OpMode;
#		define OP_MODE_MAC		(0x00 << 14)
#		define OP_MODE_PHY		(0x01 << 14)
#		define OP_MODE_DPHY		(0x02 << 14)
#		define OP_MODE_FIBER		(0x03 << 14)
#		define OP_MODE_MASK		0xC000

	u8			FixAt100F;

	struct urb		*serial;
	struct ax88172a_cbw_data *cbw;
	struct ax88172a_csw_data *csw;
};

struct ax88172_int_data {
	u16	res1;
	u8	link;
	u16	res2;
	u8	status;
	u16	res3;
} __attribute__ ((packed));

struct ax88172a_cbw_data {
	u16	sig;
#define CBW_SIG		0x5547

#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	device_code:2,
#define CBW_DEV_I2C		0x00
#define CBW_DEV_UART		0x01
#define CBW_DEV_SPI		0x02

		res1:2,
		reg_addr:4;

	u8	tag:4,
		dir:1,
#define CBW_DIR_IN		0x01
#define CBW_DIR_OUT		0x00

		zero:1,
		res2:1,
		rst:1;

#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	reg_addr:4,
		res1:2,
		device_code:2;
#define CBW_DEV_I2C		0x00
#define CBW_DEV_UART		0x01
#define CBW_DEV_SPI		0x02

	u8	rst:1,
		res2:1,
		zero:1,
		dir:1,
#define CBW_DIR_IN		0x01
#define CBW_DIR_OUT		0x00

		tag:4;
#endif

} __attribute__ ((packed));

struct ax88172a_csw_data {
	u16	sig;
#define CSW_SIG		0x4755

#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	device_code:2,
#define CSW_DEV_I2C		0x00
#define CSW_DEV_UART		0x01
#define CSW_DEV_SPI		0x02

		res1:2,
		reg_addr:4;

	u8	tag:4,
		status:2,
#define CSW_STATUS_PASS		0x00
#define CSW_STATUS_FAILED	0x01
#define CSW_STATUS_ERROR	0x02

		res2:2;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	reg_addr:4,
		res1:2,
		device_code:2;
#define CSW_DEV_I2C		0x00
#define CSW_DEV_UART		0x01
#define CSW_DEV_SPI		0x02

	u8	res2:2,
		status:2,
#define CSW_STATUS_PASS		0x00
#define CSW_STATUS_FAILED	0x01
#define CSW_STATUS_ERROR	0x02

		tag:4;
#endif
} __attribute__ ((packed));

/* we record the state for each of our queued skbs */
enum skb_state {
	illegal = 0,
	tx_start, tx_done,
	rx_start, rx_done, rx_cleanup
};

struct skb_data {	/* skb->cb is one of these */
	struct urb		*urb;
	struct ax_private	*ax_local;
	enum skb_state		state;
	size_t			length;
};

/* messaging support includes the interface name, so it must not be
 * used before it has one ... notably, in minidriver bind() calls.
 */
#ifdef DEBUG
#define devdbg(ax_local, fmt, arg...) \
	printk(KERN_DEBUG "%s: " fmt "\n" , (ax_local)->net->name , ## arg)
#else
#define devdbg(ax_local, fmt, arg...) do {} while(0)
#endif

#define deverr(ax_local, fmt, arg...) \
	printk(KERN_ERR "%s: " fmt "\n" , (ax_local)->net->name , ## arg)
#define devwarn(ax_local, fmt, arg...) \
	printk(KERN_WARNING "%s: " fmt "\n" , (ax_local)->net->name , ## arg)

#define devinfo(ax_local, fmt, arg...) \
	printk(KERN_INFO "%s: " fmt "\n" , (ax_local)->net->name , ## arg); \

