/* ---------------------------------------------------------------------------
          Copyright (c) 2007-2008 Micrel, Inc.  All rights reserved.
   ---------------------------------------------------------------------------

    dummy phy functions - for sitting behind a switch.

   ---------------------------------------------------------------------------
*/


#include "target.h"
#include "hardware.h"
#include "ks_phy.h"


/*
    HardwareGetCableStatus

    Description:
        This routine is used to get the cable status.

    Parameters:
        PHARDWARE pHardware
            Pointer to hardware instance.

        void* pBuffer
            Buffer to store the cable status.

    Return (None):
*/

static void Dummy_GetCableStatus (
    PHARDWARE pHardware,
    void*     pBuffer )
{
    UINT* pulData = ( UINT* ) pBuffer;

    UINT   ulLength[ 5 ];
    UCHAR  bStatus[ 5 ];

    memset( ulLength, 0, sizeof( ulLength ));
    memset( bStatus, 0, sizeof( bStatus ));
    bStatus[ 4 ] = CABLE_GOOD;
    ulLength[ 4 ] = 1;
    bStatus[ 0 ] = CABLE_GOOD;
    ulLength[ 0 ] = 1;
    bStatus[ 1 ] = CABLE_GOOD;
    ulLength[ 1 ] = 1;
    /* Overall status */
    *pulData++ = ulLength[ 4 ];
    *pulData++ = bStatus[ 4 ];

    /* Pair 1-2 */
    *pulData++ = ulLength[ 0 ];
    *pulData++ = bStatus[ 0 ];

    /* Pair 3-6 */
    *pulData++ = ulLength[ 1 ];
    *pulData++ = bStatus[ 1 ];

    /* Pair 4-5 */
    *pulData++ = 0;
    *pulData++ = CABLE_UNKNOWN;

    /* Pair 7-8 */
    *pulData++ = 0;
    *pulData++ = CABLE_UNKNOWN;

}  /* HardwareGetCableStatus */


/*
    HardwareGetLinkStatus

    Description:
        This routine is used to get the link status.

    Parameters:
        PHARDWARE pHardware
            Pointer to hardware instance.

        void* pBuffer
            Buffer to store the link status.

    Return (None):
*/

static void Dummy_GetLinkStatus (
    PHARDWARE pHardware,
    void*     pBuffer )
{
    UINT*  pulData = ( UINT* ) pBuffer;
    UINT   ulStatus;
    USHORT Capable;
    USHORT Data;
    USHORT Status;

    ulStatus = 1;

    /* Link status */
    *pulData++ = ulStatus;
    ulStatus = 1000000;
    /* Link speed */
    *pulData++ = ulStatus;
    ulStatus = STATUS_FULL_DUPLEX;

    /* Duplex mode with crossover and reversed polarity */
    *pulData++ = ulStatus;

    ulStatus |= LINK_100MBPS_FULL;
    ulStatus |= LINK_SYM_PAUSE;

    /* Capability */
    *pulData++ = ulStatus;

    ulStatus = 0;

    ulStatus |= LINK_AUTO_POLARITY;

    /* Auto-Negotiation advertisement */
    *pulData++ = ulStatus;

    ulStatus = 0;
    ulStatus |= LINK_100MBPS_FULL;

    /* Link parnter capabilities */
    *pulData++ = ulStatus;
}  /* HardwareGetLinkStatus */


/*
    HardwareSetCapabilities

    Description:
        This routine is used to set the link capabilities.

    Parameters:
        PHARDWARE pHardware
            Pointer to hardware instance.

        UINT ulCapabilities
            A set of flags indicating different capabilities.

    Return (None):
*/

static void Dummy_SetCapabilities (
    PHARDWARE pHardware,
    UINT      ulCapabilities )
{
	return;
}  /* HardwareSetCapabilities */


/*
    HardwareGetLinkSpeed

    Description:
        This routine reads PHY registers to determine the current link status
        of the switch ports.

    Parameters:
        PHARDWARE pHardware
            Pointer to hardware instance.

    Return (None):
*/

static void Dummy_GetLinkSpeed (
    PHARDWARE pHardware )
{
    USHORT wLinkStatus;
    USHORT wLocal;
    USHORT wRemote;
    UINT   data;
    int    change = FALSE;

    if ( MediaStateConnected != pHardware->m_ulHardwareState ) {
        change = TRUE;
        pHardware->m_ulHardwareState = MediaStateConnected;
    }

    if ( change )
    {
        if ( MediaStateConnected == pHardware->m_ulHardwareState )
        {
            pHardware->m_ulTransmitRate = 1000000;
	    pHardware->m_ulDuplex = 2;

            pHardware->m_wAdvertised = PHY_AUTO_NEG_100BTX_FD & PHY_LINK_SUPPORT;
            pHardware->m_wLinkPartner = PHY_AUTO_NEG_100BTX_FD & PHY_LINK_SUPPORT;

            HW_READ_DWORD( pHardware, REG_DMA_MISC_CFG, &data );
            data &= ~( MISC_PORT_1000M | MISC_PORT_100M | MISC_PORT_FD );
            if ( 10000000 == pHardware->m_ulTransmitRate )
                data |= MISC_PORT_1000M;
            else if ( 1000000 == pHardware->m_ulTransmitRate )
                data |= MISC_PORT_100M;
            if ( 2 == pHardware->m_ulDuplex )
                data |= MISC_PORT_FD;
            data |= 0x180;
            HW_WRITE_DWORD( pHardware, REG_DMA_MISC_CFG, data );

        }
    }
}  /* HardwareGetLinkSpeed */


/*
    HardwareSetLinkSpeed

    Description:
        This routine sets the link speed of the switch ports.

    Parameters:
        PHARDWARE pHardware
            Pointer to hardware instance.

    Return (None):
*/

static void Dummy_SetLinkSpeed (
    PHARDWARE pHardware )
{
}  /* HardwareSetLinkSpeed */


static void Dummy_SetupPhy (
    PHARDWARE pHardware )
{
}  /* HardwareSetupPhy */


static struct hw_fn dummy_hw = {
    .fnHardwareGetLinkSpeed = Dummy_GetLinkSpeed,
    .fnHardwareSetLinkSpeed = Dummy_SetLinkSpeed,
    .fnHardwareSetupPhy = Dummy_SetupPhy,

    .fnHardwareGetCableStatus = Dummy_GetCableStatus,
    .fnHardwareGetLinkStatus = Dummy_GetLinkStatus,
    .fnHardwareSetCapabilities = Dummy_SetCapabilities,
};

int Dummy_InitPhy( void ) {
	u16 chip_id;
	HardwareReadPhy(6, 1, &chip_id);
	chip_id = chip_id >> 4;
	switch(chip_id) {
		case 0x4:
			printk("Switch KSZ8895MQ Found\n");
			break;
		case 0x5:
			printk("Switch KSZ8895TMQ Found\n");
			break;
		case 0x6:
			printk("Switch KSZ8995RQ Found\n");
			break;
		default:
			printk("Unknown Switch Found\n");
			break;
	}
	HardwareWritePhy(6, 1, 0x1);
	ks8692_fn = &dummy_hw;
	return TRUE;
}
