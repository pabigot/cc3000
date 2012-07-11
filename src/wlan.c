/*****************************************************************************
*
*  wlan.c  - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

//*****************************************************************************
//
//! \addtogroup wlan_api
//! @{
//
//*****************************************************************************
#include <string.h>
#include <cc3000/wlan.h>
#include <cc3000/hci.h>
#include <cc3000/spi.h>
#include <cc3000/socket.h>
#include <cc3000/evnt_handler.h>
#include <cc3000/os.h>

volatile sSimplLinkInformation tSLInformation;

/* patches type */
#define PATCHES_HOST_TYPE_WLAN_DRIVER   0x01
#define PATCHES_HOST_TYPE_WLAN_FW       0x02
#define PATCHES_HOST_TYPE_BOOTLOADER    0x03

#define SL_SET_SCAN_PARAMS_INTERVAL_LIST_SIZE	(16)
#define SL_SIMPLE_CONFIG_PREFIX_LENGTH (3)
#define ETH_ALEN								(6)
#define MAXIMAL_SSID_LENGTH						(32)

#define SL_PATCHES_REQUEST_DEFAULT		(0)
#define SL_PATCHES_REQUEST_FORCE_HOST	(1)
#define SL_PATCHES_REQUEST_FORCE_NONE	(2)


#define      WLAN_SEC_UNSEC (0)
#define      WLAN_SEC_WEP	(1)
#define      WLAN_SEC_WPA	(2)
#define      WLAN_SEC_WPA2	(3)


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//packed is used for preventing padding before sending the structure over the SPI                       ///
//for every IDE, exist different syntax:          1.   __MSP430FR5739__ for CCS v5                      ///
//                                                2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench    ///
// THIS COMMENT IS VALID FOR EVERY STRUCT DEFENITION!                                                   ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _SimpleLink_Init_Start_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _SimpleLink_Init_Start_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _SimpleLink_Init_Start_t
#endif
{
    unsigned char ulPatchesConfiguration;
}SimpleLink_Init_Start_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_patch_conf_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_patch_conf_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_patch_conf_args_t
#endif
{
    unsigned long uiPatchToConfigure;
    unsigned long uiPatchSource;
}wlan_configure_patch_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_connect_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_connect_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_connect_args_t
#endif
{
    unsigned long ulSsidOffset;
    unsigned long ulSsidLen;
    unsigned long ulSecType;
    unsigned long ulKeyOffset;
    unsigned long ulKeyLen;
    unsigned char  pad[2];
    unsigned char  ucBssid[6];
    unsigned char  payload[1];
}wlan_connect_args_t;



#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _set_connection_policy_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_ioctl_set_connection_policy_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _set_connection_policy_args_t
#endif
{
    unsigned long ulShouldConnectToOpenAp;
    unsigned long ulShouldUseFastConnect;
    unsigned long ulAutoStart;
}wlan_ioctl_set_connection_policy_args_t;



#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_set_mask_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_set_mask_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_set_mask_args_t
#endif
{
    unsigned long mask;
}wlan_set_mask_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_unsec_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_add_profile_unsec_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_unsec_args_t
#endif
{
    unsigned long ulSecType;
    unsigned long ulSsidOffset;
    unsigned long ulSsidLen;
    unsigned char  pad[2];
    unsigned char  ucBssid[6];
    unsigned long ulPriority;
}wlan_add_profile_unsec_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_wep_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_add_profile_wep_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_wep_args_t
#endif
{
    unsigned long ulSecType;
    unsigned long ulSsidOffset;
    unsigned long ulSsidLen;
    unsigned char  pad[2];
    unsigned char  ucBssid[6];
    unsigned long ulPriority;
    unsigned long ulKeyOffset;
    unsigned long ulKeyLen;
    unsigned long ulTxKeyIndex;
}wlan_add_profile_wep_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_wpa_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_add_profile_wpa_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_add_profile_wpa_args_t
#endif
{
    unsigned long ulSecType;
    unsigned long ulSsidOffset;
    unsigned long ulSsidLen;
    unsigned char  pad[2];
    unsigned char  ucBssid[6];
    unsigned long ulPriority;
    unsigned long ulPairwiseCipher;
    unsigned long ulGroupCipher;
    unsigned long ulKeyMgmt;
    unsigned long ulPassphraseOffset;
    unsigned long ulPassphraseLen;
}wlan_add_profile_wpa_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_del_profile_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_ioctl_del_profile_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_del_profile_args_t
#endif
{
    unsigned long index;
}wlan_ioctl_del_profile_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_set_scan_params_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_ioctl_set_scan_params_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_set_scan_params_args_t
#endif
{
	unsigned long uiIntervalListOffset;
	unsigned long uiEnable;
    unsigned long uiMinDwellTime;
	unsigned long uiMaxDwellTime;
	unsigned long uiNumOfProbeResponces;
	unsigned long uiChannelMask;
	unsigned long uiRssiThreshold;
	unsigned long uiSNRThreshold;
	unsigned long uiDefaultTxPower;
	unsigned long auiIntervalList[SL_SET_SCAN_PARAMS_INTERVAL_LIST_SIZE];
}wlan_ioctl_set_scan_params_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_get_scan_results_args_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _wlan_ioctl_get_scan_results_args_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _wlan_ioctl_get_scan_results_args_t
#endif
{
    unsigned long ulScanTimeout;
}wlan_ioctl_get_scan_results_args_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _WLAN_San_Result_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _WLAN_San_Result_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _WLAN_San_Result_t
#endif
{
	//
	// Set to 1 if this entry is occuiped
	//
	unsigned long isValid    : 1;
	long rssi       : 7;	 

	//
	// Holds one of the parsed security-modes
	// SECURITY_OPEN, SECURITY_WEP, SECURITY_WPA, SECURITY_WPA2,
	//
	unsigned long securityMode : 2;
	long ulSsidLen     : 6;

	//
	// The time of which the frame has enterd (in seconds)
	//
	short         frameTime;

	//
	// AP name and MAC address
	//
	char          ssid[MAXIMAL_SSID_LENGTH];
	unsigned char ucBssid[ETH_ALEN];	
}WLAN_San_Result_t;

/**
 * \brief Configure patches 
 *
 * Configure patches source. Three types of patches: wlan 
 * driver, wlan FW and wlan bootloadr. For each one three 
 * options, get from eeprom,  get from host or disable the patch
 *  
 * \param[in]   uiPatchesToConfigure patch type: 
 *       PATCHES_HOST_TYPE_WLAN_DRIVER,
 *       PATCHES_HOST_TYPE_WLAN_FW or
 *       PATCHES_HOST_TYPE_BOOTLOADER
 * \param[in]   uiSource select patch source: 
 *       PATCHES_SOURCE_EEPROM (eeprom), PATCHES_SOURCE_HOST
 *       (host) or PATCHES_SOURCE_NONE (disable)
 * 
 *  
 * \return     None
 *
 * \sa          
 * \note        
 * \warning     
 */

void wlan_ConfigurePatches(unsigned long ulPatchesToConfigure, unsigned long ulSource)
{
    unsigned char *ptr;
	wlan_configure_patch_args_t *params;
	long ret;

    ptr = tSLInformation.pucTxCommandBuffer;
	params = (wlan_configure_patch_args_t *)(ptr + HEADERS_SIZE_CMD);

	params->uiPatchSource = ulSource;
	params->uiPatchToConfigure = ulPatchesToConfigure;
	
	hci_command_send(HCI_CMND_WLAN_CONFIGURE_PATCH, ptr, sizeof(wlan_configure_patch_args_t));
	
	SimpleLinkWaitEvent(HCI_CMND_WLAN_CONFIGURE_PATCH, &ret);
}


//*****************************************************************************
//
//!  SimpleLink_Init_Start
//!
//!  \param  scan_timeout     ...
//!  \param  num_of_entries   ...
//!
//!  \return         status of operation: ESUCCESS or EFAIL
//!
//!  \brief          Gets the WLAN scan operation result
//
//*****************************************************************************
static void SimpleLink_Init_Start(unsigned short usPatchesAvailableAtHost)
{
    unsigned char *ptr;
	SimpleLink_Init_Start_t *params;

    ptr = tSLInformation.pucTxCommandBuffer;
	params = (SimpleLink_Init_Start_t *)(ptr + HEADERS_SIZE_CMD);

	params->ulPatchesConfiguration = (usPatchesAvailableAtHost) ? SL_PATCHES_REQUEST_FORCE_HOST : SL_PATCHES_REQUEST_DEFAULT;
	
	// IRQ Line asserted - start the read buffer size command
	hci_command_send(HCI_CMND_SIMPLE_LINK_START, ptr, sizeof(SimpleLink_Init_Start_t));
	
	SimpleLinkWaitEvent(HCI_CMND_SIMPLE_LINK_START, 0);
}





/**
 * \brief Initialize wlan driver
 *
 * This function sets up wlan driver callbacks and initializes internal data structures.
 *
 *
 * \param[in]   sWlanCB  Asynchronous events callback. 0 no 
 *       event call back.\n
 *       call back parameters:\n 
 *       1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect
 *       event, HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect
 *       event, HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config
 *       done, HCI_EVNT_WLAN_UNSOL_DHCP dhcp report, 
 *       HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR 
 *       HCI_EVNT_WLAN_KEEPALIVE keepalive\n
 *       2)  data: pointer to extra data that received by the event (NULL no data)\n
 *       3)  length: data length\n
 *       Events with extra data:\n
 *       HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask, 4 bytes default gateway, 4 bytes DHCP server and 4 bytes for DNS server\n
 *       HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent, 4 bytes Packets received, 4 bytes Min round time, 4 bytes Max round time and 4 bytes for Avg round time\n
 * \param[in]   sFWPatches  0 no patch or pointer to FW patch 
 * \param[in]   sDriverPatches  0 no patch or pointer to driver 
 *       patch
 * \param[in]   sBootLoaderPatches. 0 no patch or pointer to 
 *       bootloader patch
 * [in]   sReadWlanInterruptPin init callback. the 
 *       callback read wlan interrupt status
 * \param[in]   sWlanInterruptEnable init callback. the callback
 *       enable wlan interrupt
 * \param[in]   sWlanInterruptDisable init callback. the 
 *       callback disable wlan interrupt
 * \param[in]   sWriteWlanPin init callback. the callback write 
 *       value to device pin.
 *  
 *  
 * \return     Zero
 *
 * \sa          wlan_set_event_mask wlan_start wlan_stop
 * \note        
 * \warning     this function must be called before ANY other 
 *              wlan driver function
 */
void wlan_init(		tWlanCB	 	sWlanCB,
	   			tFWPatches sFWPatches,
	   			tDriverPatches sDriverPatches,
	   			tBootLoaderPatches sBootLoaderPatches,
                tWlanReadInteruptPin  sReadWlanInterruptPin,
                tWlanInterruptEnable  sWlanInterruptEnable,
                tWlanInterruptDisable sWlanInterruptDisable,
                tWriteWlanPin         sWriteWlanPin)
{
	tSLInformation.usRxEventOpcode = 0;
	tSLInformation.usNumberOfFreeBuffers = 0;
	tSLInformation.usSlBufferLength = 0;
	tSLInformation.usBufferSize = 0;
	tSLInformation.usRxDataPending = 0;

	tSLInformation.slTransmitDataError = 0;
	tSLInformation.sDriverPatches = sDriverPatches;
	tSLInformation.usEventOrDataReceived = 0;

	tSLInformation.sFWPatches = sFWPatches;
	tSLInformation.sDriverPatches = sDriverPatches;
	tSLInformation.sBootLoaderPatches = sBootLoaderPatches;

    /* init io callback */
    tSLInformation.ReadWlanInterruptPin = sReadWlanInterruptPin;
    tSLInformation.WlanInterruptEnable  = sWlanInterruptEnable;
    tSLInformation.WlanInterruptDisable = sWlanInterruptDisable;
    tSLInformation.WriteWlanPin = sWriteWlanPin;

	//
	//	Store the init callback, wlan and ping callbacks
	//
    /* init asynchronous events callback */
	tSLInformation.sWlanCB= sWlanCB;

	tSLInformation.pucReceivedData = 0;
	tSLInformation.pucTxCommandBuffer = 0;
}

//*****************************************************************************
//
//!  void SpiReceiveHandler(void *pWorkBuff)
//!
//!  \param         pvBuffer - pointer to the received data buffer
//!                      The function triggers Received event/data processing
//!                 
//!  \param         Pointer to the received data
//!  \return        none
//!
//!  \brief         The function triggers Received event/data processing. It is called from the SPI
//! 			   library to receive the data
//
//*****************************************************************************
void SpiReceiveHandler(void *pvBuffer)
{	
	tSLInformation.usEventOrDataReceived = 1;
	tSLInformation.pucReceivedData		 = (unsigned char 	*)pvBuffer;

	hci_unsolicited_event_handler();
}



/**
 * \brief start WLAN device
 *
 * This function asserts the enable pin of the device (WLAN_EN), starting the HW initialization process.
 * The function blocked until device initalization is completed. 
 * Function also configure patches (FW, driver or bootloader) and calls appropriate device
 * callbacks.\n
 *
 *  
 * \return      None
 *
 * \sa          wlan_init  wlan_stop
 * \note        Prior calling the function wlan_init shall be called.\n
 * \warning     This function must be called after wlan_init and
 *              before any other wlan API
 */

void
wlan_start(unsigned short usPatchesAvailableAtHost)
{
	unsigned long ulSpiIRQState;
	
    //
	// Allocate the memory for the RX/TX data transactions: the maximal length is
	// of the 1700 bytes
	//
	tSLInformation.pucTxCommandBuffer = wlan_tx_buffer;

	//
	// init spi
	//
	SpiOpen(SpiReceiveHandler);

	//
	// Check the IRQ line
	//
	ulSpiIRQState = tSLInformation.ReadWlanInterruptPin();
	
    //
    // ASIC 1273 chip enable: toggle WLAN EN line
    //
    tSLInformation.WriteWlanPin( WLAN_ENABLE );

	if (ulSpiIRQState)
	{
		//
		// wait till the IRQ line goes low
		//
		while(tSLInformation.ReadWlanInterruptPin() != 0)
		{
		}
	}
	else
	{
		//
		// wait till the IRQ line goes high and than low
		//
		while(tSLInformation.ReadWlanInterruptPin() == 0)
		{
		}

		while(tSLInformation.ReadWlanInterruptPin() != 0)
		{
		}
	}
	
	SimpleLink_Init_Start(usPatchesAvailableAtHost);

	// Read Buffer's size and finish
	hci_command_send(HCI_CMND_READ_BUFFER_SIZE, tSLInformation.pucTxCommandBuffer, 0);
	SimpleLinkWaitEvent(HCI_CMND_READ_BUFFER_SIZE, 0);
}

/**
 * \brief wlan stop
 *
 * Stop WLAN device by putting it into reset state. 
 *  
 * \return    None     
 *
 * \sa wlan_start 
 * \note        
 * \warning     
 */
void
wlan_stop(void)
{
    //
    // ASIC 1273 chip disable
    //
    tSLInformation.WriteWlanPin( WLAN_DISABLE );

    //
    // Wait till IRQ line goes high...
    //
	while(tSLInformation.ReadWlanInterruptPin() == 0)
	{
	}
	
    
	//
	// Free the used by WLAN Driver memory
	//
	if (tSLInformation.pucTxCommandBuffer)
	{
	//	OS_free (tSLInformation.pucTxCommandBuffer);
		tSLInformation.pucTxCommandBuffer = 0;
	}

	SpiClose();
}

/**
 * \brief wlan connect
 *
 * Connect to station
 *
 * \param[in]   sec_type  - security options:\n WLAN_SEC_UNSEC, 
 *       WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
 * \param[in]   ssid  up  to 32 bytes and is ASCII SSID of the AP
 * \param[in]   ssid_len  A length of the SSID
 * \param[in]   bssid 6 bytes
 * \param[in]   key up to 16 bytes
 * \param[in]   key_len key len
 * 
 *  
 * \return On success, zero is returned. On error, negative is 
 *            returned. Note that even though a zero is returned on success to trigger
 *		 connection operation, it does not mean that CCC3000 is already connected.
 *		 An asynchronous "Connected" event is generated when actual assosiation process
 *		 finishes and CC3000 is connected to the AP.
 *
 * \sa  wlan disconnect        
 * \note        
 * \warning     
 */

long
wlan_connect(unsigned long ulSecType, char *ssid, long ssid_len,
             unsigned char *bssid, unsigned char *key, long key_len)
{
    unsigned short arg_len;
    long ret;
    unsigned char *ptr;
    wlan_connect_args_t *args;

    arg_len = 0;
    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_connect_args_t *)(ptr + HEADERS_SIZE_CMD);

    //
    // Fill in HCI packet structure
    //
    arg_len = sizeof(wlan_connect_args_t) + ssid_len + key_len - 1;

    //
    // Fill in command buffer
    //
    args->ulSsidOffset = 0x0000001c;
    args->ulSsidLen = ssid_len;
    args->ulSecType = ulSecType;
    args->ulKeyOffset = 0x00000010 + ssid_len;
    args->ulKeyLen = key_len;

	//
	// padding shall be zeroed
	//
    if(bssid)
    {
    	memset(args->pad, 0, sizeof(args->pad));
        memcpy(args->ucBssid, bssid, sizeof(args->ucBssid));
    }
    else
    {
        memset(args->pad, 0, sizeof(args->ucBssid) + sizeof(args->pad));
    }
    memcpy(args->payload, ssid, ssid_len);
    if(key_len && key)
    {
        memcpy(args->payload + ssid_len,
                key, key_len);
    }

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_CONNECT, ptr, arg_len);

	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_CONNECT, &ret);
	errno = ret;

    return(ret);
}


/**
 * \brief wlan disconnect
 *
 * Disconnect connection from AP. 
 * 
 *  
 * \return  0 disconnected done, other already disconnected
 *
 * \sa   wlan_connect       
 * \note        
 * \warning     
 */
long
wlan_disconnect()
{
    long ret;
    unsigned char *ptr;

    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;

    hci_command_send(HCI_CMND_WLAN_DISCONNECT, ptr, 0);

	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_DISCONNECT, &ret);
	errno = ret;

    return(ret);
}

/**
 * \brief set connection policy
 *  
 *  
 * When auto is enabled, the device tries to connect according 
 * the following policy:\n 
 * 1) If fast connect is enabled and last connection is valid, the device will try to connect it without the 
 * scanning procedure (fast). The last connection marked as 
 * invalid, due to adding/removing profile.\n 2) If profile 
 * exists, the device will try to connect it (Up to seven 
 * profiles)\n 3) If fast and profiles are not found, and open 
 * mode is enabled, the device will try to connect to any AP.
 * Note that the policy settings are stored in the CC3000 NVMEM.
 *  
 *  
 *  
 * \param[in]   should_connect_to_open_ap  enable(1), disable(0)
 *       connect to any available AP. This parameter corresponds to the 
 * 	   configuration of item # 3 in the above description.
 * \param[in]   should_use_fast_connect enable(1), disable(0). 
 *       if enabled, tries to connect to the last connected
 *       AP. This parameter describes a fast connect option above.
 * \param[in]   auto_start enable(1), disable(0) auto connect 
 *       after reset and periodically reconnect if needed.This configuration
 *	   configures option 2 in the above description.\n
 * 
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned   
 * \sa wlan_add_profile     wlan_ioctl_del_profile        
 * \note       The default policy settings are: Use fast connect, ussage of profiles is not enabled and ussage of open AP is not set. 
 *		    Note also that in case fast connection option is used, the profile is automatically generated.
 * \warning     
 */

long
wlan_ioctl_set_connection_policy(unsigned long should_connect_to_open_ap, 
                                 unsigned long ulShouldUseFastConnect,
                                 unsigned long ulUseProfiles)
{
    unsigned short arg_len;
    long ret;
    unsigned char *ptr;
    wlan_ioctl_set_connection_policy_args_t *args;

    arg_len = 0;
    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_ioctl_set_connection_policy_args_t *)(ptr + HEADERS_SIZE_CMD);

    //
    // Fill in HCI packet structure
    //
    arg_len = sizeof(wlan_ioctl_set_connection_policy_args_t);

    //
    // Fill in temporary command buffer
    //
    args->ulShouldConnectToOpenAp = should_connect_to_open_ap;
    args->ulShouldUseFastConnect = ulShouldUseFastConnect;
    args->ulAutoStart = ulUseProfiles;

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY,
			        ptr, arg_len);

    //
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY, &ret);

    return(ret);
}
/**
 * \brief add profile 
 *
 *  When auto start is enabled, the device connects to
 *  station from the profiles table. Up to 7 profiles are
 *  supported. If several profiles configured the device chose
 *  the highest priority profile, within each priority group,
 *  device will chose profile based on security policy, signal
 *  strength, etc parameters. All the profiles are stored in CC3000 
 *  NVMEM.\n
 *  
 *  
 * \param[in]   tSecType:\n WLAN_SEC_UNSEC, WLAN_SEC_WEP, 
 *       WLAN_SEC_WPA or WLAN_SEC_WPA2
 * \param[in]   ucSsid  ssid, up to 32 bytes
 * \param[in]   ulSsidLen ssid length
 * \param[in]   ucBssid  bssid, 6 bytes
 * \param[in]   ulPriority profile priority. Lowest priority:
 *       0
 * \param[in]   ulPairwiseCipher_Or_Key
 * \param[in]   ulGroupCipher_TxKeyLen
 * \param[in]   ulKeyMgmt
 * \param[in]   ucPf_OrKey
 * \param[in]   ulPassPhraseLen
 * 
 *  
 * \return  On success, index is returned. On error, -1 is 
 *            returned      
 *
 * \sa   wlan_ioctl_del_profile       
 * \note        
 * \warning     
 */

long
wlan_add_profile(unsigned long ulSecType, 
										unsigned char* ucSsid,
										unsigned long ulSsidLen, 
										unsigned char *ucBssid,
                                        unsigned long ulPriority,
                                        unsigned long ulPairwiseCipher_Or_TxKeyLen,
                                        unsigned long ulGroupCipher_TxKeyIndex,
                                        unsigned long ulKeyMgmt,
                                        unsigned char* ucPf_OrKey,
                                        unsigned long ulPassPhraseLen)
{
    unsigned short arg_len;
    long ret;
    unsigned char *ptr;
    long i = 0;
    wlan_add_profile_unsec_args_t *args_no_sec;
	wlan_add_profile_wep_args_t *args_wep;
	wlan_add_profile_wpa_args_t *args_wpa;

	ptr = tSLInformation.pucTxCommandBuffer;


	//
	// Setup arguments in accordence with the security type
	//
	switch (ulSecType)
	{
		//None
	    case WLAN_SEC_UNSEC:
	    {
			args_no_sec = (wlan_add_profile_unsec_args_t *)(ptr + HEADERS_SIZE_CMD);
	        arg_len = sizeof(wlan_add_profile_unsec_args_t) + ulSsidLen;
			//
		    // Fill in the command buffer
		    //
		    args_no_sec->ulSsidOffset = 0x00000014;
		    args_no_sec->ulSsidLen = ulSsidLen;
		    args_no_sec->ulSecType = WLAN_SEC_UNSEC;
		    args_no_sec->ulPriority = ulPriority;
		    if(ucBssid)
		    {
		    	memset(args_no_sec->pad, 0, sizeof(args_no_sec->pad));
		        memcpy(args_no_sec->ucBssid, ucBssid, sizeof(args_no_sec->ucBssid));
		    }
		    else
		    {
		        memset(args_no_sec->ucBssid, 0, sizeof(args_no_sec->ucBssid) + sizeof(args_no_sec->pad));
		    }
		    memcpy((unsigned char *)args_no_sec + sizeof(wlan_add_profile_unsec_args_t),
           			ucSsid, ulSsidLen);
	    }
		break;

		//WEP
	    case WLAN_SEC_WEP:
	    {
	        arg_len = sizeof(wlan_add_profile_wep_args_t) + ulSsidLen + ulPairwiseCipher_Or_TxKeyLen * 4;
		    args_wep = (wlan_add_profile_wep_args_t *)(ptr + HEADERS_SIZE_CMD);

		    //
		    // Fill in the command buffer
		    //
		    args_wep->ulSsidOffset = 0x00000020;
		    args_wep->ulSsidLen = ulSsidLen;
		    args_wep->ulSecType = WLAN_SEC_WEP;
		    args_wep->ulPriority = ulPriority;
		    args_wep->ulKeyOffset = 0x0000000C + ulSsidLen;
		    args_wep->ulKeyLen = ulPairwiseCipher_Or_TxKeyLen;
		    args_wep->ulTxKeyIndex = ulGroupCipher_TxKeyIndex;
		    if(ucBssid)
		    {
		    	memset(args_wep->pad, 0, sizeof(args_wep->pad));
		        memcpy(args_wep->ucBssid, ucBssid, sizeof(args_wep->ucBssid));
		    }
		    else
		    {
		        memset(args_wep->ucBssid, 0, sizeof(args_wep->ucBssid) + sizeof(args_wep->pad));
			}
			
			memcpy((unsigned char *)args_wep + sizeof(wlan_add_profile_wep_args_t),
		           ucSsid, ulSsidLen);

		   for(i = 0; i < 4; i++)
		   {
		       memcpy((unsigned char *)args_wep + sizeof(wlan_add_profile_wep_args_t) +
		              ulSsidLen + (i * ulPairwiseCipher_Or_TxKeyLen), &ucPf_OrKey[i * ulPairwiseCipher_Or_TxKeyLen], ulPairwiseCipher_Or_TxKeyLen);
		   }			
	    }
		break;

		//WPA
		//WPA2
	    case WLAN_SEC_WPA:
	    case WLAN_SEC_WPA2:
	    {
			arg_len = sizeof(wlan_add_profile_wpa_args_t) + ulSsidLen + ulPassPhraseLen;
	     	args_wpa = (wlan_add_profile_wpa_args_t *)(ptr + HEADERS_SIZE_CMD);
			//
			// Fill in temporary command buffer
			//
			args_wpa->ulSsidOffset = 0x00000028;
			args_wpa->ulSsidLen = ulSsidLen;
			args_wpa->ulSecType = ulSecType;
			args_wpa->ulPriority = ulPriority;
			args_wpa->ulPassphraseOffset = 0x00000008 + ulSsidLen;
			args_wpa->ulPassphraseLen = ulPassPhraseLen;
			args_wpa->ulGroupCipher = ulGroupCipher_TxKeyIndex;
			args_wpa->ulKeyMgmt = ulKeyMgmt;
			args_wpa->ulPairwiseCipher = ulPairwiseCipher_Or_TxKeyLen;

			if(ucBssid)
		    {
		    	memset(args_wpa->pad, 0, sizeof(args_wpa->pad));
		        memcpy(args_wpa->ucBssid, ucBssid, sizeof(args_wpa->ucBssid));
		    }
		    else
		    {
		        memset(args_wpa->ucBssid, 0, sizeof(args_wpa->ucBssid) + sizeof(args_wpa->pad));
			}

			memcpy((unsigned char *) args_wpa + sizeof(wlan_add_profile_wpa_args_t),
				   ucSsid, ulSsidLen);

			memcpy((unsigned char *) args_wpa + sizeof(wlan_add_profile_wpa_args_t) + ulSsidLen,
				   ucPf_OrKey, ulPassPhraseLen);
	    }

        break;
	}    

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_IOCTL_ADD_PROFILE,
        ptr, arg_len);

    //
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, &ret);

    return(ret);
}

/**
 * \brief Delete WLAN profile
 *
 * Delete WLAN profile  
 *  
 * \param[in]   index  number of profile to delete   
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned
 *
 * \sa   wlan_add_profile       
 * \note        
 * \warning     
 */
long
wlan_ioctl_del_profile(unsigned long ulIndex)
{
    unsigned short arg_len;
    long ret;
    unsigned char *ptr;
    wlan_ioctl_del_profile_args_t *args;

    arg_len = 0;
    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_ioctl_del_profile_args_t *)(ptr + HEADERS_SIZE_CMD);

    //
    // Fill in HCI packet structure
    //
    arg_len = sizeof(wlan_ioctl_del_profile_args_t);

    //
    // Fill in temporary command buffer
    //
    args->index = ulIndex;

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_IOCTL_DEL_PROFILE,
        ptr, arg_len);

    //
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_DEL_PROFILE, &ret);
	
    return(ret);
}

/**
 * \brief Gets the WLAN scan operation results
 *
 * Gets entry from scan result table.
 * The scan results are returned one by one, and each entry represents a single AP found in the area. The following is a format of hte scan result:
 *	- 4 Bytes: number of networks found
 *	- 4 Bytes: The status of the scan: 0 - agged results, 1 - results valid, 2 - no results
 *	- 44 bytes: Result entry, where the bytes are arranged as follows:
 *				- 4 bytes isValid - is result valid or not
 *				- 4 bytes rssi 			- RSSI value;	 
 *				- 4 bytes: securityMode - security mode of the AP: 0 - Open, 1 - WEP, 2 WPA, 3 WPA2
 *				- 4 bytes: SSID name length
 *				- 2 bytes: the time at which the entry has entered into scans result table
 *				- 32 bytes: SSID name
 *				- 6 bytes:	BSSID
 *  
 * \param[in] scan_timeout  
 * \param[out] ucResults  scan resault (WLAN_San_Result_t)
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned 
 *
 * \sa  wlan_ioctl_set_scan_params        
 * \note scan_timeout, is not supported on this version.       
 * \warning     
 */
long
wlan_ioctl_get_scan_results(unsigned long ulScanTimeout,
                            unsigned char *ucResults)
{
    unsigned char *ptr;
    wlan_ioctl_get_scan_results_args_t *args;

    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_ioctl_get_scan_results_args_t *)(ptr + HEADERS_SIZE_CMD);

    
    //
    // Fill in temporary command buffer
    //
    args->ulScanTimeout = ulScanTimeout;

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS,
        ptr, sizeof(wlan_ioctl_get_scan_results_args_t));

    //
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS, ucResults);

	//
	// There is no situation where scan result can fail...
	//
	
    return(0);
}

/**
 * \brief Sets the WLAN scan configuration
 *
 * start and stop scan procedure. 
 * Set scan parameters 
 *  
 * \param[in] uiEnable       start/stop scan (1=start,0=stop). 
 *       Saved: No 
 * \param[in] uiMinDwellTime   minimum dwell time value to be 
 *       used for each channel, in millisconds. Saved: yes
 *       Default: 20
 * \param[in] uiMaxDwellTime    maximum dwell time value to be 
 *       used for each channel, in millisconds. Saved: yes
 *       Default: 30
 * \param[in] uiNumOfProbeResponces  max probe request between 
 *       dwell time. Saved: yes. Default: 2 
 *  
 * \param[in] uiChannelMask  bitwise, up to 13 channels 
 *       (0x1fff). Saved: yes. Default: 0x7ff
 * \param[in] uiRSSIThreshold   RSSI threshold. Saved: yes 
 *       Default -80
 * \param[in] uiSNRThreshold    NSR thereshold. Saved: yes.
 *       Default: 0
 * \param[in] uiDefaultTxPower  probe Tx power. Saved: yes 
 *       Default: 205
 * \param[in] aiIntervalList    pointer to array with 16 entries
 *       (16 channels) each entry (unsigned int) holds timeout
 *       between scanning the next channel (in millisconds ).
 *       Saved: yes. Default 2000ms.
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned 
 * \sa   wlan_ioctl_get_scan_results       
 * \note uiDefaultTxPower, is not supported on this version.    
 * \warning     
 */
long
wlan_ioctl_set_scan_params(unsigned long uiEnable, unsigned long uiMinDwellTime,unsigned long uiMaxDwellTime,
										   unsigned long uiNumOfProbeResponces,unsigned long uiChannelMask,
										   long iRSSIThreshold,unsigned long uiSNRThreshold,
										   unsigned long uiDefaultTxPower, unsigned long *aiIntervalList)
{
    unsigned long  uiRes;
    unsigned char *ptr;
    wlan_ioctl_set_scan_params_args_t *args;

    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_ioctl_set_scan_params_args_t *)(ptr + HEADERS_SIZE_CMD);

    //
    // Fill in temporary command buffer
    //
    args->uiIntervalListOffset = 36;
    args->uiEnable       = uiEnable;
	args->uiMinDwellTime = uiMinDwellTime;
	args->uiMaxDwellTime = uiMaxDwellTime;
	args->uiNumOfProbeResponces = uiNumOfProbeResponces;
	args->uiChannelMask = uiChannelMask;
	args->uiRssiThreshold = iRSSIThreshold;
	args->uiSNRThreshold = uiSNRThreshold;
	args->uiDefaultTxPower = uiDefaultTxPower;
	memcpy((char *)(&args->auiIntervalList[0]), aiIntervalList, sizeof(args->auiIntervalList));

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_WLAN_IOCTL_SET_SCANPARAM,
        ptr, sizeof(wlan_ioctl_set_scan_params_args_t));

    //
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_SET_SCANPARAM, &uiRes);
	
    return(uiRes);
}
/**
 * \brief set event mask 
 *
 * Mask event according to bit mask. In case that event is 
 * masked (1), the device will not send the masked event.  
 * \param[in] mask  Saved: no. mask option:\n 
 *       HCI_EVNT_WLAN_UNSOL_CONNECT connect event\n
 *       HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event\n
 *       HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done\n
 *       HCI_EVNT_WLAN_UNSOL_INIT init done\n
 *       HCI_EVNT_WLAN_UNSOL_DHCP dhcp report\n
 *       HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report\n
 *       HCI_EVNT_WLAN_KEEPALIVE keepalive\n
 *  
 * \On success, zero is returned. On error, -1 is 
 *            returned       
 * \sa          
 * \note        
 * \warning     
 */

long
wlan_set_event_mask(unsigned long ulMask)
{
    unsigned short arg_len;
    long ret;
    unsigned char *ptr;
    wlan_set_mask_args_t *args;

    arg_len = 0;
    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (wlan_set_mask_args_t *)(ptr + HEADERS_SIZE_CMD);

    //
    // Fill in HCI packet structure
    //
    arg_len = sizeof(wlan_set_mask_args_t);

    //
    // Fill in temporary command buffer
    //
    args->mask = ulMask;

    //
    // Initiate a HCI command
    //
    hci_command_send(HCI_CMND_EVENT_MASK,
        ptr, arg_len);

   	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_EVENT_MASK, &ret);
	
	return(ret);
}
/**
 * \brief get wlan status
 *  
 * get wlan status: disconnected, scaning, connecting or 
 * connected 
 *  
 * \return WLAN_STATUS_DISCONNECTED, WLAN_STATUS_SCANING, 
 *         STATUS_CONNECTING or WLAN_STATUS_CONNECTED
 * 
 *
 * \sa          
 * \note        
 * \warning     
 */

long
wlan_ioctl_statusget(void)
{
    long ret;
    unsigned char *ptr;

    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;

    hci_command_send(HCI_CMND_WLAN_IOCTL_STATUSGET,
        ptr, 0);

    
   	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_STATUSGET, &ret);

    return(ret);    
}

/**
 * \brief Start acquire profile
 *
 * Start to acquire device profile. Device scans messages from 
 * station with specific prefix SSID 
 * (wlan_simple_config_set_prefix). The device acquire his own 
 * profile, if profile message is found. The acquired AP information is
 * stored in the profiles table of CC3000.After the profile is acquired the
 * behavior is as defined by policy. \n 
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned 
 * \sa   wlan_firs_time_config_set_prefix  wlan_first_time_config_stop
 * \note    An asynchnous event - First Time Config Done will be generated as soon as the process finishes successfully
 * \warning     
 */
long
wlan_first_time_config_start(void)
{
    long ret;
    unsigned char *ptr;

    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;

    hci_command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START, ptr, 0);

    
   	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START, &ret);

    return(ret);    
}


/**
 * \brief stop acquire profile 
 *  
 * Stop the acquire profile procedure 
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned 
 *
 * \sa   wlan_fisrt_time_config_start  wlan_first_time_config_set_prefix
 * \note      
 * \warning     
 */
long
wlan_first_time_config_stop(void)
{
    long ret;
    unsigned char *ptr;

    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;

    hci_command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP, ptr, 0);

    
   	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP, &ret);

    return(ret);    
}

/**
 * \brief config set prefix
 *  
 * Configure station ssid prefix. The prefix is used to identify
 * the station that broadcast device profile. 
 *
 * \param[in] newPrefix  3 bytes identify the SSID prefix for 
 *       the Simple Config.
 *  
 * \return  On success, zero is returned. On error, -1 is 
 *            returned   
 *
 * \sa   wlan_fist_time_config_start  wlan_first_time_config_stop
 * \note        The prefix is stored in CC3000 NVMEM.\n
 * \warning     
 */

long
wlan_first_time_config_set_prefix(char* cNewPrefix)
{
    long ret;
    unsigned char *ptr;
    char *args;

    ret = EFAIL;
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (char *)(ptr + HEADERS_SIZE_CMD);
    
    if (cNewPrefix == NULL)
        return ret;

    memcpy(args, cNewPrefix, SL_SIMPLE_CONFIG_PREFIX_LENGTH);        
    hci_command_send(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX, ptr, SL_SIMPLE_CONFIG_PREFIX_LENGTH);

    
   	//
	// Wait for command complete event
	//
	SimpleLinkWaitEvent(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX, &ret);

    return(ret);    
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
