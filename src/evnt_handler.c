/*****************************************************************************
*
*  evnt_handler.c  - CC3000 Host Driver Implementation.
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
//! \addtogroup evnt_handler_api
//! @{
//
//*****************************************************************************

#include <cc3000/cc3000_common.h>
#include <cc3000/string.h>
#include <cc3000/hci.h>
#include <cc3000/evnt_handler.h>
#include <cc3000/wlan.h>
#include <cc3000/socket.h>
#include <cc3000/netapp.h>
#include <cc3000/spi.h>


unsigned long socket_active_status = SOCKET_STATUS_INIT_VAL;  

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//packed is used for preventing padding before sending the structure over the SPI                       ///
//for every IDE, exist different syntax:          1.   __MSP430FR5739__ for CCS v5                      ///
//                                                2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench    ///
// THIS COMMENT IS VALID FOR EVERY STRUCT DEFENITION.                                                   ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _bsd_resp_params_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _bsd_resp_params_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _bsd_resp_params_t
#endif
{
    unsigned long sd;
    long lStatus;
}bsd_resp_params_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _handles_descriptor_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _handles_descriptor_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _handles_descriptor_t
#endif
{
    unsigned char handle;
    unsigned char block_mode;
    unsigned short free_bufs;
}handles_descriptor_t;

//*****************************************************************************
//
// Prototypes for the static functions
//
//*****************************************************************************
static long hci_event_unsol_flowcontrol_handler(hci_evnt_hdr_t *pEvent);


static void update_socket_active_status(bsd_resp_params_t *resp_params);

//*****************************************************************************
//
//!  ...
//!
//!  \param  string  pointer to the string contains parameters for IPERF
//!
//!  \return none    count of arguments in string
//!
//!  \brief
//
//*****************************************************************************
void hci_unsol_handle_patch_request(hci_evnt_hdr_t *event_hdr)
{
	char *params = (char *)(event_hdr + 1);
	unsigned long ucLength = 0;
	char *patch;
	
	switch (*params)
	{
		case HCI_EVENT_PATCHES_DRV_REQ:

			if (tSLInformation.sDriverPatches)
			{
				patch = tSLInformation.sDriverPatches(&ucLength);
				
				if (patch)
				{
					hci_patch_send(HCI_EVENT_PATCHES_DRV_REQ, tSLInformation.pucTxCommandBuffer, 
						patch, ucLength);
					return;
				}
			}

 			// Send 0 length Patches responce event
 			hci_patch_send(HCI_EVENT_PATCHES_DRV_REQ, tSLInformation.pucTxCommandBuffer, 0, 0);
			break;
			
		case HCI_EVENT_PATCHES_FW_REQ:
			if (tSLInformation.sFWPatches)
			{
				patch = tSLInformation.sFWPatches(&ucLength);

				//
				// Build and send a patch
				//
				if (patch)
				{
					hci_patch_send(HCI_EVENT_PATCHES_FW_REQ, tSLInformation.pucTxCommandBuffer, 
						patch, ucLength);
					return;
				}
			}

			// Send 0 length Patches responce event
 			hci_patch_send(HCI_EVENT_PATCHES_FW_REQ, tSLInformation.pucTxCommandBuffer, 0, 0);
			break;
			
		case HCI_EVENT_PATCHES_BOOTLOAD_REQ:
			if (tSLInformation.sBootLoaderPatches)
			{
				patch = tSLInformation.sBootLoaderPatches(&ucLength);

				if (patch)
				{
					hci_patch_send(HCI_EVENT_PATCHES_BOOTLOAD_REQ, tSLInformation.pucTxCommandBuffer, 
						patch, ucLength);
					return;
				}
			}

			// Send 0 length Patches responce event
			hci_patch_send(HCI_EVENT_PATCHES_BOOTLOAD_REQ, tSLInformation.pucTxCommandBuffer, 0, 0);
			break;
	}
}



//*****************************************************************************
//
//!  Event Handler
//!
//!  @param  buf     incoming data buffer
//!  @param  len     size of data buffer
//!
//!  @return         ESUCCESS if successful, EFAIL if an error occured
//!
//!  @brief          Parse the incoming events packets and issues correponding
//!                  event handler from gloabal array of handlers pointers
//
//*****************************************************************************

unsigned char *
hci_event_handler(void *pRetParams, unsigned char *from, unsigned char *fromlen)
{
	unsigned char *pucReceivedData;
	unsigned char *pucReceivedParams;
	hci_evnt_hdr_t *ptHciEventHdr;
	unsigned short usReceivedEventOpcode = 0;
	hci_data_hdr_t *ptDataHdr;
	
	while (1)
	{
		if (tSLInformation.usEventOrDataReceived != 0)
		{				
			pucReceivedData = (tSLInformation.pucReceivedData);
			
			if (*pucReceivedData == HCI_TYPE_EVNT)
			{
				//
				// Event Received
				//
				ptHciEventHdr = (hci_evnt_hdr_t *)(pucReceivedData);
				usReceivedEventOpcode = ptHciEventHdr->usEvntOpcode;
				pucReceivedParams = pucReceivedData + sizeof(hci_evnt_hdr_t);
				//
				// In case unsolicited event received - here the handling finished
				//
				if (hci_unsol_event_handler(ptHciEventHdr) == 0)
				{
					switch(usReceivedEventOpcode)
				    {		
				    	case HCI_CMND_READ_BUFFER_SIZE:
						{
							tSLInformation.usNumberOfFreeBuffers = *pucReceivedParams;
							memcpy((void *)&tSLInformation.usSlBufferLength, (pucReceivedParams + 1), sizeof(tSLInformation.usSlBufferLength));
				    	}
						break;

						case HCI_CMND_WLAN_CONFIGURE_PATCH:
						case HCI_NETAPP_DHCP:
                        case HCI_NETAPP_PING_SEND:
                        case HCI_NETAPP_PING_STOP:
						case HCI_NETAPP_ARP_FLUSH:
						case HCI_NETAPP_SET_DEBUG_LEVEL:
						case HCI_NETAPP_SET_TIMERS:
						case HCI_EVNT_NVMEM_READ:
						case HCI_EVNT_NVMEM_WRITE:
				        case HCI_EVNT_NVMEM_CREATE_ENTRY:
						case HCI_CMND_NVMEM_WRITE_PATCH:
							*(unsigned char *)pRetParams = ptHciEventHdr->ucStatus;
							break;
							
						case HCI_CMND_SETSOCKOPT:
						case HCI_CMND_WLAN_CONNECT:
						case HCI_CMND_WLAN_IOCTL_STATUSGET:
						case HCI_EVNT_WLAN_IOCTL_ADD_PROFILE:
						case HCI_CMND_WLAN_IOCTL_DEL_PROFILE:
						case HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY:
						case HCI_CMND_WLAN_IOCTL_SET_SCANPARAM:
                        case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START:
                        case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP:
                        case HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX:
						case HCI_CMND_EVENT_MASK:
						case HCI_EVNT_WLAN_DISCONNECT:
						case HCI_EVNT_SOCKET:
						case HCI_EVNT_BIND:
						case HCI_CMND_LISTEN:
						case HCI_EVNT_CLOSE_SOCKET:
						case HCI_EVNT_CONNECT:
							memcpy(pRetParams, pucReceivedParams, 4); /* sizeof long */
							break;
				        
                        case HCI_EVNT_BSD_GETHOSTBYNAME:
                            memcpy(pRetParams, pucReceivedParams, sizeof(tBsdGethostbynameParams));
                            break;
				        
				        case HCI_EVNT_ACCEPT:
				        {
							memcpy(pRetParams, pucReceivedParams, sizeof(tBsdReturnParams));
				            break;
				        }
						
				        case HCI_EVNT_RECV:
				        case HCI_EVNT_RECVFROM:
				        {
				            memcpy(pRetParams, pucReceivedParams, sizeof(tBsdReadReturnParams));
                                            if(((tBsdReadReturnParams *)pRetParams)->iNumberOfBytes == ERROR_SOCKET_INACTIVE)
                                            {
                                                set_socket_active_status(((tBsdReadReturnParams *)pRetParams)->iSocketDescriptor,
                                                                         SOCKET_STATUS_INACTIVE);
                                            }
                                              
				            break;
				        }
				        
				        case HCI_EVNT_SELECT:
				        { 
				            memcpy(pRetParams, pucReceivedParams, sizeof(tBsdSelectRecvParams));
				            break;
				        }

						case HCI_CMND_GETSOCKOPT:
							((tBsdGetSockOptReturnParams *)pRetParams)->iStatus = ((char)ptHciEventHdr->ucStatus);
							memcpy((unsigned char *)pRetParams, pucReceivedParams, 4);
							break;

						case HCI_CMND_WLAN_IOCTL_GET_SCAN_RESULTS:
							memcpy (pRetParams, pucReceivedParams, ptHciEventHdr->ucLength - 1);
							break;

						case HCI_CMND_SIMPLE_LINK_START:
							break;

						case HCI_NETAPP_IPCONFIG:
							memcpy(pRetParams, pucReceivedParams, sizeof(tNetappIpconfigRetArgs));
							break;
						case HCI_NETAPP_PING_REPORT:
							memcpy(pRetParams, pucReceivedParams, sizeof(netapp_pingreport_args_t));
							break;

					}
				}

				if (usReceivedEventOpcode == tSLInformation.usRxEventOpcode)
				{
					tSLInformation.usRxEventOpcode = 0;
				}
			}
			else
			{
				ptDataHdr = (hci_data_hdr_t *)pucReceivedData;
				
				//
				// Data received: note that the only case where from and from length are not null is in 
				// recv from, so fill the args accordingly
				//
				if (from)
				{
					*fromlen = ((tBsdRecvFromDataParams *)(pucReceivedData + sizeof(hci_data_hdr_t)))->iFromLen;
					memcpy(from, &(((tBsdRecvFromDataParams *)(pucReceivedData + sizeof(hci_data_hdr_t)))->iFrom) ,*fromlen);
				}

				memcpy(pRetParams, pucReceivedData + sizeof(hci_data_hdr_t) + ptDataHdr->ucArgsize,
							ptDataHdr->usLength - ptDataHdr->ucArgsize);
				
				tSLInformation.usRxDataPending = 0;
			}

			
			tSLInformation.usEventOrDataReceived = 0;


			
			SpiResumeSpi();

			//
			// Since we are going to TX - we need to handle this event after the ResumeSPi since we need interupts
			// 
			if ((*pucReceivedData == HCI_TYPE_EVNT) && (ptHciEventHdr->usEvntOpcode == HCI_EVNT_PATCHES_REQ))
			{
				hci_unsol_handle_patch_request(ptHciEventHdr);
			}

			if ((tSLInformation.usRxEventOpcode == 0) && (tSLInformation.usRxDataPending == 0))
			{
				return NULL;
			}
		}
	}

}

//*****************************************************************************
//
//!  Unsolicited events handler.
//!
//!  @param  event       event type
//!
//!  @param  arg         pointer to the buffer of probable arguments
//!
//!  @param  len         buffer length
//!
//!  @return             ESUCCESS if successful EAGAIN if in progress
//!                      EERROR if an error occured
//!
//!  @brief              This function is Network Application events handler.
//
//*****************************************************************************
long
hci_unsol_event_handler(hci_evnt_hdr_t *event_hdr)
{
    char * data = NULL;
    long event_type = event_hdr->usEvntOpcode;
	
    if (event_type & HCI_EVNT_UNSOL_BASE)
    {
        switch(event_type)
        {
            case HCI_EVNT_DATA_UNSOL_FREE_BUFF:
            {
                hci_event_unsol_flowcontrol_handler(event_hdr);
                return 1;
            }
        }
    }

    if(event_type & HCI_EVNT_WLAN_UNSOL_BASE)
    {           
        switch(event_type)
        {
           case HCI_EVNT_WLAN_KEEPALIVE:
           case HCI_EVNT_WLAN_UNSOL_CONNECT:
           case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
           case HCI_EVNT_WLAN_UNSOL_INIT:
           case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
           case HCI_EVNT_WLAN_UNSOL_DHCP:
           case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
                if( tSLInformation.sWlanCB )
                {
                  if( event_hdr->ucLength > 1 ) /* event with data (1== means only status filed) */ 
                  {
                     data = (char*)(event_hdr);
                     data = data + sizeof(hci_evnt_hdr_t); 
                  }
                  tSLInformation.sWlanCB(event_type, data, event_hdr->ucLength - 1);
                }
                break;
		    default: /* 'default' case which means "event not supported" */
				return (0);
        }
        return(1);
    }

	if ((event_type == HCI_EVNT_SEND) || (event_type == HCI_EVNT_SENDTO)|| (event_type == HCI_EVNT_WRITE))
	{
		//
		// The only synchronous event that can come from SL device in form of command complete is
		// "Command Complete" on data sent, in case SL device was unable to transmit
		//
		tSLInformation.slTransmitDataError = event_hdr->ucStatus;
        update_socket_active_status(M_BSD_RESP_PARAMS_OFFSET(event_hdr));

		return (1);
	}
	
    return(0);
}

//*****************************************************************************
//
//!  Event Handler for Unsolicited Events
//!
//!  @param None
//!
//!  @return         ESUCCESS if successful, EFAIL if an error occured
//!
//!  @brief          Parse the incoming events packets and issues correponding
//!                  event handler from gloabal array of handlers pointers
//
//*****************************************************************************
long
hci_unsolicited_event_handler(void)
{
	unsigned long   res = 0;
	unsigned char *pucReceivedData;
	hci_evnt_hdr_t *ptHciEventHdr;
	
	if (tSLInformation.usEventOrDataReceived != 0)
	{
		pucReceivedData = (tSLInformation.pucReceivedData);
			
		if (*pucReceivedData == HCI_TYPE_EVNT)
		{
			//
			// Event Received
			//
			ptHciEventHdr = (hci_evnt_hdr_t *)(pucReceivedData);
			
			//
			// In case unsolicited event received - here the handling finished
			//
			if (hci_unsol_event_handler(ptHciEventHdr) == 1)
			{
				//
				// There was an un-solicited event received - we can release the buffer and clean the
				// event received 
				//
				tSLInformation.usEventOrDataReceived = 0;
							
				res = 1;

				SpiResumeSpi();
			}
		}
	}

	return res;
}

void set_socket_active_status(long Sd, long Status)
{
    if(M_IS_VALID_SD(Sd) && M_IS_VALID_STATUS(Status))
    {
        socket_active_status &= ~(1 << Sd);      /* clean socket's mask */
        socket_active_status |= (Status << Sd); /* set new socket's mask */
    }
}


//*****************************************************************************
//
//!  ...
//!
//!  \param  string  pointer to the string contains parameters for IPERF
//!
//!  \return none    count of arguments in string
//!
//!  \brief
//
//*****************************************************************************
long
hci_event_unsol_flowcontrol_handler(hci_evnt_hdr_t *pEvent)
{
    long temp;
    long i;
    unsigned short *pusNumberOfHandles;
    handles_descriptor_t *pReadPayload;

    pusNumberOfHandles = (unsigned short *)((char *)pEvent +
                         sizeof(hci_evnt_hdr_t));
    pReadPayload = (handles_descriptor_t *)((char *)pEvent +
                   sizeof(hci_evnt_hdr_t) +
                   sizeof(*pusNumberOfHandles));
    temp = 0;
    for(i = 0; i < *pusNumberOfHandles; i++)
    {
        temp += pReadPayload->free_bufs;
        pReadPayload++;  
    }
        
    tSLInformation.usNumberOfFreeBuffers += temp;

    return(ESUCCESS);
}



long
get_socket_active_status(long Sd)
{
    if(M_IS_VALID_SD(Sd))
    {
        return (socket_active_status & (1 << Sd)) ? SOCKET_STATUS_INACTIVE : SOCKET_STATUS_ACTIVE;
    }
    return SOCKET_STATUS_INACTIVE;
}

void
update_socket_active_status(bsd_resp_params_t *resp_params)
{
    if(ERROR_SOCKET_INACTIVE == resp_params->lStatus)
    {
        set_socket_active_status(resp_params->sd, SOCKET_STATUS_INACTIVE);
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
