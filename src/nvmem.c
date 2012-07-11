/*****************************************************************************
*
*  nvmem.c  - CC3000 Host Driver Implementation.
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
//! \addtogroup nvmem_api
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include "nvmem.h"
#include "hci.h"
#include "socket.h"
#include "evnt_handler.h"

//*****************************************************************************
//
// Prototypes for the structures for APIs.
//
//*****************************************************************************


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//packed is used for preventing padding before sending the structure over the SPI                       ///
//for every IDE, exist different syntax:          1.   __MSP430FR5739__ for CCS v5                      ///
//                                                2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench    ///
// THIS COMMENT IS VALID FOR EVERY STRUCT DEFENITION!                                                   ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _nvmem_create_entry_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _nvmem_create_entry_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _nvmem_create_entry_t
#endif
{
    unsigned long ulFileId;
    unsigned long ulNewLen;
}nvmem_create_entry_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _nvmem_swap_entry_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _nvmem_swap_entry_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _nvmem_swap_entry_t
#endif
{
    unsigned long ulFileId_1;
    unsigned long ulFileId_2;
}nvmem_swap_entry_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _nvmem_read_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _nvmem_read_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _nvmem_read_t
#endif
{
    unsigned long ulFileId;
    unsigned long ulLength;
    unsigned long ulOffset; 
}nvmem_read_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _nvmem_write_patch_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _nvmem_write_patch_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _nvmem_write_patch_t
#endif
{
    unsigned long ulFileId;
    unsigned long ulStart;
	unsigned long ulAddress;
    unsigned long ulLength;
}nvmem_write_patch_t;


#ifdef __CCS__
typedef struct __attribute__ ((__packed__)) _nvmem_write_t
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _nvmem_write_t
#elif __GNUC__
typedef struct __attribute__ ((__packed__)) _nvmem_write_t
#endif
{
    unsigned long ulFileId;
    unsigned long ulOffset;
    unsigned long ulLength;
    unsigned long ulEntryOffset;
}nvmem_write_t;



/*****************************************************************************
 * \brief Read data from nvmem
 *
 * Reads data from the file referred by the ulFileId parameter. 
 * Reads data from file ulOffset till len. Err if the file can't
 * be used, is invalid, or if the read is out of bounds. 
 *
 *
 * \param[in] ulFileId   nvmem file id:\n
 * NVMEM_NVS_FILEID, NVMEM_NVS_SHADOW_FILEID,
 * NVMEM_WLAN_CONFIG_FILEID, NVMEM_WLAN_CONFIG_SHADOW_FILEID,
 * NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
 * NVMEM_MAC_FILEID, NVMEM_FRONTEND_VARS_FILEID,
 * NVMEM_IP_CONFIG_FILEID, NVMEM_IP_CONFIG_SHADOW_FILEID,
 * NVMEM_BOOTLOADER_SP_FILEID or NVMEM_RM_FILEID.
 * \param[in] ulLength   number of bytes to read  
 * \param[in] ulOffset   ulOffset in file from where to read  
 * \param[out] buff    output buffer pointer
 *
 * \return	    number of bytes read.
 *
 * \sa
 * \note
 * \warning
 *
 *****************************************************************************/
signed long 
nvmem_read(unsigned long ulFileId, unsigned long ulLength, unsigned long ulOffset, unsigned char *buff)
{
    unsigned char ucStatus = 0xFF;
    unsigned short arg_len;
    unsigned char *ptr;
    nvmem_read_t *args;

    arg_len = 0;
    
    ptr = tSLInformation.pucTxCommandBuffer;
    args = (nvmem_read_t *)(ptr + HEADERS_SIZE_CMD);
    
    //
    // Fill in HCI packet structure
    //
    arg_len = sizeof(nvmem_read_t);

    args->ulFileId = ulFileId;
    args->ulLength = ulLength;
    args->ulOffset = ulOffset;

	//
    // Initiate a HCI command
    //
	hci_command_send(HCI_CMND_NVMEM_READ, ptr, arg_len);

    SimpleLinkWaitEvent(HCI_CMND_NVMEM_READ, &ucStatus);

	//
	// In case there is a data - read it
	//
	if (ucStatus == 0)
	{
		//
		// Wait for the data in a synchronous way. Here we assume that the buffer is big enough
		// to store also parameters of nvmem
		//
		SimpleLinkWaitData(buff, 0, 0);
	}

    return(ucStatus);
}


/*****************************************************************************
 * \brief Write data to nvmem.
 *  
 * writes data to file referred by the ulFileId parameter. 
 * Writes data to file  ulOffset till ulLength. The file id will be 
 * marked invalid till the write is done. The file entry doesn't
 * need to be valid - only allocated.
 *  
 * \param[in] ulFileId   nvmem file id:\n
 * NVMEM_NVS_FILEID, NVMEM_NVS_SHADOW_FILEID,
 * NVMEM_WLAN_CONFIG_FILEID, NVMEM_WLAN_CONFIG_SHADOW_FILEID,
 * NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
 * NVMEM_MAC_FILEID, NVMEM_FRONTEND_VARS_FILEID,
 * NVMEM_IP_CONFIG_FILEID, NVMEM_IP_CONFIG_SHADOW_FILEID,
 * NVMEM_BOOTLOADER_SP_FILEID or NVMEM_RM_FILEID.
 * \param[in] ulLength    number of bytes to write   
 * \param[in] ulEntryOffset  offset in file to start write operation from    
 * \param[in] buff      data to write 
 *
 * \return	  on succes 0, error otherwise.
 *
 * \sa
 * \note
 * \warning
 *
 *****************************************************************************/
signed long 
nvmem_write(unsigned long ulFileId, unsigned long ulLength, unsigned long ulEntryOffset, 
            unsigned char *buff)
{
    long iRes;
    unsigned short arg_len; 
    unsigned char *ptr;
    nvmem_write_t *args;
    
    iRes = EFAIL;
    arg_len = 0;

    ptr = tSLInformation.pucTxCommandBuffer;
    args = (nvmem_write_t *)(ptr + SPI_HEADER_SIZE + sizeof(hci_data_cmd_hdr_t));

    // Fill in HCI packet structure

    arg_len = sizeof(nvmem_write_t);

    args->ulFileId = ulFileId;
    /* The 'args->ulOffset' parameter is used by receiving side for calculating beginning of data in the message.
       Data starts after 'args' parameters' end, i.e. after 12 bytes:
       ulOffset = sizeof(args->ulOffset) + sizeof(args->ulLength) + sizeof(args->entry_ulOffset) */
    args->ulOffset = 12;
	args->ulLength  = ulLength;
    args->ulEntryOffset = ulEntryOffset;
	memcpy((ptr + SPI_HEADER_SIZE + sizeof(hci_data_cmd_hdr_t) + sizeof(nvmem_write_t)),
			buff,
			ulLength);

    // Initiate a HCI command but it will come on data channel
    hci_data_command_send(HCI_CMND_NVMEM_WRITE, ptr, arg_len, ulLength);
	
	SimpleLinkWaitEvent(HCI_EVNT_NVMEM_WRITE, &iRes);

    return(iRes);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

