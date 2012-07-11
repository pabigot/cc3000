/*****************************************************************************
*
*  hci.c  - CC3000 Host Driver Implementation.
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
//! \addtogroup hci_app
//! @{
//
//*****************************************************************************

#include <cc3000/cc3000_common.h>
#include <cc3000/hci.h>
#include <cc3000/spi.h>
#include <cc3000/evnt_handler.h>
#include <cc3000/wlan.h>

#define SL_PATCH_PORTION_SIZE		(1000)


//*****************************************************************************
//
//!  Initiate an HCI cmnd.
//!
//!  \param  usOpcode     command operation code
//!  \param  ucArgs       pointer to the command's arguments buffer
//!  \param  ucArgsLength length of the arguments
//!
//!  \return              ESUCCESS if command transfer complete,EFAIL otherwise.
//!
//!  \brief               Initiate an HCI cmnd.
//
//*****************************************************************************
unsigned short 
hci_command_send(unsigned short usOpcode, unsigned char *pucBuff,
                     unsigned char ucArgsLength)
{ 
    hci_cmnd_hdr_t *hci_cmnd_hdr_ptr;
 
    hci_cmnd_hdr_ptr = (hci_cmnd_hdr_t *)(pucBuff + SPI_HEADER_SIZE);

    hci_cmnd_hdr_ptr->ucType = HCI_TYPE_CMND;
    hci_cmnd_hdr_ptr->usOpcode = usOpcode;
    hci_cmnd_hdr_ptr->ucLength = ucArgsLength;

    //
	// Update the opcode of the event we will be waiting for
	//
    SpiWrite(pucBuff, ucArgsLength + sizeof(hci_cmnd_hdr_t));


    return(0);
}

//*****************************************************************************
//
//!  HCI data command builder.
//!
//!  \param  usOpcode    command operation code
//!  \param  ucPayload   pointer to the data buffer
//!  \param  usLength    buffer length
//!
//!  \return none
//!
//!  \brief              Initiate an HCI data write operation
//
//*****************************************************************************
long
hci_data_send(unsigned char ucOpcode, 
                           unsigned char *ucArgs,
                           unsigned short usArgsLength, 
                           unsigned short usDataLength,
                           const unsigned char *ucTail,
                           unsigned short usTailLength)
{
    hci_data_hdr_t *hci_data_hdr_ptr;
	



    hci_data_hdr_ptr = (hci_data_hdr_t *)((ucArgs) + SPI_HEADER_SIZE);
    

	//
	// Fill in the HCI header of data packet
	//
    hci_data_hdr_ptr->ucType = HCI_TYPE_DATA;
    hci_data_hdr_ptr->ucOpcode = ucOpcode;
    hci_data_hdr_ptr->ucArgsize = usArgsLength;
    hci_data_hdr_ptr->usLength = usArgsLength + usDataLength + usTailLength;

	//
	// Send the packet over the SPI
	//
    SpiWrite(ucArgs, sizeof(hci_data_hdr_t) + usArgsLength + usDataLength + usTailLength);

    return(ESUCCESS);
}


//*****************************************************************************
//
//!  HCI data command send.
//!
//!  \param  usOpcode    command operation code
//!  \param  ucPayload   pointer to the data buffer
//!  \param  usLength    buffer length
//!
//!  \return none
//!
//!  \brief              Initiate an HCI data write operation
//
//*****************************************************************************
void hci_data_command_send(unsigned short usOpcode, unsigned char *pucBuff,
                     unsigned char ucArgsLength,unsigned short ucDataLength)
{ 
    hci_data_cmd_hdr_t *hci_cmnd_hdr_ptr;
 
    hci_cmnd_hdr_ptr = (hci_data_cmd_hdr_t *)(pucBuff + SPI_HEADER_SIZE);

    hci_cmnd_hdr_ptr->ucType = HCI_TYPE_DATA;
    hci_cmnd_hdr_ptr->ucOpcode = usOpcode;
    hci_cmnd_hdr_ptr->ucArgLength = ucArgsLength;
	hci_cmnd_hdr_ptr->usTotalLength = ucArgsLength + ucDataLength;

    //
	// Send teh command over SPI on data channel
	//
    SpiWrite(pucBuff, ucArgsLength + ucDataLength + sizeof(hci_data_cmd_hdr_t));


    return;
}

//*****************************************************************************
//
//!  Initiate an HCI Patch Transfer.
//!
//!  \param  usOpcode     command operation code
//!  \param  ucArgs       pointer to the command's arguments buffer
//!  \param  ucArgsLength length of the arguments
//!
//!  \return              ESUCCESS if command transfer complete,EFAIL otherwise.
//!
//!  \brief               Initiate an HCI cmnd.
//
//*****************************************************************************
void
hci_patch_send(unsigned char ucOpcode, unsigned char *pucBuff, char *patch, unsigned short usDataLength)
{ 
    hci_patch_hdr_t *hci_patch_hdr_ptr;
 	unsigned char *data_ptr = (pucBuff + SPI_HEADER_SIZE);
	unsigned short usTransLength;
	
	
    hci_patch_hdr_ptr = (hci_patch_hdr_t *)(pucBuff + SPI_HEADER_SIZE);

    hci_patch_hdr_ptr->ucType = HCI_TYPE_PATCH;
    hci_patch_hdr_ptr->ucOpcode = ucOpcode;
    hci_patch_hdr_ptr->usLength = usDataLength + sizeof(hci_patch_hdr_ptr->usTransactionLength);

	if (usDataLength <= SL_PATCH_PORTION_SIZE)
	{
		hci_patch_hdr_ptr->usTransactionLength = usDataLength;
		
		memcpy(hci_patch_hdr_ptr + 1, patch, usDataLength);

		
		//
		// Update the opcode of the event we will be waiting for
		//
		SpiWrite(pucBuff, usDataLength + sizeof(hci_patch_hdr_t));
	}
	else
	{
                usTransLength = (usDataLength/SL_PATCH_PORTION_SIZE);
		hci_patch_hdr_ptr->usLength += usTransLength*sizeof(hci_patch_hdr_ptr->usTransactionLength);
		
		hci_patch_hdr_ptr->usTransactionLength = SL_PATCH_PORTION_SIZE;
		memcpy(hci_patch_hdr_ptr + 1, patch, SL_PATCH_PORTION_SIZE);
		usDataLength -= SL_PATCH_PORTION_SIZE;
		patch += SL_PATCH_PORTION_SIZE;
		
		//
		// Update the opcode of the event we will be waiting for
		//
		SpiWrite(pucBuff, SL_PATCH_PORTION_SIZE + sizeof(hci_patch_hdr_t));

		while (usDataLength)
		{
			if (usDataLength <= SL_PATCH_PORTION_SIZE)
			{
				usTransLength = usDataLength;
				usDataLength = 0;
				
			}
			else
			{
				usTransLength = SL_PATCH_PORTION_SIZE;
				usDataLength -= usTransLength;
			}

			*(unsigned short *)data_ptr = usTransLength;
			memcpy(data_ptr + sizeof(hci_patch_hdr_ptr->usTransactionLength), patch, usTransLength);
			patch += usTransLength;

			 //
			 // Update the opcode of the event we will be waiting for
			 //
		    SpiWrite((unsigned char *)data_ptr, usTransLength + sizeof(usTransLength));
		}
	}
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//
//*****************************************************************************
