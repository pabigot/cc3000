/*****************************************************************************
*
*  cc3000_common.c.c  - CC3000 Host Driver Implementation.
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
//! \addtogroup common_api
//! @{
//
//*****************************************************************************

#include "cc3000_common.h"
#include "socket.h"
#include "wlan.h"
#include "evnt_handler.h"

//*****************************************************************************
//
//! void __error__(char *pcFilename, unsigned long ulLine)
//!
//!  \param  pcFilename - file name, where error occurred
//!  \param  ulLine     - line number, where error occurred
//!
//!  \return none
//!
//!  \brief stub function for ASSERT macro
//
//*****************************************************************************
void
__error__(char *pcFilename, unsigned long ulLine)
{
    //TODO full up function
}


void 
SimpleLinkWaitEvent(unsigned short usOpcode, void *pRetParams)
{
	//
	// In the blocking implementation the control to caller will be returned only after the 
	// end of current transaction
	//
	tSLInformation.usRxEventOpcode = usOpcode;
	hci_event_handler(pRetParams, 0, 0);
}


void 
SimpleLinkWaitData(unsigned char *pBuf, unsigned char *from, unsigned char *fromlen)
{
	//
	// In the blocking implementation the control to caller will be returned only after the 
	// end of current transaction, i.e. only after data will be received
	//
	tSLInformation.usRxDataPending = 1;
	hci_event_handler(pBuf, from, fromlen);
}






//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
