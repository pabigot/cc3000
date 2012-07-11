/*****************************************************************************
*
*  cc3000_common.h  - CC3000 Host Driver Implementation.
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
#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdlib.h>
#include <errno.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef  __cplusplus
extern "C" {
#endif

//*****************************************************************************
//                  ERROR CODES
//*****************************************************************************
#define ESUCCESS        0
#define EFAIL          -1
#define EERROR          EFAIL

#define ERROR_SOCKET_INACTIVE   -57 

#define HCI_CC_PAYLOAD_LEN      5

#define WLAN_ENABLE      (1)   
#define WLAN_DISABLE     (0)


//*****************************************************************************
//                  Compound Types
//*****************************************************************************
typedef long time_t;
typedef unsigned long clock_t;
typedef long suseconds_t;

typedef struct timeval timeval;

struct timeval 
{
    time_t         tv_sec;                  /* seconds */
    suseconds_t    tv_usec;                 /* microseconds */
};

typedef char *(*tFWPatches)(unsigned long *usLength);

typedef char *(*tDriverPatches)(unsigned long *usLength);

typedef char *(*tBootLoaderPatches)(unsigned long *usLength);

typedef void (*tWlanCB)(long event_type, char * data, unsigned char length );

typedef long (*tWlanReadInteruptPin)(void);

typedef void (*tWlanInterruptEnable)(void);

typedef void (*tWlanInterruptDisable)(void);

typedef void (*tWriteWlanPin)(unsigned char val);

typedef struct
{
	unsigned short	 usRxEventOpcode;
	unsigned short	 usEventOrDataReceived;
	unsigned char 	*pucReceivedData;
	unsigned char 	*pucTxCommandBuffer;

	tFWPatches 			sFWPatches;
	tDriverPatches 		sDriverPatches;
	tBootLoaderPatches 	sBootLoaderPatches;
	tWlanCB	 			sWlanCB;
    tWlanReadInteruptPin  ReadWlanInterruptPin;
    tWlanInterruptEnable  WlanInterruptEnable;
    tWlanInterruptDisable WlanInterruptDisable;
    tWriteWlanPin         WriteWlanPin;

	signed long		 slTransmitDataError;
	unsigned short	 usNumberOfFreeBuffers;
	unsigned short	 usSlBufferLength;
	unsigned short	 usBufferSize;
	unsigned short	 usRxDataPending;
}sSimplLinkInformation;



extern volatile sSimplLinkInformation tSLInformation;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************



/**
 *  \brief Wait event
 *
 *
 * \param[in] usOpcode 
 * \param[out] pRetParams   
 * \return						
 *                              
 * \note
 * \warning
 */

extern void SimpleLinkWaitEvent(unsigned short usOpcode, void *pRetParams);

/**
 *  \brief Wait data
 *
 *
 * \param[out] pBuf 
 * \param[out] from   
 * \param[out] fromlen
 * \return						
 *                              
 * \note
 * \warning
 */

extern void SimpleLinkWaitData(unsigned char *pBuf, unsigned char *from, unsigned char *fromlen);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __COMMON_H__
