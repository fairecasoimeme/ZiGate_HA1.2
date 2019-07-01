/****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 ****************************************************************************/
    
    
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "SerialLink.h"
#include "app_poll_control_commands_handler.h"
#include "dbg.h"
#include "string.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/


/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vAPP_PollControlSetCheckinParms(uint16 u16PacketLength,
											 uint8 *pu8LinkRxBuffer,
											 uint8 *pu8Status);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bEnableFastPolling = FALSE;
PRIVATE uint16 u16FastPollTimeout = 0;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: APP_vCMDHandleAHICommand
 *
 * DESCRIPTION:
 * The main function that figures out which AHI command is to be executed
 * and passes the data to the correct function.
 *
 *
 ****************************************************************************/
PUBLIC void APP_vCMDHandlePollControlCommand(uint16 u16PacketType,
											 uint16 u16PacketLength,
											 uint8 *pu8LinkRxBuffer,
											 uint8 *pu8Status)
{
	uint8 u8Status;

	switch (u16PacketType)
	{
		case E_SL_MSG_POLL_CONTROL_SET_CHECKIN_PARAMS:
		{
			vAPP_PollControlSetCheckinParms(u16PacketLength, pu8LinkRxBuffer, &u8Status);
			break;
		}

		default:
			u8Status = 1;
			break;
	}

	*pu8Status = u8Status;
}

/****************************************************************************
 *
 * NAME: APP_vCMDHandleAHICommand
 *
 * DESCRIPTION:
 * The main function that figures out which AHI command is to be executed
 * and passes the data to the correct function.
 *
 *
 ****************************************************************************/
PUBLIC void APP_vCMDHandlePollControlEndpointCallback(tsZCL_CallBackEvent *psEvent)
{
	tsCLD_PollControlCallBackMessage *pCustom = (tsCLD_PollControlCallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;

	switch (pCustom->u8CommandId)
	{
		case E_CLD_POLL_CONTROL_CMD_CHECKIN:
		{
			/* Construct the response payload to what was previously configured via the host */
			pCustom->uMessage.psCheckinResponsePayload->bStartFastPolling = bEnableFastPolling;
			pCustom->uMessage.psCheckinResponsePayload->u16FastPollTimeout = u16FastPollTimeout;
			break;
		}

		default:
			break;
	}
}
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vAPP_PollControlSetCheckinParms
 *
 * DESCRIPTION:
 * Parse the data from the host and set the global poll control flags ready for
 * the poll check in request
 *
 * Params:
 *
 *
 ****************************************************************************/
PRIVATE void vAPP_PollControlSetCheckinParms(uint16 u16PacketLength, uint8 *pu8LinkRxBuffer, uint8 *pu8Status)
{
	uint32 u32BytesRead = 0;

	*pu8Status = 1;

	if (3 == u16PacketLength)
	{
		/* Parse the information out */
		bEnableFastPolling = pu8LinkRxBuffer[ u32BytesRead ];
		u32BytesRead += sizeof(bEnableFastPolling);

		memcpy(&u16FastPollTimeout, &pu8LinkRxBuffer[ u32BytesRead ], sizeof(u16FastPollTimeout));
		u32BytesRead += sizeof(u16FastPollTimeout);

		DBG_vPrintf(TRUE, "Poll Control: %s bEnable = %d, Fast Polling (1/4 secs) = %d", __FUNCTION__,
				bEnableFastPolling,
				u16FastPollTimeout);

		*pu8Status = 0;
	}
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
