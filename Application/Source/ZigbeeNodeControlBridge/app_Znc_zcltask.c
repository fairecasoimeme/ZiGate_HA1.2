/*****************************************************************************
 *
 * MODULE: ZigbeeNodeControlBridge
 *
 * COMPONENT: app_Znc_zcltask.c
 *
 * $AUTHOR: Faisal Bhaiyat$
 *
 * DESCRIPTION:
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Zigbee%20Protocol%20Stack/ZPS/Trunk/ZigbeeNodeControlBridge/Source/ZigbeeNodeControlBridge/app_Znc_zcltask.c $
 *
 * $Revision: 54776 $
 *
 * $LastChangedBy: nxp29741 $
 *
 * $LastChangedDate: 2013-06-20 11:50:33 +0100 (Thu, 20 Jun 2013) $
 *
 * $Id: app_Znc_zcltask.c 54776 2013-06-20 10:50:33Z nxp29741 $
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5164,
 * JN5161, JN5148, JN5142, JN5139].
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
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <appapi.h>
#include "os.h"
#include "os_gen.h"
#include "zps_gen.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "pdm.h"
#include "dbg.h"
#include "pwrm.h"

#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "zps_apl_aib.h"
#include "zps_apl_zdp.h"
#include "rnd_pub.h"
#include "mac_pib.h"
#include "string.h"

#include "app_timer_driver.h"

#include "zcl_options.h"
#include "zll.h"
#include "zll_commission.h"
#include "commission_endpoint.h"
#include "app_common.h"
#include "app_ZncParser_task.h"
#include "ahi_aes.h"
#include "app_events.h"
#include "app_Znc_zcltask.h"
#include "Log.h"
#include "SerialLink.h"

#ifdef CLD_POLL_CONTROL
#include "app_poll_control_commands_handler.h"
#endif

#ifdef STACK_MEASURE
#include "StackMeasure.h"
#endif

#ifdef DEBUG_ZCL
#define TRACE_ZCL   TRUE
#else
#define TRACE_ZCL   FALSE
#endif

#ifdef DEBUG_ZB_CONTROLBRIDGE_TASK
#define TRACE_ZB_CONTROLBRIDGE_TASK   TRUE
#else
#define TRACE_ZB_CONTROLBRIDGE_TASK   FALSE
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/



/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent);
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent);


PRIVATE void APP_ZCL_cbZllCommissionCallback(tsZCL_CallBackEvent *psEvent);
PRIVATE void APP_ZCL_cbZllUtilityCallback(tsZCL_CallBackEvent *psEvent);

PRIVATE uint32 APP_u32CheckIfAttributeIsString(tsZCL_IndividualAttributesResponse *sIndividualAttributeResponse);
PRIVATE void APP_vCopyAttributeDataIntoBuffer(uint16 u16SizeOfAttribute,
											  teZCL_ZCLAttributeType eAttributeDataType,
											  void * pvAttributeData,
											  uint8 **ppu8Buffer);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
tsZLL_CommissionEndpoint sCommissionEndpoint;
tsZLL_ZncControlBridgeDevice sControlBridge;
#ifdef CLD_OTA
	PRIVATE uint8 attribData[CLD_OTA_MAX_NUMBER_OF_ATTRIBUTE];
#endif

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void* psGetDeviceTable(void) {
    return &sDeviceTable;
}

/****************************************************************************
 *
 * NAME: APP_ZCL_vInitialise
 *
 * DESCRIPTION:
 * Initialises ZCL related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_ZCL_vInitialise(void)
{
    teZCL_Status eZCL_Status;

    /* Initialise ZLL */
    eZCL_Status = eZLL_Initialise(&APP_ZCL_cbGeneralCallback, apduZDP);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        vLog_Printf(TRACE_ZCL,LOG_DEBUG, "Error: eZLL_Initialise returned %d\r\n", eZCL_Status);
    }

    /* Register Commission EndPoint */
    eZCL_Status = eApp_ZLL_RegisterEndpoint(&APP_ZCL_cbEndpointCallback);
    if (eZCL_Status != E_ZCL_SUCCESS)
    {
        vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG,"eZLL_RegisterCommissionEndPoint %x\n",eZCL_Status );
    }
    vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "Chan Mask %08x\n", ZPS_psAplAibGetAib()->apsChannelMask);
    vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\nRxIdle TRUE");

    sDeviceTable.asDeviceRecords[0].u64IEEEAddr = *((uint64*)pvAppApiGetMacAddrLocation());

    vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\ntsCLD_Groups %d", sizeof(tsCLD_Groups));
    vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\ntsCLD_GroupTableEntry %d", sizeof(tsCLD_GroupTableEntry));
    vAPP_ZCL_DeviceSpecific_Init();
}

/****************************************************************************
 *
 * NAME: ZCL_Task
 *
 * DESCRIPTION:
 * Main state machine for the IPD
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
OS_TASK(ZCL_Task)
{
    APP_tsEvent sAppEvent;
    APP_CommissionEvent sCommissionEvent;
    ZPS_tsAfEvent sStackEvent;
    tsZCL_CallBackEvent sCallBackEvent;

    uint8 au8StatusBuffer[256];
    uint8* pu8Buffer;
    uint16 u16Length = 0;
    uint8 				   u8LinkQuality;

    u8LinkQuality=sStackEvent.uEvent.sApsDataIndEvent.u8LinkQuality;
    sCallBackEvent.pZPSevent = &sStackEvent;

    /*
     * If the 1 second tick timer has expired, restart it and pass
     * the event on to ZCL
     */
    if (OS_eGetSWTimerStatus(APP_TickTimer) == OS_E_SWTIMER_EXPIRED)
    {

#ifdef STACK_MEASURE
    	tsStackInfo sStackInfo;

    	vGetStackMeasure( &sStackInfo );

    	DBG_vPrintf(TRUE, "\r\n TotalSize: %d, uPeakMeasure: %d, Current: %d ", sStackInfo.u32TotalSize, sStackInfo.u32PeakMeasure, sStackInfo.u32CurrentMeasure);
#endif
    	//vLog_Printf(TRACE_ZCL, LOG_DEBUG, "\nE_ZCL_CBET_TIMER");
        sCallBackEvent.eEventType = E_ZCL_CBET_TIMER;
        vZCL_EventHandler(&sCallBackEvent);
        OS_eContinueSWTimer(APP_TickTimer, ZCL_TICK_TIME, NULL);
    }

    /* If there is a stack event to process, pass it on to ZCL */
    sStackEvent.eType = ZPS_EVENT_NONE;
    if (OS_eCollectMessage(APP_msgZpsEvents_ZCL, &sStackEvent) == OS_E_OK)
    {
        vLog_Printf(TRACE_ZCL,LOG_DEBUG, "ZCL_Task got event %d\r\n",sStackEvent.eType);

        switch(sStackEvent.eType)
        {

        case ZPS_EVENT_APS_DATA_INDICATION:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\nDATA: SEP=%d DEP=%d Profile=%04x Cluster=%04x\n",
                        sStackEvent.uEvent.sApsDataIndEvent.u8SrcEndpoint,
                        sStackEvent.uEvent.sApsDataIndEvent.u8DstEndpoint,
                        sStackEvent.uEvent.sApsDataIndEvent.u16ProfileId,
                        sStackEvent.uEvent.sApsDataIndEvent.u16ClusterId);
#ifdef APP_ZCL_ON_HOST
        	{
        			pu8Buffer = au8StatusBuffer;
        			Znc_vSendDataIndicationToHost(&sStackEvent, pu8Buffer);
        	}
#endif
            break;

        case ZPS_EVENT_APS_DATA_CONFIRM:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\nCFM: SEP=%d DEP=%d Status=%d\n",
                        sStackEvent.uEvent.sApsDataConfirmEvent.u8SrcEndpoint,
                        sStackEvent.uEvent.sApsDataConfirmEvent.u8DstEndpoint,
                        sStackEvent.uEvent.sApsDataConfirmEvent.u8Status);

            if(sStackEvent.uEvent.sApsDataConfirmEvent.u8Status)
			{
            	pu8Buffer = au8StatusBuffer;
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.u8Status,sizeof(uint8));
				u16Length += sizeof(uint8);
				pu8Buffer += sizeof(uint8);
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.u8SrcEndpoint,sizeof(uint8));
				u16Length += sizeof(uint8);
				pu8Buffer += sizeof(uint8);
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.u8DstEndpoint,sizeof(uint8));
				u16Length += sizeof(uint8);
				pu8Buffer += sizeof(uint8);
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.u8DstAddrMode,sizeof(uint8));
				u16Length += sizeof(uint8);
				pu8Buffer += sizeof(uint8);
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.uDstAddr,sizeof(ZPS_tuAddress));
				u16Length += sizeof(ZPS_tuAddress);
				pu8Buffer += sizeof(ZPS_tuAddress);
				memcpy(pu8Buffer, &sStackEvent.uEvent.sApsDataConfirmEvent.u8SequenceNum,sizeof(uint8));
				u16Length += sizeof(uint8);
				vSL_WriteMessage(E_SL_MSG_APS_DATA_CONFIRM_FAILED, u16Length, au8StatusBuffer,u8LinkQuality);

                sAppEvent.eType = APP_E_EVENT_DATA_CONFIRM_FAILED;
            }
            else
            {
                sAppEvent.eType = APP_E_EVENT_DATA_CONFIRM;
            }
            OS_ePostMessage(APP_msgEvents, &sAppEvent);
            break;

        case ZPS_EVENT_APS_DATA_ACK:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\nACK: SEP=%d DEP=%d Profile=%04x Cluster=%04x\n",
                        sStackEvent.uEvent.sApsDataAckEvent.u8SrcEndpoint,
                        sStackEvent.uEvent.sApsDataAckEvent.u8DstEndpoint,
                        sStackEvent.uEvent.sApsDataAckEvent.u16ProfileId,
                        sStackEvent.uEvent.sApsDataAckEvent.u16ClusterId);
            sStackEvent.eType = ZPS_EVENT_APS_DATA_ACK;
            OS_ePostMessage(APP_msgZpsEvents, &sStackEvent);
            break;
        case ZPS_EVENT_APS_INTERPAN_DATA_CONFIRM:

            if(sStackEvent.uEvent.sApsInterPanDataConfirmEvent.u8Status)
            {
                sCommissionEvent.eType = APP_E_COMMISSION_NOACK;
            }
            else
            {
                sCommissionEvent.eType = APP_E_COMMISSION_ACK;
            }
            OS_ePostMessage(APP_CommissionEvents, &sCommissionEvent);
            break;
        default:
            break;
        }

#ifdef APP_EXTENDED_DEBUG
        if((ZPS_EVENT_APS_DATA_CONFIRM == sStackEvent.eType) && (0 != sStackEvent.uEvent.sApsDataAckEvent.u8Status))
		{
			DBG_vPrintf(TRUE, "\nExt Debug: ZPS_EVENT_APS_DATA_CONFIRM %x",sStackEvent.uEvent.sApsDataAckEvent.u8Status);
		}
		if((ZPS_EVENT_APS_DATA_ACK == sStackEvent.eType) && (0 != sStackEvent.uEvent.sApsDataAckEvent.u8Status))
		{
			DBG_vPrintf(TRUE, "\nExt Debug: ZPS_EVENT_APS_DATA_ACK %x",sStackEvent.uEvent.sApsDataAckEvent.u8Status);
		}
		if(ZPS_EVENT_ERROR == sStackEvent.eType)
		{
			DBG_vPrintf(TRUE, "\nExt Debug: ZPS_EVENT_ERROR = %x", sStackEvent.uEvent.sAfErrorEvent.eError);
		}
#endif
        sCallBackEvent.eEventType = E_ZCL_CBET_ZIGBEE_EVENT;
        sCallBackEvent.pZPSevent = &sStackEvent;
        vZCL_EventHandler(&sCallBackEvent);
    }
}


/****************************************************************************
 *
 * NAME: APP_ZCL_cbGeneralCallback
 *
 * DESCRIPTION:
 * General callback for ZCL events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbGeneralCallback(tsZCL_CallBackEvent *psEvent)
{
#if TRUE == TRACE_ZCL
    switch (psEvent->eEventType)
    {
        case E_ZCL_CBET_LOCK_MUTEX:
            //vLog_Printf(TRACE_ZCL,LOG_DEBUG,"EVT: Lock Mutex\r\n");
            break;

        case E_ZCL_CBET_UNLOCK_MUTEX:
            //vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Unlock Mutex\r\n");
            break;

        case E_ZCL_CBET_UNHANDLED_EVENT:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Unhandled Event\r\n");
            break;

        case E_ZCL_CBET_READ_ATTRIBUTES_RESPONSE:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Read attributes response\r\n");
            break;

        case E_ZCL_CBET_READ_REQUEST:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Read request\r\n");
            break;

        case E_ZCL_CBET_DEFAULT_RESPONSE:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Default response\r\n");
            break;

        case E_ZCL_CBET_ERROR:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: Error\r\n");
            break;

        case E_ZCL_CBET_TIMER:
            break;

        case E_ZCL_CBET_ZIGBEE_EVENT:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EVT: ZigBee\r\n");
            break;

        case E_ZCL_CBET_CLUSTER_CUSTOM:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EP EVT: Custom\r\n");
            break;

        default:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "Invalid event type\r\n");
            break;
    }
#endif
}


/****************************************************************************
 *
 * NAME: APP_ZCL_cbEndpointCallback
 *
 * DESCRIPTION:
 * Endpoint specific callback for ZCL events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbEndpointCallback(tsZCL_CallBackEvent *psEvent)
{
    uint16 u16Length;
    uint8 au8StatusBuffer[256];
    uint8* pu8Buffer;
    uint8 u8LinkQuality;

    u8LinkQuality=psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8LinkQuality;

      vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\nEntering cbZCL_EndpointCallback");

    switch (psEvent->eEventType)
    {
        case E_ZCL_CBET_LOCK_MUTEX:
        case E_ZCL_CBET_UNLOCK_MUTEX:
        case E_ZCL_CBET_READ_ATTRIBUTES_RESPONSE:
        case E_ZCL_CBET_READ_REQUEST:
        case E_ZCL_CBET_TIMER:
        case E_ZCL_CBET_ZIGBEE_EVENT:
            //vLog_Printf(TRACE_ZCL, "EP EVT:No action\r\n");
            break;

        case E_ZCL_CBET_ERROR:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_ERROR) - Error: 0x%02x", psEvent->eZCL_Status);
        }
        break;

        case E_ZCL_CBET_UNHANDLED_EVENT:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_UNHANDLED_EVENT)");
        }
        break;

        case E_ZCL_CBET_DEFAULT_RESPONSE:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DEFAULT_RESPONSE)");

            if(psEvent->psClusterInstance != NULL)
            {
                pu8Buffer = au8StatusBuffer;
                memcpy(pu8Buffer,&psEvent->u8TransactionSequenceNumber,sizeof(uint8) );
                u16Length = sizeof(uint8);
                pu8Buffer += sizeof(uint8);
                memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr,sizeof(uint16) );
                u16Length += sizeof(uint16);
                pu8Buffer += sizeof(uint16);
                memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint,sizeof(uint8) );
                u16Length += sizeof(uint8);
                pu8Buffer += sizeof(uint8);
                memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8DstEndpoint,sizeof(uint8) );
                u16Length += sizeof(uint8);
                pu8Buffer += sizeof(uint8);
                memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId,sizeof(uint16) );
                u16Length += sizeof(uint16);
                pu8Buffer += sizeof(uint16);
                memcpy(pu8Buffer,&psEvent->uMessage.sDefaultResponse.u8CommandId,sizeof(uint8) );
                u16Length += sizeof(uint8);
                pu8Buffer += sizeof(uint8);
                memcpy(pu8Buffer,&psEvent->uMessage.sDefaultResponse.u8StatusCode,sizeof(uint8) );
                u16Length += sizeof(uint8);
                vSL_WriteMessage(E_SL_MSG_DEFAULT_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
            }
        }
        break;


        case E_ZCL_CBET_WRITE_ATTRIBUTES_RESPONSE:
        case E_ZCL_CBET_REPORT_INDIVIDUAL_ATTRIBUTE:
        case E_ZCL_CBET_READ_INDIVIDUAL_ATTRIBUTE_RESPONSE:
        {

        	uint32 u32Elements = APP_u32CheckIfAttributeIsString(&psEvent->uMessage.sIndividualAttributeResponse);

            uint16 u16SizeOfAttribute = u16GetAttributeActualSize(psEvent->uMessage.sIndividualAttributeResponse.eAttributeDataType,u32Elements);
            vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, " Read Attrib Rsp %d %02x %d %d\n", psEvent->uMessage.sIndividualAttributeResponse.eAttributeStatus,
                    *((uint8*)psEvent->uMessage.sIndividualAttributeResponse.pvAttributeData),
                    		u32Elements,
                    		u16SizeOfAttribute);

                if(u16SizeOfAttribute!=0)
                {
                    /* Send event upwards */
                    pu8Buffer = au8StatusBuffer;
                    memcpy(pu8Buffer,&psEvent->u8TransactionSequenceNumber,sizeof(uint8) );
                    u16Length = sizeof(uint8);
                    pu8Buffer += sizeof(uint8);
                    memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr,sizeof(uint16) );
                    u16Length += sizeof(uint16);
                    pu8Buffer += sizeof(uint16);
                    memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint,sizeof(uint8) );
                    u16Length += sizeof(uint8);
                    pu8Buffer += sizeof(uint8);
                    memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId,sizeof(uint16) );
                    u16Length += sizeof(uint16);
                    pu8Buffer += sizeof(uint16);
                    memcpy(pu8Buffer,&psEvent->uMessage.sIndividualAttributeResponse.u16AttributeEnum,sizeof(uint16) );
                    u16Length += sizeof(uint16);
                    pu8Buffer += sizeof(uint16);
                    if (psEvent->eEventType != E_ZCL_CBET_REPORT_INDIVIDUAL_ATTRIBUTE)
                    {
                        memcpy(pu8Buffer,&psEvent->uMessage.sIndividualAttributeResponse.eAttributeStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                    }
                    memcpy(pu8Buffer,&psEvent->uMessage.sIndividualAttributeResponse.eAttributeDataType,sizeof(uint8) );
                    u16Length += sizeof(uint8);
                    pu8Buffer += sizeof(uint8);
                    memcpy(pu8Buffer,&u16SizeOfAttribute,sizeof(uint16));
					u16Length += sizeof(uint16);
					pu8Buffer += sizeof(uint16);

					APP_vCopyAttributeDataIntoBuffer(u16SizeOfAttribute,
													 psEvent->uMessage.sIndividualAttributeResponse.eAttributeDataType,
													 psEvent->uMessage.sIndividualAttributeResponse.pvAttributeData,
													 &pu8Buffer);

                    u16Length += u16SizeOfAttribute;
                    if((psEvent->eEventType == E_ZCL_CBET_READ_INDIVIDUAL_ATTRIBUTE_RESPONSE))
                        vSL_WriteMessage(E_SL_MSG_READ_ATTRIBUTE_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
                    else if((psEvent->eEventType == E_ZCL_CBET_REPORT_INDIVIDUAL_ATTRIBUTE))
                        vSL_WriteMessage(E_SL_MSG_REPORT_IND_ATTR_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
                    else if((psEvent->eEventType == E_ZCL_CBET_WRITE_ATTRIBUTES_RESPONSE))
                        vSL_WriteMessage(E_SL_MSG_WRITE_ATTRIBUTE_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
                }
                if((psEvent->eEventType == E_ZCL_CBET_READ_INDIVIDUAL_ATTRIBUTE_RESPONSE) &&
                        (psEvent->uMessage.sIndividualAttributeResponse.u16AttributeEnum == E_CLD_BAS_ATTR_ID_ZCL_VERSION) &&
                        (psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ClusterId == GENERAL_CLUSTER_ID_BASIC))
                {
                    uint8 u8Seq;
                    tsZCL_Address sAddress;
                    sAddress.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
                    sAddress.uAddress.u16DestinationAddress = psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr;
                    tsCLD_Identify_IdentifyRequestPayload sPayload;
                    sPayload.u16IdentifyTime = 0x0003;

                    eCLD_IdentifyCommandIdentifyRequestSend(
                            sDeviceTable.asDeviceRecords[0].u8Endpoint,
                            psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint,
                            &sAddress,
                            &u8Seq,
                            &sPayload);

                }
        }
        break;

		case E_ZCL_CBET_REPORT_ATTRIBUTES_CONFIGURE_RESPONSE:
		{
			vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_REPORT_ATTRIBUTES_CONFIGURE_RESPONSE)");

			/* Send event upwards */
            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer,&psEvent->u8TransactionSequenceNumber, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr,sizeof(uint16) );
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.eCommandStatus, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);

			vSL_WriteMessage(E_SL_MSG_CONFIG_REPORTING_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
		}
		break;

		case E_ZCL_CBET_REPORT_READ_INDIVIDUAL_ATTRIBUTE_CONFIGURATION_RESPONSE:
		{
			vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_REPORT_READ_INDIVIDUAL_ATTRIBUTE_CONFIGURATION_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer,&psEvent->u8TransactionSequenceNumber, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr,sizeof(uint16) );
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.eCommandStatus, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.sAttributeReportingConfigurationRecord.eAttributeDataType, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.sAttributeReportingConfigurationRecord.u16AttributeEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.sAttributeReportingConfigurationRecord.u16MaximumReportingInterval, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer,&psEvent->uMessage.sAttributeReportingConfigurationResponse.sAttributeReportingConfigurationRecord.u16MinimumReportingInterval, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);

			vSL_WriteMessage(E_SL_MSG_READ_REPORT_CONFIG_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
		}
		break;
		
        case E_ZCL_CBET_DISCOVER_ATTRIBUTES_RESPONSE:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_ATTRIBUTES_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.bDiscoveryComplete, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.eAttributeDataType, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.u16AttributeEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);

            vSL_WriteMessage(E_SL_MSG_ATTRIBUTE_DISCOVERY_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;

        case E_ZCL_CBET_DISCOVER_INDIVIDUAL_ATTRIBUTE_RESPONSE:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_INDIVIDUAL_ATTRIBUTE_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.bDiscoveryComplete, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.eAttributeDataType, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryResponse.u16AttributeEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);

            vSL_WriteMessage(E_SL_MSG_ATTRIBUTE_DISCOVERY_INDIVIDUAL_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;

 		case E_ZCL_CBET_DISCOVER_INDIVIDUAL_ATTRIBUTE_EXTENDED_RESPONSE:
        {
            vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_INDIVIDUAL_ATTRIBUTE_EXTENDED_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryExtenedResponse.bDiscoveryComplete, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryExtenedResponse.eAttributeDataType, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryExtenedResponse.u16AttributeEnum, sizeof(uint16));
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
            memcpy(pu8Buffer, &psEvent->uMessage.sAttributeDiscoveryExtenedResponse.u8AttributeFlags, sizeof(uint8));
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);

            vSL_WriteMessage(E_SL_MSG_ATTRIBUTE_EXT_DISCOVERY_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;

        case E_ZCL_CBET_DISCOVER_INDIVIDUAL_COMMAND_RECEIVED_RESPONSE:
        {
        	vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_INDIVIDUAL_COMMAND_RECEIVED_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsReceivedDiscoveryIndividualResponse.u8CommandEnum, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsReceivedDiscoveryIndividualResponse.u8CommandIndex, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);

        	vSL_WriteMessage(E_SL_MSG_COMMAND_RECEIVED_DISCOVERY_INDIVIDUAL_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;
        
        case E_ZCL_CBET_DISCOVER_COMMAND_RECEIVED_RESPONSE:
        {
        	vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_COMMAND_RECEIVED_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsReceivedDiscoveryResponse.bDiscoveryComplete, sizeof(bool_t));
            u16Length = sizeof(bool_t);
            pu8Buffer += sizeof(bool_t);
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsReceivedDiscoveryResponse.u8NumberOfCommands, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);

        	vSL_WriteMessage(E_SL_MSG_COMMAND_RECEIVED_DISCOVERY_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;

        case E_ZCL_CBET_DISCOVER_INDIVIDUAL_COMMAND_GENERATED_RESPONSE:
        {
        	vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_INDIVIDUAL_COMMAND_GENERATED_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsGeneratedDiscoveryIndividualResponse.u8CommandEnum, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsGeneratedDiscoveryIndividualResponse.u8CommandIndex, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);

        	vSL_WriteMessage(E_SL_MSG_COMMAND_GENERATED_DISCOVERY_INDIVIDUAL_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;

        case E_ZCL_CBET_DISCOVER_COMMAND_GENERATED_RESPONSE:
        {
        	vLog_Printf(TRACE_ZCL, LOG_DEBUG, " (E_ZCL_CBET_DISCOVER_INDIVIDUAL_COMMAND_GENERATED_RESPONSE)");

            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsGeneratedDiscoveryResponse.bDiscoveryComplete, sizeof(bool_t));
            u16Length = sizeof(bool_t);
            pu8Buffer += sizeof(bool_t);
            memcpy(pu8Buffer, &psEvent->uMessage.sCommandsGeneratedDiscoveryResponse.u8NumberOfCommands, sizeof(uint8));
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);

        	vSL_WriteMessage(E_SL_MSG_COMMAND_GENERATED_DISCOVERY_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
        }
        break;
        
        case E_ZCL_CBET_CLUSTER_CUSTOM:
        {
            /* Send event upwards */
            pu8Buffer = au8StatusBuffer;
            memcpy(pu8Buffer,&psEvent->u8TransactionSequenceNumber,sizeof(uint8) );
            u16Length = sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint,sizeof(uint8) );
            u16Length += sizeof(uint8);
            pu8Buffer += sizeof(uint8);
            memcpy(pu8Buffer,&psEvent->psClusterInstance->psClusterDefinition->u16ClusterEnum,sizeof(uint16) );
            u16Length += sizeof(uint16);
            pu8Buffer += sizeof(uint16);
        vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EP EVT: Custom %04x\r\n", psEvent->uMessage.sClusterCustomMessage.u16ClusterId);

        switch (psEvent->uMessage.sClusterCustomMessage.u16ClusterId)
        {
		#ifdef CLD_OTA
        	case OTA_CLUSTER_ID:
        	{
        		tsOTA_CallBackMessage *psCallBackMessage = (tsOTA_CallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;

        		vLog_Printf(TRACE_ZCL, LOG_DEBUG, "OTA_CLUSTER_ID - Event ID: %d\r\n", psCallBackMessage->eEventId);

        		switch (psCallBackMessage->eEventId)
        		{
        			case E_CLD_OTA_COMMAND_QUERY_NEXT_IMAGE_REQUEST:
        			{
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "E_CLD_OTA_COMMAND_QUERY_NEXT_IMAGE_REQUEST\r\n");
        			}
        			break;

        			case E_CLD_OTA_COMMAND_BLOCK_REQUEST:
        			{
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "E_CLD_OTA_COMMAND_BLOCK_REQUEST\r\n");
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "SrcAddress: %04x\r\n", psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "RequestNodeAddress: %016llx\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u64RequestNodeAddress);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "FileOffset: %08x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u32FileOffset);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "FileVersion: %08x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u32FileVersion);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "ImageType: %04x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u16ImageType);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "ManufactureCode: %04x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u16ManufactureCode);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "BlockRequestDelay: %04x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u16BlockRequestDelay);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "MaxDataSize: %02x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u8MaxDataSize);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "FieldControl: %02x\r\n", psCallBackMessage->uMessage.sBlockRequestPayload.u8FieldControl);

        				memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode, sizeof(uint8));
        				u16Length += sizeof(uint8);
        				pu8Buffer += sizeof(uint8);
        				memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u64RequestNodeAddress, sizeof(uint64));
        				u16Length += sizeof(uint64);
        				pu8Buffer += sizeof(uint64);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u32FileOffset, sizeof(uint32));
        				u16Length += sizeof(uint32);
        				pu8Buffer += sizeof(uint32);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u32FileVersion, sizeof(uint32));
        				u16Length += sizeof(uint32);
        				pu8Buffer += sizeof(uint32);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u16ImageType, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u16ManufactureCode, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u16BlockRequestDelay, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u8MaxDataSize, sizeof(uint8));
        				u16Length += sizeof(uint8);
        				pu8Buffer += sizeof(uint8);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sBlockRequestPayload.u8FieldControl, sizeof(uint8));
        				u16Length += sizeof(uint8);
        				pu8Buffer += sizeof(uint8);

        				vSL_WriteMessage(E_SL_MSG_BLOCK_REQUEST, u16Length, au8StatusBuffer,u8LinkQuality);
        			}
        			break;

        			case E_CLD_OTA_COMMAND_UPGRADE_END_REQUEST:
        			{
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "E_CLD_OTA_COMMAND_UPGRADE_END_REQUEST\r\n");
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "SrcAddress: %04x\r\n", psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "FileVersion: %08x\r\n", psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u32FileVersion);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "ImageType: %04x\r\n", psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u16ImageType);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "ManufacturerCode: %04x\r\n", psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u16ManufacturerCode);
        				vLog_Printf(TRACE_ZCL, LOG_DEBUG, "Status: %02x\r\n", psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u8Status);

        				tsCLD_PR_Ota sOTAData;
        				(void)eOTA_GetServerData(ZIGBEENODECONTROLBRIDGE_HA_ENDPOINT, 0, &sOTAData);
        				sOTAData.u32RequestOrUpgradeTime = sOTAData.u32CurrentTime + 5;
        				(void)eOTA_SetServerParams(ZIGBEENODECONTROLBRIDGE_HA_ENDPOINT, 0, &sOTAData);

        				memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode, sizeof(uint8));
        				u16Length += sizeof(uint8);
        				pu8Buffer += sizeof(uint8);
        				memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u32FileVersion, sizeof(uint32));
        				u16Length += sizeof(uint32);
        				pu8Buffer += sizeof(uint32);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u16ImageType, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u16ManufacturerCode, sizeof(uint16));
        				u16Length += sizeof(uint16);
        				pu8Buffer += sizeof(uint16);
        				memcpy(pu8Buffer, &psCallBackMessage->uMessage.sUpgradeEndRequestPayload.u8Status, sizeof(uint8));
        				u16Length += sizeof(uint8);
        				pu8Buffer += sizeof(uint8);

        				vSL_WriteMessage(E_SL_MSG_UPGRADE_END_REQUEST, u16Length, au8StatusBuffer,u8LinkQuality);
        				uint32 i;
                        for(i = 0; i< OTA_TOTAL_ACTIVE_IMAGES_ON_SERVER; i++)
                        {
                        	psCallBackMessage->aServerPrams[i].u32CurrentTime = 0;
                        	psCallBackMessage->aServerPrams[i].u32RequestOrUpgradeTime = 5;
                        }
        			}
        			break;

        			default:
        			{
        				/* Do nothing */
        			}
        			break;
        		}
        	}
        	break;
		#endif

        	case GENERAL_CLUSTER_ID_ONOFF:
            {
                    tsCLD_OnOffCallBackMessage *psCallBackMessage = (tsCLD_OnOffCallBackMessage*)psEvent->uMessage.sClusterCustomMessage.pvCustomData;

                    vLog_Printf(TRACE_ZCL,LOG_DEBUG, "- for onoff cluster\r\n");
                    vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\r\nCMD: 0x%02x\r\n", psCallBackMessage->u8CommandId);

                    memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode, sizeof(uint8));
                    u16Length += sizeof(uint8);
                    pu8Buffer += sizeof(uint8);

                    if (psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode == 0x03)
                    {
                        memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u64Addr, sizeof(uint64));
                        u16Length += sizeof(uint64);
                        pu8Buffer += sizeof(uint64);
                    }
                    else
                    {
                        memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr, sizeof(uint16));
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                    }

                    memcpy(pu8Buffer, &psCallBackMessage->u8CommandId, sizeof(uint8));
                    u16Length += sizeof(uint8);
                    pu8Buffer += sizeof(uint8);

                    vSL_WriteMessage(E_SL_MSG_ONOFF_UPDATE, u16Length, au8StatusBuffer,u8LinkQuality);
                }
                break;
            case GENERAL_CLUSTER_ID_IDENTIFY:
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "- for identify cluster\r\n");
                break;

            case GENERAL_CLUSTER_ID_GROUPS:
            {
                uint16 u16Command = 0;
                tsCLD_GroupsCallBackMessage* pCustom = ((tsCLD_GroupsCallBackMessage*)psEvent->uMessage.sClusterCustomMessage.pvCustomData);
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "- for groups cluster\r\n");


                switch(pCustom->u8CommandId)
                {
                    case (E_CLD_GROUPS_CMD_ADD_GROUP):
                    {
                        memcpy(pu8Buffer,&pCustom->uMessage.psAddGroupResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psAddGroupResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        u16Command = E_SL_MSG_ADD_GROUP_RESPONSE;
                        break;
                    }

                    case (E_CLD_GROUPS_CMD_VIEW_GROUP):
                    {
                        memcpy(pu8Buffer,&pCustom->uMessage.psViewGroupResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psViewGroupResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        u16Command = E_SL_MSG_VIEW_GROUP_RESPONSE;
                        break;
                    }

                    case (E_CLD_GROUPS_CMD_GET_GROUP_MEMBERSHIP):
                    {
                        uint8 groupCount = pCustom->uMessage.psGetGroupMembershipResponsePayload->u8GroupCount;
                        memcpy(pu8Buffer,&pCustom->uMessage.psGetGroupMembershipResponsePayload->u8Capacity,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&groupCount,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,(uint8*)pCustom->uMessage.psGetGroupMembershipResponsePayload->pi16GroupList,(sizeof(uint16)*groupCount) );
                        u16Length += (sizeof(uint16)*groupCount);
                        pu8Buffer += (sizeof(uint16)*groupCount);
                        u16Command = E_SL_MSG_GET_GROUP_MEMBERSHIP_RESPONSE;
                        break;
                    }

                    case (E_CLD_GROUPS_CMD_REMOVE_GROUP):
                    {
                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveGroupResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveGroupResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        u16Command = E_SL_MSG_REMOVE_GROUP_RESPONSE;
                        break;
                    }

                    default:
                        break;
                }
                vSL_WriteMessage(u16Command, u16Length, au8StatusBuffer,u8LinkQuality);

            }//General Group cluster id
            break;

            case 0x1000:
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\n    - for 0x1000");
                if (psEvent->pZPSevent->eType == ZPS_EVENT_APS_INTERPAN_DATA_INDICATION &&  psEvent->pZPSevent->uEvent.sApsInterPanDataIndEvent.u16ProfileId == ZLL_PROFILE_ID) {
                    APP_ZCL_cbZllCommissionCallback(psEvent);
                } else if (psEvent->pZPSevent->eType == ZPS_EVENT_APS_DATA_INDICATION && psEvent->pZPSevent->uEvent.sApsDataIndEvent.u16ProfileId == HA_PROFILE_ID) {
                    APP_ZCL_cbZllUtilityCallback(psEvent);
                }
                
                break;

            case GENERAL_CLUSTER_ID_SCENES:
            {

                uint16 u16Command = 0;
                tsCLD_ScenesCallBackMessage* pCustom = ((tsCLD_ScenesCallBackMessage*)psEvent->uMessage.sClusterCustomMessage.pvCustomData);
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "- for groups cluster\r\n");
                switch(pCustom->u8CommandId)
                {
                    case (E_CLD_SCENES_CMD_ADD):
                    {
                        memcpy(pu8Buffer,&pCustom->uMessage.psAddSceneResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psAddSceneResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer,&pCustom->uMessage.psAddSceneResponsePayload->u8SceneId,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        u16Command = E_SL_MSG_ADD_SCENE_RESPONSE;
                    }
                    break;//ADD SCENE

                    case (E_CLD_SCENES_CMD_VIEW):
                    {

                        memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->u8SceneId,sizeof(uint8) );
                        u16Length += sizeof(uint8);
						pu8Buffer += sizeof(uint8);
                        if (E_ZCL_SUCCESS == pCustom->uMessage.psViewSceneResponsePayload->eStatus)
                        {
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->u16TransitionTime,sizeof(uint16) );
							u16Length += sizeof(uint16);
							pu8Buffer += sizeof(uint16);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sSceneName.u8Length,sizeof(uint8) );
							u16Length += sizeof(uint8);
							pu8Buffer += sizeof(uint8);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sSceneName.u8MaxLength,sizeof(uint8) );
							u16Length += sizeof(uint8);
							pu8Buffer += sizeof(uint8);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sSceneName.pu8Data,(sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sSceneName.u8Length) );
							u16Length += (sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sSceneName.u8Length);
							pu8Buffer += (sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sSceneName.u8Length);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.u16Length,sizeof(uint16) );
							u16Length += sizeof(uint16);
							pu8Buffer += sizeof(uint16);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.u8MaxLength,sizeof(uint8) );
							u16Length += sizeof(uint8);
							pu8Buffer += sizeof(uint8);
							memcpy(pu8Buffer,&pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.pu8Data,(sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.u16Length) );
							u16Length += (sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.u16Length);
							pu8Buffer += (sizeof(uint8)*pCustom->uMessage.psViewSceneResponsePayload->sExtensionField.u16Length);

                        }
                        u16Command = E_SL_MSG_VIEW_SCENE_RESPONSE;
                    }
                    break; // VIEW SCENE

                    case (E_CLD_SCENES_CMD_REMOVE):
                    {

                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveSceneResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveSceneResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveSceneResponsePayload->u8SceneId,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        u16Command = E_SL_MSG_REMOVE_SCENE_RESPONSE;
                    }
                    break; // REMOVE SCENE

                    case (E_CLD_SCENES_CMD_REMOVE_ALL):
                    {

                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveAllScenesResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psRemoveAllScenesResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        u16Command = E_SL_MSG_REMOVE_ALL_SCENES_RESPONSE;
                    }
                    break; // REMOVE ALL SCENES

                    case (E_CLD_SCENES_CMD_STORE):
                    {

                        memcpy(pu8Buffer,&pCustom->uMessage.psStoreSceneResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psStoreSceneResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer,&pCustom->uMessage.psStoreSceneResponsePayload->u8SceneId,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        u16Command = E_SL_MSG_STORE_SCENE_RESPONSE;
                    }
                    break; // STORE SCENE

                    case (E_CLD_SCENES_CMD_GET_SCENE_MEMBERSHIP):
                    {

                        memcpy(pu8Buffer,&pCustom->uMessage.psGetSceneMembershipResponsePayload->eStatus,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psGetSceneMembershipResponsePayload->u8Capacity,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer,&pCustom->uMessage.psGetSceneMembershipResponsePayload->u16GroupId,sizeof(uint16) );
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer,&pCustom->uMessage.psGetSceneMembershipResponsePayload->u8SceneCount,sizeof(uint8) );
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        int i;
                        for (i=0;i<pCustom->uMessage.psGetSceneMembershipResponsePayload->u8SceneCount;i++)
                        {
                            memcpy(pu8Buffer,&pCustom->uMessage.psGetSceneMembershipResponsePayload->pu8SceneList[i],(sizeof(uint8))  );
                            u16Length += (sizeof(uint8))  ;
                            pu8Buffer += (sizeof(uint8))  ;
                        }
                        u16Command = E_SL_MSG_SCENE_MEMBERSHIP_RESPONSE;
                    }
                    break; // SCENE MEMBERSHIP RESPONSE

                    default:
                    break;
                }
                vSL_WriteMessage(u16Command, u16Length, au8StatusBuffer,u8LinkQuality);
            }
            break;

            case SECURITY_AND_SAFETY_CLUSTER_ID_IASZONE:
            {
                tsCLD_IASZoneCallBackMessage *psCallBackMessage = (tsCLD_IASZoneCallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "- for IASZone cluster\r\n");
                vLog_Printf(TRACE_ZCL,LOG_DEBUG, "\r\nCMD: 0x%02x\r\n", psCallBackMessage->u8CommandId);
                switch (psCallBackMessage->u8CommandId)
                {
                    case E_CLD_IASZONE_CMD_ZONE_ENROLL_REQUEST:
                    {
                    }
                    break;

                    case E_CLD_IASZONE_CMD_ZONE_STATUS_CHANGE_NOTIFICATION:
                    {
                        memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode, sizeof(uint8));
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        if (psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcAddrMode == 0x03)
                        {
                             memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u64Addr, sizeof(uint64));
                             u16Length += sizeof(uint64);
                             pu8Buffer += sizeof(uint64);
                        }
                        else
                        {
                            memcpy(pu8Buffer, &psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr, sizeof(uint16));
                            u16Length += sizeof(uint16);
                            pu8Buffer += sizeof(uint16);
                        }

                        memcpy(pu8Buffer, &psCallBackMessage->uMessage.psZoneStatusNotificationPayload->b16ZoneStatus, sizeof(uint16));
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        memcpy(pu8Buffer, &psCallBackMessage->uMessage.psZoneStatusNotificationPayload->b8ExtendedStatus, sizeof(uint8));
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer, &psCallBackMessage->uMessage.psZoneStatusNotificationPayload->u8ZoneId, sizeof(uint8));
                        u16Length += sizeof(uint8);
                        pu8Buffer += sizeof(uint8);
                        memcpy(pu8Buffer, &psCallBackMessage->uMessage.psZoneStatusNotificationPayload->u16Delay, sizeof(uint16));
                        u16Length += sizeof(uint16);
                        pu8Buffer += sizeof(uint16);
                        vSL_WriteMessage(E_SL_MSG_IAS_ZONE_STATUS_CHANGE_NOTIFY, u16Length, au8StatusBuffer,u8LinkQuality);
                    }
                    break; //IAS ZONE CHANGE NOTIFY

                    default:
                    {
                    }
                    break;
                }
            }
            break;
            case ENERGY_AT_HOME_CLUSTER_ID_APPLIANCE_STATISTICS:
            {
            	tsCLD_ApplianceStatisticsCallBackMessage *pCustom = (tsCLD_ApplianceStatisticsCallBackMessage *)psEvent->uMessage.sClusterCustomMessage.pvCustomData;
                memcpy(pu8Buffer, &pCustom->u8CommandId,sizeof(uint8));
                u16Length += sizeof(uint8);
                pu8Buffer += sizeof(uint8);
            	memcpy(pu8Buffer, &pCustom->uMessage.psLogNotificationORLogResponsePayload->utctTime, sizeof(uint32));
                u16Length += sizeof(uint32);
                pu8Buffer += sizeof(uint32);
                memcpy(pu8Buffer, &pCustom->uMessage.psLogNotificationORLogResponsePayload->u32LogId, sizeof(uint32));
                u16Length += sizeof(uint32);
                pu8Buffer += sizeof(uint32);
                memcpy(pu8Buffer, &pCustom->uMessage.psLogNotificationORLogResponsePayload->u32LogLength, sizeof(uint32));
                u16Length += sizeof(uint32);
                pu8Buffer += sizeof(uint32);
                memcpy(pu8Buffer, pCustom->uMessage.psLogNotificationORLogResponsePayload->pu8LogData, pCustom->uMessage.psLogNotificationORLogResponsePayload->u32LogLength);
                u16Length += pCustom->uMessage.psLogNotificationORLogResponsePayload->u32LogLength;
                pu8Buffer += pCustom->uMessage.psLogNotificationORLogResponsePayload->u32LogLength;
                vSL_WriteMessage(E_SL_MSG_ASC_LOG_MSG_RESPONSE, u16Length, au8StatusBuffer,u8LinkQuality);
            }
            break;
#ifdef CLD_POLL_CONTROL
            case GENERAL_CLUSTER_ID_POLL_CONTROL:
            {
            	APP_vCMDHandlePollControlEndpointCallback(psEvent);
            }
            break;
#endif
            default:
                break;
        }// CUSTOM CLUSTER SWITCH STATEMENT
        }// CUSTOM CASE
        break;
        default:
            vLog_Printf(TRACE_ZCL,LOG_DEBUG, "EP EVT: Invalid event type\r\n");
        break;
    }//Switch of event

}
/****************************************************************************
 *
 * NAME: APP_ZCL_cbEndpointCallback
 *
 * DESCRIPTION:
 * Endpoint specific callback for ZCL events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbZllCommissionCallback(tsZCL_CallBackEvent *psEvent)
{
    APP_CommissionEvent sEvent;
    sEvent.eType = APP_E_COMMISSION_MSG;
    sEvent.u8Lqi = psEvent->pZPSevent->uEvent.sApsInterPanDataIndEvent.u8LinkQuality;
    sEvent.sZllMessage.eCommand = ((tsCLD_ZllCommissionCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.u8CommandId;
    sEvent.sZllMessage.sSrcAddr = ((tsCLD_ZllCommissionCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sRxInterPanAddr.sSrcAddr;
    memcpy(&sEvent.sZllMessage.uPayload,
            (((tsCLD_ZllCommissionCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.uMessage.psScanRspPayload),
            sizeof(tsZllPayloads));
    OS_ePostMessage(APP_CommissionEvents, &sEvent);
}

/****************************************************************************
 *
 * NAME: APP_ZCL_cbZllUtilityCallback
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_ZCL_cbZllUtilityCallback(tsZCL_CallBackEvent *psEvent)
{
    APP_tsEvent   sEvent;

    vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\nRx Util Cmd %02x",
            ((tsCLD_ZllUtilityCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.u8CommandId);

    switch (((tsCLD_ZllUtilityCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.u8CommandId ) {
        case E_CLD_UTILITY_CMD_ENDPOINT_INFO:
            sEvent.eType = APP_E_EVENT_EP_INFO_MSG;
            sEvent.uEvent.sEpInfoMsg.u16SrcAddr = psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr;
            memcpy(&sEvent.uEvent.sEpInfoMsg.sPayload,
                    ((tsCLD_ZllUtilityCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.uMessage.psEndpointInfoPayload,
                    sizeof(tsCLD_ZllUtility_EndpointInformationCommandPayload));
            OS_ePostMessage(APP_msgEvents, &sEvent);
            break;

        case E_CLD_UTILITY_CMD_GET_ENDPOINT_LIST_REQ_RSP:
            vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\ngot ep list");
            sEvent.eType = APP_E_EVENT_EP_LIST_MSG;
            sEvent.uEvent.sEpListMsg.u8SrcEp = psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint;
            sEvent.uEvent.sEpListMsg.u16SrcAddr = psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr;
            memcpy(&sEvent.uEvent.sEpListMsg.sPayload,
                    ((tsCLD_ZllUtilityCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.uMessage.psGetEndpointListRspPayload,
                    sizeof(tsCLD_ZllUtility_GetEndpointListRspCommandPayload));
            OS_ePostMessage(APP_msgEvents, &sEvent);
            break;
        case E_CLD_UTILITY_CMD_GET_GROUP_ID_REQ_RSP:
            vLog_Printf(TRACE_ZB_CONTROLBRIDGE_TASK,LOG_DEBUG, "\ngot group list");
            sEvent.eType = APP_E_EVENT_GROUP_LIST_MSG;
            sEvent.uEvent.sGroupListMsg.u8SrcEp = psEvent->pZPSevent->uEvent.sApsDataIndEvent.u8SrcEndpoint;
            sEvent.uEvent.sGroupListMsg.u16SrcAddr = psEvent->pZPSevent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr;
            memcpy(&sEvent.uEvent.sEpListMsg.sPayload,
                    ((tsCLD_ZllUtilityCustomDataStructure*)psEvent->psClusterInstance->pvEndPointCustomStructPtr)->sCallBackMessage.uMessage.psGetGroupIdRspPayload,
                    sizeof(tsCLD_ZllUtility_GetGroupIdRspCommandPayload));
            OS_ePostMessage(APP_msgEvents, &sEvent);
            break;
    }
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_Init
 *
 * DESCRIPTION:
 * ZLL Device Specific initialization
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
void vAPP_ZCL_DeviceSpecific_Init()
{
    /* Initialise the strings in Basic */
    memcpy(sControlBridge.sBasicServerCluster.au8ManufacturerName, "NXP", CLD_BAS_MANUF_NAME_SIZE);
    memcpy(sControlBridge.sBasicServerCluster.au8ModelIdentifier, "ZLL-ControlBridge", CLD_BAS_MODEL_ID_SIZE);
    memcpy(sControlBridge.sBasicServerCluster.au8DateCode, "20121212", CLD_BAS_DATE_SIZE);
    memcpy(sControlBridge.sBasicServerCluster.au8SWBuildID, "2000-0001", CLD_BAS_SW_BUILD_SIZE);
}

/****************************************************************************
 *
 * NAME: eApp_ZLL_RegisterEndpoint
 *
 * DESCRIPTION:
 * Register ZLL endpoints
 *
 * PARAMETER
 * Type                                Name                    Descirption
 * tfpZCL_ZCLCallBackFunction          fptr                    Pointer to ZCL Callback function
 * tsZLL_CommissionEndpoint            psCommissionEndpoint    Pointer to Commission Endpoint
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
teZCL_Status eApp_ZLL_RegisterEndpoint(tfpZCL_ZCLCallBackFunction fptr)
{
	eZLL_RegisterCommissionEndPoint(ZIGBEENODECONTROLBRIDGE_ZLL_COMMISSION_ENDPOINT,
                                    fptr,
                                    &sCommissionEndpoint);
	eZLL_RegisterZncControlBridgeEndPoint(ZIGBEENODECONTROLBRIDGE_ORVIBO_ENDPOINT,
	                                              fptr,
	                                              &sControlBridge);
    return eZLL_RegisterZncControlBridgeEndPoint(ZIGBEENODECONTROLBRIDGE_HA_ENDPOINT,
                                              fptr,
                                              &sControlBridge);
}


/****************************************************************************
 *
 * NAME: u16GetAttributeActualSize
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/

PUBLIC uint16 u16GetAttributeActualSize(uint32 u32Type,uint32 u32NumberOfItems)
{
    uint32 u16Size = 0;
    switch(u32Type)
    {
    case(E_ZCL_GINT8):
    case(E_ZCL_UINT8):
    case(E_ZCL_INT8):
    case(E_ZCL_ENUM8):
    case(E_ZCL_BMAP8):
    case(E_ZCL_BOOL):
    case(E_ZCL_OSTRING):
    case(E_ZCL_CSTRING):
        u16Size = sizeof(uint8);
        break;
    case(E_ZCL_LOSTRING):
    case(E_ZCL_LCSTRING):
    case(E_ZCL_STRUCT):
    case (E_ZCL_INT16):
    case (E_ZCL_UINT16):
    case (E_ZCL_ENUM16):
    case (E_ZCL_CLUSTER_ID):
    case (E_ZCL_ATTRIBUTE_ID):
        u16Size = sizeof(uint16);
        break;


    case E_ZCL_UINT24:
    case E_ZCL_UINT32:
    case E_ZCL_TOD:
    case E_ZCL_DATE:
    case E_ZCL_UTCT:
    case E_ZCL_BACNET_OID:
    case E_ZCL_INT24:
    case E_ZCL_FLOAT_SINGLE:
        u16Size = sizeof(uint32);
        break;

    case E_ZCL_UINT40:
    case E_ZCL_UINT48:
    case E_ZCL_UINT56:
    case E_ZCL_UINT64:
    case E_ZCL_IEEE_ADDR:
        u16Size = sizeof(uint64);
        break;
    default:
        u16Size = 0;
        break;
    }
    return (u16Size*u32NumberOfItems);
}


/****************************************************************************
 *
 * NAME: eZLL_RegisterZncControlBridgeEndPoint
 *
 * DESCRIPTION:
 * Registers a control bridge device with the ZCL layer
 *
 * PARAMETERS:  Name                            Usage
 *              u8EndPointIdentifier            Endpoint being registered
 *              cbCallBack                      Pointer to endpoint callback
 *              psDeviceInfo                    Pointer to struct containing
 *                                              data for endpoint
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/

PUBLIC teZCL_Status eZLL_RegisterZncControlBridgeEndPoint(uint8 u8EndPointIdentifier,
                                              tfpZCL_ZCLCallBackFunction cbCallBack,
                                              tsZLL_ZncControlBridgeDevice *psDeviceInfo)
{
    /* Fill in end point details */
    psDeviceInfo->sEndPoint.u8EndPointNumber = u8EndPointIdentifier;
    psDeviceInfo->sEndPoint.u16ManufacturerCode = ZLL_MANUFACTURER_CODE;
    psDeviceInfo->sEndPoint.u16ProfileEnum = HA_PROFILE_ID;
    psDeviceInfo->sEndPoint.bIsManufacturerSpecificProfile = FALSE;
    psDeviceInfo->sEndPoint.u16NumberOfClusters = sizeof(tsZLL_ZncControlBridgeDeviceClusterInstances) / sizeof(tsZCL_ClusterInstance);
    psDeviceInfo->sEndPoint.psClusterInstance = (tsZCL_ClusterInstance*)&psDeviceInfo->sClusterInstance;
    psDeviceInfo->sEndPoint.bDisableDefaultResponse = ZLL_DISABLE_DEFAULT_RESPONSES;
    psDeviceInfo->sEndPoint.pCallBackFunctions = cbCallBack;


    #if (defined CLD_BASIC) && (defined BASIC_CLIENT)
        /* Create an instance of a Basic cluster as a client */
        eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicClient,
                          FALSE,
                          &sCLD_Basic,
                          NULL, //&psDeviceInfo->sBasicClientCluster,
                          NULL);
    #endif

    #if (defined CLD_BASIC) && (defined BASIC_SERVER)
        /* Create an instance of a Basic cluster as a server */
        eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicServer,
                              TRUE,
                              &sCLD_Basic,
                              &psDeviceInfo->sBasicServerCluster,
                              &au8BasicClusterAttributeControlBits[0]);
    #endif

    #if (defined CLD_ZLL_UTILITY) && (defined ZLL_UTILITY_SERVER)
        /* Create an instance of a basic cluster as a server */
        eCLD_ZllUtilityCreateUtility(&psDeviceInfo->sClusterInstance.sZllUtilityServer,
                              TRUE,
                              &sCLD_ZllUtility,
                              NULL/*&psDeviceInfo->sZllUtilityCluster*/,
                              NULL/*(uint8*)&psDeviceInfo->sZllUtilityClusterAttributeStatus*/,
                              &psDeviceInfo->sZllUtilityServerCustomDataStructure);
    #endif

    #if (defined CLD_ZLL_UTILITY) && (defined ZLL_UTILITY_CLIENT)
        /* Create an instance of a basic cluster as a server */
        eCLD_ZllUtilityCreateUtility(&psDeviceInfo->sClusterInstance.sZllUtilityClient,
                              FALSE,
                              &sCLD_ZllUtility,
                              NULL/*&psDeviceInfo->sZllUtilityCluster*/,
                              NULL/*(uint8*)&psDeviceInfo->sZllUtilityClusterAttributeStatus*/,
                              &psDeviceInfo->sZllUtilityClientCustomDataStructure);
    #endif

    /*
     * Mandatory client clusters
     */
    #if (defined CLD_ONOFF) && (defined ONOFF_CLIENT)
        /* Create an instance of an On/Off cluster as a client */
        eCLD_OnOffCreateOnOff(&psDeviceInfo->sClusterInstance.sOnOffClient,
                              FALSE,
                              &sCLD_OnOff,
                              NULL, //&psDeviceInfo->sOnOffClientCluster,
                              NULL,
                              NULL  /* no cust data struct for client */);
    #endif

    #if (defined CLD_ONOFF) && (defined ONOFF_SERVER)
        /* Create an instance of an On/Off cluster as a server */
        eCLD_OnOffCreateOnOff(&psDeviceInfo->sClusterInstance.sOnOffServer,
                              TRUE,
                              &sCLD_OnOff,
                              &psDeviceInfo->sOnOffServerCluster,
                              &au8OnOffServerAttributeControlBits[0],
                              &psDeviceInfo->sOnOffServerCustomDataStructure);
    #endif

    #if (defined CLD_LEVEL_CONTROL) && (defined LEVEL_CONTROL_CLIENT)
        /* Create an instance of a Level Control cluster as a client */
        eCLD_LevelControlCreateLevelControl(&psDeviceInfo->sClusterInstance.sLevelControlClient,
                              FALSE,
                              &sCLD_LevelControl,
                              NULL, //&psDeviceInfo->sLevelControlClientCluster,
                              NULL,
                              &psDeviceInfo->sLevelControlClientCustomDataStructure);
    #endif

    #if (defined CLD_COLOUR_CONTROL) && (defined COLOUR_CONTROL_CLIENT)
        /* Create an instance of a Colour Control cluster as a client */
        eCLD_ColourControlCreateColourControl(
                              &psDeviceInfo->sClusterInstance.sColourControlClient,
                              FALSE,
                              &sCLD_ColourControl,
                              NULL, //&psDeviceInfo->sColourControlClientCluster,
                              NULL,
                              &psDeviceInfo->sColourControlClientCustomDataStructure);
    #endif

    #if (defined CLD_SCENES) && (defined SCENES_CLIENT)
        /* Create an instance of a Scenes cluster as a client */
        eCLD_ScenesCreateScenes(&psDeviceInfo->sClusterInstance.sScenesClient,
                              FALSE,
                              &sCLD_Scenes,
                              NULL, //&psDeviceInfo->sScenesClientCluster,
                              NULL,
                              &psDeviceInfo->sScenesClientCustomDataStructure,
                              &psDeviceInfo->sEndPoint);
    #endif

    #if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
        /* Create an instance of a Groups cluster as a client */
        eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsClient,
                              FALSE,
                              &sCLD_Groups,
                              NULL, //&psDeviceInfo->sGroupsClientCluster,
                              NULL,
                              &psDeviceInfo->sGroupsClientCustomDataStructure,
                              &psDeviceInfo->sEndPoint);
    #endif

    #if (defined CLD_GROUPS) && (defined GROUPS_SERVER)
        /* Create an instance of a Groups cluster as a server */
        eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsServer,
                                TRUE,
                                &sCLD_Groups,
                                &psDeviceInfo->sGroupsServerCluster,
                                NULL,
                                &psDeviceInfo->sGroupsServerCustomDataStructure,
                                &psDeviceInfo->sEndPoint);
    #endif

    #if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
        /* Create an instance of an Identify cluster as a client */
        eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyClient,
                              FALSE,
                              &sCLD_Identify,
                              NULL, //&psDeviceInfo->sIdentifyClientCluster,
                              NULL,
                              &psDeviceInfo->sIdentifyClientCustomDataStructure);
    #endif

    #if (defined CLD_IASZONE) && (defined IASZONE_CLIENT)
        /* Create an instance of a IAS Zone cluster as a client */
        eCLD_IASZoneCreateIASZone(&psDeviceInfo->sClusterInstance.sIASZoneClient,
                                  FALSE,
                                  &sCLD_IASZone,
                                  NULL, //&psDeviceInfo->sIASZoneClientCluster,
                                  NULL,
                                  &psDeviceInfo->sIASZoneClientCustomDataStructure);
    #endif

    #if (defined CLD_DOOR_LOCK) && (defined DOOR_LOCK_CLIENT)
        /* Create an instance of a Door Lock cluster as a client */
        eCLD_DoorLockCreateDoorLock(&psDeviceInfo->sClusterInstance.sDoorLockClient,
                              FALSE,
                              &sCLD_DoorLock,
                              NULL, //&psDeviceInfo->sDoorLockClientCluster,
                              NULL);
    #endif
    #if (defined CLD_SIMPLE_METERING) && (defined SM_CLIENT)
        /* Create an instance of a Simple Metering cluster as a client */
        eSE_SMCreate(u8EndPointIdentifier,									// uint8 u8Endpoint
                     FALSE,													// bool_t bIsServer
                     NULL,		// uint8 *pu8AttributeControlBits
                     &psDeviceInfo->sClusterInstance.sMeteringClient,		// tsZCL_ClusterInstance *psClusterInstance
                     &sCLD_SimpleMetering,									// tsZCL_ClusterDefinition *psClusterDefinition
                     &psDeviceInfo->sMeteringClientCustomDataStructure,		// tsSM_CustomStruct *psCustomDataStruct
                     NULL); //&psDeviceInfo->sMeteringClientCluster);				// void *pvEndPointSharedStructPtr

        /* By default the SM cluster uses APS layer security, we don't need this for HA applications */
        psDeviceInfo->sClusterInstance.sMeteringClient.psClusterDefinition->u8ClusterControlFlags = E_ZCL_SECURITY_NETWORK;
    #endif

	#if (defined CLD_OTA) && (defined OTA_SERVER)
        /* Create an instance of an OTA cluster as a server */
        eOTA_Create(&psDeviceInfo->sClusterInstance.sOTAServerClusterInst,	// tsZCL_ClusterInstance *psClusterInstance
	  		    	TRUE,													// bool_t bIsServer
	  		    	&sCLD_OTA,												// tsZCL_ClusterDefinition *psClusterDefinition
	  		    	&psDeviceInfo->sOTAServerCluster,						// void *pvEndPointSharedStructPtr
	  		    	u8EndPointIdentifier,									// uint8 u8Endpoint
	  		    	&attribData[0],											// uint8 *pu8AttributeControlBits
	  		    	&psDeviceInfo->sOTAServerCustomDataStruct);				// tsOTA_Common *psCustomDataStruct
	#endif

#if (defined CLD_TEMPERATURE_MEASUREMENT) && (defined TEMPERATURE_MEASUREMENT_CLIENT)
    /* Create an instance of a Temperature Measurement cluster as a client */
    eCLD_TemperatureMeasurementCreateTemperatureMeasurement(
                          &psDeviceInfo->sClusterInstance.sTemperatureMeasurementClient,
                          FALSE,
                          &sCLD_TemperatureMeasurement,
                          NULL, //&psDeviceInfo->sTemperatureMeasurementClientCluster,
                          NULL);
#endif

#if (defined CLD_RELATIVE_HUMIDITY_MEASUREMENT) && (defined RELATIVE_HUMIDITY_MEASUREMENT_CLIENT)
    /* Create an instance of a Relative Humidity Measurement cluster as a client */
    eCLD_RelativeHumidityMeasurementCreateRelativeHumidityMeasurement(
                          &psDeviceInfo->sClusterInstance.sRelativeHumidityMeasurementClient,
                          FALSE,
                          &sCLD_RelativeHumidityMeasurement,
                          NULL, //&psDeviceInfo->sRelativeHumidityMeasurementClientCluster,
                          NULL);
#endif
#if (defined CLD_ILLUMINANCE_MEASUREMENT) && (defined ILLUMINANCE_MEASUREMENT_CLIENT)
    /* Create an instance of a Illuminance Measurement cluster as a client */
    eCLD_IlluminanceMeasurementCreateIlluminanceMeasurement(
                          &psDeviceInfo->sClusterInstance.sIlluminanceMeasurementClient,
                          FALSE,
                          &sCLD_IlluminanceMeasurement,
                          NULL, //&psDeviceInfo->sIlluminanceMeasurementClientCluster,
                          NULL);
#endif

#if (defined CLD_THERMOSTAT) && (defined THERMOSTAT_CLIENT)
        /* Create an instance of a Thermostat cluster as a client */
        eCLD_ThermostatCreateThermostat(&psDeviceInfo->sClusterInstance.sThermostatClient,
                              FALSE,
                              &sCLD_Thermostat,
                              NULL, //&psDeviceInfo->sThermostatClientCluster,
                              NULL,
                              &psDeviceInfo->sThermostatClientCustomDataStructure);
#endif
#if (defined CLD_APPLIANCE_STATISTICS) && (defined APPLIANCE_STATISTICS_CLIENT)
       /* Create an instance of a appliance statistics cluster as a server */
       eCLD_ApplianceStatisticsCreateApplianceStatistics(&psDeviceInfo->sClusterInstance.sASCClient,
                          FALSE,
                          &sCLD_ApplianceStatistics,
                          NULL,
                          NULL,
                          &psDeviceInfo->sASCClientCustomDataStructure);
#endif

#if (defined CLD_DIAGNOSTICS) && (defined DIAGNOSTICS_CLIENT)
	/* Create an instance of a Diagnostic cluster as a client */
	eCLD_DiagnosticsCreateDiagnostics(&psDeviceInfo->sClusterInstance.sDiagnosticClient,
                        		  	  FALSE,
                        		  	  &sCLD_Diagnostics,
                        		  	  NULL, //&psDeviceInfo->sDiagnosticClientCluster,
                        		  	  NULL);
#endif		

#if (defined CLD_APPLIANCE_STATISTICS) && (defined APPLIANCE_STATISTICS_SERVER)
    /* Create an instance of a appliance statistics cluster as a server */
    eCLD_ApplianceStatisticsCreateApplianceStatistics(&psDeviceInfo->sClusterInstance.sASCServer,
                          TRUE,
                          &sCLD_ApplianceStatistics,
                          &psDeviceInfo->sASCServerCluster,
                          &au8ApplianceStatisticsServerAttributeControlBits[0],
                          &psDeviceInfo->sASCServerCustomDataStructure);
#endif

#if (defined CLD_POLL_CONTROL) && (defined POLL_CONTROL_CLIENT)
    /* Create an instance of a Poll Control cluster as a client */
    eCLD_PollControlCreatePollControl(
                          &psDeviceInfo->sClusterInstance.sPollControlClient,
                          FALSE,
                          &sCLD_PollControl,
                          NULL,
                          NULL,
                          &psDeviceInfo->sPollControlClientCustomDataStructure);
#endif

    return eZCL_Register(&psDeviceInfo->sEndPoint);

}

/****************************************************************************
 *
 * NAME: u16ZncWriteDataPattern
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC uint16 u16ZncWriteDataPattern(uint8 *pu8Data, teZCL_ZCLAttributeType eAttributeDataType, uint8 *pu8Struct, uint32 u32Size)
{
    switch(eAttributeDataType)
    {
        /* 8 bit integer value */
        case(E_ZCL_GINT8):
        case(E_ZCL_UINT8):
        case(E_ZCL_INT8):
        case(E_ZCL_ENUM8):
        case(E_ZCL_BMAP8):
        case(E_ZCL_BOOL):
        {
            *(uint8 *)pu8Data = *pu8Struct++;
            break;
        }

        /* 16 bit integer value */
        case(E_ZCL_GINT16):
        case(E_ZCL_UINT16):
        case(E_ZCL_ENUM16):
        case(E_ZCL_INT16):
        case(E_ZCL_CLUSTER_ID):
        case(E_ZCL_ATTRIBUTE_ID):
        case(E_ZCL_BMAP16):
        case(E_ZCL_FLOAT_SEMI):
        {
            uint16 u16Val = *pu8Struct++;
            u16Val |= (*pu8Struct++) << 8;
            // Unsafe due to alignment issues - replaced by memcpy
            //*(uint16 *)(pu8Struct) = u16Val;
            memcpy(pu8Data, &u16Val, sizeof(uint16));
            break;
        }
        /* 32 bit integer value */
        case(E_ZCL_UINT32):
        case(E_ZCL_INT32):
        case(E_ZCL_GINT32):
        case(E_ZCL_BMAP32):
        case(E_ZCL_UTCT):
        case(E_ZCL_TOD):
        case(E_ZCL_DATE):
        case(E_ZCL_FLOAT_SINGLE):
        {
            uint32 u32Val = *pu8Struct++;
            u32Val |= (*pu8Struct++) << 8;
            u32Val |= (*pu8Struct++) << 16;
            u32Val |= (*pu8Struct++) << 24;
            memcpy(pu8Data, &u32Val, sizeof(uint32));
            break;
        }
        /* 64 bit integer value */
        case(E_ZCL_GINT64):
        case(E_ZCL_UINT64):
        case(E_ZCL_INT64):
        case(E_ZCL_BMAP64):
        case(E_ZCL_IEEE_ADDR):
        case(E_ZCL_FLOAT_DOUBLE):
        {
            uint64 u64Val = (uint32)*pu8Struct++;
            u64Val |= (uint64)(*pu8Struct++) << 8;
            u64Val |= (uint64)(*pu8Struct++) << 16;
            u64Val |= (uint64)(*pu8Struct++) << 24;
            u64Val |= (uint64)(*pu8Struct++) << 32;
            u64Val |= (uint64)(*pu8Struct++) << 40;
            u64Val |= (uint64)(*pu8Struct++) << 48;
            u64Val |= (uint64)(*pu8Struct++) << 56;
            memcpy(pu8Data, &u64Val, sizeof(uint64));
            break;
        }

        /* 24-bit stored as 32 bit integer value */
        case(E_ZCL_GINT24):
        case(E_ZCL_UINT24):
        case(E_ZCL_INT24):
        case(E_ZCL_BMAP24):
        {
            uint32 u32Val = *pu8Struct++;
            u32Val |= (*pu8Struct++) << 8;
            u32Val |= (*pu8Struct)   << 16;
            // account for signed-ness
            if(eAttributeDataType == E_ZCL_INT24)
            {
                // sign extend if top bit set
                if(*pu8Struct &0x80)
                {
                    u32Val |= (0xff << 24);
                }
            }
            memcpy(pu8Data, &u32Val, sizeof(uint32));
            // increment ptr to keep size calculation correct
            pu8Struct++;
            break;
        }

        /* 40-bit stored as 64 bit integer value */
        case(E_ZCL_GINT40):
        case(E_ZCL_UINT40):
        case(E_ZCL_INT40):
        case(E_ZCL_BMAP40):
        {
            uint64 u64Val = (uint32)*pu8Struct++;
            u64Val |= (uint64)(*pu8Struct++) << 8;
            u64Val |= (uint64)(*pu8Struct++) << 16;
            u64Val |= (uint64)(*pu8Struct++) << 24;
            u64Val |= (uint64)(*pu8Struct) << 32;
            // account for signed-ness
            if(eAttributeDataType == E_ZCL_INT40)
            {
                // sign extend if top bit set
                if(*pu8Struct &0x80)
                {
                    u64Val |= ((uint64)(0xffffff) << 40);
                }
            }
            memcpy(pu8Data, &u64Val, sizeof(uint64));
            // increment ptr to keep size calculation correct
            pu8Struct++;
            break;
        }

        /* 48-bit stored as 64 bit integer value */
        case(E_ZCL_GINT48):
        case(E_ZCL_UINT48):
        case(E_ZCL_INT48):
        case(E_ZCL_BMAP48):
        {
            uint64 u64Val = (uint32)*pu8Struct++;
            u64Val |= (uint64)(*pu8Struct++) << 8;
            u64Val |= (uint64)(*pu8Struct++) << 16;
            u64Val |= (uint64)(*pu8Struct++) << 24;
            u64Val |= (uint64)(*pu8Struct++) << 32;
            u64Val |= (uint64)(*pu8Struct) << 40;
            // account for signed-ness
            if(eAttributeDataType == E_ZCL_INT48)
            {
                // sign extend if top bit set
                if(*pu8Struct &0x80)
                {
                    u64Val |= ((uint64)(0xffff) << 48);
                }
            }
            memcpy(pu8Data, &u64Val, sizeof(uint64));
            // increment ptr to keep size calculation correct
            pu8Struct++;
            break;
        }

        /* 56-bit stored as 64 bit integer value */
        case(E_ZCL_GINT56):
        case(E_ZCL_UINT56):
        case(E_ZCL_INT56):
        case(E_ZCL_BMAP56):
        {
            uint64 u64Val = (uint32)*pu8Struct++;
            u64Val |= (uint64)(*pu8Struct++) << 8;
            u64Val |= (uint64)(*pu8Struct++) << 16;
            u64Val |= (uint64)(*pu8Struct++) << 24;
            u64Val |= (uint64)(*pu8Struct++) << 32;
            u64Val |= (uint64)(*pu8Struct++) << 40;
            u64Val |= (uint64)(*pu8Struct) << 48;
            // account for signed-ness
            if(eAttributeDataType == E_ZCL_INT56)
            {
                // sign extend if top bit set
                if(*pu8Data &0x80)
                {
                    u64Val |= ((uint64)(0xff) << 56);
                }
            }
            memcpy(pu8Data, &u64Val, sizeof(uint64));
            // increment ptr to keep size calculation correct
            pu8Struct++;
            break;
        }
        case(E_ZCL_KEY_128):
            memcpy(pu8Data, pu8Struct, E_ZCL_KEY_128_SIZE);
            // increment ptr to keep size calculation correct
            pu8Struct += E_ZCL_KEY_128_SIZE;
            break;
        case (E_ZCL_NULL):
        // unrecognised
        default:
        {
            return 0;
        }
    }

    return (uint16)( u32Size);
}

/****************************************************************************
 *
 * NAME: APP_u32CheckIfAttributeIsString
 *
 * DESCRIPTION: Works out if the attribute is a string and returns the number
 * of characters there are.
 *	
 *
 * RETURNS:
 * uint32
 *
 ****************************************************************************/
PRIVATE uint32 APP_u32CheckIfAttributeIsString(tsZCL_IndividualAttributesResponse *sIndividualAttributeResponse)
{
	uint8     u8Elements =  0;
	switch ( sIndividualAttributeResponse->eAttributeDataType )
	{
	case(E_ZCL_OSTRING):
	case(E_ZCL_CSTRING):
	case(E_ZCL_LOSTRING):
	case(E_ZCL_LCSTRING):
	   if ( sIndividualAttributeResponse->pvAttributeData != NULL )
	   {
		   u8Elements =  ( (uint8*)sIndividualAttributeResponse->pvAttributeData ) [ 0 ];
	   }
	   else
	   {
		   u8Elements   =  0 ;
	   }
	break;

	default:
		u8Elements   =  1;
		break;
	}

	return u8Elements;
}

/****************************************************************************
 *
 * NAME: APP_vCopyAttributeDataIntoBuffer
 *
 * DESCRIPTION: Depending on if it is a string on not, it copies the data
 * via the correct mechanism. For strings, there needs to be a level of casting
 *	
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_vCopyAttributeDataIntoBuffer(uint16 u16SizeOfAttribute,
											  teZCL_ZCLAttributeType eAttributeDataType,
											  void * pvAttributeData,
											  uint8 **ppu8Buffer)
{
	switch ( eAttributeDataType )
	{
		case(E_ZCL_OSTRING):
		case(E_ZCL_CSTRING):
		case(E_ZCL_LOSTRING):
		case(E_ZCL_LCSTRING):
		{
			tsZCL_CharacterString *sZCLCharString;

			sZCLCharString = (tsZCL_CharacterString *)pvAttributeData;

			memcpy(*ppu8Buffer,sZCLCharString->pu8Data,u16SizeOfAttribute);
			break;
		}
		default:
			memcpy(*ppu8Buffer,pvAttributeData,u16SizeOfAttribute);
			break;
	}
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
