/*****************************************************************************
 *
 * MODULE: ZigbeeNodeControlBridge
 *
 * COMPONENT: app_Znc_cmds.c
 *
 * $AUTHOR:Faisal Bhaiyat$
 *
 * DESCRIPTION:
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Zigbee%20Protocol%20Stack/ZPS/Trunk/ZigbeeNodeControlBridge/Source/ZigbeeNodeControlBridge/app_Znc_cmds.c $
 *
 * $Revision: 55059 $
 *
 * $LastChangedBy: nxp29741 $
 *
 * $LastChangedDate: 2013-07-01 17:27:09 +0100 (Mon, 01 Jul 2013) $
 *
 * $Id: app_Znc_cmds.c 55059 2013-07-01 16:27:09Z nxp29741 $
 *
 *****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2008. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <string.h>
#include "jendefs.h"
#include "zps_gen.h"
#include "dbg.h"
#include "os.h"
#include "os_gen.h"
#include "pdm.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "appapi.h"
#include "AppHardwareApi.h"
#include "uart.h"
#include "rnd_pub.h"
#include "SerialLink.h"
#include "app_ZncParser_task.h"
#include "app_timer_driver.h"
#include "zps_apl.h"
#include "zps_apl_zdo.h"
#include "zps_apl_aib.h"
#include "zps_apl_af.h"
#include "app_common.h"
#include "app_Znc_cmds.h"
#include "Log.h"
#include "app_events.h"
#include "zcl_options.h"
#include "PDM_IDs.h"
#include "app_scenes.h"
#include "ApplianceStatistics.h"
#include "appZpsBeaconHandler.h"

#include "zcl_common.h"
#include "zcl.h"

#ifdef CLD_POLL_CONTROL
#include "app_poll_control_commands_handler.h"
#endif
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifndef TRACE_APP
#define TRACE_APP   TRUE
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE ZPS_teStatus eZdpSystemServerDiscovery(uint16 u16ServerMask,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpMgmtNetworkUpdateReq(uint16 u16Addr, uint32 u32ChannelMask, uint8 u8ScanDuration, uint8 u8ScanCount,uint8* pu8Seq, uint16 u16NwkManagerAddr);
PRIVATE ZPS_teStatus eZdpMgmtLeave(uint16 u16DstAddr, uint64 u64DeviceAddr, bool_t bRejoin, bool_t bRemoveChildren,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpLeaveReq(ZPS_tuAddress uAddress, bool u8RemoveChildren, bool Rejoin);
PRIVATE ZPS_teStatus eZdpRemoveDeviceReq(ZPS_tuAddress uParentAddress, ZPS_tuAddress uChildAddress);
PRIVATE ZPS_teStatus eZdpNodeDescReq(uint16 u16Addr,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpComplexDescReq(uint16 u16Addr,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpUserDescReq(uint16 u16Addr,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpUserSetDescReq(uint16 u16Addr, uint8 u8Length, uint8 * pu8UserDescriptor, uint8* pu8SeqNum);
PRIVATE ZPS_teStatus eZdpPowerDescReq(uint16 u16Addr,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpSimpleDescReq(uint16 u16Addr, uint8 u8Endpoint,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpActiveEndpointReq(uint16 u16Addr,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpMatchDescReq(uint16 u16Addr,uint16 u16profile,uint8 u8InputCount, uint16* pu8InputList, uint8 u8OutputCount, uint16* pu8OutputList,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpIeeeAddrReq(uint16 u16Dst,
        uint16 u16Addr,
        uint8 u8RequestType,
        uint8 u8StartIndex,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpNwkAddrReq(uint16 u16Dst,
        uint64 u64Addr,
        uint8 u8RequestType,
        uint8 u8StartIndex,uint8* pu8Seq);
PRIVATE ZPS_teStatus eZdpPermitJoiningReq(uint16 u16DstAddr,
        uint8 u8PermitDuration,
        bool bTcSignificance,uint8* pu8Seq);
PRIVATE ZPS_teStatus eBindUnbindEntry(
        bool_t       bBind,
        uint64       u64SrcAddr,
        uint8        u8SrcEndpoint,
        uint16       u16ClusterId,
        ZPS_tuAddress *puDstAddress,
        uint8        u8DstEndpoint,
        uint8        u8DstAddrMode,
        uint8* pu8Seq);
PRIVATE void vControlNodeScanStart(void);
#ifdef APP_DISCOVERY_ONLY_MODE_ENABLED
PRIVATE void vScanDiscoveryOnly(void);
PUBLIC void vScanDiscoveryOnlyResponse(MAC_MlmeDcfmInd_s* psBeaconIndication);
#endif
PRIVATE void vControlNodeStartNetwork(void);
PUBLIC  teZCL_Status  eZnc_SendWriteAttributesRequest(
                    uint8                       u8SourceEndPointId,
                    uint8                       u8DestinationEndPointId,
                    uint16                      u16ClusterId,
                    bool_t                      bDirectionIsServerToClient,
                    tsZCL_Address              *psDestinationAddress,
                    uint8                      *pu8TransactionSequenceNumber,
                    bool_t                      bIsManufacturerSpecific,
                    uint16                      u16ManufacturerCode,
                    uint8                       *pu8AttributeRequestList,
                    uint8                       u8NumberOfAttrib,
                    uint16						u16SizePayload);
PRIVATE ZPS_teStatus eZdpMgmtLqiRequest(uint16 u16Addr, uint8  u8StartIndex,uint8* pu8Seq);
PRIVATE teZCL_Status eZclDiscoverAttributes(bool bIsExtReq,
											uint8 u8SourceEndPointId,
											uint8 u8DestinationEndPointId,
											uint16 u16ClusterId,
											bool_t bDirectionIsServerToClient,
											tsZCL_Address *psDestinationAddress,
											uint8 *pu8TransactionSequenceNumber,
											uint16 u16AttributeId,
											bool_t bIsManufacturerSpecific,
											uint16 u16ManufacturerCode,
											uint8 u8MaximumNumberOfIdentifiers);
PRIVATE teZCL_Status eZclDiscoverCommandReceived(uint8 u8SourceEndPointId,
												 uint8 u8DestinationEndPointId,
												 uint16 u16ClusterId,
												 bool_t bDirectionIsServerToClient,
												 tsZCL_Address *psDestinationAddress,
												 uint8 *pu8TransactionSequenceNumber,
												 uint8 u8CommandId,
												 bool_t bIsManufacturerSpecific,
												 uint16 u16ManufacturerCode,
												 uint8 u8MaximumNumberOfCommands);
PRIVATE teZCL_Status eZclDiscoverCommandGenerated(uint8 u8SourceEndPointId,
												  uint8 u8DestinationEndPointId,
												  uint16 u16ClusterId,
												  bool_t bDirectionIsServerToClient,
												  tsZCL_Address *psDestinationAddress,
												  uint8 *pu8TransactionSequenceNumber,
												  uint8 u8CommandId,
												  bool_t bIsManufacturerSpecific,
												  uint16 u16ManufacturerCode,
												  uint8 u8MaximumNumberOfCommands);
PRIVATE teZCL_Status eZclBasicResetToFactoryDefaults(uint8 u8SourceEndPointId,
        											 uint8 u8DestinationEndPointId,
        											 tsZCL_Address *psDestinationAddress,
        											 uint8 *pu8TransactionSequenceNumber);
PRIVATE uint8 eZCL_GetLastSequenceNumber(void);


/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
uint8 au8LinkRxBuffer[MAX_PACKET_SIZE];
uint16 u16PacketType,u16PacketLength;
extern int16 s_i16RxByte;
bool_t bResetIssued = FALSE;
uint32 u32ChannelMask = 0;
uint32 u32OldFrameCtr;
extern bool_t bSendFactoryResetOverAir;
extern tsLedState s_sLedState;
extern tsCommissionData sCommission;

extern tsZCL_Common *psZCL_Common;

extern void ZPS_vTCSetCallback(void*);
PRIVATE uint64 u64CallbackMacAddress = 0;

PUBLIC uint32 u32Channel;

bool_t bProcessMessages = TRUE, bBlackListEnable = FALSE;
/****************************************************************************/
/***        Exported Public Functions                                     ***/
/****************************************************************************/


/****************************************************************************/
/***        Exported Private Functions                                      */
/****************************************************************************/

PUBLIC void vProcessIncomingSerialCommands(void)
{
    uint8 u8SeqNum = 0;
    uint8 u8Status = 0;
    uint16 u16TargetAddress;
    tsZCL_Address sAddress;
    uint8 au8values[4];

    if(TRUE==bSL_ReadMessage(&u16PacketType,&u16PacketLength,MAX_PACKET_SIZE,au8LinkRxBuffer,s_i16RxByte))
    {
    	if (u16PacketType >= E_SL_MSG_POLL_CONTROL_START && u16PacketType <= E_SL_MSG_POLL_CONTROL_END)
		{
			#ifdef POLL_CONTROL_CLIENT
				APP_vCMDHandlePollControlCommand(u16PacketType, u16PacketLength, au8LinkRxBuffer, &u8Status);
			#endif
		}
		else
		{
			memcpy(&u16TargetAddress,&au8LinkRxBuffer[1],sizeof(uint16));
			sAddress.eAddressMode = au8LinkRxBuffer[0];
			sAddress.uAddress.u16DestinationAddress = u16TargetAddress;
			vLog_Printf(TRACE_APP,LOG_DEBUG, "\nPacket Type %x \n",u16PacketType );
			if(sCommission.eState == E_IDLE ||
					(sCommission.eState == E_ACTIVE &&
							sZllState.eState == NOT_FACTORY_NEW))
			{
				bProcessMessages = TRUE;
			}
			if(!bProcessMessages)
			{
				u8Status = E_SL_MSG_STATUS_BUSY;
				memcpy(au8values,&u8Status,sizeof(uint8));
				memcpy(&au8values[1],&u8SeqNum,sizeof(uint8));
				memcpy(&au8values[2],&u16PacketType,sizeof(uint16));
				vSL_WriteMessage(E_SL_MSG_STATUS, 4, au8values,0);
				return;
			}
			switch(u16PacketType)
			{
				case (E_SL_MSG_GET_VERSION):
				{
					// Version is <Installer version : 16bits><Node version : 16bits>
				   uint32 u32Version = VERSION;
				   memcpy(au8values,&u8Status,sizeof(uint8));
				   memcpy(&au8values[1],&u8SeqNum,sizeof(uint8));
				   memcpy(&au8values[2],&u16PacketType,sizeof(uint16));
				   vSL_WriteMessage(E_SL_MSG_STATUS, 4, au8values,0);
				   vSL_WriteMessage(E_SL_MSG_VERSION_LIST, sizeof(uint32), (uint8*)&u32Version,0);
				}
				break;

				case E_SL_MSG_NODE_DEFAULT_RESPONSE_REQUEST_FLAG:
				{
					sControlBridge.sEndPoint.bDisableDefaultResponse = au8LinkRxBuffer[0];
					u8Status = 0;
				}
				break;

				case E_SL_MSG_BASIC_RESET_TO_FACTORY_DEFAULTS:
				{
					u8Status = eZclBasicResetToFactoryDefaults(au8LinkRxBuffer[3],	//  u8SourceEndPointId
															   au8LinkRxBuffer[4],	//  u8DestinationEndPointId
															   &sAddress,			// *psDestinationAddress,
															   &u8SeqNum);			// *pu8TransactionSequenceNumber
				}
				break;
#ifdef APP_ZCL_ON_HOST
                case E_SL_MSG_APS_DATA_REQ:
                {
                    PDUM_thAPduInstance hAPduInst;
                    ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

                    hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

                    if (PDUM_INVALID_HANDLE != hAPduInst)
                    {
                        uint16 u16ClusterId;
                        uint16 u16ProfileId;
                        ZPS_tsAfProfileDataReq sAfProfileDataReq;

                        memcpy(&u16ClusterId, &au8LinkRxBuffer[5], sizeof(uint16));
                        memcpy(&u16ProfileId, &au8LinkRxBuffer[7], sizeof(uint16));

                        sAfProfileDataReq.uDstAddr.u16Addr = sAddress.uAddress.u16DestinationAddress;
                        sAfProfileDataReq.u16ClusterId = u16ClusterId;
                        sAfProfileDataReq.u16ProfileId = u16ProfileId;
                        sAfProfileDataReq.u8SrcEp = au8LinkRxBuffer[3];
                        sAfProfileDataReq.eDstAddrMode = sAddress.eAddressMode;
                        sAfProfileDataReq.u8DstEp = au8LinkRxBuffer[4];
                        sAfProfileDataReq.eSecurityMode = au8LinkRxBuffer[9];
                        sAfProfileDataReq.u8Radius = au8LinkRxBuffer[10];

                        uint8 i;
                        uint16 u16Len = 0;
                        for (i = 0; i < au8LinkRxBuffer[11]; i++)
                        {
                            u16Len += PDUM_u16APduInstanceWriteNBO(hAPduInst, i, "b", au8LinkRxBuffer[12+i]);
                        }
                        PDUM_eAPduInstanceSetPayloadSize(hAPduInst, u16Len);

                        eStatus = ZPS_eAplAfApsdeDataReq(hAPduInst,
                                                          &sAfProfileDataReq,
                                                          &u8SeqNum);
                        if(eStatus != ZPS_E_SUCCESS)
                        {
                            PDUM_eAPduFreeAPduInstance(hAPduInst);
                        }
                    }
                    return eStatus;
                }
                break;

	#endif
				#ifdef CLD_OTA
				case E_SL_MSG_IMAGE_NOTIFY:
				{
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nE_SL_MSG_IMAGE_NOTIFY");

					uint8 u8SrcEndPoint;
					uint8 u8DstEndPoint;
					tsOTA_ImageNotifyCommand sImageNotifyCommand;

					u8SrcEndPoint = au8LinkRxBuffer[3];
					u8DstEndPoint = au8LinkRxBuffer[4];
					memcpy(&sImageNotifyCommand.ePayloadType, &au8LinkRxBuffer[5], sizeof(uint8));
					memcpy(&sImageNotifyCommand.u32NewFileVersion, &au8LinkRxBuffer[6], sizeof(uint32));
					memcpy(&sImageNotifyCommand.u16ImageType, &au8LinkRxBuffer[10], sizeof(uint16));
					memcpy(&sImageNotifyCommand.u16ManufacturerCode, &au8LinkRxBuffer[12], sizeof(uint16));
					memcpy(&sImageNotifyCommand.u8QueryJitter, &au8LinkRxBuffer[14], sizeof(uint8));

					u8Status = eOTA_ServerImageNotify(u8SrcEndPoint, 			// u8SourceEndpoint
													  u8DstEndPoint, 	   		// u8DestinationEndpoint
													  &sAddress, 	    		// *psDestinationAddress
													  &sImageNotifyCommand);	// *psImageNotifyCommand
				}
				break;

				case E_SL_MSG_UPGRADE_END_RESPONSE:
				{
					uint8 u8SrcEndPoint;
					uint8 u8DstEndPoint;
					tsOTA_UpgradeEndResponsePayload sUpgradeResponsePayload;

					u8SrcEndPoint = au8LinkRxBuffer[3];
					u8DstEndPoint = au8LinkRxBuffer[4];
					memcpy(&sUpgradeResponsePayload.u32UpgradeTime, &au8LinkRxBuffer[6], sizeof(uint32));
					memcpy(&sUpgradeResponsePayload.u32CurrentTime, &au8LinkRxBuffer[10], sizeof(uint32));
					memcpy(&sUpgradeResponsePayload.u32FileVersion, &au8LinkRxBuffer[14], sizeof(uint32));
					memcpy(&sUpgradeResponsePayload.u16ImageType, &au8LinkRxBuffer[18], sizeof(uint16));
					memcpy(&sUpgradeResponsePayload.u16ManufacturerCode, &au8LinkRxBuffer[20], sizeof(uint16));

					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nE_SL_MSG_UPGRADE_END_RESPONSE");
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr Mode: %x", sAddress.eAddressMode);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr: %x", sAddress.uAddress.u16DestinationAddress);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nSrcEndPoint: %x", u8SrcEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nDstEndPoint: %x", u8DstEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nUpgradeTime: %x", sUpgradeResponsePayload.u32UpgradeTime);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nCurrentTime: %x", sUpgradeResponsePayload.u32CurrentTime);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nFileVersion: %x", sUpgradeResponsePayload.u32FileVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nImageType: %x", sUpgradeResponsePayload.u16ImageType);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nManufacturerCode: %x", sUpgradeResponsePayload.u16ManufacturerCode);

					u8Status = eOTA_ServerUpgradeEndResponse(u8SrcEndPoint,				//  u8SourceEndpoint
															 u8DstEndPoint,				//  u8DestinationEndpoint
															 &sAddress,					// *psDestinationAddress,
															 &sUpgradeResponsePayload,	// *psUpgradeResponsePayload,
															 au8LinkRxBuffer[5]);		// uint8 u8TransactionSequenceNumber);
				}
				break;

				case E_SL_MSG_BLOCK_SEND:
				{
					uint8 u8SrcEndPoint;
					uint8 u8DstEndPoint;
					tsOTA_ImageBlockResponsePayload sImageBlockResponsePayload;

					u8SrcEndPoint = au8LinkRxBuffer[3];
					u8DstEndPoint = au8LinkRxBuffer[4];
					sImageBlockResponsePayload.u8Status = au8LinkRxBuffer[6];
					memcpy(&sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileOffset, &au8LinkRxBuffer[7], sizeof(uint32));
					memcpy(&sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileVersion, &au8LinkRxBuffer[11], sizeof(uint32));
					memcpy(&sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ImageType, &au8LinkRxBuffer[15], sizeof(uint16));
					memcpy(&sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ManufacturerCode, &au8LinkRxBuffer[17], sizeof(uint16));
					memcpy(&sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u8DataSize, &au8LinkRxBuffer[19], sizeof(uint8));
					sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.pu8Data = &au8LinkRxBuffer[20];

					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nE_SL_MSG_BLOCK_SEND");
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr Mode: %x", sAddress.eAddressMode);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr: %x", sAddress.uAddress.u16DestinationAddress);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nSrcEndPoint: %x", u8SrcEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nDstEndPoint: %x", u8DstEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nStatus: %x", sImageBlockResponsePayload.u8Status);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nFileOffset: %x", sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileOffset);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nFileVersion: %x", sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u32FileVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nImageType: %x", sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ImageType);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nManufacturerCode: %x", sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u16ManufacturerCode);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nDataSize: %x", sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u8DataSize);

					u8Status = eOTA_ServerImageBlockResponse(u8SrcEndPoint, 														//  u8SourceEndpoint
															 u8DstEndPoint,															//  u8DestinationEndpoint
															 &sAddress,																// *psDestinationAddress
															 &sImageBlockResponsePayload,											// *psImageBlockResponsePayload
															 sImageBlockResponsePayload.uMessage.sBlockPayloadSuccess.u8DataSize,	//	u8BlockSize
															 au8LinkRxBuffer[5]);													//  u8TransactionSequenceNumber
				}
				break;

				case E_SL_MSG_LOAD_NEW_IMAGE:
				{
					tsOTA_CoProcessorOTAHeader sCoProcessorOTAHeader;
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32FileIdentifier, &au8LinkRxBuffer[3], sizeof(uint32));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderVersion, &au8LinkRxBuffer[7], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderLength, &au8LinkRxBuffer[9], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderControlField, &au8LinkRxBuffer[11], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16ManufacturerCode, &au8LinkRxBuffer[13], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16ImageType, &au8LinkRxBuffer[15], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32FileVersion, &au8LinkRxBuffer[17], sizeof(uint32));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16StackVersion, &au8LinkRxBuffer[21], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].stHeaderString, &au8LinkRxBuffer[23], OTA_HEADER_STRING_SIZE);
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32TotalImage, &au8LinkRxBuffer[55], sizeof(uint32));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u8SecurityCredVersion, &au8LinkRxBuffer[59], sizeof(uint8));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u64UpgradeFileDest, &au8LinkRxBuffer[60], sizeof(uint64));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16MinimumHwVersion, &au8LinkRxBuffer[68], sizeof(uint16));
					memcpy(&sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16MaxHwVersion, &au8LinkRxBuffer[70], sizeof(uint16));

					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nE_SL_MSG_LOAD_NEW_IMAGE");
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nFileIdentifier: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32FileIdentifier);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nHeaderVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nHeaderLength: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderLength);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nHeaderControlField: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16HeaderControlField);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nManufacturerCode: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16ManufacturerCode);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nImageType: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16ImageType);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nFileVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32FileVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nStackVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16StackVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nHeaderString: %s", sCoProcessorOTAHeader.sOTA_ImageHeader[0].stHeaderString);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nTotalImage: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u32TotalImage);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nSecurityCredVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u8SecurityCredVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nUpgradeFileDest: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u64UpgradeFileDest);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nMinimumHwVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16MinimumHwVersion);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nMaxHwVersion: %x", sCoProcessorOTAHeader.sOTA_ImageHeader[0].u16MaxHwVersion);

					u8Status = eOTA_NewImageLoaded(ZIGBEENODECONTROLBRIDGE_HA_ENDPOINT, TRUE, &sCoProcessorOTAHeader);
				}
				break;
				case E_SL_MSG_SEND_WAIT_FOR_DATA_PARAMS:
				{
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nE_SL_MSG_SEND_WAIT_FOR_DATA_PARAMS");

					uint8                              u8SrcEndPoint;
					uint8                              u8DstEndPoint;
					tsOTA_ImageBlockResponsePayload    sImageBlockResponsePayload;

					u8SrcEndPoint                                                           =  au8LinkRxBuffer[3];
					u8DstEndPoint                                                           =  au8LinkRxBuffer[4];
					sImageBlockResponsePayload.u8Status                                     =  au8LinkRxBuffer[6];

					memcpy(&sImageBlockResponsePayload.uMessage.sWaitForData.u32CurrentTime, &au8LinkRxBuffer[7], sizeof(uint32));
					memcpy(&sImageBlockResponsePayload.uMessage.sWaitForData.u32RequestTime, &au8LinkRxBuffer[11], sizeof(uint32));
					memcpy(&sImageBlockResponsePayload.uMessage.sWaitForData.u16BlockRequestDelayMs, &au8LinkRxBuffer[15], sizeof(uint16));

					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr Mode: %x", sAddress.eAddressMode);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAddr: %x", sAddress.uAddress.u16DestinationAddress);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nSrcEndPoint: %x", u8SrcEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nDstEndPoint: %x", u8DstEndPoint);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nStatus: %x", sImageBlockResponsePayload.u8Status);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nCurrentTime: %x", sImageBlockResponsePayload.uMessage.sWaitForData.u32CurrentTime);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nRequestTime: %x", sImageBlockResponsePayload.uMessage.sWaitForData.u32RequestTime);
					vLog_Printf(TRACE_APP, LOG_DEBUG, "\nBlockDelay: %x", sImageBlockResponsePayload.uMessage.sWaitForData.u16BlockRequestDelayMs);

					u8Status = eOTA_ServerImageBlockResponse( u8SrcEndPoint,                    /* u8SourceEndpoint */
															  u8DstEndPoint,                    /*  u8DestinationEndpoint */
															  &sAddress,                        /*  *psDestinationAddress */
															  &sImageBlockResponsePayload,      /* *psImageBlockResponsePayload */
															  0,                                /*  u8BlockSize           */
															  au8LinkRxBuffer[5]);              /*  u8TransactionSequenceNumb */
				}
				break;

				#endif

				case E_SL_MSG_MANY_TO_ONE_ROUTE_REQUEST:
				{
					u8Status = ZPS_eAplZdoManyToOneRouteRequest(au8LinkRxBuffer[3],				// bCacheRoute
																au8LinkRxBuffer[4]);			// u8Radius
				}
				break;

				case E_SL_MSG_ATTRIBUTE_DISCOVERY_REQUEST:
				case E_SL_MSG_ATTRIBUTE_EXT_DISCOVERY_REQUEST:
				{
					bool_t bIsExtReq = FALSE;
					uint16 u16ClusterId;
					uint16 u16AttributeId;
					uint16 u16ManufacturerCode;

					if (E_SL_MSG_ATTRIBUTE_EXT_DISCOVERY_REQUEST == u16PacketType)
					{
						bIsExtReq = TRUE;
					}

					memcpy(&u16ClusterId, &au8LinkRxBuffer[5], sizeof(uint16));
					memcpy(&u16AttributeId, &au8LinkRxBuffer[7], sizeof(uint16));
					memcpy(&u16ManufacturerCode, &au8LinkRxBuffer[11], sizeof(uint16));

					u8Status = eZclDiscoverAttributes(bIsExtReq,
													  au8LinkRxBuffer[3],	// u8SourceEndPointId
													  au8LinkRxBuffer[4],	// u8DestinationEndPointId
													  u16ClusterId,			// u16ClusterId
													  au8LinkRxBuffer[9],	// bDirectionIsServerToClient
													  &sAddress,			// *psDestinationAddress
													  &u8SeqNum,			// *pu8TransactionSequenceNumber
													  u16AttributeId, 		// u16AttributeId
													  au8LinkRxBuffer[10],	// bIsManufacturerSpecific
													  u16ManufacturerCode,	// u16ManufacturerCode
													  au8LinkRxBuffer[13]);	// u8MaximumNumberOfIdentifiers
				}
				break;

				case E_SL_MSG_COMMAND_GENERATED_DISCOVERY_REQUEST:
				{
					uint16 u16ClusterId;
					uint16 u16ManufacturerCode;

					memcpy(&u16ClusterId, &au8LinkRxBuffer[5], sizeof(uint16));
					memcpy(&u16ManufacturerCode, &au8LinkRxBuffer[10], sizeof(uint16));

					u8Status = eZclDiscoverCommandGenerated(au8LinkRxBuffer[3], 	// u8SourceEndPointId
															au8LinkRxBuffer[4], 	// u8DestinationEndPointId
															u16ClusterId,			// u16ClusterId
															au8LinkRxBuffer[7],		// bDirectionIsServerToClient
															&sAddress,				// *psDestinationAddress
															&u8SeqNum,				// *pu8TransactionSequenceNumber
															au8LinkRxBuffer[8],		// u8CommandId
															au8LinkRxBuffer[9],		// bIsManufacturerSpecific
															u16ManufacturerCode, 	// u16ManufacturerCode
															au8LinkRxBuffer[12]); 	// u8MaximumNumberOfCommands
				}
				break;

				case E_SL_MSG_COMMAND_RECEIVED_DISCOVERY_REQUEST:
				{
					uint16 u16ClusterId;
					uint16 u16ManufacturerCode;

					memcpy(&u16ClusterId, &au8LinkRxBuffer[5], sizeof(uint16));
					memcpy(&u16ManufacturerCode, &au8LinkRxBuffer[10], sizeof(uint16));

					u8Status = eZclDiscoverCommandReceived(au8LinkRxBuffer[3],		// u8SourceEndPointId
														   au8LinkRxBuffer[4],		// u8DestinationEndPointId
														   u16ClusterId,			// u16ClusterId
														   au8LinkRxBuffer[7], 	 	// bDirectionIsServerToClient
														   &sAddress,		 	 	// *psDestinationAddress
														   &u8SeqNum,		 	 	// *pu8TransactionSequenceNumber
														   au8LinkRxBuffer[8],	 	// u8CommandId
														   au8LinkRxBuffer[9],	 	// bIsManufacturerSpecific
														   u16ManufacturerCode,	 	// u16ManufacturerCode
														   au8LinkRxBuffer[12]); 	// u8MaximumNumberOfCommands
				}
				break;

				case (E_SL_MSG_SET_EXT_PANID):
				{
					uint64 u64Value ;
					memcpy(&u64Value,au8LinkRxBuffer,sizeof(uint64));
					u8Status = ZPS_eAplAibSetApsUseExtendedPanId(u64Value);
				}
				break;
				case (E_SL_MSG_SET_CHANNELMASK):
				{
					uint32 u32Value;
					memcpy(&u32Value,au8LinkRxBuffer,sizeof(uint32));
					u8Status = ZPS_eAplAibSetApsChannelMask(u32Value);
					u32ChannelMask = u32Value;
				}
				break;

				case (E_SL_MSG_GET_PERMIT_JOIN):
				{
					APP_tsEvent sAppEvent;
					sAppEvent.eType = APP_E_EVENT_SEND_PERMIT_JOIN;
					OS_ePostMessage(APP_msgEvents, &sAppEvent);
				}
				break;

				case (E_SL_MSG_NETWORK_STATE_REQ):
                {
				    APP_tsEvent sAppEvent;
				    sAppEvent.eType = APP_E_EVENT_NETWORK_STATE;
				    OS_ePostMessage(APP_msgEvents, &sAppEvent);
                }
				break;

				case (E_SL_MSG_SET_DEVICETYPE):
				{
					if(au8LinkRxBuffer[0] >= 2 )
					{
						APP_vConfigureDevice(1); /* configure it in HA compatibility mode */
					}
					else
					{
						APP_vConfigureDevice(au8LinkRxBuffer[0]);
					}
					sDeviceDesc.u8DeviceType = au8LinkRxBuffer[0];
					PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
				}
				break;

				case (E_SL_MSG_NETWORK_REMOVE_DEVICE):
					{
					ZPS_tuAddress uParentAddress;
					ZPS_tuAddress uChildAddress;

					memcpy(&uParentAddress, &au8LinkRxBuffer[0], sizeof(ZPS_tuAddress));
					memcpy(&uChildAddress, &au8LinkRxBuffer[8], sizeof(ZPS_tuAddress));

					u8Status = eZdpRemoveDeviceReq(uParentAddress, uChildAddress);
					}
				break;

				case E_SL_MSG_LEAVE_REQUEST:
				{
					ZPS_tuAddress uAddress;
					bool u8RemoveChildren;
					bool bRejoin;

					memcpy(&uAddress, &au8LinkRxBuffer[0], sizeof(ZPS_tuAddress));
					bRejoin = au8LinkRxBuffer[8];
					u8RemoveChildren = au8LinkRxBuffer[9];

					u8Status = eZdpLeaveReq(uAddress, u8RemoveChildren, bRejoin);
				}
				break;

				case (E_SL_MSG_MANAGEMENT_LEAVE_REQUEST):
				{
					uint64 u64LookupAddress;
					memcpy(&u16TargetAddress,&au8LinkRxBuffer[0],sizeof(uint16));
					memcpy(&u64LookupAddress,&au8LinkRxBuffer[2],sizeof(uint64));
					u8Status = eZdpMgmtLeave(u16TargetAddress,u64LookupAddress,au8LinkRxBuffer[10],au8LinkRxBuffer[11],&u8SeqNum);
				}
				break;

				case (E_SL_MSG_BIND):
				case (E_SL_MSG_UNBIND):
				{
					uint64 u64BindAddress;
					uint16 u16Clusterid;
					uint8 u8SrcEp,offset = 0,u8DstEp,u8DstAddrMode;
					ZPS_tuAddress uDstAddress;
					bool_t bBind;
					offset = 0;

					memcpy(&u64BindAddress,&au8LinkRxBuffer[0],sizeof(uint64));
					offset += sizeof(uint64);
					memcpy(&u8SrcEp,&au8LinkRxBuffer[offset],sizeof(uint8));
					offset += sizeof(uint8);
					memcpy(&u16Clusterid,&au8LinkRxBuffer[offset],sizeof(uint16));
					offset += sizeof(uint16);
					memcpy(&u8DstAddrMode,&au8LinkRxBuffer[offset],sizeof(uint8));
					offset += sizeof(uint8);
					if(u8DstAddrMode == 0x3)
					{
						memcpy(&uDstAddress.u64Addr,&au8LinkRxBuffer[offset],sizeof(uint64));
						offset += sizeof(uint64);
						memcpy(&u8DstEp,&au8LinkRxBuffer[offset],sizeof(uint8));
						offset += sizeof(uint8);
					}
					else
					{

						memcpy(&uDstAddress.u16Addr,&au8LinkRxBuffer[offset],sizeof(uint64));
						offset += sizeof(uint64);
						u8DstEp = 0;
					}
					if(u16PacketType == E_SL_MSG_BIND)
					{
						bBind = TRUE;
					}
					else
					{
						bBind = FALSE;
					}
					u8Status = eBindUnbindEntry(bBind,u64BindAddress,u8SrcEp,u16Clusterid,&uDstAddress,u8DstEp,u8DstAddrMode,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_BIND_GROUP):
				{
					uint16 u16Clusterid;
					uint8 u8SrcEp, offset = 0;
					uint16 u16GroupAddr;

					memcpy(&u16Clusterid,&au8LinkRxBuffer[0],sizeof(uint16));
					offset += sizeof(uint16);
					memcpy(&u8SrcEp,&au8LinkRxBuffer[offset],sizeof(uint8));
					offset += sizeof(uint8);
					memcpy(&u16GroupAddr,&au8LinkRxBuffer[offset],sizeof(uint16));
					offset += sizeof(uint16);

					ZPS_eAplZdoBindGroup(u16Clusterid, u8SrcEp, u16GroupAddr);
				}
				break;

				case (E_SL_MSG_UNBIND_GROUP):
				{
					uint16 u16Clusterid;
					uint8 u8SrcEp, offset = 0;
					uint16 u16GroupAddr;

					memcpy(&u16Clusterid,&au8LinkRxBuffer[0],sizeof(uint16));
					offset += sizeof(uint16);
					memcpy(&u8SrcEp,&au8LinkRxBuffer[offset],sizeof(uint8));
					offset += sizeof(uint8);
					memcpy(&u16GroupAddr,&au8LinkRxBuffer[offset],sizeof(uint16));
					offset += sizeof(uint16);

					ZPS_eAplZdoUnbindGroup(u16Clusterid, u8SrcEp, u16GroupAddr);
				}
				break;


				case (E_SL_MSG_RESET):
				{
					bResetIssued = TRUE;
					OS_eStartSWTimer(APP_IdTimer, APP_TIME_MS(1), NULL);

				}
				break;
				case (E_SL_MSG_START_NETWORK):
				{
					vControlNodeStartNetwork();
				}
				break;
				case (E_SL_MSG_START_SCAN):
				{
					vControlNodeScanStart();
				}
				break;
#ifdef APP_DISCOVERY_ONLY_MODE_ENABLED
				case (E_SL_MSG_START_DISCOVERY_ONLY):
                {
				    vScanDiscoveryOnly();
                }
#endif
				break;
				case (E_SL_MSG_SET_SECURITY):
				{
					ZPS_vAplSecSetInitialSecurityState(au8LinkRxBuffer[0], &au8LinkRxBuffer[3], au8LinkRxBuffer[1], au8LinkRxBuffer[2]);
				}
				break;

				case (E_SL_MSG_ERASE_PERSISTENT_DATA):
				{
					PDM_vDeleteAllDataRecords();
					bResetIssued = TRUE;
					OS_eStartSWTimer(APP_IdTimer, APP_TIME_MS(10), NULL);
				}
				break;

				case (E_SL_MSG_ZLL_FACTORY_NEW):
				{
					if(sDeviceDesc.u8DeviceType != ZPS_ZDO_DEVICE_COORD)
					{
						ZPS_tsNwkNib *psNib = ZPS_psAplZdoGetNib();
						u32OldFrameCtr = psNib->sTbl.u32OutFC;
						vFactoryResetRecords();
					}
				}
				break;

				case (E_SL_MSG_INITIATE_TOUCHLINK):
				{
					if(sCommission.eState == E_IDLE ||
							(sCommission.eState == E_ACTIVE &&
									sZllState.eState == NOT_FACTORY_NEW))
					{
						APP_CommissionEvent sEvent;
						sEvent.eType = APP_E_COMMISION_START;
						OS_ePostMessage(APP_CommissionEvents, &sEvent);
					}
					else
					{
						u8Status = E_SL_MSG_STATUS_BUSY;
						bProcessMessages = FALSE;
					}
				}
				break;

				case (E_SL_MSG_MATCH_DESCRIPTOR_REQUEST):
				{
					uint16 au16InClusterList[10],au16OutClusterList[10];
					uint16 u16Profile;
					uint8 u8InClusterCount = au8LinkRxBuffer[4];
					uint8 u8OutClusterCount = au8LinkRxBuffer[((au8LinkRxBuffer[4]*(sizeof(uint16)))+5)];
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u16Profile,&au8LinkRxBuffer[2],sizeof(uint16));
					memcpy(au16InClusterList,&au8LinkRxBuffer[5],(sizeof(uint16)*u8InClusterCount));

					if(u8OutClusterCount != 0)
					{
						memcpy(au16OutClusterList,&au8LinkRxBuffer[(au8LinkRxBuffer[4]*2)+6],(sizeof(uint16)*u8OutClusterCount));
					}
					u8Status = eZdpMatchDescReq(u16TargetAddress,u16Profile,u8InClusterCount,au16InClusterList,u8OutClusterCount,au16OutClusterList,&u8SeqNum);

				}
				break;


				case (E_SL_MSG_NODE_DESCRIPTOR_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpNodeDescReq(u16TargetAddress,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_COMPLEX_DESCRIPTOR_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpComplexDescReq(u16TargetAddress,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_SIMPLE_DESCRIPTOR_REQUEST):
				{
					uint8 u8Endpoint = au8LinkRxBuffer[2];
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpSimpleDescReq(u16TargetAddress,u8Endpoint,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_PERMIT_JOINING_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					if(sDeviceDesc.u8DeviceType >=2)
					{
						ZPS_eAplAibSetApsTrustCenterAddress(ZPS_u64NwkNibGetExtAddr(ZPS_pvAplZdoGetNwkHandle()));
					}
					u8Status = eZdpPermitJoiningReq(u16TargetAddress,au8LinkRxBuffer[2],au8LinkRxBuffer[3],&u8SeqNum);

				}
				break;


				case (E_SL_MSG_POWER_DESCRIPTOR_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpPowerDescReq(u16TargetAddress,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_USER_DESCRIPTOR_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpUserDescReq(u16TargetAddress,&u8SeqNum);
				}
				break;

				case (E_SL_MSG_USER_DESCRIPTOR_SET_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpUserSetDescReq(u16TargetAddress, au8LinkRxBuffer[2], &au8LinkRxBuffer[3],&u8SeqNum);
				}
				break;


				case (E_SL_MSG_ACTIVE_ENDPOINT_REQUEST):
				{
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					u8Status = eZdpActiveEndpointReq(u16TargetAddress,&u8SeqNum);
				}
					break;

				case (E_SL_MSG_MANAGEMENT_NETWORK_UPDATE_REQUEST):
				{
					uint32 u32ChannelMask;
					uint8 u8ScanDuration,u8ScanCount;
					uint16 u16NwkManagerAddr;

					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u32ChannelMask,&au8LinkRxBuffer[2],sizeof(uint32));
					memcpy(&u8ScanDuration,&au8LinkRxBuffer[6],sizeof(uint8));
					memcpy(&u8ScanCount,&au8LinkRxBuffer[7],sizeof(uint8));
					memcpy(&u16NwkManagerAddr,&au8LinkRxBuffer[8],sizeof(uint16));

					u8Status = eZdpMgmtNetworkUpdateReq(u16TargetAddress,u32ChannelMask,u8ScanDuration,u8ScanCount, &u8SeqNum, u16NwkManagerAddr);
				}
					break;

				case (E_SL_MSG_SYSTEM_SERVER_DISCOVERY):
				{
					uint16 u16ServerMask;
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u16ServerMask,&au8LinkRxBuffer[2],sizeof(uint16));
					u8Status = eZdpSystemServerDiscovery(u16ServerMask,&u8SeqNum);
				}
					break;

				case (E_SL_MSG_TOUCHLINK_FACTORY_RESET):
				{
					APP_CommissionEvent sEvent;
					sEvent.eType = APP_E_COMMISION_START;
					OS_ePostMessage(APP_CommissionEvents, &sEvent);
					bSendFactoryResetOverAir = TRUE;
				}
				break;

				case (E_SL_MSG_IEEE_ADDRESS_REQUEST):
				{
					uint16 u16LookupAddress;
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u16LookupAddress,&au8LinkRxBuffer[2],sizeof(uint16));
					u8Status = eZdpIeeeAddrReq(u16TargetAddress,u16LookupAddress,au8LinkRxBuffer[4],au8LinkRxBuffer[5],&u8SeqNum);
				}
				break;

				case (E_SL_MSG_NETWORK_ADDRESS_REQUEST):
				{
					uint64 u64LookupAddress;
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u64LookupAddress,&au8LinkRxBuffer[2],sizeof(uint64));
					u8Status = eZdpNwkAddrReq(u16TargetAddress,u64LookupAddress,au8LinkRxBuffer[10],au8LinkRxBuffer[11],&u8SeqNum);
				}
				break;
				case (E_SL_MSG_ADD_AUTHENTICATE_DEVICE):
				{
					APP_tsEvent sAppEvent;
					sAppEvent.eType = APP_E_EVENT_ENCRYPT_SEND_KEY;
					memcpy(&sAppEvent.uEvent.sEncSendMsg.u64Address,au8LinkRxBuffer,sizeof(uint64));
					u8Status = ZPS_bAplZdoTrustCenterSetDevicePermissions(sAppEvent.uEvent.sEncSendMsg.u64Address,ZPS_DEVICE_PERMISSIONS_ALL_PERMITED);
					if(u8Status == ZPS_E_SUCCESS)
					{
						memcpy(&sAppEvent.uEvent.sEncSendMsg.uKey.au8,&au8LinkRxBuffer[8],(sizeof(8)*16));
						OS_ePostMessage(APP_msgEvents, &sAppEvent);
					}
				}
				break;

				case (E_SL_MSG_NETWORK_WHITELIST_ENABLE):
				{
					memcpy(&bBlackListEnable,au8LinkRxBuffer,sizeof(uint8));
					ZPS_vTCSetCallback(bSendHATransportKey);
				}
				break;

				case (E_SL_MSG_MANAGEMENT_LQI_REQUEST):
				{
					uint8 u8StartIndex;
					memcpy(&u16TargetAddress,au8LinkRxBuffer,sizeof(uint16));
					memcpy(&u8StartIndex,&au8LinkRxBuffer[2],sizeof(uint8));
					u8Status = eZdpMgmtLqiRequest(u16TargetAddress,u8StartIndex,&u8SeqNum);

				}
						break;
	/* Group cluster commands */
				case (E_SL_MSG_ADD_GROUP):
				{
					if (0x0000 == u16TargetAddress)
					{
						uint16 u16GroupId;
						memcpy(&u16GroupId, &au8LinkRxBuffer[5], sizeof(uint16));

						vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAdd Group ID: %x", u16GroupId);
						vLog_Printf(TRACE_APP, LOG_DEBUG, "\nAdd EndPoint: %x", au8LinkRxBuffer[4]);

						/* Request to add the bridge to a group, no name supported... */
						u8Status = ZPS_eAplZdoGroupEndpointAdd(u16GroupId, au8LinkRxBuffer[4]);
						uint8 i;
						ZPS_tsAplAib *psAplAib = ZPS_psAplAibGetAib();
						for (i = 0; i < psAplAib->psAplApsmeGroupTable->u32SizeOfGroupTable; i++)
						{
							vLog_Printf(TRACE_APP, LOG_DEBUG, "\nGroup ID: %x", psAplAib->psAplApsmeGroupTable->psAplApsmeGroupTableId[i].u16Groupid);
							vLog_Printf(TRACE_APP, LOG_DEBUG, "\nEndPoint 0: %x", psAplAib->psAplApsmeGroupTable->psAplApsmeGroupTableId[i].au8Endpoint[0]);
						}
					}
					else
					{
					    tsCLD_Groups_AddGroupRequestPayload sRequest;
					    uint8 au8Data[16];
					    uint8 u8Len = 5;

					    memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[u8Len],sizeof(uint16));        //5
					    u8Len += 2;

					    sRequest.sGroupName.u8Length = au8LinkRxBuffer[u8Len];                      //7
					    u8Len++;

					    sRequest.sGroupName.u8MaxLength = au8LinkRxBuffer[u8Len];                   //8
					    u8Len++;

					    memcpy(au8Data,&au8LinkRxBuffer[u8Len],sRequest.sGroupName.u8Length);       //9
					    sRequest.sGroupName.pu8Data = au8Data;

					    u8Len += sRequest.sGroupName.u8MaxLength;

					    u8Status = eCLD_GroupsCommandAddGroupRequestSend(au8LinkRxBuffer[3],
					            au8LinkRxBuffer[4],
					            &sAddress,
					            &u8SeqNum,
					            &sRequest);
					}
				}
				break;

				case (E_SL_MSG_REMOVE_GROUP):
				{
					if (0x0000 == u16TargetAddress)
					{
						uint16 u16GroupId;
						memcpy(&u16GroupId, &au8LinkRxBuffer[5], sizeof(uint16));

						/* Request is for the control bridge */
						u8Status = ZPS_eAplZdoGroupEndpointRemove(u16GroupId, au8LinkRxBuffer[4]);
					}
					else
					{
						/* Request is for a remote node */
						tsCLD_Groups_RemoveGroupRequestPayload sRequest;
						memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
						u8Status = eCLD_GroupsCommandRemoveGroupRequestSend(au8LinkRxBuffer[3],
																			au8LinkRxBuffer[4],
																			&sAddress,
																			&u8SeqNum,
																			&sRequest);
					}
				}
				break;

				case (E_SL_MSG_REMOVE_ALL_GROUPS):
				{
					if (0x0000 == u16TargetAddress)
					{
						vLog_Printf(TRACE_APP, LOG_DEBUG, "\nRemove All Groups");
						vLog_Printf(TRACE_APP, LOG_DEBUG, "\nDst EndPoint: %x", au8LinkRxBuffer[4]);

						/* Request is for the control bridge */
						u8Status = ZPS_eAplZdoGroupAllEndpointRemove(au8LinkRxBuffer[4]);
					}
					else
					{
						tsZCL_Address sAddress;
						uint16 u16TargetAddress;
						memcpy(&u16TargetAddress,&au8LinkRxBuffer[1],sizeof(uint16));
						sAddress.eAddressMode = au8LinkRxBuffer[0];
						sAddress.uAddress.u16DestinationAddress = u16TargetAddress;
						u8Status = eCLD_GroupsCommandRemoveAllGroupsRequestSend(au8LinkRxBuffer[3],
																				au8LinkRxBuffer[4],
																				&sAddress,
																				&u8SeqNum);
					}
				}
				break;

				case (E_SL_MSG_ADD_GROUP_IF_IDENTIFY):
				{
					tsCLD_Groups_AddGroupRequestPayload sRequest;

					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					sRequest.sGroupName.u8Length = 0;
					sRequest.sGroupName.u8MaxLength = 0;
					sRequest.sGroupName.pu8Data = (uint8*)"";
					u8Status = eCLD_GroupsCommandAddGroupIfIdentifyingRequestSend(au8LinkRxBuffer[3],
					au8LinkRxBuffer[4],
					&sAddress,
					&u8SeqNum,
					&sRequest);
				}
				break;
				case (E_SL_MSG_VIEW_GROUP):
				{
					tsCLD_Groups_ViewGroupRequestPayload sRequest;

					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					u8Status = eCLD_GroupsCommandViewGroupRequestSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;
				case (E_SL_MSG_GET_GROUP_MEMBERSHIP):
				{
					tsCLD_Groups_GetGroupMembershipRequestPayload sRequest;
					uint16 u16GroupList[10];

					memcpy(u16GroupList,&au8LinkRxBuffer[6], (sizeof(uint16)*au8LinkRxBuffer[5]));
					sRequest.pi16GroupList = (zint16*)u16GroupList;
					sRequest.u8GroupCount = au8LinkRxBuffer[5];
					u8Status = eCLD_GroupsCommandGetGroupMembershipRequestSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

	/*Scenes Cluster */
				case (E_SL_MSG_ADD_SCENE):
				{
					tsCLD_ScenesAddSceneRequestPayload sRequest;
					uint8 au8Data[16];
					uint8 u8Len = 5;

					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[u8Len],sizeof(uint16));                //5
					u8Len+=2;

					sRequest.u8SceneId = au8LinkRxBuffer[u8Len];                                        //7
					u8Len++;

					memcpy(&sRequest.u16TransitionTime,&au8LinkRxBuffer[u8Len],sizeof(uint16));         //8
					u8Len+=2;

					sRequest.sSceneName.u8Length = au8LinkRxBuffer[u8Len];                              //10
					u8Len++;

					sRequest.sSceneName.u8MaxLength = au8LinkRxBuffer[u8Len];                           //11
					u8Len++;

					memcpy(au8Data,&au8LinkRxBuffer[u8Len],sRequest.sSceneName.u8Length);               //12
					sRequest.sSceneName.pu8Data = au8Data;
					u8Len+=sRequest.sSceneName.u8Length;

					memcpy(&sRequest.sExtensionField.u16Length,&au8LinkRxBuffer[u8Len],sizeof(uint16)); //12+u8Length
					u8Len+=2;

					sRequest.sExtensionField.pu8Data = &au8LinkRxBuffer[u8Len];                         //14+u8Length

					u8Status = eCLD_ScenesCommandAddSceneRequestSend(au8LinkRxBuffer[3],
							   au8LinkRxBuffer[4],
							   &sAddress,
							   &u8SeqNum,
							   &sRequest);
				}
					break;

				case (E_SL_MSG_REMOVE_SCENE):
				{
					tsCLD_ScenesRemoveSceneRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					sRequest.u8SceneId= au8LinkRxBuffer[7];
					u8Status = eCLD_ScenesCommandRemoveSceneRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

				case (E_SL_MSG_VIEW_SCENE):
				{
					tsCLD_ScenesViewSceneRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					sRequest.u8SceneId= au8LinkRxBuffer[7];
					u8Status = eCLD_ScenesCommandViewSceneRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;


				case (E_SL_MSG_REMOVE_ALL_SCENES):
				{
					tsCLD_ScenesRemoveAllScenesRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					u8Status = eCLD_ScenesCommandRemoveAllScenesRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

				case (E_SL_MSG_STORE_SCENE):
				{
					tsCLD_ScenesStoreSceneRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					sRequest.u8SceneId= au8LinkRxBuffer[7];
					u8Status = eCLD_ScenesCommandStoreSceneRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

				case (E_SL_MSG_RECALL_SCENE):
				{
					tsCLD_ScenesRecallSceneRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					sRequest.u8SceneId= au8LinkRxBuffer[7];
					u8Status = eCLD_ScenesCommandRecallSceneRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

				case (E_SL_MSG_SCENE_MEMBERSHIP_REQUEST):
				{
					tsCLD_ScenesGetSceneMembershipRequestPayload sRequest;
					memcpy(&sRequest.u16GroupId,&au8LinkRxBuffer[5],sizeof(uint16));
					u8Status = eCLD_ScenesCommandGetSceneMembershipRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sRequest);
				}
				break;

	/* ON/OFF cluster commands */
				case (E_SL_MSG_ONOFF_EFFECTS):
				{

					tsCLD_OnOff_OffWithEffectRequestPayload sRequest;
					sRequest.u8EffectId = au8LinkRxBuffer[5];
					sRequest.u8EffectVariant = au8LinkRxBuffer[6];
					u8Status = eCLD_OnOffCommandOffWithEffectSend(au8LinkRxBuffer[3],au8LinkRxBuffer[4],&sAddress,&u8SeqNum,&sRequest);
				}
				break;

				case (E_SL_MSG_ONOFF_NOEFFECTS):
				{
					u8Status = eCLD_OnOffCommandSend(au8LinkRxBuffer[3],au8LinkRxBuffer[4],&sAddress,&u8SeqNum,au8LinkRxBuffer[5]);
				}
				break;

				case (E_SL_MSG_ONOFF_TIMED):
				{

					tsCLD_OnOff_OnWithTimedOffRequestPayload sRequest;
					sRequest.u8OnOff = au8LinkRxBuffer[5];
					memcpy(&sRequest.u16OnTime,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sRequest.u16OffTime,&au8LinkRxBuffer[8],sizeof(uint16));
					u8Status = eCLD_OnOffCommandOnWithTimedOffSend(au8LinkRxBuffer[3],au8LinkRxBuffer[4],&sAddress,&u8SeqNum,&sRequest);
				}
				break;
	/* colour cluster commands */

				case (E_SL_MSG_MOVE_HUE):
				{
					tsCLD_ColourControl_MoveHueCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					sPayload.u8Rate = au8LinkRxBuffer[6];

					u8Status = eCLD_ColourControlCommandMoveHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_TO_HUE_SATURATION):
				{
					tsCLD_ColourControl_MoveToHueAndSaturationCommandPayload sPayload;
					sPayload.u8Saturation = au8LinkRxBuffer[6];
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[7],sizeof(uint16));
					sPayload.u8Hue = au8LinkRxBuffer[5];

					u8Status = eCLD_ColourControlCommandMoveToHueAndSaturationCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_TO_HUE):
				{
					tsCLD_ColourControl_MoveToHueCommandPayload sPayload;
					sPayload.eDirection = au8LinkRxBuffer[6];
					sPayload.u8Hue = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[7],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandMoveToHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_STEP_HUE):
				{
					tsCLD_ColourControl_StepHueCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					sPayload.u8StepSize = au8LinkRxBuffer[6];
					sPayload.u8TransitionTime = au8LinkRxBuffer[7];

					u8Status = eCLD_ColourControlCommandStepHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_TO_SATURATION):
				{
					tsCLD_ColourControl_MoveToSaturationCommandPayload sPayload;
					sPayload.u8Saturation = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[6],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandMoveToSaturationCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_SATURATION):
				{
					tsCLD_ColourControl_MoveSaturationCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					sPayload.u8Rate = au8LinkRxBuffer[6];

					u8Status = eCLD_ColourControlCommandMoveSaturationCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_STEP_SATURATION):
				{
					tsCLD_ColourControl_StepSaturationCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					sPayload.u8StepSize = au8LinkRxBuffer[6];
					sPayload.u8TransitionTime = au8LinkRxBuffer[7];

					u8Status = eCLD_ColourControlCommandStepSaturationCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_TO_COLOUR):
				{
					tsCLD_ColourControl_MoveToColourCommandPayload sPayload;
					memcpy(&sPayload.u16ColourX ,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&sPayload.u16ColourY,&au8LinkRxBuffer[7],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[9],sizeof(uint16));


					u8Status = eCLD_ColourControlCommandMoveToColourCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_MOVE_COLOUR):
				{
					tsCLD_ColourControl_MoveColourCommandPayload sPayload;
					memcpy(&sPayload.i16RateX ,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&sPayload.i16RateY,&au8LinkRxBuffer[7],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandMoveColourCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_STEP_COLOUR):
				{
					tsCLD_ColourControl_StepColourCommandPayload sPayload;
					memcpy(&sPayload.i16StepX ,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&sPayload.i16StepY,&au8LinkRxBuffer[7],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[9],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandStepColourCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);

				}
				break;

				case (E_SL_MSG_COLOUR_LOOP_SET):
				{
					tsCLD_ColourControl_ColourLoopSetCommandPayload sPayload;
					sPayload.u8UpdateFlags = au8LinkRxBuffer[5];
					sPayload.eAction = au8LinkRxBuffer[6];
					sPayload.eDirection = au8LinkRxBuffer[7];
					memcpy(&sPayload.u16Time,&au8LinkRxBuffer[8],sizeof(uint16));
					memcpy(&sPayload.u16StartHue,&au8LinkRxBuffer[10],sizeof(uint16));
					u8Status = eCLD_ColourControlCommandColourLoopSetCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sPayload);
				}
				break;

				case (E_SL_MSG_MOVE_TO_COLOUR_TEMPERATURE):
				{
					tsCLD_ColourControl_MoveToColourTemperatureCommandPayload sPayload;
					memcpy(&sPayload.u16ColourTemperatureMired,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[7],sizeof(uint16));
					u8Status =     eCLD_ColourControlCommandMoveToColourTemperatureCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sPayload);
				}
				break;

				case (E_SL_MSG_MOVE_COLOUR_TEMPERATURE):
				{
					tsCLD_ColourControl_MoveColourTemperatureCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16Rate,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sPayload.u16ColourTemperatureMiredMin,&au8LinkRxBuffer[8],sizeof(uint16));
					memcpy(&sPayload.u16ColourTemperatureMiredMax,&au8LinkRxBuffer[10],sizeof(uint16));

					u8Status =     eCLD_ColourControlCommandMoveColourTemperatureCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sPayload);
				}
				break;

				case (E_SL_MSG_STEP_COLOUR_TEMPERATURE):
				{
					tsCLD_ColourControl_StepColourTemperatureCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16StepSize,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sPayload.u16ColourTemperatureMiredMin,&au8LinkRxBuffer[8],sizeof(uint16));
					memcpy(&sPayload.u16ColourTemperatureMiredMax,&au8LinkRxBuffer[10],sizeof(uint16));

					u8Status =     eCLD_ColourControlCommandStepColourTemperatureCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sPayload);
				}
				break;


				case (E_SL_MSG_ENHANCED_MOVE_TO_HUE):
				{
					tsCLD_ColourControl_EnhancedMoveToHueCommandPayload sPayload;
					sPayload.eDirection = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16EnhancedHue,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[8],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandEnhancedMoveToHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);
				}
				break;

				case (E_SL_MSG_ENHANCED_MOVE_HUE):
				{
					tsCLD_ColourControl_EnhancedMoveHueCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16Rate,&au8LinkRxBuffer[6],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandEnhancedMoveHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);
				}
				break;

				case (E_SL_MSG_ENHANCED_STEP_HUE):
				{
					tsCLD_ColourControl_EnhancedStepHueCommandPayload sPayload;
					sPayload.eMode = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16StepSize,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[8],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandEnhancedStepHueCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);
				}
				break;



				case (E_SL_MSG_ENHANCED_MOVE_TO_HUE_SATURATION):
				{
					tsCLD_ColourControl_EnhancedMoveToHueAndSaturationCommandPayload sPayload;
					sPayload.u8Saturation = au8LinkRxBuffer[5];
					memcpy(&sPayload.u16EnhancedHue,&au8LinkRxBuffer[6],sizeof(uint16));
					memcpy(&sPayload.u16TransitionTime,&au8LinkRxBuffer[8],sizeof(uint16));

					u8Status = eCLD_ColourControlCommandEnhancedMoveToHueAndSaturationCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum,
										&sPayload);
				}
				break;


				case (E_SL_MSG_STOP_MOVE_STEP):
				{

					u8Status = eCLD_ColourControlCommandStopMoveStepCommandSend(
							au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
										&sAddress,
										&u8SeqNum);
				}
				break;

	/* level cluster commands */
				case (E_SL_MSG_MOVE_TO_LEVEL_ONOFF):
				{
					tsCLD_LevelControl_MoveToLevelCommandPayload sCommand;

					sCommand.u8Level = au8LinkRxBuffer[6];
					memcpy(&sCommand.u16TransitionTime,&au8LinkRxBuffer[7],sizeof(uint16));
					u8Status = eCLD_LevelControlCommandMoveToLevelCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							au8LinkRxBuffer[5],
							&sCommand);
				}
					break;

				case (E_SL_MSG_MOVE_TO_LEVEL):
				{
					tsCLD_LevelControl_MoveCommandPayload sCommand;

					sCommand.u8MoveMode = au8LinkRxBuffer[6];
					sCommand.u8Rate = au8LinkRxBuffer[7];
					u8Status = eCLD_LevelControlCommandMoveCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							au8LinkRxBuffer[5],
							&sCommand);
				}
					break;


				case (E_SL_MSG_MOVE_STEP):
				{
					tsCLD_LevelControl_StepCommandPayload sCommand;

					sCommand.u8StepMode = au8LinkRxBuffer[6];
					sCommand.u8StepSize = au8LinkRxBuffer[7];
					memcpy(&sCommand.u16TransitionTime,&au8LinkRxBuffer[8],sizeof(uint16));
					u8Status = eCLD_LevelControlCommandStepCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							au8LinkRxBuffer[5],
							&sCommand);
				}
					break;

				case (E_SL_MSG_MOVE_STOP_MOVE):
				{

					u8Status = eCLD_LevelControlCommandStopCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum);
				}
					break;

				case (E_SL_MSG_MOVE_STOP_ONOFF):
				{

					u8Status = eCLD_LevelControlCommandStopWithOnOffCommandSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum);
				}
					break;
	/* Identify commands*/

				case (E_SL_MSG_IDENTIFY_SEND):
				{
					tsCLD_Identify_IdentifyRequestPayload sCommand;

					memcpy(&sCommand.u16IdentifyTime,&au8LinkRxBuffer[5],sizeof(uint16));
					u8Status = eCLD_IdentifyCommandIdentifyRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							&sCommand);
				}
					break;

				case (E_SL_MSG_IDENTIFY_QUERY):
				{

					u8Status = eCLD_IdentifyCommandIdentifyQueryRequestSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum);
				}
					break;
	#ifdef  CLD_IDENTIFY_SUPPORT_ZLL_ENHANCED_COMMANDS
				case (E_SL_MSG_IDENTIFY_TRIGGER_EFFECT):
				{
					u8Status = eCLD_IdentifyCommandTriggerEffectSend(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							&sAddress,
							&u8SeqNum,
							au8LinkRxBuffer[5],
							au8LinkRxBuffer[6]);
				}
					break;
	#endif
	/* profile agnostic commands */
				case (E_SL_MSG_READ_ATTRIBUTE_REQUEST):
				{
					uint16 au16AttributeList[10];
					uint16 u16ClusterId,u16ManId;

					memcpy(&u16ClusterId,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&u16ManId,&au8LinkRxBuffer[9],sizeof(uint16));
					memcpy(au16AttributeList,&au8LinkRxBuffer[12],(sizeof(uint16)*au8LinkRxBuffer[11]));

					u8Status = eZCL_SendReadAttributesRequest(au8LinkRxBuffer[3],
							au8LinkRxBuffer[4],
							u16ClusterId,
							au8LinkRxBuffer[7],
							&sAddress,
							&u8SeqNum,
							au8LinkRxBuffer[11],
							au8LinkRxBuffer[8],
							u16ManId,
							au16AttributeList);
				}
				break;

				case (E_SL_MSG_WRITE_ATTRIBUTE_REQUEST):
				{
					uint16 u16ClusterId,u16ManId,u16SizePayload;

					memcpy(&u16ClusterId,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&u16ManId,&au8LinkRxBuffer[9],sizeof(uint16));
					u16SizePayload = u16PacketLength - (sizeof(uint8)/*add mode*/ + sizeof(uint16) /*short addr*/ + sizeof(uint16)/*cluster id*/  + sizeof(uint16)/*manf id*/  + sizeof(uint8)/*manf specific flag*/ +
							sizeof(uint8)/*src ep*/+ sizeof(uint8)/*dest ep*/+ sizeof(uint8)/*num attrib*/ + sizeof(uint8)/*direction*/);
					u8Status =  eZnc_SendWriteAttributesRequest(
															   au8LinkRxBuffer[3],
															   au8LinkRxBuffer[4],
															  u16ClusterId,
															  au8LinkRxBuffer[7],
															   &sAddress,
															  &u8SeqNum,
															  au8LinkRxBuffer[8],
															  u16ManId,
															   &au8LinkRxBuffer[12],
															   au8LinkRxBuffer[11],
																u16SizePayload);
				}
				break;


				case E_SL_MSG_CONFIG_REPORTING_REQUEST:
				{
					uint16 u16ClusterId, u16ManId;
					tsZCL_AttributeReportingConfigurationRecord asAttribReportConfigRecord[10];

					memcpy(&u16ClusterId,&au8LinkRxBuffer[5],sizeof(uint16));
					memcpy(&u16ManId,&au8LinkRxBuffer[9],sizeof(uint16));

					int i;
					uint8 u8Offset = 12;
					for (i = 0; i < au8LinkRxBuffer[11]; i++)
					{
						if (i < 10)
						{
							/* Destination structure is not packed so we have to manually load rather than just copy */
							asAttribReportConfigRecord[i].u8DirectionIsReceived = au8LinkRxBuffer[u8Offset];
							u8Offset++;
							asAttribReportConfigRecord[i].eAttributeDataType = au8LinkRxBuffer[u8Offset];
							u8Offset++;
							memcpy(&asAttribReportConfigRecord[i].u16AttributeEnum, &au8LinkRxBuffer[u8Offset], sizeof(uint16));
							u8Offset+=2;
							memcpy(&asAttribReportConfigRecord[i].u16MinimumReportingInterval, &au8LinkRxBuffer[u8Offset], sizeof(uint16));
							u8Offset+=2;
							memcpy(&asAttribReportConfigRecord[i].u16MaximumReportingInterval, &au8LinkRxBuffer[u8Offset], sizeof(uint16));
							u8Offset+=2;
							memcpy(&asAttribReportConfigRecord[i].u16TimeoutPeriodField, &au8LinkRxBuffer[u8Offset], sizeof(uint16));
							u8Offset+=2;
							asAttribReportConfigRecord[i].uAttributeReportableChange.zuint8ReportableChange = au8LinkRxBuffer[u8Offset];
						}
					}

					u8Status =  eZCL_SendConfigureReportingCommand(au8LinkRxBuffer[3], 			// u8SourceEndPointId
																   au8LinkRxBuffer[4], 			// u8DestinationEndPointId
																   u16ClusterId, 				// u16ClusterId
																   au8LinkRxBuffer[7], 			// bDirectionIsServerToClient
																   &sAddress, 					// *psDestinationAddress
																   &u8SeqNum, 					// *pu8TransactionSequenceNumber
																   au8LinkRxBuffer[11],			// u8NumberOfAttributesInRequest
																   au8LinkRxBuffer[8], 			// bIsManufacturerSpecific
																   u16ManId, 					// u16ManufacturerCode
																   asAttribReportConfigRecord);	// *psAttributeReportingConfigurationRecord
				}
				break;

				case E_SL_MSG_READ_REPORT_CONFIG_REQUEST:
				{
					uint8  i;
					uint8  u8NumberOfAttributesInRequest;
					uint16 u16ClusterId;
					uint16 u16ManufacturerCode;
					tsZCL_AttributeReadReportingConfigurationRecord asAttributeReadReportingConfigurationRecord[8];

					u8NumberOfAttributesInRequest = au8LinkRxBuffer[8];
					memcpy(&u16ClusterId, &au8LinkRxBuffer[5], sizeof(uint16));
					memcpy(&u16ManufacturerCode, &au8LinkRxBuffer[10], sizeof(uint16));

					uint8 u8BufferOffset = 12;
					for (i = 0; i < u8NumberOfAttributesInRequest; i++)
					{
						asAttributeReadReportingConfigurationRecord[i].u8DirectionIsReceived = au8LinkRxBuffer[u8BufferOffset];
						u8BufferOffset += sizeof(uint8);
						memcpy(&asAttributeReadReportingConfigurationRecord[i].u16AttributeEnum, &au8LinkRxBuffer[u8BufferOffset], sizeof(uint16));
						u8BufferOffset += sizeof(uint16);
					}

					u8Status = eZCL_SendReadReportingConfigurationCommand(au8LinkRxBuffer[3],								//  u8SourceEndPointId,
																		  au8LinkRxBuffer[4],								//  u8DestinationEndPointId,
																		  u16ClusterId, 									//  u16ClusterId,
																		  au8LinkRxBuffer[7],								//  bDirectionIsServerToClient,
																		  &sAddress, 										// *psDestinationAddress,
																		  &u8SeqNum,										// *pu8TransactionSequenceNumber,
																		  u8NumberOfAttributesInRequest,					//  u8NumberOfAttributesInRequest,
																		  au8LinkRxBuffer[9],								//  bIsManufacturerSpecific,
																		  u16ManufacturerCode,								//  u16ManufacturerCode,
																		  &asAttributeReadReportingConfigurationRecord[0]);	//  *psAttributeReadReportingConfigurationRecord);
				}
				break;

				case E_SL_MSG_SEND_IAS_ZONE_ENROLL_RSP:
				{
					tsCLD_IASZone_EnrollResponsePayload sEnrollResponsePayload;

					sEnrollResponsePayload.e8EnrollResponseCode = au8LinkRxBuffer[5];
					sEnrollResponsePayload.u8ZoneID = au8LinkRxBuffer[6];

					u8Status = eCLD_IASZoneEnrollRespSend(au8LinkRxBuffer[3], 		// u8SourceEndPointId,
														  au8LinkRxBuffer[4],		// u8DestinationEndPointId,
														  &sAddress,				// *psDestinationAddress,
															  &u8SeqNum, 				// *pu8TransactionSequenceNumber,
															  &sEnrollResponsePayload); // *psPayload);
				}
					break;
		#ifdef CLD_DOOR_LOCK
						case (E_SL_MSG_LOCK_UNLOCK_DOOR):
						{
							u8Status = eCLD_DoorLockCommandLockUnlockRequestSend(au8LinkRxBuffer[3],au8LinkRxBuffer[4],&sAddress,&u8SeqNum,au8LinkRxBuffer[5]);
						}
						break;
		#endif
					case E_SL_MSG_ASC_LOG_MSG:
					{
						tsCLD_ASC_LogNotificationORLogResponsePayload sNotificationPayload;
						uint8 asAttribReportConfigRecord[70];
						memcpy(&sNotificationPayload.utctTime, &au8LinkRxBuffer[5], sizeof(uint32));
						memcpy(&sNotificationPayload.u32LogId, &au8LinkRxBuffer[9], sizeof(uint32));
						memcpy(&sNotificationPayload.u32LogLength, &au8LinkRxBuffer[13], sizeof(uint32));
						memcpy(asAttribReportConfigRecord, &au8LinkRxBuffer[17], (sizeof(uint8)*sNotificationPayload.u32LogLength ));
						sNotificationPayload.pu8LogData = asAttribReportConfigRecord;
						u8Status =  eCLD_ASCLogNotificationSend (au8LinkRxBuffer[3], 		// u8SourceEndPointId,
																 au8LinkRxBuffer[4],		// u8DestinationEndPointId,
																 &sAddress,				// *psDestinationAddress,
																 &u8SeqNum, 				// *pu8TransactionSequenceNumber,
																 &sNotificationPayload); // *psPayload);)
					}
						break;
					default:
						u8Status = E_SL_MSG_STATUS_UNHANDLED_COMMAND;
					break;
				}
			if (u16PacketType != E_SL_MSG_GET_VERSION)
			{
				memcpy(au8values,&u8Status,sizeof(uint8));

				u8SeqNum = eZCL_GetLastSequenceNumber();

				memcpy(&au8values[1],&u8SeqNum,sizeof(uint8));
				memcpy(&au8values[2],&u16PacketType,sizeof(uint16));
				vSL_WriteMessage(E_SL_MSG_STATUS, 4, au8values,0);
			}
		}
    }
}

/****************************************************************************
 *
 * NAME: eZclBasicResetToFactoryDefaults
 *
 * DESCRIPTION:
 *
 ****************************************************************************/
PRIVATE teZCL_Status eZclBasicResetToFactoryDefaults(uint8 u8SourceEndPointId,
        											 uint8 u8DestinationEndPointId,
        											 tsZCL_Address *psDestinationAddress,
        											 uint8 *pu8TransactionSequenceNumber)
{
	teZCL_Status eStatus;

	eStatus = eCLD_BasicCommandResetToFactoryDefaultsSend(u8SourceEndPointId,
	                    								  u8DestinationEndPointId,
	                    								  psDestinationAddress,
	                    								  pu8TransactionSequenceNumber);
	return eStatus;
}

/****************************************************************************
 *
 * NAME: eZclDiscoverCommandGenerated
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE teZCL_Status eZclDiscoverCommandGenerated(uint8 u8SourceEndPointId,
												  uint8 u8DestinationEndPointId,
												  uint16 u16ClusterId,
												  bool_t bDirectionIsServerToClient,
												  tsZCL_Address *psDestinationAddress,
												  uint8 *pu8TransactionSequenceNumber,
												  uint8 u8CommandId,
												  bool_t bIsManufacturerSpecific,
												  uint16 u16ManufacturerCode,
												  uint8 u8MaximumNumberOfCommands)
{
	teZCL_Status eStatus;

	eStatus = eZCL_SendDiscoverCommandGeneratedRequest(u8SourceEndPointId,
													   u8DestinationEndPointId,
													   u16ClusterId,
													   bDirectionIsServerToClient,
													   psDestinationAddress,
													   pu8TransactionSequenceNumber,
													   u8CommandId,
													   bIsManufacturerSpecific,
													   u16ManufacturerCode,
													   u8MaximumNumberOfCommands);
	return eStatus;
}

/****************************************************************************
 *
 * NAME: eZclDiscoverCommands
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE teZCL_Status eZclDiscoverCommandReceived(uint8 u8SourceEndPointId,
												 uint8 u8DestinationEndPointId,
												 uint16 u16ClusterId,
												 bool_t bDirectionIsServerToClient,
												 tsZCL_Address *psDestinationAddress,
												 uint8 *pu8TransactionSequenceNumber,
												 uint8 u8CommandId,
												 bool_t bIsManufacturerSpecific,
												 uint16 u16ManufacturerCode,
												 uint8 u8MaximumNumberOfCommands)
{
	teZCL_Status eStatus;

	eStatus = eZCL_SendDiscoverCommandReceivedRequest(u8SourceEndPointId,
													  u8DestinationEndPointId,
													  u16ClusterId,
													  bDirectionIsServerToClient,
													  psDestinationAddress,
													  pu8TransactionSequenceNumber,
													  u8CommandId,
													  bIsManufacturerSpecific,
													  u16ManufacturerCode,
													  u8MaximumNumberOfCommands);
	return eStatus;
}

/****************************************************************************
 *
 * NAME: eZclDiscoverAttributes
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE teZCL_Status eZclDiscoverAttributes(bool bIsExtReq,
											uint8 u8SourceEndPointId,
											uint8 u8DestinationEndPointId,
											uint16 u16ClusterId,
											bool_t bDirectionIsServerToClient,
											tsZCL_Address *psDestinationAddress,
											uint8 *pu8TransactionSequenceNumber,
											uint16 u16AttributeId,
											bool_t bIsManufacturerSpecific,
											uint16 u16ManufacturerCode,
											uint8 u8MaximumNumberOfIdentifiers)
{
	teZCL_Status eStatus;

	if (TRUE == bIsExtReq)
	{
		eStatus = eZCL_SendDiscoverAttributesExtendedRequest(u8SourceEndPointId,
				  	  	  	  	  	  	  	  	  	  	     u8DestinationEndPointId,
				  	  	  	  	  	  	  	  	  	  	  	 u16ClusterId,
				  	  	  	  	  	  	  	  	  	  	  	 bDirectionIsServerToClient,
				  	  	  	  	  	  	  	  	  	  	  	 psDestinationAddress,
				  	  	  	  	  	  	  	  	  	  	  	 pu8TransactionSequenceNumber,
				  	  	  	  	  	  	  	  	  	  	  	 u16AttributeId,
				  	  	  	  	  	  	  	  	  	  	  	 bIsManufacturerSpecific,
				  	  	  	  	  	  	  	  	  	  	  	 u16ManufacturerCode,
				  	  	  	  	  	  	  	  	  	  	  	 u8MaximumNumberOfIdentifiers);

	}
	else
	{
		eStatus = eZCL_SendDiscoverAttributesRequest(u8SourceEndPointId,
													 u8DestinationEndPointId,
	                                                 u16ClusterId,
	                                                 bDirectionIsServerToClient,
	                                                 psDestinationAddress,
	                                                 pu8TransactionSequenceNumber,
	                                                 u16AttributeId,
	                                                 bIsManufacturerSpecific,
	                                                 u16ManufacturerCode,
	                                                 u8MaximumNumberOfIdentifiers);
	}
	return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpSystemServerDiscovery
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpSystemServerDiscovery(uint16 u16ServerMask,uint8* pu8Seq)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpSystemServerDiscoveryReq sSystemServerDiscReq;

            sSystemServerDiscReq.u16ServerMask = u16ServerMask;
            vLog_Printf(TRACE_APP,LOG_DEBUG, "eZdpSystemServerDiscovery Request\n");
            eStatus = ZPS_eAplZdpSystemServerDiscoveryRequest(
                    hAPduInst,
                    pu8Seq,
                    &sSystemServerDiscReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpMgmtNetworkUpdateReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpMgmtNetworkUpdateReq(uint16 u16Addr, uint32 u32ChannelMask, uint8 u8ScanDuration, uint8 u8ScanCount,uint8* pu8Seq, uint16 u16NwkManagerAddr)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpMgmtNwkUpdateReq sMgmtNwkUpdateReq;
            ZPS_tuAddress uDstAddr;
            ZPS_tsNwkNib *psNib = ZPS_psAplZdoGetNib();

            uDstAddr.u16Addr = u16Addr;
            sMgmtNwkUpdateReq.u32ScanChannels = u32ChannelMask;
            sMgmtNwkUpdateReq.u8ScanDuration = u8ScanDuration;
            sMgmtNwkUpdateReq.u8ScanCount = u8ScanCount;
            sMgmtNwkUpdateReq.u8NwkUpdateId = psNib->sPersist.u8UpdateId + 1;
            sMgmtNwkUpdateReq.u16NwkManagerAddr = u16NwkManagerAddr;

            eStatus = ZPS_eAplZdpMgmtNwkUpdateRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sMgmtNwkUpdateReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpMgmtLeave
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpMgmtLeave(uint16 u16DstAddr, uint64 u64DeviceAddr, bool_t bRejoin, bool_t bRemoveChildren,uint8* pu8Seq)
{
        PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpMgmtLeaveReq sMgmtLeaveReq;
            ZPS_tuAddress uDstAddr;

            uDstAddr.u16Addr = u16DstAddr;

            sMgmtLeaveReq.u64DeviceAddress = u64DeviceAddr;
            sMgmtLeaveReq.u8Flags =  bRejoin ? 1 : 0;
            sMgmtLeaveReq.u8Flags |= ((bRemoveChildren ? 1 : 0) << 1);

            eStatus = ZPS_eAplZdpMgmtLeaveRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sMgmtLeaveReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpLeaveReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpLeaveReq(ZPS_tuAddress uAddress, bool bRemoveChildren, bool bRejoin)
{
	if (uAddress.u64Addr != ZPS_NWK_NULL_EXT_ADDR)
	{
		return ZPS_eAplZdoLeaveNetwork(uAddress.u64Addr, bRemoveChildren, bRejoin);
	}

    return ZPS_APL_APS_E_INVALID_PARAMETER;
}

/****************************************************************************
 *
 * NAME: eZdpRemoveDeviceReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpRemoveDeviceReq(ZPS_tuAddress uParentAddress, ZPS_tuAddress uChildAddress)
{
	if (uParentAddress.u64Addr != 0 && uChildAddress.u64Addr != 0)
	{
		return ZPS_eAplZdoRemoveDeviceReq(uParentAddress.u64Addr, uChildAddress.u64Addr);
	}

    return ZPS_APL_APS_E_INVALID_PARAMETER;
}
/****************************************************************************
 *
 * NAME: eZdpNodeDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpNodeDescReq(uint16 u16Addr,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpNodeDescReq sNodeDescReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sNodeDescReq.u16NwkAddrOfInterest = u16Addr;
            eStatus = ZPS_eAplZdpNodeDescRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sNodeDescReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpNodeDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpComplexDescReq(uint16 u16Addr,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpComplexDescReq sComplexDescReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sComplexDescReq.u16NwkAddrOfInterest = u16Addr;
            eStatus = ZPS_eAplZdpComplexDescRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sComplexDescReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpNodeDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpUserDescReq(uint16 u16Addr,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpUserDescReq sUserDescReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sUserDescReq.u16NwkAddrOfInterest = u16Addr;
            eStatus = ZPS_eAplZdpUserDescRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sUserDescReq);
            if (eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpNodeDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpUserSetDescReq(uint16 u16Addr, uint8 u8Length, uint8 * pu8UserDescriptor, uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpUserDescSet sUserDescSet;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sUserDescSet.u16NwkAddrOfInterest = u16Addr;
            sUserDescSet.u8Length = u8Length;
            memcpy(&sUserDescSet.szUserDescriptor, pu8UserDescriptor, u8Length);

            eStatus = ZPS_eAplZdpUserDescSetRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sUserDescSet);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpPowerDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpPowerDescReq(uint16 u16Addr,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpPowerDescReq sPowerDescReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sPowerDescReq.u16NwkAddrOfInterest = u16Addr;
            eStatus = ZPS_eAplZdpPowerDescRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sPowerDescReq);
            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpSimpleDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpSimpleDescReq(uint16 u16Addr, uint8 u8Endpoint, uint8* pu8Seq)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpSimpleDescReq sSimpleDescReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sSimpleDescReq.u16NwkAddrOfInterest = u16Addr;
            sSimpleDescReq.u8EndPoint = u8Endpoint;
            eStatus = ZPS_eAplZdpSimpleDescRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sSimpleDescReq);
            if (eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpActiveEndpointReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpActiveEndpointReq(uint16 u16Addr,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);
        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpActiveEpReq sActiveEpReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Addr;

            sActiveEpReq.u16NwkAddrOfInterest = u16Addr;
            eStatus = ZPS_eAplZdpActiveEpRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8SeqNum,
                    &sActiveEpReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpMatchDescReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpMatchDescReq(uint16 u16Addr,uint16 u16profile,uint8 u8InputCount, uint16* pu16InputList, uint8 u8OutputCount, uint16* pu16OutputList,uint8* pu8SeqNum)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);
        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpMatchDescReq sMatchDescReq;
            ZPS_tuAddress uDstAddr;
            /* always send to node of interest rather than a cache */
           uDstAddr.u16Addr = u16Addr;

            sMatchDescReq.u16NwkAddrOfInterest = u16Addr;
            sMatchDescReq.u16ProfileId = u16profile;
            sMatchDescReq.u8NumInClusters = u8InputCount;
            sMatchDescReq.pu16InClusterList = pu16InputList;
            sMatchDescReq.u8NumOutClusters = u8OutputCount;
            sMatchDescReq.pu16OutClusterList = (u8OutputCount == 0)? NULL : pu16OutputList;
            eStatus = ZPS_eAplZdpMatchDescRequest(
                        hAPduInst,
                        uDstAddr,
                        FALSE,
                        pu8SeqNum,
                        &sMatchDescReq);

            if (eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }

        }

       return eStatus;
}

/****************************************************************************
 *
 * NAME: eZdpIeeeAddrReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpIeeeAddrReq(uint16 u16Dst,
        uint16 u16Addr,
        uint8 u8RequestType,
        uint8 u8StartIndex,uint8* pu8Seq)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpIeeeAddrReq sAplZdpIeeeAddrReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Dst;
            sAplZdpIeeeAddrReq.u16NwkAddrOfInterest = u16Addr;
            sAplZdpIeeeAddrReq.u8RequestType = u8RequestType;
            sAplZdpIeeeAddrReq.u8StartIndex = u8StartIndex;
            eStatus = ZPS_eAplZdpIeeeAddrRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sAplZdpIeeeAddrReq);
            if (eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;

}

/****************************************************************************
 *
 * NAME: eZdpNwkAddrReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpNwkAddrReq(uint16 u16Dst,
        uint64 u64Addr,
        uint8 u8RequestType,
        uint8 u8StartIndex,
        uint8* pu8Seq)
{
    PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpNwkAddrReq sAplZdpNwkAddrReq;
            ZPS_tuAddress uDstAddr;

            /* always send to node of interest rather than a cache */
            uDstAddr.u16Addr = u16Dst;
            sAplZdpNwkAddrReq.u64IeeeAddr = u64Addr;
            sAplZdpNwkAddrReq.u8RequestType = u8RequestType;
            sAplZdpNwkAddrReq.u8StartIndex = u8StartIndex;
            eStatus = ZPS_eAplZdpNwkAddrRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sAplZdpNwkAddrReq);

            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }

        return eStatus;

}
/****************************************************************************
 *
 * NAME: eZdpPermitJoiningReq
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpPermitJoiningReq(uint16 u16DstAddr,
        uint8 u8PermitDuration,
        bool bTcSignificance,uint8* pu8Seq)
{


    ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        if(u16DstAddr != ZPS_u16NwkNibGetNwkAddr(ZPS_pvAplZdoGetNwkHandle()))
        {
            PDUM_thAPduInstance hAPduInst;

            hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);
            if (PDUM_INVALID_HANDLE != hAPduInst) {
                ZPS_tsAplZdpMgmtPermitJoiningReq sAplZdpMgmtPermitJoiningReq;
                ZPS_tuAddress uDstAddr;

                /* always send to node of interest rather than a cache */
                uDstAddr.u16Addr = u16DstAddr;
                sAplZdpMgmtPermitJoiningReq.u8PermitDuration = u8PermitDuration;
                sAplZdpMgmtPermitJoiningReq.bTcSignificance = bTcSignificance;

                eStatus = ZPS_eAplZdpMgmtPermitJoiningRequest(
                        hAPduInst,
                        uDstAddr,
                        FALSE,
                        pu8Seq,
                        &sAplZdpMgmtPermitJoiningReq);
            }
            if(eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }
        else
        {
            eStatus = ZPS_eAplZdoPermitJoining(u8PermitDuration);
        }

        return eStatus;
}

/****************************************************************************
 *
 * NAME: eBindEntry
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PRIVATE ZPS_teStatus eBindUnbindEntry(
        bool_t        bBind,
        uint64       u64SrcAddr,
        uint8        u8SrcEndpoint,
        uint16       u16ClusterId,
        ZPS_tuAddress *puDstAddress,
        uint8        u8DstEndpoint,
        uint8        u8DstAddrMode,
        uint8* pu8Seq)
{
    ZPS_teStatus eReturnCode = ZPS_APL_APS_E_INVALID_PARAMETER;
    ZPS_tuAddress uAddr;
    ZPS_tsAplZdpBindUnbindReq sAplZdpBindReq;

    if(u8DstAddrMode == 0x1)
    {
        sAplZdpBindReq.uAddressField.sShort.u16DstAddress = uAddr.u16Addr = puDstAddress->u16Addr;
    }
    else if (u8DstAddrMode == 0x03)
    {
        u8DstAddrMode = 0x3;
        uAddr.u64Addr = puDstAddress->u64Addr;
        sAplZdpBindReq.uAddressField.sExtended.u64DstAddress = puDstAddress->u64Addr;
        sAplZdpBindReq.uAddressField.sExtended.u8DstEndPoint = u8DstEndpoint;
    }
    else
    {
    	return eReturnCode;
    }

    if (ZPS_u64NwkNibGetExtAddr(ZPS_pvAplZdoGetNwkHandle()) == u64SrcAddr)
    {
        if(bBind)
        {
            eReturnCode = ZPS_eAplZdoBind(u16ClusterId, u8SrcEndpoint, puDstAddress->u16Addr,puDstAddress->u64Addr,u8DstEndpoint );
        }
        else
        {
            eReturnCode = ZPS_eAplZdoUnbind(u16ClusterId, u8SrcEndpoint,  puDstAddress->u16Addr,puDstAddress->u64Addr,u8DstEndpoint );
        }
	}
    else
    {
        PDUM_thAPduInstance hAPduInst;

                hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

                if (PDUM_INVALID_HANDLE != hAPduInst)
                {
                    ZPS_tuAddress uDstAddr;
                    /* always send to node of interest rather than a cache */
                    uDstAddr.u64Addr = u64SrcAddr;
                    sAplZdpBindReq.u64SrcAddress = u64SrcAddr;
                    sAplZdpBindReq.u8SrcEndpoint = u8SrcEndpoint;
                    sAplZdpBindReq.u16ClusterId = u16ClusterId;
                    sAplZdpBindReq.u8DstAddrMode = u8DstAddrMode;

                    if(bBind)
                    {
                        eReturnCode = ZPS_eAplZdpBindUnbindRequest(
                            hAPduInst,
                            uDstAddr,
                            TRUE,
                            pu8Seq,
                            TRUE,
                            &sAplZdpBindReq);
                    }
                    else
                    {
                        eReturnCode = ZPS_eAplZdpBindUnbindRequest(
                            hAPduInst,
                            uDstAddr,
                            TRUE,
                            pu8Seq,
                            FALSE,
                            &sAplZdpBindReq);
                    }
                    if(eReturnCode != ZPS_E_SUCCESS)
                    {
                        PDUM_eAPduFreeAPduInstance(hAPduInst);
                    }
                }
            }
            return eReturnCode;
}

/****************************************************************************
 *
 * NAME: vControlNodeScanStart
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/

PRIVATE void vControlNodeScanStart(void)
{
    if(sZllState.eState == FACTORY_NEW)
    {   /* factory new start up */
        if(sDeviceDesc.u8DeviceType == ZPS_ZDO_DEVICE_COORD)
        {
            ZPS_eAplZdoStartStack();
            sZllState.eState = NOT_FACTORY_NEW;
            sDeviceDesc.eNodeState = E_RUNNING;
            PDM_eSaveRecordData( PDM_ID_APP_ZLL_CMSSION,&sZllState,sizeof(tsZllState));
            PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
        }
        else
        {
            ZPS_eAplZdoDiscoverNetworks( ZLL_CHANNEL_MASK );
            sDeviceDesc.eNodeState = E_DISCOVERY;
            PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
        }

    }
    else
    {
        if(sDeviceDesc.u8DeviceType == ZPS_ZDO_DEVICE_COORD)
        {
            ZPS_eAplZdoStartStack();
            sZllState.eState = NOT_FACTORY_NEW;
            sDeviceDesc.eNodeState = E_RUNNING;
        }
        else
        {
            ZPS_eAplZdoZllStartRouter();
        }
    }
#ifdef APP_DISCOVERY_ONLY_MODE_ENABLED
    ZPS_bAppDiscoveryComplete();
#endif
}

#ifdef APP_DISCOVERY_ONLY_MODE_ENABLED
/****************************************************************************
 *
 * NAME: vScanDiscoveryOnly
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/

PRIVATE void vScanDiscoveryOnly(void)
{
        if(sDeviceDesc.u8DeviceType == ZPS_ZDO_DEVICE_COORD)
        {
            ZPS_eAplZdoStartStack();
            sDeviceDesc.eNodeState = E_RUNNING;
            PDM_eSaveRecordData( PDM_ID_APP_ZLL_CMSSION,&sZllState,sizeof(tsZllState));
            PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
        }
        else
        {
            ZPS_eAplZdoDiscoverNetworks( ZLL_CHANNEL_MASK );
            sDeviceDesc.eNodeState = E_DISCOVERY;
            PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
        }
    ZPS_bAppDiscoveryReceived();
}

/****************************************************************************
 *
 * NAME: vScanDiscoveryOnlyResponse
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/

PUBLIC void vScanDiscoveryOnlyResponse(MAC_MlmeDcfmInd_s* psBeaconIndication)
{
    uint8 au8StatusBuffer[256];
    uint8* pu8Buffer;
    uint8 u16Length;
    uint32 u32HexChan;
    uint32 u32Remainder;

    if (u32ChannelMask > 0)
    {
        u32HexChan = u32ChannelMask;

        int count = 0;

        while (u32HexChan > 15) //Convert Channel Mask to hex
        {
            u32HexChan = u32HexChan/16;
            u32Remainder = u32Remainder * 10;
            u32Remainder = u32Remainder + (u32HexChan % 2);
            count++;
        }

        while (count>0)
        {
            u32HexChan = u32HexChan * 10;
            count--;
        }
        int i = u32HexChan;
        count = 0;

        while ((i % 10) == 0) //Get 0's
        {
            i = i/10;
            count ++;
        }
        count = count * 4; //Count up to first number that isn't 0

        switch(i) //Count up to actual channel number
        {
        case 8:
        {
            count = count + 3;
            u32Channel = count;
        }
        break;
        case 4:
        {
            count = count + 2;
            u32Channel = count;
        }
        break;
        case 2:
        {
            count = count + 1;
            u32Channel = count;
        }
        break;
        case 1:
        {
            u32Channel = count;
        }
        break;
        default:
        break;
        }
    }

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\tINCOMING CHANNEL = %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LogicalChan);
    vLog_Printf(TRACE_APP, LOG_DEBUG, "\tCHANNEL = %d", u32Channel);
    vLog_Printf(TRACE_APP, LOG_DEBUG, "\tCHANNEL MASK = %d\n", u32HexChan);

    if (psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LogicalChan == u32Channel || u32Channel == 0)    //If no channel selected or channel is selected
    {
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nAddress Mode: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.u8AddrMode);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nPAN ID: %x", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.u16PanId);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nCoordinator Short Address: %04x", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.uAddr.u16Short);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nLogical Channel: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LogicalChan);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nGTS Permit: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8GtsPermit);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nLink Quality: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LinkQuality);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nSecurity: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8SecurityUse);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nACL Security Mode: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8AclEntry);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nSecurity Failure: %d", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8SecurityFailure);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nSuperframe Specification: %x", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u16SuperframeSpec);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nTimestamp: %x", psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u32TimeStamp);

        vLog_Printf(TRACE_APP, LOG_DEBUG, "\t\nExtended PAN ID: ");

        int i = 0;
        for (i=10;i>2;i--)
        {
            vLog_Printf(TRACE_APP, LOG_DEBUG, "%02x", psBeaconIndication->uParam.sIndBeacon.u8SDU[i]);
        }

        vLog_Printf(TRACE_APP, LOG_DEBUG, "\n");

        //copy MAC_PanDescr_s
        pu8Buffer = au8StatusBuffer;
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.u8AddrMode,sizeof(uint8));       //0
        pu8Buffer += sizeof(uint8);
        u16Length  = sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.u16PanId,sizeof(uint16));        //1
        pu8Buffer += sizeof(uint16);
        u16Length += sizeof(uint16);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.sCoord.uAddr.u16Short,sizeof(uint16));  //3
        pu8Buffer += sizeof(uint16);
        u16Length += sizeof(uint16);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LogicalChan,sizeof(uint8));           //5
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8GtsPermit,sizeof(uint8));             //6
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8LinkQuality,sizeof(uint8));           //7
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8SecurityUse,sizeof(uint8));           //8
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8AclEntry,sizeof(uint8));              //9
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u8SecurityFailure,sizeof(uint8));       //10
        pu8Buffer += sizeof(uint8);
        u16Length += sizeof(uint8);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u16SuperframeSpec,sizeof(uint16));      //11
        pu8Buffer += sizeof(uint16);
        u16Length += sizeof(uint16);
        memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.sPANdescriptor.u32TimeStamp,sizeof(uint32));           //13
        pu8Buffer += sizeof(uint32);
        u16Length += sizeof(uint32);

        for (i=10;i>2;i--)
        {
            memcpy(pu8Buffer, &psBeaconIndication->uParam.sIndBeacon.u8SDU[i],sizeof(uint8));                           //17
            pu8Buffer += sizeof(uint8);
            u16Length += sizeof(uint8);
        }

        vSL_WriteMessage(E_SL_MSG_START_DISCOVERY_ONLY_RESPONSE, u16Length, au8StatusBuffer,0);
    }
}
#endif

/****************************************************************************
 *
 * NAME: vControlNodeStartNetwork
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/

PRIVATE void vControlNodeStartNetwork(void)
{
    if(sZllState.eState == FACTORY_NEW)
    {
        if(sDeviceDesc.u8DeviceType == ZPS_ZDO_DEVICE_COORD)
        {
            ZPS_eAplZdoStartStack();
            sZllState.eState = NOT_FACTORY_NEW;
            PDM_eSaveRecordData( PDM_ID_APP_ZLL_CMSSION,&sZllState,sizeof(tsZllState));

        }
        else
        {
            if(sDeviceDesc.eNodeState != E_RUNNING )
            {
                APP_tsEvent sAppEvent;
                sAppEvent.eType = APP_E_EVENT_START_ROUTER;
                OS_ePostMessage(APP_msgEvents, &sAppEvent);

            }
        }
    }
}


/****************************************************************************
 *
 * NAME: APP_vConfigureDevice
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/
PUBLIC void APP_vConfigureDevice(uint8 u8DeviceType)
{
    ZPS_vNwkSetDeviceType(ZPS_pvAplZdoGetNwkHandle(),(u8DeviceType+1)); /* coordinator is 1 - ED is 3 Router is 2*/
    ZPS_vSetZdoDeviceType(u8DeviceType); /* coordinator is 0 - ED is 2 and Router is 1 */
}

/****************************************************************************
 *
 * NAME: vForceStartRouter
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/
PUBLIC void vForceStartRouter(uint8* pu8Buffer)
{

    ZPS_tsNwkNib *psNib = ZPS_psAplZdoGetNib();
    uint8 i;
    uint64 u64Panid;
    ZPS_tsAplAib *psAib = ZPS_psAplAibGetAib();

    /* Generate a new network key */
    for (i=0; i<ZPS_SEC_KEY_LENGTH; i++)
    {
#if RAND_KEY
        psNib->sTbl.psSecMatSet[0].au8Key[i] = (uint8)(RND_u32GetRand256() & 0xFF);
#else
        psNib->sTbl.psSecMatSet[0].au8Key[i] = 0xbb;
#endif
    }
    if(psAib->u64ApsUseExtendedPanid == 0)
    {
        u64Panid = RND_u32GetRand(1, 0xffffffff);
        u64Panid <<= 32;
        u64Panid |= RND_u32GetRand(0, 0xffffffff);
    }
    else
    {
        u64Panid = psAib->u64ApsUseExtendedPanid;
    }
    ZPS_vNwkNibSetExtPanId(ZPS_pvAplZdoGetNwkHandle(), u64Panid);
    /* Add Key data into the NIB in first Key entry */
    ZPS_vNwkNibSetKeySeqNum(ZPS_pvAplZdoGetNwkHandle(), 0);
    psNib->sTbl.psSecMatSet[0].u8KeySeqNum = 0;
    psNib->sTbl.u32OutFC = 0;
    memset(psNib->sTbl.pu32InFCSet, 0, (sizeof(uint32) * psNib->sTblSize.u16NtActv));
    psNib->sTbl.psSecMatSet[0].u8KeyType = ZPS_NWK_SEC_NETWORK_KEY;

    /* save security material to flash */
    ZPS_vNwkSaveSecMat(ZPS_pvAplZdoGetNwkHandle());
    if(u32ChannelMask == 0 || (u32ChannelMask > 26))
    {
        ZPS_vNwkNibSetChannel( ZPS_pvAplZdoGetNwkHandle(), DEFAULT_CHANNEL);
        sZllState.u8MyChannel = DEFAULT_CHANNEL;
    }
    else
    {
        ZPS_vNwkNibSetChannel( ZPS_pvAplZdoGetNwkHandle(), (uint8)u32ChannelMask);
        sZllState.u8MyChannel = (uint8)u32ChannelMask;
    }
    sZllState.u16MyAddr = 0x0001;
    sZllState.u16FreeAddrLow += 1;
    ZPS_vNwkNibSetNwkAddr( ZPS_pvAplZdoGetNwkHandle(), sZllState.u16MyAddr);
    /* take our groups, adjust free range  */
    vSetGroupAddress(sZllState.u16FreeGroupLow, GROUPS_REQUIRED);
    sZllState.u16FreeGroupLow += GROUPS_REQUIRED;
    ZPS_eAplAibSetApsTrustCenterAddress( 0xffffffffffffffffULL );
    vStartAndAnnounce();
    sZllState.eState = NOT_FACTORY_NEW;
    sDeviceDesc.eNodeState = E_RUNNING;
    vSendJoinedFormEventToHost(1,pu8Buffer);

}
/****************************************************************************
 *
 * NAME: bSendHATransportKey
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/
PUBLIC bool bSendHATransportKey(uint16 u16ShortAddr, uint64 u64DeviceAddress)
{
    bool_t bStatus = TRUE;
    ZPS_teStatus eStatus = ZPS_EVENT_ERROR;
    if(bBlackListEnable)
    {
        ZPS_teDevicePermissions eDevicePermissions;

        OS_eExitCriticalSection(mutexZPS);
        eStatus = ZPS_bAplZdoTrustCenterGetDevicePermissions(u64DeviceAddress, &eDevicePermissions);
        OS_eEnterCriticalSection(mutexZPS);
        if(eStatus == ZPS_E_SUCCESS)
        {
            if(eDevicePermissions != ZPS_DEVICE_PERMISSIONS_ALL_PERMITED)
            {
                bStatus = FALSE;
            }
            else
            {
                bStatus = TRUE;
            }
        }
        else
        {
            bStatus = FALSE;
        }
        if(sDeviceDesc.u8DeviceType >= 2 && eStatus == ZPS_E_SUCCESS &&
                eDevicePermissions == ZPS_DEVICE_PERMISSIONS_ALL_PERMITED)
        {
            u64CallbackMacAddress = u64DeviceAddress;
            OS_eStartSWTimer(App_HAModeTimer,APP_TIME_MS(500),&u64CallbackMacAddress);
        }
    }
    else
    {
        if(sDeviceDesc.u8DeviceType >= 2)
        {
            u64CallbackMacAddress = u64DeviceAddress;
            OS_eStartSWTimer(App_HAModeTimer,APP_TIME_MS(500),&u64CallbackMacAddress);
        }
    }

    return bStatus;
}

/****************************************************************************
 *
 * NAME: vSendJoinedFormEventToHost
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:
 *
 * RETURNS:
 *
 * NOTES:
 *
 ****************************************************************************/
PUBLIC void vSendJoinedFormEventToHost(uint8 u8FormJoin, uint8 *pu8Buffer)
{
    uint16 u16NwkAddr ;
    uint64 u64IeeeAddr ;
    uint8 *pu8BufferCpy = pu8Buffer;
    static bool_t bReportSent = FALSE;

    if (bReportSent)
    {
        return;
    }
    bReportSent = TRUE;
    u16NwkAddr = ZPS_u16NwkNibGetNwkAddr(ZPS_pvAplZdoGetNwkHandle());
    u64IeeeAddr = ZPS_u64NwkNibGetExtAddr(ZPS_pvAplZdoGetNwkHandle());
    eAppApiPlmeGet(PHY_PIB_ATTR_CURRENT_CHANNEL, &u32Channel);
    *pu8BufferCpy = u8FormJoin;
    pu8BufferCpy+=sizeof(uint8);
    memcpy(pu8BufferCpy,(uint8*)&u16NwkAddr,sizeof(uint16));
    pu8BufferCpy+=sizeof(uint16);
    memcpy(pu8BufferCpy,(uint8*)&u64IeeeAddr,sizeof(uint64));
    pu8BufferCpy+=sizeof(uint64);
    *pu8BufferCpy = (uint8)u32Channel;
    ZPS_vSaveAllZpsRecords();
    ZNC_vSaveAllRecords();
    vSL_WriteMessage(E_SL_MSG_NETWORK_JOINED_FORMED, (sizeof(uint8)+sizeof(uint16)+sizeof(uint64)+sizeof(uint8)), pu8Buffer,0);
    OS_eStartSWTimer(APP_tmrToggleLED, s_sLedState.u32LedToggleTime, &s_sLedState);


}


/****************************************************************************
 *
 * NAME: vFactoryResetRecords
 *
 * DESCRIPTION: reset application and stack to factory new state
 *              preserving the outgoing nwk frame counter
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vFactoryResetRecords( void)
{
    void *pvNwk = ZPS_pvAplZdoGetNwkHandle();
    ZPS_tsNwkNib *psNib = ZPS_psNwkNibGetHandle( pvNwk);
    int i;

    ZPS_vNwkNibClearTables(pvNwk);
    (*(ZPS_tsNwkNibInitialValues *)psNib) = *(psNib->sTbl.psNibDefault);

   /* Clear Security Material Set - will set all 64bit addresses to ZPS_NWK_NULL_EXT_ADDR */
   /* Check if we have a frame counter for this address */
   for (i = 0; i < psNib->sTblSize.u8SecMatSet; i++)
   {
        /* Save frame counter array first */
        memset(&psNib->sTbl.psSecMatSet[i], 0, sizeof(ZPS_tsNwkSecMaterialSet));
   }

    /* Set default values in Material Set Table */
    for (i = 0; i < psNib->sTblSize.u8SecMatSet; i++)
    {
        psNib->sTbl.psSecMatSet[i].u8KeyType = ZPS_NWK_SEC_KEY_INVALID_KEY;
    }
    memset(psNib->sTbl.pu32InFCSet, 0, (sizeof(uint32)*psNib->sTblSize.u16NtActv));
    psNib->sPersist.u64ExtPanId = ZPS_NWK_NULL_EXT_PAN_ID;
    psNib->sPersist.u16NwkAddr = 0xffff;
    psNib->sPersist.u8ActiveKeySeqNumber = 0xff;
    psNib->sPersist.u16VsParentAddr = 0xffff;

    /* put the old frame counter back */
    psNib->sTbl.u32OutFC = u32OldFrameCtr+10;
    /* set the aps use pan id */
    ZPS_eAplAibSetApsUseExtendedPanId( 0 );
    ZPS_eAplAibSetApsTrustCenterAddress( 0 );

    sDeviceDesc.eNodeState = E_STARTUP;

    sZllState.eState = FACTORY_NEW;
    sZllState.u8MyChannel = ZLL_SKIP_CH1;
    sZllState.u16MyAddr= ZLL_MIN_ADDR;
    sZllState.u16FreeAddrLow = ZLL_MIN_ADDR;
    sZllState.u16FreeAddrHigh = ZLL_MAX_ADDR;
    sZllState.u16FreeGroupLow = ZLL_MIN_GROUP;
    sZllState.u16FreeGroupHigh = ZLL_MAX_GROUP;

    memset(&sEndpointTable, 0 , sizeof(tsZllEndpointInfoTable));
    memset(&sGroupTable, 0, sizeof(tsZllGroupInfoTable));

    ZPS_eAplAibSetApsUseExtendedPanId( 0);

    if(sDeviceDesc.u8DeviceType >= 2 )
    {
        APP_vConfigureDevice(1); /* configure it in HA compatibility mode */
    }
    else
    {
        APP_vConfigureDevice(sDeviceDesc.u8DeviceType);
    }

    ZPS_vSaveAllZpsRecords();
    ZNC_vSaveAllRecords();

    bResetIssued = TRUE;
    OS_eStartSWTimer(APP_IdTimer, APP_TIME_MS(1), NULL);

}

/****************************************************************************
 **
 ** NAME:       eZnc_SendWriteAttributesRequest
 **
 **
 ****************************************************************************/
PUBLIC  teZCL_Status  eZnc_SendWriteAttributesRequest(
                    uint8                       u8SourceEndPointId,
                    uint8                       u8DestinationEndPointId,
                    uint16                      u16ClusterId,
                    bool_t                      bDirectionIsServerToClient,
                    tsZCL_Address              *psDestinationAddress,
                    uint8                      *pu8TransactionSequenceNumber,
                    bool_t                      bIsManufacturerSpecific,
                    uint16                      u16ManufacturerCode,
                    uint8                       *pu8AttributeRequestList,
                    uint8                       u8NumberOfAttrib,
                    uint16						u16SizePayload)
{
    ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

    uint32 i;
    uint32 u32PdumPayloadSize = 0;

    uint16 u16offset,u16AttribId;
    PDUM_thAPduInstance myPDUM_thAPduInstance;
    uint8 u8FramControl = 0,u8CommandId = 0x02, *pu8Data;
    uint32 u16Size;
    // handle sequence number pass present value back to user
    *pu8TransactionSequenceNumber = u8GetTransactionSequenceNumber();

    // get buffer
    myPDUM_thAPduInstance = PDUM_hAPduAllocateAPduInstance(apduZDP);

    if(myPDUM_thAPduInstance == PDUM_INVALID_HANDLE)
    {
        return(E_ZCL_ERR_ZBUFFER_FAIL);
    }

    if (bIsManufacturerSpecific)
    {
    	u32PdumPayloadSize = (5+u16SizePayload);
    }
    else
    {
    	u32PdumPayloadSize = (3+u16SizePayload);
    }

    if(PDUM_E_OK != PDUM_eAPduInstanceSetPayloadSize(myPDUM_thAPduInstance,(u32PdumPayloadSize)))
    {
        return(E_ZCL_ERR_ZBUFFER_FAIL);
    }

    u8FramControl = (0x00) | (((bIsManufacturerSpecific)? 1 : 0) << 2) |
                        (((bDirectionIsServerToClient)? 1 : 0) << 3) | (1 << 4);
    // write command header

    if(bIsManufacturerSpecific)
    {
    	u16offset = PDUM_u16APduInstanceWriteNBO(myPDUM_thAPduInstance,0,"bhbb",u8FramControl,u16ManufacturerCode,*pu8TransactionSequenceNumber,u8CommandId);
    }
    else
    {
    	u16offset = PDUM_u16APduInstanceWriteNBO(myPDUM_thAPduInstance,0,"bbb",u8FramControl,*pu8TransactionSequenceNumber,u8CommandId);
    }

    DBG_vPrintf(TRUE, "\nWrite Attribute Payload Size = %d", u16SizePayload);

    i = 0;
    pu8Data = (uint8*)PDUM_pvAPduInstanceGetPayload(myPDUM_thAPduInstance);
    while( i < u16SizePayload)
    {
        memcpy(&u16AttribId,&pu8AttributeRequestList[i],sizeof(uint16));
        u16offset += PDUM_u16APduInstanceWriteNBO(myPDUM_thAPduInstance,u16offset,"h", u16AttribId);
        u16offset += PDUM_u16APduInstanceWriteNBO(myPDUM_thAPduInstance,u16offset,"b", pu8AttributeRequestList[i+2]);
        u16Size = u16GetAttributeActualSize(pu8AttributeRequestList[i+2],1);
        u16offset += u16ZncWriteDataPattern(&pu8Data[u16offset],pu8AttributeRequestList[i+2],&pu8AttributeRequestList[i+3],u16Size);

        DBG_vPrintf(TRUE, "\nWrite Attribute Size = %d, Offset = %d", u16Size, u16offset);
        i = i+u16Size+3;
    }

    // transmit the request
        eStatus = ZPS_eAplAfUnicastDataReq(myPDUM_thAPduInstance,
                                           u16ClusterId,
                                           u8SourceEndPointId,
                                           u8DestinationEndPointId,
                                           psDestinationAddress->uAddress.u16DestinationAddress,
                                           ZPS_E_APL_AF_SECURE_NWK, 0, NULL);

        if (eStatus != ZPS_E_SUCCESS)
        {
            PDUM_eAPduFreeAPduInstance(myPDUM_thAPduInstance);
        }

        return eStatus;
}


/****************************************************************************
 **
 ** NAME:       eZdpMgmtLqiRequest
 **
 **
 ****************************************************************************/
PRIVATE ZPS_teStatus eZdpMgmtLqiRequest(uint16 u16Addr, uint8  u8StartIndex,uint8* pu8Seq)
{
        PDUM_thAPduInstance hAPduInst;
        ZPS_teStatus eStatus = ZPS_EVENT_ERROR;

        hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZDP);

        if (PDUM_INVALID_HANDLE != hAPduInst) {
            ZPS_tsAplZdpMgmtLqiReq sMgmtLqiReq;
            ZPS_tuAddress uDstAddr;

            uDstAddr.u16Addr = u16Addr;

            sMgmtLqiReq.u8StartIndex = u8StartIndex;

            DBG_vPrintf(TRACE_APP, "\nManagement Lqi Request");

            eStatus = ZPS_eAplZdpMgmtLqiRequest(
                    hAPduInst,
                    uDstAddr,
                    FALSE,
                    pu8Seq,
                    &sMgmtLqiReq);
            if (eStatus != ZPS_E_SUCCESS)
            {
                PDUM_eAPduFreeAPduInstance(hAPduInst);
            }
        }
        return eStatus;
}

/****************************************************************************
 *
 * NAME:
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
OS_TASK(APP_ID_Task)
{
    /* interpan id time out, shut down */
    OS_eStopSWTimer(APP_IdTimer);
    if(bResetIssued)
    {
        vAHI_SwReset();
        bResetIssued = FALSE;
    }
    //APP_vSetLeds(FALSE, FALSE);
}

/****************************************************************************
 **
 ** NAME:       ZNC_vSaveAllRecords
 **
 **
 ****************************************************************************/
void ZNC_vSaveAllRecords(void)
{
    PDM_eSaveRecordData( PDM_ID_APP_ZLL_CMSSION,&sZllState,sizeof(tsZllState));
    PDM_eSaveRecordData( PDM_ID_APP_CONTROL_BRIDGE,&sDeviceDesc,sizeof(tsDeviceDesc));
    PDM_eSaveRecordData( PDM_ID_APP_END_P_TABLE,&sEndpointTable,sizeof(tsZllEndpointInfoTable));
    PDM_eSaveRecordData( PDM_ID_APP_GROUP_TABLE,&sGroupTable,sizeof(tsZllGroupInfoTable));
    vSaveScenesNVM();
}

/****************************************************************************
 **
 ** NAME:       eZCL_GetLastSequenceNumber
 **
 ** DESCRIPTION:
 ** Returns the last Sequence Number.
 **
 ** PARAMETERS:            Name                           Usage
 **
 ** RETURN:
 ** uint8
 **
 ****************************************************************************/
PUBLIC uint8 eZCL_GetLastSequenceNumber(void)
{
    return psZCL_Common->u8ApsSequenceNumberOfLastTransmit;
}


/***        END OF FILE                                                   ***/
/****************************************************************************/
