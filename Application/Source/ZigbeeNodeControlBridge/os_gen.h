/****************************************************************************
 *
 *                 THIS IS A GENERATED FILE. DO NOT EDIT!
 *
 * MODULE:         OS
 *
 * COMPONENT:      os_gen.h
 *
 * DATE:           Mon Jul  1 16:54:39 2019
 *
 * AUTHOR:         Jennic RTOS Configuration Tool
 *
 * DESCRIPTION:    RTOS Application Configuration
 *
 ****************************************************************************
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
 * Copyright NXP B.V. 2017. All rights reserved
 ****************************************************************************/

#ifndef _OS_GEN_H
#define _OS_GEN_H

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Module ZBPro */

/* Mutex Handles */
#define mutexZPS ((OS_thMutex)&os_Mutex_mutexZPS)
#define mutexPDUM ((OS_thMutex)&os_Mutex_mutexPDUM)
#define mutexMAC ((OS_thMutex)&os_Mutex_mutexMAC)
#define mutexFLASH ((OS_thMutex)&os_Mutex_mutexFLASH)
#define mutexPDM ((OS_thMutex)&os_Mutex_mutexPDM)

/* Message Handles */
#define zps_msgMlmeDcfmInd ((OS_thMessage)&os_Message_zps_msgMlmeDcfmInd)
#define zps_msgTimeEvents ((OS_thMessage)&os_Message_zps_msgTimeEvents)
#define zps_msgMcpsDcfmInd ((OS_thMessage)&os_Message_zps_msgMcpsDcfmInd)
#define zps_msgMcpsDcfm ((OS_thMessage)&os_Message_zps_msgMcpsDcfm)
#define zps_msgMlmeDcfmInd_C_Type MAC_tsMlmeVsDcfmInd
#define zps_msgTimeEvents_C_Type zps_tsTimeEvent
#define zps_msgMcpsDcfmInd_C_Type MAC_tsMcpsVsDcfmInd
#define zps_msgMcpsDcfm_C_Type MAC_tsMcpsVsCfmData

/* Module Zigbee_Node_Control_Bridge */

/* Cooperative Task Handles */
#define APP_taskZncb ((OS_thTask)&os_Task_APP_taskZncb)
#define APP_CommissionTimerTask ((OS_thTask)&os_Task_APP_CommissionTimerTask)
#define ZCL_Task ((OS_thTask)&os_Task_ZCL_Task)
#define APP_ID_Task ((OS_thTask)&os_Task_APP_ID_Task)
#define APP_Commission_Task ((OS_thTask)&os_Task_APP_Commission_Task)
#define zps_taskZPS ((OS_thTask)&os_Task_zps_taskZPS)
#define APP_taskAtParser ((OS_thTask)&os_Task_APP_taskAtParser)
#define APP_AgeOutChildren ((OS_thTask)&os_Task_APP_AgeOutChildren)

/* Mutex Handles */
#define APP_mutexUart ((OS_thMutex)&os_Mutex_APP_mutexUart)

/* Message Handles */
#define APP_msgZpsEvents ((OS_thMessage)&os_Message_APP_msgZpsEvents)
#define APP_msgSerialTx ((OS_thMessage)&os_Message_APP_msgSerialTx)
#define APP_msgSerialRx ((OS_thMessage)&os_Message_APP_msgSerialRx)
#define APP_CommissionEvents ((OS_thMessage)&os_Message_APP_CommissionEvents)
#define APP_msgEvents ((OS_thMessage)&os_Message_APP_msgEvents)
#define APP_msgZpsEvents_ZCL ((OS_thMessage)&os_Message_APP_msgZpsEvents_ZCL)
#define APP_msgZpsEvents_C_Type ZPS_tsAfEvent
#define APP_msgSerialTx_C_Type uint8
#define APP_msgSerialRx_C_Type uint8
#define APP_CommissionEvents_C_Type APP_CommissionEvent
#define APP_msgEvents_C_Type APP_tsEvent
#define APP_msgZpsEvents_ZCL_C_Type ZPS_tsAfEvent

/* Timer Handles */
#define APP_cntrTickTimer ((OS_thHWCounter)&os_HWCounter_APP_cntrTickTimer)
#define APP_tmrToggleLED ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_APP_tmrToggleLED)
#define APP_CommissionTimer ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_APP_CommissionTimer)
#define APP_TickTimer ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_APP_TickTimer)
#define APP_IdTimer ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_APP_IdTimer)
#define App_HAModeTimer ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_App_HAModeTimer)
#define APP_AgeOutChildrenTmr ((OS_thSWTimer)&os_SWTimer_APP_cntrTickTimer_APP_AgeOutChildrenTmr)

/* Module Exceptions */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef void (*OS_tprISR)(void);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/


/* Mutex Handles */
extern struct _os_tsMutex os_Mutex_mutexZPS;
extern struct _os_tsMutex os_Mutex_mutexPDUM;
extern struct _os_tsMutex os_Mutex_mutexMAC;
extern struct _os_tsMutex os_Mutex_mutexFLASH;
extern struct _os_tsMutex os_Mutex_mutexPDM;

/* Message Handles */
extern struct _os_tsMessage os_Message_zps_msgMlmeDcfmInd;
extern struct _os_tsMessage os_Message_zps_msgTimeEvents;
extern struct _os_tsMessage os_Message_zps_msgMcpsDcfmInd;
extern struct _os_tsMessage os_Message_zps_msgMcpsDcfm;

/* Cooperative Task Handles */
extern struct _os_tsTask os_Task_APP_taskZncb;
extern struct _os_tsTask os_Task_APP_CommissionTimerTask;
extern struct _os_tsTask os_Task_ZCL_Task;
extern struct _os_tsTask os_Task_APP_ID_Task;
extern struct _os_tsTask os_Task_APP_Commission_Task;
extern struct _os_tsTask os_Task_zps_taskZPS;
extern struct _os_tsTask os_Task_APP_taskAtParser;
extern struct _os_tsTask os_Task_APP_AgeOutChildren;

/* Mutex Handles */
extern struct _os_tsMutex os_Mutex_APP_mutexUart;

/* Message Handles */
extern struct _os_tsMessage os_Message_APP_msgZpsEvents;
extern struct _os_tsMessage os_Message_APP_msgSerialTx;
extern struct _os_tsMessage os_Message_APP_msgSerialRx;
extern struct _os_tsMessage os_Message_APP_CommissionEvents;
extern struct _os_tsMessage os_Message_APP_msgEvents;
extern struct _os_tsMessage os_Message_APP_msgZpsEvents_ZCL;

/* Timer Handles */
extern struct _os_tsHWCounter os_HWCounter_APP_cntrTickTimer;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_APP_tmrToggleLED;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_APP_CommissionTimer;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_APP_TickTimer;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_APP_IdTimer;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_App_HAModeTimer;
extern struct _os_tsSWTimer os_SWTimer_APP_cntrTickTimer_APP_AgeOutChildrenTmr;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void OS_vStart(void (*)(void), void (*)(void), void (*)(OS_teStatus , void *));

PUBLIC bool os_bAPP_cbSetTickTimerCompare(uint32 );
PUBLIC uint32 os_u32APP_cbGetTickTimer(void);
PUBLIC void os_vAPP_cbEnableTickTimer(void);
PUBLIC void os_vAPP_cbDisableTickTimer(void);
PUBLIC void os_vAPP_cbToggleLED(void *);
PUBLIC void os_vApp_TransportKeyCallback(void *);

#endif
