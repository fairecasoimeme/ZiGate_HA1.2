/*****************************************************************************
 *
 * MODULE:             Light Link Profile
 *
 * COMPONENT:
 *
 * AUTHOR:
 *
 * DESCRIPTION:        ZigBee Light Link profile functions
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/SmartEnergy/Branches/UnifiedZCL_ZLL102_HA122_Maintenance/Profiles/ZLL/Source/dimmable_plug.c $
 *
 * $Revision: 66247 $
 *
 * $LastChangedBy: nxp46755 $
 *
 * $LastChangedDate: 2014-12-03 07:49:12 +0000 (Wed, 03 Dec 2014) $
 *
 * $Id: dimmable_plug.c 66247 2014-12-03 07:49:12Z nxp46755 $
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5164,
 * JN5161, JN5148, JN5142, JN5139]. 
 * You, and any third parties must reproduce the copyright and warranty notice 
 * and any other legend of ownership on each  copy or partial copy of the software.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
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
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "zps_apl.h"
#include "zcl_heap.h"
#include "zcl.h"
#include "zcl_options.h"
#include "dimmable_plug.h"
#ifdef CLD_OTA
#include "OTA.h"
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

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: eZLL_RegisterDimmablePlugEndPoint
 *
 * DESCRIPTION:
 * Registers a dimmable plug device with the ZCL layer
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

PUBLIC teZCL_Status eZLL_RegisterDimmablePlugEndPoint(uint8 u8EndPointIdentifier,
                                              tfpZCL_ZCLCallBackFunction cbCallBack,
                                              tsZLL_DimmablePlugDevice *psDeviceInfo)
{

    /* Fill in end point details */
    psDeviceInfo->sEndPoint.u8EndPointNumber = u8EndPointIdentifier;
    psDeviceInfo->sEndPoint.u16ManufacturerCode = ZLL_MANUFACTURER_CODE;
    psDeviceInfo->sEndPoint.u16ProfileEnum = HA_PROFILE_ID;
    psDeviceInfo->sEndPoint.bIsManufacturerSpecificProfile = FALSE;
    psDeviceInfo->sEndPoint.u16NumberOfClusters = sizeof(tsZLL_DimmablePlugDeviceClusterInstances) / sizeof(tsZCL_ClusterInstance);
    psDeviceInfo->sEndPoint.psClusterInstance = (tsZCL_ClusterInstance*)&psDeviceInfo->sClusterInstance;
    psDeviceInfo->sEndPoint.bDisableDefaultResponse = ZLL_DISABLE_DEFAULT_RESPONSES;
    psDeviceInfo->sEndPoint.pCallBackFunctions = cbCallBack;

    #if (defined CLD_BASIC) && (defined BASIC_SERVER)
        /* Create an instance of a basic cluster as a server */
        eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicServer,
                              TRUE,
                              &sCLD_Basic,
                              &psDeviceInfo->sBasicServerCluster,
                              &au8BasicClusterAttributeControlBits[0]);
    #endif

    #if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
        /* Create an instance of an Identify cluster as a server */
        eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyServer,
                              TRUE,
                              &sCLD_Identify,
                              &psDeviceInfo->sIdentifyServerCluster,
                              &au8IdentifyServerAttributeControlBits[0],
                              &psDeviceInfo->sIdentifyServerCustomDataStructure);
    #endif

    #if (defined CLD_ONOFF) && (defined ONOFF_SERVER)
        /* Create an instance of a On/Off cluster as a server */
        eCLD_OnOffCreateOnOff(&psDeviceInfo->sClusterInstance.sOnOffServer,
                              TRUE,
                              &sCLD_OnOff,
                              &psDeviceInfo->sOnOffServerCluster,
                              &au8OnOffServerAttributeControlBits[0],
                              &psDeviceInfo->sOnOffServerCustomDataStructure);
    #endif

    #if (defined CLD_SCENES) && (defined SCENES_SERVER)
        /* Create an instance of a Scenes cluster as a server */
        eCLD_ScenesCreateScenes(&psDeviceInfo->sClusterInstance.sScenesServer,
                              TRUE,
                              &sCLD_Scenes,
                              &psDeviceInfo->sScenesServerCluster,
                              &au8ScenesServerAttributeControlBits[0],
                              &psDeviceInfo->sScenesServerCustomDataStructure,
                              &psDeviceInfo->sEndPoint);
    #endif

    #if (defined CLD_GROUPS) && (defined GROUPS_SERVER)
        /* Create an instance of a Groups cluster as a server */
        eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsServer,
                              TRUE,
                              &sCLD_Groups,
                              &psDeviceInfo->sGroupsServerCluster,
                              &au8GroupsServerAttributeControlBits[0],
                              &psDeviceInfo->sGroupsServerCustomDataStructure,
                              &psDeviceInfo->sEndPoint);
    #endif

    #if (defined CLD_LEVEL_CONTROL) && (defined LEVEL_CONTROL_SERVER)
        /* Create an instance of a Level Control cluster as a server */
        eCLD_LevelControlCreateLevelControl(&psDeviceInfo->sClusterInstance.sLevelControlServer,
                              TRUE,
                              &sCLD_LevelControl,
                              &psDeviceInfo->sLevelControlServerCluster,
                              &au8LevelControlServerAttributeControlBits[0],
                              &psDeviceInfo->sLevelControlServerCustomDataStructure);
    #endif

    #if (defined CLD_OTA) && (defined OTA_CLIENT)
        /* Create an instance of an OTA cluster as a client */
       if(eOTA_Create(&psDeviceInfo->sClusterInstance.sOTAClient,
                      FALSE,  /* client */
                      &sCLD_OTA,
                      &psDeviceInfo->sCLD_OTA,  /* cluster definition */
                      u8EndPointIdentifier,
                      NULL, //au8OTAClusterAttributesControlBits,
                      &psDeviceInfo->sCLD_OTA_CustomDataStruct)!= E_ZCL_SUCCESS)
        {
            // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
            return E_ZCL_FAIL;
        }
    #endif

    return eZCL_Register(&psDeviceInfo->sEndPoint);
}


/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

