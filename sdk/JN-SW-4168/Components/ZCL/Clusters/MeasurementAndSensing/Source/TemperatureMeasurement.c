/*****************************************************************************
 *
 * MODULE:             Temperature Measurement Cluster
 *
 * COMPONENT:          TemperatureMeasurement.c
 *
 * AUTHOR:             Lee Mitchell
 *
 * DESCRIPTION:        Temperature Measurement cluster definition
 *
 * $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/SmartEnergy/Branches/UnifiedZCL_ZLL102_HA122_Maintenance/Clusters/MeasurementAndSensing/Source/TemperatureMeasurement.c $
 *
 * $Revision: 66148 $
 *
 * $LastChangedBy: nxp58432 $
 *
 * $LastChangedDate: 2014-11-28 11:07:32 +0000 (Fri, 28 Nov 2014) $
 *
 * $Id: TemperatureMeasurement.c 66148 2014-11-28 11:07:32Z nxp58432 $
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
 *
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
 *
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "zcl.h"
#include "TemperatureMeasurement.h"

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
#ifdef TEMPERATURE_MEASUREMENT_SERVER
    const tsZCL_AttributeDefinition asCLD_TemperatureMeasurementClusterAttributeDefinitions[] = {

        {E_CLD_TEMPMEAS_ATTR_ID_MEASURED_VALUE,         (E_ZCL_AF_RD|E_ZCL_AF_RP),        E_ZCL_INT16,    (uint32)(&((tsCLD_TemperatureMeasurement*)(0))->i16MeasuredValue),      0},   /* Mandatory */

        {E_CLD_TEMPMEAS_ATTR_ID_MIN_MEASURED_VALUE,     E_ZCL_AF_RD,        E_ZCL_INT16,    (uint32)(&((tsCLD_TemperatureMeasurement*)(0))->i16MinMeasuredValue),   0},/* Mandatory */

        {E_CLD_TEMPMEAS_ATTR_ID_MAX_MEASURED_VALUE,     E_ZCL_AF_RD,        E_ZCL_INT16,    (uint32)(&((tsCLD_TemperatureMeasurement*)(0))->i16MaxMeasuredValue),   0},/* Mandatory */

        #ifdef CLD_TEMPMEAS_ATTR_TOLERANCE
            {E_CLD_TEMPMEAS_ATTR_ID_TOLERANCE,              (E_ZCL_AF_RD|E_ZCL_AF_RP),        E_ZCL_UINT16,   (uint32)(&((tsCLD_TemperatureMeasurement*)(0))->u16Tolerance),          0},
        #endif
    };

    tsZCL_ClusterDefinition sCLD_TemperatureMeasurement = {
            MEASUREMENT_AND_SENSING_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            FALSE,
            E_ZCL_SECURITY_NETWORK,
            (sizeof(asCLD_TemperatureMeasurementClusterAttributeDefinitions) / sizeof(tsZCL_AttributeDefinition)),
            (tsZCL_AttributeDefinition*)asCLD_TemperatureMeasurementClusterAttributeDefinitions,
            NULL
    };

    uint8 au8TemperatureMeasurementServerAttributeControlBits[(sizeof(asCLD_TemperatureMeasurementClusterAttributeDefinitions) / sizeof(tsZCL_AttributeDefinition))];
#else
    tsZCL_ClusterDefinition sCLD_TemperatureMeasurement = {
            MEASUREMENT_AND_SENSING_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            FALSE,
            E_ZCL_SECURITY_NETWORK,
            0,
            NULL,
            NULL
    };
#endif
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 **
 ** NAME:       eCLD_TemperatureMeasurementCreateTemperatureMeasurement
 **
 ** DESCRIPTION:
 ** Creates a temperature measurement cluster
 **
 ** PARAMETERS:                 Name                        Usage
 ** tsZCL_ClusterInstance    *psClusterInstance             Cluster structure
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_TemperatureMeasurementCreateTemperatureMeasurement(
                tsZCL_ClusterInstance              *psClusterInstance,
                bool_t                              bIsServer,
                tsZCL_ClusterDefinition            *psClusterDefinition,
                void                               *pvEndPointSharedStructPtr,
                uint8                              *pu8AttributeControlBits)
{
    #ifdef STRICT_PARAM_CHECK 
        /* Parameter check */
        if((psClusterInstance==NULL) ||
           (psClusterDefinition==NULL))
        {
            return E_ZCL_ERR_PARAMETER_NULL;
        }
    #endif

    /* Create an instance of a temperature measurement cluster */
    vZCL_InitializeClusterInstance(
                                   psClusterInstance, 
                                   bIsServer,
                                   psClusterDefinition,
                                   pvEndPointSharedStructPtr,
                                   pu8AttributeControlBits,
                                   NULL,
                                   NULL);    
                                   
    #ifdef TEMPERATURE_MEASUREMENT_SERVER
        if(pvEndPointSharedStructPtr != NULL)
        {
            /* Initializing these values to invalid as ZCL spec does not mention the defualt */
            ((tsCLD_TemperatureMeasurement*)pvEndPointSharedStructPtr)->i16MinMeasuredValue = 0x8000;
            ((tsCLD_TemperatureMeasurement*)pvEndPointSharedStructPtr)->i16MaxMeasuredValue = 0x8000;
        }
    #endif
    
    return E_ZCL_SUCCESS;

}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

