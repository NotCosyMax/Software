/******************************************************************************

 @file  remote_display.h

 @brief This file contains the Remote Display BLE application
        definitions and prototypes.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef REMOTE_DISPLAY_H
#define REMOTE_DISPLAY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "jdllc.h"

#define RemoteDisplay_JOIN_STATE_SYNC_LOSS  7

typedef enum
{
    ProvisionAttr_PanId,
    ProvisionAttr_ExtPanId,
    ProvisionAttr_Freq,
    ProvisionAttr_SensorChannelMask,
    ProvisionAttr_FFDAddr,
    ProvisionAttr_NtwkKey,
    ProvisionAttr_ProvState
} ProvisionAttr_t;

typedef enum
{
    RemoteDisplayAttr_ReportInterval,
    RemoteDisplayAttr_CollLed,
    RemoteDisplayAttr_SensorData
} RemoteDisplayAttr_t;

//! \brief Callback function type for getting the provisioning attributes
typedef void (*RemoteDisplay_setProvisioningAttrCb_t)(ProvisionAttr_t provisioningAttr, void *const value, uint8_t len);

//! \brief Callback function type for getting the provisioning attributes
typedef void (*RemoteDisplay_getProvisioningAttrCb_t)(ProvisionAttr_t provisioningAttr, void *value, uint8_t len);

//! \brief Callback function type for starting provisioning of a device
typedef void (*RemoteDisplay_provisionConnectCb_t)(void);

//! \brief Callback function type for stopping a provisioned device
typedef void (*RemoteDisplay_provisionDisconnectCb_t)(void);

//! \brief Structure for sensor provisioning callbacks
typedef struct
{
    RemoteDisplay_setProvisioningAttrCb_t setProvisioningAttrCb;
    RemoteDisplay_getProvisioningAttrCb_t getProvisioningAttrCb;
    RemoteDisplay_provisionConnectCb_t provisionConnectCb;
    RemoteDisplay_provisionDisconnectCb_t provisionDisconnectCb;
} RemoteDisplay_clientProvisioningtCbs_t;

//! \brief Callback function type for getting the remote display attributes
typedef void (*RemoteDisplay_setRDAttrCb_t)(RemoteDisplayAttr_t remoteDisplayAttr, void *const value, uint8_t len);

//! \brief Callback function type for getting the remote display attributes
typedef void (*RemoteDisplay_getRDAttrCb_t)(RemoteDisplayAttr_t remoteDisplayAttr, void *value, uint8_t len);

//! \brief Structure for sensor remote display callbacks
typedef struct
{
    RemoteDisplay_setRDAttrCb_t setRDAttrCb;
    RemoteDisplay_getRDAttrCb_t getRDAttrCb;
} RemoteDisplayCbs_t;

#ifdef DISPLAY_PER_STATS
//! \brief Structure for PER 
typedef struct
{
    unsigned int successes;
    unsigned int failures;
    float per;
} RemoteDisplay_perData_t;
#endif


/*********************************************************************

*@fn RemoteDisplay_registerClientProvCbs
*@brief Register the commissioning client callbacks
*/
extern void RemoteDisplay_registerClientProvCbs(RemoteDisplay_clientProvisioningtCbs_t clientCbs);

/*********************************************************************

*@fn RemoteDisplay_registerclientProvCbs
*@brief Register the commissioning client callbacks
*/
extern void RemoteDisplay_registerRDCbs(RemoteDisplayCbs_t clientCbs);

/*********************************************************************

*@fn RemoteDisplay_updateSensorData
*@brief Update sensor data
*/
extern void RemoteDisplay_updateSensorData(void);

/*********************************************************************
 *  @fn      RemoteDisplay_createTask
 *
 * @brief   Task creation function for the Remote Display Peripheral.
 */
extern void RemoteDisplay_createTask(void);

/*********************************************************************
 *  @fn      RemoteDisplay_updateSensorJoinState
 *
 * @brief   Sets the sensors sensor reading characteristic
 */
extern void RemoteDisplay_updateSensorJoinState(Jdllc_states_t state);

#ifdef DISPLAY_PER_STATS
/*********************************************************************
 *  @fn      RemoteDisplay_updatePER
 *
 * @brief   Updates the display with new PER values
 */
extern void RemoteDisplay_updatePER(RemoteDisplay_perData_t* smsgs);
#endif /* DISPLAY_PER_STATS */

/*********************************************************************
 *  @fn      RemoteDisplay_bleFastStateUpdateCb
 *
 * @brief   Callback from BLE link layer to indicate a state change
 */
extern void RemoteDisplay_bleFastStateUpdateCb(uint32_t StackRole, uint32_t stackState);

/*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_DISPLAY_H */
