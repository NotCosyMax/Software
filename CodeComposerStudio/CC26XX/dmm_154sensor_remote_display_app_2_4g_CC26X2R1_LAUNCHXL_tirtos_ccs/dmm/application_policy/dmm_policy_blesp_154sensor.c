/******************************************************************************

 @file dmm_policy_blesp_154sensor.c

 @brief Dual Mode Manager Policy for 15.4Sensor and BLE SP

 Group: WCS LPC
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2017-2019, Texas Instruments Incorporated
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

#include <dmm/dmm_policy_blesp_154sensor.h>
#include <config.h>
#include <jdllc.h>

/***** Defines *****/

//! \brief BLE POLICY INDEX
#define BLE_STACK_POLICY_IDX      0
#define TI154_STACK_POLICY_IDX    1

DMMPolicy_Policy DMMPolicy_bleSp154SensorPolicy[] = {
// State 1 - BLE Any, 15.4 Uninitialised: BLE High Priorty, 15.4 Sensor Low Priority
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_ANY),
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,          
        },
        //15.4 Node Policy
        .appState[TI154_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_154SENSOR_UNINIT,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,            
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
//State 2 - BLE Advertising or connected, sensor commissioning: 15.4 Sensor On Min = 500ms, Off Max 70ms
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_ADV | DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED),
#if (CONFIG_FH_ENABLE)
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_PAUSED,
#else
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
#endif
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //15.4 Node Policy
        .appState[TI154_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_154SENSOR_PROVISIONING,
#if (CONFIG_FH_ENABLE)
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
#else
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
#endif
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
#if (CONFIG_FH_ENABLE)
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
#else
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_TIME_MODE_1( 500, //Zigbee (High Pri Stack) onMin 100ms
                                                        70), //Zigbee (High Pri Stack) offMax 70ms
#endif
   },
//State 3 - BLE Connecting, 15.4 Sensor Any: BLE High Priorty, 15.4 Sensor Low Priority
   {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING,
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //15.4 Node Policy
        .appState[TI154_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_154SENSOR_ANY,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
//Default State: BLE Low Priorty, 15.4 Sensor High Priority
   {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_BLEPERIPH_ANY,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //15.4 Node Policy
        .appState[TI154_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_154SENSOR_ANY,
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
};

DMMPolicy_PolicyTable DMMPolicy_bleSp154SensorPolicyTable = {
     //Stack Roles
     .stackRole[BLE_STACK_POLICY_IDX] = DMMPolicy_StackRole_BlePeripheral,
     .stackRole[TI154_STACK_POLICY_IDX] = DMMPolicy_StackRole_154Sensor,
     //policy table
     .policy = DMMPolicy_bleSp154SensorPolicy,
     // Index Table for future use
     .indexTable = NULL,
};

//! \brief The policy table size for the BLE Simple Peripheral and 15.4 Sensor use case
uint32_t DMMPolicy_bleSp154SensorPolicySize = (sizeof(DMMPolicy_bleSp154SensorPolicy) / sizeof(DMMPolicy_Policy));
