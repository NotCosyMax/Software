/******************************************************************************

 @file dmm_policy_blesp_wsnnode.c

 @brief Dual Mode Manager Policy for WsnNode and BLE SP

 Group: WCS LPC
 Target Device: cc13x2_26x2


 
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

#include <dmm/dmm_policy_blesp_wsnnode.h>

/***** Defines *****/
//! \brief BLE POLICY INDEX
#define BLE_STACK_POLICY_IDX    0
#define WSN_STACK_POLICY_IDX    1

DMMPolicy_Policy DMMPolicy_wsnNodeBleSpPolicy[] = {
//State 1: BleSp connectable advertisements and Wsn Node TX or Ack: BleSp  = Low Priority | Time None Critical, WsnNode = High Priority | Time Critical
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_BLEPERIPH_ADV,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,         
        },

        //WSN Node Policy
        .appState[WSN_STACK_POLICY_IDX] = {
            .state = (DMMPOLICY_STACKSTATE_WSNNODE_SLEEPING | DMMPOLICY_STACKSTATE_WSNNODE_TX | DMMPOLICY_STACKSTATE_WSNNODE_ACK),
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
//State 2: BleSp connecting or connected and Wsn Node Tx: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | Time None Critical
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING | DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED),
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //WSN Node Policy
        .appState[WSN_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_WSNNODE_TX,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
//State 3: BleSp connecting or connected and Wsn Node Ack Rx: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | Time None Critical
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING | DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED),
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //WSN Node Policy
        .appState[WSN_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_WSNNODE_ACK,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },

        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
//Default State: If matching state is not found default to: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | none Time None Critical
    {
        //BLE Policy
        .appState[BLE_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_BLEPERIPH_ANY,
            .weight = DMMPOLICY_PRIORITY_HIGH,
            .timingConstraint = DMMPOLICY_TIME_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //WSN Node Policy
        .appState[WSN_STACK_POLICY_IDX] = {
            .state = DMMPOLICY_STACKSTATE_WSNNODE_ANY,
            .weight = DMMPOLICY_PRIORITY_LOW,
            .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
            .pause = DMMPOLICY_NOT_PAUSED,
            .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
        },
        //Balanced Mode Policy
        .balancedMode = DMMPOLICY_BALANCED_NONE,
    },
};

DMMPolicy_PolicyTable DMMPolicy_wsnNodeBleSpPolicyTable = {
     //Stack Roles
     .stackRole[BLE_STACK_POLICY_IDX] = DMMPolicy_StackRole_BlePeripheral,
     .stackRole[WSN_STACK_POLICY_IDX] = DMMPolicy_StackRole_WsnNode,
     //policy table
     .policy = DMMPolicy_wsnNodeBleSpPolicy,
     // Index Table for future use
     .indexTable = NULL,
};

//! \brief The policy table size for the BLE Simple Peripheral and WSN Node use case
uint32_t DMMPolicy_wsnNodeBleSpPolicySize = (sizeof(DMMPolicy_wsnNodeBleSpPolicy) / sizeof(DMMPolicy_Policy));
