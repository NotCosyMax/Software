/******************************************************************************

 @file dmm_policy_blesp_wsnnode.h

 @brief Policy for the BLE SImple Peripheral and 15.4 Sensor DMM use case

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
#ifndef dmm_policy_blesp_154sensor__H_
#define dmm_policy_blesp_154sensor__H_

#include <dmm/dmm_policy.h>

/***** Defines *****/
//! \brief The supported stack states bit map for BLE Simple Peripheral
#define DMMPOLICY_STACKSTATE_BLEPERIPH_IDLE         0x00000001 //!< State for BLE Simple Peripheral when advertising connectable
#define DMMPOLICY_STACKSTATE_BLEPERIPH_ADV          0x00000002 //!< State for BLE Simple Peripheral when advertising connectable
#define DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING   0x00000004 //!< State for BLE Simple Peripheral when in the process of connecting to a master device
#define DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED    0x00000008 //!< State for BLE Simple Peripheral when connected to a master device
#define DMMPOLICY_STACKSTATE_BLEPERIPH_ANY          0xFFFFFFFF //!< Allow any policy

//! \brief The supported stack states bit map for 15.4 Sensor
#define DMMPOLICY_STACKSTATE_154SENSOR_UNINIT           0x00000001 //!< State for 15.4 Sensor when uninitialized
#define DMMPOLICY_STACKSTATE_154SENSOR_PROVISIONING     0x00000002 //!< State for 15.4 Sensor joining a network
#define DMMPOLICY_STACKSTATE_154SENSOR_CONNECTED        0x00000004 //!< State for 15.4 Sensor when connected to a network
#define DMMPOLICY_STACKSTATE_154SENSOR_ANY              0xFFFFFFFF //!< Allow any policy

//! \brief The policy table for the BLE Simple Peripheral and 15.4 Sensor use case
extern DMMPolicy_PolicyTable DMMPolicy_bleSp154SensorPolicyTable;

//! \brief The policy table size for the BLE Simple Peripheral and 15.4 Sensor use case
extern uint32_t DMMPolicy_bleSp154SensorPolicySize;

#endif //dmm_policy_blesp_154sensor__H_
