/******************************************************************************

 @file dmm_policy.h

 @brief dmm policy Header

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
/*!****************************************************************************
 *  @file  dmm_policy.h
 *
 *  @brief      Dual Mode Policy Manager
 *
 *  The DMMPolicy interface provides a service to for the stack applications to
 *  update the stack states, which is then used to make scheduling decisions
 *
 *
 *  # Usage #
 *
 *  To use the DMMPolicy module to set the scheduling policy, the application
 *  calls the following APIs:
 *    - DMMPolicy_init(): Initialize the DMMPolicy module/task.
 *    - DMMPolicy_Params_init():  Initialize a DMMPolicy_Params structure
 *      with default values.  Then change the parameters from non-default
 *      values as needed.
 *    - DMMPolicy_open():  Open an instance of the DMMPolicy module,
 *      passing the initialized parameters.
 *    - Stack A/B application - DMMPolicy_updateStackState: Update the policy
 *                              used by DMMSch for scheduling RF commands from
 *                              stack A and B
 *
 *   An example of a policy table (define in a use case specific policy header,
 *   such as dmm_policy_blesp_wsnnode.h)
 *
 *   \code
 * DMMPolicy_Policy DMMPolicy_wsnNodeBleSpPolicy[] = {
 * //State 1: BleSp connectable advertisements and Wsn Node TX or Ack: BleSp  = Low Priority | Time None Critical, WsnNode = High Priority | Time Critical
 *     {
 *         //BLE Policy
 *         .appState[BLE_STACK_POLICY_IDX] = {
 *             .state = DMMPOLICY_STACKSTATE_BLEPERIPH_ADV,
 *             .weight = DMMPOLICY_PRIORITY_LOW,
 *             .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *
 *         //WSN Node Policy
 *         .appState[WSN_STACK_POLICY_IDX] = {
 *             .state = (DMMPOLICY_STACKSTATE_WSNNODE_SLEEPING | DMMPOLICY_STACKSTATE_WSNNODE_TX | DMMPOLICY_STACKSTATE_WSNNODE_ACK),
 *             .weight = DMMPOLICY_PRIORITY_HIGH,
 *             .timingConstraint = DMMPOLICY_TIME_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //Balanced Mode Policy
 *         .balancedMode = DMMPOLICY_BALANCED_NONE,
 *     },
 * //State 2: BleSp connecting or connected and Wsn Node Tx: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | Time None Critical
 *     {
 *         //BLE Policy
 *         .appState[BLE_STACK_POLICY_IDX] = {
 *             .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING | DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED),
 *             .weight = DMMPOLICY_PRIORITY_HIGH,
 *             .timingConstraint = DMMPOLICY_TIME_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //WSN Node Policy
 *         .appState[WSN_STACK_POLICY_IDX] = {
 *             .state = DMMPOLICY_STACKSTATE_WSNNODE_TX,
 *             .weight = DMMPOLICY_PRIORITY_LOW,
 *             .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //Balanced Mode Policy
 *         .balancedMode = DMMPOLICY_BALANCED_NONE,
 *     },
 * //State 3: BleSp connecting or connected and Wsn Node Ack Rx: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | Time None Critical
 *     {
 *         //BLE Policy
 *         .appState[BLE_STACK_POLICY_IDX] = {
 *             .state = (DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING | DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED),
 *             .weight = DMMPOLICY_PRIORITY_HIGH,
 *             .timingConstraint = DMMPOLICY_TIME_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //WSN Node Policy
 *         .appState[WSN_STACK_POLICY_IDX] = {
 *             .state = DMMPOLICY_STACKSTATE_WSNNODE_ACK,
 *             .weight = DMMPOLICY_PRIORITY_LOW,
 *             .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *
 *         //Balanced Mode Policy
 *         .balancedMode = DMMPOLICY_BALANCED_NONE,
 *     },
 * //Default State: If matching state is not found default to: BleSp  = High Priority | Time Critical, WsnNode = Low Priority | none Time None Critical
 *     {
 *         //BLE Policy
 *         .appState[BLE_STACK_POLICY_IDX] = {
 *             .state = DMMPOLICY_STACKSTATE_BLEPERIPH_ANY,
 *             .weight = DMMPOLICY_PRIORITY_HIGH,
 *             .timingConstraint = DMMPOLICY_TIME_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //WSN Node Policy
 *         .appState[WSN_STACK_POLICY_IDX] = {
 *             .state = DMMPOLICY_STACKSTATE_WSNNODE_ANY,
 *             .weight = DMMPOLICY_PRIORITY_LOW,
 *             .timingConstraint = DMMPOLICY_TIME_NONE_CRITICAL,
 *             .pause = DMMPOLICY_NOT_PAUSED,
 *             .appliedActivity = DMMPOLICY_APPLIED_ACTIVITY_NONE,
 *         },
 *         //Balanced Mode Policy
 *         .balancedMode = DMMPOLICY_BALANCED_NONE,
 *     },
 * };
 *
 * DMMPolicy_PolicyTable DMMPolicy_wsnNodeBleSpPolicyTable = {
 *      //Stack Roles
 *      .stackRole[BLE_STACK_POLICY_IDX] = DMMPolicy_StackRole_BlePeripheral,
 *      .stackRole[WSN_STACK_POLICY_IDX] = DMMPolicy_StackRole_WsnNode,
 *      //policy table
 *      .policy = DMMPolicy_wsnNodeBleSpPolicy,
 *      // Index Table for future use
 *      .indexTable = NULL,
 * };
 *
 * //! \brief The policy table size for the BLE Simple Peripheral and WSN Node use case
 * uint32_t DMMPolicy_wsnNodeBleSpPolicySize = (sizeof(DMMPolicy_wsnNodeBleSpPolicy) / sizeof(DMMPolicy_Policy));
 *
 *   \endcode
 *
 ********************************************************************************/

#ifndef DMMPolicy_H_
#define DMMPolicy_H_

#include "stdint.h"
#include <ti/drivers/rf/RF.h>

//! \brief number of stacks supported by this policy manager
#define DMMPOLICY_NUM_STACKS    2

//! \brief stack priority
#define DMMPOLICY_PRIORITY_LOW     0
#define DMMPOLICY_PRIORITY_HIGH    1

//! \brief stack timing
#define DMMPOLICY_TIME_NONE_CRITICAL    0
#define DMMPOLICY_TIME_CRITICAL         1

//! \brief defines for the app pause callback
#define DMMPOLICY_PAUSED                0xFFFF
#define DMMPOLICY_NOT_PAUSED            0

//! \brief dynamic mode defines
#define DMMPOLICY_BALANCED_NONE             0          //!< Does not use dynamic mode

#define DMMPOLICY_BALANCED_TIME_BM_1               0x80000000 //!< Dynamic mode is time (MSB = 1)
#define DMMPOLICY_BALANCED_TIME_MODE_1(onMin, offMax)     (DMMPOLICY_BALANCED_TIME_BM_1 | (onMin & 0xFFF) | ((offMax & 0xFFF) << 12))
#define DMMPOLICY_BALANCED_TIME_MODE_1_ON_MIN(RatioTime)     (RatioTime & 0xFFF)
#define DMMPOLICY_BALANCED_TIME_MODE_1_OFF_MAX(RatioTime)     ((RatioTime & 0xFFF000) >> 12)

#define DMMPOLICY_APPLIED_ACTIVITY_NONE     0

//! \brief the stack roles supported
typedef enum
{
    DMMPolicy_StackRole_invalid = 0,          //!< invalid stack role
    DMMPolicy_StackRole_BlePeripheral,        //!< stack role for a BLE Simple Peripheral
    DMMPolicy_StackRole_WsnNode,              //!< stack role for an EasyLink Wireless Sensor Network Node
    DMMPolicy_StackRole_154Sensor,            //!< stack role for a 15.4 Sensor or Zigbee Device
    DMMPolicy_StackRole_reserved1,
    DMMPolicy_StackRole_reserved2,
} DMMPolicy_StackRole;

typedef struct
{
    uint32_t     state;
    uint8_t      weight;            //!< 0 being lowest priority
    uint16_t     timingConstraint;  //!< 0=Time critical and cannot be changed, 0xFFFF=No timing constraint, 1-0xFFFE timing constraint in ms units
    uint32_t     appliedActivity;   //!< reserved for future use
    uint16_t     pause;             //!< 0 not paused, 0xFFFF paused
} DMMPolicy_State;

//! \brief Structure used to decide the policy for a particular stack state
typedef struct
{
    DMMPolicy_State appState[DMMPOLICY_NUM_STACKS];
    uint32_t        balancedMode;          //!< 0x0 = no ratio mode, 0x0000xxyy = stack 1:stack 2 = xx:yy, 0x80xxxyyy = Hi Pri Stack xxx ms min on yyy ms max off.
} DMMPolicy_Policy;

typedef struct
{
    uint8_t *CmdIndex;
    uint8_t tableSize;
}DMMPolicy_StackCmdIndexTable;

//! \brief policy table entry
typedef struct
{
    DMMPolicy_StackRole stackRole[DMMPOLICY_NUM_STACKS];
    DMMPolicy_Policy* policy;
    DMMPolicy_StackCmdIndexTable *indexTable;
} DMMPolicy_PolicyTable;

/** @brief RF parameter struct
 *  DMM Scheduler parameters are used with the DMMPolicy_open() and DMMPolicy_Params_init() call.
 */
typedef struct {
    DMMPolicy_PolicyTable policyTable; //!< policy table to be used for the DMM use case
    uint32_t numPolicyTableEntries;            //!< entries in policy table
} DMMPolicy_Params;

/** @brief Status codes for various DMM Policy functions.
 *
 *  RF_Stat is reported as return value for DMM Policy functions.
 */
typedef enum {
    DMMPolicy_StatusError,          ///< Error
    DMMPolicy_StatusNoPolicyError,  ///< Error with policy table
    DMMPolicy_StatusParamError,     ///< Parameter Error
    DMMPolicy_StatusSuccess         ///< Function finished with success
} DMMPolicy_Status;

//! \brief Callback function type for app pause/reseume
typedef void (*DMMPolicy_appPauseCb_t)(uint16_t pause);

//! \brief Structure for app callbacks
typedef struct
{
    DMMPolicy_appPauseCb_t appPauseCb;
} DMMPolicy_AppCbs_t;

/** @brief  Function to initialize the DMMPolicy_Params struct to its defaults
 *
 *  @param  params      An pointer to RF_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 */
extern void DMMPolicy_Params_init(DMMPolicy_Params *params);

/** @brief   Register the application policy callbacks
 *
 *  @param  AppCbs      application callback take
 *  @param  StackRole   application stack role
 */
extern void DMMPolicy_registerAppCbs(DMMPolicy_AppCbs_t AppCbs, DMMPolicy_StackRole StackRole);

/** @brief  Function that initializes the DMMPolicy module
 *
 */
extern void DMMPolicy_init(void);

/** @brief  Function to open the DMMPolicy module
 *
 *  @param  params      An pointer to RF_Params structure for initialization
 *
 *  @return DMMPolicy_Stat status
 */
extern DMMPolicy_Status DMMPolicy_open(DMMPolicy_Params *params);

/** @brief  Updates the policy used to make scheduling decisions
 *
 *  @param  StackRole     The stack role that has changed state
 *  @param  newState      The state the stack has changed to
 *
 *  @return DMMPolicy_Stat status
 */
extern DMMPolicy_Status DMMPolicy_updateStackState(DMMPolicy_StackRole StackRole, uint32_t newState);

#endif /* DMMPolicy_H_ */
