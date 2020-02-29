/******************************************************************************

 @file dmm_scheduler.h

 @brief Dual Mode Manager Scheduler

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
 *  @file  dmm_scheduler.h
 *
 *  @brief      Dual Mode Manager Scheduler
 *
 *  The DMMSch interface provides a service to adjust the timing of RF commands to meet the
 *  needs of multiple stacks. The stacks will call into a jump table for the RF commands, and
 *  any RF API's pertinent to the timing of a command will be remapped to a DMMSch alternative.
 *
 *  The adjustment of timing will be done based on a policy set by the DMM Policy Manager, which
 *  will be set by passing a policy structure.
 *
 *
 *  # Usage #
 *
 *  To use the DMMSch module to schedule a tacks RF commands, the application
 *  calls the following APIs:
 *    - DMMSch_init(): Initialize the DMMSch module/task.
 *    - DMMSch_Params_init():  Initialize a DMMSch_Params structure
 *      with default values.  Then change the parameters from non-default
 *      values as needed.
 *    - DMMSch_open():  Open an instance of the DMMSch module,
 *      passing the initialized parameters.
 *    - Stack A application - DMMSch_registerClient: Passes the rfMode point so DMMSch can map an RF handle
 *                            to a stack role (and hance known which policy applies to an RF handle passed
 *                            in a command)
 *    - Stack A application - Rf_open -> DMMSch_rfOpen: DMMSch opens an RF client and stores maps the Stack
 *                            Type to an RF Handle (vie the rfMode passed in DMMSch_registerClient)
 *    - Stack B application - DMMSch_registerClient: Passes the rfMode point so DMMSch can map an RF handle
 *                            to a stack role (and hence known which policy applies to an RF handle passed
 *                            in a command)
 *    - Stack B application - Rf_open -> DMMSch_rfOpen: DMMSch opens an RF client and stores maps the Stack
 *                            Type to an RF Handle (vie the rfMode passed in DMMSch_registerClient)
 *    - Stack A application - Rf_postCmd -> DMMSch_rfPostCmd: DMMSch adjusted timing based on policy
 *    - Stack B application - Rf_postCmd -> DMMSch_rfPostCmd: DMMSch adjusted timing based on policy
 *
 *
 ********************************************************************************/

#ifndef DMMSch_H_
#define DMMSch_H_

#include "stdint.h"

#include <ti/drivers/rf/RF.h>
#include <ti/sysbios/knl/Task.h>

#include "dmm_policy.h"


#define xDMM_DEBUG_LOGGING

#ifdef DMM_DEBUG_LOGGING
#include <xdc/runtime/Log.h>
#define DMM_dbgLog0(str)                     Log_info0(str)
#define DMM_dbgLog1(str, a0)                 Log_info1(str, a0)
#define DMM_dbgLog2(str, a0, a1)             Log_info2(str, a0, a1)
#define DMM_dbgLog3(str, a0, a1, a2)         Log_info3(str, a0, a1, a2)
#define DMM_dbgLog4(str, a0, a1, a2, a3)     Log_info4(str, a0, a1, a2, a3)
#define DMM_dbgLog5(str, a0, a1, a2, a3, a4) Log_info5(str, a0, a1, a2, a3, a4)
#else
#define DMM_dbgLog0(str)
#define DMM_dbgLog1(str, a0)
#define DMM_dbgLog2(str, a0, a1)
#define DMM_dbgLog3(str, a0, a1, a2)
#define DMM_dbgLog4(str, a0, a1, a2, a3)
#define DMM_dbgLog5(str, a0, a1, a2, a3, a4)
#endif

/** @brief RF parameter struct
 *  DMM Scheduler parameters are used with the DMMSch_open() and DMMSch_Params_init() call.
 */
typedef struct {
    DMMPolicy_StackRole stackRoles[DMMPOLICY_NUM_STACKS];
    DMMPolicy_StackCmdIndexTable *indexTable;
} DMMSch_Params;


/**
 *  @brief Defines the preemption callback.
 *
 *  The function is invoked when a preemption occurs in DMM Scheduler.
 *
 *  The arguments are:
 *      - \a stackRolePreempted stack role for command was preempted.
 */
typedef void (*DMMSch_PreemptionCb)(DMMPolicy_StackRole stackRolePreempted);

/** @brief  Function to initialize the DMMSch_Params struct to its defaults
 *
 *  @param  params      An pointer to RF_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 */
extern void DMMSch_Params_init(DMMSch_Params *params);

/** @brief  Function that initializes the DMMSch module
 *
 */
extern void DMMSch_init(void);

/** @brief  allows policy manager to register a callback on command preemption
 *
 *  @param  dmmSchPreemptionCb     callback to register
 */
extern void DMMSch_registerPreemptionCb(DMMSch_PreemptionCb dmmSchPreemptionCb);

/** @brief  Function to open the DMMSch module
 *
 *  @param  params      An pointer to RF_Params structure for initialization
 */
extern void DMMSch_open(DMMSch_Params *params);

/** @brief  Updates the policy used to make scheduling decisions
 *
 *  @param  policy     A new policy
 */
extern void DMMSch_updatePolicy(DMMPolicy_Policy policy);

/** @brief  Register an DMM Scheduler client
 *
 *  @param  pTaskHndl Task handle that the stack is running in, used to map the
 *                    RF Client handle to a stack role
 *
 *  @param  StackRole   stack role associated with Task handle
 */
void DMMSch_registerClient(Task_Handle* pTaskHndl, DMMPolicy_StackRole StackRole);

/** @brief  Intercepts calls from a stack to RF_postCmd (re-mapped to DMMSch_rfOpen),
 *          The DMMSch module uses this to tie
 *
 *  @param  params      An pointer to RF_Params structure for initialization
 */
extern RF_Handle DMMSch_rfOpen(RF_Object *pObj, RF_Mode *pRfMode, RF_RadioSetup *pOpSetup, RF_Params *params);

/**
 *  @brief  Handles calls from a stack to RF_postCmd (re-mapped to DMMSch_postCmd),
 *  adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @sa RF_pendCmd(), RF_runCmd(), RF_scheduleCmd(), RF_RF_cancelCmd(), RF_flushCmd(), RF_getCmdOp()
 *
 *  @param h         Driver handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF operation command.
 *  @param ePri      Priority of this RF command (used for arbitration in multi-client systems)
 *  @param pCb       Callback function called during command execution and upon completion.
 *                   If RF_postCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback or that can be pended on.
 *  @return          A handle to the RF command. Return value of RF_ALLOC_ERROR indicates error.
 */
extern RF_CmdHandle DMMSch_rfPostCmd(RF_Handle h, RF_Op* pOp, RF_Priority ePri, RF_Callback pCb, RF_EventMask bmEvent);

/**
 *  @brief  Handles calls from a stack to RF_scheduleCmd (re-mapped to DMMSch_scheduleCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @param h         Handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF_Op. Must normally be in persistent and writeable memory
 *  @param pSchParams Pointer to the schedule command parameter structure
 *  @param pCb       Callback function called upon command completion (and some other events).
 *                   If RF_scheduleCmd() fails no callback is made
 *  @param bmEvent   Bitmask of events that will trigger the callback.
 *  @return          A handle to the RF command. Return value of RF_ALLOC_ERROR indicates error.
 */
extern RF_CmdHandle DMMSch_rfScheduleCmd(RF_Handle h, RF_Op* pOp, RF_ScheduleCmdParams *pSchParams, RF_Callback pCb, RF_EventMask bmEvent);

/** @brief  Handles calls from a stack to RF_runCmd (re-mapped to DMMSch_runCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *  @param h         Driver handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF operation command.
 *  @param ePri      Priority of this RF command (used for arbitration in multi-client systems)
 *  @param pCb       Callback function called during command execution and upon completion.
 *                   If RF_runCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback or that can be pended on.
 *  @return          The relevant termination event.
 */
extern RF_EventMask DMMSch_rfRunCmd(RF_Handle h, RF_Op* pOp, RF_Priority ePri, RF_Callback pCb, RF_EventMask bmEvent);

/**
 *  @brief  Handles calls from a stack to RF_runScheduleCmd (re-mapped to DMMSch_runScheduleCmd),
 *          adjusts timing as necessary and schedules then accordingly with RF_scheduleCmd.
 *
 *
 *  @param h         Handle previously returned by RF_open()
 *  @param pOp       Pointer to the RF_Op. Must normally be in persistent and writeable memory
 *  @param pSchParams Pointer to the schdule command parameter structure
 *  @param pCb       Callback function called upon command completion (and some other events).
 *                   If RF_runScheduleCmd() fails, no callback is made.
 *  @param bmEvent   Bitmask of events that will trigger the callback.
 *  @return          The relevant command completed event.
 */
extern RF_EventMask DMMSch_rfRunScheduleCmd(RF_Handle h, RF_Op* pOp, RF_ScheduleCmdParams *pSchParams, RF_Callback pCb, RF_EventMask bmEvent);

/**
 *  @brief  Abort/stop/cancel single command in command queue.
 *
 *  If command is running, aborts/stops it and posts callback for the
 *  aborted/stopped command. <br>
 *  If command has not yet run, cancels it it and posts callback for the
 *  cancelled command. <br>
 *  If command has already run or been aborted/stopped/cancelled, has no effect.<br>
 *  If RF_cancelCmd is called from a Swi context with same or higher priority
 *  than RF Driver Swi, when the RF core is powered OFF -> the cancel callback will be delayed
 *  until the next power-up cycle.<br>
 *
 *  @note Calling context : Task/SWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param ch           Command handle previously returned by RF_postCmd().
 *  @param mode         1: Stop gracefully, 0: abort abruptly
 *  @return             RF_Stat indicates if command was successfully completed
 */
extern RF_Stat DMMSch_rfCancelCmd(RF_Handle h, RF_CmdHandle ch, uint8_t mode);

/**
 *  @brief  Abort/stop/cancel command and any subsequent commands in command queue.
 *
 *  If command is running, aborts/stops it and then cancels all later commands in queue.<br>
 *  If command has not yet run, cancels it and all later commands in queue.<br>
 *  If command has already run or been aborted/stopped/cancelled, has no effect.<br>
 *  The callbacks for all cancelled commands are issued in chronological order.<br>
 *  If RF_flushCmd is called from a Swi context with same or higher priority
 *  than RF Driver Swi, when the RF core is powered OFF -> the cancel callback will be delayed
 *  until the next power-up cycle.<br>
 *
 *  @note Calling context : Task/SWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param ch           Command handle previously returned by RF_postCmd().
 *  @param mode         1: Stop gracefully, 0: abort abruptly
 *  @return             RF_Stat indicates if command was successfully completed
 */
extern RF_Stat DMMSch_rfFlushCmd(RF_Handle h, RF_CmdHandle ch, uint8_t mode);

/**
 *  @brief Send any Immediate command. <br>
 *
 *  Immediate Comamnd is send to RDBELL, if radio is active and the RF_Handle points
 *  to the current client. <br>
 *  In other appropriate RF_Stat values are returned. <br>
 *
 *  @note Calling context : Task/SWI/HWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param pCmdStruct   Pointer to the immediate command structure
 *  @return             RF_Stat indicates if command was successfully completed
*/
extern RF_Stat DMMSch_rfRunImmediateCmd(RF_Handle h, uint32_t* pCmdStruct);

/**
 *  @brief Send any Direct command. <br>
 *
 *  Direct Comamnd value is send to RDBELL immediately, if radio is active and
 *  the RF_Handle point to the current client. <br>
 *  In other appropriate RF_Stat values are returned. <br>
 *
 *  @note Calling context : Task/SWI/HWI
 *
 *  @param h            Handle previously returned by RF_open()
 *  @param cmd          Direct command value.
 *  @return             RF_Stat indicates if command was successfully completed.
*/
extern RF_Stat DMMSch_rfRunDirectCmd(RF_Handle h, uint32_t cmd);


extern RF_Stat DMMSch_rfRequestAccess(RF_Handle h, RF_AccessParams *pParams);

#endif /* DMMSch_H_ */
