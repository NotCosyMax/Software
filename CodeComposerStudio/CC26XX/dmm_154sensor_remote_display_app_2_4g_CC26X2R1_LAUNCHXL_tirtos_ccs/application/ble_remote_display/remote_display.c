/******************************************************************************

 @file  remote_display.c

 @brief This file contains the 15.4 Remote Display sample application for use
        with the CC13x2 Bluetooth Low Energy Protocol Stack.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <remote_display_gatt_profile.h>
#include <provisioning_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <Board.h>
#include <board_key.h>

#include "util.h"
#include "remote_display.h"

#include <dmm/dmm_policy_blesp_154sensor.h>
#include "ssf.h"

#ifdef RF_PROFILING
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "Board.h"
#endif

#include "sensor.h"

/*********************************************************************
 * MACROS
 */

// Connection indices map to events by shifting 1 left by the index
#define CONN_INDEX_TO_EVENT(x)             (1 << x)

// Count the trailing zeros to map back from event to index
#if defined __TI_COMPILER_VERSION__
#define CONN_EVENT_TO_INDEX(x) (sizeof(unsigned int) * 8 - 1 - __clz((unsigned int) x))
#else
#define CONN_EVENT_TO_INDEX(x) (sizeof(unsigned int) * 8 - 1 - __CLZ((unsigned int) x))
#endif

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     104

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          600

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// How often to perform periodic event (in ms)
#define RD_PERIODIC_EVT_PERIOD               5000

#ifdef DISPLAY_PER_STATS
// How often to perform PER read event (in ms)
#define RD_PER_EVT_PERIOD                    1000
#endif /* DISPLAY_PER_STATS */

// How often to read current current RPA (in ms)
#define RD_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update request
#define RD_SEND_PARAM_UPDATE_DELAY           6000

// Task configuration
#define RD_TASK_PRIORITY                     1

#ifndef RD_TASK_STACK_SIZE
#define RD_TASK_STACK_SIZE                   1024
#endif

// Application events
#define RD_STATE_CHANGE_EVT                  0
#define RD_CHAR_CHANGE_EVT                   1
#define PROV_CHAR_CHANGE_EVT                 2
#define RD_KEY_CHANGE_EVT                    3
#define RD_ADV_EVT                           4
#define RD_PAIR_STATE_EVT                    5
#define RD_PASSCODE_EVT                      6
#define RD_READ_RPA_EVT                      7
#define RD_SEND_PARAM_UPDATE_EVT             8
#define RD_UPDATE_PROV_SENSOR_EVT            9
#define RD_UPDATE_PROV_STATE_EVT             10
#define RD_UPDATE_RD_DATA_EVT                11
#define RD_POLICY_PAUSE_EVT                  12

#ifdef DISPLAY_PER_STATS
#define RD_UPDATE_PER_EVT                    13
#define RD_PER_RESET_EVT                     14
#define RD_PER_READ_EVT                      15
#endif /* DISPLAY_PER_STATS */


// Internal Events for RTOS application
#define RD_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define RD_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define RD_ALL_EVENTS                        (RD_ICALL_EVT             | \
                                              RD_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define RD_ADDR_STR_SIZE     15

// Row numbers for display
#define RD_ROW_TITLE            0
#define RD_ROW_SEPARATOR_1      1
#define RD_ROW_STATUS_1         2
#define RD_ROW_STATUS_2         3
#define RD_ROW_CONNECTION       4
#define RD_ROW_ADVSTATE         5
#define RD_ROW_IDA              6
#define RD_ROW_RPA              7
#define RD_ROW_WSN_DATA         8
#define RD_ROW_SENSOR_STATS_1   9
#define RD_ROW_SENSOR_STATS_2   10
#define RD_ROW_SENSOR_STATS_3   11
#define RD_ROW_DEBUG            12

// For storing the active connections
#define RD_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define RD_MAX_RSSI_STORE_DEPTH    5
#define RD_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30
#define RSSI_1M_THRSHLD           -40
#define RSSI_S2_THRSHLD           -50
#define RSSI_S8_THRSHLD           -60
#define RD_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define REMOTEDISPLAY_ASSERT(expr) if (!(expr)) HAL_ASSERT_SPINLOCK;
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} rdEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} rdPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} rdPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} rdGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} rdClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t connHandle;
} rdConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t              connHandle;                        // Connection Handle
  rdClockEventData_t*   pParamUpdateEventData;
  Clock_Struct*         pUpdateClock;                      // Pointer to clock struct
} rdConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration
Task_Struct rdTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(rdTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t rdTaskStack[RD_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;
#ifdef DISPLAY_PER_STATS
// Clock instance for PER update events.
static Clock_Struct clkReadPer;
#endif /* DISPLAY_PER_STATS */


// Memory to pass RPA read event ID to clock handler
rdClockEventData_t argRpaRead =
{ .event = RD_READ_RPA_EVT };

#ifdef DISPLAY_PER_STATS
// Memory to pass PER event ID to clock handler
rdClockEventData_t argReadPer =
{ .event = RD_PER_READ_EVT };
#endif /* DISPLAY_PER_STATS */


// Per-handle connection info
static rdConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "DMM 15.4 Sensor RD";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(PROVPROFILE_SERV_UUID),
  HI_UINT16(PROVPROFILE_SERV_UUID)
};

// Scan Response Data
static uint8_t scanRspData[] =
{
 // complete name
 19,   // length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'D',
 'M',
 'M',
 ' ',
 '1',
 '5',
 '.',
 '4',
 ' ',
 'S',
 'e',
 'n',
 's',
 'o',
 'r',
 ' ',
 'R',
 'D',
  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

#ifdef RF_PROFILING
/* Pin driver handle */
static PIN_Handle rfBleProfilingPinHandle;
static PIN_State rfBleProfilingPinState;

#define BLE_CONNECTED_GPIO  Board_DIO22

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config rfBleProfilingPinTable[] =
{
    BLE_CONNECTED_GPIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
#endif

static RemoteDisplay_clientProvisioningtCbs_t clientProvCbs;
static RemoteDisplayCbs_t remoteDisplayCbs;

static bool dmmPolicyBlePaused = false;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void RemoteDisplay_init( void );
static void RemoteDisplay_taskFxn(UArg a0, UArg a1);
static uint8_t RemoteDisplay_processStackMsg(ICall_Hdr *pMsg);
static uint8_t RemoteDisplay_processGATTMsg(gattMsgEvent_t *pMsg);
static void RemoteDisplay_processGapMessage(gapEventHdr_t *pMsg);
static void RemoteDisplay_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void  RemoteDisplay_processAdvEvent(rdGapAdvEventData_t *pEventData);
static void RemoteDisplay_processAppMsg(rdEvt_t *pMsg);
static void RemoteDisplay_processRDCharValueChangeEvt(uint8_t paramId);
static void RemoteDisplay_processProvCharValueChangeEvt(uint8_t paramId);
static void RemoteDisplay_updateRPA(void);
static void RemoteDisplay_clockHandler(UArg arg);
static void RemoteDisplay_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void RemoteDisplay_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void RemoteDisplay_processPairState(rdPairStateData_t *pPairState);
static void RemoteDisplay_processPasscode(rdPasscodeData_t *pPasscodeData);
static void RemoteDisplay_charValueChangeCB(uint8_t paramId);
static void Provisioning_charValueChangeCB(uint8_t paramId);
static void RemoteDisplay_enqueueMsg(uint8_t event, void *pData);
static void RemoteDisplay_handleKeys(uint8_t keys);
static uint8_t RemoteDisplay_addConn(uint16_t connHandle);
static uint8_t RemoteDisplay_getConnIndex(uint16_t connHandle);
static uint8_t RemoteDisplay_removeConn(uint16_t connHandle);
static void RemoteDisplay_processParamUpdate(uint16_t connHandle);
static uint8_t RemoteDisplay_clearConnListEntry(uint16_t connHandle);
static void RemoteDisplay_blePausePolicyCb(uint16_t pause);
static void RemoteDisplay_syncProvAttr(void);
static void RemoteDisplay_syncRDAttr(void);
#ifdef DISPLAY_PER_STATS
static void RemoteDisplay_processUpdatePerEvt(void);
static void RemoteDisplay_processVSCmdCompleteEvt(hciEvt_VSCmdComplete_t *pMsg);
#endif /* DISPLAY_PER_STATS */


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t RemoteDisplay_BondMgrCBs =
{
  RemoteDisplay_passcodeCb,       // Passcode callback
  RemoteDisplay_pairStateCb       // Pairing/Bonding state Callback
};

// Provisioning Profile Callbacks
static provisioningProfileCBs_t Provisioning_ProfileCBs =
{
  Provisioning_charValueChangeCB // Provisioning GATT Characteristic value change callback
};

// Remote Display GATT Profile Callbacks
static remoteDisplayProfileCBs_t RemoteDisplay_ProfileCBs =
{
  RemoteDisplay_charValueChangeCB // Remote Display GATT Characteristic value change callback
};


/*********************************************************************
 * DMM Policy Callbacks
 */
static DMMPolicy_AppCbs_t dmmPolicyAppCBs =
{
    RemoteDisplay_blePausePolicyCb
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RemoteDisplay_registerClientProvCbs
 *
 * @brief   Register the 15.4 provisioning client callbacks
 */
void RemoteDisplay_registerClientProvCbs(RemoteDisplay_clientProvisioningtCbs_t clientCbs)
{
    clientProvCbs = clientCbs;
}

/*********************************************************************
 * @fn      RemoteDisplay_registerRDCbs
 *
 * @brief   Register the 15.4 Remote display callbacks
 */
void RemoteDisplay_registerRDCbs(RemoteDisplayCbs_t rdCbs)
{
    remoteDisplayCbs = rdCbs;
}

/*********************************************************************
 * @fn      RemoteDisplay_createTask
 *
 * @brief   Task creation function for the Remote Display.
 */
void RemoteDisplay_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = rdTaskStack;
  taskParams.stackSize = RD_TASK_STACK_SIZE;
  taskParams.priority = RD_TASK_PRIORITY;

  Task_construct(&rdTask, RemoteDisplay_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 *  @fn      RemoteDisplay_updateSensorJoinState
 *
 * @brief   Updates the sensor state value within the provisioning profile
 */
void RemoteDisplay_updateSensorJoinState(Jdllc_states_t state)
{
    uint8_t *pValue = ICall_malloc(sizeof(Jdllc_states_t));

    if (pValue)
    {
      *pValue = (uint8_t) state;
      RemoteDisplay_enqueueMsg(RD_UPDATE_PROV_STATE_EVT, pValue);
    }
}

#ifdef DISPLAY_PER_STATS
/*********************************************************************
 *  @fn      RemoteDisplay_updatePER
 *
 * @brief   Updates the display with new PER values
 */
void RemoteDisplay_updatePER(RemoteDisplay_perData_t* pData)
{
    RemoteDisplay_perData_t *pValue = ICall_malloc(sizeof(RemoteDisplay_perData_t));

    if (pValue)
    {
      memcpy(pValue, pData, sizeof(RemoteDisplay_perData_t));
      RemoteDisplay_enqueueMsg(RD_UPDATE_PER_EVT, pValue);
    }
}
#endif /* DISPLAY_PER_STATS */

/*********************************************************************
 *  @fn      RemoteDisplay_updateSensorData
 *
 * @brief   Updates the sensor data within the remote display profile
 */
void RemoteDisplay_updateSensorData(void)
{
  RemoteDisplay_enqueueMsg(RD_UPDATE_RD_DATA_EVT, NULL);
}


/*********************************************************************
 *  @fn      RemoteDisplay_syncProvAttr
 *
 * @brief   Helper function to initialize all provisioning profile characteristic
 *          values based on the 15.4 application.
 */
void RemoteDisplay_syncRDAttr(void)
{
    uint8_t newSensorData[RDPROFILE_SENSOR_DATA_LEN];
    uint8_t newReportInterval[RDPROFILE_SENSOR_REPORT_INTERVAL_LEN];

    // Callback has not been properly initialized
    if (!remoteDisplayCbs.getRDAttrCb) {
        return;
    }

    // Data from 15.4 Application
    remoteDisplayCbs.getRDAttrCb(RemoteDisplayAttr_ReportInterval, newReportInterval,
        RDPROFILE_SENSOR_REPORT_INTERVAL_LEN);
    // Write data to BLE remote display profile
        RemoteDisplay_SetParameter(RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN,
                                   newReportInterval);

    // Data from 15.4 Application
    remoteDisplayCbs.getRDAttrCb(RemoteDisplayAttr_SensorData, newSensorData,
        RDPROFILE_SENSOR_DATA_LEN);
    // Write data to BLE remote display profile
    RemoteDisplay_SetParameter(RDPROFILE_SENSOR_DATA_CHAR, RDPROFILE_SENSOR_DATA_LEN,
                                   newSensorData);
}



/*********************************************************************
 *  @fn      RemoteDisplay_syncProvAttr
 *
 * @brief   Helper function to initialize all provisioning profile characteristic
 *          values based on the 15.4 application.
 */
void RemoteDisplay_syncProvAttr(void)
{
    uint8_t newGenericAttr;
    uint8_t newPanId[PROVPROFILE_NTWK_PAN_ID_CHAR_LEN];
    uint8_t newExtPanId[PROVPROFILE_EXT_NTWK_PAN_ID_CHAR_LEN];
    uint8_t newChannMask[PROVPROFILE_SENSOR_CHANNEL_CHAR_LEN];
    uint8_t newFfdAddr[PROVPROFILE_IEEE_FFD_ADDR_CHAR_LEN];
    uint8_t newNtwkKey[PROVPROFILE_NTWK_KEY_CHAR_LEN];
    
    // Callback has not been properly initialized
    if (!clientProvCbs.getProvisioningAttrCb) {
        return;
    }

    // Data from 15.4 Application
    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_Freq, &newGenericAttr,
        PROVISIONING_GENERIC_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_SENSOR_FREQ_CHAR, sizeof(uint8_t),
                                         &newGenericAttr);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_PanId, newPanId,
        PROVPROFILE_NTWK_PAN_ID_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_NTWK_PAN_ID_CHAR, PROVPROFILE_NTWK_PAN_ID_CHAR_LEN,
                                         newPanId);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_ExtPanId, newExtPanId,
        PROVPROFILE_EXT_NTWK_PAN_ID_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_EXT_NTWK_PAN_ID_CHAR, PROVPROFILE_EXT_NTWK_PAN_ID_CHAR_LEN,
                                         newExtPanId);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_SensorChannelMask, newChannMask,
        PROVPROFILE_SENSOR_CHANNEL_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_SENSOR_CHANNEL_CHAR, PROVPROFILE_SENSOR_CHANNEL_CHAR_LEN,
                                         newChannMask);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_FFDAddr, newFfdAddr,
        PROVPROFILE_IEEE_FFD_ADDR_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_IEEE_FFD_ADDR_CHAR, PROVPROFILE_IEEE_FFD_ADDR_CHAR_LEN,
                                         newFfdAddr);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_NtwkKey, newNtwkKey,
        PROVPROFILE_NTWK_KEY_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_NTWK_KEY_CHAR, PROVPROFILE_NTWK_KEY_CHAR_LEN,
                                         newNtwkKey);

    clientProvCbs.getProvisioningAttrCb(ProvisionAttr_ProvState, &newGenericAttr,
        PROVISIONING_GENERIC_CHAR_LEN);
    // Write data to BLE provisioning profile
    ProvisioningProfile_SetParameter(PROVPROFILE_PROV_STATE_CHAR, sizeof(uint8_t),
                                         &newGenericAttr);
}

/*********************************************************************
 * @fn      RemoteDisplay_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void RemoteDisplay_init(void)
{
#ifdef RF_PROFILING
    /* Open LED pins */
    rfBleProfilingPinHandle = PIN_open(&rfBleProfilingPinState, rfBleProfilingPinTable);
    /* Clear Debug pins */
    PIN_setOutputValue(rfBleProfilingPinHandle, BLE_CONNECTED_GPIO, 0);
#endif

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

#ifdef DISPLAY_PER_STATS
  // Construct the clock that controls the PER display period
  Util_constructClock(&clkReadPer, RemoteDisplay_clockHandler,
                      RD_PER_EVT_PERIOD, 0 , false, (UArg)&argReadPer);
#endif /* DISPLAY_PER_STATS */

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  RemoteDisplay_AddService(GATT_ALL_SERVICES); // Remote Display GATT Profile
  ProvisioningProfile_AddService(GATT_ALL_SERVICES); // Provisioning GATT Profile

  // Initialize provisioning profile attributes based on 15.4 application values
  RemoteDisplay_syncProvAttr();

  // Initialize remote display profile attributes based on 15.4 application values
  RemoteDisplay_syncRDAttr();

  // Register callback with Remote Display GATT profile
  RemoteDisplay_RegisterAppCBs(&RemoteDisplay_ProfileCBs);

  // Register callback with Provisioning GATT profile
  ProvisioningProfile_RegisterAppCBs(&Provisioning_ProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&RemoteDisplay_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Initialize Connection List
  RemoteDisplay_clearConnListEntry(LL_CONNHANDLE_ALL);

  // Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // The type of display is configured based on the BOARD_DISPLAY_USE
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // update display
  Display_printf(dispHandle, RD_ROW_TITLE, 0, "15.4 Sensor Remote Display");
  Display_printf(dispHandle, RD_ROW_SEPARATOR_1, 0, "==========================");

  // register the app callbacks
  DMMPolicy_registerAppCbs(dmmPolicyAppCBs, DMMPolicy_StackRole_BlePeripheral);
}

/*********************************************************************
 * @fn      RemoteDisplay_taskFxn
 *
 * @brief   Application task entry point for the Remote Display.
 *
 * @param   a0, a1 - not used.
 */
static void RemoteDisplay_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  RemoteDisplay_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, RD_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for non-BLE stack events
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = RemoteDisplay_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & RD_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          rdEvt_t *pMsg = (rdEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            RemoteDisplay_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RemoteDisplay_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      RemoteDisplay_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = RemoteDisplay_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;
#ifdef DISPLAY_PER_STATS
        case HCI_VE_EVENT_CODE:
          RemoteDisplay_processVSCmdCompleteEvt((hciEvt_VSCmdComplete_t *) pMsg);
          break;
#endif /* DISPLAY_PER_STATS */

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          //[TODO: the BLE simple peripheral changed this section. Need to merge.]
          break;
        }

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RemoteDisplay_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RemoteDisplay_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "TI BLE: FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "TI BLE: MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      RemoteDisplay_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void RemoteDisplay_processAppMsg(rdEvt_t *pMsg)
{
  bool dealloc = TRUE;

  switch (pMsg->event)
  {
    case RD_CHAR_CHANGE_EVT:
      RemoteDisplay_processRDCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case PROV_CHAR_CHANGE_EVT:
        RemoteDisplay_processProvCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case RD_KEY_CHANGE_EVT:
      RemoteDisplay_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case RD_ADV_EVT:
      RemoteDisplay_processAdvEvent((rdGapAdvEventData_t*)(pMsg->pData));
      break;

    case RD_PAIR_STATE_EVT:
      RemoteDisplay_processPairState((rdPairStateData_t*)(pMsg->pData));
      break;

    case RD_PASSCODE_EVT:
      RemoteDisplay_processPasscode((rdPasscodeData_t*)(pMsg->pData));
      break;

    case RD_READ_RPA_EVT:
      RemoteDisplay_updateRPA();
      break;

#ifdef DISPLAY_PER_STATS
    case RD_PER_READ_EVT:
      RemoteDisplay_processUpdatePerEvt();
      break;
#endif /* DISPLAY_PER_STATS */

    case RD_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((rdClockEventData_t *)pMsg->pData)->data);

      RemoteDisplay_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;

      break;
    }
    // Provisioning state update event
    case RD_UPDATE_PROV_STATE_EVT:
    {
        Jdllc_states_t state = *((Jdllc_states_t*)(pMsg->pData));

        Display_printf(dispHandle, RD_ROW_SENSOR_STATS_1, 0, "TI 15.4: Sensor State: %x", (uint8_t) state);

        if((state == Jdllc_states_joined) || (state == Jdllc_states_rejoined))
        {
            if(dmmPolicyBlePaused)
            {
                //unpause BLE
                DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_ADV);
                dmmPolicyBlePaused = false;
            }
        }

        // Re-sync 15.4 attributes with provisioning profile(15.4->BLE)
        RemoteDisplay_syncProvAttr();
        break;
    }

    // Remote display data update event
    case RD_UPDATE_RD_DATA_EVT:
    {
        // Re-sync 15.4 attributes with remote display profile(15.4->BLE)
        RemoteDisplay_syncRDAttr();
        break;
    }


    case RD_POLICY_PAUSE_EVT:
    {
        uint8_t numActive = linkDB_NumActive();
        uint16_t pause =  *((uint16_t*)(pMsg->pData));

        if(pause == DMMPOLICY_PAUSED)
        {
            if(numActive < MAX_NUM_BLE_CONNS)
            {
                // Stop advertising
                GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
            }

            //Stop terminate from starting advertisements
            dmmPolicyBlePaused = true;

            //terminate all connections
            GAP_TerminateLinkReq(LINKDB_CONNHANDLE_ALL, HCI_DISCONNECT_REMOTE_USER_TERM);
        }
        else
        {
            /* We should not have an active connection when resuming from a pause, but just
             * in case check max connections before starting advertising
             */
            if(numActive < MAX_NUM_BLE_CONNS)
            {
                GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
            }

            dmmPolicyBlePaused = false;
        }

        break;
    }

#ifdef DISPLAY_PER_STATS
    // PER update event
    case RD_UPDATE_PER_EVT:
    {
        RemoteDisplay_perData_t stats = *((RemoteDisplay_perData_t*)(pMsg->pData));
        
        Display_printf(dispHandle, RD_ROW_SENSOR_STATS_3, 0, "Sensor PER: %d%%", (unsigned int)stats.per);
        break;
    }
#endif /* DISPLAY_PER_STATS */

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void RemoteDisplay_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "TI BLE: Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;
        advParamLegacy.primIntMin = DEFAULT_ADVERTISING_INTERVAL;
        advParamLegacy.primIntMax = DEFAULT_ADVERTISING_INTERVAL;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&RemoteDisplay_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_ADV);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, RD_ROW_IDA, 0, "TI BLE: %s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        if (addrMode > ADDRMODE_RANDOM)
        {
          RemoteDisplay_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, RemoteDisplay_clockHandler,
                              RD_READ_RPA_EVT_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }
      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "TI BLE: Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        RemoteDisplay_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "TI BLE: Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
#ifdef DISPLAY_PER_STATS
        // Clear PER stats
        HCI_EXT_PacketErrorRateCmd(0, HCI_EXT_PER_RESET);
        // Start PER Clock.
        Util_startClock(&clkReadPer);
#endif /* DISPLAY_PER_STATS */
      }

      DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED);

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "TI BLE: Device Disconnected!");
      Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list and disable RSSI if needed
      RemoteDisplay_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);

#ifdef DISPLAY_PER_STATS
        // Clear PER stats
        HCI_EXT_PacketErrorRateCmd(0, HCI_EXT_PER_RESET);
        // Stop PER clock
        Util_stopClock(&clkReadPer);
#endif /* DISPLAY_PER_STATS */
      }

      if( (!dmmPolicyBlePaused) && (numActive < MAX_NUM_BLE_CONNS))
      {
          // Start advertising since there is room for more connections and we are not paused
          GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
          DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_ADV);
      }

      // Clear remaining lines
      Display_clearLine(dispHandle, RD_ROW_CONNECTION);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        // Display the address of the connection update
        Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "TI BLE: Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, RD_ROW_STATUS_2, 0,
                       "TI BLE: Link Param Update Failed 0x%h: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      rdConnHandleEntry_t *connHandleEntry = (rdConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
          // Attempt to send queued update now
          RemoteDisplay_processParamUpdate(connHandleEntry->connHandle);

          // Free list element
          ICall_free(connHandleEntry);
      }

      break;
    }

    default:
      Display_clearLines(dispHandle, RD_ROW_STATUS_1, RD_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 *  @fn      RemoteDisplay_bleFastStateUpdateCb
 *
 * @brief   Callback from BLE link layer to indicate a state change
 */
void RemoteDisplay_bleFastStateUpdateCb(uint32_t StackRole, uint32_t stackState)
{
  if(StackRole == DMMPolicy_StackRole_BlePeripheral)
  {
    static uint32_t prevStackState = 0;

    if( !(prevStackState & LL_TASK_ID_SLAVE) && (stackState & LL_TASK_ID_SLAVE))
    {
      //We are connecting
#ifdef RF_PROFILING
        PIN_setOutputValue(rfBleProfilingPinHandle, BLE_CONNECTED_GPIO, 1);
#endif

        /* update DMM policy */
        DMMPolicy_updateStackState(DMMPolicy_StackRole_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTING);
    }

    prevStackState = stackState;
  }
}
/*********************************************************************
 * @fn      RemoteDisplay_blePausePolicyCb
 *
 * @brief   DMM Policy callback to pause BLE Stack
 */
static void RemoteDisplay_blePausePolicyCb(uint16_t pause)
{
    uint16_t *pValue = ICall_malloc(sizeof(uint16_t));

    if (pValue)
    {
      *pValue = pause;

      RemoteDisplay_enqueueMsg(RD_POLICY_PAUSE_EVT, pValue);
    }
}

/*********************************************************************
 * @fn      RemoteDisplay_charValueChangeCB
 *
 * @brief   Callback from Remote Display Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void RemoteDisplay_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    RemoteDisplay_enqueueMsg(RD_CHAR_CHANGE_EVT, pValue);
  }
}

/*********************************************************************
 * @fn      Provisioning_charValueChangeCB
 *
 * @brief   Callback from provisioning Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void Provisioning_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    RemoteDisplay_enqueueMsg(PROV_CHAR_CHANGE_EVT, pValue);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processRDCharValueChangeEvt
 *
 * @brief   Process a pending Remote Display Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void RemoteDisplay_processRDCharValueChangeEvt(uint8_t paramId)
{
    // Remote display profile new parameters
    uint8_t newReportInterval[RDPROFILE_SENSOR_REPORT_INTERVAL_LEN];
    uint8_t newCollLedVal;

 switch(paramId)
  {
     // Remote display profile characteristics
     case RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR:
       // Obtain current value of parameter from profile
       RemoteDisplay_GetParameter(RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR, &newReportInterval);

       Display_printf(dispHandle, RD_ROW_SENSOR_STATS_2, 0, "TI 15.4: Sensor Reporting Interval: 0x%02x%02x%02x%02x",
           newReportInterval[0], newReportInterval[1], newReportInterval[2], newReportInterval[3]);

       // Call stack specific callback function to update parameter
       if(remoteDisplayCbs.setRDAttrCb != NULL)
       {
         remoteDisplayCbs.setRDAttrCb(RemoteDisplayAttr_ReportInterval, newReportInterval, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN);
       }
       break;   
     case RDPROFILE_COLL_LED_CHAR:
       // Obtain current value of parameter from profile
       RemoteDisplay_GetParameter(RDPROFILE_COLL_LED_CHAR, &newCollLedVal);

       Display_printf(dispHandle, RD_ROW_SENSOR_STATS_2, 0, "TI 15.4: Collector LED toggle: %02x", &newCollLedVal);

       // Call stack specific callback function to update parameter
       if(remoteDisplayCbs.setRDAttrCb != NULL)
       {
         remoteDisplayCbs.setRDAttrCb(RemoteDisplayAttr_CollLed, &newCollLedVal, RDPROFILE_COLL_LED_LEN);
       }
       break;

    default:
      // should not reach here!
      break;
  }
}


/*********************************************************************
 * @fn      RemoteDisplay_processProvCharValueChangeEvt
 *
 * @brief   Process a pending Remote Display Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void RemoteDisplay_processProvCharValueChangeEvt(uint8_t paramId)
{
    // Provisioning profile new parameters
    uint8_t newSensorFreq;
    uint8_t newProvSensor;
    uint8_t newPanId[PROVPROFILE_NTWK_PAN_ID_CHAR_LEN];
    uint8_t newExtPanId[PROVPROFILE_EXT_NTWK_PAN_ID_CHAR_LEN];
    uint8_t newSensorChannel[PROVPROFILE_SENSOR_CHANNEL_CHAR_LEN];
    uint8_t newIEEEFfdAddr[PROVPROFILE_IEEE_FFD_ADDR_CHAR_LEN];
    uint8_t newNtwkKey[PROVPROFILE_NTWK_KEY_CHAR_LEN];

 switch(paramId)
  {
     // Provisioning profile characteristics
     case PROVPROFILE_SENSOR_FREQ_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_SENSOR_FREQ_CHAR, &newSensorFreq);

       Display_printf(dispHandle, RD_ROW_SENSOR_STATS_2, 0, "TI 15.4: Sensor Frequency: %02x", &newSensorFreq);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_Freq, &newSensorFreq, PROVISIONING_GENERIC_CHAR_LEN);
       }
       break;
     case PROVPROFILE_PROV_SENSOR_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_PROV_SENSOR_CHAR, &newProvSensor);

       // Call stack specific callback function to update parameter
       if((clientProvCbs.provisionConnectCb != NULL) && (newProvSensor == PROVPROFILE_PAN_CONNECT))
       {
          clientProvCbs.provisionConnectCb();
       }
       else if((clientProvCbs.provisionDisconnectCb != NULL) && (newProvSensor == PROVPROFILE_PAN_DISCONNECT))
       {
          clientProvCbs.provisionDisconnectCb();
       }
       break;
     case PROVPROFILE_NTWK_PAN_ID_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_NTWK_PAN_ID_CHAR, newPanId);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_PanId, newPanId, PROVISIONING_NTWK_ID_LEN);
       }
       break;
     case PROVPROFILE_EXT_NTWK_PAN_ID_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_EXT_NTWK_PAN_ID_CHAR, newExtPanId);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_ExtPanId, newExtPanId, PROVISIONING_EXT_NTWK_ID_LEN);
       }

       break;
     case PROVPROFILE_SENSOR_CHANNEL_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_SENSOR_CHANNEL_CHAR, newSensorChannel);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_SensorChannelMask, newSensorChannel, PROVISIONING_NTWK_CHNL_LEN);
       }

       break;

     case PROVPROFILE_NTWK_KEY_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_NTWK_KEY_CHAR, newNtwkKey);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_NtwkKey, newNtwkKey, PROVISIONING_NTWK_KEY_LEN);
       }
       break;
     case PROVPROFILE_IEEE_FFD_ADDR_CHAR:
       // Obtain current value of parameter from profile
       ProvisioningProfile_GetParameter(PROVPROFILE_IEEE_FFD_ADDR_CHAR, newIEEEFfdAddr);

       // Call stack specific callback function to update parameter
       if(clientProvCbs.setProvisioningAttrCb != NULL)
       {
         clientProvCbs.setProvisioningAttrCb(ProvisionAttr_FFDAddr, newIEEEFfdAddr, PROVISIONING_IEEE_FFD_ADDR_LEN);
       }
       break;
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void RemoteDisplay_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    Display_printf(dispHandle, RD_ROW_RPA, 0, "TI BLE: RP Addr: %s",
                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void RemoteDisplay_clockHandler(UArg arg)
{
  rdClockEventData_t *pData = (rdClockEventData_t *)arg;

  if (pData->event == RD_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Post event to read the current RPA
    RemoteDisplay_enqueueMsg(RD_READ_RPA_EVT, NULL);
  }
  else if (pData->event == RD_SEND_PARAM_UPDATE_EVT)
  {
    // Send message to app
    RemoteDisplay_enqueueMsg(RD_SEND_PARAM_UPDATE_EVT, pData);
  }
#ifdef DISPLAY_PER_STATS
  else if (pData->event == RD_PER_READ_EVT)
  {
    // Satrt the next PER Read Period
    Util_startClock(&clkReadPer);
    // Post event to wake up the application
    RemoteDisplay_enqueueMsg(RD_PER_READ_EVT, NULL);
  }
#endif /* DISPLAY_PER_STATS */
}

/*********************************************************************
 * @fn      RemoteDisplay_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void RemoteDisplay_handleKeys(uint8_t keys)
{
    // Process keys for 15.4 application
    Ssf_processKeyChange(keys);
}

/*********************************************************************
 * @fn      RemoteDisplay_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void RemoteDisplay_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  rdGapAdvEventData_t *pData = ICall_malloc(sizeof(rdGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    RemoteDisplay_enqueueMsg(RD_ADV_EVT, pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void RemoteDisplay_processAdvEvent(rdGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "TI BLE: Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "TI BLE: Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "TI BLE: Adv Set %d disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
#endif
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      RemoteDisplay_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void RemoteDisplay_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  rdPairStateData_t *pData = ICall_malloc(sizeof(rdPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    RemoteDisplay_enqueueMsg(RD_PAIR_STATE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void RemoteDisplay_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  rdPasscodeData_t *pData = ICall_malloc(sizeof(rdPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    RemoteDisplay_enqueueMsg(RD_PASSCODE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void RemoteDisplay_processPairState(rdPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Pairing success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Encryption success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Bond save success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void RemoteDisplay_processPasscode(rdPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "TI BLE: Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      RemoteDisplay_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static void RemoteDisplay_enqueueMsg(uint8_t event, void *pData)
{
  rdEvt_t *pMsg = ICall_malloc(sizeof(rdEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LL_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(rdClockEventData_t) +
                                          sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
          connList[i].pParamUpdateEventData->event = RD_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                            RemoteDisplay_clockHandler,
                            RD_SEND_PARAM_UPDATE_DELAY, 0, true,
                            (UArg) connList[i].pParamUpdateEventData);
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      RemoteDisplay_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      RemoteDisplay_clearConnListEntry
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LL_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = RemoteDisplay_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LL_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LL_CONNHANDLE_INVALID;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      RemoteDisplay_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = RemoteDisplay_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear Connection List Entry
    RemoteDisplay_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      RemoteDisplay_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void RemoteDisplay_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = RemoteDisplay_getConnIndex(connHandle);
  REMOTEDISPLAY_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct
  ICall_free(connList[connIndex].pUpdateClock);
  connList[connIndex].pUpdateClock = NULL;
  // Free ParamUpdateEventData
  ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    rdConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(rdConnHandleEntry_t));
    if (connHandleEntry)
    {
        connHandleEntry->connHandle = connHandle;

        List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
}

#ifdef DISPLAY_PER_STATS
/*********************************************************************
 * @fn      RemoteDisplay_processVSCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI EXT Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void RemoteDisplay_processVSCmdCompleteEvt(hciEvt_VSCmdComplete_t *pMsg)
{
  uint8_t rowOffSet = 0;
  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
      case HCI_EXT_PER:
      {
          uint8_t status = pMsg->pEventParam[2];
          if (status == SUCCESS)
          {
              uint8_t cmdVal = pMsg->pEventParam[3];

              if (cmdVal == HCI_EXT_PER_READ)
              {
                uint16_t numPkts = BUILD_UINT16(pMsg->pEventParam[4], pMsg->pEventParam[5]);
                uint16_t numCrcErr = BUILD_UINT16(pMsg->pEventParam[6], pMsg->pEventParam[7]);
                uint16_t numEvents = BUILD_UINT16(pMsg->pEventParam[8], pMsg->pEventParam[9]);
                uint16_t numMissedEvents = BUILD_UINT16(pMsg->pEventParam[10], pMsg->pEventParam[11]);

                Display_printf(dispHandle, RD_ROW_DEBUG + rowOffSet++, 0, "BLE PER:");
                Display_printf(dispHandle, RD_ROW_DEBUG + rowOffSet++, 0, "NumPkts = %d", numPkts);
                Display_printf(dispHandle, RD_ROW_DEBUG + rowOffSet++, 0, "NumCrcErr = %d", numCrcErr);
                Display_printf(dispHandle, RD_ROW_DEBUG + rowOffSet++, 0, "NumEvents = %d",numEvents);
                Display_printf(dispHandle, RD_ROW_DEBUG + rowOffSet++, 0, "NumMissedEvents = %d", numMissedEvents);
              }
          }

          break;
      }
      default:
          break;
  }
}


/*********************************************************************
 * @fn      RemoteDisplay_processUpdatePerEvt
 *
 * @brief   Request a read of the PER counters for a connection.
 *
 * @return  HCI_SUCCESS
 */
static void RemoteDisplay_processUpdatePerEvt(void)
{
    HCI_EXT_PacketErrorRateCmd(0, HCI_EXT_PER_READ);
}
#endif  /* DISPLAY_PER_STATS */
/*********************************************************************
*********************************************************************/
