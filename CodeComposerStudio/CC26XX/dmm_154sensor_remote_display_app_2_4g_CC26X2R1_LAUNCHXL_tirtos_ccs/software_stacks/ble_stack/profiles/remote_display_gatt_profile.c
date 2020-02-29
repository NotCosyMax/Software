/******************************************************************************

 @file  remote_display_gatt_profile.c

 @brief This file contains the 15.4 Remote Display GATT profile implementation.


 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2010-2019, Texas Instruments Incorporated
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
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "remote_display_gatt_profile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Remote Display GATT Profile Service UUID
CONST uint8 remoteDisplayProfileServUUID[ATT_UUID_SIZE] =
{ 
  RDPROFILE_SERV_UUID_BASE128(RDPROFILE_SERV_UUID)
};

// Sensor Data Characteristic UUID
CONST uint8 remoteDisplayProfileSensorDataCharUUID[ATT_UUID_SIZE] =
{
  RDPROFILE_CHAR_UUID_BASE128(RDPROFILE_SENSOR_DATA_CHAR_UUID)
};

// Sensor Report Interval Characteristic UUID
CONST uint8 remoteDisplayProfileSensorReportIntervalCharUUID[ATT_UUID_SIZE] =
{ 
  RDPROFILE_CHAR_UUID_BASE128(RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR_UUID)
};

// Sensor Report Interval Characteristic UUID
CONST uint8 remoteDisplayProfileCollLedCharUUID[ATT_UUID_SIZE] =
{
  RDPROFILE_CHAR_UUID_BASE128(RDPROFILE_COLL_LED_CHAR_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static remoteDisplayProfileCBs_t *remoteDisplayProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

//  Remote Display Profile Service attribute
static CONST gattAttrType_t remoteDisplayProfileService = { ATT_UUID_SIZE, remoteDisplayProfileServUUID };

// Remote Display Profile Sensor Data Characteristic Properties
static uint8 remoteDisplayProfileSensorDataCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Sensor Data Characteristic Value
static uint8 remoteDisplayProfileSensorDataCharVal[RDPROFILE_SENSOR_DATA_LEN];

// Remote Display Profile Sensor Data  Characteristic Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *remoteDisplayProfileSensorDataConfig;

// Remote Display Profile Sensor Data Characteristic User Description
static uint8 remoteDisplayProfileSensorDataUserDesp[RDPROFILE_MAX_DESCRIPTION_LEN] = "Sensor Data";

// Remote Display Profile Sensor Report Interval Characteristic Properties
static uint8 remoteDisplayProfileSensorReportIntervalCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Sensor Report Interval Characteristic Value
static uint8 remoteDisplayProfileSensorReportIntervalCharVal[RDPROFILE_SENSOR_REPORT_INTERVAL_LEN];

// Remote Display Profile Sensor Report Interval Characteristic User Description
static uint8 remoteDisplayProfileSensorReportIntervalUserDesp[RDPROFILE_MAX_DESCRIPTION_LEN] = "Sensor Report Interval";

// Remote Display Profile Collector LED Characteristic Properties
static uint8 remoteDisplayProfileCollLedCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Collector LED Characteristic Value
static uint8 remoteDisplayProfileCollLedCharVal = 0;

// Remote Display Profile Collector LED Characteristic User Description
static uint8 remoteDisplayProfileCollLedUserDesp[RDPROFILE_MAX_DESCRIPTION_LEN] = "LED Identify";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t remoteDisplayProfileAttrTbl[] =
{
  // Remote Display Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&remoteDisplayProfileService     /* pValue */
  },
      // Sensor Data Characteristic Declaration
      {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &remoteDisplayProfileSensorDataCharProps
      },

        // Sensor Data Characteristic Value
        {
          { ATT_UUID_SIZE, remoteDisplayProfileSensorDataCharUUID },
          GATT_PERMIT_READ,
          0,
          remoteDisplayProfileSensorDataCharVal
        },

        // Sensor Data Characteristic configuration
        {
          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0,
          (uint8 *)&remoteDisplayProfileSensorDataConfig
        },

        // Sensor Data Characteristic User Description
        {
          { ATT_BT_UUID_SIZE, charUserDescUUID },
          GATT_PERMIT_READ,
          0,
          remoteDisplayProfileSensorDataUserDesp
        },

    // Sensor Report Interval Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &remoteDisplayProfileSensorReportIntervalCharProps
    },

      // Sensor Report Interval Characteristic Value
      { 
        { ATT_UUID_SIZE, remoteDisplayProfileSensorReportIntervalCharUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        remoteDisplayProfileSensorReportIntervalCharVal
      },

      // Sensor Report Interval Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        remoteDisplayProfileSensorReportIntervalUserDesp
      },      

    // Coll LED Toggle Characteristic 1 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &remoteDisplayProfileCollLedCharProps
    },

      // Sensor Report Interval Characteristic Value
      {
        { ATT_UUID_SIZE, remoteDisplayProfileCollLedCharUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &remoteDisplayProfileCollLedCharVal
      },

      // Sensor Report Interval Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        remoteDisplayProfileCollLedUserDesp
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t remoteDisplayProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t remoteDisplayProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Remote Display Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and 
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the 
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an 
// operation on a characteristic requires authorization the Stack will call 
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be 
// made within these functions.
CONST gattServiceCBs_t remoteDisplayProfileCBs =
{
  remoteDisplayProfile_ReadAttrCB,  // Read callback function pointer
  remoteDisplayProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t RemoteDisplay_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  remoteDisplayProfileSensorDataConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            MAX_NUM_BLE_CONNS );
  if ( remoteDisplayProfileSensorDataConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LL_CONNHANDLE_INVALID, remoteDisplayProfileSensorDataConfig );
  
  if ( services & RDPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( remoteDisplayProfileAttrTbl, 
                                          GATT_NUM_ATTRS( remoteDisplayProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &remoteDisplayProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t RemoteDisplay_RegisterAppCBs( remoteDisplayProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    remoteDisplayProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RemoteDisplay_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR:
      if ( len == RDPROFILE_SENSOR_REPORT_INTERVAL_LEN )
      {
          VOID memcpy(remoteDisplayProfileSensorReportIntervalCharVal, value, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case RDPROFILE_COLL_LED_CHAR:
      if ( len == sizeof ( uint8 ) )
      {
        remoteDisplayProfileCollLedCharVal = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case RDPROFILE_SENSOR_DATA_CHAR:
      if ( len == RDPROFILE_SENSOR_DATA_LEN )
      {
        VOID memcpy(remoteDisplayProfileSensorDataCharVal, value, RDPROFILE_SENSOR_DATA_LEN);

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( remoteDisplayProfileSensorDataConfig, remoteDisplayProfileSensorDataCharVal, FALSE,
                                    remoteDisplayProfileAttrTbl, GATT_NUM_ATTRS( remoteDisplayProfileAttrTbl ),
                                    INVALID_TASK_ID, remoteDisplayProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      RemoteDisplay_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RemoteDisplay_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR:
      VOID memcpy(value, remoteDisplayProfileSensorReportIntervalCharVal, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN);
      break;
    case RDPROFILE_COLL_LED_CHAR:
      *((uint8*)value) = remoteDisplayProfileCollLedCharVal;
      break;
    case RDPROFILE_SENSOR_DATA_CHAR:
      VOID memcpy(value, remoteDisplayProfileSensorDataCharVal, RDPROFILE_SENSOR_DATA_LEN);
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t remoteDisplayProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  bStatus_t status = SUCCESS;
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    if(!memcmp(pAttr->type.uuid, remoteDisplayProfileCollLedCharUUID, pAttr->type.len))
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads
      *pLen = RDPROFILE_GENERIC_LEN;
      pValue[0] = *pAttr->pValue;
    }
    else if(!memcmp(pAttr->type.uuid, remoteDisplayProfileSensorReportIntervalCharUUID, pAttr->type.len))
    {
      *pLen = RDPROFILE_SENSOR_REPORT_INTERVAL_LEN;
      VOID memcpy( pValue, pAttr->pValue, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN );
    }
    else if(!memcmp(pAttr->type.uuid, remoteDisplayProfileSensorDataCharUUID, pAttr->type.len))
    {
      *pLen = RDPROFILE_SENSOR_DATA_LEN;
      VOID memcpy( pValue, pAttr->pValue, RDPROFILE_SENSOR_DATA_LEN );
    }
    else
    {
      // Should never get here! (other characteristics do not have read permissions)
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t remoteDisplayProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    if(!memcmp(pAttr->type.uuid, remoteDisplayProfileSensorReportIntervalCharUUID, pAttr->type.len))
    {
      //Validate the value
      // Make sure it's not a blob operation
      if ( offset == 0 )
      {
        if ( len != RDPROFILE_SENSOR_REPORT_INTERVAL_LEN )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        VOID memcpy( pAttr->pValue, pValue, RDPROFILE_SENSOR_REPORT_INTERVAL_LEN );

        notifyApp = RDPROFILE_SENSOR_REPORT_INTERVAL_CHAR;
      }
    }
    else if(!memcmp(pAttr->type.uuid, remoteDisplayProfileCollLedCharUUID, pAttr->type.len))
    {
      //Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != RDPROFILE_COLL_LED_LEN )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        *pCurValue = pValue[0];

        notifyApp = RDPROFILE_COLL_LED_CHAR;
      }
    }
    else
    {
      // Should never get here! (characteristics 2 and 4 do not have write permissions)
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else if (( pAttr->type.len == ATT_BT_UUID_SIZE ) &&
          (BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]) == GATT_CLIENT_CHAR_CFG_UUID))
  {
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY );
  }
  else
  {
    // unsupported length
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && remoteDisplayProfile_AppCBs && remoteDisplayProfile_AppCBs->pfnRemoteDispalyProfileChange )
  {
    remoteDisplayProfile_AppCBs->pfnRemoteDispalyProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/