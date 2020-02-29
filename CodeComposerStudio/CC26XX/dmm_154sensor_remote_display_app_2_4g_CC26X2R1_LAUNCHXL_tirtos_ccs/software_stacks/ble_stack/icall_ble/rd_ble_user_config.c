/******************************************************************************

 @file  rd_ble_user_config.c

 @brief This file contains user configurable variables for the BLE
        Application.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2016-2019, Texas Instruments Incorporated
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

/*******************************************************************************
 * INCLUDES
 */

#include "hal_types.h"
#include "ble_user_config.h"
#include <ti/sysbios/BIOS.h>

#ifndef USE_DMM
#include <ti/drivers/rf/RF.h>
#else
#include "rf_mac_api.h"
#endif //USE_DMM

#include "ecc/ECCROMCC26XX.h"

#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/drivers/utils/Random.h>
#include <ti/drivers/TRNG.h>

#include <driverlib/pka.h>

#include "simple_peripheral.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

// Tx Power
#define NUM_TX_POWER_VALUES (sizeof(TxPowerTable) / sizeof(txPwrVal_t))

// Default Tx Power Index
#if defined(CC13X2P)
#define DEFAULT_TX_POWER               4 //HCI_EXT_TX_POWER_0_DBM
#else // !CC13X2
#define DEFAULT_TX_POWER               4 //HCI_EXT_TX_POWER_0_DBM
#endif // CC13X2

// Override NOP
#define OVERRIDE_NOP                   0xC0000001

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void driverTable_fnSpinlock(void);

/*******************************************************************************
 * GLOBAL VARIABLES
 */

// RF Override Registers
// Note: Used with CMD_RADIO_SETUP; called at boot time and after wake.
// Note: Must be in RAM as these overrides may need to be modified at runtime
//       based on temperature compensation, although it is possible this may
//       be automated in CM0 in PG2.0.
#if defined(USE_FPGA)
  regOverride_t pOverridesCommon[] = {
  #if defined(CC26X2)
    // CC2642, as update by Helge on 12/12/17: Common Overrides for BLE5
    0x000151D0,
    0x00041110,
    0x00000083,
    0x00800403,
    0x80000303,
    0x02980243,
    0x01080263,
    0x08E90AA3,
    0x00068BA3,
    0x0E490C83,
    0x00005100, // Update matched filter for wired input
    0x721C5104, // Update matched filter for wired input
    0x00725108, // Update matched filter for wired input
    0x48f450d4, // Update matched filter gain for wired input
    END_OVERRIDE };
  #endif // CC26X2

  regOverride_t pOverrides1Mbps[] = {
    0x02405320,
    0x010302A3,
    END_OVERRIDE };

  #if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & (PHY_2MBPS_CFG | PHY_LR_CFG))
  regOverride_t pOverrides2Mbps[] = {
    0x02405320,
    0x00B502A3,
    END_OVERRIDE };

  regOverride_t pOverridesCoded[] = {
    0x01013487,
    0x02405320,
    0x069802A3,
    END_OVERRIDE };
  #endif // PHY_2MBPS_CFG | PHY_LR_CFG

  #if defined(CC13X2P)
  // high gain PA overrides
  regOverride_t pOverridesTx20[] = {
    TX20_POWER_OVERRIDE(0x1234),
    0x01C20703, // Function of loDivider, frontEnd, and PA (High)
    END_OVERRIDE };

  // default PA overrides
  regOverride_t pOverridesTxStd[] = {
    TX_STD_POWER_OVERRIDE(0x1234),
    0x05320703, // Function loDivider, frontEnd, and PA (Default)
    END_OVERRIDE };
  #endif //CC13X2P

#elif defined(CC26XX)
  // CC26xx Normal Package with Flash Settings for 48 MHz device
  #if defined(CC26X2)
  regOverride_t pOverridesCommon[] = {
	// override_ble5_setup_override_common.xml
    // DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0x3 (DITHER_EN=0 and IPEAK=3).
    (uint32_t)0x00F388D3,
    // Bluetooth 5: Set pilot tone length to 20 us Common
    HW_REG_OVERRIDE(0x6024,0x2E20),
    // Bluetooth 5: Compensate for reduced pilot tone length
    (uint32_t)0x01280263,
    // Bluetooth 5: Default to no CTE.
    HW_REG_OVERRIDE(0x5328,0x0000),
    END_OVERRIDE };

   regOverride_t pOverrides1Mbps[] = {
    // override_ble5_setup_override_1mbps.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x015302A3,
     END_OVERRIDE };

   #if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & (PHY_2MBPS_CFG | PHY_LR_CFG))
   regOverride_t pOverrides2Mbps[] = {
    // override_ble5_setup_override_2mbps.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeAdjust
    (uint32_t)0x00F102A3,
     END_OVERRIDE };

   regOverride_t pOverridesCoded[] = {
    // override_ble5_setup_override_coded.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x07A902A3,
    // Rx: Set AGC reference level to 0x1B (default: 0x2E)
    HW_REG_OVERRIDE(0x609C,0x001B),
     END_OVERRIDE };
   #endif // PHY_2MBPS_CFG | PHY_LR_CFG

   #if defined(RTLS_CTE)
   // CTE overrides
   regOverride_t pOverridesCte[] = {
     0x00158000, // S2RCFG: Capture S2R from FrontEnd, on event (CM0 will arm)
     0x000E51D0, // After FRAC
     ((CTE_CONFIG << 16) | 0x8BB3), // Enable CTE capture
     ((CTE_OFFSET << 24) | ((CTE_SAMPLING_CONFIG_1MBPS | (CTE_SAMPLING_CONFIG_2MBPS << 4)) << 16) | 0x0BC3), // Sampling rate, offset
     0xC0040341, // Pointer to antenna switching table in next entry
     (uint32_t) NULL, // Pointer to antenna switching table
     END_OVERRIDE };
   #endif
  #else // unknown device package
    #error "***BLE USER CONFIG BUILD ERROR*** Unknown package type!"
  #endif // <board>

#elif defined(CC13XX)
  #if defined(CC13X2)
  regOverride_t pOverridesCommon[] = {
    // override_ble5_setup_override_common.xml
    // DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0x3 (DITHER_EN=0 and IPEAK=3).
    (uint32_t)0x00F388D3,
    // Bluetooth 5: Set pilot tone length to 20 us Common
    HW_REG_OVERRIDE(0x6024,0x2E20),
    // Bluetooth 5: Compensate for reduced pilot tone length
    (uint32_t)0x01280263,
    // Bluetooth 5: Default to no CTE.
    HW_REG_OVERRIDE(0x5328,0x0000),
    END_OVERRIDE };

  regOverride_t pOverrides1Mbps[] = {
    // override_ble5_setup_override_1mbps.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x015302A3,
    END_OVERRIDE };

  #if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & (PHY_2MBPS_CFG | PHY_LR_CFG))
  regOverride_t pOverrides2Mbps[] = {
    // override_ble5_setup_override_2mbps.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeAdjust
    (uint32_t)0x00F102A3,
    END_OVERRIDE };

  regOverride_t pOverridesCoded[] = {
    // override_ble5_setup_override_coded.xml
    // Bluetooth 5: Set pilot tone length to 20 us
    HW_REG_OVERRIDE(0x5320,0x03C0),
    // Bluetooth 5: Compensate syncTimeadjust
    (uint32_t)0x07A902A3,
    // Rx: Set AGC reference level to 0x1B (default: 0x2E)
    HW_REG_OVERRIDE(0x609C,0x001B),
    END_OVERRIDE };
  #endif // PHY_2MBPS_CFG | PHY_LR_CFG

  #if defined(RTLS_CTE)
  // CTE overrides
  regOverride_t pOverridesCte[] = {
    0x00158000, // S2RCFG: Capture S2R from FrontEnd, on event (CM0 will arm)
    0x000E51D0, // After FRAC
    ((CTE_CONFIG << 16) | 0x8BB3), // Enable CTE capture
    ((CTE_OFFSET << 24) | ((CTE_SAMPLING_CONFIG_1MBPS | (CTE_SAMPLING_CONFIG_2MBPS << 4)) << 16) | 0x0BC3), // Sampling rate, offset
    0xC0040341, // Pointer to antenna switching table in next entry
    (uint32_t) NULL, // Pointer to antenna switching table
    END_OVERRIDE };
  #endif

  #elif defined(CC13X2P)
  regOverride_t pOverridesCommon[] = {
    // override_ble5_setup_override_common_hpa.xml
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x6024,0x5B20),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x01640263,
    // Bluetooth 5: Set IPEAK = 3 and DCDC dither off for TX
    (uint32_t)0x00F388D3,
    // Bluetooth 5: Default to no CTE.
    HW_REG_OVERRIDE(0x5328,0x0000),
    END_OVERRIDE };

  regOverride_t pOverrides1Mbps[] = {
    // override_ble5_setup_override_1mbps_hpa.xml
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x018F02A3,
    END_OVERRIDE };

  #if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & (PHY_2MBPS_CFG | PHY_LR_CFG))
  regOverride_t pOverrides2Mbps[] = {
    // override_ble5_setup_override_2mbps_hpa.xml
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x012D02A3,
    END_OVERRIDE };

  regOverride_t pOverridesCoded[] = {
    // override_ble5_setup_override_coded_hpa.xml
    // Bluetooth 5: Reconfigure pilot tone length for high output power PA
    HW_REG_OVERRIDE(0x5320,0x0690),
    // Bluetooth 5: Compensate for modified pilot tone length
    (uint32_t)0x07E502A3,
    // Bluetooth 5: Set AGC mangnitude target to 0x1B.
    HW_REG_OVERRIDE(0x609C,0x001B),
    END_OVERRIDE };
  #endif // PHY_2MBPS_CFG | PHY_LR_CFG

  // high gain PA overrides
  regOverride_t pOverridesTx20[] = {
    // The TX Power element should always be the first in the list
    TX20_POWER_OVERRIDE(0x003F75F5),
    // The ANADIV radio parameter based on the LO divider (0) and front-end (0) settings
    (uint32_t)0x01C20703,
    // Bluetooth 5: Set RTIM offset to 3 for high power PA
    (uint32_t)0x00038783,
    // Bluetooth 5: Set synth mux for high power PA
    (uint32_t)0x010206C3,
    END_OVERRIDE };

  // default PA overrides
  regOverride_t pOverridesTxStd[] = {
    // The TX Power element should always be the first in the list
    TX_STD_POWER_OVERRIDE(0x7217),
    // The ANADIV radio parameter based on the LO divider (0) and front-end (0) settings
    (uint32_t)0x05320703,
    // Use normal RTRIM for standard PA
    (uint32_t) 0x00008783,
    // Bluetooth 5: Set synth mux to default value for standard PA
    (uint32_t)0x050206C3,
    END_OVERRIDE };

  #if defined(RTLS_CTE)
  // CTE overrides
  regOverride_t pOverridesCte[] = {
    0x00158000, // S2RCFG: Capture S2R from FrontEnd, on event (CM0 will arm)
    0x000E51D0, // After FRAC
    ((CTE_CONFIG << 16) | 0x8BB3), // Enable CTE capture
    ((CTE_OFFSET << 24) | ((CTE_SAMPLING_CONFIG_1MBPS | (CTE_SAMPLING_CONFIG_2MBPS << 4)) << 16) | 0x0BC3), // Sampling rate, offset
    0xC0040341, // Pointer to antenna switching table in next entry
    (uint32_t) NULL, // Pointer to antenna switching table
    END_OVERRIDE };
  #endif

  #else // unknown board package
    #error "***BLE USER CONFIG BUILD ERROR*** Unknown board type!"
  #endif // <board>

#else // unknown platform

  #error "ERROR: Unknown platform!"

#endif // <board>

//
// Tx Power Table Used Depends on Device Package
//

#if defined(USE_FPGA)

  // Differential Output (same as CC2650EM_7ID for now)
  // Tx Power Values (Pout, Tx Power)
  const txPwrVal_t TxPowerTable[] =
  { { TX_POWER_MINUS_21_DBM, 0x06C7 },
    { TX_POWER_MINUS_18_DBM, 0x06C9 },
    { TX_POWER_MINUS_15_DBM, 0x0C88 },
    { TX_POWER_MINUS_12_DBM, 0x108A },
    { TX_POWER_MINUS_9_DBM,  0x0A8D },
    { TX_POWER_MINUS_6_DBM,  0x204D },
    { TX_POWER_MINUS_3_DBM,  0x2851 },
    { TX_POWER_0_DBM,        0x3459 },
    { TX_POWER_1_DBM,        0x385C },
    { TX_POWER_2_DBM,        0x440D },
    { TX_POWER_3_DBM,        0x5411 },
    { TX_POWER_4_DBM,        0x6C16 },
    { TX_POWER_5_DBM,        0x941E } };

#elif defined(CC26XX)

  #if defined(CC26X2)

  // Differential Output
  // Tx Power Values (Pout, Tx Power)
  const txPwrVal_t TxPowerTable[] =
  { { TX_POWER_MINUS_20_DBM,  RF_TxPowerTable_DefaultPAEntry(6, 3, 0, 2)  },
    { TX_POWER_MINUS_15_DBM,  RF_TxPowerTable_DefaultPAEntry(10, 3, 0, 3) },
    { TX_POWER_MINUS_10_DBM,  RF_TxPowerTable_DefaultPAEntry(15, 3, 0, 5) },
    { TX_POWER_MINUS_5_DBM,   RF_TxPowerTable_DefaultPAEntry(22, 3, 0, 9) },
    { TX_POWER_0_DBM,         RF_TxPowerTable_DefaultPAEntry(19, 1, 0, 20) },
    { TX_POWER_1_DBM,         RF_TxPowerTable_DefaultPAEntry(22, 1, 0, 20) },
    { TX_POWER_2_DBM,         RF_TxPowerTable_DefaultPAEntry(25, 1, 0, 25) },
    { TX_POWER_3_DBM,         RF_TxPowerTable_DefaultPAEntry(29, 1, 0, 28) },
    { TX_POWER_4_DBM,         RF_TxPowerTable_DefaultPAEntry(35, 1, 0, 39) },
    { TX_POWER_5_DBM,         RF_TxPowerTable_DefaultPAEntry(23, 0, 0, 57) } };

  #else // unknown board package

  #error "***BLE USER CONFIG BUILD ERROR*** Unknown CC26x2 board type!"

  #endif // <board>

#elif defined(CC13XX)

  #if defined(CC13X2)
  // Tx Power Values (Pout, Tx Power)
  const txPwrVal_t TxPowerTable[] =
    { { TX_POWER_MINUS_20_DBM, RF_TxPowerTable_DefaultPAEntry(6, 3, 0, 2)  },
      { TX_POWER_MINUS_15_DBM, RF_TxPowerTable_DefaultPAEntry(10, 3, 0, 3) },
      { TX_POWER_MINUS_10_DBM, RF_TxPowerTable_DefaultPAEntry(15, 3, 0, 5) },
      { TX_POWER_MINUS_5_DBM,  RF_TxPowerTable_DefaultPAEntry(22, 3, 0, 9) },
      { TX_POWER_0_DBM,        RF_TxPowerTable_DefaultPAEntry(19, 1, 0, 20) },
      { TX_POWER_1_DBM,        RF_TxPowerTable_DefaultPAEntry(22, 1, 0, 20) },
      { TX_POWER_2_DBM,        RF_TxPowerTable_DefaultPAEntry(25, 1, 0, 25) },
      { TX_POWER_3_DBM,        RF_TxPowerTable_DefaultPAEntry(29, 1, 0, 28) },
      { TX_POWER_4_DBM,        RF_TxPowerTable_DefaultPAEntry(35, 1, 0, 39) },
      { TX_POWER_5_DBM,        RF_TxPowerTable_DefaultPAEntry(23, 0, 0, 57) } };

 #elif defined(CC13X2P)
 // Tx Power Values (Pout, Tx Power)
 const txPwrVal_t TxPowerTable[] =
   { {TX_POWER_MINUS_20_DBM, RF_TxPowerTable_DefaultPAEntry( 6, 3, 0, 2 ) },    // 0x000006C7
     {TX_POWER_MINUS_15_DBM, RF_TxPowerTable_DefaultPAEntry(10, 3, 0, 3 ) },    // 0x00000C88
     {TX_POWER_MINUS_10_DBM, RF_TxPowerTable_DefaultPAEntry(15, 3, 0, 5 ) },    // 0x0000168C
     {TX_POWER_MINUS_5_DBM,  RF_TxPowerTable_DefaultPAEntry(22, 3, 0, 9 ) },    // 0x0000224E
     {TX_POWER_0_DBM,        RF_TxPowerTable_DefaultPAEntry(19, 1, 0, 20) },    // 0x00003459
     {TX_POWER_1_DBM,        RF_TxPowerTable_DefaultPAEntry(22, 1, 0, 20) },    // 0x0000385C
     {TX_POWER_2_DBM,        RF_TxPowerTable_DefaultPAEntry(25, 1, 0, 25) },    // 0x0000440D
     {TX_POWER_3_DBM,        RF_TxPowerTable_DefaultPAEntry(29, 1, 0, 28) },    // 0x00005411
     {TX_POWER_4_DBM,        RF_TxPowerTable_DefaultPAEntry(35, 1, 0, 39) },    // 0x00006C16
     {TX_POWER_5_DBM,        RF_TxPowerTable_DefaultPAEntry(23, 0, 0, 57) },    // 0x0000941E
     {TX_POWER_14_DBM,       RF_TxPowerTable_HighPAEntry(22, 3, 1, 19, 27) },   // 0x801B2798
     {TX_POWER_15_DBM,       RF_TxPowerTable_HighPAEntry(26, 3, 1, 23, 27) },   // 0x80272997
     {TX_POWER_16_DBM,       RF_TxPowerTable_HighPAEntry(30, 3, 1, 28, 27) },   // 0x801735A2
     {TX_POWER_17_DBM,       RF_TxPowerTable_HighPAEntry(37, 3, 1, 39, 27) },   // 0x801943A6
     {TX_POWER_18_DBM,       RF_TxPowerTable_HighPAEntry(32, 3, 1, 35, 48) },   // 0x80354B9E
     {TX_POWER_19_DBM,       RF_TxPowerTable_HighPAEntry(34, 3, 1, 48, 63) },   // 0x803B73A4
     {TX_POWER_20_DBM,       RF_TxPowerTable_HighPAEntry(53, 3, 1, 58, 63) } }; // 0x803F5BB8

  #else // unknown board package

  #error "***BLE USER CONFIG BUILD ERROR*** Unknown CC135x board type!"

  #endif // <board>

#else // unknown platform

  #error "ERROR: Unknown platform!"

#endif // <board>

// Tx Power Table
txPwrTbl_t appTxPwrTbl = { TxPowerTable,
                           NUM_TX_POWER_VALUES,  // max
                           DEFAULT_TX_POWER};    // default

ECCParams_CurveParams eccParams_NISTP256 = {
    .curveType      = ECCParams_CURVE_TYPE_SHORT_WEIERSTRASS,
    .length         = NISTP256_PARAM_SIZE_BYTES,
    .prime          = NISTP256_prime.byte,
    .order          = NISTP256_order.byte,
    .a              = NISTP256_a.byte,
    .b              = NISTP256_b.byte,
    .generatorX     = NISTP256_generator.x.byte,
    .generatorY     = NISTP256_generator.y.byte
};

#ifdef ICALL_JT
#include <icall.h>

// RF Driver API Table
rfDrvTblPtr_t rfDriverTableBLE[] =
  { (uint32)RF_open,
    (uint32)driverTable_fnSpinlock, // RF_close
#ifdef RF_SINGLEMODE
    (uint32)RF_postCmd,
#else // !RF_SINGLEMODE
    (uint32)driverTable_fnSpinlock, // RF_postCmd
#endif // RF_SINGLEMODE
    (uint32)driverTable_fnSpinlock, // RF_pendCmd
#ifdef RF_SINGLEMODE
    (uint32)RF_runCmd,
#else // !RF_SINGLEMODE
    (uint32)driverTable_fnSpinlock, // RF_runCmd
#endif // RF_SINGLEMODE
    (uint32)RF_cancelCmd,
    (uint32)RF_flushCmd,
    (uint32)driverTable_fnSpinlock, // RF_yield
    (uint32)RF_Params_init,
    (uint32)RF_runImmediateCmd,
    (uint32)RF_runDirectCmd,
    (uint32)RF_ratCompare,
    (uint32)driverTable_fnSpinlock, // RF_ratCapture
    (uint32)RF_ratDisableChannel,
    (uint32)RF_getCurrentTime,
    (uint32)RF_getRssi,
    (uint32)RF_getInfo,
    (uint32)RF_getCmdOp,
    (uint32)RF_control,
    (uint32)driverTable_fnSpinlock, // RF_getTxPower
    (uint32)RF_setTxPower, // RF_setTxPower
    (uint32)driverTable_fnSpinlock, // RF_TxPowerTable_findPowerLevel
    (uint32)driverTable_fnSpinlock, // RF_TxPowerTable_findValue
#ifndef RF_SINGLEMODE
    (uint32)RF_scheduleCmd,
    (uint32)RF_runScheduleCmd,
    (uint32)driverTable_fnSpinlock, // RF_requestAccess
#endif // !RF_SINGLEMODE
  };

cryptoDrvTblPtr_t cryptoDriverTableBLE[] =
  { (uint32)AESCCM_init,
    (uint32)AESCCM_open,
    (uint32)AESCCM_close,
    (uint32)AESCCM_Params_init,
    (uint32)AESCCM_Operation_init,
    (uint32)AESCCM_oneStepEncrypt,
    (uint32)AESCCM_oneStepDecrypt,
    (uint32)AESECB_init,
    (uint32)AESECB_open,
    (uint32)AESECB_close,
    (uint32)AESECB_Params_init,
    (uint32)AESECB_Operation_init,
    (uint32)AESECB_oneStepEncrypt,
    (uint32)AESECB_oneStepDecrypt,
    (uint32)CryptoKeyPlaintext_initKey,
    (uint32)CryptoKeyPlaintext_initBlankKey};

// Swi APIs needed by BLE controller
rtosApiTblPtr_t rtosApiTable[] =
{
  (uint32_t) Swi_disable,
  (uint32_t) Swi_restore
};

// BLE Stack Configuration Structure
const stackSpecific_t bleStackConfig =
{
  .maxNumConns                          = MAX_NUM_BLE_CONNS,
  .maxNumPDUs                           = MAX_NUM_PDU,
  .maxPduSize                           = MAX_PDU_SIZE,
  .maxNumPSM                            = L2CAP_NUM_PSM,
  .maxNumCoChannels                     = L2CAP_NUM_CO_CHANNELS,
  .maxWhiteListElems                    = MAX_NUM_WL_ENTRIES,
  .maxResolvListElems                   = MAX_NUM_RL_ENTRIES,
  .pfnBMAlloc                           = &pfnBMAlloc,
  .pfnBMFree                            = &pfnBMFree,
  .rfDriverParams.powerUpDurationMargin = RF_POWER_UP_DURATION_MARGIN,
  .rfDriverParams.inactivityTimeout     = RF_INACTIVITY_TIMEOUT,
  .rfDriverParams.powerUpDuration       = RF_POWER_UP_DURATION,
  .rfDriverParams.pErrCb                = &(RF_ERR_CB),
  .eccParams                            = &eccParams_NISTP256,
  .fastStateUpdateCb                    = SimplePeripheral_bleFastStateUpdateCb,
  .bleStackType                         = DMMPolicy_StackRole_BlePeripheral
};

uint16_t bleUserCfg_maxPduSize = MAX_PDU_SIZE;

#ifdef OSAL_SNV_EXTFLASH
const extflashDrvTblPtr_t extflashDriverTable[] =
{
  (uint32) ExtFlash_open,
  (uint32) ExtFlash_close,
  (uint32) ExtFlash_read,
  (uint32) ExtFlash_write,
  (uint32) ExtFlash_erase
};
#endif // OSAL_SNV_EXTFLASH

// Table for Driver can be found in icall_user_config.c
// if a driver is not to be used, then the pointer shoul dbe set to NULL,
// for this example, this is done in ble_user_config.h
const drvTblPtr_t driverTable =
{
  .rfDrvTbl         = rfDriverTableBLE,
  .eccDrvTbl        = eccDriverTable,
  .cryptoDrvTbl     = cryptoDriverTableBLE,
  .trngDrvTbl       = trngDriverTable,
  .rtosApiTbl       = rtosApiTable,
  .nvintfStructPtr  = &nvintfFncStruct,
#ifdef OSAL_SNV_EXTFLASH
  .extflashDrvTbl = extflashDriverTable,
#endif // OSAL_SNV_EXTFLASH
};

const boardConfig_t boardConfig =
{
  .rfFeModeBias = RF_FE_MODE_AND_BIAS,
  .rfRegTbl      = pOverridesCommon,
  .rfRegTbl1M    = pOverrides1Mbps,
#if defined(BLE_V50_FEATURES) && (BLE_V50_FEATURES & (PHY_2MBPS_CFG | PHY_LR_CFG))
  // Currently, no overrides for 2M and Coded, so exclude from build.
  .rfRegTbl2M    = pOverrides2Mbps,
  .rfRegTblCoded = pOverridesCoded,
#endif // PHY_2MBPS_CFG | PHY_LR_CFG
  .txPwrTbl      = &appTxPwrTbl,
#if defined(CC13X2P)
  .rfRegOverrideTxStdTblptr  = pOverridesTxStd,   // Default PA
  .rfRegOverrideTx20TblPtr  =  pOverridesTx20   ,// High power PA
#endif //CC13X2P
#if defined(RTLS_CTE)
  .rfRegOverrideCtePtr = pOverridesCte,
#else
  .rfRegOverrideCtePtr = NULL,
#endif
};

#else /* !(ICALL_JT) */

// RF Driver API Table
rfDrvTblPtr_t rfDriverTable[] =
{
  (uint32) RF_open,
  (uint32) driverTable_fnSpinlock, // RF_close
#ifdef RF_SINGLEMODE
  (uint32) RF_postCmd,
#else // !RF_SINGLEMODE
  (uint32) driverTable_fnSpinlock, // RF_postCmd
#endif// RF_SINGLEMODE
  (uint32) driverTable_fnSpinlock, // RF_pendCmd
#ifdef RF_SINGLEMODE
  (uint32) RF_runCmd,
#else // !RF_SINGLEMODE
  (uint32) driverTable_fnSpinlock, // RF_runCmd
#endif// RF_SINGLEMODE
  (uint32) RF_cancelCmd,
  (uint32) RF_flushCmd,
  (uint32) driverTable_fnSpinlock, // RF_yield
  (uint32) RF_Params_init,
  (uint32) RF_runImmediateCmd,
  (uint32) RF_runDirectCmd,
  (uint32) RF_ratCompare
  (uint32) driverTable_fnSpinlock, // RF_ratCapture
  (uint32) driverTable_fnSpinlock, // RF_ratDisableChannel
  (uint32) RF_getCurrentTime,
  (uint32) RF_getRssi,
  (uint32) RF_getInfo,
  (uint32) RF_getCmdOp,
  (uint32) RF_control,
  (uint32) driverTable_fnSpinlock, // RF_getTxPower
  (uint32) RF_setTxPower, // RF_setTxPower
  (uint32) driverTable_fnSpinlock, // RF_TxPowerTable_findPowerLevel
  (uint32) driverTable_fnSpinlock, // RF_TxPowerTable_findValue
#ifndef RF_SINGLEMODE
  (uint32) RF_scheduleCmd,
  (uint32) RF_runScheduleCmd,
  (uint32) driverTable_fnSpinlock  // RF_requestAccess
#endif // !RF_SINGLEMODE
};

// ECC Driver API Table
eccDrvTblPtr_t eccDriverTable[] =
{
  (uint32) ECDH_init,
  (uint32) ECDH_Params_init,
  (uint32) ECDH_open,
  (uint32) ECDH_close,
  (uint32) ECDH_OperationGeneratePublicKey_init,
  (uint32) ECDH_OperationComputeSharedSecret_init,
  (uint32) ECDH_generatePublicKey,
  (uint32) ECDH_computeSharedSecret
};

// Crypto Driver API Table
cryptoDrvTblPtr_t cryptoDriverTable[] =
{
  (uint32)AESCCM_init,
  (uint32)AESCCM_open,
  (uint32)AESCCM_close,
  (uint32)AESCCM_Params_init,
  (uint32)AESCCM_Operation_init,
  (uint32)AESCCM_oneStepEncrypt,
  (uint32)AESCCM_oneStepDecrypt,
  (uint32)AESECB_init,
  (uint32)AESECB_open,
  (uint32)AESECB_close,
  (uint32)AESECB_Params_init,
  (uint32)AESECB_Operation_init,
  (uint32)AESECB_oneStepEncrypt,
  (uint32)AESECB_oneStepDecrypt,
  (uint32)CryptoKeyPlaintext_initKey,
  (uint32)CryptoKeyPlaintext_initBlankKey
};

trngDrvTblPtr_t trngDriverTable[] =
{
  (uint32) TRNG_init,
  (uint32) TRNG_open,
  (uint32) TRNG_generateEntropy,
  (uint32) TRNG_close
};


#endif /* ICALL_JT */

/*******************************************************************************
 * @fn          RegisterAssertCback
 *
 * @brief       This routine registers the Application's assert handler.
 *
 * input parameters
 *
 * @param       appAssertHandler - Application's assert handler.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void RegisterAssertCback(assertCback_t appAssertHandler)
{
  appAssertCback = appAssertHandler;

#ifdef EXT_HAL_ASSERT
  // also set the Assert callback pointer used by halAssertHandlerExt
  // Note: Normally, this pointer will be intialized by the stack, but in the
  //       event HAL_ASSERT is used by the Application, we initialize it
  //       directly here.
  halAssertCback = appAssertHandler;
#endif // EXT_HAL_ASSERT

  return;
}

/*******************************************************************************
 * @fn          driverTable_fnSpinLock
 *
 * @brief       This routine is used to trap calls to unpopulated indexes of
 *              driver function pointer tables.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void driverTable_fnSpinlock(void)
{
  volatile uint8 i = 1;

  while(i);
}

/*******************************************************************************
 * @fn          DefaultAssertCback
 *
 * @brief       This is the Application default assert callback, in the event
 *              none is registered.
 *
 * input parameters
 *
 * @param       assertCause    - Assert cause as defined in hal_assert.h.
 * @param       assertSubcause - Optional assert subcause (see hal_assert.h).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void DefaultAssertCback(uint8 assertCause, uint8 assertSubcause)
{
#ifdef HAL_ASSERT_SPIN
  driverTable_fnSpinlock();
#endif // HAL_ASSERT_SPIN

  return;
}

// Application Assert Callback Function Pointer
assertCback_t appAssertCback = DefaultAssertCback;

/*******************************************************************************
 */
