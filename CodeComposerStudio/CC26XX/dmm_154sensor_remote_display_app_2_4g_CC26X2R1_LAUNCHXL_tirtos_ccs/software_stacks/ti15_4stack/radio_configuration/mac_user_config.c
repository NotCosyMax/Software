/******************************************************************************

 @file  mac_user_config.c

 @brief User configurable variables for the TIMAC radio.

 Group: WCS LPC
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

/******************************************************************************
 * INCLUDES
 */

#include "Board.h"
#include "mac_user_config.h"
#if !defined(DeviceFamily_CC13X2) && !defined(DeviceFamily_CC26X2)
#include <ti/drivers/crypto/CryptoCC26XX.h>
#else
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/AESECB.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#endif

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

#if defined( DeviceFamily_CC13X0 )
#include "mac_user_config_cc13x0_2_4g.h"
#elif defined( DeviceFamily_CC26X0 )
#include "mac_user_config_cc26x0_2_4g.h"
#elif defined( DeviceFamily_CC26X2 )
#include "mac_user_config_cc26x2r1_2_4g.h"
#elif defined( DeviceFamily_CC13X2 )
#include "mac_user_config_cc13x2r1_2_4g.h"
#else // unknown device package
#error "***MAC USER CONFIG BUILD ERROR*** Unknown package type!"
#endif

// RF Driver API Table
const uint32_t macRfDriverTable[] =
  { (uint32_t) RF_open,
    (uint32_t) RF_close,
    (uint32_t) RF_postCmd,
	(uint32_t) NULL,
	(uint32_t) NULL,
    (uint32_t) RF_cancelCmd,
    (uint32_t) RF_flushCmd,
    (uint32_t) RF_yield,
    (uint32_t) RF_Params_init,
    (uint32_t) RF_runImmediateCmd,
    (uint32_t) RF_runDirectCmd,
    (uint32_t) RF_ratCompare,
	(uint32_t) NULL,
	(uint32_t) RF_RatConfigOutput_init,
    (uint32_t) RF_ratDisableChannel,
    (uint32_t) RF_getCurrentTime,
    (uint32_t) RF_getRssi,
    (uint32_t) RF_getInfo,
    (uint32_t) RF_getCmdOp,
    (uint32_t) RF_control,
    (uint32_t) RF_scheduleCmd,
    (uint32_t) RF_getTxPower,
    (uint32_t) RF_setTxPower,
    (uint32_t) NULL,
    (uint32_t) RF_TxPowerTable_findValue,
    (uint32_t) RF_ScheduleCmdParams_init,
    (uint32_t) RF_requestAccess };

// Crypto Driver API Table
#if !defined(DeviceFamily_CC13X2) && !defined(DeviceFamily_CC26X2)
const uint32_t macCryptoDriverTable[] =
  { (uint32_t) CryptoCC26XX_close,
    (uint32_t) CryptoCC26XX_init,
    (uint32_t) CryptoCC26XX_open,
    (uint32_t) CryptoCC26XX_Params_init,
    (uint32_t) CryptoCC26XX_Transac_init,
    (uint32_t) CryptoCC26XX_allocateKey,
    (uint32_t) CryptoCC26XX_releaseKey,
    (uint32_t) CryptoCC26XX_transact,
    (uint32_t) CryptoCC26XX_transactPolling,
    (uint32_t) CryptoCC26XX_transactCallback };
#else
const uint32_t macCryptoDriverTable[] =
  {   (uint32_t) AESCCM_init,
      (uint32_t) AESCCM_open,
      (uint32_t) AESCCM_close,
      (uint32_t) AESCCM_Params_init,
      (uint32_t) AESCCM_Operation_init,
      (uint32_t) AESCCM_oneStepEncrypt,
      (uint32_t) AESCCM_oneStepDecrypt,
#if defined(FEATURE_SECURE_COMMISSIONING) || defined(USE_DMM) || defined(ZSTACK_AESECB)
      (uint32_t) AESECB_init,
      (uint32_t) AESECB_open,
      (uint32_t) AESECB_close,
      (uint32_t) AESECB_Params_init,
      (uint32_t) AESECB_Operation_init,
      (uint32_t) AESECB_oneStepEncrypt,
      (uint32_t) AESECB_oneStepDecrypt,
#else
      (uint32_t) NULL,
      (uint32_t) NULL,
      (uint32_t) NULL,
      (uint32_t) NULL,
      (uint32_t) NULL,
      (uint32_t) NULL,
      (uint32_t) NULL,
#endif
      (uint32_t) CryptoKeyPlaintext_initKey,
      (uint32_t) CryptoKeyPlaintext_initBlankKey
  };
#endif
/******************************************************************************
 */

