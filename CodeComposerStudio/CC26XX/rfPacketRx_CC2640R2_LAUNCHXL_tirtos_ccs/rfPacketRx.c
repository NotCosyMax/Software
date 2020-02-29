/*
 * Copyright (c) 2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include <ti/drivers/PWM.h>

/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */



/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

Semaphore_Struct semStruct;
Semaphore_Handle semHandle;
PWM_Handle pwm1 = NULL;
PWM_Handle pwm2 = NULL;
PWM_Handle pwm3 = NULL;
uint16_t   dutyInc = 4;
uint16_t   pwmPeriod = 1000;

typedef struct
{
    uint32_t cylinder;
    uint8_t mode;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} slushData_t;

volatile slushData_t cylinderData = {0};

long HSBtoRGB(float _hue, float _sat, float _brightness) {
   float red = 0.0;
   float green = 0.0;
   float blue = 0.0;

   if (_sat == 0.0) {
       red = _brightness;
       green = _brightness;
       blue = _brightness;
   } else {
       if (_hue == 360.0) {
           _hue = 0;
       }

       int slice = _hue / 60.0;
       float hue_frac = (_hue / 60.0) - slice;

       float aa = _brightness * (1.0 - _sat);
       float bb = _brightness * (1.0 - _sat * hue_frac);
       float cc = _brightness * (1.0 - _sat * (1.0 - hue_frac));

       switch(slice) {
           case 0:
               red = _brightness;
               green = cc;
               blue = aa;
               break;
           case 1:
               red = bb;
               green = _brightness;
               blue = aa;
               break;
           case 2:
               red = aa;
               green = _brightness;
               blue = cc;
               break;
           case 3:
               red = aa;
               green = bb;
               blue = _brightness;
               break;
           case 4:
               red = cc;
               green = aa;
               blue = _brightness;
               break;
           case 5:
               red = _brightness;
               green = aa;
               blue = bb;
               break;
           default:
               red = 0.0;
               green = 0.0;
               blue = 0.0;
               break;
       }
   }

   long ired = red * 255.0;
   long igreen = green * 255.0;
   long iblue = blue * 255.0;

   return ((long) ((ired << 16) | (igreen << 8) | (iblue)));
}

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    Semaphore_Params semParams;
    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 0, &semParams);

    /* Obtain instance handle */
    semHandle = Semaphore_handle(&semStruct);

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;
    RF_cmdPropRx.pktConf.bChkAddress = 0x0;
    RF_cmdPropRx.address0 = 0xC1;
    RF_cmdPropRx.address1 = 0xDD;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Period and duty in microseconds */
    uint32_t counter = 0;
    uint32_t numColorsScaler = 16;
    uint32_t animationDelay = 20; // number milliseconds before RGB LED changes to next color

    /* Sleep time in microseconds */
    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);
    if (pwm1 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }

    PWM_start(pwm1);

    pwm2 = PWM_open(Board_PWM1, &params);
    if (pwm2 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }

    PWM_start(pwm2);

    pwm3 = PWM_open(Board_PWM2, &params);
     if (pwm3 == NULL) {
         /* Board_PWM0 did not open */
         while (1);
     }

     PWM_start(pwm3);

     /* Set the frequency */
     RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

     /* Enter RX mode and stay forever in RX */
     RF_EventMask terminationReason = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                RF_PriorityNormal, &callback,
                                                RF_EventRxEntryDone);


     float colorNumber = 0.0;
     float saturation = 0.0;
     float brightness = 0.0;
     float hue = 0.0;
    /* Loop forever incrementing the PWM duty */
    while (1) {
        if (cylinderData.mode == 0) {
            colorNumber = counter > (cylinderData.blue * numColorsScaler) ? counter - (cylinderData.blue * numColorsScaler): counter;
            if (Semaphore_pend(semHandle, BIOS_NO_WAIT)) {
                saturation = ((float) cylinderData.red) / 255.0; // Between 0 and 1 (0 = gray, 1 = full color)
                brightness = ((float) cylinderData.green) / 255.0; // Between 0 and 1 (0 = dark, 1 is full brightness)
            }
            hue = (colorNumber / ((float) (cylinderData.blue * numColorsScaler))) * 360; // Number between 0 and 360
            long color = HSBtoRGB(hue, saturation, brightness);
            // Get the red, blue and green parts from generated color
            int red = color >> 16 & 255;
            int green = color >> 8 & 255;
            int blue = color & 255;

            PWM_setDuty(pwm1, ((dutyInc * red) > pwmPeriod) ? pwmPeriod : (dutyInc * red));
            PWM_setDuty(pwm2, ((dutyInc * green) > pwmPeriod) ? pwmPeriod : (dutyInc * green));
            PWM_setDuty(pwm3, ((dutyInc * blue) > pwmPeriod) ? pwmPeriod : (dutyInc * blue));

            counter = (counter + 1) % ((cylinderData.blue * numColorsScaler) * 2);
            Task_sleep(animationDelay*100);
        }
        else {
            Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
            PWM_setDuty(pwm1, ((dutyInc * cylinderData.red) > pwmPeriod) ? pwmPeriod : (dutyInc * cylinderData.red));
            PWM_setDuty(pwm2, ((dutyInc * cylinderData.green) > pwmPeriod) ? pwmPeriod : (dutyInc * cylinderData.green));
            PWM_setDuty(pwm3, ((dutyInc * cylinderData.blue) > pwmPeriod) ? pwmPeriod : (dutyInc * cylinderData.blue));
        }
    }
}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        if (packetLength == 5) {
            cylinderData.mode = *(packetDataPointer + 1);
            cylinderData.red = *(packetDataPointer + 2);
            cylinderData.green = *(packetDataPointer + 3);
            cylinderData.blue = *(packetDataPointer + 4);

            Semaphore_post(semHandle);
            if (cylinderData.mode == 0) {
                Semaphore_post(semHandle);
            }
        }
        RFQueue_nextEntry();
    }
}
