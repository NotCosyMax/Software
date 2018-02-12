/*
 * RTC_MCP7940M.c
 *
 *  A small code implementation to set/get the current time
 *  and trigger a interrupt each minute from the MCP7940M RTC.
 *
 *  Created on: 11 jan. 2018
 *      Author: Maxen
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include <I2C.h>
#include <RTC_MCP7940M.h>

/* ==================================== Local global variables ==================================== */

/* Pre-defined txBuffers */
const uint8_t checkIfRtcIsStarted[] = {0x00};
const uint8_t stopAndClearTheRtc[]  = {0x00, 0x00, 0x00};
const uint8_t startRtc[]            = {0x00, 0x80};
const uint8_t readTime[]            = {0x01, 0x02};

/* Local I2C buffers */
uint8_t rxBuffer[10];
uint8_t txBuffer[10];

/* ==================================== Function definition  ==================================== */

/* Initialize the RTC: setup the I2C given the SMCLK frequency */
void RTC_init(uint32_t clkFrequency)
{
    // Initialize I2C peripheral
    I2C_init(clkFrequency, I2C_STANDARD_SPEED, I2C_BLOCKING_MODE);
}

/* Checks of the RTC oscillator needs to be started, or of the RTC is running */
void RTC_start()
{
    /* Read register 0x00 containing ST bit */
    I2C_transfer(RTC_ADDR, sizeof(checkIfRtcIsStarted), (uint8_t *)checkIfRtcIsStarted, 1, rxBuffer);
    /* If bit 7 is set, RTC is already running and we don't need to start it */
    if (!(rxBuffer[0] & 0x80)) {
        /* Bit 7 was not set, start the RTC by writing bit 7.
           don't preserve the old time as this is already wrong. */
        I2C_transfer(RTC_ADDR, sizeof(startRtc), (uint8_t *)startRtc, 0, rxBuffer);
    }
}

/* Gets current time from RTC, returns it via the arguments */
void RTC_getTime(uint8_t* singleMinutes, uint8_t* tensOfMinutes, uint8_t* singleHours, uint8_t* tensOfHours)
{
    // Get minutes and hours (try to read it all in one singel read, assuming the MCP940M have address incremention)
    I2C_transfer(RTC_ADDR, 1, (uint8_t*)&readTime[0], 2, &rxBuffer[0]);
    I2C_transfer(RTC_ADDR, 1, (uint8_t*)&readTime[1], 1, &rxBuffer[1]); // Read hours into rxBuffer[3]
    /* Extract the minutes and hours and pass them back via the arguments */
    *tensOfMinutes  = (((rxBuffer[0] & 0x70) >> 4));
    *singleMinutes  = (rxBuffer[0] & 0x0F);
    *tensOfHours    = (((rxBuffer[1] & 0x30) >> 4));
    *singleHours    = (rxBuffer[1] & 0x0F);
}

/* Set the current time of the RTC */
void RTC_setTime(uint8_t singleMinutes, uint8_t tensOfMinutes, uint8_t singleHours, uint8_t tentsOfHours)
{
    /* Stop RTC first and clear the time */
    I2C_transfer(RTC_ADDR, sizeof(stopAndClearTheRtc), (uint8_t *)stopAndClearTheRtc, 0, rxBuffer);

    /* Populate txBuffer to set the new time. */
    txBuffer[0] = 0x01;                                                        /* Minute register */
    txBuffer[1] = (0x70 & (tensOfMinutes << 4)) | (0x0F & singleMinutes);      /* Minutes register value */
    txBuffer[2] = (0x30 & (tentsOfHours << 4)) | (0x0F & singleHours);         /* Hours register value */

    /* Try to write both register in one operation */
    I2C_transfer(RTC_ADDR, 3, txBuffer, 0, rxBuffer);

    /* Restart the RTC again */
    I2C_transfer(RTC_ADDR, sizeof(startRtc), (uint8_t *)startRtc, 0, rxBuffer);
}

/*Enable the RTC alarm to go of every */
void RTC_setMinuteAlarm()
{
    // Disable alarm interrupt
    txBuffer[0] = 0x07; // RTCC CONTROL REGISTER
    txBuffer[1] = 0x80; // Alarm0  enabled
    I2C_transfer(RTC_ADDR, 2,(unsigned char *) txBuffer, 0,(unsigned char *) rxBuffer);

    // Set alarm to go off every minute by ping-pong:ing alarm0 and alarm1:
    // alarm0 goes of every 1st second of a minute.
    // alarm1 goes of every 2nd second of a minute.
    // At alarm0 interrupt, disable alarm0 and reset interrupt flag, enable alarm 1
    // At alarm1 interrupt, disable alarm1 and reset interrupt flag, enable alarm 0

    // Set alarm mask to trigger on minutes
     txBuffer[0] = 0x0A;     // Alarm0 Seconds
     txBuffer[1] = 0x09;     // Alarm0 Seconds Match
     I2C_transfer(RTC_ADDR, 2,(unsigned char *) txBuffer, 0,(unsigned char *) rxBuffer);

     txBuffer[0] = 0x0D; // Clear alarm 0
     txBuffer[1] = 0x00;
     I2C_transfer(RTC_ADDR, 2,(unsigned char *) txBuffer, 0,(unsigned char *) rxBuffer);

     // Enable alarm0 interrupt
     txBuffer[0] = 0x07; // RTCC CONTROL REGISTER
     txBuffer[1] = BIT4; // Alarm0  enabled
     I2C_transfer(RTC_ADDR, 2,(unsigned char *) txBuffer, 0,(unsigned char *) rxBuffer);

//     // Set alarm mask to trigger on minutes
//      txBuffer[0] = 0x011;     // Alarm0 Seconds
//      txBuffer[1] = 0x30;     // Trigger on seconds roll-over
//      txBuffer[2] = 0x00;     // Alarm0 Minutes   (N/A)
//      txBuffer[3] = 0x00;     // Alarm0 Hours     (N/A)
//      txBuffer[4] = 0x00;     // Alarm0 Seconds Match
//      txBuffer[5] = 0x00;     // Alarm0 Date      (N/A)
//      txBuffer[6] = 0x00;     // Alarm0 Month     (N/A)
//      I2C_transfer(RTC_ADDR, 7,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);
//
//     // Enable alarm0 interrupt
//     txBuffer[0] = 0x07; // RTCC CONTROL REGISTER
//     txBuffer[1] = BIT4; // Alarm0  enabled
//     I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);

     // Set alarm0 to go of every 1st sdecond, and alarm1 to go of every 30 second, of a minute
//     txBuffer[0] = 0x0A; // Alarm0
//     txBuffer[1] = 0x01; // 1st second
//     I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);
//     txBuffer[0] = 0x011; // Alarm1
//     txBuffer[1] = 0x30; // 2nd second
//     I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);

    // // Enable alarm0 interrupt
    // txBuffer[0] = 0x07; // RTCC CONTROL REGISTER
    // txBuffer[1] = BIT4; // Alarm0  enabled
    // I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);

    // Set alarm mask to trigger on seconds
//    txBuffer[0] = 0x0D; // Alarm0 Second mask
//    txBuffer[1] = 0x00;
//    I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);
//    txBuffer[0] = 0x014; // Alarm1 Second mask
//    txBuffer[1] = 0x00;
//    I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);
//
//    // Set alarm0 to go of every 1st second, and alarm1 to go of every 2nd second, of a minute
//    txBuffer[0] = 0x0A; // Alarm0
//    txBuffer[1] = BIT0; // 1st second
//    I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);
//    txBuffer[0] = 0x011; // Alarm1
//    txBuffer[1] = 0x30; // 2nd second
//    I2C_transfer(RTC_ADDR, 2,(unsigned char *) &txBuffer, 0,(unsigned char *) &rxBuffer);


}

int alarmToggler = 0;

void RTC_resetAlarm()
{
    txBuffer[0] = 0x0D; // Clear alarm 0
    txBuffer[1] = 0x00;
    I2C_transfer(RTC_ADDR, 2,(unsigned char *) txBuffer, 0,(unsigned char *) rxBuffer);
}
