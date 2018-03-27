/*
 * RTC_MCP7940M.h
 *
 *  A small code implementation to set/get the current time
 *  and trigger a interrupt each minute from the MCP7940M RTC.
 *
 *  Created on: 11 jan. 2018
 *      Author: Maxen
 */

#ifndef RTC_MCP7940M_H_
#define RTC_MCP7940M_H_

/* Defines */
/* The MCP7940M slave address, shifted to the left for the I2C drivers */
#define RTC_ADDR    0xDE >> 1

/* ==================================== Function declaration ==================================== */

/* Initialize the RTC: setup the I2C given the SMCLK frequency */
void RTC_init(uint32_t clkFrequency);

/* Checks of the RTC oscillator needs to be started, or of the RTC is running */
void RTC_start();

/* Gets current time from RTC, returns it via the arguments */
void RTC_getTime(uint8_t* singleMinutes, uint8_t* tensOfMinutes, uint8_t* singleHours, uint8_t* tentsOfHours);

/* Set the current time of the RTC */
void RTC_setTime(uint8_t singleMinutes, uint8_t tensOfMinutes, uint8_t singleHours, uint8_t tentsOfHours);

/* Enable the RTC alarm to go of every */
void RTC_enableMinuteAlarm();

/* Disable the RTC alarm to go of every */
void RTC_disableMinuteAlarm();

/* Resets the alarm */
void RTC_resetAlarm();

#endif /* RTC_MCP7940M_H_ */
