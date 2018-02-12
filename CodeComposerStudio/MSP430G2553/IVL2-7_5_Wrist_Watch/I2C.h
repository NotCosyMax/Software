/*
 * I2C.h
 *  Simple I2C driver for MSP430G2553
 *  Created on: 7 jan. 2018
 *      Author: Max Wennerfeldt
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>

/* Defines */
#define I2C_SCL_PIN     BIT6
#define I2C_SDA_PIN     BIT7

/* I2C operation speed: Standard or Fast. */
typedef enum I2C_operationSpeed_
{
    I2C_STANDARD_SPEED = 0,  /* 100 kHz */
    I2C_FAST_SPEED,          /* 400 kHz */
} I2C_operationSpeed;

/* I2C tranfer mode: blocking or non-blocking. */
typedef enum I2C_mode_
{
    I2C_BLOCKING_MODE = 0,
    I2C_NON_BLOCKING_MODE,
} I2C_mode;

/* ==================================== Function declaration ==================================== */

/* Returns the state of the I2C transfer */
bool I2C_busy();

/*
    Initialize the I2C peripheral, keep the peripheral in reset until a transfer is started.
    Arguments:
        clkFrequency:       SMCLK frequency.
        speed:              The I2C operation speed.
        mode:               The transfer operation mode.

*/
void I2C_init(uint32_t clkFrequency, I2C_operationSpeed speed, I2C_mode mode);

/*
    Perform an I2C transaction to the specified slave adress (shifted by user).
    Arguments:
        txCount:    Size of txData.
        txData:     Data to be sent.
        rxCount:    Size of rxData.
        rxData:     Data to be received.
*/
void I2C_transfer(uint8_t slaveAddress, uint8_t txCount, uint8_t* txData, uint8_t rxCount, uint8_t* rxData);

#endif /* I2C_H_ */
