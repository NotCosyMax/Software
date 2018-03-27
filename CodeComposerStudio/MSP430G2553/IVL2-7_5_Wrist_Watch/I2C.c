/*
 * I2C.h
 *  Simple I2C driver for MSP430G2553
 *  Created on: 7 jan. 2018
 *      Author: Max Wennerfeldt
 */
#include <I2C.h>
#include <msp430g2553.h>
#include <stdint.h>
#include <stdbool.h>

/* ==================================== Local global variables ==================================== */
static uint8_t* txBufferPosition = 0;
static uint8_t* txBufferEnd = 0;
static uint8_t* rxBufferPosition = 0;
static uint8_t* rxBufferEnd = 0;
static uint8_t waitUntilFinish = 1;
static uint8_t  transferDone = 0;

/* ==================================== Function definition  ==================================== */

/* Returns busy state of the I2C peripheral */
bool I2C_busy()
{
    return (UCB0STAT & UCBBUSY);
}

/* Initialize the I2C peripheral, keep the peripheral in reset until a transfer is started. */
void I2C_init(uint32_t clkFrequency, I2C_operationSpeed speed, I2C_mode mode)
{
    /* Assign I2C pins to USCI_B0 */
    P1SEL |= I2C_SCL_PIN + I2C_SDA_PIN;
    P1SEL2|= I2C_SCL_PIN + I2C_SDA_PIN;

    /* Enable SW reset */
    UCB0CTL1 |= UCSWRST;
    /* I2C Master, synchronous mode */
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
    /* Use SMCLK, keep SW reset */
    UCB0CTL1 = UCSSEL_2 + UCSWRST;

    /* Set bit rate depending on specified operation speed */
    if (speed == I2C_STANDARD_SPEED) {
        UCB0BR0 = clkFrequency / 100000;      // Set clock speed: fSCL = SMCLK/UCB0BR0 = ~100kHz ...
    }
    else {
        // Wtf is up with the I2C?
        UCB0BR0 = clkFrequency / 400000;      // Set clock speed: fSCL = SMCLK/UCB0BR0 = ~400kHz ...
    }

    /* UCB0BR1 can remain at 0 as CPU frequency wont go above 16 Mhz */
    UCB0BR1 = 0;

    /* Blocking or Non-blocking mode */
    if (mode == I2C_BLOCKING_MODE) {
        waitUntilFinish = 1;
    }
    else {
        waitUntilFinish = 0;
    }
}


/* Perform an I2C transaction to the specified slave adress (shifted by user). */
void I2C_transfer(uint8_t slaveAddress, uint8_t txCount, uint8_t* txData, uint8_t rxCount, uint8_t* rxData)
{
    /* Store RX and TX buffer positions */
    txBufferPosition = txData;
    txBufferEnd      = txData + txCount;
    rxBufferPosition = rxData;
    rxBufferEnd      = rxData + rxCount;

    transferDone = 0;

    /* Enable SW reset if not done already */
    UCB0CTL1 |= UCSWRST;
    /* Clear TX buffer */
    UCB0TXBUF = 0;
    /* Set Slave Address */
    UCB0I2CSA = slaveAddress;
    /* Clear SW reset, resume operation */
    UCB0CTL1 &= ~UCSWRST;
    /* Enable NACK interrupt */
    UCB0I2CIE = UCNACKIE;
    /* Enable TX interrupt */
    IE2 |= UCB0TXIE;
    /* Enable TX mode, send start condition*/
    UCB0CTL1 |= UCTR + UCTXSTT;

    /* Enter LPM0 (CPUOFF) if suppose to wait until the transfer finishes */
    if (waitUntilFinish && !transferDone)
        __bis_SR_register(CPUOFF + GIE);    // Enter LPM1 w/ interrupts for TX

    /* Make sure the I2C is done before returning */
    while(I2C_busy());
}


/* ==================================== Interrupt routines ==================================== */

/* I2C RX and TX ISR. This interrupt will both handle TX and RX during a I2C transmit */
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    /* If in TX mode and TX interrupt is set ... */
    if((UCB0CTL1 & UCTR) && (IFG2 & UCB0TXIFG)) {
        /* ... and there if more data to send ... */
        if (txBufferPosition < txBufferEnd)
        {
            /* ... load next byte into the I2C TX buffer (and post-increment address pointer).
               Writing the TX buffer will clear the interrupt flag! */
            UCB0TXBUF = *txBufferPosition++;
        }
        /* ... or if there is not anymore data to send ... */
        else
        {
            /* ... disable TX interrupt and ... */
            IE2 &= ~UCB0TXIE;
            /* ... clear TX interrupt flag */
            IFG2 &= ~UCB0TXIFG;
            /* If also expecting to receive data ... */
            if (rxBufferPosition < rxBufferEnd)
            {
                /* ... configure I2C into RX mode, ... */
                UCB0CTL1 &= ~UCTR;
                /* ... set send restart condition and ... */
                UCB0CTL1 |= UCTXSTT;
                /* ... enable I2C RX interrupt */
                IE2 |= UCB0RXIE;
                /* If only expecting to receive one byte ... */
                if (rxBufferPosition + 1 == rxBufferEnd)
                {
                    /* ... wait for START condition to be set, then set "send STOP" condition */
                    while ((UCB0CTL1 & UCTXSTT))
                    {
                        if (UCB0STAT & UCNACKIFG)
                            break;
                    }
                    UCB0CTL1 |= UCTXSTP;
                }
            }
            // ... else, if not receiving anything ...*/
            else
            {
                /* ... send STOP condition and ... */
                UCB0CTL1 |= UCTXSTP;
                /* ... exit Low Power Mode 0 aka "Let the CPU run" */
                LPM0_EXIT;
                /* Signal transfer done */
                transferDone = 1;
            }
        }
    }
    /* ... else if in RX mode (UCTR = 0) and I2X RX interrupt is set ... */
    else if (IFG2 & UCB0RXIFG) {
        /* ... read byte into RX buffer (also clears interrupt flag!) and ... */
        *rxBufferPosition++ = UCB0RXBUF;
        /* ... if 2nd last byte, send STOP */
        if (rxBufferPosition + 1 == rxBufferEnd)
            UCB0CTL1 |= UCTXSTP;
        /* ... if last byte, shutdown I2C */
        if (rxBufferPosition == rxBufferEnd)
        {
            /* Disable I2C RX interrupt and clear interrupt flag */
            IE2 &= ~UCB0RXIE;
            IFG2 &= ~UCB0RXIFG;
            /* Exit Low Power Mode 0 aka "Let the CPU run" */
            LPM0_EXIT;
            /* Signal transfer done */
            transferDone = 1;
        }
    }
}

/* I2C NACK ISR. If an NACK is received, tear-down I2C connection */
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    /* If NACK ... */
    if (UCB0STAT & UCNACKIFG) {
        /* ... generate stop condition ... */
        UCB0CTL1 |= UCTXSTP;
        /* ... clear NACK interrupt flag and ... */
        UCB0STAT &= ~UCNACKIFG;
        /* ... exit Low Power Mode 0 aka "Let the CPU run" */
        LPM0_EXIT;
        /* Signal transfer done */
        transferDone = 1;
    }
}

