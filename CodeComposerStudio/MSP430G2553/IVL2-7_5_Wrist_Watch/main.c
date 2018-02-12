#include <msp430g2553.h>
#include <stdint.h>

#include <I2C.h>
#include <RTC_MCP7940M.h>

/* Defines */
#define POWER_ON    BIT3
#define ALARM_INPUT BIT4
#define BUTTON      BIT1

#define RTC_ADDR    0xDE >> 1

#define VFD_DI      BIT0
#define VFD_LOAD    BIT1
#define VFD_CLK     BIT2
#define VFD_BLANK   BIT3
#define VFD_ON      BIT4

/* Enums */
enum LED {
    RED = BIT5,
    GREEN = BIT6,
    BLUE = BIT7
};

enum tasks {
    RESET_ALARM     = BIT0,
    WAKE_UP         = BIT1,
    SET_TIME        = BIT2,
};

unsigned char x = 0;

/* Functions */

void onWakeUp();
void onSetTime();
void onAlarmReset();
void setNewTime();
uint8_t getButtonPress();

// Help function
#define setCase(a) ({ x = a; })
#define protectI2CInterrupt() ({ TA1CCTL0 &= ~CCIE; P1IE &= ~(ALARM_INPUT + BUTTON);})
#define unprotectI2CInterrupt() ({ TA1CCTL0 |= CCIE; P1IE |= ALARM_INPUT + BUTTON; })
unsigned char getCase() { unsigned char tmp = x; x = 0; return tmp; }

uint8_t txBuffer[10];
uint8_t rxBuffer[10];

uint8_t singleMinutes = 0;
uint8_t tensOfMinutes = 0;
uint8_t singleHours   = 0;
uint8_t tensOfHours   = 0;

char buttonIsPressed = 0;
int buttonCounter = 0;
char buttonWasPressed = 0 ;
/**
 * main.c
 */

void test()
{
    txBuffer[0] = 0x0A;
    I2C_transfer(RTC_ADDR, 1, &txBuffer[0], 1, &rxBuffer[0]);
    txBuffer[0] = 0x07;
    I2C_transfer(RTC_ADDR, 1, &txBuffer[0], 1, &rxBuffer[0]);
    txBuffer[0] = 0x0D;
    I2C_transfer(RTC_ADDR, 1, &txBuffer[0], 1, &rxBuffer[0]);
}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    // Set Master and Sub-Master CLK to 16 MHz (don't think we need much more)
	BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    BCSCTL2 = DIVM_0 + DIVS_3;               // MCLK = DCO, SMCLK =  DCO;

    // Set pin function (make P2.6 and P2.7 available as GPIO)
    P2SEL = 0x00;
    P2SEL2 = 0x00;

    // Set pin directions
    P1DIR = POWER_ON;                                                   // POWER_ON is output and ALARM_INPUT is input
    P2DIR = (VFD_DI + VFD_LOAD + VFD_CLK + VFD_BLANK + VFD_ON + RED + GREEN + BLUE);    // All is output

    // Turn off LDOs at start
    P1OUT &= ~POWER_ON;

    // Keep VFD driver in reset and turn power and LEDs off
    P2OUT &= ~(VFD_DI + VFD_LOAD + VFD_CLK + VFD_BLANK + VFD_ON + RED + GREEN + BLUE);

    // Setup ALARM_INPUT and BUTTON for interrupt on falling edge
    P1IES |= ALARM_INPUT + BUTTON;
    // Clear interrupt if it was set when writing IES register
    P1IFG &= ~ALARM_INPUT + BUTTON;
    // Enable interrupt
    P1IE |= ALARM_INPUT + BUTTON;

    // Enable global interrupt
    __bis_SR_register(GIE);

    // Setup the RTC: SMCLK is running at 16 Mhz
    RTC_init(2000000);

    RTC_start();

    RTC_setMinuteAlarm();

    P2OUT ^= RED;
//
    txBuffer[0] = 0x0A;
    I2C_transfer(RTC_ADDR, 1, &txBuffer[0], 1, &rxBuffer[0]);
    txBuffer[0] = 0x07;
    I2C_transfer(RTC_ADDR, 1, &txBuffer[0], 1, &rxBuffer[0]);

    // CPU speed test
    TA1CCTL0 = CCIE;// | XCAP_3; // Enable interrupts when TAR = TACCR0
    TA1CCR0  = 500; // @ 500 Hz
    TA1CTL   = TASSEL_2 + MC_1 + ID_3;

    while(1) {


        switch(getCase()) {
            // Handle alarm
            case (RESET_ALARM)  :   onAlarmReset();  break;
            case (WAKE_UP)      :   onWakeUp(); break;
            case (SET_TIME)     :   onSetTime(); break;//onSetTime(); break;

            default:         break;
        }

        //RTC_getTime(&singleMinutes, &tensOfMinutes, &singleHours, &tensOfHours);
    }
	return 0;
}

void onAlarmReset()
{
    do
    {
        if (!(P1IN & ALARM_INPUT))
        {
            protectI2CInterrupt();
            RTC_resetAlarm();
            unprotectI2CInterrupt();
        }
        else
        {
            break;
        }
    }while(1);
}

void onWakeUp()
{
    // Start by reading out the current time from the RTC
    protectI2CInterrupt();
    RTC_getTime(&singleMinutes, &tensOfMinutes, &singleHours, &tensOfHours);
    unprotectI2CInterrupt();

    // Check button press
    char buttonPress = getButtonPress();

    if (buttonPress == 2) {
        // Set time
        setNewTime();
    }
    else
    {
        // Nothing for now, turn on/off VFD later
    }
}

void setNewTime() {
    P2OUT ^= BLUE;

    char step = 0;
    char buttonPress = 0;

    // First digit
    while(1) {
        buttonPress = getButtonPress();

        if (buttonPress == 1) {
            // Depending on what step we are on ...
            switch (step) {
                case 1: {
                    // Increment single minutes
                    break;
                }
                case 2: {
                    // Increment tens minutes
                    break;
                }
                case 3: {
                    // Increment single hours
                    break;
                }
                case 4: {
                    // Increment tens hours
                    break;
                }
            }
        }

        // Move on to next step
        if ((buttonPress == 2)) {
           step++;
        }

        // We are done
        if (step == 5) {
            break;
        }
    }

    P2OUT ^= BLUE;
}

void onSetTime() {

    uint8_t tmpSingleMinutes = 0;
    uint8_t tmpTensOfMinutes = 0;
    uint8_t tmpSingleHours   = 0;
    uint8_t tmpTensOfHours   = 0;

    while ((tmpSingleMinutes != singleMinutes) &&
           (tmpTensOfMinutes != tensOfMinutes) &&
           (tmpSingleHours != singleHours) &&
           (tmpTensOfHours != tensOfHours))
    {
        // Store the new time to the RTC
        protectI2CInterrupt();
        RTC_setTime(singleMinutes, tensOfMinutes, singleHours, tensOfHours);
        unprotectI2CInterrupt();

        // Verify it got written as it should
        protectI2CInterrupt();
        RTC_getTime(&tmpSingleMinutes, &tmpTensOfMinutes, &tmpSingleHours, &tmpTensOfHours);
        unprotectI2CInterrupt();
    }
}

uint8_t getButtonPress() {

    if (buttonIsPressed && !buttonWasPressed)
    {
        while(buttonIsPressed);
        return buttonWasPressed;
    }
    else
    {
        char tmp = buttonWasPressed;
        buttonWasPressed = 0;
        return tmp;
    }
}


int resetAlarm = 0;
int resetAlarmCounter = 0;
int counter = 0;
#pragma vector=TIMER1_A0_VECTOR
__interrupt  void timer1A3ISR(void)
{
    counter++;
    if (counter > 500)
    {
        P2OUT ^= RED;
        counter = 0;
    }

    if (resetAlarm != 0)
    {
        resetAlarmCounter++;
        if (resetAlarmCounter > 125)
        {
            resetAlarm = 0;
            resetAlarmCounter = 0;
            setCase(RESET_ALARM);
        }
    }

    // In case alarm reset fails, post another one
    if ((!(P1IN & ALARM_INPUT) && !(P1IFG & ALARM_INPUT)))
    {
        setCase(RESET_ALARM);
    }

    // Button
    if (buttonIsPressed)
    {
        if (!(P1IN & BUTTON))
        {
            buttonCounter++;
        }
        else
        {
            buttonIsPressed = 0;
            if (buttonCounter > 1000)
                buttonWasPressed = 2;
            else
                buttonWasPressed = 1;
        }
    }
}

// Was it alarm0 or alarm1? (might need to check this as well in the ISR to not drift)
#pragma vector = PORT1_VECTOR
__interrupt void GPIOPORT1_ISR(void)
{
    //__bic_SR_register(GIE);
    // Was it an alarm interrupt?
    if(P1IFG & ALARM_INPUT)
    {
        // ... Reset ALARM_INPUT IFG flag
        P1IFG &= ~ALARM_INPUT;
        // ... Set next case to handle the alarm
        resetAlarm = 1;
        P2OUT ^= GREEN;
    }
    if(P1IFG & BUTTON)
    {
        P1IFG &= ~BUTTON;
        buttonCounter = 0;
        buttonIsPressed = 1;
        buttonWasPressed = 0;
        // Turn on screen
        //P2OUT |= VFD_ON;
        //TA0CCTL0 |= BIT1;
        //TA1CCTL0 |= BIT1;

//            P2OUT ^= (VFD_ON);
//            P1OUT ^= POWER_ON;

        setCase(WAKE_UP);

    }
}
