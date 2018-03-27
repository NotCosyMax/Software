#include <msp430g2553.h>
#include <stdint.h>

#include <I2C.h>
#include <RTC_MCP7940M.h>

/* Defines */
#define POWER_ON    BIT3
#define ALARM_INPUT BIT4
#define BUTTON      BIT1
#define BATT_SENSE  BIT0

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
};

const char mappingVFD[10][7] = {
        {1, 0, 1, 1, 1, 1, 1}, // 0
        {0, 0, 0, 1, 0, 1, 0}, // 1
        {0, 1, 1, 0, 1, 1, 1}, // 2
        {0, 1, 0, 1, 1, 1, 1}, // 3
        {1, 1, 0, 1, 0, 1, 0}, // 4
        {1, 1, 0, 1, 1, 0, 1}, // 5
        {1, 1, 1, 1, 1, 0, 1}, // 6
        {0, 0, 0, 1, 0, 1, 1}, // 7
        {1, 1, 1, 1, 1, 1, 1}, // 8
        {1, 1, 0, 1, 0, 1, 1}  // 9
        };

unsigned char x[5] = {0};
uint8_t caseCounter = 0;

/* Functions */
uint8_t batteryCheck();
void onWakeUp();
void onSetTime();
void onAlarmReset();
void setNewTime();
uint8_t getButtonPress();

// Help function

#define protectI2CInterrupt() ({ TA1CCTL0 &= ~CCIE; P1IE &= ~(ALARM_INPUT + BUTTON);})
#define unprotectI2CInterrupt() ({ TA1CCTL0 |= CCIE; P1IE |= ALARM_INPUT + BUTTON; })
void setCase(uint8_t a) {
    x[0] = a;
}
unsigned char getCase() {
    protectI2CInterrupt();
    unsigned char tmp = 0;
        tmp = x[0];
        x[0] = 0;
    unprotectI2CInterrupt();
    return tmp;
}

uint8_t txBuffer[10];
uint8_t rxBuffer[10];

// Current time
uint8_t currentTime[5] = {0, 0, 0, 0, 0};   // currentTime[0..4] : singleMinutes, tensOfMinutes, Dots, singleHours, tensOfHours

char buttonIsPressed = 0;
int buttonCounter = 0;
char buttonWasPressed = 0 ;

int resetAlarm = 0;

uint8_t inCase = 0;

const uint8_t pwmLED[3] = {3, 0, 3}; // RGB
uint8_t pwmLEDCounter[3] = {3, 0, 3}; // RGB

const double batteryLimit = 3.3 / 15.0;

unsigned int blinkDigit = 2; // Blink dots by default
/**
 * main.c
 */

int main(void)
 {
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    // Set Master and Sub-Master CLK to 16 MHz (don't think we need much more)
	BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    BCSCTL2 = DIVM_0 + DIVS_3;               // MCLK = DCO, SMCLK =  DCO/8;

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
    //P2OUT |= VFD_BLANK;

    // Enable button pull up
    P1REN |= BUTTON;
    P1OUT |= BUTTON;

    // Setup ALARM_INPUT and BUTTON for interrupt on falling edge
    P1IES |= ALARM_INPUT + BUTTON;
    // Clear interrupt if it was set when writing IES register
    P1IFG &= ~ALARM_INPUT + BUTTON;
    // Enable interrupt
    P1IE |= ALARM_INPUT + BUTTON;

    // Pins not used set as low outputs
    P1DIR |= BIT2 + BIT5;
    P1OUT &= ~(BIT2 + BIT5);

    // Enable global interrupt
    __bis_SR_register(GIE);

    // Setup ADC
    ADC10CTL1 = INCH_0;                      // A0
    ADC10CTL0 = SREF_1 + REFON + ADC10SHT_2 + ADC10ON + ADC10SR + ADC10IE;
    ADC10AE0 |= BATT_SENSE; // Mux P1.0 to A0


    // If battery is to bad, don't let the clock start up until it is charged again
//    while (1) {
//        if (!batteryCheck()) {
//            // Battery was not OK, go into deep sleep
//            __bis_SR_register(LPM4 + GIE);
//        }
//        else {
//            // We are initially OK on with battery, proceed!
//            break;
//        }
//    }

    // Setup the RTC: SMCLK is running at 16 Mhz
    RTC_init(2000000);

    RTC_start();

    // Enable the timer
    TA1CCR0  = 500; // @ 500 Hz
    TA1CTL   = TASSEL_2 + MC_1 + ID_3; // SMCLK / 8 = 2 MHz / 8 = 250 kHz

    while(1) {


        switch(getCase()) {
            // Handle alarm
            case (RESET_ALARM)  :   onAlarmReset();  break;
            case (WAKE_UP)      :   onWakeUp(); break;
            // If no new case, enter LPM
            default             :
                                    // Enter low power mode
                                    if ((P1OUT & POWER_ON)) { // Bypass for now
                                        __bis_SR_register(CPUOFF + GIE);
                                    }
                                    else {
                                        // Turn of RTC interrupts, we enable these again when we wake up
                                        protectI2CInterrupt();
                                        RTC_disableMinuteAlarm();
                                        unprotectI2CInterrupt();

                                        P2OUT &= ~(RED + GREEN + BLUE);

                                        // Turn of ADC to save power
                                        ADC10CTL0 &= ~(REFON + ADC10IE + ADC10ON);

                                        __bis_SR_register(LPM4 + GIE);

                                        // Turn ADC back on
                                        ADC10CTL0 |= REFON + ADC10IE + ADC10ON;
                                    }
                                    break;
        }
    }

	return 0;
}

uint8_t batteryCheck() {
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit

    // If below 3.6V
    if ((double)(((double) ADC10MEM) * (1.5 / 1024.0) ) < (batteryLimit)) {
        // Blink RED led a few times then go into LPM4
        uint32_t i = 0;
        uint8_t j = 0;
        for (; j < 4; j++) {
            P2OUT ^= RED;
            // Delay
            for (i = 0; i < 500000; i++) {};
        }
        return 0;
    }

    return 1;
}

void onAlarmReset()
{
    // Increment time if first time we try to clear the alarm (we don't want to count retries)
    if (resetAlarm == 1) {

        currentTime[0]++;
        if (currentTime[0] > 9) {
            currentTime[0] = 0;
            currentTime[1]++;

            if (currentTime[1] > 5) {
                currentTime[1] = 0;
                currentTime[3]++;

                if ((currentTime[4] < 2) && (currentTime[3] > 9)) {
                    currentTime[3] = 0;
                    currentTime[4]++;
                }
                else if ((currentTime[4] > 1) && (currentTime[3] > 3)) {
                    currentTime[3] = 0;
                    currentTime[4] = 0;
                }
            }
        }

        // Now, we have registered the alarm
        resetAlarm = 0;
    }

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
    inCase = 1;

    // Perform a battery check
    if (!batteryCheck()) {
        // Battery was NOT OK, make sure to turn of screen and go back to deep sleep
        P1OUT &= ~(POWER_ON);
        P2OUT &= ~VFD_ON;

        // Do not do anything else
        inCase = 0;
        return;
    }

    protectI2CInterrupt();
    // If returning from sleep enable alarm again
    if (!(P1OUT & POWER_ON)) {
        RTC_enableMinuteAlarm();
    }

    // Reading out the current time from the RTC
    RTC_getTime(&currentTime[0], &currentTime[1], &currentTime[3], &currentTime[4]);
    unprotectI2CInterrupt();

    // Check button press
    char buttonPress = getButtonPress();

    if (buttonPress == 2) {


        P1OUT |= POWER_ON;

        unsigned int i = 0;
        for(;i < 65000; i++) {};

        // Turn on VFD and regulators
        P2OUT |= VFD_ON;

        //P2OUT &= ~VFD_BLANK;

        // Set time
        setNewTime();
    }
    else
    {
        // Nothing for now, turn on/off VFD later

        // Toggle screen on or off
        P1OUT ^= POWER_ON;

        unsigned int i = 0;
        for(;i < 65000; i++) {};

        P2OUT ^= VFD_ON;


        inCase = 0;
    }
}

void setNewTime() {
    char step = 1;
    char buttonPress = 0;

    // Start by blinking first digit
    blinkDigit = 0;

    while(1) {
        buttonPress = getButtonPress();

        if (buttonPress == 1) {
            // Depending on what step we are on ...
            switch (step) {
                case 1: {
                    // Increment single minutes
                    currentTime[0]++;
                    if (currentTime[0] > 9) {
                        currentTime[0] = 0;
                    }
                    break;
                }
                case 2: {
                    // Increment tens minutes
                    currentTime[1]++;
                    if (currentTime[1] > 5) {
                        currentTime[1] = 0;
                    }
                    break;
                }
                case 3: {
                    // Increment single hours
                    currentTime[3]++;
                    if (currentTime[3] > 9) {
                        currentTime[3] = 0;
                    }
                    break;
                }
                case 4: {
                    // Increment tens hours
                    currentTime[4]++;
                    if ((currentTime[4] > 2) || (currentTime[3] > 3)) {
                        currentTime[4] = 0;
                    }
                    break;
                }
            }
        }

        // Move on to next step
        if ((buttonPress == 2)) {
           step++;

           // What digit to blink?
           blinkDigit = step - 1;
           // If any of the first two digits we need to subtract one (due to the the dots offset in time
           if (step > 2) {
               blinkDigit += 1;
           }
        }

        // We are done
        if (step == 5) {

            // Strap the time at 24h
            if ((currentTime[4] > 2) || (currentTime[3] > 3)) {
                currentTime[4] = 0;
            }

            break;
        }
    }

    onSetTime();

    blinkDigit = 2;

    inCase = 0;
}

void onSetTime() {
    uint8_t tmpSingleMinutes = 0;
    uint8_t tmpTensOfMinutes = 0;
    uint8_t tmpSingleHours   = 0;
    uint8_t tmpTensOfHours   = 0;

    // Toggle screen off
    P1OUT ^= POWER_ON;

    unsigned int i = 0;
    for(;i < 65000; i++) {};

    P2OUT ^= VFD_ON;


    while ((tmpSingleMinutes != currentTime[0]) ||
           (tmpTensOfMinutes != currentTime[1]) ||
           (tmpSingleHours != currentTime[3]) ||
           (tmpTensOfHours != currentTime[4]))
    {
        // Store the new time to the RTC
        protectI2CInterrupt();
        RTC_setTime(currentTime[0], currentTime[1], currentTime[3], currentTime[4]);
        // Verify it got written as it should
        RTC_getTime(&tmpSingleMinutes, &tmpTensOfMinutes, &tmpSingleHours, &tmpTensOfHours);
        unprotectI2CInterrupt();
    }

    // Toggle screen on again
    P1OUT ^= POWER_ON;

    for(i = 0; i < 65000; i++) {};

    P2OUT ^= VFD_ON;

}

uint8_t getButtonPress() {

    if (buttonIsPressed && !buttonWasPressed)
    {
        while(buttonIsPressed);
        char tmp = buttonWasPressed;
        buttonWasPressed = 0;
        return tmp;
    }
    else
    {
        char tmp = buttonWasPressed;
        buttonWasPressed = 0;
        return tmp;
    }
}


int resetAlarmCounter = 0;
int counter = 0;
static unsigned char currentGrid = 0;
unsigned char blinkDots = 1;
unsigned int  dotCounter = 0;

#pragma vector=TIMER1_A0_VECTOR
__interrupt  void timer1A0ISR(void)
{
    // Test case for blinking led
    if (P1OUT & POWER_ON) {
        pwmLEDCounter[0]--;
        pwmLEDCounter[2]--;

        if (!pwmLEDCounter[0])
        {
            P2OUT ^= RED;
            pwmLEDCounter[0] = pwmLED[0];
        }
        if (!pwmLEDCounter[2])
        {
            P2OUT ^= BLUE;
            pwmLEDCounter[2] = pwmLED[2];
        }
    }

    // Due to the shady hardware patch, we need to make sure not to clock anything out
    // if screen is not enabled
    unsigned int i;
    // Update VFD
    unsigned char digit;
    if(currentGrid > 4)
        digit = *(((unsigned char *)&currentTime) + 4);
    else
        digit = *(((unsigned char *)&currentTime) + currentGrid);


    // First 5 bits sets the active grid
    for(i = 0; i < 5; i++) {
        // Default data value set to zero
        P2OUT &= ~VFD_DI;

        if(currentGrid < 5)
        {
            if(i == currentGrid)
                P2OUT |= VFD_DI;

        }
        else
        {
            if(i == 4)
                P2OUT |= VFD_DI;
        }

        // Blink the dots @ ~1s
        if(((currentGrid == blinkDigit) || ((currentGrid > 4) && (i == 4) && (blinkDigit == 4))) && (blinkDots))
        {
            dotCounter++;
            if((dotCounter == 500) || (inCase && (dotCounter == 200)))
            {
                dotCounter = 0;
            }
            else if(dotCounter > 75)
            {
                P2OUT &= ~VFD_DI;
            }
        }

        // Clock data
        P2OUT |= VFD_CLK;
        // Waste time?
        P2OUT &= ~VFD_CLK;
    }

    // Last 7 bits forms the digit
    for(; i < 12; i++) {
        // Default data value set to zero
        P2OUT &= ~VFD_DI;

        if(mappingVFD[digit][i-5] == 1)
            P2OUT |= VFD_DI;

        //Clock data
        P2OUT |= VFD_CLK;
        // Waste time?
        P2OUT &= ~VFD_CLK;
    }

    // Latch the new data to the output
    P2OUT |= VFD_LOAD;
    P2OUT &= ~VFD_LOAD;

    // Increment counters
    if(currentGrid == 5)
        currentGrid = 0;
    else
        currentGrid++;

    // Due to the major mistake and shady hardware patch to correct this,
    // We need to ensure we do not drive the VFD IC via GPIOs when cutting the power.
    // Soo when leaving the ISR, all the signals should be low!
    P2OUT &= ~VFD_DI;

    // Delay the alarm reset so it do not re-trigger right away
    if (resetAlarm > 1)
    {
        resetAlarmCounter++;
        if (resetAlarmCounter > 125)
        {
            resetAlarm--; // Decrement by one, reaches 0 when we have changed the actual time
            resetAlarmCounter = 0;
            setCase(RESET_ALARM);
            LPM0_EXIT;
        }
    }

    // In case alarm reset fails, post another one
    if ((!(P1IN & ALARM_INPUT) && !(P1IFG & ALARM_INPUT)))
    {
        setCase(RESET_ALARM);
        LPM0_EXIT;
    }

    // Button press counter to distinguish between "short" and "long" press
    if (buttonIsPressed)
    {
        if (!(P1IN & BUTTON))
        {
            buttonCounter++;
        }
        else
        {
            buttonIsPressed = 0;
            if (buttonCounter > 800) {
                buttonWasPressed = 2;
                if (!inCase) {
                    setCase(WAKE_UP);
                    LPM0_EXIT;
                }
            }
            else if (buttonCounter > 150) {
                buttonWasPressed = 1;
                if (!inCase) {
                    setCase(WAKE_UP);
                    LPM0_EXIT;
                }
            }
            else
                buttonWasPressed = 0;
        }
    }
}

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
        resetAlarm = 2;
        P2OUT ^= GREEN;
        LPM0_EXIT;
    }
    if(P1IFG & BUTTON)
    {
        P1IFG &= ~BUTTON;
        buttonCounter = 0;
        buttonIsPressed = 1;
        buttonWasPressed = 0;
        // Turn on screen

        if (!inCase)
        {
            LPM4_EXIT;
        }

    }
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
