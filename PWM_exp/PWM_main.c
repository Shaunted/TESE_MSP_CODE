#include <msp430.h> 

/**
 * main.c
 */


int main(void)
{

    WDTCTL = WDTPW | WDTHOLD;	    // stop watchdog timer


    // Configure PWM 1.6 pin
    P1DIR |= BIT6;                  // P1.6 output
    P1SEL0 &= ~BIT6;                 // P1.6 options select
    P1SEL1 |= BIT6;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked





    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;


//    TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR;        // SMCLK, up mode, clear TBR

//    TB1CCR0 = 10450;                                // PWM Period -> 100 Hz
//    TB0CCTL0 = CLLD_1;
    TB0CCTL1 = OUTMOD_7;                            // CCR1 reset/set


    TB0CCR1 = 32767;
//    TB1CCR1 = 523;                 // CCR1 PWM duty cycle 5%
//    TB1CCR1 = 1045;                // CCR1 PWM duty cycle 10%
//    TB1CCR1 = 1568;                // CCR1 PWM duty cycle 15%
//    TB1CCR1 = 2090;                // CCR1 PWM duty cycle 20%
//    TB1CCR1 = 2613;                // CCR1 PWM duty cycle 25%
//    TB1CCR1 = 3135;                // CCR1 PWM duty cycle 30%
//    TB1CCR1 = 3658;                // CCR1 PWM duty cycle 35%
//    TB1CCR1 = 4180;                // CCR1 PWM duty cycle 40%
//    TB1CCR1 = 4703;                // CCR1 PWM duty cycle 45%
//    TB1CCR1 = 5225;                // CCR1 PWM duty cycle 50%
//    TB1CCR1 = 5748;                // CCR1 PWM duty cycle 55%
//    TB1CCR1 = 6270;                // CCR1 PWM duty cycle 60%
//    TB1CCR1 = 6793;                // CCR1 PWM duty cycle 65%
//    TB1CCR1 = 7315;                // CCR1 PWM duty cycle 70%
//    TB1CCR1 = 7838;                // CCR1 PWM duty cycle 75%
//    TB1CCR1 = 8360;                // CCR1 PWM duty cycle 80%
//    TB1CCR1 = 8883;                // CCR1 PWM duty cycle 85%
//    TB1CCR1 = 9405;                // CCR1 PWM duty cycle 90%
//    TB1CCR1 = 9928;                // CCR1 PWM duty cycle 95%
//    TB1CCR1 = 10450;               // CCR1 PWM duty cycle 100%

    TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR | CNTL__16;

    return 0;
}






