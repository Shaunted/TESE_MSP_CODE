/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430FR231x Demo - ADC, Sample A2/A1/A0, internal 1.5V Ref.
//
//  Description: This example works on Repeat-sequence-of-channels Mode
//  with TB1.1 as the trigger signal.
//  A2/A1/A0 is sampled 16ADCclks with reference to 1.5V.
//  Internal oscillator times sample (16x) and conversion(13x).
//  Inside ADC_ISR A2/A1/A0 sample value put into array ADC_Result[3].
//  ACLK = default REFO ~32768Hz, MCLK = SMCLK = default DCODIV ~1MHz.
//
//  Note: The TB1.1 is configured for 200us 50% PWM, which will trigger ADC
//  sample-and-conversion every 200us. The period of TB1.1 trigger event
//  should be more than the time period taken for ADC sample-and-conversion
//  and ADC interrupt service routine of each channel, which is about 57us in this code
//
//                MSP430FR2311
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//        >---|P1.2/A2          |
//        >---|P1.1/A1          |
//        >---|P1.0/A0          |
//
//
//   Eason Zhou
//   Texas Instruments Inc.
//   January 2020
//   Built with IAR Embedded Workbench v7.12.1 & Code Composer Studio v9.2.0
//******************************************************************************
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Function Defines~

void PID_function(uint16_t sensor, int64_t *u, int64_t *error_i, bool set);

// PID Defines

#define UMAX    65535 // Around 3.3V
#define UMIN    19858 // Around 1V
#define DT      65
#define KP      200
#define KI      100

#define ideal 200

#define ideal_heat 700

uint16_t ADC_Result[3];                    // 12-bit ADC conversion result array
uint8_t i;

uint32_t val = 0;

int32_t error;      // Q0
int64_t u_pump = 0;
int64_t error_i_pump = 0;
int64_t u_heater = 0;
int64_t error_i_heater = 0;

bool set;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                   // Stop WDT

    // Configure ADC A0~2 pins
    P1SEL0 |= BIT0 + BIT1 + BIT2;
    P1SEL1 |= BIT0 + BIT1 + BIT2;

    // Configure PWM 1.6 pin
    P1DIR |= BIT6;                  // P1.6 output
    P1SEL0 &= ~BIT6;                 // P1.6 options select
    P1SEL1 |= BIT6;

    // Configure PWM 1.6 pin
    P1DIR |= BIT7;                  // P1.6 output
    P1SEL0 &= ~BIT7;                 // P1.6 options select
    P1SEL1 |= BIT7;

    P2DIR |= BIT0 | BIT1;
    P2OUT &= ~(BIT0 | BIT1);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // CLOCK CONFIG

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;
    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL0 = 0;                                   // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);             // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1))
        ;         // FLL locked

    // Configure ADC
    ADCCTL0 |= ADCSHT_2 | ADCON;                            // 16ADCclks, ADC ON
    ADCCTL1 |= ADCSHP | ADCSHS_2 | ADCCONSEQ_3; // ADC clock MODCLK, sampling timer, TB1.1B trig.,repeat sequence
    ADCCTL2 |= ADCRES;                               // 8-bit conversion results
    ADCMCTL0 |= ADCINCH_2 | ADCSREF_1;                   // A0~2(EoS); Vref=1.5V
    ADCIE |= ADCIE0;                       // Enable ADC conv complete interrupt

    // Configure reference
    PMMCTL0_H = PMMPW_H;                             // Unlock the PMM registers
    PMMCTL2 |= INTREFEN;                            // Enable internal reference
    __delay_cycles(400);                         // Delay for reference settling

    // Configure TB1.1B as ADC trigger signal
    // Note: The TB1.1B is configured for 200us 50% PWM, which will trigger ADC
    // sample-and-conversion every 200us. The period of TB1.1B trigger event
    // should be more than the time period taken for ADC sample-and-conversion
    // and ADC interrupt service routine of each channel, which is about 57us in this code
    TB1CCR0 = 10000;                                      // PWM Period, 200us
    TB1CCTL1 = OUTMOD_7;                                       // CCR1 reset/set
    TB1CCR1 = 5000;                                  // CCR1 PWM duty cycle, 50%
    TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR | CNTL__16 | ID_3; // SMCLK, up mode, clear TAR

    // PWM Config (TB0)
    TB0CCTL1 = OUTMOD_7;                            // CCR1 reset/set
    TB0CCTL2 = OUTMOD_7;                            // CCR1 reset/set
    TB0CCR1 = UMAX * 0;
    TB0CCR2 = 32767;
    TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR | CNTL__16;

    i = 2;
    ADCCTL0 |= ADCENC;                                       // Enable ADC
    TB1CTL |= TBCLR;

    while (1)
    {          // Clear TAR to start the ADC sample
        __bis_SR_register(LPM0_bits | GIE);          // Enter LPM0 w/ interrupts
        if (i == 2)
        {
            val = (ADC_Result[1] + ADC_Result[2]) >> 1;
            PID_function(ADC_Result[0], &u_pump, &error_i_pump, 0);
            if (u_pump >= 0)
            {
                if (u_pump > UMAX)
                {
                    TB0CCR1 = UMAX;
                }

                else
                {
                    TB0CCR1 = u_pump;
                }

            }
            else
            {
                TB0CCR1 = 0;
            }
            PID_function(val, &u_heater, &error_i_heater, 1);
            if (u_heater >= 0)
            {
                if (u_heater > UMAX)
                {
                    TB0CCR2 = UMAX;
                }

                else
                {
                    TB0CCR2 = u_heater;
                }

            }
            else
            {
                TB0CCR2 = 0;
            }
        }

    }
}

void PID_function(uint16_t sensor, int64_t *u, int64_t *error_i, bool set)
{
    P2OUT |= BIT1;
// Control Logic
    if (set == 1)
    {

        error = ideal_heat;
        error = sensor - error;
    }
    else
    {
        error = ideal;
        error = error - sensor;
    }
    *u = KP * error;                // Q0 = Q0*Q0
    *u = (*u << 16) + KI * (*error_i);            // Q16 = Q16 + Q16
    *u = *u >> 16;

// Anti-windup
    if ((*u >= UMAX || *u < 0)
            && (((error >= 0) && (*error_i >= 0))
                    || ((error < 0) && (*error_i < 0))))
    {
        *error_i = *error_i;
    }
    else
    {
        *error_i = *error_i + DT * error;  // Q16
    }
    P2OUT &= ~BIT1;
}

// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(ADCIV, ADCIV_ADCIFG))
    {
    case ADCIV_NONE:
        break;
    case ADCIV_ADCOVIFG:
        break;
    case ADCIV_ADCTOVIFG:
        break;
    case ADCIV_ADCHIIFG:
        break;
    case ADCIV_ADCLOIFG:
        break;
    case ADCIV_ADCINIFG:
        break;
    case ADCIV_ADCIFG:
        ADC_Result[i] = ADCMEM0;
        if (i == 0)
        {
            P2OUT ^= BIT0;
            i = 2;
            __bic_SR_register_on_exit(LPM0_bits); // Clear CPUOFF bit from LPM0
        }
        else
        {
            i--;
        }
        break;
    default:
        break;
    }
}
