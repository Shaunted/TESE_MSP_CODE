/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2014, Texas Instruments Incorporated
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
//   MSP430FR231x Demo - eUSCI_B0, SPI 4-Wire Slave Data Echo
//                       eUSCI_B0 port remapped to P2.2-P2.5.
//   Description: SPI slave talks to SPI master using 4-wire mode. Data received
//   from master is echoed back.
//   eUSCI_B0 remap functionality is controlled by the USCIBRMP bit of SYSCFG2
//   register. Set USCIBRMP bit to remap eUSCI_B0 port to P2.2-P2.5.
//   ACLK = default REFO ~32768Hz, MCLK = SMCLK = default DCODIV ~1MHz.
//   Note: Ensure slave is powered up before master to prevent delays due to
//   slave init.
//
//
//                   MSP430FR2311
//                -----------------
//            /|\|                 |
//             | |                 |
//             --|RST              |
//               |                 |
//               |             P2.2|-> Slave Select (UCB0STE)
//               |             P2.3|-> Serial Clock Out (UCB0CLK)
//               |             P2.5|-> Data Out (UCB0SOMI)
//               |             P2.4|<- Data In (UCB0SIMO)
//
//
//   Darren Lu
//   Texas Instruments Inc.
//   Oct. 2016
//   Built with IAR Embedded Workbench v6.30 & Code Composer Studio v6.1
//******************************************************************************
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


// Function Defines~

void PID_function(uint16_t sensor);

// PID Defines

#define UMAX    65535
#define DT      655
#define KP      5
#define KI      5
//#define IDEAL   512


// SPI Variables

uint8_t ReceiveBuffer[8] = {0};
uint8_t ReceiveIndex = 0;
uint16_t ReceivedValue = 0;
uint8_t TransmitBuffer[8] = {0};
uint8_t TransmitIndex = 0;


// ADC/PID Variables
uint16_t ADC_Result;
uint16_t ideal;

int32_t error = 0;      // Q0
int64_t u = 0;
int64_t error_i = 0;



int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer


// PWM CONFIG
  P1DIR |= BIT6;                  // P1.6 output pwm
  P1SEL0 |= BIT6;                 // P1.6 options select
  P1SEL1 &= ~BIT6;

// Configure ADC A1 pin
  P1SEL0 |= BIT5;
  P1SEL1 |= BIT5;

  P1DIR |= BIT0;
  P1OUT &= ~BIT0;

  P2DIR |= BIT0;
  P2OUT &= ~BIT0;

// CLOCK CONFIG

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



// SPI CONFIG

  P2SEL0 |= BIT2 | BIT3 | BIT4 | BIT5;      // set 4-SPI pin as second function
  SYSCFG2 |= USCIBRMP;                      // eUSCI_B0 port remapped to P2.2-P2.5

  UCB0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
                                            // 4-pin, 8-bit SPI slave
  UCB0CTLW0 = UCSYNC|UCMSB|UCMODE_2|UCSTEM|UCCKPH;
                                                  // MSB
  UCB0CTLW0 &= ~UCSSEL;
  UCB0CTLW0 |= UCSSEL__SMCLK;               // Select SMCLK
  UCB0BR0 = 0x02;                           // BRCLK = SMCLK/2
  UCB0BR1 = 0;                              //
  UCB0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
  UCB0IE |= UCRXIE;                         // Enable USCI_B0 RX interrupt


  PM5CTL0 &= ~LOCKLPM5;                     // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

  // Configure ADC10
   ADCCTL0 |= ADCMSC | ADCON;                              // ADCON, S&H=16 ADC clks
   ADCCTL1 |= ADCSHP | ADCSHS_2 | ADCSSEL_2 | ADCCONSEQ_2;             // ADCCLK = MODOSC; sampling timer
   ADCCTL2 |= ADCRES;                                        // 10-bit conversion results
   ADCMCTL0 |= ADCINCH_5 | ADCSREF_1;                        // A5 ADC input select; Vref=1.5V
   ADCIE |= ADCIE0;                                          // Enable ADC conv complete interrupt
                                        // Enable ADC conv complete interrupt

  // Configure reference
  PMMCTL0_H = PMMPW_H;                                      // Unlock the PMM registers
  PMMCTL2 |= INTREFEN;                                      // Enable internal reference
  __delay_cycles(400);                                      // Delay for reference settling


  // ADC Timer CONFIG (TB1)
     //    TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR;        // SMCLK, up mode, clear TBR

   //  TB1CCTL0 = CLLD_1;
   TB1CCTL1 = OUTMOD_3;                            // CCR1 reset/set
   TB1CCR0 = 32788;                                // PWM Period -> around 61 Hz
   TB1CCR1 = 16394;
   TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR | CNTL__16 | ID_3; // SMCLK/8, up mode



  // PWM Config (TB0)
  TB0CCTL1 = OUTMOD_7;                            // CCR1 reset/set
  TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR | CNTL__16;
  TB0CCR1 = UMAX*0;

//  while(1){
//      __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0, enable interrupts
//      TB1CCR1 = ReceivedValue;
//
//  }
  ADCCTL0 |= ADCENC;                            // Sampling and conversion start


  while(1)
  {
//      __bis_SR_register(LPM0_bits | GIE);                   // LPM0, ADC_ISR will force exit

      PID_function(ADC_Result);
      memcpy(&TransmitBuffer[6], &ADC_Result, 2);
      if(u>=0){
          if (u>UMAX){
              TB0CCR1 = UMAX;
              memcpy(&TransmitBuffer[4], &u, sizeof(u));
          }
          else{
              TB0CCR1 = u;
              memcpy(&TransmitBuffer[4], &u, sizeof(u));
          }
      }
      else{
          TB0CCR1 = 0;
          memcpy(&TransmitBuffer[4], &u, sizeof(u));
      }
      __delay_cycles(160000);
  }
}

// Functions Definitions

void PID_function(uint16_t sensor){
    // Control Logic
    error = ideal;
    error = error - sensor;
    u = KP*error;                // Q0 = Q0*Q0
    u = (u<<16) + KI*error_i;            // Q16 = Q16 + Q16
    u = u>>16;


    // Anti-windup
    if((u >= UMAX || u < 0) && (((error >= 0) && (error_i >= 0)) || ((error < 0) && (error_i < 0)))){
        error_i = error_i;
    }
    else{
        error_i = error_i + DT*error;  // Q16
    }
}



// INTERRUPTS

// UART

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P2OUT &= ~BIT0;
    P1OUT &= ~BIT0;
    uint8_t ucb0_rx_val = UCB0RXBUF;
//    UCB0IFG &= ~UCRXIFG;

    ReceiveBuffer[ReceiveIndex++] = ucb0_rx_val;

    switch(ReceiveBuffer[0]){
        case 0xFF:
            UCB0TXBUF = ReceiveBuffer[0];
//            if(TransmitIndex < sizeof(TransmitBuffer)){
//                UCB0TXBUF = TransmitBuffer[TransmitIndex];
                break;
//            }

        case 0x00:
            UCB0TXBUF = ReceiveBuffer[0];
//            if(ReceiveIndex > 2){
//                ReceivedValue = ((ReceiveBuffer[2]<<8)|ReceiveBuffer[3]);
//                ideal = ReceivedValue;
                break;
//            }
    }

    if(ReceiveIndex == sizeof(ReceiveBuffer)){
        ReceiveIndex = 0;
        TransmitIndex = 0;
//        __bic_SR_register_on_exit(LPM0_bits);              // Clear CPUOFF bit from LPM0
    }


//    UCB0TXBUF = ucb0_rx_val;
}


// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
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
            ADC_Result = ADCMEM0;

//            __bic_SR_register_on_exit(LPM0_bits);              // Clear CPUOFF bit from LPM0
            break;
        default:
            break;
    }
}



