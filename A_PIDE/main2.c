#include <msp430.h>
#include <stdint.h>

void PID_function(uint16_t sensor);


#define UMAX    65535
#define DT      655
#define KP      5
#define KI      5
#define IDEAL   512


//unsigned int ADC_Result;
uint16_t ADC_Result;

int32_t error = 0;      // Q0
int64_t u = 0;
int64_t error_i = 0;


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                 // Stop WDT

    // Configure GPIO
    P1DIR |= BIT0;                                            // Set P1.0/LED to output direction
    P1OUT &= ~BIT0;                                           // P1.0 LED off

    // Configure ADC A5 pin
    P1SEL0 |= BIT5;
    P1SEL1 |= BIT5;

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

    // Configure ADC10
    ADCCTL0 |= ADCMSC | ADCON;                              // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP | ADCSHS_2 | ADCSSEL_2 | ADCCONSEQ_2;             // ADCCLK = MODOSC; sampling timer
    ADCCTL2 |= ADCRES;                                        // 10-bit conversion results
    ADCMCTL0 |= ADCINCH_5 | ADCSREF_1;                        // A5 ADC input select; Vref=1.5V
    ADCIE |= ADCIE0;                                          // Enable ADC conv complete interrupt


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
    TB0CCR1 = UMAX*0;
    TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR | CNTL__16;

    ADCCTL0 |= ADCENC;                            // Sampling and conversion start

    __bis_SR_register(LPM0_bits | GIE);

    while(1)
    {
        PID_function(ADC_Result);
        if(u>=0){
            if (u>UMAX){
                TB0CCR1 = UMAX;
            }
            else{
                TB0CCR1 = u;
            }
        }
        else{
            TB0CCR1 = 0;
        }
//        __delay_cycles(160000);
    }
}


void PID_function(uint16_t sensor){
    // Control Logic
    error = IDEAL;
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
            __bic_SR_register_on_exit(LPM0_bits);              // Clear CPUOFF bit from LPM0

            break;
        default:
            break;
    }
}

