/*
 * adc.c
 *
 *  Created on: Oct 23, 2025
 *      Author: jjsadler
 */
#include "timer.h"
#include "adc.h"


// initialize ADC
void adc_init(void) {
    // define gpio ports
    SYSCTL_RCGCGPIO_R |= 0b00000010;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCADC_R |= 0x0001; // enabling clock for ADC0
    while((SYSCTL_PRGPIO_R & 0b10) != 0b10){};
    GPIO_PORTB_AMSEL_R |= 0b00010000; //ADC AM select for pin 4
    GPIO_PORTB_AFSEL_R |= 0b00010000; // sets PB4
    GPIO_PORTB_PCTL_R &= 0xFFF0FFFF; // pmc4      (page 688)  also refer to page 650
    GPIO_PORTB_DEN_R |= 0b00010000;        // enables pb4
    GPIO_PORTB_DIR_R &= 0b000000000;        // sets pb0 as output, pb1 as input
    GPIO_PORTB_ADCCTL_R |= 0b00000000; //ADC control for pin 4

    ADC0_PC_R &= ~0xF;
    ADC0_PC_R |= 0x1;
    ADC0_SSPRI_R = 0x0123;
    ADC0_ACTSS_R &= ~0x0008; //enabling ADC0
    ADC0_EMUX_R &= 0xF000; //enable gpio
    ADC0_SSMUX0_R &= ~0x000F;
    ADC0_SSMUX0_R |= 0xA;
    ADC0_SSCTL0_R = 0x0006;
    ADC0_SAC_R = 0x01;
    ADC0_ACTSS_R |= 0x0001; //enabling ADC0

}

// read raw ADC value of IR sensor
int adc_read(void) {
    timer_waitMillis(10);
    unsigned int result;

    ADC0_PSSI_R = 0x0001;

    result = ADC0_SSFIFO0_R & 0xFFF;


    return result;
}


