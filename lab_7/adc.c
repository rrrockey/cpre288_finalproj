#include "adc.h"
#include <math.h>

//initialize ADC
void  adc_init_pb4(void) {
    //enable clock port b
    SYSCTL_RCGCGPIO_R |= 0x02;
    //enable adc
    SYSCTL_RCGCADC_R |= 0x01;
    // enable alt function for port b
    GPIO_PORTB_AFSEL_R |= 0x10;
    // disable digital port b
    GPIO_PORTB_DEN_R &= ~0x10;
    // enable analog port b
    GPIO_PORTB_AMSEL_R |= 0x10;
    // disable ss3
    ADC0_ACTSS_R &= ~0x08;
    // software trigger
    ADC0_EMUX_R &= ~0xF000;
    // AIN10 on PB4
    ADC0_SSMUX3_R = 10;

    ADC0_SSCTL3_R = 0x06;
    // reenable ss3
    ADC0_ACTSS_R |= 0x08;
}

//read value
int adc_read_pb4(void) {
    ADC0_PSSI_R = 0x08;
    while((ADC0_RIS_R & 0x08) == 0);
    int reslt = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R = 0x08;
    return reslt;
}

//converts the raw adc value from the IR to the distance in CM
double adc_to_distance(int raw_adc) {
    double distance = 4150000 * pow(raw_adc, -1.64);
    return distance;
}
