#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>

//initialize ADC
void  adc_init_pb4(void);

//read value
int adc_read_pb4(void);

//converts the raw adc value from the IR to the distance in CM
double adc_to_distance(int raw_adc);

#endif
