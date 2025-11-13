#ifndef PING_H_
#define PING_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

// Initialize ping sensor. Uses PB3 and Timer 3B
void ping_init (void);


void ping_trigger (void);

/**
 * @brief Timer3B ping ISR
 */
void TIMER3B_Handler(void);

unsigned long ping_getPulseWidth(void);

float ping_getTimeMs(unsigned long ticks);

float ping_getDistanceCm(float time_ms);

#endif /* PING_H_ */
