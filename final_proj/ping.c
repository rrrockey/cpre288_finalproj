

/*
 * ping.c
 *
 *  Created on: Oct 30, 2025
 *      Author: jkmurphy
 */

#include "timer.h"
#include "lcd.h"
#include "ping.h"
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include "math.h"
#include "uart.h"

volatile int rising_time = 0;
volatile int falling_time = 0;
volatile int overflowCount = 0;
volatile enum {LOW, HIGH, DONE} state;



void ping_init(void) {
    // Enable Port B clock
    SYSCTL_RCGCGPIO_R |= 0b00000010;
    SYSCTL_RCGCTIMER_R |= 0b1000;

    // Disable alternate function on PB3 (GPIO mode)
    GPIO_PORTB_AFSEL_R &= ~0x08;

    // Enable digital functionality on PB3
    GPIO_PORTB_DEN_R |= 0x08;

    // Set PB3 as output
    GPIO_PORTB_DIR_R |= 0x08;

    // Ensure PB3 starts low
    GPIO_PORTB_DATA_R &= ~0x08;

    //TIMER3 ENABLE
    TIMER3_CTL_R |= 0b0000110000000000;
    TIMER3_CTL_R &= ~(0x100); //disable timer
    TIMER3_CFG_R |= 0x4;
    TIMER3_TBMR_R = 0b000000000111;
    TIMER3_TBILR_R = 0xFFFF;
    TIMER3_TBPR_R = 0xFF; 
    TIMER3_ICR_R |= 0b10000000000;
    TIMER3_IMR_R = 0b10000000000;

    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntMasterEnable();

    state = DONE;
}

void send_pulse(void) {
    // Generate short high pulse (~5 �s)

    TIMER3_CTL_R &= ~(0x100);  // maybe move

    GPIO_PORTB_AFSEL_R &= ~0x08;

    GPIO_PORTB_DIR_R |= 0x08;
    GPIO_PORTB_DATA_R &= ~(0x08);

        // Generate short HIGH pulse (5 �s per datasheet)
        GPIO_PORTB_DATA_R |= 0x08;        // HIGH
        timer_waitMicros(5);
        GPIO_PORTB_DATA_R &= ~0x08;
        GPIO_PORTB_DIR_R &= ~(0x08);
}

unsigned int ping_read(void) {
    //uart_init();
    //char buffer[200];
    if (state != DONE){
        return;
    }
    // For Part 1: only send the trigger pulse
              // mask capture event
    TIMER3_IMR_R &= 0b101111111111;
    send_pulse();
    state = LOW;
    GPIO_PORTB_AFSEL_R |= 0x08;
    GPIO_PORTB_PCTL_R &= 0xFFFF0FFF;
    GPIO_PORTB_PCTL_R |= 0x00007000;
    TIMER3_ICR_R |= 0b010000000000;
    TIMER3_IMR_R |= 0b010000000000;



    //unsigned int conversion2 = (((rising_time - falling_time) * 0.5 ) *34000) / 16000000;


    TIMER3_CTL_R |= 0x100;

    while(state != DONE){};
    if(rising_time > falling_time){
        overflowCount++;

        return 0xDFFFFF & (rising_time - falling_time);
    }

    return rising_time - falling_time; // at 133 it returns a negative value


}

void TIMER3B_Handler(){
    TIMER3_ICR_R |= 0b010000000000;
        if(state == LOW){
            rising_time = TIMER3_TBR_R;
            state = HIGH;

        }
        else if(state == HIGH){
            falling_time = TIMER3_TBR_R;
            state = DONE;
        }



}
int getOverflowCount(){
    return overflowCount;
}

