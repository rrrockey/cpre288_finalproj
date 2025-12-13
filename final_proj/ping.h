/*
 * ping.h
 *
 *  Created on: Oct 30, 2025
 *      Author: jkmurphy
 */

// initialize ping
void ping_init();

// send short pulse to be listened back for
void send_pulse();

// read time difference in echo
unsigned int ping_read();

// set state of the listener based on echo
void TIMER3B_Handler();
 
// return number of overflows (used in lab)
int getOverflowCount();
