/*
 * ping.h
 *
 *  Created on: Oct 30, 2025
 *      Author: jkmurphy
 */

void ping_init();

unsigned int ping_read();

void TIMER3B_Handler();

void send_pulse();

int getOverflowCount();
