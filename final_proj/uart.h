/*
 * uart.h
 *
 *      Author: rockey
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

// These two varbles have been declared
// in the file containing main
extern volatile  char uart_data;  // Your UART interupt code can place read data here
extern volatile  char flag;       // Your UART interupt can update this flag
                                  // to indicate that it has placed new data
                                  // in uart_data


// initialize registers for UART communication given a baud rate
void uart_init(int baud);

// send a char over UART
void uart_sendChar(char data);

// receive a char over UART
char uart_receive(void);

// send a string (multiple chars and a \0 bit) over UART
void uart_sendStr(const char *data);

// initialize registers for interrupts for UART
void uart_interrupt_init();

// handle interrupts for UART
void uart_interrupt_handler();

#endif /* UART_H_ */
