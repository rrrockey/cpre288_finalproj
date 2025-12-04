/*
*
*   uart.c
*
*
*
*
*
*   @author
*   @date
*/

#include "uart.h"
#include "timer.h"
#include <stdlib.h>

volatile char inputChar;


void uart_init(void){
    SYSCTL_RCGCGPIO_R |=  0b00000010;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCUART_R |=  0b00000010;      // enable clock UART1 (page 344)
    GPIO_PORTB_AFSEL_R |= 0b00000011;      // sets PB0 and PB1 as peripherals (page 671)
    GPIO_PORTB_PCTL_R  &= 0xFFFFFF00;       //cleaing pin 1 and 0 before adding
    GPIO_PORTB_PCTL_R  |= 0x00000011;       // pmc0 and pmc1       (page 688)  also refer to page 650
    GPIO_PORTB_DEN_R   |= 0b00000011;        // enables pb0 and pb1
    GPIO_PORTB_DIR_R   |= 0b00000010;        // sets pb0 as output, pb1 as input
    GPIO_PORTB_DIR_R   &= 0b11111110;           //set pb0

    //compute baud values [UART clock= 16 MHz]
    double fbrd;
    int    ibrd;

     // page 903
    ibrd = 8;
    fbrd = 44;

    UART1_CTL_R &= 0xFFFE;      // disable UART1 (page 918)
    UART1_IBRD_R = ibrd;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = fbrd;   // write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0b0000;  // use system clock as clock source (page 939)
    UART1_CTL_R |= 0b1100000001;        // enable UART1


}

void init_uart_interrupts(void) {
    UART1_CTL_R &= 0xFFFE;      // disable UART1 (page 918)

    UART1_ICR_R |= 0b00010000;

    UART1_IM_R |= 0x10;

    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    NVIC_EN0_R |= 0x00000040;

    IntRegister(INT_UART1, gpiob_handler);

    IntMasterEnable();

    UART1_CTL_R |= 0b1100000001;

}

void gpiob_handler(void){


  //Rec
  if((UART1_MIS_R & 0b000000010000) == 0b000000010000){
        //UART1_DR_R & 0xFF;
        UART1_ICR_R |= 0b000000010000;


        inputChar = (char) UART1_DR_R & 0xFF;

        UART1_DR_R = inputChar;
        lcd_putc(inputChar);

   }



}




void uart_sendChar(char data){
    while (UART1_FR_R & (1 << 5));
	UART1_DR_R = data;
}

char uart_receive(void){

    while (UART1_FR_R & 0x10){
    }
    if(UART1_DR_R & 0xFF == '\r'){
        uart_sendChar('\n');
    }
    return (char)(UART1_DR_R & 0xFF);
}


void uart_sendStr(const char *data){
    int i =0;
    while(data[i] != '\0') {
        uart_sendChar(data[i]);
        i++;
    }
}

double uart_receive_double(void) {
    char buffer[32];
    int i = 0;

    char c;
    while ((c = uart_receive()) != '\r') {
        uart_sendChar(c);
        buffer[i++] = c;
    }
    buffer[i] = '\0';

    uart_sendStr("double in uart receive double: ");
    uart_sendStr(buffer);

    return atof(buffer);
}





