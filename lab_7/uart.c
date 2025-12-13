/*
*
*   uart_extra_help.c
* Description: This is file is meant for those that would like a little
*              extra help with formatting their code, and followig the Datasheet.
*/

#include "uart.h"
#include "timer.h"

// indicates if char was read from input
volatile bool char_ready = 0;


void uart_init(int baud)
{
    SYSCTL_RCGCGPIO_R |= 0x2;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCUART_R |= 0x2;      // enable clock UART1 (page 344)
    GPIO_PORTB_AFSEL_R |= 0x3;      // sets PB0 and PB1 as peripherals (page 671)
    GPIO_PORTB_PCTL_R  &= ~0xFF; //clear
    GPIO_PORTB_PCTL_R  |= 0x11;       // pmc0 and pmc1       (page 688)  also refer to page 650
    GPIO_PORTB_DEN_R   |= 0x3;        // enables pb0 and pb1
    GPIO_PORTB_DIR_R = (GPIO_PORTB_DIR_R & ~0x2) | 0x1;   // sets pb0 as output, pb1 as input

    //compute baud values [UART clock= 16 MHz]
    double fbrd;
    int    ibrd;

    fbrd = 16000000.0 / (16.0 * (double)baud); // page 903
    ibrd = (int)fbrd; // integer part
    fbrd = ((fbrd-ibrd) * 64 + 0.5); // fraction part


    UART1_CTL_R &= ~0x1;      // disable UART1 (page 918)
    UART1_IBRD_R = ibrd;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = (int)fbrd;   // write fractional portion of BRD to FBRD

    UART1_LCRH_R |= 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0x0;          // use system clock as clock source (page 939)
    UART1_CTL_R |= 0x0301;        // enable UART1 and RXE and TXE
}



void uart_sendChar(char data)
{
    // wait until there is room to send data
    while (UART1_FR_R & UART_FR_TXFF){  // UART Transmit FIFO Full
        if (UART1_FR_R == 0) {
            break;
        }
    }

    // send data
    UART1_DR_R = data;
   
}

char uart_receive(void){
    char data = 0;
    while (UART1_FR_R & UART_FR_RXFE) {
        if (UART1_FR_R == 0) {
            break;
        }
    }
    data = (char) (UART1_DR_R & 0xFF);
    return data;
}

char uart_receive_interrupt(void){
    while(!char_ready); // wait until data was received via interrupt

    char data = 0;
    while (UART1_FR_R & UART_FR_RXFE) {
        if (UART1_FR_R == 0) {
            break;
        }
    }
    data = (char) (UART1_DR_R & 0xFF);
    char_ready = 0; // set back to 0 until new char is received
    return data;
}



void uart_sendStr(const char *data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        uart_sendChar(data[i]);
        i++;
    }
}

// _PART3


void uart_interrupt_init()
{

  //enable clock to GPIO port B
  SYSCTL_RCGCGPIO_R |= 0x02;

  //enable clock to UART1
  SYSCTL_RCGCUART_R |= 0x02;

  //wait for GPIOB and UART1 peripherals to be ready
  while ((SYSCTL_PRGPIO_R & 0x02) == 0);
  while ((SYSCTL_PRUART_R & 0x02) == 0);

  //enable digital functionality on port B pins
  GPIO_PORTB_DEN_R |= 0x03;

  //enable alternate functions on port B pins
  GPIO_PORTB_AFSEL_R |= 0x03;

  //enable UART1 Rx and Tx on port B pins
  GPIO_PORTB_PCTL_R = 0x00000011;

  double fbrd;
  int    ibrd;

  fbrd = 16000000.0 / (16.0 * (double)115200); // page 903
  ibrd = (int)fbrd; // integer part
  fbrd = ((fbrd-ibrd) * 64 + 0.5); // fraction part

  UART1_CTL_R &= ~0x1;      // disable UART1 (page 918)
  UART1_IBRD_R = ibrd;        // write integer portion of BRD to IBRD
  UART1_FBRD_R = (int)fbrd;   // write fractional portion of BRD to FBRD

  //set frame, 8 data bits, 1 stop bit, no parity, no FIFO
  //note: this write to LCRH must be after the BRD assignments
  UART1_LCRH_R = 0x60;

  //use system clock as source
  //note from the datasheet UARTCCC register description:
  //field is 0 (system clock) by default on reset
  //Good to be explicit in your code
  UART1_CC_R = 0x0;

  //////Enable interrupts

  //first clear RX interrupt flag (clear by writing 1 to ICR)
  UART1_ICR_R |= 0b00010000;

  //enable RX raw interrupts in interrupt mask register
  UART1_IM_R |= 0x10;

  //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
  NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

  //NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
  NVIC_EN0_R |= 0x40;

  //tell CPU to use ISR handler for UART1 (see interrupt.h file)
  //from system header file: #define INT_UART1 22
  IntRegister(INT_UART1, uart_interrupt_handler);

  //globally allow CPU to service interrupts (see interrupt.h file)
  IntMasterEnable();

  //re-enable UART1 and also enable RX, TX (three bits)
  //note from the datasheet UARTCTL register description:
  //RX and TX are enabled by default on reset
  //Good to be explicit in your code
  //Be careful to not clear RX and TX enable bits
  //(either preserve if already set or set them)
  UART1_CTL_R = 0x0301;

}


void uart_interrupt_handler(void)
{
    // STEP1: Check the Masked Interrup Status
    if (UART1_MIS_R & 0b00010000) {
        // clear the flag
        UART1_ICR_R |= 0b00010000;

        // set flag to 1 ("char is ready to be read")
        char_ready = 1;
    }

}
