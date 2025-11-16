/*
 * button.c
 *
 *  Created on: Jul 18, 2016
 *      Author: Eric Middleton, Zhao Zhang, Chad Nelson, & Zachary Glanz.
 *
 *  @edit: Lindsey Sleeth and Sam Stifter on 02/04/2019
 *  @edit: Phillip Jone 05/30/2019: Mearged Spring 2019 version with Fall 2018
 */
 


//The buttons are on PORTE 3:0
// GPIO_PORTE_DATA_R -- Name of the memory mapped register for GPIO Port E, 
// which is connected to the push buttons
#include "button.h"


/**
 * Initialize PORTE and configure bits 0-3 to be used as inputs for the buttons.
 */
void button_init() {
	static uint8_t initialized = 0;

	//Check if already initialized
	if(initialized){
		return;
	}

	// delete warning after implementing 

	
	// Reading: To initialize and configure GPIO PORTE, visit pg. 656 in the 
	// Tiva datasheet.
	
	// Follow steps in 10.3 for initialization and configuration. Some steps 
	// have been outlined below.
	
	// Ignore all other steps in initialization and configuration that are not 
	// listed below. You will learn more about additional steps in a later lab.

	// 1) Turn on PORTE system clock, do not modify other clock enables
	SYSCTL_RCGCGPIO_R |= 0b00010000;

	// 2) Set the buttons as inputs, do not modify other PORTE wires
	GPIO_PORTE_DIR_R &= 0b11110000;
	
	// 3) Enable digital functionality for button inputs, 
	//    do not modify other PORTE enables
	GPIO_PORTE_DEN_R |= 0b00001111;

	
	initialized = 1;
}



/**
 * Returns the position of the rightmost button being pushed.
 * @return the position of the righttmost button being pushed. 4 is the rightmost button, 1 is the leftmost button.  0 indicates no button being pressed
 */
uint8_t button_getButton() {



	//
	// DELETE ME - How bitmasking works
	// ----------------------------------------
	// In embedded programming, often we only care about one or a few bits in a piece of 
	// data.  There are several bitwise operators that we can apply to data in order
	// to "mask" the bits that we don't care about.
	//
	//	| = bitwise OR		& = bitwise AND		^ = bitwise XOR		~ = bitwise NOT
	//		  << x = shift left by x bits		 >> x = shift right by x bits 
	//
	// Let's say we want to know if push button 3 (S3) of GPIO_PORTE_DATA_R is
	// pushed.  Since push buttons are high (1) initially, and low (0) if pushed, PORTE should
	// look like:
	// GPIO_PORTE_DATA_R => 0b???? ?0?? if S3 is pushed
	// GPIO_PORTE_DATA_R => 0b???? ?1?? if S3 is not pushed
	//
	// This is not useful: There are 128 different 8 bit numbers that have the 3rd bit high or low.
	// We can make it more clear if we mask the other 7 bits:
	//	
	// Bitwise AND:
	// (GPIO_PORTE_DATA_R & 0b0000 0100) => 0b0000 0000 if S3 is pushed
	// (GPIO_PORTE_DATA_R & 0b0000 0100) => 0b0000 0100 if S3 is not pushed
	//
	// Bitwise OR:
	// (GPIO_PORTE_DATA_R | 0b1111 1011) => 0b1111 1011 if S3 is pushed
	// (GPIO_PORTE_DATA_R | 0b1111 1011) => 0b1111 1111 if S3 is not pushed
	//
	// Other techniques (Shifting and bitwise AND)
	// ((GPIO_PORTE_DATA_R >> 2) & 1) => 0 if S3 is pushed
	// ((GPIO_PORTE_DATA_R >> 2) & 1) => 1 if S3 is not pushed

	// TODO: Write code below -- Return the left must button position pressed
	
	// INSERT CODE HERE!
	uint8_t button = 0;
	uint8_t porte = GPIO_PORTE_DATA_R;

	char buttons[4];
	buttons[3] = (porte & (1<<3)) >>3;
	buttons[2] = (porte & (1<<2)) >>2;
	buttons[1] = (porte & (1<<1)) >>1;
	buttons[0] = (porte & (1<<0)) >>0;

	if(!buttons[3]){
	    button=4;
	}
	else if(!buttons[2]){
	    button=3;
	}
	else if(!buttons[1]){
	    button=2;
	}
	else if(!buttons[0]){
	    button=1;
	}
	else button = 0;


//	if (porte ==15) {
//	    button = 1;
//	} else if (porte == 13) {
//	    button = 2;
//	} else if (porte == 11) {
//	    button = 3;
//	} else if (porte == 7) {
//	    button = 4;
//	}

	return button; // EDIT ME
}





