#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "button.h"

/*
 * servo.h
 *
 *      Author: rockey
 */

#define PERIOD_TICKS 320000  // 20 ms period @ 16 MHz
#define SERVO_0DEG_PULSE (320000-312201)   // 1.0 ms high pulse
#define SERVO_180DEG_PULSE (320000-285400) // 2.0 ms high pulse
#define SERVO_RANGE (SERVO_180DEG_PULSE - SERVO_0DEG_PULSE)

// Initialize Timer1B to generate PWM on PB5
void servo_init(void);

// Move servo to specified pulse width (in timer ticks)
void servo_move_ticks(uint32_t pulse_width_ticks);

// Move servo to specified angle
void servo_move(float angle);

// Start calibration sequence to get pulse width
void servo_calibrate(void);
