#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "button.h"


#define PERIOD_TICKS 320000  // 20 ms period @ 16 MHz
#define SERVO_0DEG_PULSE (320000-312550)   // 1.0 ms high pulse
#define SERVO_180DEG_PULSE (320000-284666) // 2.0 ms high pulse
#define SERVO_RANGE (SERVO_180DEG_PULSE - SERVO_0DEG_PULSE)

// Initializes Timer1B to generate PWM on PB5
void servo_init(void);

void servo_move_ticks(uint32_t pulse_width_ticks);

void servo_move(float angle);

void servo_calibrate(void);
