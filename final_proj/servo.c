#include "servo.h"

// Initializes Timer1B to generate PWM on PB5
void servo_init(void) {
    // 1. Enable clocks for GPIOB and Timer1
    SYSCTL_RCGCGPIO_R |= 0x02;    // Port B
    SYSCTL_RCGCTIMER_R |= 0x02;   // Timer 1
    while ((SYSCTL_PRGPIO_R & 0x02) == 0); // Wait for GPIO ready

    // 2. Configure PB5 for T1CCP1 (Timer1B)
    GPIO_PORTB_AFSEL_R |= 0x20;      // Enable alternate function on PB5
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |=  0x00700000; // PB5 = T1CCP1
    GPIO_PORTB_DEN_R |= 0x20;         // Enable digital
    GPIO_PORTB_DIR_R |= 0x20;         // Set as output

    // 3. Disable Timer1B during configuration
    TIMER1_CTL_R &= ~0x0100;

    // 4. Configure Timer1 for PWM mode (16-bit, periodic)
    TIMER1_CFG_R = 0x4;            // 16-bit mode
    TIMER1_TBMR_R = 0x0A;          // PWM mode, periodic, count-down
    TIMER1_CTL_R &= ~0x4000;       // Output not inverted

    // 5. Set PWM period (20ms)
    TIMER1_TBILR_R = PERIOD_TICKS & 0xFFFF;         // Lower 16 bits
    TIMER1_TBPR_R = (PERIOD_TICKS >> 16) & 0xFF;    // Upper 8 bits

    // 6. Set initial match value for 0 position (1ms pulse)
    uint32_t match = PERIOD_TICKS - SERVO_0DEG_PULSE;
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R = (match >> 16) & 0xFF;

    // 7. Enable Timer1B
    TIMER1_CTL_R |= 0x0100;
}

// Moves servo to specified pulse width (in timer ticks)
void servo_move_ticks(uint32_t pulse_width_ticks) {
//    if (pulse_width_ticks > SERVO_180DEG_PULSE) pulse_width_ticks = SERVO_180DEG_PULSE;
//    if (pulse_width_ticks < SERVO_0DEG_PULSE) pulse_width_ticks = SERVO_0DEG_PULSE;

    uint32_t match = PERIOD_TICKS - pulse_width_ticks;
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R = (match >> 16) & 0xFF;
}

void servo_move(float angle) {
//    if (angle < 0) angle = 0;
//    if (angle > 180) angle = 180;

    uint32_t pulse_width = SERVO_0DEG_PULSE + (uint32_t)((angle / 180.0) * SERVO_RANGE);
    uint32_t match = PERIOD_TICKS - pulse_width;
    timer_waitMillis(20);
    TIMER1_TBMATCHR_R = match & 0xFFFF;
    TIMER1_TBPMR_R = (match >> 16) & 0xFF;
}

void servo_calibrate(void) {
    float angle = 90.0;      // start centered
    bool clockwise = true;   // start rotating CW
    uint8_t button;          // holds current button press

    servo_move(angle);
    lcd_printf("Angle: %.1f\nDir: CW\nMatch: %u", angle, PERIOD_TICKS - (SERVO_0DEG_PULSE + (uint32_t)((angle / 180.0) * SERVO_RANGE)));

    while (1) {
        button = button_getButton();

        if (button == 1) {          // SW1: move 1�
            if (clockwise) angle += 1;
            else angle -= 1;
        } else if (button == 2) {   // SW2: move 5�
            if (clockwise) angle += 5;
            else angle -= 5;
        } else if (button == 3) {   // SW3: toggle direction
            clockwise = !clockwise;
            timer_waitMillis(300);
        } else if (button == 4) {   // SW4: go to 0� or 180�
            if (clockwise) angle = 0;
            else angle = 180;
        }

        // Clamp within 0�180�
//        if (angle < 0) angle = 0;
//        if (angle > 180) angle = 180;

        // Move servo
        servo_move(angle);

        // Update LCD
        uint32_t match_val = PERIOD_TICKS - (SERVO_0DEG_PULSE + (uint32_t)((angle / 180.0) * SERVO_RANGE));
        lcd_printf("Angle: %.1f\nDir: %s\nMatch: %u",
                   angle, clockwise ? "CW  <--" : "CCW -->", match_val);

        timer_waitMillis(200);  // debounce and motion delay
    }
}
