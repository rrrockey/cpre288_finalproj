#include "timer.h"
#include "stdint.h"
#include "lcd.h"
#include "button.h"




void servo_init(void){


    SYSCTL_RCGCGPIO_R |= 0b10;
    SYSCTL_RCGCTIMER_R |= 0b10;

    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;

    GPIO_PORTB_AFSEL_R |=0b00100000;
    GPIO_PORTB_PCTL_R &= 0xFF0FFFFF;
    GPIO_PORTB_PCTL_R |= 0x00700000;

    TIMER1_CTL_R |=0b00110000000000;
    TIMER1_CFG_R |= 0x4;
    TIMER1_TBMR_R |= 0b000000001010;
    TIMER1_TBILR_R = 0xE200; // 04E2
    TIMER1_TBPMR_R = 0x04;
    TIMER1_TBMATCHR_R = 0x8440;
    TIMER1_TBPR_R |= 0x04; //
    TIMER1_CTL_R |= 0x100;

    servo_move(0);


}

int servo_move(float degrees){
//bot 1
    int max = 320000 - 6810;
    int low = 320000 - 34953;
    uint32_t match_val = max + ((-1 * ((max - low) / 180)) *degrees);
    TIMER1_TBMATCHR_R = (0x00FFFF &  match_val);
    TIMER1_TBPMR_R =  match_val >> 16;

    timer_waitMillis(20);


    return 1;
}
void servo_calibrate()
{
    button_init();
    servo_init();
    int clockwise = 1;
    unsigned int angle = 90;
    while (1)
    {




        if (button_getButton() == 1)
        {
            //if(!((angle >179 && clockwise == 1 )|| (angle<1 &&clockwise == -1))){
                angle += clockwise;
            servo_move(angle);
            //}

        }

        else if (button_getButton() == 2)
        {
            //if(!((angle >174 && clockwise == 1 )|| (angle<6 &&clockwise == -1))){
                angle += 5*clockwise;
            servo_move(angle);
            //}

        }
        else if (button_getButton() == 3)
        {
            clockwise *= -1;
        }
        else if (button_getButton() == 4)
        {
            if (clockwise == 1)
            {
                angle = 0;
                servo_move(0);
            }
            else{
                angle = 180;
                servo_move(180);
            }
        }
        timer_waitMillis(5);
        uint32_t highCount = (TIMER1_TBPMR_R >> 16) + TIMER1_TBMATCHR_R - 16000;

        lcd_printf("the current match value is %d  %d %d", highCount, clockwise,  angle);
    }
}

