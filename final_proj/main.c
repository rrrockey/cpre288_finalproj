#include "open_interface.h"
#include "Timer.h"
#include "uart.h"
#include "servo.h"
#include "button.h"
#include "adc.h"
#include <math.h>
#include "movement_f1.h"
#include <stdio.h>
#include <stdlib.h>
#include "ping.h"
#include <inc/tm4c123gh6pm.h>
/**
 * main.c
 */

typedef struct {
    int start_angle;
    int end_angle;
    int mid_angle;
    double ping_distance;
    double width;
} object_t;


int main(void)
{
    timer_init();
    lcd_init();
    uart_init();
    servo_init();
    ping_init();
    adc_init();
    button_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    //servo_calibrate();
    int stop = 0;
    int turnStatus = 0;
    double horizontalSpan =0;
    double verticalSpan = 0;


    while (!stop)
    {
        oi_update(sensor_data);
        int pingVal = (((ping_read()/2)*.5)*34000)/16000000;
        lcd_printf("%d, %d, %d", pingVal, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        int status = move_forward(sensor_data, fmin(200, pingVal-10));

        if(pingVal < 10){
            avoidObject(sensor_data);

        }
        if (status == BOUNDARY)
        {
            if (!turnStatus)
            {
                turnStatus = 1;
                turn_counterclockwise(sensor_data, 90);
                sensor_data->distance = 0;
            }
            else
            {
                turn_counterclockwise(sensor_data, 180);
                stop = 1;
            }
        }
        else if (status == CLIFF)
        {
            stop = 1;
        }
        if (!turnStatus)
        {
            horizontalSpan += sensor_data->distance;
        }
        else
        {
            verticalSpan += sensor_data->distance;
        }
    }
//    int RightOrLeft = 0; // 0 is right, 1 is left
//
//   // int adcVal =0;
//    object_t[10] objects;
//    while(1){
//        if(RightOrLeft){
//            for(int i = 0 + 60*RightOrLeft; i<120 +60*RightOrLeft; i++){
//               // adcVal = adc_read();
//                ticks = ping_getPulseWidth();
//                time_ms = ping_getTimeMs(ticks);
//                pingVal = ping_getDistanceCm(time_ms);
//                //double  estimation = 0.0000228813 * (adcValue * adcValue) - 0.0981288 * adcValue + 115.33455;
//
//            }
//        }
//
//        //scan area
//
//        //drive short distance in y
//
//        //move 35 cm downwards in x
//
//
//    }
    //lcd_printf("%.2f, %.2f", horizontalSpan, verticalSpan);

    oi_free(sensor_data);

    return 0;
}

void avoidObject(oi_t *sensor_data)
{
    turn_counterclockwise(sensor_data, 90);
    int pingVal = (((ping_read() / 2) * .5) * 34000) / 16000000;


    if (pingVal < 10)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35);
    turn_clockwise(sensor_data, 90);

    pingVal = (((ping_read() / 2) * .5) * 34000) / 16000000;
    if (pingVal < 10)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35 /*possibly, add the size of the pillar*/);
    turn_clockwise(sensor_data, 90);
    pingVal = (((ping_read() / 2) * .5) * 34000) / 16000000;
    if (pingVal < 10)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35);
    turn_counterclockwise(sensor_data, 90);

    //drive forward

}

