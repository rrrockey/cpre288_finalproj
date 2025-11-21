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

typedef struct {
    int averagePing;
    double averageAdc;
} scan_info;

#define POSITIVE_Y 0
#define POSITIVE_X 1
#define NEGATIVE_Y 2
#define NEGATIVE_X 3
int facing_array[4] = {POSITIVE_Y, POSITIVE_X, NEGATIVE_Y, NEGATIVE_X};

scan_info scan_cone(int low, int high){
    scan_info scanData = {0,0};
    int adcTickAmnt = 0;
    int pingTickAmnt = 0;
    double averageADC = 0;
    int averagePing = 0;

    double estimation = 0;
    int pingVal = 0;
    int adcVal = 0;
    int i =low;
    char buffer[200] = "";
    for(i =low; i<high; i++){
        servo_move(i);
        adcVal = adc_read();
//        estimation = 0.0000228813 * (adcVal * adcVal) - 0.0981288 * adcVal + 115.33455;
        estimation = 4150000 * pow(adcVal, -1.64);
        if(estimation < 25){
            sprintf(buffer, "Object hit at: %.2f, at angle %d \r\n", estimation,i);
                    uart_sendStr(buffer);
            adcTickAmnt++;
            averageADC += estimation;

        }

        pingVal = (((ping_read()/2)*.5)*34000)/16000000;
        if(pingVal < 25){
            pingTickAmnt++;
            averagePing+= pingVal;
        }

        sprintf(buffer, "angle: %d scan: %.2f, average: %.2f, object count: %d \r\n",i,  estimation, averageADC / ((double)adcTickAmnt), adcTickAmnt);
        uart_sendStr(buffer);



    }

    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!");
                uart_sendStr(buffer);
        scanData.averageAdc = 100;
        scanData.averagePing = 100;
        return scanData;
    }
    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;
    sprintf(buffer, "Final output:  average: %.2f, object count: %d \r\n", averageADC, adcTickAmnt);
            uart_sendStr(buffer);
    lcd_printf("%.2f, %d", averageADC, adcTickAmnt);
    scanData.averageAdc = averageADC;
    scanData.averagePing = averagePing;
    if(adcTickAmnt > 0){
        return scanData;
    }
    else{
        scanData.averageAdc = 100;
                scanData.averagePing = 100;
                return scanData;
    }

}

void avoidObject(oi_t *sensor_data)
{
    int objects_passed = 0; // bool

    sensor_data->distance = 0;
    double zero_y_distance = sensor_data->distance;
    double zero_x_distance = sensor_data->distance;
    double distance_traveled_x = 0;
    double distance_traveled_y = 0;
    int facing = POSITIVE_Y;

    while(!objects_passed) {
        oi_update(sensor_data);
        scan_info distance_to_object = scan_cone(60, 120);
        lcd_printf("average distance: %.2f", distance_to_object.averageAdc);
        if (distance_to_object.averageAdc < 25) { // if close object in front of bot (facing positive y)
            turn_counterclockwise(sensor_data, 90);
            facing = NEGATIVE_X;
            int found_object_to_left = 1; // assume object to the left
            while (found_object_to_left) { //facing -x
                oi_update(sensor_data);
//                turn_counterclockwise(sensor_data, 90);
//                facing = NEGATIVE_X; // turn back to the left   facing -x
                distance_to_object = scan_cone(60, 120);
                lcd_printf("average distance: %.2f", distance_to_object.averagePing);
                if (distance_to_object.averagePing > 50) { // clear path to move
                    found_object_to_left = 0;
                    oi_update(sensor_data);
                    double distance_before = sensor_data->distance*10; //call
                    move_forward(sensor_data, 50);
                    oi_update(sensor_data);
                    double distance_moved = sensor_data->distance*10 - distance_before;
                    lcd_printf("traveled %.2f cm", (distance_moved));
                    timer_waitMillis(1000);
                    distance_traveled_x -= distance_moved; //subtract bc moving on -x direction
                    turn_clockwise(sensor_data, 90);
                    facing = POSITIVE_Y;        // facing +y, break
                }
                else { // object   facing -x
                    turn_counterclockwise(sensor_data, 90);
                    facing = NEGATIVE_Y;
                    oi_update(sensor_data);
                    double distance_before = sensor_data->distance*10; //call
                    move_forward(sensor_data, 35+ 6);
                    oi_update(sensor_data);
                    double distance_moved = sensor_data->distance*10 - distance_before;
                    distance_traveled_y -= distance_moved; //subtract bc moving on -y direction
                    turn_clockwise(sensor_data, 90);
                    facing = NEGATIVE_X; // turn back to the left   facing -x
                }
                lcd_printf("moving forward %.2f cm", (-distance_traveled_y));
                timer_waitMillis(1000);
                move_forward(sensor_data, -(distance_traveled_y) + 35);
                turn_clockwise(sensor_data, 90);
                facing = POSITIVE_X;
                lcd_printf("moving forward %.2f cm", (-distance_traveled_x));
                timer_waitMillis(1000);
                move_forward(sensor_data, -(distance_traveled_x));
                turn_counterclockwise(sensor_data, 90);
                facing = POSITIVE_Y;
                while (1) {
                    lcd_printf("success");
                }
            }

        }
    }
}

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
    double estimation = 0;
    double distance_traveled = 0;
    char buffer[100];

    while(1){
        int button = button_getButton();
        if(button == 1){

        while (!stop)
        {
            sprintf(buffer, "distance movead: %.2f\r\n", distance_traveled);
            uart_sendStr(buffer);
            oi_update(sensor_data);
            int pingVal = (((ping_read()/2)*.5)*34000)/16000000;
            int IR_val = adc_read();
            estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
            lcd_printf("%d, %.2f, %d, %d", pingVal, estimation, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
            servo_move(90);
            int status = move_scan(sensor_data, fmin(200, pingVal-10), 60, 120, &distance_traveled);

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
            else if (status == OBJECT) {
                avoidObject(sensor_data);
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

        }
    }
    oi_free(sensor_data);

    return 0;
}





