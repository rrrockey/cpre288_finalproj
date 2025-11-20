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

void avoidObject(oi_t *sensor_data);
scan_info scan_cone(int low, int high);
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

    char buffer[200];
    scan_info scanData;//servo_calibrate();
    int stop = 0;
    int turnStatus = 0;
    double horizontalSpan =0;
    double verticalSpan = 0;
    double estimation = 0;
    while(1){
        int button = button_getButton();
        if(button == 1){



    while (!stop)
    {
        oi_update(sensor_data);
//        int pingVal = (((ping_read()/2)*.5)*34000)/16000000;
//        int IR_val = adc_read();
        scanData = scan_cone(60,120);
//         estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
        lcd_printf("%d, %.2f, %d, %d", scanData.averagePing, scanData.averageAdc, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        int driveDist = fmin(200, scanData.averagePing);
        int status = move_scan(sensor_data, driveDist, 60, 120);
        if(OBJECT == status){

            scanData = scan_cone(60,120);
        }
        sprintf(buffer, "scan data ADC %d, PING  %.2f \r\n", scanData.averageAdc, scanData.averagePing);
                            uart_sendStr(buffer);
        if(scanData.averagePing < 10){
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
        }}
    oi_free(sensor_data);

    return 0;
}

void avoidObject(oi_t *sensor_data)
{
    scan_info scanData;
    char buffer[200];
    turn_counterclockwise(sensor_data, 90);
    double adcValue = 0;
    scanData = scan_cone(45, 135);

    sprintf(buffer, "adcValue in recursion: %.2f", scanData.averageAdc);
    uart_sendStr(buffer);
    if (scanData.averageAdc < 25)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35);
    turn_clockwise(sensor_data, 90);

    scanData = scan_cone(45, 135);
    if (scanData.averageAdc < 25)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35 /*possibly, add the size of the pillar*/);
    turn_clockwise(sensor_data, 90);
    scanData = scan_cone(45, 135);
    if (scanData.averageAdc < 25)
    {
        avoidObject(sensor_data);
    }
    move_forward(sensor_data, 35);
    turn_counterclockwise(sensor_data, 90);

    //drive forward

}

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
