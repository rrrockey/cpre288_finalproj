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
    double adcWidth;
    double pingWidth;
    double driveDist;


} scan_info;

void avoidObject(oi_t *sensor_data,  scan_info *scanData);
void scan_cone(int low, int high, scan_info *scanData);
int rotate_degrees(int angle, int direction, oi_t *sensor_data);
void update_distance(double distance, int direction);
double horizontalPos = 0;
double verticalPos = 0;
#define POSITIVE_X = 0;
#define POSITIVE_Y = 1;
#define NEGATIVE_X = 2;
#define NEGATIVE_Y = 3;
int direction = 0; //0,1,2,3 for NWSE

int turnStatus = 0;

int rotate_degrees(int angle, int direction, oi_t *sensor_data){ //CCW is positive
    int angleChange = direction/90;
    if(angleChange>0){
        angle+= angleChange;
        angle %=4;
        turn_counterclockwise(sensor_data, angleChange*90);
    }
    else{
        angle--;
        if(angle<0){
            angle +=3;
            angleChange*=-1;
            turn_clockwise(sensor_data, angleChange*90);
        }
    }
}
void update_distance(double distance, int direction ){
    if(direction == 0){
        horizontalPos+=distance;
    }
    else if(direction ==1){
        verticalPos+=distance;
    }
    else if(direction ==2){
            horizontalPos-=distance;
        }
        else if(direction ==3){
            verticalPos-=distance;
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

    char buffer[200];
    scan_info scanData;
    scanData.averageAdc = 0;
    scanData.averagePing = 0;
    scanData.pingWidth = 0;
    scanData.adcWidth =0;

    int stop = 0;

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
        scan_cone(60,120, &scanData);
//         estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
        lcd_printf("%d, %.2f, %d, %d", scanData.averagePing, scanData.averageAdc, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        int driveDist = fmin(200, scanData.averagePing);
        update_distance(driveDist, direction);

        int status = move_scan(sensor_data, driveDist, 60, 120);
        if(OBJECT == status){

            scan_cone(60,120, &scanData);
        }
        sprintf(buffer, "scan data ADC %.2f, PING  %d \r\n", scanData.averageAdc, scanData.averagePing);
                            uart_sendStr(buffer);
        if(scanData.averagePing < 10){
            avoidObject(sensor_data , &scanData);

        }
        if (status == BOUNDARY)
        {
            if (!turnStatus)
            {

                turnStatus = 1;
                rotate_degrees(direction, -90, sensor_data);

                sensor_data->distance = 0;

            }
            else
            {
                rotate_degrees(direction, 180, sensor_data);


                stop = 1;
            }
        }
        else if (status == CLIFF)
        {
            stop = 1;
        }
        if (!turnStatus)
        {
            //horizontalSpan += sensor_data->distance;
        }
        else
        {
            //verticalSpan += sensor_data->distance;
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

void avoidObject(oi_t *sensor_data, scan_info *scanData)
{

    char buffer[50];
    //update_distance(driveDist, direction);
    rotate_degrees(direction, 90, sensor_data);


    scan_cone(45, 135, scanData);


    sprintf(buffer, "adcValue in recursion: %.2f", scanData->averageAdc);
    uart_sendStr(buffer);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }
    move_forward(sensor_data, scanData->driveDist);
    update_distance(scanData->driveDist, direction);
    rotate_degrees(direction, -90, sensor_data);


    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist)
    {
        avoidObject(sensor_data, scanData);
    }
    move_forward(sensor_data, scanData->driveDist /*possibly, add the size of the pillar*/);
    update_distance(scanData->driveDist, direction);
    rotate_degrees(direction, -90, sensor_data);

    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }
    move_forward(sensor_data, scanData->driveDist);
    update_distance(scanData->driveDist, direction);
//    if (!turnStatus)
//    {
//    if(direction != NEGATIVE_X)
//    {
//        direction = NEGATIVE_X;
//    }
//}
//else
//{
//if(direction != POSITIVE_Y)
//{
//    direction = POSITIVE_Y;
//}
//}





    turn_counterclockwise(sensor_data, 90);

    //drive forward

}

void scan_cone(int low, int high, scan_info *scanData){

    int adcTickAmnt = 0;
    int pingTickAmnt = 0;
    double averageADC = 0;
    int averagePing = 0;
    double adcRad = 0;
    double pingRad = 0;
    double pingWidth = 0;
    double adcWidth = 0;
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

        sprintf(buffer, "angle: %d scan: %.2f, avercage: %.2f, object count: %d \r\n",i,  estimation, averageADC / ((double)adcTickAmnt), adcTickAmnt);
        uart_sendStr(buffer);



    }

    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!");
                uart_sendStr(buffer);
        scanData->averageAdc = 100;
        scanData->averagePing = 100;
        return;
    }
    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;
    sprintf(buffer, "Final output:  average: %.2f, object count: %d \r\n", averageADC, adcTickAmnt);
            uart_sendStr(buffer);
    lcd_printf("%.2f, %d", averageADC, adcTickAmnt);
    scanData->averageAdc = averageADC;
    scanData->averagePing = averagePing;
     adcRad = (adcTickAmnt) * (3.14 / 180.0);
    adcWidth = 2 * averageADC * tan(adcRad / 2.0);

    pingRad = (pingTickAmnt) * (3.14 / 180.0);
    pingWidth = 2 *averagePing * tan(pingRad / 2.0);
    scanData->adcWidth = adcWidth;
    scanData->pingWidth = pingWidth;
    if (adcTickAmnt > 0)
    {

        if (scanData->adcWidth > 10)
        {
            scanData->driveDist = 17 + 35+scanData->averageAdc;
        }
        else if (scanData->adcWidth <= 10 && scanData->adcWidth > 5)
        {
            scanData->driveDist = 35 + 11+scanData->averageAdc;
        }
        else if (scanData->adcWidth > 0)
        {
            scanData->driveDist = 35+5+scanData->averageAdc;
        }

    }
    else
    {
        scanData->driveDist = 200;
    }

    sprintf(buffer, " drive distance %.2f", scanData->driveDist);
    uart_sendStr(buffer);
    lcd_printf("adc width: %.2f\r\n ping width: %/2f\r\n", adcWidth, pingWidth);
    if(adcTickAmnt > 0){
        return;
    }
    else{
        scanData->averageAdc = 100;
                scanData->averagePing = 100;
                return;
    }

}
