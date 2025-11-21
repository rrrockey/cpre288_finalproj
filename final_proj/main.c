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
void rotate_degrees(int angle, int direction, oi_t *sensor_data);
void update_distance(double distance, int direction);
void face_direction(int startDir, int finalDir /*0,1,2,3*/, oi_t *sensor_data);
double horizontalPos = 0;
double verticalPos = 0;
#define POSITIVE_X 0
#define POSITIVE_Y 1
#define NEGATIVE_X 2
#define NEGATIVE_Y 3
int directionGlobal = 0; //0,1,2,3 for NWSE

int turnStatus = 0;
void face_direction(int startDir, int finalDir /*0,1,2,3*/, oi_t *sensor_data){
    int direction = 90*(startDir - finalDir);
    if(direction <0){
        direction+= 360;
    }
    else if(direction > 360){
        direction -= 360;
    }
    if(direction > 180){

        rotate_degrees(startDir, -1 * 90, sensor_data);
    }
    else if(direction <180){
        rotate_degrees(startDir, 1 * 90, sensor_data);
    }
    else if(direction == 180){
        rotate_degrees(startDir, 2 * 90, sensor_data);
    }
}
void rotate_degrees(int angle, int direction, oi_t *sensor_data){ //CCW is positive
    int angleChange = direction/90;
    if(angleChange>0){
        angle+= angleChange;         //maybe problem?
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
    directionGlobal = angle;
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

//        while (1) {
//            scan_cone(60, 120, &scanData);
//            sprintf(buffer, "Width: %.2f Distance: %.2f \n\r", scanData.adcWidth, scanData.averageAdc);
//            uart_sendStr(buffer);
//        }
        while(1) {
            button = button_getButton();
            lcd_printf("%d",directionGlobal);
            if(button == 1){
                face_direction(directionGlobal, NEGATIVE_X, sensor_data);

            }
            else if(button == 2){
                face_direction(directionGlobal, POSITIVE_X, sensor_data);
            }
            else if(button == 3){
                face_direction(directionGlobal, NEGATIVE_Y, sensor_data);
            }
            else if(button == 4){
                face_direction(directionGlobal, POSITIVE_Y, sensor_data);
            }
            else{
                continue;
            }
        }
//        int pingVal = (((ping_read()/2)*.5)*34000)/16000000;
//        int IR_val = adc_read();
        scan_cone(45,135, &scanData);
//         estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
        lcd_printf("%d, %.2f, %d, %d", scanData.averagePing, scanData.averageAdc, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        int driveDist = fmin(200, scanData.averagePing);
        update_distance(driveDist, directionGlobal);

        int status = move_scan(sensor_data, driveDist, 60, 120);
        if(OBJECT == status){

            scan_cone(45,135, &scanData);
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
                rotate_degrees(directionGlobal, -90, sensor_data);

                sensor_data->distance = 0;

            }
            else
            {
                rotate_degrees(directionGlobal, 180, sensor_data);


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

        }}
    oi_free(sensor_data);

    return 0;
}

void avoidObject(oi_t *sensor_data, scan_info *scanData)
{

    char buffer[50];
    //update_distance(driveDist, direction);
    rotate_degrees(directionGlobal, 90, sensor_data);


    scan_cone(45, 135, scanData);


    sprintf(buffer, "adcValue in recursion: %.2f", scanData->averageAdc);
    uart_sendStr(buffer);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }
  //  move_forward(sensor_data, scanData->driveDist);
    move_forward(sensor_data, scanData->driveDist - scanData->averageAdc); //sideways
    update_distance(scanData->driveDist, directionGlobal);
    rotate_degrees(directionGlobal, -90, sensor_data);


    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist)
    {
        avoidObject(sensor_data, scanData);
    }
    move_forward(sensor_data, scanData->driveDist ); //straight
    update_distance(scanData->driveDist, directionGlobal);
    rotate_degrees(directionGlobal, -90, sensor_data);

    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }




    if (!turnStatus)
    {

        face_direction(directionGlobal, NEGATIVE_X, sensor_data);
    }
    else
    {

        face_direction(directionGlobal, NEGATIVE_Y, sensor_data);
    }
    while (1)
    {
        scan_cone(45, 135, sensor_data);
        if (scanData->averageAdc < 25)
        {
            avoidObject(sensor_data, scanData);
        }
        else
        {
            if (move_forward(sensor_data, 25) == BOUNDARY)
            {
                break;
            }
        }

    }
    //move_forward(sensor_data, scanData->driveDist); //move TO THE WHITE TAPE
    //update_distance(scanData->driveDist, direction);





//    }









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
 //                   uart_sendStr(buffer);
            adcTickAmnt++;
            averageADC += estimation;

        }

        pingVal = (((ping_read()/2)*.5)*34000)/16000000;
        if(pingVal < 25){
            pingTickAmnt++;
            averagePing+= pingVal;
        }

        sprintf(buffer, "angle: %d scan: %.2f, avercage: %.2f, object count: %d \r\n",i,  estimation, averageADC / ((double)adcTickAmnt), adcTickAmnt);
 //       uart_sendStr(buffer);



    }

    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!");
//                uart_sendStr(buffer);
        scanData->averageAdc = 200;
        scanData->averagePing = 200;
        scanData->driveDist = 25+35;
        return;
    }
    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;
    sprintf(buffer, "Final output:  average: %.2f, object count: %d \r\n", averageADC, adcTickAmnt);
//            uart_sendStr(buffer);
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
        if (scanData->averageAdc < 10)
        {
            if (scanData->adcWidth > 7)   // check the values for bounds
            {
                scanData->driveDist = 17 + 35 + scanData->averageAdc;

            }
            else
            {
                scanData->driveDist = 35 + 11 + scanData->averageAdc;
            }

        }
        else
        {
            if (scanData->adcWidth > 11)   // check the values for bounds
            {
                scanData->driveDist = 17 + 35 + scanData->averageAdc;
            }
            else if (  scanData->adcWidth > 6)
            {
                scanData->driveDist = 35 + 11 + scanData->averageAdc;
            }
            else if (scanData->adcWidth > 0)
            {
                scanData->driveDist = 35 + 5 + scanData->averageAdc;
            }
        }



    }
    else
    {
        scanData->driveDist = 200;
    }

    sprintf(buffer, " drive distance %.2f", scanData->driveDist);
//    uart_sendStr(buffer);
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
