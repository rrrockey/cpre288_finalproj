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
    double averageAngle;
<<<<<<< HEAD
=======
//    double driveDistHorizontal;
//    double driveDistVertical;




>>>>>>> main
} scan_info;



int directionGlobal = 0; //0,1,2,3 for NWSE: starts at 0 for positive X
double horizontalPos = 0;
double verticalPos = 0;
#define POSITIVE_X 0
#define POSITIVE_Y 1
#define NEGATIVE_X 2
#define NEGATIVE_Y 3


int turnStatus = 0;
char buffer[200];

void rotate_degrees(int angle/*global direction*/, int turnChange/*change in angle*/, oi_t *sensor_data){ //CCW is positive
    char buffer[200];
    int angleChange = turnChange/90;
    int startAngle = directionGlobal;
    if(angleChange>0){
        angle+= angleChange;         //maybe problem?
        angle %=4;
        turn_counterclockwise(sensor_data, angleChange*90);
    }
    else{
        angle--;
        if(angle<0){
            angle +=4;
            angleChange *= -1;
            turn_clockwise(sensor_data, angleChange*90);
        } else {
            turn_clockwise(sensor_data, 90);
        }


    }
    sprintf(buffer, "TURN %d %d\r\n", startAngle, angle);
                uart_sendStr(buffer);
    directionGlobal = angle;
}

void face_direction(int startDir, int finalDir /*0,1,2,3*/, oi_t *sensor_data){
    int direction = 90*(startDir - finalDir);
    if(direction == 0) {
        return;
    }

//    if(direction <0){
//        direction+= 360;
//    }
//    else if(direction > 360){
//        direction -= 360;
//    }
    if(direction == -270){
        rotate_degrees(startDir, -1 * 90, sensor_data);
    }
    else if(direction == 270){
        rotate_degrees(startDir, 1 * 90, sensor_data);
    }
    else if(direction == 90){
        rotate_degrees(startDir, -1 * 90, sensor_data);
    }
    else if(direction == -90){
        rotate_degrees(startDir, 1 * 90, sensor_data);
    }
    else if(direction == 180 || direction == -180){
        rotate_degrees(startDir, 2 * 90, sensor_data);
    }

    lcd_printf("direction: %d", directionGlobal);
}

void update_distance(double distance, int direction ){
    char buffer[200];
    if(direction == 0){
        horizontalPos+=distance;
    }
    else if(direction == 1){
        verticalPos+=distance;
    }
    else if(direction == 2){
            horizontalPos-=distance;
        }
        else if(direction == 3){
            verticalPos-=distance;
        }
    sprintf(buffer, "MOVE %.0f %.0f %d %.0f\r\n", horizontalPos, verticalPos, direction, distance);
                uart_sendStr(buffer);
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
    double averageAngle = 0;
    int pingVal = 0;
    int adcVal = 0;
    int i =low;
    sprintf(buffer, "SCAN %.0f %.0f %d\r\n", horizontalPos, verticalPos, directionGlobal);

                uart_sendStr(buffer);
    for(i =low; i<high; i+=2){
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
            averageAngle+=i;

        }

        sprintf(buffer, "DATA %d, %.0f, %d\r\n", i, estimation, pingVal);
        uart_sendStr(buffer);



    }
    averageAngle /= pingTickAmnt;
    scanData->averageAngle = averageAngle;
    sprintf(buffer, "ENDSCAN, average angle: %.2f \r\n", averageAngle);
                   uart_sendStr(buffer);
    //uart_sendStr("ENDSCAN");

    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!\r\n");
               uart_sendStr(buffer);
//        scanData->driveDist = 50 + scanData->adcWidth; //need to find a fix for this logic, current just a hard patched 15 cm + the adcWidth (12/1)
        scanData->averageAdc = 200;
        scanData->averagePing = 200;

        sprintf(buffer, "no object found:  \r\naverage width: %.2f, \r\ndrive dist %.2f, \r\nobject count: %d \r\n", scanData->adcWidth, scanData->driveDist, adcTickAmnt);
            uart_sendStr(buffer);
        return;
    }
    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;

    sprintf(buffer, "Final output:  average: %.2f, drive dist %.2f, object count: %d \r\n", averageADC, scanData->driveDist, adcTickAmnt);
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

void avoidObject(oi_t *sensor_data, scan_info *scanData)
{
    int check = -1;

    //update_distance(driveDist, direction);
    rotate_degrees(directionGlobal, 90, sensor_data);
    scan_cone(45, 135, scanData);

while(check != BOUNDARY){

    sprintf(buffer, "adcValue in recursion: %.2f", scanData->averageAdc);

    uart_sendStr(buffer);

    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }
    scanData->driveDist = (scanData->driveDist) * (.85 + ((scanData->averageAngle - 45) *.3) / 90);
    lcd_printf("%.2f, %.2f", scanData->driveDist, scanData->averageAngle);
    check = move_forward(sensor_data, scanData->driveDist *.75); // - scanData->averageAdc); //sideways
    if(check == BOUNDARY){continue;}
    else if(check == CLIFF || check == BUMP){
            check = -1;
            move_backward(sensor_data, 10);
            scanData->driveDist = 50;
            avoidObject(sensor_data, scanData);
            sprintf(buffer, "cliff sensor hit");
            uart_sendStr(buffer);
        }

    update_distance(scanData->driveDist / 2, directionGlobal);
    rotate_degrees(directionGlobal, -90, sensor_data);


    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist)
    {
        avoidObject(sensor_data, scanData);
    }
    scanData->driveDist = (scanData->driveDist) * (.85 + ((scanData->averageAngle - 45) *.3) / 90);
    lcd_printf("%.2f, %.2f", scanData->driveDist, scanData->averageAngle);
    check = move_forward(sensor_data, scanData->driveDist ); //straight
    if(check == BOUNDARY){continue;}
    else if(check == CLIFF || check == BUMP){
            check = -1;
            move_backward(sensor_data, 10);
            scanData->driveDist = 50;
            avoidObject(sensor_data, scanData);
            sprintf(buffer, "cliff sensor hit");
            uart_sendStr(buffer);
        }

    update_distance(scanData->driveDist, directionGlobal);
    rotate_degrees(directionGlobal, -90, sensor_data);

    scan_cone(45, 135, scanData);
    if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
    {
        avoidObject(sensor_data, scanData);
    }
    scanData->driveDist = (scanData->driveDist) * (.85 + ((scanData->averageAngle - 45) *.3) / 90);
    lcd_printf("%.2f, %.2f", scanData->driveDist, scanData->averageAngle);
    check = move_forward(sensor_data, scanData->driveDist); //straight
    if(check == BOUNDARY){continue;}
    else if(check == CLIFF || check == BUMP){
            check = -1;
            move_backward(sensor_data, 10);
            scanData->driveDist = 50;
            avoidObject(sensor_data, scanData);
            sprintf(buffer, "cliff sensor hit");
            uart_sendStr(buffer);
        }


    break;
}
//when you finish recursion

    if (!turnStatus)
    {
        face_direction(directionGlobal, NEGATIVE_Y, sensor_data);
    }
    else
    {

        face_direction(directionGlobal, POSITIVE_X, sensor_data);
    }

    while (check != BOUNDARY)
    {
        scan_cone(45, 135, scanData);
        if (scanData->averageAdc < 25)
        {
            avoidObject(sensor_data, scanData);
        }
        else
        {
            check = move_forward(sensor_data, 25);
        }

    }

    if (!turnStatus)
       {
            face_direction(directionGlobal, POSITIVE_X, sensor_data);
       }
       else
       {
            face_direction(directionGlobal, POSITIVE_Y, sensor_data);

       }
    //move_forward(sensor_data, scanData->driveDist); //move TO THE WHITE TAPE
    //update_distance(scanData->driveDist, direction);





//    }









    //drive forward

}

void mowing_sequence(oi_t *sensor_data, move_scan_t *moveScanData) {
#define RIGHT 0
#define LEFT 1
#define cyBot_length 35
//    char buffer[200];
    scan_info scanData;
    int turn_dir = RIGHT; // the next direction the bot has to turn after encountering tape
    int final_row = 0; // bool to keep track if cyBot is on the last row of mowing

    while (1) {
        scan_cone(45,135, &scanData);
        lcd_printf("%d, %.2f, %d, %d", scanData.averagePing, scanData.averageAdc, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        int driveDist = 50;

//        oi_update(sensor_data);
//        double distance_before = sensor_data->distance;

        move_scan(sensor_data, moveScanData, driveDist, 45, 135);
        int status = moveScanData->status;

//        oi_update(sensor_data);
//        double distance_moved = sensor_data->distance - distance_before;
        double distance_moved = moveScanData->distanceTraveled;
        update_distance(distance_moved, directionGlobal);




        if(status == OBJECT){

            scan_cone(45,135, &scanData);
        }

        if (status == BOUNDARY && !final_row) // if white tape found, start new row
        {
            if (turn_dir == RIGHT) {
                lcd_printf("found bound");
//                turn_clockwise(sensor_data, 90);

                int lastDirection = directionGlobal;
                rotate_degrees(directionGlobal, -90, sensor_data); // clockwise
                sprintf(buffer, "TURN %d %d\r\n", lastDirection, directionGlobal);
                uart_sendStr(buffer);

                move_scan(sensor_data, moveScanData, cyBot_length, 45, 135);
                int status = moveScanData->status;

                double distance_moved = moveScanData->distanceTraveled;
                update_distance(distance_moved, directionGlobal);




                if (status == BOUNDARY) {
                    final_row = 1;
                }
                lastDirection = directionGlobal;
                rotate_degrees(directionGlobal, -90, sensor_data); // clockwise
                sprintf(buffer, "TURN %d %d\r\n", lastDirection, directionGlobal);
                uart_sendStr(buffer);
                turn_dir = LEFT;
            }
            else { // turn_dir == LEFT
                lcd_printf("found bound");
                int lastDirection = directionGlobal;
                rotate_degrees(directionGlobal, 90, sensor_data); // counterclockwise
                sprintf(buffer, "TURN %d %d\r\n", lastDirection, directionGlobal);
                uart_sendStr(buffer);


                move_scan(sensor_data, moveScanData, cyBot_length, 45, 135);
                int status = moveScanData->status;

                double distance_moved = moveScanData->distanceTraveled;
                update_distance(distance_moved, directionGlobal);


                if (status == BOUNDARY) {
                    final_row = 1;
                }
                lastDirection = directionGlobal;
                rotate_degrees(directionGlobal, 90, sensor_data); // counterclockwise
                sprintf(buffer, "TURN %d %d\r\n", lastDirection, directionGlobal);
                uart_sendStr(buffer);
                turn_dir = RIGHT;
            }
        }
        else if (status == BOUNDARY && final_row) // final tape is found, mowing complete
        {
            break;
        }
    }
    lcd_printf("mowing finished");
    while(1);
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

    scan_info scanData;
    scanData.averageAdc = 0;
    scanData.averagePing = 0;
    scanData.pingWidth = 0;
    scanData.adcWidth =0;

    move_scan_t moveScanData;
    moveScanData.distanceTraveled = 0;
    moveScanData.status = CLEAR;

    int stop = 0;
    int lastDirection;
    double horizontalSpan = 0;
    double verticalSpan = 0;
    double estimation = 0;
    while (1)
    {

        char c = uart_receive();

        int moveStatus = 0;
        if (c == 'w')
        {
//            move_forward(sensor_data, 50);
//            update_distance(50, directionGlobal);


            /*DELTE BELOW*/

            move_scan(sensor_data, &moveScanData, 50, 45, 135);
            int status = moveScanData.status;

            double distance_moved = moveScanData.distanceTraveled;
//            update_distance(distance_moved, directionGlobal);

            char buffer[100];
            sprintf(buffer, "distance traveled: %.2f\r\n", distance_moved);
            uart_sendStr(buffer);
            sprintf(buffer, "object encountered: %d\r\n", status);
            uart_sendStr(buffer);

            /*DELETE ABOVE*/








        }
        else if (c == 'a')
        {
            lastDirection = directionGlobal;
            rotate_degrees(directionGlobal, 90, sensor_data);
        }
        else if (c == 'd')
        {
            lastDirection = directionGlobal;
            rotate_degrees(directionGlobal, -90, sensor_data);
        }
        else if (c == 's')
        {
            move_backward(sensor_data, 50);
            update_distance(-50, directionGlobal);
        }
        else if (c == 'h')
        {
            scan_cone(0, 180, &scanData);
        }
        else if (c == 'm')
        {

            /*BEGIN TESTING MOWING SEQUENCE*/
            //mowing_sequence(sensor_data);
            /*END TESTING MOWING SEQUENCE*/


    while (!stop)
    {
        oi_update(sensor_data);

//        while (1) {
//            scan_cone(60, 120, &scanData);
//            sprintf(buffer, "Width: %.2f Distance: %.2f \n\r", scanData.adcWidth, scanData.averageAdc);
//            uart_sendStr(buffer);
//        }
//        while(1) {
//            button = button_getButton();
//            lcd_printf("%d",directionGlobal);
//            if(button == 1){
//                face_direction(directionGlobal, NEGATIVE_X, sensor_data);
//
//            }
//            else if(button == 2){
//                face_direction(directionGlobal, POSITIVE_X, sensor_data);
//            }
//            else if(button == 3){
//                face_direction(directionGlobal, NEGATIVE_Y, sensor_data);
//            }
//            else if(button == 4){
//                face_direction(directionGlobal, POSITIVE_Y, sensor_data);
//            }
//            else{
//                continue;
//            }
//        }
//        int pingVal = (((ping_read()/2)*.5)*34000)/16000000;
//        int IR_val = adc_read();

        scan_cone(45,135, &scanData);
//         estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
        lcd_printf("%d, %.2f, %d, %d", scanData.averagePing, scanData.averageAdc, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
        servo_move(90);
        double startDistance = 0;
        double distanceChange;

        int driveDist = fmin(200, scanData.averagePing);
        move_scan(sensor_data, &moveScanData, driveDist, 45, 135);
        int status = moveScanData.status;
        distanceChange = moveScanData.distanceTraveled;
        update_distance(distanceChange, directionGlobal);
        if(OBJECT == status){

            scan_cone(45,135, &scanData);
        }
      //  sprintf(buffer, "scan data ADC %.2f, PING  %d \r\n", scanData.averageAdc, scanData.averagePing);
                         //   uart_sendStr(buffer);
        if(scanData.averagePing < 10){
            avoidObject(sensor_data , &scanData);
            sprintf(buffer, "Broke recursion %d <---------------------------\r\n", directionGlobal);
            uart_sendStr(buffer);

        }
        if (status == BOUNDARY)
        {
            if (!turnStatus)
            {

                turnStatus = 1;
                rotate_degrees(directionGlobal, 90, sensor_data);

                sprintf(buffer, "EDGE HORIZONTAL %.0f\r\n", horizontalPos);

                uart_sendStr(buffer);

                sensor_data->distance = 0;

            }
            else
            {
                rotate_degrees(directionGlobal, 180, sensor_data);

                sprintf(buffer, "EDGE VERTICAL %.0f\r\n", verticalPos);

                uart_sendStr(buffer);

                stop = 1;
            }
        }
        else if(status == CLIFF || status == BUMP){

                    status = -1;
                    move_backward(sensor_data, 15);
                    avoidObject(sensor_data, &scanData);
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
    mowing_sequence(sensor_data, &moveScanData);

        }
    }
    oi_free(sensor_data);

    return 0;
}
