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
    double driveDistHorizontal;
    double driveDistVertical;

//    double driveDistHorizontal;
//    double driveDistVertical;

} scan_info;



int directionGlobal = 0; //0,1,2,3 for NWSE: starts at 0 for positive X /*should be 0 <----------- */
double horizontalPos = 0;
double verticalPos = 0;
#define POSITIVE_X 0
#define POSITIVE_Y 90
#define NEGATIVE_X 180
#define NEGATIVE_Y 270


int turnStatus = 0;
char buffer[200];

void rotate_degrees(int angle_deg, int turnChange_deg, oi_t *sensor_data){
    char buffer[200];
    int startAngle = directionGlobal;

    // Perform Turn
    if (turnChange_deg > 0) {
        turn_counterclockwise(sensor_data, turnChange_deg);
    } else {
        turn_clockwise(sensor_data, -turnChange_deg); // Pass positive value
    }

    // Update Global Direction (0-359)
    directionGlobal += turnChange_deg;

    // Normalize to 0-359
    while (directionGlobal >= 360) directionGlobal -= 360;
    while (directionGlobal < 0) directionGlobal += 360;
    sprintf(buffer, "TURN %d %d\r\n", startAngle, directionGlobal);
    uart_sendStr(buffer);
}

void face_direction(int startDir, int finalDir /*0,1,2,3*/, oi_t *sensor_data){
    int direction = 90*(startDir - finalDir);
    if(direction == 0) {
        return;
    }


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


}

void update_distance(double distance, int direction ){
    char buffer[200];
//    if(direction == 0){
//        horizontalPos+=distance;
//    }
//    else if(direction == 1){
//        verticalPos+=distance;
//    }
//    else if(direction == 2){
//            horizontalPos-=distance;
//        }
//        else if(direction == 3){
//            verticalPos-=distance;
//        }
    horizontalPos += distance * cos(direction * 3.14 / 180);
    verticalPos += distance * sin(direction * 3.14 / 180);
    sprintf(buffer, "MOVE %.0f %.0f %d %.0f\r\n", horizontalPos, verticalPos, direction, distance);
                uart_sendStr(buffer);
}


void scan_cone(int low, int high,  move_scan_t *moveScanData, scan_info *scanData){
    #define cybotLength 40
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
    int pillarWidth = 0;
    sprintf(buffer, "SCAN %.0f %.0f %d\r\n", horizontalPos, verticalPos, directionGlobal);

                uart_sendStr(buffer);
    for(i =low; i<high; i+=2){
        servo_move(i);
        adcVal = adc_read();

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

    //DECIDE DIST FOR BUMP AND CLIFF
    if (moveScanData->status == CLIFF) {
        scanData->driveDistHorizontal = 25;
        scanData->driveDistVertical = 25;
    } else if (moveScanData->status == BUMPLEFT) {
        scanData->driveDistHorizontal = cybotLength;
        scanData->driveDistVertical = 25 + cybotLength;
    }
    else if (moveScanData->status == BUMPRIGHT) {
            scanData->driveDistHorizontal = 20;
            scanData->driveDistVertical = cybotLength + 25;
        }


    lcd_printf("Hori: %.2f Vert %.2f", scanData->driveDistHorizontal, scanData->driveDistVertical);
    // no objects found
    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!\r\n");
               uart_sendStr(buffer);

//        scanData->driveDist = 50 + scanData->adcWidth; //need to find a fix for this logic, current just a hard patched 15 cm + the adcWidth (12/1)

        scanData->averageAdc = 200;
        scanData->averagePing = 200;

        sprintf(buffer, "no object found:  \r\naverage width: %.2f, \r\ndrive dist hori %.2f, \r\nddv %.2f \r\nobject count: %d \r\n", scanData->adcWidth, scanData->driveDistHorizontal, scanData->driveDistVertical, adcTickAmnt);
            uart_sendStr(buffer);
        return;
    }
    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;




    scanData->averageAdc = averageADC;
    scanData->averagePing = averagePing;

    adcRad = (adcTickAmnt * 2) * (3.14 / 180.0);
    adcWidth = 2 * averageADC * tan(adcRad / 2.0);

    pingRad = (pingTickAmnt * 2) * (3.14 / 180.0);
    pingWidth = 2 *averagePing * tan(pingRad / 2.0);

    scanData->adcWidth = adcWidth;
    scanData->pingWidth = pingWidth;

    if (adcTickAmnt > 0)
    {

        if (scanData->averageAdc < 10)
        {
            if (scanData->adcWidth > 7)   // check the values for bounds
            {

                pillarWidth = 24;
            }
            else
            {


                pillarWidth = 17;

            }

        }
        else
        {
            if (scanData->adcWidth > 11)   // check the values for bounds
            {

                pillarWidth = 24;

            }
            else if (  scanData->adcWidth > 6)
            {

                pillarWidth = 17;

            }
            else if (scanData->adcWidth > 0)
            {

                pillarWidth = 11;
            }
        }



    }


    //handle driveDistHorizontal
    if(scanData->averageAngle >= 90){
        double tempAng = scanData->averageAngle - 180;
        sprintf(buffer, "averageADC %.2f  cos (tempAng)%.2f tempAng %.2f pillar width %d\r\n", (scanData->averageAdc), cos(tempAng * M_PI / 180.0), tempAng, pillarWidth);
        uart_sendStr(buffer);
        scanData->driveDistHorizontal = (scanData->averageAdc) * cos(tempAng * M_PI / 180.0) + pillarWidth * cos(tempAng * M_PI / 180.0) + (cybotLength / 1.5);
    }
    else{
        double tempAng = scanData->averageAngle - 135;
        scanData->driveDistHorizontal = pillarWidth * cos(tempAng * M_PI / 180.0) + (cybotLength / 4);
    }

    scanData->driveDistVertical = (scanData->averageAdc)*sin(scanData->averageAngle * M_PI / 180.0) + pillarWidth + (cybotLength / 1.5);


    sprintf(buffer, "Final output:  average: %.2f, drive dist Hori %.2f, drive dist Vert %.2f,  pillar Width: %d \r\n", averageADC, scanData->driveDistHorizontal, scanData->driveDistVertical, pillarWidth);
    uart_sendStr(buffer);




}

int avoidObject(oi_t *sensor_data, move_scan_t *moveScanData, scan_info *scanData)
{
    int result;
    //update_distance(driveDist, direction);
    rotate_degrees(directionGlobal, 90, sensor_data);
    scan_cone(40, 140, moveScanData, scanData);

    while(moveScanData->status != BOUNDARY){

        sprintf(buffer, "adcValue in recursion: %.2f\r\n", scanData->averageAdc);

        uart_sendStr(buffer);

        if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
        {
            result = avoidObject(sensor_data, moveScanData, scanData);
            if (result) return 1;
        }


        move_forward(sensor_data, moveScanData, scanData->driveDistHorizontal); // - scanData->averageAdc); //sideways
        update_distance(moveScanData->distanceTraveled, directionGlobal);
        if(moveScanData->status == BOUNDARY){
//            re_center_tape(sensor_data, &moveScanData);
            continue;
        }

        else if(moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT){
                sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                uart_sendStr(buffer);
            move_backward(sensor_data, 6);
            update_distance(-6, directionGlobal);

                result = avoidObject(sensor_data, moveScanData, scanData);
                if (result) return 1;

            }

    //    update_distance(scanData->driveDist / 2, directionGlobal);

        rotate_degrees(directionGlobal, -90, sensor_data);


        scan_cone(40, 140, moveScanData, scanData);
        if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist)
        {
            result = avoidObject(sensor_data, moveScanData, scanData);
            if (result) return 1;
        }


        move_scan(sensor_data, moveScanData, scanData->driveDistVertical, 40, 140); //straight
        update_distance(moveScanData->distanceTraveled, directionGlobal);
        if(moveScanData->status == BOUNDARY){
//            re_center_tape(sensor_data, &moveScanData);
            continue;
        }

        else if(moveScanData->status == CLIFF ||moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT){
                move_backward(sensor_data, 6);
                update_distance(-6, directionGlobal);
                sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                                uart_sendStr(buffer);

                result = avoidObject(sensor_data, moveScanData, scanData);
                if (result) return 1;

            }

    //    update_distance(scanData->driveDist, directionGlobal);

        rotate_degrees(directionGlobal, -90, sensor_data);

        scan_cone(40, 140, moveScanData, scanData);
        if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2)
        {
            result = avoidObject(sensor_data, moveScanData, scanData);
            if (result) return 1;
        }


        move_scan(sensor_data, moveScanData, scanData->driveDistHorizontal, 40, 140); //straight
        update_distance(moveScanData->distanceTraveled, directionGlobal);
        if(moveScanData->status == BOUNDARY){
//            re_center_tape(sensor_data, &moveScanData);
            continue;
        }
        else if(moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT){
                move_backward(sensor_data, 6);
                update_distance(-6, directionGlobal);
                sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                                uart_sendStr(buffer);

                result = avoidObject(sensor_data, moveScanData, scanData);
                if (result) return 1;

            }


        break;
    }
//when you finish recursion look for tape

    if (!turnStatus)
    {
        face_direction(directionGlobal, NEGATIVE_Y, sensor_data);
    }
    else if (turnStatus == 1)
    {

        face_direction(directionGlobal, POSITIVE_X, sensor_data);
    }
    else if (turnStatus == 2)
    {

        face_direction(directionGlobal, POSITIVE_Y, sensor_data);
    }
    else if (turnStatus == 3)
    {
        face_direction(directionGlobal, NEGATIVE_X, sensor_data);
    }

    //moving to white tape at end of seq
    while (moveScanData->status != BOUNDARY)
    {
        scan_cone(40, 140, moveScanData, scanData);
        if (scanData->averageAdc < 25)
        {
            result = avoidObject(sensor_data, moveScanData, scanData);
            if (result) return 1;
        }
        else
        {
           move_scan(sensor_data, moveScanData, 25, 40, 140);
           update_distance(moveScanData->distanceTraveled, directionGlobal);
        }
    }
//    re_center_tape(sensor_data, &moveScanData);

    //get back into direction to continue until next EDGE
    if (!turnStatus)
    {
        face_direction(directionGlobal, POSITIVE_X, sensor_data);
    }
    else if (turnStatus == 1)
    {
        face_direction(directionGlobal, POSITIVE_Y, sensor_data);

    }
    else if (turnStatus == 2)
    {
        face_direction(directionGlobal, NEGATIVE_X, sensor_data);

    }
    else if (turnStatus == 3)
    {
        face_direction(directionGlobal, NEGATIVE_Y, sensor_data);

    }

    return 1;
/*END avoid_object()*/
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

    int lastDirection;

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    scan_info scanData;
    scanData.averageAdc = 0;
    scanData.averagePing = 0;
    scanData.pingWidth = 0;

    scanData.adcWidth =0;
    scanData.driveDist = 0;
    scanData.driveDistHorizontal = 0;
    scanData.driveDistVertical = 0;


    move_scan_t moveScanData;
    moveScanData.distanceTraveled = 0;
    moveScanData.status = CLEAR;

    int stop = 0;
    while (1)
    {

        char c = uart_receive();


        if (c == 'w')
        {
            move_forward(sensor_data, &moveScanData, 25);
            update_distance(moveScanData.distanceTraveled, directionGlobal);
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPLEFT)
            {

                sprintf(buffer, "BUMPLEFT %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPRIGHT)
            {

                sprintf(buffer, "BUMPRIGHT %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPBOTH)
            {

                sprintf(buffer, "BUMPBOTH %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }

        }
        if (c == 'e')
        {
            move_scan(sensor_data, &moveScanData, 25, 45, 135);
            update_distance(moveScanData.distanceTraveled, directionGlobal);
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPLEFT)
            {

                sprintf(buffer, "BUMPLEFT %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPRIGHT)
            {

                sprintf(buffer, "BUMPRIGHT %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }
            else if (moveScanData.status == BUMPBOTH)
            {

                sprintf(buffer, "BUMPBOTH %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                moveScanData.status = CLEAR;
                move_backward(sensor_data, 7);
                update_distance(-7, directionGlobal);
            }

        }
        else if (c == 'a')
        {
            lastDirection = directionGlobal;
            rotate_degrees(directionGlobal, 90, sensor_data);
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }
        else if (c == 'd')
        {
            lastDirection = directionGlobal;
            rotate_degrees(directionGlobal, -90, sensor_data);
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }
        else if (c == 'i')
        {
            rotate_degrees(directionGlobal, 30, sensor_data);  // Left 30
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }
        else if (c == 'o')
        {
            rotate_degrees(directionGlobal, -30, sensor_data); // Right 30
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }
        else if (c == 'j')
        {
            rotate_degrees(directionGlobal, 60, sensor_data);  // Left 60
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }
        else if (c == 'k')
        {
            rotate_degrees(directionGlobal, -60, sensor_data); // Right 60
            if (moveScanData.status == BOUNDARY)
            {

                sprintf(buffer, "BOUNDARY %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
            else if (moveScanData.status == CLIFF)
            {

                sprintf(buffer, "CLIFF %.0f %.0f %d\r\n", horizontalPos,
                        verticalPos, directionGlobal);
                uart_sendStr(buffer);
                move_backward(sensor_data, 10);
                update_distance(-10, directionGlobal);
                moveScanData.status = CLEAR;
            }
        }

        else if (c == 's')
        {
            move_backward(sensor_data, 50);
            update_distance(-50, directionGlobal);
        }
        else if (c == 'h')
        {
            scan_cone(0, 180, &moveScanData, &scanData);
        }
        else if (c == '1' || button_getButton() == 1)
        {
            unsigned char rickrollNumNotes = 11;
            unsigned char rickrollNotes[11]    = {53, 55, 48, 55, 57, 60, 58, 57, 53, 55, 48};
            unsigned char rickrollDuration[11] = {48, 64, 16, 48, 48, 8,  8,  8,  48, 64, 64};
            int RICK_ROLL = 0;
            oi_loadSong(RICK_ROLL, rickrollNumNotes, rickrollNotes, rickrollDuration);
            oi_play_song(RICK_ROLL);
        }

        else if (c == 't')
        {
            //used to check white sensors
            while(1){

                oi_update(sensor_data);
                timer_waitMillis(10);
                sprintf(buffer, "left %d\n FrontLeft %d\n FrontRight %d\n right %d", sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
                lcd_printf(buffer);
            }
        }
        else if (c == 'c')
        {
            servo_calibrate();
        }
        else if (c == 'z') // calibrate wheels
               {
       uart_sendStr("received z, calibrating...\r\n");
       calibrate_turn(sensor_data);
       //            sprintf(buffer, "correction: %.2f\r\n", correction);
       //            uart_sendStr(buffer);
               }

        else if (c == 'm')
        {

            while (!stop)
            {
                oi_update(sensor_data);

                // scan before movement
                scan_cone(40, 140, &moveScanData, &scanData);
//                 estimation = 0.0000228813 * (IR_val * IR_val)- 0.0981288 * IR_val + 115.33455;

                int driveDist = fmin(200, scanData.averageAdc);
                move_scan(sensor_data, &moveScanData, driveDist, 40, 140);
                int status = moveScanData.status;
                double distanceChange = moveScanData.distanceTraveled;
                update_distance(distanceChange, directionGlobal);
                if(OBJECT == status){

                    scan_cone(40, 140, &moveScanData, &scanData);
                }
//                sprintf(buffer, "scan data ADC %.2f, PING  %d \r\n", scanData.averageAdc, scanData.averagePing);
//                uart_sendStr(buffer);
                if(scanData.averageAdc < 20){
                    avoidObject(sensor_data , &moveScanData, &scanData);
                    sprintf(buffer, "Broke recursion %d <---------------------------\r\n", directionGlobal);
                    uart_sendStr(buffer);

                }
                else if (status == BOUNDARY)
                {
                    re_center_tape(sensor_data, &moveScanData);
                    if (!turnStatus) // first row
                    {

                        turnStatus = 1;
                        rotate_degrees(directionGlobal, 90, sensor_data);

                        sprintf(buffer, "EDGE HORIZONTAL %.0f\r\n", horizontalPos);
                        uart_sendStr(buffer);
                    }
                    else if (turnStatus == 1) // second row
                    {
                        turnStatus = 2; // set to third row
                        rotate_degrees(directionGlobal, 90, sensor_data);

                        sprintf(buffer, "EDGE VERTICAL %.0f\r\n", verticalPos);
                        uart_sendStr(buffer);

                    }
                    else if (turnStatus == 2) {
                        turnStatus = 3;
                        rotate_degrees(directionGlobal, 90, sensor_data);

                        sprintf(buffer, "FOUND THIRD SIDE %.0f\r\n", verticalPos);
                        uart_sendStr(buffer);
                    }
                    else if (turnStatus == 3) {
                        stop = 1;
                        rotate_degrees(directionGlobal, 90, sensor_data);

                        lcd_printf("finished perimeter!\r\n");
                        sprintf(buffer, "finished perimeter!\r\n");
                        uart_sendStr(buffer);
                        oi_free(sensor_data);
                        while (1);
                    }
                }
                else if(status == CLIFF || status == BUMPLEFT || status == BUMPRIGHT){

                    move_backward(sensor_data, 6);
                    update_distance(-6, directionGlobal);
                    avoidObject(sensor_data, &moveScanData, &scanData);
                }

            }
        }
    }
    oi_free(sensor_data);

    return 0;
}
