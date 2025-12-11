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
#include "IMU.h"
#include <inc/tm4c123gh6pm.h>
/**
 * main.c
 */


/**
 * Scan info struct holds every thing that has to do with objects and the distance our bot should drive when seeing an object.
 * All of these values are calculated in scan_cone
 */

typedef struct {
    int averagePing;
    double averageAdc;
    double adcWidth;
    double pingWidth;
    double driveDist;
    double averageAngle;
    double driveDistHorizontal;
    double driveDistVertical;
} scan_info;


/**
 * Variables that hold the direction and headVals that we get through the IMU.
 * VerticalPos and HorizontalPos store the updated values of distance based on OI.
 */

int directionGlobal = 0; //0,1,2,3 for NWSE: starts at 0 for positive X /*should be 0 <----------- */
int headVal = 0;
double horizontalPos = 0;
double verticalPos = 0;
#define POSITIVE_X 0
#define POSITIVE_Y 1
#define NEGATIVE_X 2
#define NEGATIVE_Y 3

/**
 * Global values that help us determine which stage of our code we are at
 * a global buffer for debugging
 * global head value to pull imu heading val
 */

int turnStatus = 0;
char buffer[200];
int head;


/**
 * get rick rolled
 */

void rickroll(void) {
    unsigned char rickrollNumNotes = 11;
    unsigned char rickrollNotes[11]    = {53, 55, 48, 55, 57, 60, 58, 57, 53, 55, 48};
    unsigned char rickrollDuration[11] = {48, 64, 16, 48, 48, 8,  8,  8,  48, 64, 64};
    int RICK_ROLL = 0;
    oi_loadSong(RICK_ROLL, rickrollNumNotes, rickrollNotes, rickrollDuration);
    oi_play_song(RICK_ROLL);
}

/**
 * rotate degrees that has direction handling built in. This makes optimal turns and updates global direction so that we keep things consistent
 */


void rotate_degrees(int angle/*global direction*/, int turnChange/*change in angle*/, oi_t *sensor_data, move_scan_t *moveScanData, compassVals *compassVals){ //CCW is positive
    char buffer[200];
    int angleChange = turnChange/90;
    int startAngle = directionGlobal;

    int i =0;

    if(angleChange>0){
        angle+= angleChange;
        angle %=4;

        for( i = 0; i < 1; i++){
            turn_counterclockwise(sensor_data, angleChange * 90);
        }
    }
    else
    {
        angle--;
        if (angle < 0)
        {
            angle += 4;
            angleChange *= -1;
            for ( i = 0; i < 1; i++)
            {
                turn_clockwise(sensor_data, angleChange * 90);
            }
        }
        else
        {
            for ( i = 0; i < 1; i++)
            {
                turn_clockwise(sensor_data, 90);
            }
        }

    }
    sprintf(buffer, "TURN %d %d\r\n", startAngle, angle);
                uart_sendStr(buffer);

    directionGlobal = angle;
    angle_correct(sensor_data, moveScanData, directionGlobal, compassVals);
}

/**
 * face direction uses our rotate degrees function, but based on which direction we are going and which direction we want to go it simplifies
 * the code so we can make moves based on direction instead of angles.
 */

void face_direction(int startDir, int finalDir /*0,1,2,3*/, oi_t *sensor_data, move_scan_t *moveScanData, compassVals *compassVals){

    int direction = 90*(startDir - finalDir);
    if(direction == 0) {
        return;
    }


    if(direction == -270){
        rotate_degrees(startDir, -1 * 90, sensor_data, moveScanData, compassVals);
    }
    else if(direction == 270){
        rotate_degrees(startDir, 1 * 90, sensor_data, moveScanData, compassVals);
    }
    else if(direction == 90){
        rotate_degrees(startDir, -1 * 90, sensor_data, moveScanData, compassVals);
    }
    else if(direction == -90){
        rotate_degrees(startDir, 1 * 90, sensor_data, moveScanData, compassVals);
    }
    else if(direction == 180 || direction == -180){
        rotate_degrees(startDir, 2 * 90, sensor_data, moveScanData, compassVals);
    }



}


/**
 * keeps distance updated properly based on the direction we are facing. Making sure our total distance is handled properly.
 */
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

/**
 * scan_cone is how we decide and calculate if there is or isnt an object.
 * this is partly decided in move scan where we set a status that we check here.
 */
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
    double tempAng = 0;

    /**
     * uart to send to GUI
     */

    sprintf(buffer, "SCAN %.0f %.0f %d\r\n", horizontalPos, verticalPos, directionGlobal);
    uart_sendStr(buffer);


    /**
     * actual scan loop that calculates the amount of ticks wide an object is.
     * We used below 25 cm because we did not want to avoid objects that were further away than that.
     * The final uart print in this loop is also for GUI
     */

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


    /**
     * where we calculate the "mid point" of the object and place it in the scanData struct
     */

    averageAngle /= pingTickAmnt;
    scanData->averageAngle = averageAngle;




    sprintf(buffer, "ENDSCAN, average angle: %.2f \r\n", averageAngle);
    uart_sendStr(buffer);



    /**
     * if status is cliff or bump left or right we want to store a different value for move distances.
     */

    if (moveScanData->status == CLIFF) {
        scanData->driveDistHorizontal = 40;
        scanData->driveDistVertical = 25;
    }
    else if (moveScanData->status == BUMPLEFT) {
        scanData->driveDistHorizontal = cybotLength+15;
        scanData->driveDistVertical = 25 + cybotLength;
    }
    else if (moveScanData->status == BUMPRIGHT) {
            scanData->driveDistHorizontal = cybotLength;
            scanData->driveDistVertical = cybotLength + 25;
        }



    lcd_printf("Hori: %.2f Vert %.2f", scanData->driveDistHorizontal, scanData->driveDistVertical);


    /**
     * if there are no ticks scanned return from scan_cone
     */

    if(adcTickAmnt == 0 || pingTickAmnt == 0){
        sprintf(buffer, "No objects found!\r\n");
               uart_sendStr(buffer);

        //set scanData to a maximum value

        scanData->averageAdc = 200;
        scanData->averagePing = 200;

        return;
    }


    /**
     * set the average ping and ADC based on the tick amount and average adc from above.
     */

    averageADC = averageADC / ((double)(adcTickAmnt));
    averagePing = averagePing/pingTickAmnt;

    scanData->averageAdc = averageADC;
    scanData->averagePing = averagePing;

    /**
     * calculating the radial width based on the adc tick amount and pint tick amount. Both a times 2 because we scan every 2 degrees.
     * places them in the scanData
     */

    adcRad = (adcTickAmnt * 2) * (3.14 / 180.0);
    adcWidth = 2 * averageADC * tan(adcRad / 2.0);

    pingRad = (pingTickAmnt * 2) * (3.14 / 180.0);
    pingWidth = 2 *averagePing * tan(pingRad / 2.0);

    scanData->adcWidth = adcWidth;
    scanData->pingWidth = pingWidth;

    /**
     * because our adcRad and width was not passing objects fully, we made sure to find values that it was reading
     * Then we manually set the pillar width based on those found values.
     */

    if (adcTickAmnt > 0)
    {

        if (scanData->averageAdc < 10)
        {
            if (scanData->adcWidth > 7)   // check the values for bounds
            {

                pillarWidth = 26;
            }
            else
            {


                pillarWidth = 17;

            }

        }
        else
        {
            if (scanData->adcWidth > 9.5)   // check the values for bounds
            {

                pillarWidth = 26;

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




    /**
     * set the horizontal distance and the vertical drive distance based on where the angle is and different guess and check values.
     * The cosine function are based off the trig and angle from where the sensor is and, assuming our avergae angle is correct, the middle of the object.
     */


    if(scanData->averageAngle >= 90){
        tempAng = scanData->averageAngle - 180;

        scanData->driveDistHorizontal = (scanData->averageAdc) * cos(tempAng * M_PI / 180.0) + pillarWidth * cos(tempAng * M_PI / 180.0) + (cybotLength / 2);
    }
    else{
        double tempAng = scanData->averageAngle - 135;
        scanData->driveDistHorizontal = pillarWidth * cos(tempAng * M_PI / 180.0) + (cybotLength / 3);
    }

    scanData->driveDistVertical = (scanData->averageAdc)*sin(scanData->averageAngle * M_PI / 180.0) + pillarWidth + (cybotLength / 2);
}



int avoidObject(oi_t *sensor_data, move_scan_t *moveScanData, scan_info *scanData, compassVals *compassVals)
{
    #define cybotLength 40

    // Stack to simulate recursion - stores the state at each "recursive call"
    typedef struct {
        int oldStatus;
        int phase;  // Tracks which part of the avoidance sequence we're in
    } AvoidState;

    #define MAX_RECURSION_DEPTH 50
    AvoidState stateStack[MAX_RECURSION_DEPTH];
    int stackPointer = 0;

    // Initialize first state
    int oldStatus = moveScanData->status;
    stateStack[stackPointer].oldStatus = oldStatus;
    stateStack[stackPointer].phase = 0;
    stackPointer++;

    // Main iterative loop - continues until all states are processed
    while (stackPointer > 0) {
        // Pop current state
        stackPointer--;
        AvoidState currentState = stateStack[stackPointer];
        oldStatus = currentState.oldStatus;
        int phase = currentState.phase;

        // If starting fresh at this level, do initial rotation and scan
        if (phase == 0) {
            rotate_degrees(directionGlobal, 90, sensor_data, moveScanData, compassVals);
            scan_cone(38, 142, moveScanData, scanData);
            phase = 1;
        }

        // Main avoidance loop for this recursion level
        int continueLoop = 1;
        while (continueLoop && moveScanData->status != BOUNDARY) {

            // Handle cliff/bump encountered before movement
            if (phase == 1 && (moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT
                    || moveScanData->status == BUMPRIGHT))
            {
                sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                uart_sendStr(buffer);
                move_backward(sensor_data, compassVals, 6, directionGlobal);
                moveScanData->status = CLEAR;
                update_distance(-6, directionGlobal);

                // Push current state back to resume after "recursive" call
                stateStack[stackPointer].oldStatus = oldStatus;
                stateStack[stackPointer].phase = 1;
                stackPointer++;

                // Push new "recursive" call state
                stateStack[stackPointer].oldStatus = moveScanData->status;
                stateStack[stackPointer].phase = 0;
                stackPointer++;

                continueLoop = 0;  // Break to process new state
                continue;
            }

            moveScanData->status = CLEAR;

            // Check if object too close before first move
            if (phase == 1 && (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2))
            {
                // Push current state back
                stateStack[stackPointer].oldStatus = oldStatus;
                stateStack[stackPointer].phase = 1;
                stackPointer++;

                // Push new "recursive" call
                stateStack[stackPointer].oldStatus = moveScanData->status;
                stateStack[stackPointer].phase = 0;
                stackPointer++;

                continueLoop = 0;
                continue;
            }

            // First move - sideways
            if (phase == 1) {
                move_forward(sensor_data, moveScanData, scanData->driveDistHorizontal, compassVals, directionGlobal);
                update_distance(moveScanData->distanceTraveled, directionGlobal);

                if (moveScanData->status == BOUNDARY) {
                    phase = 10;  // Jump to end
                    continue;
                }
                else if (moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT) {
                    sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                    uart_sendStr(buffer);
                    move_backward(sensor_data, compassVals, 6, directionGlobal);
                    moveScanData->status = CLEAR;
                    update_distance(-6, directionGlobal);

                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 1;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 2;
            }

            // First rotation and scan
            if (phase == 2) {
                rotate_degrees(directionGlobal, -90, sensor_data, moveScanData, compassVals);
                scan_cone(38, 142, moveScanData, scanData);

                if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist) {
                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 2;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 3;
            }

            // Second move - straight
            if (phase == 3) {
                move_scan(sensor_data, moveScanData, scanData->driveDistVertical, 37, 142, compassVals, directionGlobal);
                update_distance(moveScanData->distanceTraveled, directionGlobal);

                if (moveScanData->status == BOUNDARY) {
                    phase = 10;
                    continue;
                }
                else if (moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT) {
                    move_backward(sensor_data, compassVals, 6, directionGlobal);
                    moveScanData->status = CLEAR;
                    update_distance(-6, directionGlobal);
                    sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                    uart_sendStr(buffer);

                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 3;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 4;
            }

            // Second rotation and scan
            if (phase == 4) {
                rotate_degrees(directionGlobal, -90, sensor_data, moveScanData, compassVals);
                scan_cone(38, 142, moveScanData, scanData);

                if (scanData->averageAdc < 25 || scanData->averageAdc <= scanData->driveDist/2) {
                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 4;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 5;
            }

            // Third move - straight again
            if (phase == 5) {
                if (oldStatus == CLIFF) {
                    scanData->driveDistHorizontal += 5;
                }

                move_scan(sensor_data, moveScanData, scanData->driveDistHorizontal, 37, 142, compassVals, directionGlobal);
                update_distance(moveScanData->distanceTraveled, directionGlobal);

                if (moveScanData->status == BOUNDARY) {
                    phase = 10;
                    continue;
                }
                else if (moveScanData->status == CLIFF || moveScanData->status == BUMPLEFT || moveScanData->status == BUMPRIGHT) {
                    move_backward(sensor_data, compassVals, 6, directionGlobal);
                    moveScanData->status = CLEAR;
                    update_distance(-6, directionGlobal);
                    sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                    uart_sendStr(buffer);

                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 5;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 6;
            }

            // Check if need special handling based on direction
            if (phase == 6) {
                if (fabs(directionGlobal - (turnStatus - 1)) == 2) {
                    rotate_degrees(directionGlobal, -90, sensor_data, moveScanData, compassVals);

                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 6;
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    continueLoop = 0;
                    continue;
                }

                phase = 10;  // Done with main sequence, go to tape finding
                break;
            }
        }

        // Phase 10: After completing avoidance, look for tape
        if (phase == 10 || moveScanData->status == BOUNDARY) {
            // Face appropriate direction based on turnStatus
            if (!turnStatus) {
                face_direction(directionGlobal, NEGATIVE_Y, sensor_data, moveScanData, compassVals);
            }
            else if (turnStatus == 1) {
                face_direction(directionGlobal, POSITIVE_X, sensor_data, moveScanData, compassVals);
            }
            else if (turnStatus == 2) {
                face_direction(directionGlobal, POSITIVE_Y, sensor_data, moveScanData, compassVals);
            }
            else if (turnStatus == 3) {
                face_direction(directionGlobal, NEGATIVE_X, sensor_data, moveScanData, compassVals);
            }

            // Moving to white tape at end of sequence
            while (moveScanData->status != BOUNDARY) {
                scan_cone(38, 142, moveScanData, scanData);

                if (scanData->averageAdc < 20) {
                    // Need to avoid another object - push state to continue tape search after
                    stateStack[stackPointer].oldStatus = oldStatus;
                    stateStack[stackPointer].phase = 11;  // Resume tape search after recursion
                    stackPointer++;

                    stateStack[stackPointer].oldStatus = moveScanData->status;
                    stateStack[stackPointer].phase = 0;
                    stackPointer++;

                    break;  // Process the new avoidance
                }
                else {
                    move_scan(sensor_data, moveScanData, 25, 37, 142, compassVals, directionGlobal);

                    if (moveScanData->status == CLIFF
                            || moveScanData->status == BUMPLEFT
                            || moveScanData->status == BUMPRIGHT)
                    {
                        move_backward(sensor_data, compassVals, 6, directionGlobal);
                        moveScanData->status = CLEAR;
                        update_distance(-6, directionGlobal);
                        sprintf(buffer, "cliff sensor hit: status %d/r/n", moveScanData->status);
                        uart_sendStr(buffer);

                        stateStack[stackPointer].oldStatus = oldStatus;
                        stateStack[stackPointer].phase = 11;
                        stackPointer++;

                        stateStack[stackPointer].oldStatus = moveScanData->status;
                        stateStack[stackPointer].phase = 0;
                        stackPointer++;

                        break;
                    }

                    update_distance(moveScanData->distanceTraveled, directionGlobal);
                }
            }

            // If we found boundary, recenter and update compass
            if (moveScanData->status == BOUNDARY) {
                re_center_tape(sensor_data, moveScanData, compassVals, directionGlobal);
                head = read_euler_heading(BNO055_ADDRESS_B) / 16;

                if (directionGlobal == POSITIVE_X) {
                    compassVals->headPosX = head;
                    compassVals->headNegY = ((head + 90) % 360);
                    compassVals->headNegX = ((head + 180) % 360);
                    compassVals->headPosY = ((head + 270) % 360);
                }
                if (directionGlobal == POSITIVE_Y) {
                    compassVals->headPosY = head;
                    compassVals->headPosX = ((head + 90) % 360);
                    compassVals->headNegY = ((head + 180) % 360);
                    compassVals->headNegX = ((head + 270) % 360);
                }
                if (directionGlobal == NEGATIVE_X) {
                    compassVals->headNegX = head;
                    compassVals->headPosY = ((head + 90) % 360);
                    compassVals->headPosX = ((head + 180) % 360);
                    compassVals->headNegY = ((head + 270) % 360);
                }
                if (directionGlobal == NEGATIVE_Y) {
                    compassVals->headNegY = head;
                    compassVals->headNegX = ((head + 90) % 360);
                    compassVals->headPosY = ((head + 180) % 360);
                    compassVals->headPosX = ((head + 270) % 360);
                }

                // Get back into direction to continue
                if (!turnStatus) {
                    face_direction(directionGlobal, POSITIVE_X, sensor_data, moveScanData, compassVals);
                }
                else if (turnStatus == 1) {
                    face_direction(directionGlobal, POSITIVE_Y, sensor_data, moveScanData, compassVals);
                }
                else if (turnStatus == 2) {
                    face_direction(directionGlobal, NEGATIVE_X, sensor_data, moveScanData, compassVals);
                }
                else if (turnStatus == 3) {
                    face_direction(directionGlobal, NEGATIVE_Y, sensor_data, moveScanData, compassVals);
                }
            }
        }
    }

    return 1;
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
    I2C1_Init();



    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    scan_info scanData;
    scanData.averageAdc = 0;
    scanData.averagePing = 0;
    scanData.pingWidth = 0;

    scanData.adcWidth =0;
    scanData.driveDist = 0;
    scanData.driveDistHorizontal = 25;
    scanData.driveDistVertical = 25;

    compassVals compassVals;
    compassVals.headNegX = 0;
    compassVals.headNegY = 0;
    compassVals.headPosX = 0;
    compassVals.headPosY = 0;

    move_scan_t moveScanData;

    moveScanData.distanceTraveled = 0;
    moveScanData.status = CLEAR;
    head = 0;
    int stop = 0;
    while (1)
    {

        char c = uart_receive();
        int b = button_getButton();


        if (c == 'w')
        {
            move_forward(sensor_data, &moveScanData, 70, &compassVals, directionGlobal);
            update_distance(70, directionGlobal);

        }
        if (c == 'e')
                {
                    move_scan(sensor_data, &moveScanData, 70 , 45, 135, &compassVals, directionGlobal);
                    update_distance(70, directionGlobal);

                }
        else if (c == 'a')
        {
          rotate_degrees(directionGlobal, 90, sensor_data, &moveScanData, &compassVals);
          angle_correct(sensor_data, &moveScanData, directionGlobal, &compassVals);
        }
        else if (c == 'd')
        {
            rotate_degrees(directionGlobal, -90, sensor_data, &moveScanData, &compassVals);
            angle_correct(sensor_data, &moveScanData, directionGlobal, &compassVals);
        }
        else if (c == 's')
        {
            move_backward(sensor_data, &compassVals, 30, directionGlobal);

            update_distance(-50, directionGlobal);

        }
        else if (c == 'g')
        {
            BNO055_Init();
            while (b != 1) {
                head =  read_euler_heading(BNO055_ADDRESS_B) / 16;
                b = button_getButton();
                lcd_printf("value %d \n 1 to start rotation", head); // divide by 16 and correlate to a value that is eithe NSWE I believe
                timer_waitMillis(50);

            }


            head =  read_euler_heading(BNO055_ADDRESS_B) / 16;
            compassVals.headPosX = head;
            compassVals.headNegY = ((head + 90) % 360);
            compassVals.headNegX = ((head + 180) % 360);
            compassVals.headPosY = ((head + 270) % 360);
            sprintf(buffer, "after headX set in main: posX %d negY: %d negX: %d headY: %d\r\n ", compassVals.headPosX, compassVals.headNegY, compassVals.headNegX, compassVals.headPosY);
            uart_sendStr(buffer);

            horizontalPos = 0;
            verticalPos = 0;
            directionGlobal = 0;

        }
        else if (c == 'h')
        {
            scan_cone(0, 180, &moveScanData, &scanData);
        }
        else if (c == '1' || button_getButton() == 1)
        {
            rickroll();
        }

        else if (c == 't')
        {
            //used to check white sensors
            while(1){

                oi_update(sensor_data);
                timer_waitMillis(100);
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

               }

        else if (c == 'm')
        {

            while (!stop)
            {
                oi_update(sensor_data);

                // scan before movement
                scan_cone(38, 142, &moveScanData, &scanData);


                int driveDist = fmin(30, scanData.averageAdc);
                move_scan(sensor_data, &moveScanData, driveDist, 38, 142, &compassVals, directionGlobal);

                int status = moveScanData.status;
                double distanceChange = moveScanData.distanceTraveled;
                update_distance(distanceChange, directionGlobal);
                if(OBJECT == status){

                    scan_cone(38, 142, &moveScanData, &scanData);
                }

                if(scanData.averageAdc < 20){



                    avoidObject(sensor_data, &moveScanData, &scanData, &compassVals);
                    sprintf(buffer, "Broke recursion %d <---------------------------\r\n", directionGlobal);
                    uart_sendStr(buffer);

                }
                else if (status == BOUNDARY)
                {
                    re_center_tape(sensor_data, &moveScanData, &compassVals, directionGlobal);
                    head = read_euler_heading(BNO055_ADDRESS_B) / 16;
                    if (directionGlobal == POSITIVE_X)
                    {
                        compassVals.headPosX = head;
                        compassVals.headNegY = ((head + 90) % 360);
                        compassVals.headNegX = ((head + 180) % 360);
                        compassVals.headPosY = ((head + 270) % 360);

                    }
                    if (directionGlobal == POSITIVE_Y)
                    {
                        compassVals.headPosY = head;
                        compassVals.headPosX = ((head + 90) % 360);
                        compassVals.headNegY = ((head + 180) % 360);
                        compassVals.headNegX = ((head + 270) % 360);
                    }
                    if (directionGlobal == NEGATIVE_X)
                    {
                        compassVals.headNegX = head;
                        compassVals.headPosY = ((head + 90) % 360);
                        compassVals.headPosX = ((head + 180) % 360);
                        compassVals.headNegY = ((head + 270) % 360);
                    }
                    if (directionGlobal == NEGATIVE_Y)
                    {
                        compassVals.headNegY = head;
                        compassVals.headNegX = ((head + 90) % 360);
                        compassVals.headPosY = ((head + 180) % 360);
                        compassVals.headPosX = ((head + 270) % 360);

                    }
                    if (!turnStatus) // first row
                    {

                        turnStatus = 1;
                        rotate_degrees(directionGlobal, 90, sensor_data, &moveScanData, &compassVals);

                        sprintf(buffer, "EDGE HORIZONTAL %.0f\r\n", horizontalPos);
                        uart_sendStr(buffer);
                    }
                    else if (turnStatus == 1) // second row
                    {
                        turnStatus = 2; // set to third row
                        rotate_degrees(directionGlobal, 90, sensor_data, &moveScanData, &compassVals);

                        sprintf(buffer, "EDGE VERTICAL %.0f\r\n", verticalPos);
                        uart_sendStr(buffer);

                    }
                    else if (turnStatus == 2) {
                        turnStatus = 3;
                        rotate_degrees(directionGlobal, 90, sensor_data, &moveScanData, &compassVals);

                        sprintf(buffer, "FOUND THIRD SIDE %.0f\r\n", verticalPos);
                        uart_sendStr(buffer);
                    }
                    else if (turnStatus == 3) {
                        stop = 1;
                        rotate_degrees(directionGlobal, 90, sensor_data, &moveScanData, &compassVals);

                        lcd_printf("finished perimeter!\r\n");
                        sprintf(buffer, "finished perimeter!\r\n");
                        rickroll();
                        uart_sendStr(buffer);
                        oi_free(sensor_data);
                        while (1);
                    }
                }
                else if(status == CLIFF || status == BUMPLEFT || status == BUMPRIGHT){

                    move_backward(sensor_data, &compassVals, 6, directionGlobal);
                    sprintf(buffer, "stuck in main/r/n", moveScanData.status);
                                            uart_sendStr(buffer);

                    moveScanData.status = CLEAR;
                    update_distance(-6, directionGlobal);
                    avoidObject(sensor_data, &moveScanData, &scanData, &compassVals);


                }

            }
        }
    }
    oi_free(sensor_data);

    return 0;
}
