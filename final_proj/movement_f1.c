#include "open_interface.h"
#include "movement_f1.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>
#include "servo.h"
#include "adc.h"
#include "IMU.h"


//Making sure that we have calibration values for sensors. 

#define WHITETAPE 2600
#define WHITETAPE_RIGHT 2450
#define BLACKTAPE 500 //500
#define POSITIVE_X 0
#define POSITIVE_Y 1
#define NEGATIVE_X 2
#define NEGATIVE_Y 3

volatile double TURN_CORRECTION = 1;
volatile double TURN_CORRECTION_180 = 1;
char buffer[100];

// basic move forward function that has all the dections that we need inside of it. 
void move_forward(oi_t *sensor_data, move_scan_t *moveScanData, int cm, compassVals *compassVals, int directionGlobal)
{
    moveScanData->status = CLEAR;
    moveScanData->distanceTraveled = 0;
    double distanceTraveled = 0;
    double angleTurned = 0;
    oi_setWheels(((int16_t)(200*RIGHTNOTSCAN)), ((int16_t)(200*LEFTNOTSCAN)));

    while (distanceTraveled < cm * MM_IN_CM)
    {

        oi_update(sensor_data);
        

        //bump dection split between left and right to change handling
        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            if(sensor_data->bumpLeft){
                moveScanData->status = BUMPLEFT;
            }
            else{
                moveScanData->status = BUMPRIGHT;
            }

            break;
        }

        //white tape sensors for if we are hitting a boundary head on
        if (sensor_data->cliffFrontLeftSignal > WHITETAPE
                || sensor_data->cliffFrontRightSignal > WHITETAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = BOUNDARY;
            break;
        }



        //white time on the rightside for when our bot is drifting
        if(sensor_data->cliffRightSignal > WHITETAPE_RIGHT && (!(sensor_data->cliffFrontLeftSignal > WHITETAPE
                || sensor_data->cliffFrontRightSignal > WHITETAPE))){
            oi_setWheels(0,0);
            turn_counterclockwise(sensor_data, 90);

            oi_setWheels(((int16_t)(200*RIGHTNOTSCAN)), ((int16_t)(200*LEFTNOTSCAN)));
            timer_waitMillis(300);
            oi_setWheels(0,0);


            turn_clockwise(sensor_data, 90);
            angle_correct(sensor_data, moveScanData, directionGlobal, compassVals);
            break;
        }


        //cliff sensors for if we are driving into a hole
        if (sensor_data->cliffFrontLeftSignal < BLACKTAPE
                  || sensor_data->cliffFrontRightSignal < BLACKTAPE
                  || sensor_data->cliffLeftSignal < BLACKTAPE
                  || sensor_data->cliffRightSignal < BLACKTAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = CLIFF;
            break;
        }
        else
        {

        }

        distanceTraveled += sensor_data->distance;
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);

    //straight correct called to make sure our turns match our IMU value
    straight_correct(sensor_data, compassVals, angleTurned, directionGlobal);
    timer_waitMillis(300);

#if DEBUG
    printf("Moved forward: %.2f mm\n", distanceTraveled);
#endif

    moveScanData->distanceTraveled = distanceTraveled / 10;
    return;
}




/**
 * Same as move forward but the ping and IR are detecting obejcts as well. This takes a low angle and high angle so that you can customize how much is being seen. 
 */
void move_scan(oi_t *sensor_data, move_scan_t *moveScanData, int cm, float low_angle, float high_angle, compassVals *compassVals, int directionGlobal)
{
    moveScanData->status = CLEAR;
    moveScanData->distanceTraveled = 0;
    double distanceTraveled = 0;
    double angleTurned = 0;



    // Slower motion because there should be enough time to get through the scan
    oi_setWheels( ((int16_t)(75*RIGHTWHEELSCALAR)),  ((int16_t)(75*LEFTWHEELSCALAR)));
    float current_angle = low_angle;
    int IR_val = 0;
    double estimation = 0;

    while (distanceTraveled < cm * MM_IN_CM)
    {
        oi_update(sensor_data);
        //bump detect
        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            if(sensor_data->bumpLeft){
                moveScanData->status =  BUMPLEFT;
                        }
                        else{
                            moveScanData->status =  BUMPRIGHT;
                        }

            break;
        }

        //front white tape
        if (sensor_data->cliffFrontLeftSignal > WHITETAPE
                || sensor_data->cliffFrontRightSignal > WHITETAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = BOUNDARY;
            break;
        }


        // right tape sensor
        if(sensor_data->cliffRightSignal > WHITETAPE_RIGHT && (!(sensor_data->cliffFrontLeftSignal > WHITETAPE
                        || sensor_data->cliffFrontRightSignal > WHITETAPE))){
                    oi_setWheels(0,0);
                    turn_counterclockwise(sensor_data, 90);

                    oi_setWheels(((int16_t)(200*RIGHTNOTSCAN)), ((int16_t)(200*LEFTNOTSCAN)));
                    timer_waitMillis(300);
                    oi_setWheels(0,0);

                    turn_clockwise(sensor_data, 90);
                    angle_correct(sensor_data, moveScanData, directionGlobal, compassVals);
                    break;
                }

        // black tape sensor
        if (sensor_data->cliffFrontLeftSignal < BLACKTAPE
                || sensor_data->cliffFrontRightSignal < BLACKTAPE
                || sensor_data->cliffLeftSignal < BLACKTAPE
                || sensor_data->cliffRightSignal < BLACKTAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = CLIFF;
            break;
        }

        // adding to the angle to move the servor
        current_angle+= 5;
        if(current_angle > high_angle) current_angle = low_angle;
        servo_move(current_angle);

        IR_val = adc_read();
        estimation = 4150000 * pow(IR_val, -1.64);

        //breakout if IR is less than 20
        if(estimation < 20){
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = OBJECT;
            break;
        }

        distanceTraveled += sensor_data->distance;

        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);

    //straight correct to make sure that the it is matching IMU direction
    straight_correct(sensor_data, compassVals, angleTurned, directionGlobal);

    timer_waitMillis(300);
    moveScanData->distanceTraveled = distanceTraveled / 10; // get cm


#if DEBUG
    printf("Moved forward: %.2f mm\n", distanceTraveled);
#endif

    return;

}

// simple move backward function
double move_backward(oi_t *sensor_data, compassVals *compassVals, int cm, int directionGlobal) {
    double distanceTraveled = 0;
    double angleTurned = 0;
    oi_setWheels( ((int16_t)(-200*RIGHTNOTSCAN)),  ((int16_t)(-200*LEFTNOTSCAN)));

    while (distanceTraveled > -cm * MM_IN_CM) {
        oi_update(sensor_data);
        distanceTraveled += sensor_data->distance;
        angleTurned += sensor_data->angle;
    }


    oi_setWheels(0, 0);
    straight_correct(sensor_data, compassVals, angleTurned, directionGlobal);
    timer_waitMillis(300);
    sprintf(buffer, "angle turned in move backward: %.2f\r\n", angleTurned);
    uart_sendStr(buffer);
#if DEBUG
    printf("Moved backward: %.2f mm\n", distanceTraveled);
#endif

    return -distanceTraveled; // Return positive value of distance moved
}


//moving backward without straight correcting because it was causing issue with the direction
double move_backward_no_straight_correct(oi_t *sensor_data, compassVals *compassVals, int cm, int directionGlobal) {
    double distanceTraveled = 0;
    double angleTurned = 0;
    oi_setWheels( ((int16_t)(-200*RIGHTNOTSCAN)),  ((int16_t)(-200*LEFTNOTSCAN)));

    while (distanceTraveled > -cm * MM_IN_CM) {
        oi_update(sensor_data);
        distanceTraveled += sensor_data->distance;
        angleTurned += sensor_data->angle;
    }


    oi_setWheels(0, 0);
    timer_waitMillis(300);
    sprintf(buffer, "angle turned in move backward: %.2f\r\n", angleTurned);
    uart_sendStr(buffer);
#if DEBUG
    printf("Moved backward: %.2f mm\n", distanceTraveled);
#endif

    return -distanceTraveled; // Return positive value of distance moved
}

// simple turn clockwise function
void turn_clockwise(oi_t *sensor_data, int degrees) {


    if (degrees == 180) {
        degrees *= TURN_CORRECTION_180;
    }
    else {
        degrees *= TURN_CORRECTION;
    }

    double angleTurned = 0;
    oi_setWheels(-40, 40);

    while (angleTurned > -degrees) {
        oi_update(sensor_data);

        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}


// simple turn_counterclockwise function
void turn_counterclockwise(oi_t *sensor_data, int degrees) {
    if (degrees == 180) {
        degrees *= TURN_CORRECTION_180;
    }
    else {
        degrees *= TURN_CORRECTION;
    }

    double angleTurned = 0;
    oi_setWheels(40, -40);

    while (angleTurned < degrees) {
        oi_update(sensor_data);
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}


// used to calibrate the wheels so that turns are more accurate
void calibrate_turn(oi_t *sensor_data) {
    sprintf(buffer, "Enter TURN_CORRECTION values. Example: 1.10\r\n");
    uart_sendStr(buffer);

    while (1) {
        uart_sendStr(">> ");
        double newValue = uart_receive_double();

        TURN_CORRECTION = newValue;
        sprintf(buffer, " TURN_CORRECTION = %.2f\r\n", TURN_CORRECTION);
        uart_sendStr(buffer);

        // Run test turn
        turn_counterclockwise(sensor_data, 90);

        printf("Done.\r\n");
    }
}

// rechecks tape to make sure the bot is centered at any occasion it can be.
// Checks that both front white sensors are above a threshold to make sure that they are both on tape and in theory centered
void re_center_tape(oi_t *sensor_data, move_scan_t *moveScanData, compassVals *compassVals, int directionGlobal) {

    move_backward_no_straight_correct(sensor_data, compassVals, 2, directionGlobal);
    move_forward_slow(sensor_data, moveScanData, 50);

    oi_update(sensor_data);

    if (sensor_data->cliffFrontLeftSignal > WHITETAPE) {

        while (1) {

            oi_update(sensor_data);
            sprintf(buffer, "out\nFrontLeft %d\n FrontRight %d\n",  sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
            lcd_printf(buffer);
            if (sensor_data->cliffFrontRightSignal > WHITETAPE && sensor_data->cliffFrontLeftSignal) {
                move_backward_no_straight_correct(sensor_data, compassVals, 2, directionGlobal);
                return;
            }
            move_backward_no_straight_correct(sensor_data, compassVals, 2, directionGlobal);
            turn_counterclockwise(sensor_data, 2);
            move_forward_slow(sensor_data, moveScanData, 50);


        }

    }

    else if (sensor_data->cliffFrontRightSignal > WHITETAPE) {
        while (1) {

            oi_update(sensor_data);
            if (sensor_data->cliffFrontLeftSignal > WHITETAPE) {
                return;
            }
            move_backward_no_straight_correct(sensor_data,compassVals, 2,  directionGlobal);
            turn_clockwise(sensor_data, 2);
            move_forward_slow(sensor_data, moveScanData, 50);

        }

    }
}


/**
 * This is the function that takes in the angle shifted by OI and corrects the direction we are facing with the IMU value. We were having trouble relying only on IMU so this was our compromise
 */
void straight_correct(oi_t *sensor_data, compassVals *compassVals, double angleTurned, int directionGlobal) {
    timer_waitMillis(1000);



    lcd_printf("straight_correct");

    int currentAngle =  read_euler_heading(BNO055_ADDRESS_B) / 16;


    // calc the angle that we are supposed to be facing
    int goingTo = (currentAngle + (int) angleTurned) % 360;
    if (goingTo < 0){
        goingTo += 360;
    }
    int lower = goingTo -1;
    int upper = goingTo+1;
    if (lower < 0){
            lower += 360;
        }
    if (upper < 0){
            upper += 360;
        }


    // determining the direction that is most effecient to reach the angle. logic for CW and CCW is the same other than the situation that they are called
    if (angleTurned > 0){ //cw

        //Wheel speed is set slow so we dont miss thevalue 
        oi_setWheels(-15, 15);

            while (1)
            {
                if(lower<=upper){
                    if(currentAngle >= lower && currentAngle <= upper) break;
                }
                else{
                    if(currentAngle >= lower || currentAngle <= upper) break;
                }

                oi_update(sensor_data);
                currentAngle = read_euler_heading(BNO055_ADDRESS_B) / 16;

            }
        oi_setWheels(0,0);
    } else if (angleTurned < 0) { //ccw
        oi_setWheels(15, -15);
            while(1)
            {
                if(lower<=upper){
                    if(currentAngle >= lower && currentAngle <= upper) break;
                }
                else{
                    if(currentAngle >= lower || currentAngle <= upper) break;
                }
                lcd_printf("head: %d", currentAngle);
            oi_update(sensor_data);
            currentAngle = read_euler_heading(BNO055_ADDRESS_B) / 16;

        }
            oi_setWheels(0,0);
    }


    //This is making sure that after we correct we redo the heading value because we were having drifting issue still if we didnt do this. 

    currentAngle = read_euler_heading(BNO055_ADDRESS_B) / 16;
    if (directionGlobal == POSITIVE_X) {
        compassVals->headPosX = currentAngle;
        compassVals->headNegY = ((currentAngle + 90) % 360);
        compassVals->headNegX = ((currentAngle + 180) % 360);
        compassVals->headPosY = ((currentAngle + 270) % 360);

    }
    if (directionGlobal == POSITIVE_Y) {
        compassVals->headPosY = currentAngle;
        compassVals->headPosX = ((currentAngle + 90) % 360);
        compassVals->headNegY = ((currentAngle + 180) % 360);
        compassVals->headNegX = ((currentAngle + 270) % 360);
    }
    if (directionGlobal == NEGATIVE_X) {
        compassVals->headNegX = currentAngle;
        compassVals->headPosY = ((currentAngle + 90) % 360);
        compassVals->headPosX = ((currentAngle + 180) % 360);
        compassVals->headNegY = ((currentAngle + 270) % 360);
    }
    if (directionGlobal == NEGATIVE_Y) {
        compassVals->headNegY = currentAngle;
        compassVals->headNegX = ((currentAngle + 90) % 360);
        compassVals->headPosY = ((currentAngle + 180) % 360);
        compassVals->headPosX = ((currentAngle + 270) % 360);

    }



}


/**
 * angle correct is the same as straight correct other than the logic for turning is fully based on the IMU. We found the 90 degree turns were the most accurate when only relying on the IMU
 */
void angle_correct(oi_t *sensor_data, move_scan_t *moveScanData, int directionGlobal, compassVals *compassVals) {

    timer_waitMillis(1000);

    int currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;

    int intendedDirection;

    //sets the intended IMU direction based on direction we are facing. 
    if (directionGlobal == 0) {
        intendedDirection = compassVals->headPosX;
    }
    if (directionGlobal == 1) {
        intendedDirection = compassVals->headPosY;
    }
    if (directionGlobal == 2) {
        intendedDirection = compassVals->headNegX;
    }
    if (directionGlobal == 3) {
        intendedDirection = compassVals->headNegY;
    }


    // if the direction and intended direction are the same the loop should break because it means the bot is at the correct angle
    while(currentDirection != intendedDirection){
       
        int delta = currentDirection - intendedDirection;
        if ((delta > 0 && delta < 180) || delta < -180)
        {  //counter clockwise
            oi_setWheels(15, -15);
        }
        else
        {
            oi_setWheels(-15, 15);
        }

        oi_update(sensor_data);
        currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);

    // We put this here because we were already correcting the IMU after every move forward so we decided we should also do it after a turn. 
    currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
        if (directionGlobal == POSITIVE_X) {
            compassVals->headPosX = currentDirection;
            compassVals->headNegY = ((currentDirection + 90) % 360);
            compassVals->headNegX = ((currentDirection + 180) % 360);
            compassVals->headPosY = ((currentDirection + 270) % 360);

        }
        if (directionGlobal == POSITIVE_Y) {
            compassVals->headPosY = currentDirection;
            compassVals->headPosX = ((currentDirection + 90) % 360);
            compassVals->headNegY = ((currentDirection + 180) % 360);
            compassVals->headNegX = ((currentDirection + 270) % 360);
        }
        if (directionGlobal == NEGATIVE_X) {
            compassVals->headNegX = currentDirection;
            compassVals->headPosY = ((currentDirection + 90) % 360);
            compassVals->headPosX = ((currentDirection + 180) % 360);
            compassVals->headNegY = ((currentDirection + 270) % 360);
        }
        if (directionGlobal == NEGATIVE_Y) {
            compassVals->headNegY = currentDirection;
            compassVals->headNegX = ((currentDirection + 90) % 360);
            compassVals->headPosY = ((currentDirection + 180) % 360);
            compassVals->headPosX = ((currentDirection + 270) % 360);

        }


}


/**
 * move_forward_slow is only used in the white tape correct scenario but it was implemented to avoid having to rewrite all the moves in main.
 * it also has no straight correction because we use it in white tape correction
 */
void move_forward_slow(oi_t *sensor_data, move_scan_t *moveScanData, int cm) {
    moveScanData->status = CLEAR;
        moveScanData->distanceTraveled = 0;
        double distanceTraveled = 0;
        double angleTurned = 0;
        oi_setWheels(30, 30);

        while (distanceTraveled < cm * MM_IN_CM)
        {
            oi_update(sensor_data);

            if (sensor_data->bumpLeft || sensor_data->bumpRight)
            {
                oi_setWheels(0, 0);
                distanceTraveled += sensor_data->distance;
                if(sensor_data->bumpLeft){
                    moveScanData->status =  BUMPLEFT;
                }
                else{
                    moveScanData->status =  BUMPRIGHT;
                }
                break;
            }
            if (sensor_data->cliffFrontLeftSignal > WHITETAPE
                    || sensor_data->cliffFrontRightSignal > WHITETAPE)
            {
                oi_setWheels(0, 0);
                distanceTraveled += sensor_data->distance;
                moveScanData->status = BOUNDARY;
                break;
            }
            if (sensor_data->cliffFrontLeftSignal < BLACKTAPE
                    || sensor_data->cliffFrontRightSignal < BLACKTAPE)
            {
                oi_setWheels(0, 0);
                distanceTraveled += sensor_data->distance;
                moveScanData->status = CLIFF;
                break;
            }
            else
            {

            }

            distanceTraveled += sensor_data->distance;
            angleTurned += sensor_data->distance;
        }
        oi_setWheels(0, 0);

        timer_waitMillis(300);

    #if DEBUG
        printf("Moved forward: %.2f mm\n", distanceTraveled);
    #endif

        moveScanData->distanceTraveled = distanceTraveled / 10;
        return;
}


