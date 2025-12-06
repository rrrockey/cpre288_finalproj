#include "open_interface.h"
#include "movement_f1.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>
#include "servo.h"
#include "adc.h"
#include "IMU.h"



#define WHITETAPE 2650
#define BLACKTAPE 500


volatile double TURN_CORRECTION = 0.99;
volatile double TURN_CORRECTION_180 = 0.98;
char buffer[100];

void move_forward(oi_t *sensor_data, move_scan_t *moveScanData, int cm)
{
    moveScanData->status = CLEAR;
    moveScanData->distanceTraveled = 0;
    double distanceTraveled = 0;
    oi_setWheels(150, 150);

    while (distanceTraveled < cm * MM_IN_CM)
    {
        oi_update(sensor_data);

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
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);

#if DEBUG
    printf("Moved forward: %.2f mm\n", distanceTraveled);
#endif

    moveScanData->distanceTraveled = distanceTraveled / 10;
    return;
}





void move_scan(oi_t *sensor_data, move_scan_t *moveScanData, int cm, float low_angle, float high_angle)
{
    moveScanData->status = CLEAR;
    moveScanData->distanceTraveled = 0;
    double distanceTraveled = 0;
    oi_setWheels(75, 75);
    float current_angle = low_angle;
    int IR_val = 0;
    double estimation = 0;
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
                || sensor_data->cliffFrontRightSignal < BLACKTAPE
                || sensor_data->cliffLeftSignal < BLACKTAPE
                || sensor_data->cliffRightSignal < BLACKTAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = CLIFF;
            break;
        }
        current_angle+= 5;
        if(current_angle > high_angle) current_angle = low_angle;
        servo_move(current_angle);
        IR_val = adc_read();
        estimation = 4150000 * pow(IR_val, -1.64);
        if(estimation < 20){
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = OBJECT;
            break;
        }
        distanceTraveled += sensor_data->distance;

    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
    moveScanData->distanceTraveled = distanceTraveled / 10; // get cm

#if DEBUG
    printf("Moved forward: %.2f mm\n", distanceTraveled);
#endif

    return;

}

double move_to_bounds(oi_t *sensor_data){
    double dist =0;
    oi_setWheels(200, 200);
    while(sensor_data->cliffFrontLeftSignal < WHITETAPE || sensor_data->cliffFrontRightSignal < WHITETAPE){

        oi_setWheels(0, 0);
        return dist;
    }
    dist += sensor_data->distance;

}

double move_backward(oi_t *sensor_data, int cm) {
    double distanceTraveled = 0;
    oi_setWheels(-200, -200);

    while (distanceTraveled > -cm * MM_IN_CM) {
        oi_update(sensor_data);
        distanceTraveled += sensor_data->distance;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);

#if DEBUG
    printf("Moved backward: %.2f mm\n", distanceTraveled);
#endif

    return -distanceTraveled; // Return positive value of distance moved
}


//void avoid_obstacle(oi_t *sensor_data, int bump_status) {
//
//    if (bump_status == LEFT_BUMPED) {
//        move_backward(sensor_data, 4);
//        turn_clockwise(sensor_data, 75);
//        move_forward(sensor_data, 33);
//        turn_counterclockwise(sensor_data, 120);
//    } else if (bump_status == RIGHT_BUMPED) {
//        move_backward(sensor_data, 4);
//        turn_counterclockwise(sensor_data, 75);
//        move_forward(sensor_data, 33);
//        turn_clockwise(sensor_data, 120);
//    }
//}


void turn_clockwise(oi_t *sensor_data, int degrees) {
    sprintf(buffer, "moving clockwise %.2f \r\n", TURN_CORRECTION);
    uart_sendStr(buffer);

    if (degrees == 180) {
        degrees *= TURN_CORRECTION_180;
    }
    else {
        degrees *= TURN_CORRECTION;
    }

    double angleTurned = 0;
    oi_setWheels(-50, 50);

    while (angleTurned > -degrees) {
        oi_update(sensor_data);
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}

void turn_counterclockwise(oi_t *sensor_data, int degrees) {
    if (degrees == 180) {
        degrees *= TURN_CORRECTION_180;
    }
    else {
        degrees *= TURN_CORRECTION;
    }

    double angleTurned = 0;
    oi_setWheels(50, -50);

    while (angleTurned < degrees) {
        oi_update(sensor_data);
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}

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

void re_center_tape(oi_t *sensor_data, move_scan_t *moveScanData) {
//    while(1){
//        oi_update(sensor_data);
//        timer_waitMillis(10);
//        sprintf(buffer, " left %d\n FrontLeft %d\n FrontRight %d\n right %d", sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
//        lcd_printf(buffer);
//    }



    move_backward(sensor_data, 2);
    move_forward_slow(sensor_data, moveScanData, 5);

    oi_update(sensor_data);
    sprintf(buffer, "in\nFrontLeft %d\n FrontRight %d\n",  sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
    lcd_printf(buffer);
    if (sensor_data->cliffFrontLeftSignal > WHITETAPE) {

        while (1) {

            oi_update(sensor_data);
            sprintf(buffer, "out\nFrontLeft %d\n FrontRight %d\n",  sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
            lcd_printf(buffer);
            if (sensor_data->cliffFrontRightSignal > WHITETAPE) {
                return;
            }
            move_backward(sensor_data, 2);
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
            move_backward(sensor_data, 2);
            turn_clockwise(sensor_data, 2);
            move_forward_slow(sensor_data, moveScanData, 50);

//            sprintf(buffer, "FrontLeft %d\n FrontRight %d\n",  sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
//            lcd_printf(buffer);
        }

    }
}

void angle_correct(oi_t *sensor_data, move_scan_t *moveScanData, int directionGlobal, compassVals *compassVals) {

    timer_waitMillis(1000);


    int currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
    int clockwise = 0;

    int intendedDirection;
    if (directionGlobal == 0) {
        intendedDirection = compassVals->headPosX;
        if(currentDirection - compassVals->headPosX > 0 || currentDirection-compassVals->headPosX < -180) {
            clockwise = 0;

        }
        else{
            clockwise = 1;
        }

    }
    if (directionGlobal == 1) {
        intendedDirection = compassVals->headPosY;
        if(currentDirection - compassVals->headPosY > 0  || currentDirection-compassVals->headPosY < -180) {
            clockwise = 0;

        }
        else{
            clockwise = 1;
        }

    }
    if (directionGlobal == 2) {
        intendedDirection = compassVals->headNegX;
        if(currentDirection - compassVals->headNegX > 0  || currentDirection-compassVals->headNegX < -180) {
            clockwise = 0;

        }
        if(currentDirection - compassVals->headNegX > 0  /*|| currentDirection-compassVals->headNegX < -180*/) {
            return;
        }
        else{
            clockwise = 1;
        }

    }
    if (directionGlobal == 3) {
        intendedDirection = compassVals->headNegY;
        if(currentDirection - compassVals->headNegY > 0  || currentDirection-compassVals->headNegY < -180){
            clockwise = 0;

        }
        else if(currentDirection - compassVals->headNegY == 0/*  || currentDirection-compassVals->headNegY == -180*/){
            return;
        }
        else{
            clockwise = 1;
        }

    }
    if(!clockwise){

        oi_setWheels(15, -15);

        while (currentDirection != intendedDirection) {
            lcd_printf("current: %d going %d\n dg: %d\nclockwise %d", currentDirection, intendedDirection, directionGlobal, clockwise);
            oi_update(sensor_data);

            currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
        }

        oi_setWheels(0, 0);
        timer_waitMillis(300);
    }
    else{
        oi_setWheels(-15, 15);

        while (currentDirection != intendedDirection) {
            lcd_printf("current: %d going %d\n dg: %d\nclockwise %d", currentDirection, intendedDirection, directionGlobal, clockwise);
                    oi_update(sensor_data);

                    currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
                }
                oi_setWheels(0, 0);

                timer_waitMillis(300);
    }



//    int point = (currentDirection - directionGlobalCompass ) % 360; //finding halfway for if statements
//
//
//
//
//    while(currentDirection != directionGlobalCompass) {
//        lcd_printf("current: %d \nGoing: %d dg %d\n point: %d\nheadVal %d", currentDirection, directionGlobalCompass, directionGlobal, point, headVal);
//
//        if(point > 0) {
//            turn_counterclockwise(sensor_data, 2);
//        } else {
//            turn_clockwise(sensor_data, 2);
//        }
//
//
//        currentDirection = read_euler_heading(BNO055_ADDRESS_B) / 16;
//        point = (currentDirection - directionGlobalCompass ) % 360;
//    }


}

void move_forward_slow(oi_t *sensor_data, move_scan_t *moveScanData, int cm) {
    moveScanData->status = CLEAR;
        moveScanData->distanceTraveled = 0;
        double distanceTraveled = 0;
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
        }

        oi_setWheels(0, 0);
        timer_waitMillis(300);

    #if DEBUG
        printf("Moved forward: %.2f mm\n", distanceTraveled);
    #endif

        moveScanData->distanceTraveled = distanceTraveled / 10;
        return;
}

