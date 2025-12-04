#include "open_interface.h"
#include "movement_f1.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>
#include "servo.h"
#include "adc.h"


#define WHITETAPE 2650
#define BLACKTAPE 100

volatile double TURN_CORRECTION = 0.96;
volatile double TURN_CORRECTION_180 = 0.955;
char buffer[100];

void move_forward(oi_t *sensor_data, move_scan_t *moveScanData, int cm)
{
    moveScanData->status = CLEAR;
    moveScanData->distanceTraveled = 0;
    double distanceTraveled = 0;
    oi_setWheels(200, 200);

    while (distanceTraveled < cm * MM_IN_CM)
    {
        oi_update(sensor_data);

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            moveScanData->status = BUMP;
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


int trace_hole(oi_t *sensor_data)
{
    double distanceTraveled = 0;
    oi_setWheels(50, 50);

    while (1)
    {
        oi_update(sensor_data);

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BUMP;
        }
        if (sensor_data->cliffFrontLeftSignal > WHITETAPE
                || sensor_data->cliffFrontRightSignal > WHITETAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BOUNDARY;
        }
        if (sensor_data->cliffLeftSignal >= BLACKTAPE
                && sensor_data->cliffRightSignal >= BLACKTAPE)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return CLIFF;
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

    return distanceTraveled;
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
            moveScanData->status = BUMP;
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
        turn_clockwise(sensor_data, 90);

        printf("Done.\r\n");
    }
}

