#include "open_interface.h"
#include "movement_f1.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>
#include "servo.h"
#include "adc.h"


int move_forward(oi_t *sensor_data, int cm)
{
    double distanceTraveled = 0;
    oi_setWheels(200, 200);

    while (distanceTraveled < cm * MM_IN_CM)
    {
        oi_update(sensor_data);

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BUMP;
        }
        if (sensor_data->cliffFrontLeftSignal > 2500
                || sensor_data->cliffFrontRightSignal > 2500)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BOUNDARY;
        }
        if (sensor_data->cliffFrontLeftSignal < 50
                || sensor_data->cliffFrontRightSignal < 50)
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
        if (sensor_data->cliffFrontLeftSignal > 2500
                || sensor_data->cliffFrontRightSignal > 2500)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BOUNDARY;
        }
        if (sensor_data->cliffLeftSignal >= 100
                && sensor_data->cliffRightSignal >= 100)
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


int move_scan(oi_t *sensor_data, int cm, float low_angle, float high_angle)
{
    double distanceTraveled = 0;
    oi_setWheels(125, 125);
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
            return BUMP;
        }
        if (sensor_data->cliffFrontLeftSignal > 2500
                || sensor_data->cliffFrontRightSignal > 2500)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return BOUNDARY;
        }
        if (sensor_data->cliffFrontLeftSignal < 50
                || sensor_data->cliffFrontRightSignal < 50)
        {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return CLIFF;
        }
        else
        {

        }
        current_angle+= 5;
        if(current_angle > high_angle) current_angle = low_angle;
        servo_move(current_angle);
        IR_val = adc_read();
        estimation = 0.0000228813 * (IR_val * IR_val) - 0.0981288 * IR_val + 115.33455;
        if(estimation < 25){
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return OBJECT;
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

double move_to_bounds(oi_t *sensor_data){
    double dist =0;
    oi_setWheels(200, 200);
    while(sensor_data->cliffFrontLeftSignal < 2500 || sensor_data->cliffFrontRightSignal < 2500){

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
    double angleTurned = 0;
    degrees *= TURN_CORRECTION;
    oi_setWheels(-50, 50);

    while (angleTurned > -degrees) {
        oi_update(sensor_data);
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}

void turn_counterclockwise(oi_t *sensor_data, int degrees) {
    double angleTurned = 0;
    degrees *= TURN_CORRECTION;
    oi_setWheels(50, -50);

    while (angleTurned < degrees) {
        oi_update(sensor_data);
        angleTurned += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}
