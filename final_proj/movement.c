#include "open_interface.h"
#include "movement.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>

double move_forward(oi_t *sensor_data, int cm) {
    double distanceTraveled = 0;
    oi_setWheels(200, 200);

    while (distanceTraveled < cm * MM_IN_CM) {
        oi_update(sensor_data);

        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            oi_setWheels(0, 0);
            distanceTraveled += sensor_data->distance;
            return distanceTraveled;
        }
        else {

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


void avoid_obstacle(oi_t *sensor_data, int bump_status) {

    if (bump_status == LEFT_BUMPED) {
        move_backward(sensor_data, 4);
        turn_clockwise(sensor_data, 75);
        move_forward(sensor_data, 33);
        turn_counterclockwise(sensor_data, 120);
    } else if (bump_status == RIGHT_BUMPED) {
        move_backward(sensor_data, 4);
        turn_counterclockwise(sensor_data, 75);
        move_forward(sensor_data, 33);
        turn_clockwise(sensor_data, 120);
    }
}


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
