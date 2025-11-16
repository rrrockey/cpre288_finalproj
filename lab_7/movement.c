#include "open_interface.h"
#include "movement.h"
#include "timer.h"
#include "lcd.h"
#include <stdio.h>

/**
 * move forward the specified distance 'cm'
 * and return the distance traveled captured by the CyBot in millimeters
 */
double move_forward(oi_t *sensor_data, int cm) {
    double distanceTraveled = 0;
    oi_setWheels(200, 200);

    while (distanceTraveled < cm * MM_IN_CM * FORWARD_CORRECTION) {
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

    while (distanceTraveled > -cm * MM_IN_CM * FORWARD_CORRECTION) {
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

// function to calibrate forward distance, we may not end up needing this
// if we use the depth sensor to find the white tape
void calibrate_forward_movement(oi_t *sensor_data) {


    lcd_printf("Calibrating...\nStand by");
    timer_waitMillis(500);

    // --- step 0: set the FORWARD_CORRECTION to 1.0 ---

    // --- step 1: set number of tiles to move and move that distance
    // 1 tile (24 inches = 609.6 mm == 61 cm) ---
    int num_tiles = 1;
    double measured_mm = move_forward(sensor_data, 61 * num_tiles * FORWARD_CORRECTION);
                                     // what the cybot THINKS it moved, make sure FORWARD_CORRECTION is 1.0 at this step

    // --- step 2: read value from cyBot---
    lcd_printf("Robot measured: \n%f mm", measured_mm);

    // --- step 3: do the following and update the code ---
    // 1. Measure how many millimeters it actually moved (using meter stick on ground)
    // 3. Compute correction = actual_mm / measured_mm
    // 4. Update FORWARD_CORRECTION in source code
}

// function used for avoiding an obstacle in a linear drive
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
