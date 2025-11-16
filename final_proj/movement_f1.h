#include "open_interface.h"

#define MM_IN_CM 10 // 10 millimeters in a cm
#define TURN_CORRECTION 0.955  // adjust for turn being slightly off
#define FORWARD_CORRECTION = 0.99757824; // movement calibration multiplier TODO: SET THIS TO 1.0 BEFORE CALIBRATION

#define BUMP 0
#define BOUNDARY 1
#define CLIFF 2
#define DEBUG 0

int move_forward(oi_t *sensor_data, int cm);

double move_backward(oi_t *sensor_data, int cm);

void turn_clockwise(oi_t *sensor_data, int degrees);

void turn_counterclockwise(oi_t *sensor_data, int degrees);

void avoid_obstacle(oi_t *sensor_data, int bump_status);

void calibrate_forward_movement(oi_t *sensor_data);

void calibrate_turning(oi_t *sensor_data);
