#include "open_interface.h"


#define MM_IN_CM 10 // 10 millimeters in a cm
#define TURN_CORRECTION 0.955  // adjust for turn being slightly off
#define FORWARD_CORRECTION = 0.99757824; // movement calibration multiplier TODO: SET THIS TO 1.0 BEFORE CALIBRATION

#define BUMP 0
#define BOUNDARY 1
#define CLIFF 2
#define OBJECT 3
#define CLEAR 4

#define DEBUG 0

typedef struct {
    double distanceTraveled;
    int status; // object encountered (e.g. BOUNDARY, BUMP, CLIFF, OBJECT, etc.)
} move_scan_t;



int move_forward(oi_t *sensor_data, int cm);

double move_backward(oi_t *sensor_data, int cm);

void turn_clockwise(oi_t *sensor_data, int degrees);

void turn_counterclockwise(oi_t *sensor_data, int degrees);

void avoid_obstacle(oi_t *sensor_data, int bump_status);

void calibrate_forward_movement(oi_t *sensor_data);

void calibrate_turning(oi_t *sensor_data);

void move_scan(oi_t *sensor_data, move_scan_t* moveScanData, int cm, float low_angle, float high_angle);
