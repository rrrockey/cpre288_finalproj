#include "open_interface.h"


#define MM_IN_CM 10 // 10 millimeters in a cm

#define BUMPLEFT 0
#define BOUNDARY 1
#define CLIFF 2
#define OBJECT 3
#define CLEAR 4
#define BUMPRIGHT 5

#define RIGHT 0
#define LEFT 1

#define DEBUG 0

extern volatile double TURN_CORRECTION;  // adjust for turn being slightly off

typedef struct {
    double distanceTraveled;
    int status; // object encountered (e.g. BOUNDARY, BUMP, CLIFF, OBJECT, etc.)
} move_scan_t;

typedef struct {
    int headPosX;
    int headPosY;
    int headNegX;
    int headNegY;
} compassVals;



void move_forward(oi_t *sensor_data, move_scan_t *moveData, int cm);

double move_backward(oi_t *sensor_data, int cm);

void turn_clockwise(oi_t *sensor_data, int degrees);

void turn_counterclockwise(oi_t *sensor_data, int degrees);

void avoid_obstacle(oi_t *sensor_data, int bump_status);

void calibrate_forward_movement(oi_t *sensor_data);

void calibrate_turning(oi_t *sensor_data);

void move_scan(oi_t *sensor_data, move_scan_t* moveScanData, int cm, float low_angle, float high_angle);

void calibrate_turn(oi_t *sensor_data);

void re_center_tape(oi_t *sensor_data, move_scan_t *moveScanData);
void move_forward_slow(oi_t *sensor_data, move_scan_t *moveScanData, int cm);

void angle_correct(oi_t *sensor_data, move_scan_t *moveScanData, int directionGlobal, compassVals *compassVals);

void calibrate_gyro_turn(oi_t *sensor_data);
