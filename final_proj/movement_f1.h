#include "open_interface.h"


#define MM_IN_CM 10 // 10 millimeters in a cm

#define BOUNDARY 1
#define CLIFF 2
#define OBJECT 3
#define CLEAR 4
#define BUMPRIGHT 5
#define BUMPLEFT 6

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
#define LEFTWHEELSCALAR 1.00
#define RIGHTWHEELSCALAR 1.00
#define LEFTNOTSCAN 1.00
#define RIGHTNOTSCAN 1.00

// basic move forward function that has all the dections that we need inside of it. 
void move_forward(oi_t *sensor_data, move_scan_t *moveData, int cm, compassVals *compassVals, int directionGlobal);

// Same as move forward but the ping and IR are detecting obejcts as well. This takes a low angle and high angle so that you can customize how much is being seen. 
void move_scan(oi_t *sensor_data, move_scan_t* moveScanData, int cm, float low_angle, float high_angle, compassVals *compassVals, int directionGlobal);

// simple move backward function
double move_backward(oi_t *sensor_data, compassVals *compassVals, int cm, int directionGlobal);

// moving backward without straight correcting because it was causing issue with the direction
double move_backward_no_straight_correct(oi_t *sensor_data, compassVals *compassVals, int cm, int directionGlobal);

// simple turn clockwise function
void turn_clockwise(oi_t *sensor_data, int degrees);

// simple turn_counterclockwise function
void turn_counterclockwise(oi_t *sensor_data, int degrees);

// used to calibrate the wheels so that turns are more accurate
void calibrate_turn(oi_t *sensor_data);

// rechecks tape to make sure the bot is centered at any occasion it can be.
// Checks that both front white sensors are above a threshold to make sure that they are both on tape and in theory centered
void re_center_tape(oi_t *sensor_data, move_scan_t *moveScanData, compassVals *compassVals, int directionGlobal);

// This is the function that takes in the angle shifted by OI and corrects the direction we are facing with the IMU value. We were having trouble relying only on IMU so this was our compromise
void straight_correct(oi_t *sensor_data, compassVals *compassVals, double angleTurned, int directionGlobal);

// angle correct is the same as straight correct other than the logic for turning is fully based on the IMU. We found the 90 degree turns were the most accurate when only relying on the IMU
void angle_correct(oi_t *sensor_data, move_scan_t *moveScanData, int directionGlobal, compassVals *compassVals);

// move_forward_slow is only used in the white tape correct scenario but it was implemented to avoid having to rewrite all the moves in main.
// it also has no straight correction because we use it in white tape correction
void move_forward_slow(oi_t *sensor_data, move_scan_t *moveScanData, int cm);



