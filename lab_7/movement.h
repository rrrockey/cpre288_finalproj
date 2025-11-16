#include "open_interface.h"

#define MM_IN_CM 10 // 10 millimeters in a cm
#define TURN_CORRECTION 0.955  // adjust for turn being slightly off
#define MOVE_OK 0
#define LEFT_BUMPED 1
#define RIGHT_BUMPED 2
#define DEBUG 0

double move_forward(oi_t *sensor_data, int cm);

double move_backward(oi_t *sensor_data, int cm);

void turn_clockwise(oi_t *sensor_data, int degrees);

void turn_counterclockwise(oi_t *sensor_data, int degrees);

void avoid_obstacle(oi_t *sensor_data, int bump_status);
