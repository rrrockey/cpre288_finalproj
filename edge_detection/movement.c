/*
 * movement.c
 *
 *  Created on: Sep 11, 2025
 *      Author: jkmurphy
 */
#include "open_interface.h"

double move_forward(oi_t *sensor, int centimeters){
         double sum = 0;
         oi_setWheels(200,200); // move forward; full speed
         while (sum < centimeters) {
         oi_update(sensor);

        if (sensor->bumpLeft)
        {

       //     oi_setWheels(0, 0);
            return sum;
        }
        if (sensor->bumpRight)
        {
        //    oi_setWheels(0, 0);

            return sum;
        }



         sum += sensor->distance;
         }
         oi_setWheels(0, 0); // stop

         return sum;
}
void move_backward(oi_t *sensor, int centimeters){
         double sum = 0;
         oi_setWheels(-200,-200); // move forward; full speed
         while (sum < centimeters) {
         oi_update(sensor);
         sum -= sensor->distance;
         }
         oi_setWheels(0, 0); // stop

}
void turn_clockwise(oi_t *sensor, double degrees){
    double sum = 0;
    oi_setWheels(-50,50); // move forward; full speed
    sensor->angle = 0.0;
    while (sum < degrees) {
        oi_update(sensor);
        //oi_getDegrees(sensor);
        sum -= sensor->angle;
        }
    oi_setWheels(0, 0); // stop


}
void turn_counter_clockwise(oi_t *sensor, double degrees){
    double sum = 0;
    oi_setWheels(50,-50); // move forward; full speed
    sensor->angle = 0.0;
    while (sum < degrees) {
        oi_update(sensor);

        //oi_getDegrees(sensor);
        sum += sensor->angle;
        }
    oi_setWheels(0, 0); // stop


}







