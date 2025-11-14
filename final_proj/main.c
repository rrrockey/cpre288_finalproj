#include "open_interface.h"
#include "movement.h"
#include "Timer.h"




/**
 * main.c
 */
int main(void)
{
    timer_init();
    lcd_init();
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    calibrate_forward_movement(sensor_data);

    oi_free(sensor_data);

    while (1);
	return 0;
}
