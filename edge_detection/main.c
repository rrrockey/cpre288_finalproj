#include "open_interface.h"
#include "lcd.h"
#include <inc/tm4c123gh6pm.h>
/**
 * main.c
 */
int main(void)
{
    char buffer[200];
    timer_init();
    lcd_init();
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    while(1){

       oi_update(sensor_data);
    sprintf(buffer, "left %d\n FrontLeft %d\n FrontRight %d\n right %d", sensor_data->cliffLeftSignal, sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal, sensor_data->cliffRightSignal);
    lcd_printf(buffer);
}
	return 0;
}
