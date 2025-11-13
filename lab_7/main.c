//#include "cyBot_Scan.h"
#include "open_interface.h"
#include "movement.h"
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "servo.h"
#include "button.h"
#include "ping.h"
#include "servo_calibration.h"
#include "adc.h"

/**
 * main.c for lab7
 */

typedef struct {
    float distance_PING;
    double distance_IR;
    int angle;
} scan_t;


typedef struct {
    int start_angle;
    int end_angle;
    int mid_angle;
    double ping_distance;
    double width;
} object_t;

static scan_t scan_array[91];
static char uart_buffer[100];  // Reusable buffer for UART messages

int mode = 0; // 0-autonomous, 1-manual

int cyBot_getByte_nb(char *c) {
    if ((UART1_FR_R & UART_FR_RXFE) == 0) { // if RX FIFO is not empty
        *c = UART1_DR_R & 0xFF;
        return 1; // byte received
    }
    return 0; // no byte available
}


// Convert degrees to radians
#define DEG_TO_RAD(angle) ((angle) * M_PI / 180.0)
// finds the actual angle the cybot must turn based on the distance and angle captured
double calculate_phi(double angle, double distance)
{
    double theta_rad = DEG_TO_RAD(angle);
    double a = distance * sin(theta_rad);
    double b = distance * cos(theta_rad) + 13.0;

    // Calculate phi in radians
    double phi_rad = atan(a / b);

    // Convert phi back to degrees
    double phi_deg = phi_rad * 180.0 / M_PI;

    return phi_deg;
}

double convert_rawIR_to_distance(int IR_raw_val)
{
    double base = (double)IR_raw_val / 11106.0;
    double exponent = -1.0 / 0.599;
    volatile double distance = pow(base, exponent);

    printf("Base: %f, Exponent: %f, Distance: %f\n", base, exponent, distance);

    return distance;
}


int incremental_drive(oi_t *sensor_data, int clockwise, int new_angle, double distance) {

    if (clockwise) sprintf(uart_buffer, "Turn %d degrees clockwise, move forward %f cm\r\n", new_angle, distance);
    else sprintf(uart_buffer, "Turn %d degrees counterclockwise, move forward %f cm\r\n", new_angle, distance);
    uart_sendStr(uart_buffer);
    uart_sendStr("Enter [f] to proceed\r\n");

    char input = uart_receive();
    if (input) {
        // Echo command back
        sprintf(uart_buffer, "\r\n[%c] ", input);
        uart_sendStr(uart_buffer);

        if (input == 'f') {
            if (clockwise) turn_clockwise(sensor_data, new_angle);
            else  turn_counterclockwise(sensor_data, new_angle);

            move_forward(sensor_data, distance);

            oi_update(sensor_data);

            if (sensor_data->bumpLeft || sensor_data->bumpRight) {
                int bump_type = sensor_data->bumpLeft ? LEFT_BUMPED : RIGHT_BUMPED;
                uart_sendStr("avoiding obstacle...\r\n");
                avoid_obstacle(sensor_data, bump_type);

                //bumped, failure
                return 0;
            }
            else {
                //success
                uart_sendStr("found obstacle\r\n");
                return 1;
            }
        }
        else {
            uart_sendStr("did not receive [f]. aborting...\r\n");
            return 1;
        }
    }
    return 0;

}

int drive_to_smallest_object(oi_t *sensor_data, int angle, double distance) {
    int status;

    distance-=10;
    //turn
    if (angle != 90)
    {
        // 91 - 180
        if (angle > 90)
        {
            int new_angle = calculate_phi(angle - 90, distance);

            // 0 = counterclockwise
            status = incremental_drive(sensor_data, 0, new_angle, distance);
        }
        // 0 - 89
        else if (angle < 90)
        {
            int new_angle = calculate_phi(90 - angle, distance);

            status = incremental_drive(sensor_data, 1, new_angle, distance);
        }
    }
    else {
        int new_angle = calculate_phi(0, distance);

        status = incremental_drive(sensor_data, 1, new_angle, distance);
    }

    return status;
}

#define MIN_EDGE_DIFF 30.0  // cm: difference in IR to detect edge
#define MIN_OBJECT_WIDTH_DEG 4 // degrees: filter out tiny blips

void detect_objects(scan_t *scan_array, object_t *objects, int *object_count) {
    int in_object = 0;
    int start_index = 0;
    *object_count = 0;

    int i;
    for (i = 1; i < 90; i++) {
        double delta = scan_array[i - 1].distance_IR - scan_array[i].distance_IR;
        //require objects be closer than 1.4m
        if (!in_object && delta > MIN_EDGE_DIFF && ((scan_array[i-1].distance_IR < 300) || (scan_array[i].distance_IR < 300))) {
            // Starting edge found
            in_object = 1;
            start_index = i;
        } else if (in_object && delta < -MIN_EDGE_DIFF) {
            // Ending edge found
            in_object = 0;
            int end_index = i;
            int start_angle = scan_array[start_index].angle;
            int end_angle = scan_array[end_index].angle;

            if ((end_angle - start_angle) >= MIN_OBJECT_WIDTH_DEG) {
                // Valid object detected
                int mid_index = (start_index + end_index) / 2;

                object_t obj;
                obj.start_angle = start_angle;
                obj.end_angle = end_angle;
                obj.mid_angle = scan_array[mid_index].angle;
                obj.ping_distance = scan_array[mid_index].distance_PING;

                // Width estimation using trig
                double theta_rad = (end_angle - start_angle) * (M_PI / 180.0);
                obj.width = 2 * obj.ping_distance * tan(theta_rad / 2.0);

                objects[*object_count] = obj;
                (*object_count)++;
            }
        }
    }
}


void clean_data_array(scan_t *scan_array) {
    const short DISTANCE_THRESHOLD = 30;

    int i;
    for (i=1; i<90; i++) {
        float prev = scan_array[i-1].distance_PING;
        float curr = scan_array[i].distance_PING;
        float next = scan_array[i+1].distance_PING;

        if ((prev-curr > DISTANCE_THRESHOLD) && (next-curr > DISTANCE_THRESHOLD)) {
            scan_array[i].distance_PING = ((prev+next) / 2.0);
        }
    }

    for (i=1; i<90; i++) {
        float prev = scan_array[i-1].distance_IR;
        float curr = scan_array[i].distance_PING;
        float next = scan_array[i+1].distance_PING;

        if ((prev-curr > DISTANCE_THRESHOLD) && (next-curr > DISTANCE_THRESHOLD)) {
            scan_array[i].distance_PING = ((prev+next) / 2.0);
        }
    }
    scan_array[0].distance_PING = scan_array[2].distance_PING;
    scan_array[1].distance_PING = scan_array[2].distance_PING;
    scan_array[0].distance_IR = scan_array[2].distance_IR;
    scan_array[1].distance_IR = scan_array[2].distance_IR;

    scan_array[90].distance_PING = scan_array[88].distance_PING;
    scan_array[89].distance_PING = scan_array[88].distance_PING;
    scan_array[90].distance_IR = scan_array[88].distance_IR;
    scan_array[89].distance_IR = scan_array[88].distance_IR;
}

void scan_180(scan_t *scan_array) {

    strcpy(uart_buffer, "\rAngle(Degrees)\tPING(cm)\tIR(cm)\r\n");
    uart_sendStr(uart_buffer);

    servo_move(0);
    timer_waitMillis(1000);

    int index = 0;
    float angle;
    for (angle = 0; angle <= 180; angle += 2) {
        timer_waitMillis(200);
        servo_move(angle);
        timer_waitMillis(200);


        double total_ping = 0.0;
        double total_ir = 0.0;

        // Take 3 scans at the same angle
        int i;
        for (i = 0; i < 2; i++) {

            unsigned long ticks = ping_getPulseWidth();
            float time_ms = ping_getTimeMs(ticks);
            float distance_cm = ping_getDistanceCm(time_ms);
            total_ping += distance_cm;

//            int IR_raw = getScan.IR_raw_val;
//            double predicted_ir_distance = convert_rawIR_to_distance(IR_raw);

            int res = adc_read_pb4();
            double predicted_ir_distance = adc_to_distance(res);
            total_ir += predicted_ir_distance;
        }

        // Calculate averages
        double avg_ping = total_ping / 2.0;
        double avg_ir = total_ir / 2.0;

        // Store results
        scan_array[index].angle = angle;
        scan_array[index].distance_PING = avg_ping;
        scan_array[index].distance_IR = avg_ir;

        // Print to UART
        sprintf(uart_buffer, "%.1f\t\t%.2f\t\t%.2f\r\n", angle, avg_ping, avg_ir);
        uart_sendStr(uart_buffer);

        index++;
    }
}


navigate_to_smallest_width_object(oi_t *sensor_data, scan_t *scan_array) {
    int status;
    object_t objects[20];
    int object_count;

    scan_180(scan_array);

    // Step 2: Clean IR data
    clean_data_array(scan_array);

    // Step 3: Detect objects
    detect_objects(scan_array, objects, &object_count);

    // Print all detected objects
    printf("Detected %d objects:\n", object_count);
    sprintf(uart_buffer, "Detected %d objects:\r\n", object_count);
    uart_sendStr(uart_buffer);
    printf("Index\tStart\tEnd\tMid\tPING(cm)\tWidth(cm)\n");
//    sprintf(uart_buffer, ("Detected %d objects:\n", object_count);
    uart_sendStr("Index\tStart\tEnd\tMid\tPING(cm)\tWidth(cm)\r\n");

    int min_idx = 0;
    int i;
    for (i = 0; i < object_count; i++) {
        object_t obj = objects[i];
        if (obj.width < objects[min_idx].width) {
            min_idx = i;
        }
        sprintf(uart_buffer, "%d\t%d\t%d\t%d\t%.2f\t\t%.2f\r\n", i + 1, obj.start_angle,
                obj.end_angle,
                obj.mid_angle,
                obj.ping_distance,
                obj.width);
        uart_sendStr(uart_buffer);
    }
    // if there were any objects found
    if (object_count) {
        object_t smallest = objects[min_idx];
        status = drive_to_smallest_object(sensor_data, smallest.mid_angle, smallest.ping_distance);
    }
    return status;
}

void remote_navigate(oi_t *sensor_data) {


    lcd_printf("Running");

    int moving = 0;
    char input;

    while (1) {
        lcd_printf("Waiting...");
        timer_waitMillis(65);
        // Try to get input without blocking
        if (cyBot_getByte_nb(&input)) {
            // Echo command back
            sprintf(uart_buffer, "\r\n[%c] ", input);
            uart_sendStr(uart_buffer);
            moving = 0; // Reset moving flag each time a new input comes in

            switch (input) {
                case 't': // toggles between modes (0=manual, 1=autonomous)
                    oi_setWheels(0,0);
                    if (mode == 0) {
                        mode = 1;
                        uart_sendStr("Autonomous mode enabled\r");
                    } else {
                        mode = 0;
                        uart_sendStr("Manual mode enabled\r");

                    }
                    break;
                case 'c': // calibrate servo
                    servo_calibrate();
                    break;
                case 'w':
                    if (!mode) {
                        oi_setWheels(200, 200); // forward
                        moving = 1;
                    }
                    else {
                        uart_sendStr("Autonomous mode enabled!\r");
                    }
                    break;
                case 's':
                    if (!mode) {

                    oi_setWheels(-200, -200); // backward
                    moving = 1;
                    }
                    else {
                        uart_sendStr("Autonomous mode enabled!\r");
                    }
                    break;
                case 'a':
                    if (!mode) {

                        oi_setWheels(150, -150); // turn left
                        moving = 1;
                    }
                    else {
                        uart_sendStr("Autonomous mode enabled!\r");
                    }
                    break;
                case 'd':
                    if (!mode) {
                        oi_setWheels(-150, 150); // turn right
                        moving = 1;
                    }
                    else {
                        uart_sendStr("Autonomous mode enabled!\r");
                    }
                    break;
                case 'm':
                    if (!mode) { // if in manual mode, fall through
                        oi_setWheels(0, 0);

                        object_t objects[20];
                        int object_count;

                        scan_180(scan_array);

                        clean_data_array(scan_array);

                        // Step 3: Detect objects
                        detect_objects(scan_array, objects, &object_count);

                        // Print all detected objects
                        printf("Detected %d objects:\n", object_count);
                        sprintf(uart_buffer, "Detected %d objects:\r\n", object_count);
                        uart_sendStr(uart_buffer);
                        printf("Index\tStart\tEnd\tMid\tPING(cm)\tWidth(cm)\n");
                    //    sprintf(uart_buffer, ("Detected %d objects:\n", object_count);
                        uart_sendStr("Index\tStart\tEnd\tMid\tPING(cm)\tWidth(cm)\r\n");

                        int min_idx = 0;
                        int i;
                        for (i = 0; i < object_count; i++) {
                            object_t obj = objects[i];
                            if (obj.width < objects[min_idx].width) {
                                min_idx = i;
                            }
                            sprintf(uart_buffer, "%d\t%d\t%d\t%d\t%.2f\t\t%.2f\r\n", i + 1, obj.start_angle,
                                    obj.end_angle,
                                    obj.mid_angle,
                                    obj.ping_distance,
                                    obj.width);
                            uart_sendStr(uart_buffer);
                        }
                    }
                    else { //if autonomous mode enabled
                        while (1) {
                            int status = navigate_to_smallest_width_object(sensor_data, scan_array);
                            if (!status) continue;
                            else break;
                        }
                    }
                    break;
                case 'n':
                    oi_setWheels(0, 0);
                    servo_move(90); // scan in front while calibrating



//                    int scan = getScan.IR_raw_val;
//                    volatile double predicted_distance;
//                    predicted_distance = convert_rawIR_to_distance(scan);

                    int res = adc_read_pb4();
                    double predicted_distance = adc_to_distance(res);
                    sprintf(uart_buffer, "distance: %f\r\n", predicted_distance);
                    uart_sendStr(uart_buffer);
                    break;
                case 'o':
                    avoid_obstacle(sensor_data, LEFT_BUMPED);

                    break;
                case 'q':
                    oi_setWheels(0, 0);
                    uart_sendChar('\n');
                    uart_sendStr("Quitting\n");
                    oi_free(sensor_data);
                    return;
                default:
                    break;
            }
            if (!moving) {
                oi_setWheels(0, 0); // stop if not moving forward or backward
            }

//            uart_sendChar('\r\n');
        } else {
            // No new input; optionally, stop wheels or continue current action
            // For example, to stop when no input:
            oi_setWheels(0, 0);
        }

        timer_waitMillis(10);  // Small delay to avoid busy loop
    }
}


int main(void)
{
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    timer_init();       // Initialize Timer
    lcd_init();         // Initialize LCD
    uart_init(115200);  // Initialize UART
    ping_init();
    servo_init();
    button_init();
    adc_init_pb4();     // Initialize ADC for IR

    // main function to control manual driving and autonomous mode
    remote_navigate(sensor_data);

    oi_free(sensor_data);
	return 0;
}
