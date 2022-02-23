/*Copyright 2021 Eric Baxter <ericwbaxter85@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining 
a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation 
the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software 
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

File:  drone_controller.c

My attempt at creating an autonomous drone that will follow a predefined path.  This
is definitely a work in progress...

*/

#define X_AXIS_SERVO	0
#define Y_AXIS_SERVO	1
#define SERVO_MIN_LIMIT	-90
#define SERVO_MAX_LIMIT 90

#include <stdio.h>
#include "common.h"
#include "altitude_package.h"
#include "log.h"

#include "mpu6050.h"
#include "aux_peripherals.h"
#include "servo_controller.h"
#include "arm_timer.h"

int __errno = 0;

int test_control()
{
    Error_Returns status = RPi_Success;
	
	log_indicate_system_ok();
	
	status = log_init();
	if (status != RPi_Success)
	{
		log_indicate_system_error();
	}

	status = altitude_initialize();
	if (status != RPi_Success)
	{
		log_string_plus("altitude_initialize failed: ", status);
		log_dump_buffer();
		log_indicate_system_error();
	}

	log_string("Altitude test ready\n\r");

	while (1)
	{
		double delta_meter;
		spin_wait_milliseconds(150);
		//MPU6050_Accel_Gyro_Values mpu_values;
		status = altitude_get_delta(&delta_meter);

		if (status != RPi_Success)
		{
			log_string("altitude_get_delta failed!");
			break;
		}

		printf("delta_meter: %f\n\r", delta_meter);
		/*status = mpu6050_retrieve_values(&mpu_values);
		if (status == RPi_Success)
		{
			log_string_plus("Quat w: ", (uint32_t)mpu_values.quat_w);
			log_string_plus("Quat x: ", (uint32_t)mpu_values.quat_x);
			log_string_plus("Quat y: ", (uint32_t)mpu_values.quat_y);
			log_string_plus("Quat z: ", (uint32_t)mpu_values.quat_z);	
		}
		else if (status == MPU6050_Data_Overflow)
		{
			log_string("MPU data overflow, aborting...");
			break;
		}*/

		char tty_char = log_getchar();
		if (tty_char == 'd')
		{
			log_string("See ya!");
			break;
		}
		else if (tty_char == 'r')
		{
			status = altitude_reset();
		}
	}
	
	mpu6050_reset();
	log_dump_buffer();
    return(0);
}
