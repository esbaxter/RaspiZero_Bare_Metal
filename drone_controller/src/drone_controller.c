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

My attempt at creating a robotic drone that will follow a predefined path.  This 
is definitely a work in progress...

*/

#include <stdio.h>
#include "common.h"
#include "altitude_package.h"
#include "log.h"

#include "mpu6050.h"
#include "aux_peripherals.h"

int __errno = 0;

int drone_control()
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
		log_indicate_system_error();
	}
	
	log_string("Altitude test ready\n\r");
	while (1)
	{
		//int32_t delta_meter;
		MPU6050_Accel_Gyro_Values mpu_values; 
		//altitude_get_delta(&delta_meter);
		status = mpu6050_retrieve_values(&mpu_values);
		if (status == RPi_Success)
		{
			log_string_plus("Accel X: ", (uint32_t)mpu_values.accel_x);
			log_string_plus("Accel Y: ", (uint32_t)mpu_values.accel_y);
			log_string_plus("Accel Z: ", (uint32_t)mpu_values.accel_z);
			log_string_plus("Gyro X: ", (uint32_t)mpu_values.gyro_x);
			log_string_plus("Gyro Y: ", (uint32_t)mpu_values.gyro_y);
			log_string_plus("Gyro Z: ", (uint32_t)mpu_values.gyro_z);			
		}
		else if (status == MPU6050_Data_Overflow)
		{
			log_string("MPU data overflow, aborting...");
			break;
		}

		char tty_char = log_getchar();
		if (tty_char == 'd')
		{
			log_string("See ya!");
			break;
		}
	}
	mpu6050_reset();
	log_dump_buffer();
    return(0);
}
