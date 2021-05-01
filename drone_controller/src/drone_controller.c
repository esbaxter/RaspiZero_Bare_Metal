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
#include "arm_timer.h"

#include "mpu6050.h"

int __errno = 0;

void tick_handler(void)
{
	int32_t delta_meter;
	altitude_get_delta(&delta_meter);
	printf("Delta: %d\n\r", (int)delta_meter/256);
}

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
	
	status = arm_timer_init();
	if (status != RPi_Success)
	{
		log_string_plus("timer init failed: ", status);
		log_indicate_system_error();
	}
	
	status = arm_timer_enable(tick_handler, 500);
	if (status != RPi_Success)
	{
		log_string_plus("timer enable failed: ", status);
		log_indicate_system_error();
	}
	
	log_string("Altitude test ready\n\r");
	while (1)
	{
		char tty_char = log_getchar();
		if (tty_char == 'd')
		{
			status = arm_timer_disable();
			if (status != RPi_Success)
				{
					log_string_plus("timer disable failed: ", status);
					log_indicate_system_error();
				}
			log_string("See ya!");
			log_dump_buffer();
			break;
		}
	}
	
    return(0);
}
