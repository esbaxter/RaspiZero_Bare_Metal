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

File:  servo_controller.c

Module that keeps track of and controls servos, currently only supports
a single pca9685 controller.

*/

#include "servo_controller.h"
#include "log.h"
#include <stdio.h>

#define PCA9685_ID 0x40
#define MAX_SERVOS_SUPPORTED 2

#define SERVO_FULL_SWING_PULSE_WIDTH 500 //In microseconds
#define SERVO_CENTER_PULSE_WIDTH	1500 //In microseconds

typedef struct Servo_Params
{
	uint32_t servo_idx;
	uint32_t servo_channel;
	int servo_min_degrees; //In degrees from center
	int servo_max_degrees; //In degrees from center
} Servo_Parameters;

static uint32_t pca_idx = 0;
static uint32_t servo_count = 0;

static Servo_Parameters servos[MAX_SERVOS_SUPPORTED];

Error_Returns servo_controller_init()
{
	Error_Returns to_return = RPi_Success;
	to_return = pca9685_init(PCA9685_ID, PCA_9685_Internal_Clock,
			0, 50, &pca_idx);
	if (to_return != RPi_Success)
	{
		log_string_plus("servo_controller_init:  pca9685_init failed: ", to_return);
		log_dump_buffer();
	}
	return to_return;
}

Error_Returns servo_controller_create_servo(uint32_t servo_channel, int min_degrees, int max_degrees,
		uint32_t *servo_idx)
{
	Error_Returns to_return = RPi_Success;
	do
	{
		if (servo_count == MAX_SERVOS_SUPPORTED)
		{
			log_string("servo_controller_create_servo():  Max devices met.");
			to_return = RPi_InsufficientResources;
			break;
		}


		for(uint32_t index = 0; index < servo_count; index++)
		{
			if (servos[index].servo_channel == servo_channel)
			{
				log_string_plus("servo_controller_create_servo: servo channel already in use: ", servo_channel);
				to_return = RPi_InvalidParam;
				break;
			}
		}

		if (to_return != RPi_Success)
		{
			break;
		}

		to_return = pca9685_register_servo(pca_idx, servo_channel, &servos[servo_count].servo_idx);
		if (to_return != RPi_Success)
		{
			log_string_plus("servo_controller_create_servo: failed to register servo: ", to_return);
			break;
		}

		servos[servo_count].servo_channel = servo_channel;
		servos[servo_count].servo_min_degrees = min_degrees;
		servos[servo_count].servo_max_degrees = max_degrees;
		*servo_idx = servo_count++;
	} while(0);
	return to_return;
}

Error_Returns servo_controller_set_servo(uint32_t servo_idx, int position)
{
	Error_Returns to_return = RPi_Success;
	uint32_t pulse_width = SERVO_CENTER_PULSE_WIDTH;  //Set pulse width to center, will adjust later if needed

	do
	{
		if (servo_idx >= servo_count)
		{
			log_string_plus("servo_controller_set_servo():  Invalid servo ID.", servo_idx);
			to_return = RPi_InvalidParam;
			break;
		}

		if ((position < servos[servo_idx].servo_min_degrees) ||
				(position > servos[servo_idx].servo_max_degrees))
		{
			log_string_plus("servo_controller_set_servo():  Invalid servo position.", (uint32_t)position);
			to_return = RPi_InvalidParam;
			break;
		}

		if (position < 0) //Move servo to a position counter clockwise of center
		{
			pulse_width -= position * SERVO_FULL_SWING_PULSE_WIDTH / servos[servo_idx].servo_min_degrees;
		}

		else if (position > 0) //Move servo to a position clockwise of center
		{
			pulse_width += position * SERVO_FULL_SWING_PULSE_WIDTH / servos[servo_idx].servo_max_degrees;
		}

		to_return = pca9685_move_servo(pca_idx, servos[servo_idx].servo_idx, pulse_width);
		if (to_return != RPi_Success)
		{
			log_string_plus("servo_controller_set_servo: failed to move servo: ", servo_idx);
		}
	} while(0);
	return to_return;
}
