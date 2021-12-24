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

File:  pca9685.c

Provides support for the NXP PCA9685 PWM controller.

*/

#include <stdio.h>
#include "i2c.h"
#include "log.h"
#include "pca9685.h"

#define PCA9685_MICROSECONDS_PER_SECOND	1000000
#define PCA9685_INTERNAL_CLK_FREQ	25000000
#define PCA9685_FULL_SCALE_VALUE	4096
#define PCA9685_MIN_PRESCALE_VALUE	3
#define PCA9685_MAX_PRESCALE_VALUE	0xFF
#define PCA9685_MAX_CHANNEL_ID		15
#define PCA9685_CONTROL_REGS_PER_CHANNEL	4

#define PCA9685_NUMBER_SUPPORTED_DEVICES 1 //Can be anything up to 62
#define PCA9685_NUMBER_SUPPORTED_SERVOS 2 //Can be anything up to 16

#define PCA9685_SW_RESET_ID 0x06
#define PCA9685_MODE_REG_1_RESET_VALUE 0x11

#define PCA9685_MODE_REGISTER_1 0x00
#define PCA9685_FIRST_SERVO_CONTROL_REG 0x06
#define PCA9685_ALL_LED_OFF_HIGH_REG 0xFD
#define PCA9685_PRESCALE_REGISTER 0xFE

#define PCA9685_MODE_1_REGISTER_SLEEP_BIT 0x04
#define PCA9685_MODE_1_AUTO_INCREMENT_BIT 0x05
#define PCA9685_MODE_1_REGISTER_EXTERNAL_CLK_BIT 0x06

#define PCA9685_ALL_LED_RESET_BIT 0x04

typedef struct PCA9685_Servo_Params {
	uint32_t channel_id;
} PCA9685_Servo_Parameters;

typedef struct PCA9685_Params {
	uint32_t i2c_id;
	uint32_t servo_count;
	uint32_t microseconds_per_tick;
	PCA9685_Servo_Parameters servos[PCA9685_NUMBER_SUPPORTED_SERVOS];
} PCA9685_Parameters;

static PCA9685_Parameters pca_configuration_params[PCA9685_NUMBER_SUPPORTED_DEVICES];

static uint32_t pca9685_count = 0;

static Error_Returns pca9685_read(uint32_t id, unsigned char *buffer, unsigned int rx_bytes)
{
	Error_Returns to_return = RPi_Success;
	to_return = i2c_write(id, buffer, 1);
	if (to_return == RPi_Success)
	{
		to_return = i2c_read(id, buffer, rx_bytes);
	}
	return to_return;
}


static Error_Returns pca9685_write(uint32_t id, unsigned char *buffer, unsigned int tx_bytes)
{
	Error_Returns to_return = RPi_Success;
	to_return = i2c_write(id, buffer, tx_bytes);

	return to_return;
}

//TODO:  Pass parameters for invert/no invert and output driver.

Error_Returns pca9685_init(uint32_t i2c_id, PCA9685_Clock_Source clk_src,
		uint32_t input_clk_frequency, uint32_t output_frequency, uint32_t *pca9685_idx)
{
	Error_Returns to_return = RPi_Success;
	uint32_t pca9685_clk_frequency = 0;
	unsigned char buffer[2];
	do
	{
		if (pca9685_count == PCA9685_NUMBER_SUPPORTED_DEVICES)
		{
			log_string("pca9685_init():  Max devices met.");
			to_return = PCA_9685_Insufficient_Device_Structures;
			break;
		}

		if (pca9685_count == 0)
		{
			for (uint32_t counter = 0; counter < PCA9685_NUMBER_SUPPORTED_DEVICES; counter++)
			{
				//Initialize the i2c address of each possible device to something
				//we can't ever have.
				pca_configuration_params[counter].i2c_id = PCA9685_SW_RESET_ID;
				pca_configuration_params[counter].servo_count = 0;
			}
		}
		to_return = i2c_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Error initializing I2C bus ", to_return);
			break;  //No need to continue just return the failure
		}

		//Cleanly shut down an PWM channels that may be operating to avoid
		//getting a reset bit set in the mode 1 register when we put the chip
		//to sleep.
		//buffer[0] = PCA9685_ALL_LED_OFF_HIGH_REG;
		//buffer[1] = (1<<PCA9685_ALL_LED_RESET_BIT);
		//to_return = pca9685_write(i2c_id, buffer, 2);
		//if (to_return != RPi_Success)
		//{
		//	log_string_plus("pca9685_init():  Failed to write all led off high:  ", to_return);
		//	break;  //No need to continue just return the failure
		//}
		buffer[0] = PCA9685_MODE_REGISTER_1;
		buffer[1] = PCA9685_MODE_REG_1_RESET_VALUE;
		to_return = pca9685_write(i2c_id, buffer, 2);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Failed to write 1 to mode register 1:  ", to_return);
			break;  //No need to continue just return the failure
		}

		buffer[0] = PCA9685_MODE_REGISTER_1;
		to_return = pca9685_read(i2c_id, buffer, 1);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Failed to read mode register 1:  ", to_return);
			break;  //No need to continue just return the failure
		}

		if (!(buffer[0] & (1<<PCA9685_MODE_1_REGISTER_SLEEP_BIT)))
		{
			log_string_plus("pca9685_init():  Failed to write mode register 1 readback =   ", (uint32_t)buffer[0]);
			to_return = PCA_9685_Register_Access_Failure;
			break;  //No need to continue just return the failure
		}

		unsigned char mode_register_1_value = buffer[0];

		//If necessary enable external clock
		if (clk_src == PCA_9685_External_Clock)
		{
			pca9685_clk_frequency = input_clk_frequency;
			mode_register_1_value |= (1<<PCA9685_MODE_1_REGISTER_EXTERNAL_CLK_BIT);
		}
		else
		{
			//Check to see if external clock bit is set.  If it is we have
			//a problem since this is a "sticky" bit and must be reset by
			//a power cycle or software reset.
			if (buffer[1] & (1<<PCA9685_MODE_1_REGISTER_EXTERNAL_CLK_BIT))
			{
				log_string_plus("pca9685_init():  Request for internal clk but external bit set regsiter:  ", (uint32_t)buffer[1]);
				to_return = PCA_9685_Configuration_Error;
				break;  //No need to continue just return the failure
			}
			pca9685_clk_frequency = PCA9685_INTERNAL_CLK_FREQ;
		}

		//Set up the prescale register to get the appropriate cycle frequency
		uint32_t prescale_value = (pca9685_clk_frequency / (PCA9685_FULL_SCALE_VALUE * output_frequency)) - 1;
		if ((prescale_value < PCA9685_MIN_PRESCALE_VALUE) || (prescale_value > PCA9685_MAX_PRESCALE_VALUE))
		{
			log_string_plus("pca9685_init():  Prescale value out of bounds:  ", prescale_value);
			to_return = PCA_9685_Configuration_Error;
			break;
		}
		buffer[0] = PCA9685_PRESCALE_REGISTER;
		buffer[1] = (unsigned char)(prescale_value);
		to_return = pca9685_write(i2c_id, buffer, 2);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Failed to write prescale register:  ", to_return);
			break;  //No need to continue just return the failure
		}

		//Set up to auto increment register accesses
		mode_register_1_value |= (1<<PCA9685_MODE_1_AUTO_INCREMENT_BIT);

		//Enable the chip by bringing it out of sleep mode
		mode_register_1_value &= ~(1<<PCA9685_MODE_1_REGISTER_SLEEP_BIT);

		buffer[0] = PCA9685_MODE_REGISTER_1;
		buffer[1] = mode_register_1_value;
		to_return = pca9685_write(i2c_id, buffer, 2);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Failed to write 2 to mode register 1:  ", to_return);
			break;  //No need to continue just return the failure
		}

		pca_configuration_params[pca9685_count].i2c_id = i2c_id;
		pca_configuration_params[pca9685_count].microseconds_per_tick = PCA9685_MICROSECONDS_PER_SECOND/(PCA9685_FULL_SCALE_VALUE * output_frequency);
		*pca9685_idx = pca9685_count++; //Increment index and return to caller

	} while(0);

	return to_return;
}

Error_Returns pca9685_register_servo(uint32_t pca9685_idx, uint32_t servo_channel,
		uint32_t *servo_idx)
{
	Error_Returns to_return = RPi_Success;
	do
	{
		if (pca9685_idx >= pca9685_count)
		{
			log_string_plus("pca9685_register_servo():  Incorrect chip index:  ", pca9685_idx);
			to_return = PCA_9685_Configuration_Error;
			break;
		}
		if (pca_configuration_params[pca9685_idx].servo_count >= PCA9685_NUMBER_SUPPORTED_SERVOS)
		{
			log_string("pca9685_register_servo():  Insufficient servo structures");
			to_return = PCA_9685_Insufficient_Device_Structures;
			break;
		}
		if (servo_channel > PCA9685_MAX_CHANNEL_ID)
		{
			log_string_plus("pca9685_register_servo():  Channel ID exceeds chip capabilities, ID:  ", servo_channel);
			to_return = PCA_9685_Configuration_Error;
			break;
		}
		pca_configuration_params[pca9685_idx].servos[pca_configuration_params[pca9685_idx].servo_count].channel_id = servo_channel;
		*servo_idx = pca_configuration_params[pca9685_idx].servo_count++;
	} while(0);
	return to_return;
}

//Plus one to cover the initial register address
#define PCA9685_MOVE_SERVO_BUFFER_SIZE PCA9685_CONTROL_REGS_PER_CHANNEL + 1

Error_Returns pca9685_move_servo(uint32_t pca9685_idx, uint32_t servo_idx,
		uint32_t active_pulse_width)
{
	Error_Returns to_return = RPi_Success;

	unsigned char buffer[PCA9685_MOVE_SERVO_BUFFER_SIZE];
	do
	{
		if (pca9685_idx >= pca9685_count)
		{
			log_string_plus("pca9685_move_servo():  Incorrect chip index:  ", pca9685_idx);
			to_return = PCA_9685_Configuration_Error;
			break;
		}

		if (servo_idx >= pca_configuration_params[pca9685_idx].servo_count)
		{
			log_string_plus("pca9685_move_servo():  Incorrect servo index:  ", servo_idx);
			to_return = PCA_9685_Configuration_Error;
			break;
		}

		uint32_t signal_low_ticks = PCA9685_FULL_SCALE_VALUE -
				(active_pulse_width / pca_configuration_params[pca9685_idx].microseconds_per_tick);

		buffer[0] = PCA9685_FIRST_SERVO_CONTROL_REG + (PCA9685_CONTROL_REGS_PER_CHANNEL *
				pca_configuration_params[pca9685_idx].servos[servo_idx].channel_id);

		//Active pulse starts at the beginning of the output cycle
		buffer[1] = 0;
		buffer[2] = 0;
		buffer[3] = signal_low_ticks & 0xFF;
		buffer[4] = (signal_low_ticks >> 8) && 0xFF;
		to_return = pca9685_write(pca_configuration_params[pca9685_idx].i2c_id,
				buffer, PCA9685_MOVE_SERVO_BUFFER_SIZE);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_move_servo():  Set high/low register write failed:  ", to_return);
			break;
		}
	} while(0);
	return to_return;
}
