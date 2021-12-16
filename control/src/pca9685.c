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

#define PCA9685_NUMBER_SUPPORTED_DEVICES 1 //Can be anything up to 62
#define PCA9685_SW_RESET_ID 0x06
#define PCA9685_MODE_REG_1_RESET_VALUE 0x11

#define PCA9685_MODE_REGISTER_1 0x00

#define PCA9685_MODE_1_REGISTER_SLEEP_BIT 0x04

typedef struct PCA9685_Params {
	uint32_t i2c_id;
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

Error_Returns pca9685_init(uint32_t i2c_id, uint32_t *pca9685_idx)
{
	Error_Returns to_return = RPi_Success;
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
			}
		}
		to_return = i2c_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Error initializing I2C bus ", to_return);
			break;  //No need to continue just return the failure
		}

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

		if (buffer[0] != PCA9685_MODE_REG_1_RESET_VALUE)
		{
			log_string_plus("pca9685_init():  Failed to write mode register 1 readback =   ", (uint32_t)buffer[0]);
			to_return = PCA_9685_Register_Access_Failure;
			break;  //No need to continue just return the failure
		}
		//Enable the chip by bringing it out of sleep mode
		buffer[1] = buffer[0];
		buffer[1] &= ~(1<<PCA9685_MODE_1_REGISTER_SLEEP_BIT);
		buffer[0] = PCA9685_MODE_REGISTER_1;
		to_return = pca9685_write(i2c_id, buffer, 2);
		if (to_return != RPi_Success)
		{
			log_string_plus("pca9685_init():  Failed to write 2 to mode register 1:  ", to_return);
			break;  //No need to continue just return the failure
		}

		pca_configuration_params[pca9685_count].i2c_id = i2c_id;
		*pca9685_idx = pca9685_count++; //Increment index and return to caller

	} while(0);

	return to_return;
}
