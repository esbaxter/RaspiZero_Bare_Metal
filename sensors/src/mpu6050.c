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

File:  mpu6050.c

Implements support for the InvenSense MPU 6050 three axis gyro and accelerometer.



*/

#include <stdio.h>
#include "mpu6050.h"
#include "i2c.h"
#include "gpio.h"
#include "arm_timer.h"
#include "interrupt_handler.h"
#include "log.h"

#define I2C_SLAVE_ADDRESS 0x68
#define MPU6050_WHO_AM_I 0x68

#define MPU6050_WHO_AM_I_REG 0x75

#define TIME_DELAY 2000000
#define MPU_INTERRUPT_GPIO_PIN gpio_pin_4

int32_t interrupt_handler_index;

InterruptHandlerStatus mpu6050_interrupt_handler(void)
{
	InterruptHandlerStatus to_return = Interrupt_Not_Claimed;
	if (gpio_get_event_detect_status(MPU_INTERRUPT_GPIO_PIN) == event_detected)
	{
		//Clear MPU interrupt, then clear event status
		gpio_clear_event_detect_status(MPU_INTERRUPT_GPIO_PIN);
		to_return = Interrupt_Claimed;
	}
	return to_return;
}

Error_Returns mpu6050_write(unsigned char *buffer, unsigned int tx_bytes)
{
	Error_Returns to_return = RPi_Success;
	to_return = i2c_write(I2C_SLAVE_ADDRESS, buffer, tx_bytes);
	return to_return;
}

static Error_Returns mpu_read(unsigned char *buffer, unsigned int rx_bytes)
{
	Error_Returns to_return = RPi_Success;
	to_return = i2c_write(I2C_SLAVE_ADDRESS, buffer, 1);
	if (to_return == RPi_Success)
	{
		to_return = i2c_read(I2C_SLAVE_ADDRESS, buffer, rx_bytes);
	}
	return to_return;
}

Error_Returns mpu6050_init()
{	
	Error_Returns to_return = RPi_Success;
	unsigned char buffer[2];
	interrupt_handler_index = -1;

	do
	{	
		to_return = i2c_init();
		if (to_return != RPi_Success) 
		{
			log_string_plus("mpu6050_init():  Error initializing I2C bus ", to_return);
			break;  //No need to continue just return the failure
		}
		
		to_return = gpio_init();
		if (to_return != RPi_Success) 
		{
			log_string_plus("mpu6050_init():  Error initializing GPIO ", to_return);
			break;  //No need to continue just return the failure
		}
		
		buffer[0] = MPU6050_WHO_AM_I_REG;
		to_return = mpu_read(buffer, 1);
		if (to_return != RPi_Success) 
		{
			log_string_plus("mpu6050_init():  Error reading chip ID read was ", to_return);
			break;  //No need to continue just return the failure
		}
		if (buffer[0] != MPU6050_WHO_AM_I)
		{
			log_string_plus("mpu6050_init():  Error chip ID read was ", buffer[0]);
		}
		
		/* 	Configure the GPIO pin as an input for the MPU interrupt with no PUPD.
			The interrupt pin on the MPU will be set as push-pull, active high,
			latched and only cleared when the interrupt status register is read.
			Enable FIFO overflow and data ready interrupts
		*/
		to_return = gpio_set_function_select(MPU_INTERRUPT_GPIO_PIN, gpio_input);
		if (to_return != RPi_Success) 
		{
			log_string_plus("mpu6050_init():  Error selecting interrupt pin ", to_return);
			break;  //No need to continue just return the failure
		}
		gpio_set_pullup_pulldown(MPU_INTERRUPT_GPIO_PIN, pupd_disable);
		to_return = gpio_set_high_detect_pin(MPU_INTERRUPT_GPIO_PIN);
		if (to_return != RPi_Success) 
		{
			log_string_plus("mpu6050_init():  Error setting high detect pin ", to_return);
			break;  //No need to continue just return the failure
		}
		interrupt_handler_index = interrupt_handler_add(mpu6050_interrupt_handler, Int_GPIO_Pin, MPU_INTERRUPT_GPIO_PIN);
		if (interrupt_handler_index < 0)
		{
			log_string("mpu6050_init:  failed to add interrupt handler");
			to_return = RPi_OperationFailed;
			break;
		}
		
	} while(0);
	return to_return;
}

Error_Returns mpu6050_reset()
{
	Error_Returns to_return = RPi_Success;

	do
	{	
		i2c_init();
		spin_wait(TIME_DELAY); 				 
	} while(0);
		
	return to_return;
}
