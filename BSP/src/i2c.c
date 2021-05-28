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

File:  i2c.c

Implementation of the I2C peripheral interface protocol using the Broadcom
Serial Controller in the 2835.

*/

#include "i2c.h"
#include "log.h"
#include "gpio.h"
#include "reg_definitions.h"

#define BSC_CONTROL_I2CEN		(1 << 15)
#define BSC_CONTROL_INTR		(1 << 10)
#define BSC_CONTROL_INTT		(1 << 9)
#define BSC_CONTROL_INTD		(1 << 8)
#define BSC_CONTROL_ST			(1 << 7)
#define BSC_CONTROL_CLEAR		(1 << 5) | (1 << 4)
#define BSC_CONTROL_READ		(1 << 0)
#define BSC_CONTROL_RESET		0

#define BSC_STATUS_CLKT			(1 << 9)
#define BSC_STATUS_ERR			(1 << 8)
#define BSC_STATUS_RXF			(1 << 7)
#define BSC_STATUS_TXE			(1 << 6)
#define BSC_STATUS_RXD			(1 << 5)
#define BSC_STATUS_TXD			(1 << 4)
#define BSC_STATUS_RXR			(1 << 3)
#define BSC_STATUS_TXW			(1 << 2)
#define BSC_STATUS_DONE			(1 << 1)
#define BSC_STATUS_TA			(1 << 0)

#define BSC_BYTE_MASK			0xFF

#define BASE_CLOCK_SPEED 		150000000

#define I2C_SPEED 				100000  //This is fairly slow due to the experimental nature of my setup

typedef struct {
	uint32_t bsc_control;
	uint32_t bsc_status;
	uint32_t bsc_data_length;
	uint32_t bsc_slave_address;
	uint32_t bsc_data_FIFO;
	uint32_t bsc_clock_divider;
	uint32_t bsc_data_delay;
	uint32_t bsc_clock_stretch;
} BSC_Registers;

static unsigned char i2c_ready = 0;
static volatile BSC_Registers *bsc1_registers = (BSC_Registers *)BSC1_BASE;

void i2c_dump_registers()
{
	log_string_plus("BSC Control: ", bsc1_registers->bsc_control);
	log_string_plus("BSC Status: ", bsc1_registers->bsc_status);
	log_string_plus("BSC Data Length: ", bsc1_registers->bsc_data_length);
	log_string_plus("BSC Slave Address: ", bsc1_registers->bsc_slave_address);
	log_string_plus("BSC Clock Divider: ", bsc1_registers->bsc_clock_divider);
	log_string_plus("BSC Data Delay: ", bsc1_registers->bsc_data_delay);
	log_string_plus("BSC Clock Stretch: ", bsc1_registers->bsc_clock_stretch);
}

Error_Returns i2c_init() 
{
	Error_Returns to_return = RPi_Success;
	if (!i2c_ready)
	{
		do
		{
			to_return = gpio_set_function_select(gpio_pin_2, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("i2c_init:  failed to set up pin ", gpio_pin_2);
				break;
			}
			
			to_return = gpio_set_function_select(gpio_pin_3, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("i2c_init:  failed to set up pin ", gpio_pin_3);
				break;
			}
			
			gpio_set_pullup_pulldown(gpio_pin_2, pupd_disable);
			gpio_set_pullup_pulldown(gpio_pin_3, pupd_disable);

			bsc1_registers->bsc_clock_divider = (BASE_CLOCK_SPEED / I2C_SPEED);
			i2c_ready = 1;
		} while(0);
	}
	return to_return;
}

Error_Returns i2c_read(uint32_t slave_address, unsigned char *data, 
   uint32_t number_bytes)
{
	uint32_t count = 0;
	Error_Returns to_return = RPi_Success;
	
	bsc1_registers->bsc_slave_address = slave_address;
	bsc1_registers->bsc_control = BSC_CONTROL_CLEAR;
	bsc1_registers->bsc_status = (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE);
	bsc1_registers->bsc_data_length = number_bytes;
	bsc1_registers->bsc_control = (BSC_CONTROL_I2CEN | BSC_CONTROL_ST | BSC_CONTROL_READ);
	
	do
	{
		while(!(bsc1_registers->bsc_status & (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE)))
		{
			while(bsc1_registers->bsc_status & (BSC_STATUS_RXD))
			{
				*data++ = bsc1_registers->bsc_data_FIFO & BSC_BYTE_MASK;
				count++;
			}
		}
		
		if (bsc1_registers->bsc_status & (BSC_STATUS_CLKT | BSC_STATUS_ERR))
		{
			if (bsc1_registers->bsc_status & BSC_STATUS_CLKT)
			{
				to_return = I2CS_Clock_Timeout;
			}
			else
			{
				to_return = I2CS_Ack_Error;
			}
			break;
		}
		
		while((count < number_bytes) && (bsc1_registers->bsc_status & BSC_STATUS_RXD))
		{
			*data++ = bsc1_registers->bsc_data_FIFO & BSC_BYTE_MASK;
			count++;
		}
		
		if (count != number_bytes)
		{
			to_return = I2CS_Data_Loss;
		}
	} while (0);
	
	bsc1_registers->bsc_status = (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE);
	bsc1_registers->bsc_control = BSC_CONTROL_RESET;
	return to_return;
}

Error_Returns i2c_write(uint32_t slave_address, unsigned char *data, 
   uint32_t number_bytes)
{
	uint32_t count = 0;
	Error_Returns to_return = RPi_Success;
	
	bsc1_registers->bsc_slave_address = slave_address;
	bsc1_registers->bsc_control = BSC_CONTROL_CLEAR;
	bsc1_registers->bsc_status = (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE);
	bsc1_registers->bsc_data_length = number_bytes;
	bsc1_registers->bsc_control = (BSC_CONTROL_I2CEN | BSC_CONTROL_ST);

	do
	{
		while(!(bsc1_registers->bsc_status & (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE)))
		{
			while(count < number_bytes && bsc1_registers->bsc_status & BSC_STATUS_TXD)
			{
				bsc1_registers->bsc_data_FIFO = *data++;
				count++;
			}
		}
		
		if (bsc1_registers->bsc_status & (BSC_STATUS_CLKT | BSC_STATUS_ERR))
		{
			if (bsc1_registers->bsc_status & BSC_STATUS_CLKT)
			{
				to_return = I2CS_Clock_Timeout;
			}
			else
			{
				to_return = I2CS_Ack_Error;
			}
			break;
		}
		
		if (count != number_bytes)
		{
			to_return = I2CS_Data_Loss;
		}	
	} while(0);
	
	bsc1_registers->bsc_status = (BSC_STATUS_CLKT | BSC_STATUS_ERR | BSC_STATUS_DONE);
	bsc1_registers->bsc_control = BSC_CONTROL_RESET;
	return to_return;
}