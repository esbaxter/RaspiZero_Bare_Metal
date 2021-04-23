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

File:  spi.c

Implementation of the SPI peripheral interface using the Broadcom 2835 peripheral 
function.

*/

#include "spi.h"
#include "log.h"
#include "reg_definitions.h"
#include "gpio.h"

#define SPI_INIT_BIT 1
#define SPI_CLOCK 2500
#define DEADMAN_TIMEOUT 1000000

/* Bitfields in CS */
#define SPI_CS_TX_READY_BIT 18
#define SPI_CS_RX_DATA_BIT 	17
#define SPI_CS_CMD_DONE_BIT 16
#define SPI_CS_TA_BIT		7
#define SPI_CS_CSPOL_BIT	6
#define SPI_CS_CLEAR_RX_BIT	5
#define SPI_CS_CLEAR_TX_BIT	4
#define SPI_CS_CPOL_BIT		3
#define SPI_CS_CPHA_BIT		2

/*  These registers refer to the SPI controller in section 10 
of the BCM2835 ARM Peripherals documentation */

typedef struct {
	uint32_t spi_command_status;
	uint32_t spi_FIFOs;
	uint32_t spi_clock_divider;
	uint32_t spi_data_length;
	uint32_t spi_lossi_TOH;
	uint32_t spi_DMA_control;
} SPI_Registers;

static unsigned char spi_ready = 0;
static volatile SPI_Registers *spi_registers = (SPI_Registers *)SPI0_BASE;

void spi_dump_registers()
{
	log_string_plus("SPI Command Status: ", spi_registers->spi_command_status);
	log_string_plus("SPI Clock Divider: ", spi_registers->spi_clock_divider);
	log_string_plus("SPI Data Length: ", spi_registers->spi_data_length);
	log_string_plus("SPI LOSSI TOH: ", spi_registers->spi_lossi_TOH);
	log_string_plus("SPI DMA Control: ", spi_registers->spi_DMA_control);
}

Error_Returns spi_init()
{
	Error_Returns to_return = RPi_Success;
	if (!spi_ready)
	{
		do
		{
			to_return = gpio_set_function_select(gpio_pin_7, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("spi_init: failed to set up pin ", gpio_pin_7);
				break;
			}
			
			to_return = gpio_set_function_select(gpio_pin_8, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("spi_init: failed to set up pin ", gpio_pin_8);
				break;
			}

			to_return = gpio_set_function_select(gpio_pin_9, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("spi_init: failed to set up pin ", gpio_pin_9);
				break;
			}

			to_return = gpio_set_function_select(gpio_pin_10, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("spi_init: failed to set up pin ", gpio_pin_10);
				break;
			}

			to_return = gpio_set_function_select(gpio_pin_11, gpio_alt_0);
			if (to_return != RPi_Success)
			{
				log_string_plus("spi_init: failed to set up pin ", gpio_pin_11);
				break;
			}

			gpio_set_pullup_pulldown(gpio_pin_7, pupd_disable);
			gpio_set_pullup_pulldown(gpio_pin_8, pupd_disable);
			gpio_set_pullup_pulldown(gpio_pin_9, pupd_disable);
			gpio_set_pullup_pulldown(gpio_pin_10, pupd_disable);
			gpio_set_pullup_pulldown(gpio_pin_11, pupd_disable);

			spi_registers->spi_command_status = ( 1 << SPI_CS_CLEAR_RX_BIT | 1 << SPI_CS_CLEAR_TX_BIT);
			spi_registers->spi_clock_divider = SPI_CLOCK;
			spi_ready = 1;
		} while(0);
	}
	return to_return;
}

Error_Returns spi_read(SPI_CE chip_enable, SPI_POL clock_polarity, SPI_CPHA clock_phase,
   SPI_POL chip_select_polarity, unsigned char *command_buffer, uint32_t command_bytes)
{
	uint32_t deadman = 0;
	uint32_t index = 0;
	Error_Returns to_return = RPi_Success;
	
	spi_registers->spi_command_status = (1 << SPI_CS_CLEAR_RX_BIT | 1 << SPI_CS_CLEAR_TX_BIT);
	spi_registers->spi_command_status = (1 << SPI_CS_TA_BIT | chip_select_polarity << SPI_CS_CSPOL_BIT | 
	   clock_polarity << SPI_CS_CPOL_BIT | clock_phase << SPI_CS_CPHA_BIT | chip_enable);
	
	do
	{	
		deadman = 0;
		while (deadman < DEADMAN_TIMEOUT)
		{
			if (spi_registers->spi_command_status & (1 << SPI_CS_TX_READY_BIT)) break;
			deadman++;	
		}
		
		if (deadman >= DEADMAN_TIMEOUT)
		{
			to_return = RPi_Timeout;
			log_string("Error:  Deadman timeout on SPI TX");
			break;  //Jump to clean up and return
		}
		
		if (index < command_bytes)
			spi_registers->spi_FIFOs = command_buffer[index];
		else
			spi_registers->spi_FIFOs = 0;
				
		deadman = 0;
		while(deadman < DEADMAN_TIMEOUT) 
		{
			if(spi_registers->spi_command_status & (1 << SPI_CS_RX_DATA_BIT)) break;
			deadman++;
		}
		
		if (deadman >= DEADMAN_TIMEOUT)
		{
			to_return = RPi_Timeout;
			log_string("Error:  Deadman timeout on SPI RX");
			break;  //Jump to clean up and return
		}
		
		if (index == 0)  //Dump the first word out of the read FIFO, it is garbage
		{
			command_buffer[index] = spi_registers->spi_FIFOs;
		}
		else
		{
			command_buffer[index - 1] = spi_registers->spi_FIFOs;
		}
		index++;
	} while(index < command_bytes + 1);
	
	if (to_return == RPi_Success)
	{
		deadman = 0;
		while (deadman < DEADMAN_TIMEOUT)
		{
			if (spi_registers->spi_command_status & (1 << SPI_CS_CMD_DONE_BIT)) break;
			deadman++;	
		}		
		if (deadman >= DEADMAN_TIMEOUT)
		{
			to_return = RPi_Timeout;
			log_string("Error:  Deadman timeout on SPI wait for DONE");
		}
	}
	
	spi_registers->spi_command_status = (1 << SPI_CS_CLEAR_RX_BIT | 1 << SPI_CS_CLEAR_TX_BIT);
	return to_return;
}

Error_Returns spi_write(SPI_CE chip_enable, SPI_POL clock_polarity, SPI_CPHA clock_phase,
   SPI_POL chip_select_polarity, unsigned char *command_buffer, uint32_t command_bytes)
{
	uint32_t deadman = 0;
	uint32_t index = 0;
	Error_Returns to_return = RPi_Success;
	
	spi_registers->spi_command_status = (1 << SPI_CS_CLEAR_RX_BIT | 1 << SPI_CS_CLEAR_TX_BIT);
	spi_registers->spi_command_status = (1 << SPI_CS_TA_BIT | chip_select_polarity << SPI_CS_CSPOL_BIT | 
	   clock_polarity << SPI_CS_CPOL_BIT | clock_phase << SPI_CS_CPHA_BIT | chip_enable);
	
	do
	{	
		deadman = 0;
		while (deadman < DEADMAN_TIMEOUT)
		{
			if (spi_registers->spi_command_status & (1 << SPI_CS_TX_READY_BIT)) break;
			deadman++;	
		}
		
		if (deadman >= DEADMAN_TIMEOUT)
		{
			to_return = RPi_Timeout;
			log_string("Error:  Deadman timeout on SPI TX");
			break;  //Jump to clean up and return
		}
		
		spi_registers->spi_FIFOs = command_buffer[index++];
	} while(index < command_bytes);
	
	if (to_return == RPi_Success)
	{
		deadman = 0;
		while (deadman < DEADMAN_TIMEOUT)
		{
			if (spi_registers->spi_command_status & (1 << SPI_CS_CMD_DONE_BIT)) break;
			deadman++;	
		}		
		if (deadman >= DEADMAN_TIMEOUT)
		{
			to_return = RPi_Timeout;
			log_string("Error:  Deadman timeout on SPI wait for DONE");
		}
	}

	spi_registers->spi_command_status = (1 << SPI_CS_CLEAR_RX_BIT | 1 << SPI_CS_CLEAR_TX_BIT);
	return to_return;
}