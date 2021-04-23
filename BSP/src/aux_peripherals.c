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

File:  aux_peripherals.c

Implementation of auxillary functions on the Broadcom 2835, currently only the 
mini-UART is supported.

This does support both sending and receiving to/from the UART.  Receiving is 
handled by an interrupt handler and the received character is stored to a buffer
relieving a client from having to poll for characters.

*/

#include "reg_definitions.h"
#include "gpio.h"
#include "aux_peripherals.h"
#include "interrupt_handler.h"
#include "log.h"

//Read https://elinux.org/BCM2835_datasheet_errata#p10 through p19 to clear up much of the
//confusion in the mini-uart section of the BCM2835 ARM Peripherals document

#define UART_RX_BUFFER_SIZE	8

#define ENABLE_UART 0x01
#define DISABLE_TX_RX 0 
#define LCR_ENABLE_EIGHT_BIT 0x03 //Per errata information set bits 0 and 1 to get eight bit operation
#define MCR_SET_RTS_LOW 0
#define IER_ENABLE_RX_INTERRUPT 0x02
#define IIR__CLEAR_FIFOS  0x06
#define BAUD_RATE 0x10E  //Set up for 115200
#define CNTL_TX_RX_ENABLE 0x03
#define UART_TX_IDLE 0x20
#define AUX_UART_IRQ_ACTIVE 0x01
#define UART_READ_INTERRUPT 0x02
#define UART_DATA_RX_READY 0x01
#define UART_RX_MASK 0xFF

typedef struct {
	uint32_t aux_irq;
	uint32_t aux_enables;
	uint32_t reserve1[14];
	uint32_t aux_mu_io_reg;
	uint32_t aux_mu_ier_reg;
	uint32_t aux_mu_iir_reg;
	uint32_t aux_mu_lcr_reg;
	uint32_t aux_mu_mcr_reg;
	uint32_t aux_mu_lsr_reg;
	uint32_t aux_mu_msr_reg;
	uint32_t aux_mu_scratch;
	uint32_t aux_mu_cntl_reg;
	uint32_t aux_mu_stat_reg;
	uint32_t aux_mu_baud_reg;
	uint32_t reserve2[5];
	uint32_t aux_spi0_cntl0_reg;
	uint32_t aux_spi0_cntl1_reg;
	uint32_t aux_spi0_stat_reg;
	uint32_t aux_spi0_io_reg;
	uint32_t aux_spi0_peek_reg;
	uint32_t reserve3[10];
	uint32_t aux_spi1_cntl0_reg;
	uint32_t aux_spi1_cntl1_reg;
	uint32_t aux_spi1_stat_reg;
	uint32_t aux_spi1_io_reg;
	uint32_t aux_spi1_peek_reg;	
} Aux_Peripherals_Registers;

static unsigned char uart_ready = 0;
static volatile uint32_t write_index = 0;
static volatile uint32_t read_index = 0;

static volatile Aux_Peripherals_Registers *aux_perihperals_registers = (Aux_Peripherals_Registers *)AUX_BASE;

static char rx_buffer[UART_RX_BUFFER_SIZE] = {0};


/*  Receive a character and stuff it into the RX buffer.  This isn't set up
	to handle intensive transfers, just the occasional typed command character.
	This is a circular buffer so characters can be lost.
*/

InterruptHandlerStatus uart_char_interrupt_handler(void)
{
	InterruptHandlerStatus to_return = Interrupt_Not_Claimed;
	if (uart_ready)
	{
		if ((aux_perihperals_registers->aux_irq & AUX_UART_IRQ_ACTIVE) &&
			(aux_perihperals_registers->aux_mu_ier_reg & UART_READ_INTERRUPT))
		{
			while (aux_perihperals_registers->aux_mu_lsr_reg & UART_DATA_RX_READY)
			{
				rx_buffer[write_index] = aux_perihperals_registers->aux_mu_io_reg & UART_RX_MASK;
				write_index++;
				write_index = write_index % UART_RX_BUFFER_SIZE;
			}
			to_return = Interrupt_Claimed;
		}
	}
	return to_return;
}

/*  Set up the mini UART for 8 bit, no parity and to
	interrupt on character receive.
*/

Error_Returns uart_init()
{
	Error_Returns to_return = RPi_Success;
	if (uart_ready == 0)
	{
		do
		{
			to_return = interrupt_handler_init();
			
			if (to_return != RPi_Success)
			{
				log_indicate_system_error();
			}
			
			to_return = interrupt_handler_add(uart_char_interrupt_handler);
			
			if (to_return != RPi_Success)
			{
				log_indicate_system_error();
			}
			
			aux_perihperals_registers->aux_enables = ENABLE_UART;
			aux_perihperals_registers->aux_mu_ier_reg = IER_ENABLE_RX_INTERRUPT; 
			aux_perihperals_registers->aux_mu_cntl_reg = DISABLE_TX_RX;
			aux_perihperals_registers->aux_mu_lcr_reg = LCR_ENABLE_EIGHT_BIT; 
			aux_perihperals_registers->aux_mu_mcr_reg = MCR_SET_RTS_LOW; 
			aux_perihperals_registers->aux_mu_iir_reg = IIR__CLEAR_FIFOS;
			aux_perihperals_registers->aux_mu_baud_reg = BAUD_RATE;
			to_return = gpio_set_function_select(gpio_pin_14, gpio_alt_5);
			if (to_return != RPi_Success)
			{
				log_string_plus("uart_init:  failed to set up pin ", gpio_pin_14);
				break;
			}
			to_return = gpio_set_function_select(gpio_pin_15, gpio_alt_5);
			if (to_return != RPi_Success)
			{
				log_string_plus("uart_init:  failed to set up pin ", gpio_pin_15);
				break;
			}

			aux_perihperals_registers->aux_mu_cntl_reg = CNTL_TX_RX_ENABLE;
			uart_ready = 1;
		} while(0);
	} 
	return to_return;
}

void aux_putchar(uint32_t c)
{
    while(1)
    {
		if(aux_perihperals_registers->aux_mu_lsr_reg & UART_TX_IDLE) break;
    }
    aux_perihperals_registers->aux_mu_io_reg = c;
}

/*  Get a character, there is the possibilty of an interrupt occurring at just
	the wrong time, but this is really only designed for simple single character
	commands from a TTY.
*/

char aux_getchar(void)
{
	char to_return = 0;
	if (read_index != write_index)
	{
		to_return = rx_buffer[read_index];
		rx_buffer[read_index] = 0;
		read_index++;
		read_index = read_index % UART_RX_BUFFER_SIZE;
	}
	return to_return;
}
