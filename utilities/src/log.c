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

File:  log.c

Implements a simple logging function that either prints directly to the serial port
or saves the logged message to a buffer to dump at a later time.  Where the logging
goes is controlled by a -D flag specified in the makefile.

*/

#include "aux_peripherals.h"
#include "gpio.h"
#include "arm_timer.h"
#include "log.h"

#define TIMER_VAL 600000
#define LED_GPIO_PIN gpio_pin_47

#define ASCII_SPACE 0x20
#define ASCII_ZERO 0x30
#define ASCII_CAPITAL_X 0x58
#define ASCII_CARRIAGE_RETURN 0x0D
#define ASCII_LINE_FEED 0x0A
#define ASCII_HEX_ALPHABETIC 0x37
#define ASCII_HEX_NUMBER 0x30
#define ASCII_VALUE_MASK 0x0F

#define LOG_BUFFER_SIZE 1024
#define LOG_BUFFER_MASK 0xFF
#if defined LOG_INTERNAL
#define LOG_CHAR log_char_to_buffer
#else
#define LOG_CHAR aux_putchar
#endif

static char log_buffer[LOG_BUFFER_SIZE] = {0};
static uint32_t buffer_rolled_over = 0;
static uint32_t buffer_write_index = 0;

static char interrupt_log_buffer[LOG_BUFFER_SIZE] = {0};
static uint32_t interrupt_buffer_rolled_over = 0;
static uint32_t interrupt_buffer_write_index = 0;

/* This log buffer potentially leaves a partial message that will show up
at the beginning of the log dump.  We won't worry about that since the
partial message is the oldest in the system and hopefully we won't be
writing to this buffer too much */

void log_char_to_interrupt_buffer(uint32_t c)
{
	interrupt_log_buffer[interrupt_buffer_write_index++] = c & LOG_BUFFER_MASK;
	if (!interrupt_buffer_rolled_over)
	{
		interrupt_buffer_rolled_over = interrupt_buffer_write_index / LOG_BUFFER_SIZE;
	}
	interrupt_buffer_write_index = interrupt_buffer_write_index % LOG_BUFFER_SIZE;
}

static void log_interrupt_hex_value(uint32_t value)
{
	log_char_to_interrupt_buffer(ASCII_ZERO);
	log_char_to_interrupt_buffer(ASCII_CAPITAL_X);
	for(uint32_t counter = 0; counter < 8; counter++)
	{
		value= value<<4 | value>>28;
		uint32_t character = value & ASCII_VALUE_MASK;
		if (character <= 9)
		{
			character += ASCII_HEX_NUMBER;
		}
		else
		{
			character += ASCII_HEX_ALPHABETIC;
		}
		log_char_to_interrupt_buffer(character);
	}
	log_char_to_interrupt_buffer(ASCII_SPACE);
}

void log_char_to_buffer(uint32_t c)
{
	log_buffer[buffer_write_index++] = c & LOG_BUFFER_MASK;
	if (!buffer_rolled_over)
	{
		buffer_rolled_over = buffer_write_index / LOG_BUFFER_SIZE;
	}
	buffer_write_index = buffer_write_index % LOG_BUFFER_SIZE;
}

static void log_hex_value(uint32_t value)
{
	LOG_CHAR(ASCII_ZERO);
	LOG_CHAR(ASCII_CAPITAL_X);
	for(uint32_t counter = 0; counter < 8; counter++)
	{
		value= value<<4 | value>>28;
		uint32_t character = value & ASCII_VALUE_MASK;
		if (character <= 9)
		{
			character += ASCII_HEX_NUMBER;
		}
		else
		{
			character += ASCII_HEX_ALPHABETIC;
		}
		LOG_CHAR(character);
	}
	LOG_CHAR(ASCII_SPACE);
}

void log_indicate_system_ok(void)
{
	Error_Returns status = gpio_init();
	status = gpio_set_function_select(LED_GPIO_PIN, gpio_output);
	if (status != RPi_Success)
	{
		log_indicate_system_error();
	}
	
	status = gpio_clear_pin(LED_GPIO_PIN);
	if (status != RPi_Success)
	{
		log_indicate_system_error();
	}	
}

void log_indicate_system_error(void)
{
	Error_Returns status = gpio_init();
	status = gpio_clear_pin(LED_GPIO_PIN);
	if (status != RPi_Success)
	{
		gpio_set_function_select(LED_GPIO_PIN, gpio_output);
	}
	while (1)
	{
		spin_wait(TIMER_VAL);
		gpio_clear_pin(LED_GPIO_PIN);
		spin_wait(TIMER_VAL);
		gpio_set_pin(LED_GPIO_PIN);
		char tty_char = log_getchar();
		if (tty_char == 'd')
		{
			log_dump_buffer();
			break;
		}
	}
}

void log_cpu_registers(Exception_Types error_source, uint32_t stack_pointer, uint32_t link_return)
{
	log_string(" ");
	switch (error_source)
	{
		case Exc_Reset: log_string("Exc_Reset");
			break;
		case Exc_Undefined: log_string("Exc_Undefined");
			break;
		case Exc_SoftwareInterrupt: log_string("Exc_SoftwareInterrupt");
			break;
		case Exc_Prefetch: log_string("Exc_Prefetch");
			break;
		case Exc_DataAbort: log_string("Exc_DataAbort");
			break;
		case Exc_Unused: log_string("Exc_Unused");
			break;
		case Exc_Interrupt: log_string("Exc_Interrupt");
			break;
		case Exc_FastInterrupt: log_string("Exc_FastInterrupt");
			break;
		default: log_string("Uknown!");
			break;				
	}

	log_string_plus("link return: ", link_return);
	uint32_t *stack_val_ptr = (uint32_t *)stack_pointer;
	for(; stack_val_ptr < (uint32_t *)SVC_INITIAL_STACK; stack_val_ptr++)
	{
		log_string_plus("stack_val pointer: ", (uint32_t)stack_val_ptr);
		log_string_plus("stack_val : ", *stack_val_ptr);
	}
	log_indicate_system_error();
}

Error_Returns log_init(void)
{
	Error_Returns to_return = gpio_init();
	if (to_return == RPi_Success)
	{
		to_return = uart_init();
	}
	return to_return;
}

void log_string(const char *log_string)
{
	while (*log_string != 0x00)
	{
		LOG_CHAR(*log_string);
		log_string++;
	}

	LOG_CHAR(ASCII_CARRIAGE_RETURN);
    LOG_CHAR(ASCII_LINE_FEED);
}

void log_string_plus(const char *log_string, uint32_t value)
{
	while (*log_string != 0x00)
	{
		LOG_CHAR(*log_string);
		log_string++;
	}
	
	log_hex_value(value);

	LOG_CHAR(ASCII_CARRIAGE_RETURN);
    LOG_CHAR(ASCII_LINE_FEED);
}

void log_interrupt_string(const char *log_string)
{
	while (*log_string != 0x00)
	{
		log_char_to_interrupt_buffer(*log_string);
		log_string++;
	}

	log_char_to_interrupt_buffer(ASCII_CARRIAGE_RETURN);
    log_char_to_interrupt_buffer(ASCII_LINE_FEED);
}

void log_interrupt_string_plus(const char *log_string, uint32_t value)
{
	while (*log_string != 0x00)
	{
		log_char_to_interrupt_buffer(*log_string);
		log_string++;
	}
	
	log_interrupt_hex_value(value);

	log_char_to_interrupt_buffer(ASCII_CARRIAGE_RETURN);
    log_char_to_interrupt_buffer(ASCII_LINE_FEED);
}

char log_getchar(void)
{
	return aux_getchar();
}

void log_putchar(char c)
{
	LOG_CHAR(c);
}

void log_dump_buffer(void)
{
	/* If the buffer rolled over we could just skip over any
	partial message, but since we are doing this for debugging purposes
	be aware that the partial message is the oldest in the log, but
	it might just provide that vital clue as to what was going on... */
	
	if (buffer_rolled_over)
	{
		uint32_t buffer_read_index = buffer_write_index + 1;
		do
		{
			aux_putchar(log_buffer[buffer_read_index++]);
			buffer_read_index = buffer_read_index % LOG_BUFFER_SIZE;
		} while (buffer_read_index != buffer_write_index);
	}
	else{
		uint32_t buffer_read_index = 0;
		do
		{
			aux_putchar(log_buffer[buffer_read_index++]);
			buffer_read_index = buffer_read_index % LOG_BUFFER_SIZE;
		} while (log_buffer[buffer_read_index] != 0);
	}
	
	if (interrupt_buffer_rolled_over)
	{
		uint32_t buffer_read_index = interrupt_buffer_write_index + 1;
		do
		{
			aux_putchar(interrupt_log_buffer[buffer_read_index++]);
			buffer_read_index = buffer_read_index % LOG_BUFFER_SIZE;
		} while (buffer_read_index != interrupt_buffer_write_index);
	}
	else{
		uint32_t buffer_read_index = 0;
		do
		{
			aux_putchar(interrupt_log_buffer[buffer_read_index++]);
			buffer_read_index = buffer_read_index % LOG_BUFFER_SIZE;
		} while (interrupt_log_buffer[buffer_read_index] != 0);
	}
}