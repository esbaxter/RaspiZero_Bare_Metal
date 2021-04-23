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

File:  gpio.c

Implementation of access to the GPIO ports on the Broadcom 2835.  Currently only
supports setting up input or output, setting pullup or pulldown, and setting or clearing a port.

Note:  GPIO pins are numbered based on their Broadcom 2835 numbers, not the pintout of
the Raspberry Pi header.

*/

#include "common.h"
#include "reg_definitions.h"
#include "gpio.h"
#include "log.h"
#include "arm_timer.h"

#define ALL_FUNCTION_BITS 7
#define BITS_PER_FUNCTION_SELECT 3
#define FUNCTION_SELECT_PINS_PER_REGISTER	10
#define ENABLE_PINS_PER_REGISTER	32

#define GPIO_FUNCTION_SELECT_SIZE	6
#define GPIO_ENABLE_ARRAY_SIZE		2

#define GPIO_PUPD_SPIN_WAIT			150

typedef struct {
	uint32_t gpio_function_select[GPIO_FUNCTION_SELECT_SIZE];
	uint32_t reserved_0;
	uint32_t gpio_output_set[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_1;
	uint32_t gpio_output_clear[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_2;
	uint32_t gpio_level[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_3;
	uint32_t gpio_event_detect_status[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_4;
	uint32_t gpio_rising_edge_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_5;
	uint32_t gpio_falling_edge_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_6;
	uint32_t gpio_pin_high_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_7;
	uint32_t gpio_pin_low_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_8;
	uint32_t gpio_async_rising_edge_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_9;
	uint32_t gpio_async_falling_edge_detect_enable[GPIO_ENABLE_ARRAY_SIZE];
	uint32_t reserved_10;
	uint32_t gpio_pull_up_pull_down_enable;
	uint32_t gpio_pull_up_pull_down_clock[GPIO_ENABLE_ARRAY_SIZE];	
} GPIO_Registers;

static volatile GPIO_Registers *gpio_registers = (GPIO_Registers *)GPIO_BASE;
static uint32_t pin_in_use_array[GPIO_ENABLE_ARRAY_SIZE] = {0, 0};

Error_Returns gpio_set_function_select(GPIO_Pins pin, GPIOFunction function)
{
	Error_Returns to_return = RPi_Success;
	uint32_t in_use_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t in_use_pin_index = pin % ENABLE_PINS_PER_REGISTER;
	
	if (!(pin_in_use_array[in_use_index] & (1 << in_use_pin_index)))
	{
		uint32_t register_index = pin / FUNCTION_SELECT_PINS_PER_REGISTER;
		uint32_t pin_index = (pin % FUNCTION_SELECT_PINS_PER_REGISTER) * BITS_PER_FUNCTION_SELECT;
		//Clear the bits at the appropriate location
		gpio_registers->gpio_function_select[register_index] &= ~(ALL_FUNCTION_BITS << pin_index); 
		//Set the appropriate bits
		gpio_registers->gpio_function_select[register_index] |= (function << pin_index); 
		pin_in_use_array[in_use_index] |= (1 << in_use_pin_index);
	}
	else
	{
		log_string_plus("gpio_set_function_select:  pin in use: ", pin);
		to_return = GPIO_Pin_In_Use;
	}
	return to_return;
}

void gpio_set_pullup_pulldown(GPIO_Pins pin, GPIOPullUpPullDown function)
{
	uint32_t register_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t pin_index = pin % ENABLE_PINS_PER_REGISTER;

	gpio_registers->gpio_pull_up_pull_down_enable = function;
	spin_wait(GPIO_PUPD_SPIN_WAIT);
	gpio_registers->gpio_pull_up_pull_down_clock[register_index] =  1 << pin_index;
	spin_wait(GPIO_PUPD_SPIN_WAIT);
	gpio_registers->gpio_pull_up_pull_down_enable = 0;
	gpio_registers->gpio_pull_up_pull_down_clock[register_index] =  0;	
}

Error_Returns gpio_set_pin(GPIO_Pins pin)
{
	Error_Returns to_return = RPi_Success;
	uint32_t in_use_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t in_use_pin_index = pin % ENABLE_PINS_PER_REGISTER;
	
	if ((pin_in_use_array[in_use_index] & (1 << in_use_pin_index)))
	{
		gpio_registers->gpio_output_set[in_use_index] |= (1 << in_use_pin_index);
	}
	else
	{
		to_return = RPi_InvalidParam;
	}
	return to_return;
}

Error_Returns gpio_clear_pin(GPIO_Pins pin)
{
	Error_Returns to_return = RPi_Success;
	uint32_t in_use_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t in_use_pin_index = pin % ENABLE_PINS_PER_REGISTER;
	
	if ((pin_in_use_array[in_use_index] & (1 << in_use_pin_index)))
	{
		gpio_registers->gpio_output_clear[in_use_index] |= (1 << in_use_pin_index);
	}
	else
	{
		to_return = RPi_InvalidParam;
	}
	return to_return;
}