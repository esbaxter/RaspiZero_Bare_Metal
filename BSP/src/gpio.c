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

Implementation of access to the GPIO ports on the Broadcom 2835.

Note:  GPIO pins are numbered based on their Broadcom 2835 numbers, not the pintout of
the Raspberry Pi header.

*/

#include "common.h"
#include "reg_definitions.h"
#include "gpio.h"
#include "log.h"
#include "arm_timer.h"

#define GPIO_PIN_COUNT 54
#define ALL_FUNCTION_BITS 7
#define BITS_PER_FUNCTION_SELECT 3
#define FUNCTION_SELECT_PINS_PER_REGISTER	10
#define ENABLE_PINS_PER_REGISTER	32

#define GPIO_FUNCTION_SELECT_SIZE	6
#define GPIO_ENABLE_ARRAY_SIZE		2

#define SINGLE_BIT_MASK 0x01

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
static GPIOFunction pin_direction_array[GPIO_PIN_COUNT];
static uint32_t pin_in_use_array[GPIO_ENABLE_ARRAY_SIZE];
static uint32_t gpio_initialized = 0;

static Error_Returns gpio_set_detect_register(char *error_string, volatile uint32_t *register_array, GPIO_Pins pin)
{
	Error_Returns to_return = RPi_Success;

	if (gpio_initialized)
	{		
		if (pin_direction_array[pin] == gpio_input)
		{
			uint32_t index = pin / ENABLE_PINS_PER_REGISTER;
			uint32_t pin_index = pin % ENABLE_PINS_PER_REGISTER;
			register_array[index] |= (1 << pin_index);
		}
		else
		{
			log_string_plus(error_string, pin);
			to_return = RPi_InvalidParam;
		}
	}
	else
	{
		to_return = RPi_NotInitialized;
	}
	return to_return;
}

Error_Returns gpio_init()
{
	Error_Returns to_return = RPi_Success;
	if (!gpio_initialized)
	{
		for(uint32_t index = 0; index < GPIO_PIN_COUNT; index++)
		{
			pin_direction_array[index] = gpio_input;
		}
		for (uint32_t index = 0; index < GPIO_ENABLE_ARRAY_SIZE; index++)
		{
			pin_in_use_array[index] = 0;
		}
		gpio_initialized = 1;
	}
	return to_return;
}

Error_Returns gpio_set_function_select(GPIO_Pins pin, GPIOFunction function)
{
	Error_Returns to_return = RPi_Success;
	uint32_t in_use_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t in_use_pin_index = pin % ENABLE_PINS_PER_REGISTER;
	
	if (gpio_initialized)
	{
		if (!(pin_in_use_array[in_use_index] & (1 << in_use_pin_index)))
		{
			uint32_t register_index = pin / FUNCTION_SELECT_PINS_PER_REGISTER;
			uint32_t pin_index = (pin % FUNCTION_SELECT_PINS_PER_REGISTER) * BITS_PER_FUNCTION_SELECT;
			//Clear the bits at the appropriate location
			gpio_registers->gpio_function_select[register_index] &= ~(ALL_FUNCTION_BITS << pin_index); 
			//Set the appropriate bits
			gpio_registers->gpio_function_select[register_index] |= (function << pin_index); 
			pin_in_use_array[in_use_index] |= (1 << in_use_pin_index);
			pin_direction_array[pin] = function;
		}
		else
		{
			log_string_plus("gpio_set_function_select:  pin in use: ", pin);
			to_return = GPIO_Pin_In_Use;
		}
	}
	else
	{
		to_return = RPi_NotInitialized;
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
	
	if (gpio_initialized)
	{
		if (pin_direction_array[pin] == gpio_output)
		{
			uint32_t index = pin / ENABLE_PINS_PER_REGISTER;
			uint32_t pin_index = pin % ENABLE_PINS_PER_REGISTER;
			gpio_registers->gpio_output_set[index] |= (1 << pin_index);
		}
		else
		{
			log_string_plus("gpio_set_pin:  pin not configured for use: ", pin);	
			to_return = RPi_InvalidParam;
		}
	}
	else
	{
		to_return = RPi_NotInitialized;
	}
	return to_return;
}

Error_Returns gpio_clear_pin(GPIO_Pins pin)
{
	Error_Returns to_return = RPi_Success;
	
	if (gpio_initialized)
	{
		if (pin_direction_array[pin] == gpio_output)
		{
			uint32_t index = pin / ENABLE_PINS_PER_REGISTER;
			uint32_t pin_index = pin % ENABLE_PINS_PER_REGISTER;
			gpio_registers->gpio_output_clear[index] |= (1 << pin_index);
		}
		else
		{
			log_string_plus("gpio_clear_pin:  pin not configured for use: ", pin);
			to_return = RPi_InvalidParam;
		}
	}
	else
	{
		to_return = RPi_NotInitialized;
	}
	return to_return;
}

Error_Returns gpio_get_level(GPIO_Pins pin, uint32_t *level_value)
{
	Error_Returns to_return = RPi_Success;

	if (gpio_initialized)
	{	
		if (pin_direction_array[pin] == gpio_input)
		{
			uint32_t index = pin / ENABLE_PINS_PER_REGISTER;
			uint32_t pin_index = pin % ENABLE_PINS_PER_REGISTER;
			uint32_t register_value = gpio_registers->gpio_level[index];
			*level_value = (register_value >> pin_index) & SINGLE_BIT_MASK;
		}
		else
		{
			log_string_plus("gpio_get_level:  pin not configured for use: ", pin);
			to_return = RPi_InvalidParam;
		}
	}
	else
	{
		to_return = RPi_NotInitialized;
	}
	return to_return;
}

Error_Returns gpio_set_high_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_high_detect_pin: pin not input: ", 
		gpio_registers->gpio_pin_high_detect_enable, pin);
}

Error_Returns gpio_set_low_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_low_detect_pin: pin not input: ", 
		gpio_registers->gpio_pin_low_detect_enable, pin);
}

Error_Returns gpio_set_rising_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_rising_detect_pin: pin not input: ", 
		gpio_registers->gpio_rising_edge_detect_enable, pin);
}

Error_Returns gpio_set_falling_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_falling_detect_pin: pin not input: ", 
		gpio_registers->gpio_falling_edge_detect_enable, pin);
}

Error_Returns gpio_set_async_rising_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_async_rising_detect_pin: pin not input: ", 
		gpio_registers->gpio_async_rising_edge_detect_enable, pin);
}

Error_Returns gpio_set_async_falling_detect_pin(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_set_async_falling_detect_pin: pin not input: ", 
		gpio_registers->gpio_async_falling_edge_detect_enable, pin);
}

GPIOEventDetectStatus gpio_get_event_detect_status(GPIO_Pins pin)
{
	uint32_t in_use_index = pin / ENABLE_PINS_PER_REGISTER;
	uint32_t in_use_pin_index = pin % ENABLE_PINS_PER_REGISTER;
	uint32_t register_value = gpio_registers->gpio_event_detect_status[in_use_index];
	GPIOEventDetectStatus to_return = (GPIOEventDetectStatus) register_value >> in_use_pin_index;
	return to_return;
}

Error_Returns gpio_clear_event_detect_status(GPIO_Pins pin)
{
	return gpio_set_detect_register("gpio_clear_event_detect_status: pin not input: ", 
		gpio_registers->gpio_event_detect_status, pin);
}