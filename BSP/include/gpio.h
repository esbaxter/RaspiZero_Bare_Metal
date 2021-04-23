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

File:  gpio.h

Provides access to the GPIO pins on the Raspberry Pi.

Note:  GPIO pins are numbered based on their Broadcom 2835 numbers, not the pintout of
the Raspberry Pi header.

*/

#pragma once
#include "common.h"

typedef enum {
	gpio_input = 0,
	gpio_output = 1,
	gpio_alt_0 = 4,
	gpio_alt_1 = 5,
	gpio_alt_2 = 6,
	gpio_alt_3 = 7,
	gpio_alt_4 = 3,
	gpio_alt_5 = 2
} GPIOFunction;

typedef enum {
	func_sel_0 = 0,
	func_sel_1 = 3,
	func_sel_2 = 6,
	func_sel_3 = 9,
	func_sel_4 = 12,
	func_sel_5 = 15,
	func_sel_6 = 18,
	func_sel_7 = 21,
	func_sel_8 = 24,
	func_sel_9 = 27
} GPIOPinFunctionSelect;

typedef enum {
	gpio_pin_0,
	gpio_pin_1,
	gpio_pin_2,
	gpio_pin_3,
	gpio_pin_4,
	gpio_pin_5,
	gpio_pin_6,
	gpio_pin_7,
	gpio_pin_8,
	gpio_pin_9,
	gpio_pin_10,
	gpio_pin_11,
	gpio_pin_12,
	gpio_pin_13,
	gpio_pin_14,
	gpio_pin_15,
	gpio_pin_16,
	gpio_pin_17,
	gpio_pin_18,
	gpio_pin_19,
	gpio_pin_20,
	gpio_pin_21,
	gpio_pin_22,
	gpio_pin_23,
	gpio_pin_24,
	gpio_pin_25,
	gpio_pin_26,
	gpio_pin_27,
	gpio_pin_28,
	gpio_pin_29,
	gpio_pin_30,
	gpio_pin_31,
	gpio_pin_32,
	gpio_pin_33,
	gpio_pin_34,
	gpio_pin_35,
	gpio_pin_36,
	gpio_pin_37,
	gpio_pin_38,
	gpio_pin_39,
	gpio_pin_40,
	gpio_pin_41,
	gpio_pin_42,
	gpio_pin_43,
	gpio_pin_44,
	gpio_pin_45,
	gpio_pin_46,
	gpio_pin_47,
	gpio_pin_48,
	gpio_pin_49,
	gpio_pin_50,
	gpio_pin_51,
	gpio_pin_52,
	gpio_pin_53,
	gpio_pin_54
}  GPIO_Pins;

typedef enum {
	pupd_disable,
	pupd_pull_down,
	pupd_pull_up
} GPIOPullUpPullDown;
	

extern Error_Returns gpio_set_function_select(GPIO_Pins pin, GPIOFunction function);

extern void gpio_set_pullup_pulldown(GPIO_Pins pin, GPIOPullUpPullDown function);

extern Error_Returns gpio_set_pin(GPIO_Pins pin);

extern Error_Returns gpio_clear_pin(GPIO_Pins pin);