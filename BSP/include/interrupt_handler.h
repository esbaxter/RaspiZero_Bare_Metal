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

File:  interrupt_handler.h

Provides a simple way to make use of interrupts on the Broadcom 2835.

*/

#pragma once
#include "common.h"
#include "gpio.h"

typedef enum {
	Interrupt_Claimed,
	Interrupt_Not_Claimed
} InterruptHandlerStatus;

typedef enum {
	Int_Basic,
	Int_GPIO_Pin,
	Int_GPIO_All
} InterruptType;

extern Error_Returns interrupt_handler_init(void);

extern int interrupt_handler_add(InterruptHandlerStatus (*handler_ptr)(void), InterruptType type,
GPIO_Pins pin);

extern int interrupt_handler_basic_add(InterruptHandlerStatus (*handler_ptr)(void));

extern Error_Returns interrupt_handler_remove(int handler_index);

extern void interrupt_handler_dump_registers(void);