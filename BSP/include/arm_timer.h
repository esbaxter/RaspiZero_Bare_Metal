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

File:  arm_timer.h

Interface into the ARM timer on the Broadcom 2835.  Along with a busy wait timer
that just spins the ARM CPU.

*/

#pragma once
#include "common.h"

#define SPIN_WAIT_ONE_SECOND 3700000
#define SPIN_WAIT_ONE_MILLISECOND 3700

Error_Returns arm_timer_init(void);

Error_Returns arm_timer_enable(void (*handler_ptr)(void), uint32_t time_out);

Error_Returns arm_timer_disable(void);

void arm_timer_dump_registers(void);

void spin_wait(uint32_t spin_count);

void spin_wait_seconds(uint32_t seconds);

void spin_wait_milliseconds(uint32_t milliseconds);