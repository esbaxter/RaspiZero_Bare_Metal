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

File:  reg_definitions.h

Provides base register set definitions for the functions in the Broadcom 2835.

*/

#pragma once

#define P_BASE 0x20000000

//GPIO control registers
#define GPIO_BASE (P_BASE + 0x200000)

//SPI and UART registers
#define AUX_BASE     (P_BASE + 0x215000)

//SPI control registers Note:  the documentation is confusing since it 
//lists a couple of "SPI 0" (aux peripheral section and section 10 not
//to mention the Pi Zero schematic has a note about SPI 0)
#define SPI0_BASE     (P_BASE + 0x204000)

//BSC control registers (for I2C)
#define BSC0_BASE	(P_BASE + 0x205000)
#define BSC1_BASE	(P_BASE + 0x804000)
#define BSC2_BASE	(P_BASE + 0x805000)

//ARM interrupt register
#define ARM_INTERRUPTS_BASE	(P_BASE + 0xB200)

//ARM timer register
#define ARM_TIMER_BASE	(P_BASE + 0xB400)