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

File:  interrupt_handler.c

Implementation of interrupts using the Broadcom 2835 interrupt function and the
processor interrupt function.

*/

#include "common.h"
#include "reg_definitions.h"
#include "interrupt_handler.h"
#include "log.h"

#define MAX_INTERRUPT_HANDLER_FUNCTIONS 4
#define INTERRUPT_BASIC 1

typedef struct {
	uint32_t irq_basic_pending;
	uint32_t irq_pending_1;
	uint32_t irq_pending_2;
	uint32_t fiq_control;
	uint32_t enable_irqs_1;
	uint32_t enable_irqs_2;
	uint32_t enable_basic_irqs;
	uint32_t disable_irqs_1;
	uint32_t disable_irqs_2;
	uint32_t disable_basic_irqs;
} ARM_Interrupt_Registers;

static volatile ARM_Interrupt_Registers *arm_interrupt_registers = (ARM_Interrupt_Registers *)ARM_INTERRUPTS_BASE;

static InterruptHandlerStatus (*interrupt_handler_ptr_array[MAX_INTERRUPT_HANDLER_FUNCTIONS])(void);
static int number_of_handlers = 0;

static unsigned char interrupt_handler_initialized = 0;

void interrupt_handler_dump_registers(void)
{
	log_string_plus("irq_basic_pending: ", arm_interrupt_registers->irq_basic_pending);
	log_string_plus("enable_basic_irqs: ", arm_interrupt_registers->enable_basic_irqs);
}

Error_Returns interrupt_handler_init()
{
	Error_Returns to_return = RPi_Success;
	if (!interrupt_handler_initialized)
	{
		for(uint32_t index = 0; index < MAX_INTERRUPT_HANDLER_FUNCTIONS; index++)
		{
			interrupt_handler_ptr_array[index] = NULL_PTR;
		}
		number_of_handlers = 0;
		interrupt_handler_initialized = 1;
	}
	return to_return;
}

void interrupt_handler(void)
{
	uint32_t interrupt_handled = 0;
	for(uint32_t index = 0; index < MAX_INTERRUPT_HANDLER_FUNCTIONS; index++)
	{
		if (interrupt_handler_ptr_array[index] != NULL_PTR)
		{
			InterruptHandlerStatus status = interrupt_handler_ptr_array[index]();
			if (status == Interrupt_Claimed)
			{
				interrupt_handled = 1;
			}
		}
	}
	if (!interrupt_handled)
	{
		log_string_plus("Interrupt not handled:  irq_basic_pending: ", arm_interrupt_registers->irq_basic_pending);
		log_string_plus("Interrupt not handled:  irq_pending_1: ", arm_interrupt_registers->irq_pending_1);
		log_string_plus("Interrupt not handled:  irq_pending_2: ", arm_interrupt_registers->irq_pending_2);
	}
}

/*  Add an interrupt handler and enable interrupts on both the peripheral section
	and the CPU.
*/

int interrupt_handler_add(InterruptHandlerStatus (*handler_ptr)(void))
{
	int to_return = number_of_handlers;
	if (number_of_handlers < MAX_INTERRUPT_HANDLER_FUNCTIONS)
	{
		int index = 0;
		for(; interrupt_handler_ptr_array[index] != NULL_PTR; index++);
		interrupt_handler_ptr_array[index] = handler_ptr;
		number_of_handlers++;
		arm_interrupt_registers->enable_basic_irqs = INTERRUPT_BASIC;
		enable_cpu_interrupts();
	}
	else
	{
		to_return = -1;
	}
	return to_return;
}

/*  Remove a handler and if no one else is expecting interrupts then
	disable interrupts on both the peripherals section and the CPU.
*/
	
Error_Returns interrupt_handler_remove(int handler_index)
{
	Error_Returns to_return = RPi_Success;
	if ((handler_index >= 0) &&
	  (handler_index < MAX_INTERRUPT_HANDLER_FUNCTIONS) && 
	  (interrupt_handler_ptr_array[handler_index] != NULL_PTR))
	  {
		  interrupt_handler_ptr_array[handler_index] = NULL_PTR;
		  number_of_handlers--;
		  if (number_of_handlers == 0)
		  {
			  disable_cpu_interrupts();
			  arm_interrupt_registers->disable_basic_irqs = INTERRUPT_BASIC;
		  }
	  }
	 else
	 {
		 to_return = RPi_InvalidParam;
	 }
	 return to_return;
}