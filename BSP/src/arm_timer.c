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

File:  arm_timer.c

Provides ARM timer support using the Broadcom 2835.

*/

#include "common.h"
#include "reg_definitions.h"
#include "arm_timer.h"
#include "interrupt_handler.h"
#include "log.h"

typedef struct {
	uint32_t load;
	uint32_t value;
	uint32_t control;
	uint32_t irq_clear_ack;
	uint32_t raw_irq;
	uint32_t masked_irq;
	uint32_t reload;
	uint32_t predivider;
	uint32_t free_running_counter;
} ARM_Timer_Registers;

#define CORE_CLOCK_SPEED 250000000
#define PRE_DIVIDER_VALUE 249 //The chip clocks on pre_divider + 1, we want 250 so set it to 249
#define DIVIDED_CLOCK_SPEED (CORE_CLOCK_SPEED / (PRE_DIVIDER_VALUE + 1))
#define CLOCKS_PER_MILLISECOND (DIVIDED_CLOCK_SPEED/1000)
#define ARM_TIMER_INTERRUPT_ACTIVE 1
#define ARM_TIMER_CLEAR_INTERRUPT 0
#define ARM_TIMER_ENABLE 7
#define ARM_TIMER_INTERRUPT_ENABLE 5
#define ARM_TIMER_32_BIT_COUNTER 1

static void (*timer_handler_ptr)(void); //We handle one client at a time
static unsigned char arm_timer_initialized = 0;
static int interrupt_handler_index = 0;

static volatile ARM_Timer_Registers *arm_timer_registers = (ARM_Timer_Registers *)ARM_TIMER_BASE;

void arm_timer_dump_registers(void)
{
	log_string_plus("load: ", arm_timer_registers->load);
	log_string_plus("value: ", arm_timer_registers->value);
	log_string_plus("control: ", arm_timer_registers->control);
	log_string_plus("raw_irq: ", arm_timer_registers->raw_irq);
	log_string_plus("masked_irq: ", arm_timer_registers->masked_irq);
	log_string_plus("reload: ", arm_timer_registers->reload);
	log_string_plus("predivider: ", arm_timer_registers->predivider);
	log_string_plus("free_running_counter: ", arm_timer_registers->free_running_counter);
}

/*  When the ARM timer interrupts the CPU the interrupt handler will call 
    this routine, which in turn will notify the client of the timeout.  If
	the ARM timer didn't interrupt we will return unclaimed to the interrupt
	handler.
*/

InterruptHandlerStatus arm_timer_interrupt_handler(void)
{
	InterruptHandlerStatus to_return = Interrupt_Not_Claimed;
	if (arm_timer_registers->masked_irq & ARM_TIMER_INTERRUPT_ACTIVE)
	{
		if (timer_handler_ptr != NULL_PTR)
		{
			timer_handler_ptr();
			arm_timer_registers->irq_clear_ack = ARM_TIMER_CLEAR_INTERRUPT;
			to_return = Interrupt_Claimed;
		}
	}
	return to_return;
}

/* Initialize the interrupt handler and set up some basic housekeeping stuff. */

Error_Returns arm_timer_init(void)
{
	Error_Returns to_return = RPi_Success;
	if (!arm_timer_initialized)
	{
		to_return = interrupt_handler_init();
		if (to_return != RPi_Success)
		{
			log_string_plus("Failed interrupt_handler_init, status: ", to_return);
			log_indicate_system_error();
		}
		timer_handler_ptr = NULL_PTR;
		arm_timer_registers->predivider = PRE_DIVIDER_VALUE;
		interrupt_handler_index = -1;
		arm_timer_initialized = 1;
	}	
	return to_return;
}

/*  A client uses this function to set up a recurring tick timer.  The time out
	is in milliseconds.  Note:  Only one client can be using the timer at a time.
*/

Error_Returns arm_timer_enable(void (*handler_ptr)(void), uint32_t time_out)
{
	Error_Returns to_return = RPi_Success;
	do
	{
		if (!arm_timer_initialized)
		{
			to_return = RPi_NotInitialized;
			break;
		}
		if (timer_handler_ptr != NULL_PTR)
		{
			to_return = RPi_InUse;
			break;
		}
		interrupt_handler_index = interrupt_handler_basic_add(arm_timer_interrupt_handler);
		if (interrupt_handler_index == -1)
		{
			to_return = RPi_OperationFailed;
			break;
		}

		timer_handler_ptr = handler_ptr;
		uint32_t counter_load_value = CLOCKS_PER_MILLISECOND * time_out;
		arm_timer_registers->load = counter_load_value;
		arm_timer_registers->reload = counter_load_value;
		arm_timer_registers->control |= (1 << ARM_TIMER_ENABLE) | (1 << ARM_TIMER_INTERRUPT_ENABLE) | (1 << ARM_TIMER_32_BIT_COUNTER);
		
	} while(0);
	return to_return;
}

/*  A client uses this function to stop a repeating tick timer.
*/

Error_Returns arm_timer_disable(void)
{
	Error_Returns to_return = RPi_Success;
	do
	{
		if ((!arm_timer_initialized) || (timer_handler_ptr == NULL_PTR) || (interrupt_handler_index == -1))
		{
			to_return = RPi_NotInitialized;
			break;
		}
		
		arm_timer_registers->control &= ~((1 << ARM_TIMER_ENABLE) | (1 << ARM_TIMER_INTERRUPT_ENABLE));
		timer_handler_ptr = NULL_PTR;
		
		Error_Returns status = interrupt_handler_remove(interrupt_handler_index);
		if (status != RPi_Success)
		{
			to_return = RPi_OperationFailed;
		}

	} while(0);
	return to_return;
}


void spin_wait(uint32_t spin_count)
{
	for(uint32_t counter = 0; counter < spin_count; counter++) dummy(counter);
}

void spin_wait_seconds(uint32_t seconds)
{
	for(uint32_t counter = 0; counter < seconds; counter++) spin_wait(SPIN_WAIT_ONE_SECOND);
}

void spin_wait_milliseconds(uint32_t milliseconds){
	for(uint32_t counter = 0; counter < milliseconds; counter++) spin_wait(SPIN_WAIT_ONE_MILLISECOND);
}