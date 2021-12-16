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

File:  common.h

Common definitions and error types, also provides the interface into the assembly
language code for CPU operations.

*/

#pragma once
#include <stdint.h>
#include <stdio.h>

#define NULL_PTR 0
#define SVC_INITIAL_STACK 0x8000000

typedef enum {
	RPi_Success,
	RPi_Timeout,
	RPi_NotInitialized,
	RPi_InvalidParam,
	RPi_InUse,
	RPi_OperationFailed,
	I2CS_Ack_Error,
    I2CS_Data_Loss,
    I2CS_Clock_Timeout,
	GPIO_Pin_In_Use,
	MPU6050_Memory_Out_Of_Bounds,
	MPU6050_No_New_Data,
	MPU6050_Data_Overflow,
	PCA_9685_Insufficient_Device_Structures,
	PCA_9685_Register_Access_Failure
}  Error_Returns;

typedef enum {
	Exc_Reset,
	Exc_Undefined,
	Exc_SoftwareInterrupt,
	Exc_Prefetch,
	Exc_DataAbort,
	Exc_Unused,
	Exc_Interrupt,
	Exc_FastInterrupt
} Exception_Types;

extern void dummy();

extern void enable_cpu_interrupts(void);

extern void disable_cpu_interrupts(void);

