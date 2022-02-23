;@  This init file was inspired by reading through David Welch's raspberry pi zero
;@  programming examples.  I based this on blinker05 and left his copyright notice
;@  at the bottom.  Please note that I am still learning ARM assembly so things done
;@  here might not take full advantage of the instruction set, but I am looking for
;@  easy readability instead of performance, hopefully I have accomplished the former.

.globl _start
_start:
    ldr pc,reset_handler
    ldr pc,undefined_handler
    ldr pc,svc_handler
    ldr pc,prefetch_handler
    ldr pc,data_handler
    ldr pc,unused_handler
    ldr pc,irq_handler
    ldr pc,fiq_handler
	
reset_handler:      .word reset
undefined_handler:  .word undefined_dump
svc_handler:        .word svc_dump
prefetch_handler:   .word prefetch_dump
data_handler:       .word data_abort_dump
unused_handler:     .word unused_dump
irq_handler:        .word irq
fiq_handler:        .word fiq_dump

reset:
	;@ Set up exception vector table
    mov r0, #0x8000
    mov r1, #0x0000
	mov r2, #16
vec_loop: ldr r3, [r0], #4
	str r3, [r1], #4
	subs r2, r2, #1
	bne vec_loop
	
	;@ zero out the bss section
	ldr r2, bss_start 
	ldr r1, bss_end
	cmp r2, r1
	beq data_move
	mov r0, #0
bss_loop: str r0, [r2], #4
	cmp r2, r1
	bne bss_loop

	;@copy the data section
data_move: ldr r3, data_start 
	ldr r0, data_end
	ldr r1, data_rom_start
	cmp r3, r0
	bcs setup_stacks
data_loop: ldr	r2, [r1], #4
	str	r2, [r3], #4
	cmp	r3, r0
	bne data_loop
	
    ;@ Set up stack pointer for IRQ mode
setup_stacks: mov r0,#0xD2
    msr cpsr_c,r0
    mov sp,#0x8000
	
    ;@ Set up stack pointer for SVC mode
    mov r0,#0xD3
    msr cpsr_c,r0
    mov sp,#0x8000000
	
    ;@ This took some serious reading and rereading of the ARM ARM and a bunch
	;@ of web searches to make sure I understood what to do
    mrc p15, 0, r0, c1, c0, 2  ;@ Read from the system control coprocessor into r0
    orr r0,r0,#0xF00000 ;@ single precision and double precision
	;@ Write the updated control word back to the control coprocessor
    mcr p15, 0, r0, c1, c0, 2
	;@ Now execute a floating point coprocessor instruction to enable the floating point coprocessor!
    mov r0,#0x40000000 
    fmxr fpexc,r0

	;@ do a branch link in case our "main" returns 
	;@ we will fall through to the hang loop
launch: bl test_control
	
hang: b hang

undefined_dump: mov r0, #0x01 ;@ save source error as param to log_cpu_registers
	b common_dump
	
svc_dump: mov r0, #0x02 ;@ save source error as param to log_cpu_registers
	b common_dump
	
prefetch_dump: mov r0, #0x03 ;@ save source error as param to log_cpu_registers
	b common_dump
	
data_abort_dump:  mov r0,#0x4 ;@ save source error as param to log_cpu_registers
	b common_dump

unused_dump:  mov r0,#0x5 ;@ save source error as param to log_cpu_registers
	b common_dump

;@ irq_dump:  mov r0,#0x6 ;@ save source error as param to log_cpu_registers
;@  	b common_dump

fiq_dump:  mov r0,#0x7 ;@ save source error as param to log_cpu_registers
	b common_dump

common_dump: mov r3,#0xD3 ;@ set to SVC mode
    msr cpsr_c,r3
	movs r1, sp ;@ save stack pointer as param to log_cpu_registers
	movs r2, lr ;@ save link as param to log_cpu_registers
	bl log_cpu_registers

.globl dummy
dummy:
    bx lr

.global get_cpsr
get_cpsr:
	mrs r0, cpsr
	bx lr
	
.globl enable_cpu_interrupts
enable_cpu_interrupts:
    mrs r0,cpsr
    bic r0,r0,#0x80
    msr cpsr_c,r0
    bx lr
	
.globl disable_cpu_interrupts
disable_cpu_interrupts:
    mrs r0,cpsr
    orr r0,r0,#0x80
    msr cpsr_c,r0
    bx lr

irq:
    push {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,lr}
    bl interrupt_handler
    pop  {r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,lr}
    subs pc,lr,#4
	
.globl bss_start
bss_start: .word __bss_start__
.globl bss_end
bss_end: .word __bss_end__

.globl data_rom_start
data_rom_start:
.word __data_rom_start__

.globl data_start
data_start:
.word __data_start__
.globl data_end
data_end:
.word __data_end__

	
;@-------------------------------------------------------------------------
;@
;@ Copyright (c) 2012 David Welch dwelch@dwelch.com
;@
;@ Permission is hereby granted, free of charge, to any person obtaining a 
;@ copy of this software and associated documentation files (the "Software"), 
;@ to deal in the Software without restriction, including without limitation 
;@ the rights to use, copy, modify, merge, publish, distribute, sublicense, 
;@ and/or sell copies of the Software, and to permit persons to whom the 
;@ Software is furnished to do so, subject to the following conditions:
;@
;@ The above copyright notice and this permission notice shall be included 
;@ in all copies or substantial portions of the Software.
;@
;@ THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
;@ OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
;@ FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
;@ THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
;@ LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
;@ OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
;@ THE SOFTWARE.
;@
;@-------------------------------------------------------------------------
