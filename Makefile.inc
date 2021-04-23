GCCINSTALLDIR = C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2020-q4-major
PROJINCLUDES = -I..\..\include -I..\..\control\include -I..\..\BSP\include -I..\..\sensors\include -I..\..\utilities\include -I.\include
OBJDIR = obj
LIBDIR = ..\..\lib

ARMCOMP ?= arm-none-eabi-gcc
ARMAS ?= arm-none-eabi-as
ARMARCHIVE ?= arm-none-eabi-ar
ARMLINKER ?= arm-none-eabi-ld
ARMOBJ ?= arm-none-eabi

AOPS = --warn --fatal-warnings -mcpu=arm1176jzf-s -march=armv6 -mfpu=vfp
COPS = -Wall -Werror -O2 -nostdlib -nostartfiles -ffreestanding $(PROJINCLUDES)  -mcpu=arm1176jzf-s -mfloat-abi=hard
