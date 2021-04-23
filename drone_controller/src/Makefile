include ..\..\Makefile.inc

LIBPATH = -L"$(GCCINSTALLDIR)\arm-none-eabi\lib\arm\v5te\hard" -L"$(GCCINSTALLDIR)\lib\gcc\arm-none-eabi\10.2.1\arm\v5te\hard" -L$(LIBDIR)
LIBS = -lcontrol -lsensors -lutilities -lbsp -lgcc -lm

all: init.o drone_controller.o
	$(ARMLINKER) init.o drone_controller.o $(LIBPATH) $(LIBS) -T memmap -o drone_controller.elf
	$(ARMOBJ)-objdump -D drone_controller.elf > drone_controller.list
	$(ARMOBJ)-objcopy --srec-forceS3 drone_controller.elf -O srec drone_controller.srec
	$(ARMOBJ)-objcopy drone_controller.elf -O binary kernel.img
	
init.o: init.s
	$(ARMAS) $(AOPS) init.s -o init.o
	
drone_controller.o: drone_controller.c
	$(ARMCOMP) $(COPS) -c drone_controller.c -o drone_controller.o

clean:
	$(shell rm -f *.o)
	$(shell rm -f *.bin)
	$(shell rm -f *.hex)
	$(shell rm -f *.srec)
	$(shell rm -f *.elf)
	$(shell rm -f *.list)
	$(shell rm -f *.img)
