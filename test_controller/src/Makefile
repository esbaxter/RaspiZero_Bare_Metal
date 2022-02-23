include ..\..\Makefile.inc

LIBPATH = -L"$(GCCINSTALLDIR)\arm-none-eabi\lib\arm\v5te\hard" -L"$(GCCINSTALLDIR)\lib\gcc\arm-none-eabi\10.2.1\arm\v5te\hard" -L$(LIBDIR)
LIBS = -lcontrol -lsensors -lutilities -lbsp -lgcc -lm

all: init.o altitude_package.o servo_controller.o test_driver.o
	$(ARMLINKER) init.o altitude_package.o servo_controller.o test_driver.o $(LIBPATH) $(LIBS) -T memmap -o test_driver.elf
	$(ARMOBJ)-objdump -D test_driver.elf > test_driver.list
	$(ARMOBJ)-objcopy --srec-forceS3 test_driver.elf -O srec test_driver.srec
	$(ARMOBJ)-objcopy test_driver.elf -O binary kernel.img
	
init.o: init.s
	$(ARMAS) $(AOPS) init.s -o init.o
	
altitude_package.o: altitude_package.c
	$(ARMCOMP) $(COPS) -c altitude_package.c -o altitude_package.o
	
servo_controller.o: servo_controller.c
	$(ARMCOMP) $(COPS) -c servo_controller.c -o servo_controller.o
		
test_driver.o: test_driver.c
	$(ARMCOMP) $(COPS) -c test_driver.c -o test_driver.o

clean :
	$(shell rm -f *.o)
	$(shell rm -f *.bin)
	$(shell rm -f *.hex)
	$(shell rm -f *.srec)
	$(shell rm -f *.elf)
	$(shell rm -f *.list)
	$(shell rm -f *.img)
