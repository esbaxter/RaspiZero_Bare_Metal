include ..\..\Makefile.inc

CSRC = aux_peripherals.c spi.c i2c.c gpio.c interrupt_handler.c arm_timer.c
OBJS = aux_peripherals.o spi.o i2c.o gpio.o interrupt_handler.o arm_timer.o

all : $(OBJS) libbsp.a
	
clean :
	$(shell rm -f *.o)
	$(shell rm -f $(LIBDIR)\bsp.a)
	
aux_peripherals.o : aux_peripherals.c 
	$(ARMCOMP) $(COPS) -c aux_peripherals.c -o aux_peripherals.o

spi.o : spi.c
	$(ARMCOMP) $(COPS) -c spi.c -o spi.o
	
i2c.o : i2c.c 
	$(ARMCOMP) $(COPS) -c i2c.c -o i2c.o	

gpio.o : gpio.c 
	$(ARMCOMP) $(COPS) -c gpio.c -o gpio.o
	
interrupt_handler.o : interrupt_handler.c
	$(ARMCOMP) $(COPS) -c interrupt_handler.c -o interrupt_handler.o

arm_timer.o : arm_timer.c
	$(ARMCOMP) $(COPS) -c arm_timer.c -o arm_timer.o

libbsp.a : $(OBJS)
	$(ARMARCHIVE) cr $(LIBDIR)\libbsp.a $(OBJS)

