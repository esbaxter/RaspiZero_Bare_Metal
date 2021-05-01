include ..\..\Makefile.inc
#USESPI = -DSPI_MODE
CSRC = bme280.c mpu6050.c
OBJS = bme280.o mpu6050.o

all : $(OBJS) libsensors.a
	
clean :
	$(shell rm -f *.o)
	$(shell rm -f $(LIBDIR)\sensors.a)

bme280.o : bme280.c 
	$(ARMCOMP) $(COPS) $(USESPI) -c bme280.c -o bme280.o
	
mpu6050.o : mpu6050.c 
	$(ARMCOMP) $(COPS) $(USESPI) -c mpu6050.c -o mpu6050.o

libsensors.a : $(OBJS)
	$(ARMARCHIVE) cr $(LIBDIR)\libsensors.a $(OBJS)

