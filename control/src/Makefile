include ..\..\Makefile.inc

CSRC = pca9685.c 
OBJS = pca9685.o 

all : $(OBJS) libcontrol.a
	
clean :
	$(shell rm -f *.o)
	$(shell rm -f $(LIBDIR)\control.a)

pca9685.o : pca9685.c 
	$(ARMCOMP) $(COPS) -c pca9685.c -o pca9685.o

libcontrol.a : $(OBJS)
	$(ARMARCHIVE) cr $(LIBDIR)\libcontrol.a $(OBJS)

