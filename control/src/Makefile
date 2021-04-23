include ..\..\Makefile.inc

CSRC = altitude_package.c 
OBJS = altitude_package.o 

all : $(OBJS) libcontrol.a
	
clean :
	$(shell rm -f *.o)
	$(shell rm -f $(LIBDIR)\control.a)

altitude_package.o : altitude_package.c 
	$(ARMCOMP) $(COPS) -c altitude_package.c -o altitude_package.o

libcontrol.a : $(OBJS)
	$(ARMARCHIVE) cr $(LIBDIR)\libcontrol.a $(OBJS)

