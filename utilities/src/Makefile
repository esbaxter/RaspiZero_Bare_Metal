include ..\..\Makefile.inc

#Uncomment the following line to send messages and errors
#to an internal buffer rather than directly to the serial port
LOG_INT = -DLOG_INTERNAL
CSRC = log.c printf-stdarg.c
OBJS = log.o printf-stdarg.o

all : $(OBJS) libutilities.a
	
clean :
	$(shell rm -f *.o)
	$(shell rm -f $(LIBDIR)\utilities.a)

log.o : log.c 
	$(ARMCOMP) $(COPS) $(LOG_INT) -c log.c -o log.o
	
printf-stdarg.o : printf-stdarg.c 
	$(ARMCOMP) $(COPS) -c printf-stdarg.c -o printf-stdarg.o
	
libutilities.a : $(OBJS)
	$(ARMARCHIVE) cr $(LIBDIR)\libutilities.a $(OBJS)

