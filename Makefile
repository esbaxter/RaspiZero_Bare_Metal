#Not the best implementation of makefiles but it works.
#Before anything else run make dirs, then you can run
#make without any arguments and it will build the project.
#To clean just run make TARGET=clean.  If you want to fully
#clean up run make TARGET=clean then make clean.

MKDIRAPP = "C:\Program Files\Git\usr\bin\mkdir"

LIB_BSP = BSP\src
LIB_CTRL = control\src
LIB_SENSORS = sensors\src
LIB_UTILS = utilities\src
LIBRARIES := $(LIB_BSP) $(LIB_CTRL) $(LIB_SENSORS) $(LIB_UTILS)
DRONE = drone_controller\src

LOCALDIRS = lib

.PHONY: all $(DRONE) $(LIBRARIES)
	
all: $(DRONE)

$(DRONE) $(LIBRARIES):
	$(MAKE) --directory=$@ $(TARGET)
	
$(DRONE): $(LIBRARIES)

dirs:
	$(MKDIRAPP) -p $(LOCALDIRS)
	
clean:
	$(shell rm -rf $(LOCALDIRS))