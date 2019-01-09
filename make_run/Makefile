##################################################
# PROJECT: LabArm API, compile example running the API
# AUTHOR : Benjamin IOLLER
##################################################

#---------------------------------------------------------------------
# Makefile template inspired by the Dynamixel Makefile
#
# This Makefile will set the compiler option for a x86_64 (Ubuntu) or armv71 (Raspberry 3)
# Add additionnal library (OPENCV, ...) with the LIBRARIES variable
# Add additionnal sources code with the SOURCES variable
#
# Makefile will work for c and cpp source code.
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET      =  exampleArm

# Directories path creation: Include and Object
DIR_DXL    = ../
DIR_OBJS   = .objects

# Get host architecture, set the correct Dynamixel library, and compiler option
ARCH :=$(shell arch)
ifeq ($(ARCH),armv71)
	DYNAMIXEL = -ldxl_sbc_cpp
	CCFLAGS   = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
	CXFLAGS   = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) -g
	INCLUDES += -I$(DIR_DXL)/include/dynamixel_sdk_raspberry
endif

ifeq ($(ARCH),x86_64)
	LIBRARIES  += -ldxl_x64_cpp
	CCFLAGS     = -std=c++11 -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
	CXFLAGS     = -std=c++11 -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
	INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk_ubuntu
	FORMAT      = -m64
endif
	
# General compiler options
CC          = gcc
CX          = g++
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS)

#---------------------------------------------------------------------
# Core components 
#---------------------------------------------------------------------
LIBRARIES  += -lrt
REALSENSE = -lrealsense2
OPENCV = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_dnn
LIBRARIES += $(REALSENSE)
LIBRARIES += $(OPENCV)

#---------------------------------------------------------------------
# Source files
#---------------------------------------------------------------------
SOURCES = exampleArm.cpp
SOURCES += motorMX430.cpp
SOURCES += joystick.cpp
SOURCES += LabArm.cpp

    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) $(LIBRARIES) -o $(TARGET)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ../%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
