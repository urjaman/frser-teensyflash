
# The name of your project (used to name the compiled .hex file)
TARGET = core

# configurable options
OPTIONS ?= -DF_CPU=72000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -D__MK20DX256__ -DTEENSYDUINO=120


# compiler options for C only
CFLAGS ?= -Wall -g -Os -mcpu=cortex-m4 -mthumb -nostdlib

CFLAGS += -ffunction-sections -fdata-sections

# CPPFLAGS = compiler options for C and C++
CPPFLAGS ?= $(CFLAGS) -MMD $(OPTIONS) -I.

# compiler options for C++ only
CXXFLAGS ?= -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

CXXFLAGS += -ffunction-sections -fdata-sections


# names for the compiler programs
CC ?= $(TCDIR)/arm-none-eabi-gcc
CXX ?= $(TCDIR)/arm-none-eabi-g++
AR ?= $(TCDIR)/arm-none-eabi-ar

# automatically create lists of the sources and objects
# TODO: this does not handle Arduino libraries yet...
C_FILES := $(wildcard *.c)
CPP_FILES := $(wildcard *.cpp)
OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)


# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).a

$(TARGET).a: $(OBJS)
	$(AR) rcs $(TARGET).a $(OBJS)



# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	rm -f *.o *.d $(TARGET).a


