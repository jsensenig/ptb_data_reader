# Variables
MACHINE := $(shell uname -m)
CC      := gcc
CFLAGS 	:= -Wall


# PTB Build
ifeq ($(MACHINE), armv7l)
   	$(info =================================)
   	$(info +++ Compiling inside the uZed +++)
   	$(info =================================)
	CC		:= gcc
   	CFLAGS   := -O3 -march=armv7-a -mcpu=cortex-a9 -Wall
#   CFLAGS   := -O1 -march=armv7-a -mcpu=cortex-a9 $(DEF_CFLAGS)
#   CFLAGS   := -march=armv7-a -O0 $(DEF_CFLAGS) $(DMA_TYPE)
# Server Build
else
   	$(info =====================================)
   	$(info +++ Compiling with cross compiler +++)
   	$(info =====================================)
   	CC       := arm-xilinx-linux-gnueabi-gcc 
   	CFLAGS   := -O3
endif

TARGET := xadc_reader

.PHONY: all clean

all: $(TARGET)

clean: 
	rm -rf $(TARGET)
	
$(TARGET) : $(TARGET).c
	$(CC) $(CFLAGS) -o $@ $<

