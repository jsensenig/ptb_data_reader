########################################################################
## Simple Makefile for compiling GPIO setup test application
########################################################################
CC = g++

DEF_CFLAGS = -std=c++11 -Wall

#Compiled for Microzed
CFLAGS = -g $(DEF_CFLAGS)
INCLUDES = -I./
#SRCS = test_ep_gpio.cc
#SRCS = read_config.cc
#SRCS = prgrm_config.cc
SRCS = timing_ctrl.cc
#SRCS = prbs_check.cc
OBJS = $(SRCS:.cc=.o)
#MAIN = test_ep_gpio
#MAIN = read_config
#MAIN = prgrm_config
MAIN = timing_ctrl
#MAIN = prbs_check

.PHONY: depend clean

all:    $(MAIN)
	@echo  GPIO program $(MAIN) has been compiled!
$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) 
.cc.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	rm -rf *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
