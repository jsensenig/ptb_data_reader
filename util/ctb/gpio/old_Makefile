########################################################################
## Simple Makefile for compiling GPIO setup test application
########################################################################
CC = g++

DEF_CFLAGS = -std=c++11 -Wall 

#Compiled for Microzed
CFLAGS = -g $(DEF_CFLAGS)
INCLUDES = -I./ 
SRCS = test_dac_calib.cc dac_setup.cc 
#SRCS = dac_setup.cc test_dac_set.cc
OBJS = $(SRCS:.cc=.o)
MAIN = test_dac_calib 
#MAIN = test_dac_set

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

