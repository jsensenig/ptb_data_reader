########################################################################
## Simple Makefile for compiling GPIO setup test application
########################################################################
CC = g++

DEF_CFLAGS = -std=c++11 -Wall -O3

BOOST_INCS = 
BOOST_LIBS = -lboost_system -lboost_thread
#Compiled for Microzed
CFLAGS = -g -m32 $(DEF_CFLAGS)
INCLUDES = -I./
#SRCS = test_ep_gpio.cc
#SRCS = read_config.cc
#SRCS = prgrm_config.cc

CTB_SERVER = ctb_ctrl_server
CTB_SERVER_SRCS = memory_utils.cc ctb_ctrl_server.cc
CTB_CLIENT = ctb_ctrl
CTB_CLIENT_SRCS = ctb_ctrl_client.cc
PRBS_CHECK = prbs_check
PRBS_CHECK_SRCS = prbs_check.cc
#OBJS = $(SRCS:.cc=.o)
#MAIN = test_ep_gpio
#MAIN = read_config
#MAIN = prgrm_config
#MAIN = timing_ctrl
#MAIN = prbs_check


#.PHONY: all depend clean
.PHONY: all clean server client

all: $(CTB_CLIENT) 
	@echo  CTB utilities have been compiled!

server: $(CTB_SERVER)

client:$(CTB_CLIENT)


$(CTB_CLIENT): $(CTB_CLIENT_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BOOST_INCS)-o $@ $^ $(BOOST_LIBS) -pthread

$(CTB_SERVER): $(CTB_SERVER_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BOOST_INCS)-o $@ $^ $(BOOST_LIBS) 

$(PRBS_CHECK): $(PRBS_CHECK_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ 

clean: 
	rm -rf $(CTB_SERVER) $(CTB_CLIENT) $(PRBS_CHECK)

#$(MAIN): $(OBJS)
#	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) 
#.cc.o:
#	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

#clean:
#	rm -rf *.o *~ $(MAIN)

#depend: $(SRCS)
#	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
