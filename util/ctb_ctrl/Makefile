########################################################################
## Simple Makefile for compiling GPIO setup test application
########################################################################
CC = g++

DEF_CFLAGS = -std=c++11 -Wall -O2

BOOST_INCS = 
BOOST_LIBS = -lboost_system -lboost_thread -pthread

#Compiled for Microzed
#CFLAGS = -g $(DEF_CFLAGS)
CFLAGS = $(DEF_CFLAGS)
INCLUDES = -I./

CTB_SERVER = ctb_ctrl_server
CTB_SERVER_SRCS = memory_utils.cc ctb_ctrl_server.cc
CTB_CLIENT = ctb_control
CTB_CLIENT_SRCS = ctb_ctrl_client.cc
PRBS_CHECK = prbs_check
PRBS_CHECK_SRCS = prbs_check.cc


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
	rm -rf $(CTB_SERVER) $(CTB_CLIENT) $(PRBS_CHECK) *.o

