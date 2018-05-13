# Variables
MACHINE    := $(shell uname -m)
OPT_FLAGS_DEBUG  := -O0 -g
OPT_FLAGS_PROD   := -O3 
CC         := g++
DEF_CFLAGS := -std=c++11 -Wall -I. 
DEF_LFLAGS := -lpthread
BOOST_CFLAGS_LINUX := -I/usr/local/boost/1.66.00/include
BOOST_LFLAGS_LINUX := /usr/local/boost/1.66.00/lib/libboost_thread.a /usr/local/boost/1.66.00/lib/libboost_system.a
BOOST_LFLAGS_CTB := /usr/lib/arm-linux-gnueabihf/libboost_thread.a /usr/lib/arm-linux-gnueabihf/libboost_system.a
BOOST_CFLAGS_CTB := 

ROOT_CFLAGS := $(shell root-config --cflags)
ROOT_LFLAGS := $(shell root-config --ldflags --libs)
# TPB Build


OPT_FLAGS := $(OPT_FLAGS_DEBUG)

ifeq ($(MACHINE), armv7l)
   $(info ======================)
   $(info ++++Compiling for uZed)
   $(info ======================)
   LFLAGS   := -g $(DEF_LFLAGS)
#   CFLAGS   := -g -O0 $(DEF_CFLAGS)
   CFLAGS   := $(OPT_FLAGS) -march=armv7-a $(DEF_CFLAGS) 
#   CFLAGS   := -O1 -march=armv7-a -mcpu=cortex-a9 $(DEF_CFLAGS)
#   CFLAGS   := -g -march=armv7-a -O0 $(DEF_CFLAGS) $(DMA_TYPE)
# Server Build
   BOOST_LFLAGS := $(BOOST_LFLAGS_CTB)
   BOOST_CFLAGS := $(BOOST_CFLAGS_CTB)
else
   $(info +++ Compiling for Server)
   CC         := g++ 
   CFLAGS   := $(DEF_CFLAGS) $(OPT_FLAGS) -fpermissive 
   LFLAGS   := $(DEF_LFLAGS)
   BOOST_LFLAGS := $(BOOST_LFLAGS_LINUX)
   BOOST_CFLAGS := $(BOOST_CFLAGS_LINUX)
endif

.PHONY: all clean boost

#all: standalone_run_boost standalone_run
all: standalone_run

boost: standalone_run_boost

root: standalone_run_root

clean:
	rm -f standalone_run standalone_run_boost

standalone_run: standalone_run.cpp PracticalSocket.cpp
	$(CC) -I./ $(CFLAGS) -o $@ $? $(LFLAGS)

standalone_run_boost: ctbdatareceiver.cpp ctbclient.cpp standalone_run_boost.cpp 
	$(CC) -I./ $(CFLAGS) $(BOOST_CFLAGS) -o $@ $^ $(LFLAGS) $(BOOST_LFLAGS)

standalone_run_root: PracticalSocket.cpp standalone_run_root.cpp
	$(CC) -I./ $(CFLAGS) $(ROOT_CFLAGS) $(BOOST_CFLAGS) -o $@ $^ $(LFLAGS) $(ROOT_LFLAGS) $(BOOST_LFLAGS)
 

