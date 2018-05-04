# Variables
MACHINE    := $(shell uname -m)
CC         := g++
DEF_CFLAGS := -std=c++11 -Wall -I. 
BOOST_CFLAGS := -I/usr/local/boost/1.66.00/include
DEF_LFLAGS := -lpthread
BOOST_LFLAGS := /usr/local/boost/1.66.00/lib/libboost_thread.a /usr/local/boost/1.66.00/lib/libboost_system.a

# TPB Build
ifeq ($(MACHINE), armv7l)
   $(info ======================)
   $(info ++++Compiling for uZed)
   $(info ======================)
    LFLAGS   := -g $(DEF_LFLAGS)
#   CFLAGS   := -g -O0 $(DEF_CFLAGS)
   CFLAGS   := -O3 -march=armv7-a $(DEF_CFLAGS) 
#   CFLAGS   := -O1 -march=armv7-a -mcpu=cortex-a9 $(DEF_CFLAGS)
#   CFLAGS   := -g -march=armv7-a -O0 $(DEF_CFLAGS) $(DMA_TYPE)
# Server Build
else
   $(info +++ Compiling for Server)
   CC         := g++ 
   CFLAGS   := $(DEF_CFLAGS) -O3 -fpermissive 
   LFLAGS   := $(DEF_LFLAGS) 
endif

.PHONY: all clean

#all: standalone_run_boost standalone_run
all: standalone_run


clean:
	rm standalone_run  #standalone_run_boost

standalone_run: standalone_run.cpp PracticalSocket.cpp
	$(CC) -I./ $(CFLAGS) -o $@ $? $(LFLAGS)

#standalone_run_boost: standalone_run_boost.cpp
#	$(CC) -I./ $(CFLAGS) $(BOOST_CFLAGS) -o $@ $? $(BOOST_LFLAGS)

