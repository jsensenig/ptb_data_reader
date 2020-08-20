#CXX      = g++ -pthread -std=c++11 -m32
CXX      = g++ -pthread -std=c++11
CFLAGS  = -I. -I../../src/ -I../../ -I../../contrib/linux_dma/pothos-zynq/driver -I../../contrib/linux_dma/pothos-zynq/kernel -I/usr/include/
CXXFLAGS := -Wall -Wextra -pedantic-errors 
CC = g++
LFLAGS = -lpthread -lzmq

OBJ := ../../.obj
PTB_DIR := ../../src
PTB_SRC := ../../src/*.cpp
PTB_OBJ := $(PTB_DIR)/%.cpp,$(OBJ)/%.o,$(PTB_SRC)

all: dma_test zmq_test clean

dma_test: dma_test.o 
	$(CXX) $(CFLAGS) $(OBJ)/* -o $@ $^ $(LFLAGS) 

dma_test.o: dma_test.cpp
	$(CXX) $(CFLAGS) -c $(CXXFLAGS) $<

zmq_test: zmq_test.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS) 

zmq_test.o: zmq_test.cpp
	$(CXX) $(CFLAGS) -c $(CXXFLAGS) $<


.PHONY: clean

clean:
	rm *.o
	rm zmq_test
	rm dma_test


