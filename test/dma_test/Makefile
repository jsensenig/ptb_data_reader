CXX      = gcc -pthread -std=c++11
CFLAGS  = -I. -I/usr/include/
CXXFLAGS := -Wall -Wextra -pedantic-errors
CC = g++
LFLAGS = -lpthread -lzmq

all: dma_test zmq_test clean

dma_test: dma_test.o
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS) 

dma_test.o: dma_test.cpp
	$(CXX) $(CFLAGS) -c $(CXXFLAGS) $<

zmq_test: zmq_test.o
	$(CC) $(CFLAGS) -o $@ $^ $(LFLAGS) 

zmq_test.o: zmq_test.cpp
	$(CXX) $(CFLAGS) -c $(CXXFLAGS) $<




.PHONY: clean cleanest

clean:
	rm *.o

cleanest: clean
	rm dma_test
	rm zmq_test
