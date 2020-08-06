CXX      = gcc -std=c++11
#CFLAGS  = -I/.
CXXFLAGS := -Wall -Wextra -pedantic-errors
#CXXFLAGS  = -std=c++11
#LDFLAGS = -L/Users/jsen/jsoncpp/build/debug/src/lib_json -ljsoncpp

all: test_queue clean

test_queue: test_queue.o
	$(CXX) -o $@ $^ 

test_queue.o: test_queue.cc ProducerConsumerQueue.h
	$(CXX) -c $(CXXFLAGS) $<

.PHONY: clean cleanest

clean:
	rm *.o

cleanest: clean
	rm test_queue
