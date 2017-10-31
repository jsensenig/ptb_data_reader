########################################################################
## Simple Makefile for cross compiling DMA loopback test application
########################################################################
CC=$(CROSS_COMPILE)gcc

INCLUDES=-I./ -I../kernel/

CFLAGS=-std=gnu99 -O3 $(INCLUDES)
#CFLAGS=-std=gnu99 -O2 $(INCLUDES)

LDFLAGS=-static

OBJ = loopback_test.o test_read.o

DEPS = \
	pothos_zynq_dma_driver.h \
	../kernel/pothos_zynq_dma_common.h

.PHONY: all

all: loopback_test test_read read_dma_v2 read_dma_v3 read_dma_example test_dma_read

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

loopback_test: loopback_test.o 
	$(CC) -o $@ $^ $(LDFLAGS)

test_read: test_read.o
	$(CC) -o $@ $^ $(LDFLAGS)

read_dma_v2: read_dma_v2.o
	$(CC) -o $@ $^ $(LDFLAGS)

read_dma_v3: read_dma_v3.o
	$(CC) -o $@ $^ $(LDFLAGS)

read_dma_example: read_dma_example.o
	$(CC) -o $@ $^ $(LDFLAGS)

test_dma_read: test_dma_read.o
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf *.o loopback_test test_read read_dma_v2 read_dma_v3 read_dma_example test_dma_read
