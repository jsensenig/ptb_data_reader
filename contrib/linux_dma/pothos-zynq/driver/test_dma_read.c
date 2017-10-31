// Copyright (c) 2014-2014 Josh Blum
// SPDX-License-Identifier: BSL-1.0

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include "pothos_zynq_dma_driver.h"
#include <signal.h>

#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8


pzdud_t *s2mm;
double accumulated_time;
unsigned int number_transfers;
unsigned int tot_volume_bytes;
void *mmapaddr;

bool run_ctrl;
int g_mem_fd; // File descriptor pointing to the mmap buffer

struct timespec diff(struct timespec start, struct timespec end)
{
  struct timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
}



// -- Map a physical address into a virtual one
void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr) {
  //int memfd;
  void *mapped_addr;
  off_t dev_base = base_addr;
  if (g_mem_fd == 0) {
    g_mem_fd = open("/dev/mem",O_RDWR | O_SYNC);
    if (g_mem_fd == -1) {
      printf("Failed to map register [0x%08X 0x%08X].",base_addr,high_addr);
      g_mem_fd = 0;
      return NULL;
    }
  }
  // Map into user space the area of memory containing the device
  mapped_addr = mmap(0, (high_addr-base_addr), PROT_READ | PROT_WRITE, MAP_SHARED, g_mem_fd, dev_base & ~(high_addr-base_addr-1));
  if ( (int32_t)mapped_addr == -1) {
    printf("Failed to map register [0x%08X 0x%08X] into virtual address.",base_addr,high_addr);
    mapped_addr = NULL;
  }

  return mapped_addr;

}

// -- Release a physical memory map
void UnmapPhysMemory(void * address, size_t size) {
  // If it is open
  munmap(address,size);
}

// -- Read from a memory address (or memory mapped register)
uint32_t Xil_In32(uint32_t Addr)
{
  return *(volatile uint32_t *) Addr;
}

// -- Write into a memory address (or memory mapped register)
void Xil_Out32(uint32_t OutAddress, uint32_t Value)
{
  *(volatile uint32_t *) OutAddress = Value;
}


void SigIntHandler() {
  printf("Received a signal\n");
  printf("Stopping the run...\n");
  run_ctrl = false;

}



int test(const int index)
{

  

  uint32_t i = 0;
  const uint32_t nbuffs = 4096;
  const uint32_t buffsize = 4096; // in bytes
  uint32_t addr[nbuffs];
  int ret = 0;
  printf("Begin axi stream data transfer test %d\n", index);
  // -- Register the signals to be captured when stopping the application
  signal(SIGINT,SigIntHandler);
  signal(SIGTERM,SigIntHandler);

  // -- Map the GPIO
  // -- The GPIO has two purposes:
  // -- Control the data generation execution
  // -- Read the number of entries being kept in the data FIFO
  printf("Mapping the GPIO ...\n");
  mmapaddr = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
  printf("Received pointer to GPIO at %p\n",mmapaddr);
    
  /////////////////////////// init the DMA ///////////////////////////
  printf("Create DMA channels\n");
  s2mm = pzdud_create(index, PZDUD_S2MM);
  if (s2mm == NULL) return EXIT_FAILURE;

  printf("Allocate DMA channels\n");
  ret = pzdud_alloc(s2mm, nbuffs, buffsize);
  printf("pzdud_alloc(s2mm) %d\n", ret);
  if (ret != PZDUD_OK) return EXIT_FAILURE;

  // -- Initialize the buffer table. This creates pointers in virtual memory
  //    that will allow to access the contents transferred from the DMA
  //    These pointers remain immutable since allocation to release
  printf("Initializing buffer table...\n");
  for (i = 0; i < nbuffs; i++) {
    addr[i] = (uint32_t)pzdud_addr(s2mm,i);
    printf("Buffer %u : 0x%X\n",i,addr[i]);
  }
  
  // -- Initialize the system
  ret = pzdud_init(s2mm, true);
  printf("pzdud_init(s2mm) %d\n", ret);
  if (ret != PZDUD_OK) return EXIT_FAILURE;
  
  // Sleep the DMA for a bit. This actually uses an interrupt, meaning that
  // while sleeping the CPU will be idle
  ret = pzdud_wait(s2mm, 10);
  // 
  printf("Received %d on wait...\n",ret);
  /* if (ret != PZDUD_ERROR_TIMEOUT) */
  /* { */
  /*     printf("Fail pzdud_wait(s2mm) %d\n", ret); */
  /*     return EXIT_FAILURE; */
  /* } */

  // -- A bunch of variables to control the data flow and volumes involved
  size_t len;
  int handle;
  
  /* sleep(1); */
  
  size_t counter = 0, j = 0;
  uint32_t *data;
  printf("Entering acquisition loop...\n");
  
  double cur_tdiff;
  accumulated_time = 0.0;
  number_transfers = 0;
  tot_volume_bytes = 0;
  
  // -- Let it run for a bit, and only latter enable it
  struct timespec time1, time2;
  uint32_t prev_last = 0,cur_first = 0;
  
  // -- Just sleep for a little bit and then get on to work
  sleep(3);
  
  
  run_ctrl = true;
  printf("Enabling the data stream...press CTRL+C to stop the test...\n");
  Xil_Out32((uint32_t)(mmapaddr+GPIO_CH1_OFFSET),1);

  while(run_ctrl) {
    /* iter++; */
    /* // -- Just iterate for a while  */
    /* if (iter == iter_start) { */
    /*   printf("Enabling the data stream...\n"); */
    /*   Xil_Out32((uint32_t)(mmapaddr+0x8),1); */
    /* } */

    // -- Grab a DMA buffer
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    handle = pzdud_acquire(s2mm, &len);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    cur_tdiff = (double)diff(time1,time2).tv_nsec/1000.0;

    // Deal with the case that there is no data transferred yet
    if (handle < 0)
      {
	// Failed because there are no complete
	// transactions in RAM yet 
	// -- This is not really a failure
	// Could avoid it by implementing a kernel
	// signaling to be caught here
	if (handle == PZDUD_ERROR_COMPLETE) {
	  // -- Set the system to sit for a few usec 
	  //pzdud_wait(s2mm,1);
	  continue;
	}
	// Failed because it timed out
	// (meaning that the DMA didn't answer back)
	// This is dangerous and usually means that
	// something went belly up
	if (handle == PZDUD_ERROR_TIMEOUT) {
	  printf("Fail with timeout\n");
	}
	pzdud_halt(s2mm);
	pzdud_free(s2mm);
	return EXIT_FAILURE;
      }
    // We have a good handle.  
    accumulated_time += cur_tdiff;
    number_transfers += 1;
    tot_volume_bytes += len;

    // Store the first word in the packet
    cur_first = *((uint32_t*)(addr[handle]));
    /* printf("iter %u recv %zu bytes handle %d (time %lf us) [fifo : %u] [%u %u]\n", */
    /* 	   counter, */
    /* 	   len, */
    /* 	   handle, */
    /* 	   cur_tdiff, */
    /* 	   Xil_In32((uint32_t)mmapaddr + GPIO_CH0_OFFSET), */
    /* 	  cur_first, */
    /* 	   *((uint32_t*)(addr[handle]+(((len/16)-1)*4*sizeof(uint32_t))))); */

    // If this is the first iteration just assign the last value
    if (prev_last == 0) {
      prev_last = *((uint32_t*)(addr[handle]+(((len/16)-1)*4*sizeof(uint32_t))));
    } else {
      // if not, check for data consistency
      // The first of this packet should increment the last by 1
      if (cur_first != prev_last+1) {
	printf("ERROR: Data consistency failure at iteration %u\n",counter);
	printf("       First entry %u while last entry of previous buffer %u\n",cur_first,prev_last);
	run_ctrl = false;
      }
      prev_last = *((uint32_t*)(addr[handle]+(((len/16)-1)*4*sizeof(uint32_t))));
    }

    
    // Print the first and last value
    // in the acquired buffer
    // If we are not losing data, the values should monotonically
    // increase without gaps from a buffer to the next
    /* printf("first [%p] %u last [%p] %u\n", */
    /* 	   (void*)addr[handle], */
    /* 	   *((uint32_t*)(addr[handle])), */
    /* 	   (void*)addr[handle]+(((len/16)-1)*4*sizeof(uint32_t)), */
    /* 	   *((uint32_t*)(addr[handle]+(((len/16)-1)*4*sizeof(uint32_t))))); */

    // Release the handle so that the
    // DMA can overwrite its contents
    pzdud_release(s2mm, handle, 0);

    // -- Test consistency

    counter++;
  } // counter;

  // -- If it reached this point, a stop was requested.

  // -- Print some statistics
  printf("Spent a total of %lf us in %u transfers\n",accumulated_time,number_transfers); 
  printf("Average transfer time %lf us\n",(accumulated_time/(double)number_transfers));
  printf("Estimated data volume %u bytes (%lf MB) (rate of %lf mbps)\n",
	 tot_volume_bytes,
	 ((double)tot_volume_bytes*1e-6),
	 ((double)tot_volume_bytes/(1024*1024))/((double)accumulated_time*1e-6));
  
  
  // -- Clean up the house and exit

  printf("Stopping the data stream\n");
  Xil_Out32((uint32_t)(mmapaddr+GPIO_CH1_OFFSET),0);

  printf("Halting the DMA...\n");
  
  ret = pzdud_halt(s2mm);
  printf("pzdud_halt(s2mm) %d\n", ret);
  if (ret != PZDUD_OK) return EXIT_FAILURE;
    
  printf("Free DMA channels\n");
  pzdud_free(s2mm);
  
  printf("Destroy DMA channels\n");
  pzdud_destroy(s2mm);
  
  printf("Done!\n");
  
  if (g_mem_fd != -1) {
    close(g_mem_fd);
  }
  return EXIT_SUCCESS;
}

int main(int argc, const char* argv[])
{
  if (test(0) != 0) return EXIT_FAILURE;
  return EXIT_SUCCESS;
}
