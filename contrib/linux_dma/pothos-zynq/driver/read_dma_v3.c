// Copyright (c) 2014-2014 Josh Blum
// SPDX-License-Identifier: BSL-1.0

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include "pothos_zynq_dma_driver.h"
#include <signal.h>

#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF

pzdud_t *s2mm;
double accumulated_time;
unsigned int number_transfers;
unsigned int tot_volume_bytes;
void *mmapaddr;


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

int g_mem_fd;

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
  printf("Stopping the data stream\n");
  Xil_Out32((uint32_t)(mmapaddr+0x8),0);
  printf("Halting...\n");
  int ret;
  ret = pzdud_halt(s2mm);
  printf("pzdud_halt(s2mm) %d\n", ret);

  printf("Spent a total of %lf us in %u transfers\n",accumulated_time,number_transfers);
  printf("Average transfer time %lf us\n",(accumulated_time/(double)number_transfers));
  printf("Estimated data volume %u bytes (rate of %lf mbps)\n",tot_volume_bytes,((double)tot_volume_bytes/(1024*1024))/((double)accumulated_time*1e-6));
  printf("Freeing buffers...\n");
  pzdud_free(s2mm);
  printf("Destroying device instance...\n");
  pzdud_destroy(s2mm);

  printf("Unmapping the GPIO\n");
  UnmapPhysMemory(mmapaddr,(GPIO_HIGHADDR-GPIO_BASEADDR));
  printf("All Done!\n");
  exit(0);
}



int test(const int index)
{

  uint32_t i = 0;
  const uint32_t nbuffs = 30;
  const uint32_t buffsize = 4096; // in bytes
  uint32_t addr[nbuffs];
  int ret = 0;
  printf("Begin pothos axi stream userspace test %d\n", index);

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

  // -- In case we wanted to also send data to memory
  // -- NFB: Might implement this later on to configure the board
  // -- This would save a substantial amount of BRAM in the uZed
  /* pzdud_t *mm2s = pzdud_create(index, PZDUD_MM2S); */
  /* if (mm2s == NULL) return EXIT_FAILURE; */

  /////////////////////////// allocate ///////////////////////////
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
  /* ret = pzdud_alloc(mm2s, 4, 4096); */
  /* printf("pzdud_alloc(mm2s) %d\n", ret); */
  /* if (ret != PZDUD_OK) return EXIT_FAILURE; */
  
  /////////////////////////// init ///////////////////////////
  // -- Initialize the system
  ret = pzdud_init(s2mm, true);
  printf("pzdud_init(s2mm) %d\n", ret);
  if (ret != PZDUD_OK) return EXIT_FAILURE;
  
  /* ret = pzdud_init(mm2s, true); */
  /* printf("pzdud_init(mm2s) %d\n", ret); */
  /* if (ret != PZDUD_OK) return EXIT_FAILURE; */
  
  
  //expect a timeout here
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

    //dont expect a timeout here
    /* ret = pzdud_wait(mm2s, 100); */
    /* if (ret != PZDUD_OK) */
    /* { */
    /*     printf("Fail pzdud_wait(mm2s) %d\n", ret); */
    /*     return EXIT_FAILURE; */
    /* } */

    size_t len;
    int handle;
    /* handle = pzdud_acquire(mm2s, &len); */
    /* if (handle < 0) */
    /* { */
    /*     printf("Fail pzdud_acquire(mm2s) %d\n", handle); */
    /*     return EXIT_FAILURE; */
    /* } */
    /* printf("available %zu bytes\n", len); */
    /* pzdud_release(mm2s, handle, 64); */

    /* sleep(1); */
    size_t counter = 0, j = 0;
    uint32_t *data;
    printf("Entering acquisition loop...\n");

    size_t prev_occ = 0;
    uint32_t cur_occ = 0;
    uint32_t previous_entry;
    uint32_t current_entry; 
    double cur_tdiff;
    accumulated_time = 0.0;
    number_transfers = 0;
    // -- Let it run for a bit, and only latter enable it
    const uint32_t iter_start = 500; // only after these many iterations we will start working properly
    uint32_t iter = 0;
    struct timespec time1, time2;
    while(true) {
      
      //struct timeval tv1,tv2;
      /* cur_occ = Xil_In32((uint32_t)mmapaddr); */
      /* if (cur_occ != prev_occ) { */
      /* 	printf("GPIO report before : %u \n",cur_occ); */
      /* 	prev_occ = cur_occ; */
      /* } */
      iter++;
      /* if (iter < iter_start ) { */
      /* 	printf("%u...\n",iter); */
      /* } */
      if (iter == iter_start) {
	printf("Enabling the data stream...\n");
	Xil_Out32((uint32_t)(mmapaddr+0x8),1);
      }
      //gettimeofday(&tv1,NULL);
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
      handle = pzdud_acquire(s2mm, &len);
      //gettimeofday(&tv2,NULL);
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
      cur_tdiff = (double)diff(time1,time2).tv_nsec/1000.0;
      
      if (handle < 0)
	{
	  
	  if (handle == -7) {
	    pzdud_wait(s2mm,1);
	    continue;
	    //printf("Fail completed transactions\n");
	    //continue;
	  }
	  //printf("Fail pzdud_acquire(s2mm) %d\n", handle);
	  if (handle == -2) {
	    printf("Fail with timeout\n");
	  }
	  pzdud_halt(s2mm);
	  pzdud_free(s2mm);
	  return EXIT_FAILURE;
	}
      //printf("iter %u recv %zu bytes into handle %d (time %f)\n", counter,len,handle,(double)(tv2.tv_usec - tv1.tv_usec));
      
      accumulated_time += cur_tdiff;
      number_transfers += 1;
      printf("iter %u recv %zu bytes into handle %d (time %lf us) [fifo %u]\n",counter,len,handle,cur_tdiff,Xil_In32((uint32_t)mmapaddr));
      
      /* printf("GPIO report after : %u \n",Xil_In32((uint32_t)mmapaddr)); */

      // Here's the tricky part: We now have multiple 128 bit words being streamed
      // And we want them to stream properly
      
      printf("first [%p] %u last [%p] %u\n",
	     (void*)addr[handle],
	     *((uint32_t*)(addr[handle])),
	     (void*)addr[handle]+(((len/16)-1)*4*sizeof(uint32_t)),
	     *((uint32_t*)(addr[handle]+(((len/16)-1)*4*sizeof(uint32_t)))));
      /* for (i = 0; i < (len/16); i++) { */
      /* 	for (j = 0; j < 4; j++) { */
      /* 	  printf("%X (%u) ",(addr[handle]+i*sizeof(uint32_t)),*((uint32_t*)(addr[handle]+(i*4*sizeof(uint32_t))+j*sizeof(uint32_t)))); */
      /* 	} */
      /* 	printf("\n"); */
      /* } */
      pzdud_release(s2mm, handle, 0);
      counter++;
    } // counter;
    /////////////////////////// halt ///////////////////////////
    ret = pzdud_halt(s2mm);
    printf("pzdud_halt(s2mm) %d\n", ret);
    if (ret != PZDUD_OK) return EXIT_FAILURE;
    
    /* ret = pzdud_halt(mm2s); */
    /* printf("pzdud_halt(mm2s) %d\n", ret); */
    /* if (ret != PZDUD_OK) return EXIT_FAILURE; */

    /////////////////////////// free ///////////////////////////
    printf("Free DMA channels\n");
    pzdud_free(s2mm);
    /* pzdud_free(mm2s); */

    /////////////////////////// cleanup ///////////////////////////
    printf("Destroy DMA channels\n");
    pzdud_destroy(s2mm);
    /* pzdud_destroy(mm2s); */

    printf("Done!\n");

    return EXIT_SUCCESS;
}

int main(int argc, const char* argv[])
{
    if (test(0) != 0) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
