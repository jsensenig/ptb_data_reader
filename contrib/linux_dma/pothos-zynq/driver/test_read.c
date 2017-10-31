// Copyright (c) 2014-2014 Josh Blum
// SPDX-License-Identifier: BSL-1.0

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include "pothos_zynq_dma_driver.h"
#include <signal.h>
//#include <sys/time.h>

pzdud_t *s2mm;
double accumulated_time;
unsigned int number_transfers;

void SigIntHandler() {
  printf("Received a signal\n");
  printf("halting...\n");
  int ret;
  ret = pzdud_halt(s2mm);
  printf("pzdud_halt(s2mm) %d\n", ret);

  printf("Spent a total of %lf us in %u transfers\n",accumulated_time,number_transfers);
  printf("Average transfer time %lf us\n",(accumulated_time/(double)number_transfers));

  printf("Freeing buffers...\n");
  pzdud_free(s2mm);
  printf("Destroying device instance...\n");
  pzdud_destroy(s2mm);
  printf("Done!\n");
  exit(0);
}

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


// -- Grab the functions to manipulate the GPIO
int g_mem_fd = 0;


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

  // not sure this can actually be done this way
  //FIXME: Check if /dev/mem can actually be closed after mmap
  //close(memfd);

  return mapped_addr;

}

void UnmapPhysMemory(void * address, size_t size, bool close_file) {
  // If it is open
  munmap(address,size);
  if (g_mem_fd && close_file) {
    close(g_mem_fd);
  }
}

uint32_t Xil_In32(uint32_t Addr)
{
  return *(volatile uint32_t *) Addr;
}



void Xil_Out32(uint32_t OutAddress, uint32_t Value)
{
  *(volatile uint32_t *) OutAddress = Value;
}




int test(const int index)
{

  uint32_t i = 0;
  const uint32_t nbuffs = 100;
  const uint32_t buffsize = 8192; //4096; // in bytes
  uint32_t addr[nbuffs];
    int ret = 0;
    printf("Begin pothos axi stream userspace test %d\n", index);

    signal(SIGINT,SigIntHandler);
    signal(SIGTERM,SigIntHandler);

    printf("Mapping the GPIO to track FIFO contents...\n");
    void *mmapaddr = MapPhysMemory(0x41200000,0x4120FFFF);
    printf("Received pointer to GPIO at %p\n",mmapaddr);
    
    /////////////////////////// init ///////////////////////////
    printf("Create DMA channels\n");
    s2mm = pzdud_create(index, PZDUD_S2MM);
    if (s2mm == NULL) return EXIT_FAILURE;

    /* pzdud_t *mm2s = pzdud_create(index, PZDUD_MM2S); */
    /* if (mm2s == NULL) return EXIT_FAILURE; */

    /////////////////////////// allocate ///////////////////////////
    printf("Allocate DMA channels\n");
    ret = pzdud_alloc(s2mm, nbuffs, buffsize);
    printf("pzdud_alloc(s2mm) %d\n", ret);
    if (ret != PZDUD_OK) return EXIT_FAILURE;

    printf("Initializing buffer table...\n");
    for (i = 0; i < nbuffs; i++) {
      addr[i] = (uint32_t)pzdud_addr(s2mm,i);
      printf("Buffer %u : 0x%X\n",i,addr[i]);
    }
    /* ret = pzdud_alloc(mm2s, 4, 4096); */
    /* printf("pzdud_alloc(mm2s) %d\n", ret); */
    /* if (ret != PZDUD_OK) return EXIT_FAILURE; */

    /////////////////////////// init ///////////////////////////
    ret = pzdud_init(s2mm, true);
    printf("pzdud_init(s2mm) %d\n", ret);
    if (ret != PZDUD_OK) return EXIT_FAILURE;

    /* ret = pzdud_init(mm2s, true); */
    /* printf("pzdud_init(mm2s) %d\n", ret); */
    /* if (ret != PZDUD_OK) return EXIT_FAILURE; */

    ////////////////////////// loopback ////////////////////////

    //expect a timeout here
    ret = pzdud_wait(s2mm, 10);
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
    while(true) {
      //struct timeval tv1,tv2;
      struct timespec time1, time2;
      cur_occ = Xil_In32((uint32_t)mmapaddr);
      if (cur_occ != prev_occ) {
	printf("GPIO report before : %u \n",cur_occ);
	prev_occ = cur_occ;
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
	    pzdud_wait(s2mm,100);
	    continue;
	    //printf("Fail completed transactions\n");
	    //continue;
	  }
	  printf("Fail pzdud_acquire(s2mm) %d\n", handle);
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
      printf("iter %u recv %zu bytes into handle %d (time %lf us)\n",counter,len,handle,cur_tdiff);
      
      printf("GPIO report after : %u \n",Xil_In32((uint32_t)mmapaddr));

      // Here's the tricky part: We now have multiple 128 bit words being streamed
      // And we want them to stream properly
      
      for (i = 0; i < (len/16); i++) {
	for (j = 0; j < 4; j++) {
	  printf("%X (%u) ",(addr[handle]+i*sizeof(uint32_t)),*((uint32_t*)(addr[handle]+(i*4*sizeof(uint32_t))+j*sizeof(uint32_t))));
	}
	printf("\n");
      }
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
