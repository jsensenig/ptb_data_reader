/*
 * util.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

extern "C" {
#include <stdint.h>
};

#include <ctime>
#include <string>

#include "util.h"
#include "Logger.h"
#include "ptb_registers.h"

uint32_t CreateMask(uint32_t begin, uint32_t end) {

  if (begin > end) {
    Log(error,"Trying to create an invalid mask [%u,%u]", begin ,end );
    return 0;
  }
  uint32_t mask = 0;
  for (uint32_t i = begin; i <= end; ++i) {
    mask |= 1 << i;
  }
  return mask;
}


uint32_t SetBitRange(uint32_t content, uint32_t value, uint32_t pos, uint32_t len) {
	uint32_t mask = (((uint32_t)1 << len)-1) << pos;
	uint32_t result = (content & ~mask) | (value & mask);
	return result;
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


/**
 * MapPhysMemory
 */
int g_mem_fd = 0;


void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr) {
  //int memfd;
  void *mapped_addr;
  off_t dev_base = base_addr;
  if (g_mem_fd == 0) {
    g_mem_fd = open("/dev/mem",O_RDWR | O_SYNC);
    if (g_mem_fd == -1) {
      Log(error,"Failed to map register [0x%08X 0x%08X].",base_addr,high_addr);
      g_mem_fd = 0;
      return NULL;
    }
  }
  // Map into user space the area of memory containing the device
  mapped_addr = mmap(0, (high_addr-base_addr), PROT_READ | PROT_WRITE, MAP_SHARED, g_mem_fd, dev_base & ~(high_addr-base_addr-1));
  if ( reinterpret_cast<int32_t>(mapped_addr) == -1) {
    Log(error,"Failed to map register [0x%08X 0x%08X] into virtual address.",base_addr,high_addr);
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


