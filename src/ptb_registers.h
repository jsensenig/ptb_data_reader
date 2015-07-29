/*
 * ptb_registers.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Nuno Barros <nfbarros@hep.upenn.edu>
 * Description: This file contains the physical memory addresses of several registers
 *              necessary for the operation of the PTB.
 */

#ifndef PTB_REGISTERS_H_
#define PTB_REGISTERS_H_

#include <cstdint>
extern "C" {
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

};
#include <new>

#include "Logger.h"

/**
 * Structure containing the information of the configuration registers.
 * This structure is populated further down in the file and then used in the software.
 */
typedef struct mapped_register {
    uint32_t dev_id;
    uint32_t base_addr;
    uint32_t high_addr;
    uint32_t n_registers;
    uint32_t *addr_offset;
} mapped_register;

uint32_t g_n_registers_;

mapped_register conf_reg;

/**
 * MapPhysMemory
 */

void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr) {
  int memfd;
  void *mapped_addr;
  off_t dev_base = base_addr;
  memfd = open("/dev/mem",O_RDWR | O_SYNC);
  if (memfd == -1) {
    Log(error,"Failed to map register [0x%08X 0x%08X].",base_addr,high_addr);
    return NULL;
  }
  // Map into user space the area of memory containing the device
  mapped_addr = mmap(0, (high_addr-base_addr), PROT_READ | PROT_WRITE, MAP_SHARED, memfd, dev_base & ~(high_addr-base_addr-1));
  if (static_cast<int>(mapped_addr) == -1) {
    Log(error,"Failed to map register [0x%08X 0x%08X] into virtual address.",base_addr,high_addr);
    return NULL;
  }

  return mapped_addr;

}

//void WriteRegister()

//uint32_t ReadRegister()

#endif /* PTB_REGISTERS_H_ */
