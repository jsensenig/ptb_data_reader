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
#include <new>
extern "C" {
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
}

#include "Logger.h"

namespace ptb {

namespace config {

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

extern int g_mem_fd;

extern mapped_register conf_reg;

void setup_ptb_registers();

}
}
//void WriteRegister()

//uint32_t ReadRegister()

#endif /* PTB_REGISTERS_H_ */
