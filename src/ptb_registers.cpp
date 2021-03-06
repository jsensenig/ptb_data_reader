/*
 * ptb_registers.cpp
 *
 *  Created on: Dec 21, 2015
 *      Author: nbarros
 *      03-10-2018: NFB: Changed the number of registers to 256, as several more are now being used.
 */

#include "ptb_registers.h"
namespace ptb {
namespace config {

  mapped_register conf_reg;

void setup_ptb_registers() {
  conf_reg.dev_id = 0;
  conf_reg.base_addr = 0x43C00000;
  conf_reg.high_addr = 0x43C0FFFF;
  conf_reg.n_registers = 256;
  conf_reg.addr_offset = new uint32_t[conf_reg.n_registers];
  unsigned int i = 0;
  for (i = 0; i < conf_reg.n_registers; ++i) {
   conf_reg.addr_offset[i] = i*4;
    Log(debug,
        "Setting configuration register (%u) to address 0x%08X + 0x%08X =  [0x%08X].",
        i,
        conf_reg.base_addr,
        conf_reg.addr_offset[i],
        conf_reg.base_addr+conf_reg.addr_offset[i]);
  }
  Log(debug,"Configuration registers set.");
}

  // Map a single GPIO for general config use
  mapped_register gpio_reg;

void setup_ctb_gpio() {
  gpio_reg.dev_id = 0;
  gpio_reg.base_addr = 0x41200000;
  gpio_reg.high_addr = 0x4120FFFF;
  gpio_reg.n_registers = 1;
  gpio_reg.addr_offset = new uint32_t[gpio_reg.n_registers];
  unsigned int i = 0;
  for (i = 0; i < gpio_reg.n_registers; ++i) {
   gpio_reg.addr_offset[i] = i*4;
    Log(debug,
        "Setting GPIO register (%u) to address 0x%08X + 0x%08X =  [0x%08X].",
        i,
        gpio_reg.base_addr,
        gpio_reg.addr_offset[i],
        gpio_reg.base_addr+gpio_reg.addr_offset[i]);
  }
  Log(debug,"GPIO registers set.");
}

#ifdef ARM_MMAP
void SetupDataRegisters() {
  data_reg.dev_id = 0;
  data_reg.base_addr = 0x43C10000;
  data_reg.high_addr = 0x43C1FFFF;
  data_reg.n_registers = 6;
  data_reg.addr_offset = new uint32_t[data_reg.n_registers];
  unsigned int i = 0;
  for (i = 0; i < data_reg.n_registers; ++i) {
   data_reg.addr_offset[i] = i*4;
    Log(debug,
        "Setting data register (%u) to address 0x%08X + 0x%08X =  [0x%08X].",
        i,
        data_reg.base_addr,
        data_reg.addr_offset[i],
        data_reg.base_addr+data_reg.addr_offset[i]);
  }
  Log(debug,"Data registers set.");
}
  
#endif
}
}
