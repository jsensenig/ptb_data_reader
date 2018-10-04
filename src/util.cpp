/*
 * util.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

#include <cstdint>
#include <ctime>
#include <string>
#include <cstring>
#include <sstream>
#include <bitset>

#include "util.h"
#include "Logger.h"
#include "ptb_registers.h"

namespace ptb {
  namespace util {

        char llt_cop_tochar(const uint32_t val) {
      char res;
      switch(val) {
      case 0x1:
        res = '>';
        break;
      case 0x2:
        res = '=';
        break;
      case 0x4:
        res = '<';
        break;
      default:
        res = '?';
        break;
      }
      return res;
    }

    void print_bits(void* memstart, size_t nbytes) {

      std::cout << "The " << nbytes << "-byte chunk of memory beginning at "
          << static_cast<void*>(memstart) << " is : [" << std::endl;

      for(unsigned int i = 0; i < nbytes; i++) {
        std::cout << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
      }
      std::cout << "\n]" << std::endl;
    }

    std::string display_bits(void* memstart, size_t nbytes) {

      std::stringstream bitstr;
      bitstr << "The " << nbytes << "-byte chunk of memory beginning at "
          << static_cast<void*>(memstart) << " is : [";

      for(unsigned int i = 0; i < nbytes; i++) {
        bitstr << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
      }
      bitstr << "]";
      return bitstr.str();
    }


    uint32_t create_mask(const uint32_t &begin, const uint32_t &end) {
      if (begin > end) {
        Log(error,"Trying to create an invalid mask [%u,%u] | [0x%X,0x%X]", begin ,end,begin ,end );
        return 0;
      }
      uint32_t mask = 0;
      for (uint32_t i = begin; i <= end; ++i) {
        mask |= 1 << i;
      }
      return mask;
    }

    // -- This sets len bits starting at position pos
    uint32_t set_bit_range(const uint32_t &content, const uint32_t &value, const uint32_t &pos, const uint32_t &len) {
      // -- build a mask of len starting at pos
      uint32_t mask = (((uint32_t)1 << len)-1) << pos;
      uint32_t result = (content & ~mask) | (value & mask);
      return result;
    }

    const std::string current_DateTime() {
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


    void *map_physical_memory(const uint32_t &base_addr, const uint32_t &high_addr) {
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

      //parameters
      // -- I am using variables because I don't want to forget again
      // the defintion of each argument.
      void *p_addr = NULL;
      size_t len = high_addr-base_addr;
      int protection = PROT_READ | PROT_WRITE;
      int flags = MAP_SHARED;
      int fd = ptb::util::g_mem_fd;
      off_t offset = dev_base & ~(high_addr-base_addr-1);
      mapped_addr = mmap(p_addr,len,protection,flags,fd,offset);
      //  mapped_addr = mmap(0, (high_addr-base_addr), PROT_READ | PROT_WRITE, MAP_SHARED, g_mem_fd, dev_base & ~(high_addr-base_addr-1));
      if ( mapped_addr == MAP_FAILED) {
        Log(error,"Failed to map register [0x%08X 0x%08X] into virtual address.",base_addr,high_addr);
        Log(error,"Kernel message : %s",strerror(errno));
        mapped_addr = nullptr;
      }

      return mapped_addr;

    }
    void unmap_physical_memory(void * address, const size_t &size, bool close_file) {
      //void UnmapPhysMemory(void * address, size_t size, bool close_file) {
      // If it is open
      munmap(address,size);
      if (g_mem_fd && close_file) {
        close(g_mem_fd);
      }
    }

    // These are shamelessly imported from the Vivado examples
    uint32_t Xil_In32(uint32_t Addr)
    {
      return *(volatile uint32_t *) Addr;
    }


    void Xil_Out32(uint32_t OutAddress, uint32_t Value)
    {
      *(volatile uint32_t *) OutAddress = Value;
    }

  }
}
