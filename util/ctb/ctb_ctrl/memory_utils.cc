
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <cstdio>

extern "C" {
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>  // implements int  open(const char *, int, ...);
#include <sys/mman.h> //mmap
};

//namespace util {
// -- File descriptor for memory mapping
int g_mem_fd;

// -- Map a physical address into a virtual one
void *map_phys_mem(uint32_t base_addr, uint32_t high_addr) {
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
  if ( mapped_addr == MAP_FAILED) {
    printf("Failed to map register [0x%08X 0x%08X] into virtual address.\n",base_addr,high_addr);
    printf("Error returned : %d : %s\n",errno,strerror(errno));
    mapped_addr = NULL;
  }

  return mapped_addr;
}

void release_mem () {
  if (g_mem_fd != -1) {
    close(g_mem_fd);
    g_mem_fd = -1;
  }
}

// -- Release a physical memory map
void unmap_phys_mem(void * address, size_t size) {
  // If it is open
  munmap(address,size);
}
// -- Read from a memory address (or memory mapped register)
uint32_t read_reg32(uint32_t addr) {
  return *(volatile uint32_t *) addr;
}
//-- Write into a memory address (or memory mapped register)
void write_reg32(uint32_t addr, uint32_t val) {
  *(volatile uint32_t *) addr = val;
}



//};
