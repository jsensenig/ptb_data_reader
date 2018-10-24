#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h>
#include <vector>
// Configuration regs
#define BASEADDR 0x43C00000
#define HIGHADDR 0x43C0FFFF
#define CH0_OFFSET 0x0
#define CH1_OFFSET 0x4

int g_mem_fd;
void *mmapaddr_config;

// -- Map a physical address into a virtual one
void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr) {
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

void CloseMem () {
        if (g_mem_fd != -1)
                close(g_mem_fd);
}

// -- Release a physical memory map
void UnmapPhysMemory(void * address, size_t size) {
  // If it is open
  munmap(address,size);
}

// -- Read from a memory address (or memory mapped register)
uint32_t Xil_In32(uint32_t Addr) {
  return *(volatile uint32_t *) Addr;
}


//-- Write into a memory address (or memory mapped register)
void Xil_Out32(uint32_t OutAddress, uint32_t Value) {
  *(volatile uint32_t *) OutAddress = Value;
}

void SigIntHandler() {
  printf("Received a signal\n");
  printf("Stopping the run...\n");
 // run_ctrl = false;

}

int readgpio() {

  //Map GPIO
  printf("Mapping the GPIO ...\n");
  mmapaddr_config = MapPhysMemory(BASEADDR,HIGHADDR);
  printf("Received pointer to GPIO at %p\n",mmapaddr_config);

  std::vector<uint32_t> addr(4, 0);
  std::vector<uint32_t> value(4, 0);

///////////////////////////////////////////////////////////////////
  // -- Enter the values and addresses in the vector

  addr[0] = 63;
  value[0] = 42;
  addr[1] = 64;
  value[1] = 72;
  addr[2] = 65;
  value[2] = 26;
  addr[3] = 66;
  value[3] = 62;

 // - Enter "true" to reset the timing endpoint
 bool e_rst = true;

///////////////////////////////////////////////////////////////////

  printf("Programming configuration \n");

  // Write to the registers
  for(int i = 0; i < addr.size(); i++) {
    Xil_Out32((uint32_t)(mmapaddr_config+CH1_OFFSET*addr.at(i)), value.at(i));
    printf("Register Addr %i : [ %u ] [ 0x%X ] \n", addr.at(i), value.at(i), value.at(i));
    usleep(500 *1000);
  }

  printf("\n ================================================= \n");
  printf(" Read back the configuration registers \n \n");

  //Read back the configuration registers
  for (int i = 0; i < 102; i++ ) {
      uint32_t config = Xil_In32((uint32_t)(mmapaddr_config + CH1_OFFSET*i));
      printf("Register %i : [ %u ] [ 0x%X ] \n",i, config, config);
      usleep(50 *1000);
  }

  // Timing endpoint reset
  if (e_rst) {
    printf("\n Resetting timing endpoint \n \n");
    Xil_Out32((uint32_t)(mmapaddr_config+CH1_OFFSET*70), 2); 
    sleep(1);
    //Xil_Out32((uint32_t)(mmapaddr_config+CH1_OFFSET*70), 0); 
  }

  printf("Shutting down... \n");

  //Reset everything to 0 before shutdown
  //Xil_Out32((uint32_t)(mmapaddr+GPIO_CH0_OFFSET),0); //Don't reset for now
  sleep(1.0);

  //Close GPIO
  CloseMem();

  printf("Finished!! \n");

  return 0;
}

int main(int argc, const char* argv[]) {

  if (readgpio()) {
    printf("Failure \n");
  }
}

