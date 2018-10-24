#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h> 

#define BASEADDR 0x43C00000
#define HIGHADDR 0x43C0FFFF
#define CH0_OFFSET 0x0
#define CH1_OFFSET 0x4

int g_mem_fd;
void *mmapaddr_config;

//void MapGPIO () {
//  printf("Mapping GPIO ... \n");
//  mmapaddr = MapPhysMemory(GPIO_BASEADDR, GPIO_HIGHADDR);
//  printf("Recieved pointer to GPIO at %p \n", mmapaddr);
//}

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
  //run_ctrl = false;

}


///////////////////////////////////////////////////////////////////

int readgpio() {

  //Map GPIO
  printf("Mapping the GPIO ...\n");
  mmapaddr_config = MapPhysMemory(BASEADDR,HIGHADDR);
  printf("Received pointer to GPIO at %p\n",mmapaddr_config);
  //MapGPIO();

  printf("Begin config dump \n");
  //sleep(1);

  int num_reg = 102;
  //Read something!
  while (true) {
    for (int i = 92; i < 139; i++ ) {
        uint32_t config = Xil_In32((uint32_t)(mmapaddr_config + CH1_OFFSET*i));
        printf("Register %i : [ %u ] [ 0x%X ] \n",i, config, config);
  
        usleep(50 *1000);
    }
  }

  //Xil_Out32((uint32_t)(mmapaddr_config + CH1_OFFSET*27),0x3F9A9FA);

  //printf("Dumping updated registers values \n");

  //for (int i = 0; i < num_reg; i++ ) {
  //    uint32_t config = Xil_In32((uint32_t)(mmapaddr_config + CH1_OFFSET*i));
  //    printf("Register %i : [ %u ] [ 0x%X ] \n",i, config, config);
  
  //    usleep(50 *1000);
  //}


  printf("Shutting down... \n");

  sleep(1.0); 

  //Close GPIO
  CloseMem();

  printf("Finished!! \n");

  return 1;
}

int main(int argc, const char* argv[]) {

  if (!readgpio()) { 
    printf("Failure \n");
  }   
}

