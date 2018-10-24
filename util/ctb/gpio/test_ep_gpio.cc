#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h> 

#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8

#define GPIO1_BASEADDR 0x41210000
#define GPIO1_HIGHADDR 0x4121FFFF

#define GPIO2_BASEADDR 0x41220000
#define GPIO2_HIGHADDR 0x4122FFFF

#define BASEADDR 0x43C00000
#define HIGHADDR 0x43C0FFFF
#define CH0_OFFSET 0x0
#define CH1_OFFSET 0x4



int g_mem_fd;
void *mmap0;
void *mmapaddr;
void *mmapaddr1;
//void *mmapaddr2;
bool run_ctrl;

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
  run_ctrl = false;

}


///////////////////////////////////////////////////////////////////

int readgpio() {

  //Map GPIO
  printf("Mapping the GPIO ...\n");
  mmapaddr = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
  printf("Received pointer to GPIO at %p\n",mmapaddr);

 // Xil_Out32((uint32_t)mmapaddr + GPIO_CH0_OFFSET, 0);
  sleep(3);
  printf("Entering aquisition loop \n");
 // Xil_Out32((uint32_t)mmapaddr + GPIO_CH0_OFFSET, 0);
  while (true) {
    //Read something!
    //uint32_t word0 = Xil_In32((uint32_t)mmapaddr + GPIO_CH0_OFFSET);
    uint32_t word1 = Xil_In32((uint32_t)mmapaddr + GPIO_CH1_OFFSET);
    //printf(" PLL Lock Count %u \n", word1);
    printf(" Ack Count %u \n", word1);
    //Sleep for 1/2 sec
    usleep(500 *1000);
  }

  printf("Shutting down... \n");

  sleep(1.0); 

  //Close GPIO
  CloseMem();

  printf("Finished!! \n");

  return 0;
}

int main(int argc, const char* argv[]) {

  if (!readgpio()) { 
    printf("Failure \n");
  }   
}

