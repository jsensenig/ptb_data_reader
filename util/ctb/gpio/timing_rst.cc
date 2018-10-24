#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h> 
// GPIO 0
#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8

int g_mem_fd;
void *mmapaddr;
bool run_ctrl;

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

int readgpio() {

  //Map GPIO
  printf("Mapping the GPIO ...\n");
  mmapaddr = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
  printf("Received pointer to GPIO at %p\n",mmapaddr);

///////////////////////////////////////////////////////////////////
//    ext_cmd_en <= gpio_in(0); 
//    ext_cmd <= gpio_in(4 downto 1);
//    timing_ep_rst <= gpio_in(5);
//    pll_rst <= gpio_in(6);
//    timing_ep_addr <= gpio_in(14 downto 7);
//    timing_grp <= gpio_in(16 downto 15);

uint8_t ext_command_en = 0; //Enable external trigger commands to the timing endpoint (1=enable, 0=disable)
uint8_t ext_command = 0xC; //Set the external trigger command, 0x8 - 0xF
uint8_t timing_ep_reset = 1; //Reset the timing endpoint (1=reset, 0=no reset)
uint8_t pll_rst = 0; //Reset the PLL (1=reset, 0=no reset
uint8_t timing_addr = 0x0; //Set the timing address, 0x0 for now will be 0xF0
uint8_t timing_group = 0x0; //Set the timing group, value unknown

///////////////////////////////////////////////////////////////////

  //Throw everything into a single 32b word
  uint32_t gpio_output = (timing_group<<15) + (timing_addr<<7) + (pll_rst<<6) + (timing_ep_reset<<5)  + (ext_command<<1) + ext_command_en;
  //Program the GPIO
  Xil_Out32((uint32_t)mmapaddr + GPIO_CH0_OFFSET, gpio_output);

  sleep(2);
  //Turn off the PLL reset
  pll_rst = 0;
  gpio_output = (timing_group<<15) + (timing_addr<<7) + (pll_rst<<6) + (timing_ep_reset<<5)  + (ext_command<<1) + ext_command_en;
  Xil_Out32((uint32_t)mmapaddr + GPIO_CH0_OFFSET, gpio_output);
  sleep(2);
  //Turn off the endpoint reset
  timing_ep_reset = 0;
  gpio_output = (timing_group<<15) + (timing_addr<<7) + (pll_rst<<6) + (timing_ep_reset<<5)  + (ext_command<<1) + ext_command_en;
  Xil_Out32((uint32_t)mmapaddr + GPIO_CH0_OFFSET, gpio_output);
  sleep(2);
  
  printf("Finished programming GPIO! \n");
  sleep(1);

  printf("Shutting down... \n");

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

