#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <csignal>
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
// Config memory mapped regs
#define CONFIG_BASEADDR 0x43C00000
#define CONFIG_HIGHADDR 0x43C0FFFF
#define CONFIG_CH0_OFFSET 0x0
#define CONFIG_CH1_OFFSET 0x4

using std::cout;
using std::endl;

// -- External stuff that should be built at link time

extern int g_mem_fd;
extern void *mmapaddr_timing;
extern void *mmapaddr_config;

// -- Map a physical address into a virtual one
extern void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr);
extern void release_mem();
extern void unmap_phys_mem(void * address, size_t size);
extern uint32_t read_reg32(uint32_t addr);
extern uint32_t write_reg32(uint32_t addr,uint32_t val);


// -- Should build something simple listening server



bool read_ = true;

void SigIntHandler(int param) {
  printf("Stopping status output...\n");
  read_ = false;
}



// Reset the timing endpoint
int ResetEP() {

///////////////////////////////////////////////////////////////////
// Connections in the GPIO slice block:
//    ext_cmd_en <= gpio_in(0); 
//    ext_cmd <= gpio_in(4 downto 1);
//    timing_ep_rst <= gpio_in(5);
//    pll_rst <= gpio_in(6);
//    timing_ep_addr <= gpio_in(14 downto 7);
//    timing_grp <= gpio_in(16 downto 15);

uint32_t timing_ep_reset = 1; //Reset the timing endpoint (1=reset, 0=no reset)
// Only using the reset for now
/*
uint32_t ext_command_en = 0; //Enable external trigger commands to the timing endpoint (1=enable, 0=disable)
uint32_t ext_command = 0x0; //Set the external trigger command, 0x8 - 0xF
uint32_t pll_rst = 0; //Reset the PLL (1=reset, 0=no reset
uint32_t timing_addr = 0x0; //Set the timing address, 0x0 for now will be 0xF0
uint32_t timing_group = 0x0; //Set the timing group, value unknown
*/
///////////////////////////////////////////////////////////////////

  //Enable the reset bit
  uint32_t gpio_output =  (0x20 & (timing_ep_reset << 5));
  // Reset the timing endpoint for a couple sec, then de-assert the reset bit
  Xil_Out32((uint32_t)mmapaddr0 + GPIO_CH0_OFFSET, gpio_output);
  sleep(2);

  //Turn off the endpoint reset
  gpio_output =  0x0;
  Xil_Out32((uint32_t)mmapaddr0 + GPIO_CH0_OFFSET, gpio_output);
  sleep(2);
  
  printf("Finished reseting endpoint! Wait a few seconds for it to recover. \n");
  sleep(3);
  printf("Now check the endpoint status. \n");

  // Check the endpoint status
  uint32_t reg = 91; // Timing status register
  uint32_t config = Xil_In32((uint32_t)(mmapaddr_config + (CONFIG_CH1_OFFSET * reg)));
  printf("Timing Status: Register %i : [ %u ] [ 0x%X ] \n", reg, (config >> 28), (config >> 28));
  sleep(1.0); 

  printf("Finished!! \n");
  return 0;
}

// Read from the timing status 
int ReadTimingStat() {

  signal(SIGINT, SigIntHandler);
  read_ = true;
  cout << "Use 'ctrl+c' to exit timing status output." << endl;
  cout << endl;

  // Read the timing status
  while (read_) {
    uint32_t reg = 91; // Timing status register
    uint32_t config = Xil_In32((uint32_t)(mmapaddr_config + (CONFIG_CH1_OFFSET * reg)));
    printf("Timing Status: Register %i : [ %u ] [ 0x%X ] \n", reg, (config >> 28), (config >> 28));
    sleep(1);
  }

  printf("Finished!! \n");
  return 0;
}


int main(int argc, const char* argv[]) {

  // Map GPIO and Config regs addresses
  printf("Mapping GPIO_0 and Config Registers ...\n");
  mmapaddr0 = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
  mmapaddr_config = MapPhysMemory(CONFIG_BASEADDR,CONFIG_HIGHADDR);
  printf("Received pointer to GPIO at %p\n", mmapaddr0);
  printf("Received pointer to Config Regs at %p\n", mmapaddr_config);  

  // User control
  // std::cout.setf(std::ios::unitbuf);
  enum commands {read_stat=1,reset=2,quit=3};
  int command = -1;
  bool stop_req_ = false;

  while(!stop_req_) {
    cout << "### Select command ### " << endl;
    cout << " 1 : Read Timing Endpoint Status" << endl;
    cout << " 2 : Reset Endpoint" << endl;
    cout << " 3 : Quit" << endl;

    std::cin >> command;
    switch(command) {
      case read_stat:
        if (ReadTimingStat()) {           
          printf("Failure \n");
        }   
        break;
      case reset:
        if (ResetEP()) {                  
          printf("Failure \n");
        }   
        break;
      case quit:
        stop_req_ = true;
        break;
      default:
        cout << "Wrong option (" << command << ")." << endl;
        break;
    }
  }
  cout << "### Stop requested. ###" << endl;
  // Shut things down
  sleep(1);
  
  printf("Shutting down... \n");
  sleep(1.0);

  //Close GPIO
  CloseMem();

}

