#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h> 
// GPIO 1
#define GPIO1_BASEADDR 0x41220000
#define GPIO1_HIGHADDR 0x4122FFFF
#define GPIO1_CH0_OFFSET 0x0
#define GPIO1_CH1_OFFSET 0x8
// GPIO 2
#define GPIO2_BASEADDR 0x41230000
#define GPIO2_HIGHADDR 0x4123FFFF
#define GPIO2_CH0_OFFSET 0x0
#define GPIO2_CH1_OFFSET 0x8
// GPIO 3
#define GPIO3_BASEADDR 0x41240000
#define GPIO3_HIGHADDR 0x4124FFFF
#define GPIO3_CH0_OFFSET 0x0
#define GPIO3_CH1_OFFSET 0x8

int g_mem_fd;
void *mmapaddr1;
void *mmapaddr2;
void *mmapaddr3;
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
  mmapaddr1 = MapPhysMemory(GPIO1_BASEADDR,GPIO1_HIGHADDR);
  mmapaddr2 = MapPhysMemory(GPIO2_BASEADDR,GPIO2_HIGHADDR);
  mmapaddr3 = MapPhysMemory(GPIO3_BASEADDR,GPIO3_HIGHADDR);
  printf("Received pointer to GPIO_1 at %p\n",mmapaddr1);
  printf("Received pointer to GPIO_2 at %p\n",mmapaddr2);
  printf("Received pointer to GPIO_3 at %p\n",mmapaddr3);


  // For the GPIO output on GPIO_1 we have,
  // bits [4:0] tap value
  // bit    [5] load (load tap value into delay)  
  // bit    [6] check init (enables checking of the received PRBS)

  // For the GPIO inputs on GPIO_2 we have,
  // bits [23:0] error count low
  // bits [47:24] error count high (the number of errors in the received PRBS)  

  // For the GPIO inputs on GPIO_3 we have,
  // bits [23:0] cycle count low
  // bits [47:24] cycle count high (the number of errors in the received PRBS)  

  //Enable the check_init and load a tap value of 0x0
  uint32_t gpio_output = (1<<6) | (1<<5) | 0x0;
  //Program the GPIO
  Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, gpio_output);
  sleep(1);
  //Deassert everything, ready to go!
  gpio_output = (1<<6) | 0x0;
  Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, gpio_output);
  
  // Loop over the tap values
  for(uint8_t tapinc=0; tapinc<32; tapinc++) {

    printf("------------------------------------------------------------------- \n");

    //Load new tap value and reset the check counters
    gpio_output = (1<<6) | (1<<5) | (tapinc & 0x1F);
    Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, gpio_output);
    sleep(1);
    gpio_output = (tapinc & 0x1F);
    Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, gpio_output);
    printf("Tap updated: 0x%X \n", tapinc);
    sleep(1);

    uint32_t prgrm_chk = Xil_In32((uint32_t)(mmapaddr1 + GPIO1_CH1_OFFSET));
    printf("Check Init: 0x%X Load: 0x%X Tap: 0x%X Zflag: 0x%X Full Word 0x%X \n", ((prgrm_chk>>7) & 0x1), 
           ((prgrm_chk>>6) & 0x1), ((prgrm_chk>>1) & 0x1F), (prgrm_chk & 0x1), prgrm_chk);

    //Wait a bit and check for errors for the set delay
    int cnt = 0;
    while (cnt<11) {
      uint64_t errchk_lo = Xil_In32((uint32_t)(mmapaddr2 + GPIO1_CH0_OFFSET));
      uint64_t errchk_hi = Xil_In32((uint32_t)(mmapaddr2 + GPIO1_CH1_OFFSET));
      uint64_t cycle_lo = Xil_In32((uint32_t)(mmapaddr3 + GPIO3_CH0_OFFSET));
      uint64_t cycle_hi = Xil_In32((uint32_t)(mmapaddr3 + GPIO3_CH1_OFFSET));
      uint64_t cycle_cnt = ((cycle_hi & 0xFFFFFF)<<24) | (cycle_lo & 0xFFFFFF); // concat the two 24b words for a total of 48b
      uint64_t err_cnt = ((errchk_hi & 0xFFFFFF)<<24) | (errchk_lo  & 0xFFFFFF); // concat the two 24b words for a total of 48b

      printf("------------------------------------------------------------------- \n");
      printf("Cycle Count: %" PRIu64 " Error Count: %" PRIu64 "\n", cycle_cnt, err_cnt);
      printf("Cycle Count HI: %" PRIu64 " Cycle Count LO: %" PRIu64 "\n", cycle_hi, cycle_lo);
      sleep(1);
      cnt++;
   }
  }

  //Reset tap to 0 before shutting down
  // Doing this seems to throw the endpoint into an error state
  gpio_output = (1<<5) + 0x0;
  Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, gpio_output);
  sleep(1.0); 
  Xil_Out32((uint32_t)mmapaddr1 + GPIO1_CH0_OFFSET, 0x0);

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

