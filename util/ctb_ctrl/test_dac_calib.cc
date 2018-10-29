extern "C" {
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <inttypes.h>
};
#include <cstdint>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cinttypes>
#include <bitset>

#include <dacsetup.h>
//#include <json.hpp>
//#include <gpio2.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close

#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8

#define GPIO_1_BASEADDR 0x41210000
#define GPIO_1_HIGHADDR 0x4121FFFF
#define GPIO_1_CH0_OFFSET 0x0
#define GPIO_1_CH1_OFFSET 0x8

#define COUNT 5000

//using json = nlohmann::json;

I2C* dacsetup;
//GPIO2* gpiosetup;


int g_mem_fd;
//void *mmapaddr;
void *mmapaddr1;

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


int MapDacs(std::ofstream &dacmap, bool debug) {

          dacmap << "DAC " << "Channel " << "Ticks " << "Freq " << std::endl;
          dacmap << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
	  const size_t ndacs = 3;
	  const size_t nchannels = 8;
	  uint8_t DACs [3] = {0x48, 0x4A, 0x4C};
	  int32_t fd;

	  //Map GPIO
	  printf("Mapping the GPIO ...\n");
	  //mmapaddr = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
	  mmapaddr1 = MapPhysMemory(GPIO_BASEADDR,GPIO_HIGHADDR);
	  printf("Received pointer to GPIO at %p\n",mmapaddr1);

	  if (!dacsetup->i2c_init(fd)) { //Init i2c device
	    printf("Failed to initialize.\n");
	    return 0;
	  }

	  I2C::device dev[3];
	  I2C::command c;
	  I2C::level l;

	  for (size_t i = 0; i < ndacs; i++) {
	    dev[i].channel = i;
	    dev[i].addr = DACs[i];
	    dev[i].devname = "/dev/i2c-0";
	    dev[i].fd = fd;
	  }

        // Enable freq check
        Xil_Out32((uint32_t)(mmapaddr1+GPIO_1_CH0_OFFSET),16777216); // 24th bit
        usleep(1100*1000); //Make sure the freq check interval has elapsed  

        std::cout << "                  " << std::flush;
        for (int k=0; k<101; k++) {
          if (k==0) { std::cout << "  [" << std::flush; } 
          std::cout << " " << std::flush;
          if (k == 99) { std::cout << "]" << std::flush;}
        }

        int dac = 0;
	//for (int i=0; i<dacsetup->nchannels; i++) {
	for (int i=0; i<23; i++) {

	  int channel = i%nchannels;
	  //Mask in one channel at a time
          uint32_t mask = (uint32_t)pow(2,i)+16777216;
	  Xil_Out32((uint32_t)(mmapaddr1+GPIO_1_CH0_OFFSET),mask);

          std::cout << "  Mask: " << std::bitset<24>(mask) << std::endl;
          std::cout << "[Channel:DAC] [" << channel << ":" << dac << "]  " << std::flush;

	  uint16_t baseline = 2300; //Baseline ~2048 ticks (1.65V) 
	  //uint16_t baseline = 1800; //Starting from the "bottom" of signal
          for (int j=0; j<100; j++) {

            std::cout << "#" << std::flush;
    	    uint16_t value = baseline - (uint16_t)j*6; //Decrement in 4.8mV steps
	    c.cmd.command = I2C_WR_UPD_N;
	    c.cmd.channel = channel;
	    l.set_int_level(value);
                                                                                            
	    if (!dacsetup->set_dac_level(dev[dac],c,l,false)) {
	      printf("Failed to set channel [%u:%u]\n",channel,dac);
	    }

            // -- sleep a short while to make sure that we are not updating this too fast
            //and to make sure the freq was checked. It's checked every 1/2 sec so wait
            //a bit longer than a sec to make sure we get a good measurement
            usleep(1200*1000);
            uint32_t count = Xil_In32((uint32_t)mmapaddr1 + GPIO_1_CH1_OFFSET);
                                                                                            
            dacmap << dac << " " << channel << " " << value << " " << count*2 << std::endl;
            
	  }  //tick loop

          if (channel == (nchannels-1)) { dac++; }

        } //channel loop

        std::cout << std::endl;
	return 0;
}


int main() {

  std::ifstream i;
  std::ofstream dacmap;
/*  json j;

  //--Configure DACs from config file

  //Read in DAC settings from config file
  printf("\nReading DAC settings from file...\n");
  i.open("ptb_config.json");
  std::cout << "Parsing document" << std::endl;
  i >> j;

  std::cout << "JSON object filled..." << std::endl;
  //Dig down to the DAC level settings
  json ctbconf = j.at("ctb");
  json subsys = ctbconf.at("subsystems");
  json ssp = subsys.at("ssp");

 // json ssp = dacconf;
  std::vector<uint32_t> dacv = ssp.at("dac_thresholds");
  for(size_t i=0; i<dacv.size(); i++) {
    std::cout << dacv[i] << std::flush;
  }
  std::cout << std::endl;

  if (dacsetup->ConfigureDacs(dacv, false)) {
     printf("Failed R/W test. \n");
     return 1;
  }
*/
  dacmap.open("dac_map.txt");

  std::vector<uint32_t> dacrst (24);
  printf("Setting all DAC channels to 0 \n");
  printf("============================================= \n");
  if (dacsetup->ConfigureDacs(dacrst, false)) {
     printf("Failed R/W test. \n");
     return 1;
  }

  if (MapDacs(dacmap, false)) {
     printf("Failed R/W test. \n");
     return 1;
  }
  std::cout << "Finished mapping DACs" << std::endl;
  dacmap.close();

  return 0;
}

