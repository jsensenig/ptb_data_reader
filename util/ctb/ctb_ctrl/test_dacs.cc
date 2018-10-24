extern "C" {
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
};
#include <cstdint>
#include <math.h>
#include <cstdlib>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cinttypes>

#include <dacsetup.h>

#define COUNT 5000

I2C* dacsetup;

int TestDacs(bool debug) {

  const size_t ndacs = 3;
  const size_t nchannels = 8;
  uint8_t DACs [3] = {0x48, 0x4A, 0x4C};
  int32_t fd;
  
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
  
  int dac = 0;
  int value = 0;

  while (true) {
    
    std::cout << "DAC " << dac << std::endl;
  
    for (int j=0; j<8191; j++) {
  
      if(j > 4095) { 
        value = 8191 - j; 
      } else {
        value = j;
      }                                                                                 
      c.cmd.command = I2C_WR_UPD_N;
      c.cmd.channel = ALL_CH; //Set this to all channels
      l.set_int_level(value);
                                                                                      
      if (!dacsetup->set_dac_level(dev[dac],c,l,false)) {
        printf("Failed to set channel [%u:%u]\n",channel,dac);
      }
  
      usleep(500);
      
    }  //tick loop
  
    dac ++;
    if( dac > 2 ) {
      dac = 0; 
    }
  
  } //run loop
  
  std::cout << std::endl;
  return 0;
}


int main() {


  std::vector<uint32_t> dacrst (24);
  printf("Setting all DAC channels to 0 \n");
  printf("============================================= \n");
  if (dacsetup->ConfigureDacs(dacrst, false)) {
     printf("Failed R/W test. \n");
     return 1;
  }

  if (TestDacs(false)) {
     printf("Failed R/W test. \n");
     return 1;
  }
  std::cout << "Finished mapping DACs" << std::endl;

  return 0;
}

