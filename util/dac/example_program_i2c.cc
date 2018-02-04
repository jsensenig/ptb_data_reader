/*
A user-space C++ program to deal with a I2C device.
*/

extern "C" {
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <inttypes.h>
};
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>
#include <cstring>

namespace i2c {

////////////////////////////////////////////////
// I2C structures for content manipulation
////////////////////////////////////////////////

// -- Channel masks

#define CH0 0x0
#define CH1 0x1
#define CH2 0x2
#define CH3 0x3
#define CH4 0x4
#define CH5 0x5
#define CH6 0x6
#define CH7 0x7
#define ALL_CH 0xF


// -- Commands

/// Define a couple of commands
// not sure of what it does
#define I2C_WR_IN_N     0x0 
// Read one channel
#define I2C_RD_N        0x1
// Write all channels
#define I2C_WR_UPD_ALL  0X2
// Write channel N (mask)
#define I2C_WR_UPD_N    0x3
// Power on/off
#define I2C_POWER       0x4
// Clear 
#define I2C_CLEAR       0x5
// Load registers?
#define I2C_LOAD        0x6
// Reset
#define I2C_RESET       0x7

  static const uint32_t dac_nbytes = 3;

  typedef struct device {
    int32_t     fd;
    uint8_t     addr;
    uint16_t    channel;
    std::string devname;
  } device;

  typedef struct command_t {
    uint8_t channel : 4;
    uint8_t command : 4;
  } command_t;

  typedef union command {
    command_t   cmd;
    uint8_t         val;
  } command;

  // -- NFB : I am flipping these on purpose, so that I can deal with the 
  // endianness without having to go insane
  typedef struct level_t {
    uint16_t lsdb : 8;
    uint16_t msdb : 8;
  } level_t;

  typedef union level {
    level_t  level;
    uint16_t     val;
    /** utility function that given a DAC integer value, 
    // sets the appropriate bits
     - This is because this device is stupid and the level are set on the 
     -- higher bits while the lower are ignored.
     level : word[15:4]
      The range is 12 bits, so max value is 4095
    */
    void set_int_level(uint16_t lvl) {
      if (lvl > 0xFFF) {
        printf("Attempting to set a threshold larger than possible (max 4095, 0xFFF). Truncating to max.\n");
        lvl = 0xFFF;
      }
      level.msdb = ((lvl << 4) & 0xF0);
      level.lsdb = (((lvl << 4) >> 8) & 0xFF);
    }

    uint16_t get_int_level() {
      return ((level.lsdb << 4) | (level.msdb >> 4));
    }
  } level;


  const size_t ndacs = 3;
  uint8_t DAC[ndacs] = {0x48,0x4A,0x4C};
  
  int32_t i2c_init( int32_t &fd,const std::string &device = "/dev/i2c-0", bool debug = true) {
    int32_t file = -1;
    int32_t res = -1;
    if (debug) {
      printf("==================================================================\n");
      printf("i2c_init : Initializing I2C device %s:\n",device.c_str());
      printf("==================================================================\n");
    }
    file = open(device.c_str(),O_RDWR);
    if (file < 0) {
      printf("i2c_init : Failed to open the bus.\n");
      return 0;
    } else {
      if (debug) {
        printf("i2c_init : Received fd %i\n",file);
      }
    }
    
    // Extract the functions available
    uint64_t funcs = 0x0;
    res = ioctl(file, I2C_FUNCS, &funcs);
    if ( res < 0) {
      printf("i2c_init : Failed to query device for functionality.\n");
      printf("i2c_init : Msg : %s\n",strerror(errno));
      return 0;
    } else {
      printf("i2c_init : Got func word for device : %" PRIu64 " (%" PRIx64 ")\n",funcs,funcs);
    }

    fd = file;
    return 1;
    
    // uint8_t address = dac.addr;
    // if (ioctl(file,I2C_SLAVE,0x48) < 0) {
    //   printf("i2c_init : Failed to acquire bus access and/or talk to the slave.\n");
    //   printf("i2c_init : msg : %s\n",strerror(errno));
    //   //close(file);
    //   //dac.fd = -1;
    //   //exit(1);
    //   //return 0;
    // }
    // dac.fd = file;
    // return file+1;
  }

  int32_t i2c_set_device(const device &dac) {
    // -- set the device to be operated on
    int32_t ret;
    ret = ioctl(dac.fd,I2C_SLAVE,dac.addr);
    if (ret < 0) {
      printf("i2c_set_device : Failing addressing the appropriate slave register.\n");
      printf("message: %s\n",strerror(errno));
      return 0;
    }
    return 1;
  }

  void i2c_exit(const int32_t &fd,  bool debug = true) {

    int32_t file;
    if (fd < 0) return;
    if (debug) {
      printf("==================================================================\n");
      printf("i2c_exit : Terminating I2C device %u:\n",fd);
      printf("==================================================================\n");
    }
    close(fd);
  }
  
  int32_t i2c_write_smbus(const device &dac, uint8_t reg, uint16_t value, bool debug = false) {
    if (dac.fd < 0) return 0;

    if (debug) {
      printf("==================================================================\n");
      printf("i2c_write_smbus : Writing contents to I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
                                                      dac.fd,
                                                      dac.channel,dac.addr);
      printf(" * Write register : 0x%X\n",reg);
      printf(" * Value : 0x%X\n",value);
    }

    if (!i2c_set_device(dac)) {
      printf("i2c_read_smbus : Failed to update device pointer.\n");
      return 0;
    }

    int32_t ret = i2c_smbus_write_word_data(dac.fd,reg,value);
    if (ret == 0) { // returns 0 on success
      if (debug) {
        printf("i2c_write_smbus : Contents written with success.\n");
      }
    } else {
      printf("i2c_write_smbus : Failed to write contents to DAC.\n");
      printf("message : %s\n",strerror(errno));
      return 0;
    }
    return 1;
  }

  int32_t i2c_read_smbus(const device &dac, uint8_t reg, uint16_t &value, bool debug = false) {
    if (dac.fd < 0) return 0;

    if (debug) {
      printf("==================================================================\n");
      printf("i2c_read_smbus : Reading contents to I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
                                                      dac.fd,
                                                      dac.channel,dac.addr);
      printf(" * Read register : 0x%X\n",reg);
      // printf(" * Value : 0x%X\n",value);
    }

    if (!i2c_set_device(dac)) {
      printf("i2c_read_smbus : Failed to update device pointer.\n");
      return 0;
    }

    int32_t ret = i2c_smbus_read_word_data(dac.fd,reg);
    if (ret < 0) { // Returns contents and -1 on failure
      printf("i2c_read_smbus : Failed to read contents of DAC.\n");
      printf("message : %s\n",strerror(errno));
      return 0;
    } else {
      value = ret & 0xFFFF;
      if (debug) {
        printf("i2c_read_smbus : Contents from 0x%X : 0x%X.\n",reg,value);
      }
    }
    return 1;
  }


  // Lower level functions, that deal with the I2C devices at a lower level 
  // by writing and reading directly from the file descriptor
  // Does the same as above, but using direct device memory write
  int32_t i2c_write(const device& dac, uint8_t*buffer, const size_t& bytes, bool debug = false) {
    // refuse to write to an unopen device
    if (dac.fd < 0) {
      printf("i2c_write : Attempting to write to invalid FD (%i). Aborting.\n",dac.fd);
      return 0;
    }

    ssize_t wr_bytes;
    size_t idx = 0;
    if (debug) {
      printf("==================================================================\n");
      printf("i2c_write : Writing contents to I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
                                                      dac.fd,
                                                      dac.channel,dac.addr);
      printf(" * Full buffer :\n");
      for (idx = 0; idx < bytes; idx++) {
        printf(" Byte %u : 0x%X\n",idx,buffer[idx]);
      }
      printf("==================================================================\n");
    }

    // -- Set the address first
    if (!i2c_set_device(dac)) {
      printf("i2c_write : Failed to update device pointer.\n");
      return 0;
    }

    wr_bytes = write(dac.fd,buffer,bytes);
    if (wr_bytes != bytes) {
      printf("i2c_write : Failed to write the message to DAC %i. Received %i (expected %u).\n",dac.channel,wr_bytes,bytes);
      return 0;
    }
    return 1;
  }

  int32_t i2c_read(const device& dac, uint8_t reg, uint8_t*&buffer, const size_t& bytes, bool debug = false) {
    if (dac.fd < 0) {
      printf("i2c_write : Attempting to read from an invalid FD (%i). Aborting.\n",dac.fd);
      return 0;
    }

    ssize_t rd_bytes;
    size_t idx = 0;
    if (debug) {
      printf("==================================================================\n");
      printf("i2c_read : Reading contents from I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
                                                      dac.fd,
                                                      dac.channel,dac.addr);
      printf("==================================================================\n");
    }

    // -- Set the address first
    if (!i2c_set_device(dac)) {
      printf("i2c_write : Failed to update device pointer.\n");
      return 0;
    }

    // -- First set the address
    buffer[0] = reg;
    size_t nbt = 1;
    if (!i2c_write(dac,buffer,nbt,debug)) {
      printf("i2c_read : Failed to set pointer.\n");
      return 0;
    }

    rd_bytes = read(dac.fd,buffer,bytes);
    if (rd_bytes != bytes) {
      printf("i2c_read : Failed to read the register from DAC %i. Received %i (expected %u).\n",dac.channel,rd_bytes,bytes);
      return 0;
    }
    if (debug) {
      printf("i2c_read : Contents from register 0x%X:\n",reg);
      for (idx = 0; idx < bytes; idx++) {
        printf("[0x%X] ",buffer[idx]);
      }
      printf("\n");
    }

    return 1;
  }

  /** There is a problem with this function. In principle it is not needed
      But should check at some point.
    int32_t exec_dac_cmd(const device& dac, const command &cmd, bool debug = false) {
    uint8_t buffer[dac_nbytes];
    buffer[0] = dac.addr;
    buffer[1] = cmd.val;

    // refuse to write to an unopen device
    if (dac.fd < 0) {
      printf("i2c_write : Attempting to write to invalid FD (%u). Aborting.\n",dac.fd);
      return 0;
    }
    if (debug) {
      printf("==================================================================\n");
      printf("exec_dac_cmd : Writing contents to I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
             dac.fd,
             dac.channel,dac.addr);
      printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
             cmd.val, cmd.cmd.channel,cmd.cmd.channel,
             cmd.cmd.command,cmd.cmd.command);

      printf(" * Full buffer : [0x%X] [0x%X]\n",buffer[0],buffer[1]);
      printf("==================================================================\n");

    }
    if (!i2c_write(dac,buffer,2)) {
      printf("exec_dac_cmd : Failed to program DAC %u (0x%X)\n",dac.channel,dac.addr);
    }
    return 1;
    }
  */
  // This is a rather specialized function that uses the other i2c aux functions
  // to set the DAC thresholds 
  int32_t set_dac_level(const device& dac, const command &cmd, const level &lvl, bool debug = false) {

    // refuse to write to an unopen device
    if (dac.fd < 0) {
      printf("set_dac_level : Attempting to write to invalid FD (%i). Aborting.\n",dac.fd);
      return 0;
    }
    if (debug) {
      printf("==================================================================\n");
      printf("set_dac_level : Writing contents to I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
             dac.fd,
             dac.channel,dac.addr);
      printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
             cmd.val, cmd.cmd.channel,cmd.cmd.channel,
             cmd.cmd.command,cmd.cmd.command);

      printf(" * Values : [lvl value 0x%X (%u)] [msdb 0x%X (%u)] [msdb 0x%X (%u)]\n",
             lvl.val,lvl.val,lvl.level.msdb,lvl.level.msdb,lvl.level.lsdb,lvl.level.lsdb);
      printf("==================================================================\n");

    }


    // use the smbus, or the i2c_write?
    // let's go with the smbus
    if (!i2c_write_smbus(dac,cmd.val,lvl.val,debug)) {
      printf("set_dac_level : Failed to write to register 0x%X\n",cmd.val);
      return 0;
    }

    // -- If I wanted to use the i2c_write function, I would instead do:
    /**
    uint8_t buffer[sizeof(cmd) + sizeof(lvl)];
    buffer[0] = cmd.val;
    buffer[1] = lvl.level.msdb
    buffer[2] = lvl.level.lsdb
    if (!i2c_write(dac,buffer,sizeof(cmd) + sizeof(lvl))) {
      printf("set_dac_level : Failed to program DAC %u (0x%X)\n",dac.channel,dac.addr);
      printf("set_dac_level : Failed to write to register 0x%X\n",cmd.val);
      return 0;
    }
    */
    return 1;
  }

  int32_t get_dac_level(const device& dac, const command &c, level &lvl,bool debug = false) {

    // refuse to write to an unopen device
    if (dac.fd < 0) {
      printf("i2c_write : Attempting to write to invalid FD (%i). Aborting.\n",dac.fd);
      return 0;
    }

    if (debug) {
      printf("get_dac_level : Reading from I2C DAC %u:\n",dac.channel);
      printf(" * device : %s [fd %i] [DAC %u 0x%X]\n",dac.devname.c_str(),
             dac.fd,
             dac.channel,dac.addr);
      printf(" * command : [cmd addr 0x%X] [ch 0x%X (%u)] [cmd 0x%X (%u)]\n",
             c.val, c.cmd.channel,c.cmd.channel,
             c.cmd.command,c.cmd.command);
    }

    if (!i2c_read_smbus(dac,c.val,lvl.val,debug)) {
      printf("get_dac_level : Failed to read DAC %i.\n",dac.channel);
      return 0;
    }

    /** If I wanted to read using the file descriptor directly, 
    the usage would be:

    uint8_t buffer[sizeof(c) + sizeof(lvl)];
    if (!i2c_read(dac,c.val,buffer,sizeof(lvl))) {
      printf("get_dac_level : Failed to read DAC %i.\n",dac.channel);
      return 0;
    } else {
      lvl.level.msdb = buffer[0];
      lvl.level.lsdb = buffer[1];
      if (debug) {
        printf("get_dac_level : Contents read: 0x%X (%u) : [0x%X,0x%X]\n",
               lvl.val,lvl.val,lvl.level.msdb,lvl.level.lsdb);
      }
    }
    */
    return 1;
  }

};




// -- Function to do a consistency check in the DACs. 
// -- It writes a random table, and then reads back the values 
void dac_galore(){
  const size_t ndacs = 3;
  const size_t nchannels = 8;
  uint16_t threshold_table[3][8]; 
  std::srand((int)std::time(0));

  i2c::device dev[3];
  i2c::command c;
  i2c::level l;

  int32_t fd;
  if (!i2c::i2c_init(fd)) {
    printf("Failed to initialize.\n");
    return;
  }

  for (size_t i = 0; i < ndacs; i++) {
    dev[i].channel = i;
    dev[i].addr = i2c::DAC[i];
    dev[i].devname = "/dev/i2c-0";
    dev[i].fd = fd;
  }

  // -- First read the existing contents
  printf("\nReading the existing thresholds....\n");

  c.cmd.command = I2C_RD_N;

  for (size_t i = 0; i < ndacs; i++) {
    printf("DAC %u : ",i);
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      if (!i2c::get_dac_level(dev[i],c,l,false)) {
        printf("Failed to read channel [%u:%u]\n",i,j);
        return;
      }
      printf("%04u ",l.get_int_level());
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
    printf("\n");
  }

  printf("\nInitializing random settings...\n");
  for (size_t i = 0; i < ndacs; i++) {
    printf("DAC %u : ",i);
    for (size_t j = 0; j < nchannels; j++) {
      threshold_table[i][j] = (uint16_t) (rand() % 4095);
      printf("%04u ",threshold_table[i][j]);
    }
    printf("\n");
  }

  printf("\nSetting the new thresholds...\n");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs; i++) {
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      l.set_int_level(threshold_table[i][j]);
      if (!i2c::set_dac_level(dev[i],c,l,false)) {
        printf("Failed to set channel [%u:%u]\n",i,j);
        return;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }

  printf("\nPerforming consistency check...: [O : OK; X : Fail]\n");
  c.cmd.command = I2C_RD_N;
  for (size_t i = 0; i < ndacs; i++) {
    printf("DAC %u : ",i);
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      if (!i2c::get_dac_level(dev[i],c,l,false)) {
        printf("Failed to read channel [%u:%u]\n",i,j);
        return;
      }
      printf("%04u(%s) ",l.get_int_level(),(l.get_int_level() == threshold_table[i][j])?"O":"X");
      sleep(0.1);
    }
    printf("\n");
  }


  printf("\nSetting all thresholds to 0...\n");                                      
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs; i++) {
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      l.set_int_level(0x0);
      if (!i2c::set_dac_level(dev[i],c,l,false)) {
        printf("Failed to set channel [%u:%u]\n",i,j);
        return;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }


  i2c::i2c_exit(fd);

}

int main() {
  
  // -- let's start with something simple 
  i2c::device dev;
  int32_t fd;

  printf("Opening device...\n");
  if (!i2c::i2c_init(fd)) {
    printf("Failed to initialize.\n");
    return 1;
  }
  dev.channel = 0;
  dev.devname = "/dev/i2c-0";
  dev.fd = fd;
  dev.addr = i2c::DAC[0];

  // -- Let's read what is in the device
  i2c::command c;
  c.cmd.command = I2C_RD_N;
  c.cmd.channel = CH0;

  i2c::level l;


  printf("Reading channels\n");

  if (!i2c::get_dac_level(dev,c,l,true)) {
    printf("Failed to read\n");
    return 1;
  }
  printf("Current levels : 0x%X [M 0x%X L 0x%X]\n",l.val,l.level.msdb,l.level.lsdb);
  printf("Current level in human readable form : %u\n",l.get_int_level());

  printf("Writing a new level into ch %u : %u (0x%X)\n",c.cmd.channel,0xA3C,0xA3C);
  // -- Write a new level on this same channel
  c.cmd.command = I2C_WR_UPD_N;
  l.set_int_level(0xA3C); // -- the function should swap the bytes automatically
  // putting in the contents as 0xC0A3, which is what the i2c wants 
  // damn endianess...
  if (!i2c::set_dac_level(dev,c,l,true)) {
    printf("Failed to write\n");
    return 1;
  }

  // -- And now read it back again
  printf("Reading everything back again...\n");
  c.cmd.command = I2C_RD_N;
  if (!i2c::get_dac_level(dev,c,l,true)) {
    printf("Failed to read\n");
    return 1;
  }
  printf("Current levels : 0x%X [M 0x%X L 0x%X]\n",l.val,l.level.msdb,l.level.lsdb);
  printf("Current level in human readable form : %u (0x%X)\n",l.get_int_level(),l.get_int_level());

  i2c::i2c_exit(fd);

  printf("\n\n== Performing consistency test on the DAC ==\n\n");
  printf("Will be assigning random values to the DAC and then \n");
  printf(" checking that all are read back consistently\n\n\n\n");

  dac_galore();

  return 0;
  }
