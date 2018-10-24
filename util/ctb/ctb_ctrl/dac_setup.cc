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
#include <vector>

#include <dacsetup.h>
#include <json.hpp>

//using namespace std;
//using json = nlohmann::json;

I2C::I2C() {

	//set_dac_level(I2C::device, I2C::command, I2C::levels, false);
	 //set_dac_level(const device &dac, const command &cmd, const level &levels, bool debug);
	//set_dac_level(const device dac, const command cmd, const level levels, bool debug);
}

uint8_t DACs [3] = {0x48, 0x4A, 0x4C};

int32_t I2C::i2c_init( int32_t &fd,const std::string &device, bool debug) {
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

int32_t I2C::i2c_set_device(const device &dac) {
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

void I2C::i2c_exit(const int32_t &fd,  bool debug) {

  //int32_t file;
  if (fd < 0) return;
  if (debug) {
    printf("==================================================================\n");
    printf("i2c_exit : Terminating I2C device %u:\n",fd);
    printf("==================================================================\n");
  }
  close(fd);
}

int32_t I2C::i2c_write_smbus(const device &dac, uint8_t reg, uint16_t value, bool debug) {
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

  if (!I2C::i2c_set_device(dac)) {
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
    printf("i2c_write_smbus : Tried with DAC %x and register %x and with value %x.\n",dac.fd,reg,value);
    printf("message : %s\n",strerror(errno));
    return 0;
  }
  return 1;
}

int32_t I2C::i2c_read_smbus(const device &dac, uint8_t reg, uint16_t &value, bool debug=true) {
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

  if (!I2C::i2c_set_device(dac)) {
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
int32_t I2C::i2c_write(const device& dac, uint8_t*buffer, const size_t& bytes, bool debug) {
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
  if (!I2C::i2c_set_device(dac)) {
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

int32_t I2C::i2c_read(const device& dac, uint8_t reg, uint8_t*&buffer, const size_t& bytes, bool debug) {
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
  if (!I2C::i2c_write(dac,buffer,nbt,debug)) {
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
  int32_t exec_dac_cmd(const device& dac, const command &cmd, bool debug) {
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
int32_t I2C::set_dac_level(const device& dac, const command &cmd, const level &lvl, bool debug) {

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
  if (!I2C::i2c_write_smbus(dac,cmd.val,lvl.val,debug)) {
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

int32_t I2C::get_dac_level(const device& dac, const command &c, level &lvl,bool debug) {

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

  if (!I2C::i2c_read_smbus(dac,c.val,lvl.val,debug)) {
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
///////////////////////////////////////////////////////////////////
int32_t I2C::ConfigureDacs(std::vector<uint32_t> &dacs, bool debug) {

  const size_t ndacs = 3;
  const size_t nchannels = 8;
  uint8_t DACs [3] = {0x48, 0x4A, 0x4C};
  int32_t fd;
  
  if (!I2C::i2c_init(fd)) { //Init i2c device
    printf("Failed to initialize.\n");
    return 0;
  }

  uint16_t threshold_table[ndacs][nchannels];

  I2C::device dev[3];
  I2C::command c;
  I2C::level l;

  for (size_t i = 0; i < ndacs; i++) {
    dev[i].channel = i;
    dev[i].addr = DACs[i];
    dev[i].devname = "/dev/i2c-0";
    dev[i].fd = fd;
  }

  printf("Setting %u dac thresholds to: \n ", dacs.size());
  //Load DAC values into a table
  int ii = 0;
  for (size_t jj = 0; jj < dacs.size(); jj++) {
     printf("DAC %u Channel %u Value %u \n", ii, (jj%nchannels), dacs.at(jj));
    threshold_table[ii][jj%nchannels] = dacs.at(jj);
    if (jj%nchannels == (nchannels-1)) { ii++; }
  }

  printf("\n");

  if (debug) {
    printf("Printing the DAC level values. \n");
    for (size_t i = 0; i < ndacs; i++) { //Dac settings
       printf("DAC %u : ",i);
      for (size_t j = 0; j < nchannels; j++) { //Channel values
        printf("%04u ",threshold_table[i][j]);
      }
      printf("\n");
    }
  }

  //Reset all DAC levels to 0
  printf("\nResetting all DAC channels to 0. . .\n");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs; i++) {
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      l.set_int_level(0x0);
      if (!I2C::set_dac_level(dev[i],c,l,false)) {
        printf("Failed to set channel [%u:%u]\n",i,j);
        return 1;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }
  //Set the DAC values from config
  printf("\nConfiguring thresholds...\n");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs; i++) {
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      l.set_int_level(threshold_table[i][j]);
      if (!I2C::set_dac_level(dev[i],c,l,debug)) {
        printf("Failed to set channel [%u:%u]\n",i,j);
        return 1;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }

  printf("\nPerforming consistency check...\n");
  if (debug) { printf("Debugging : Consistency check indicators [O : OK; X : Fail]\n"); }
   c.cmd.command = I2C_RD_N;
   for (size_t i = 0; i < ndacs; i++) {
     if (debug) { printf("DAC %u : ",i); }
     for (size_t j = 0; j < nchannels; j++) {
       c.cmd.channel = j;
       if (!I2C::get_dac_level(dev[i],c,l,debug)) {
         printf("Failed to read channel [%u:%u]\n",i,j);
         return 1;
       }
       if (debug) { printf("%04u(%s) ",l.get_int_level(),(l.get_int_level() == threshold_table[i][j])?"O":"X"); }
       if (l.get_int_level() != threshold_table[i][j]) { return 1; }
       sleep(0.1);
     }
     if (debug) { printf("\n"); }
   }

  printf("Programmed and checked DAC levels successfully! \n");

  I2C::i2c_exit(fd);
  return 0;
}
///////////////////////////////////////////////////////////////////

// -- Function to do a consistency check in the DACs. 
// -- It writes a random table, and then reads back the values 
int32_t I2C::dac_test(bool debug) {
  const size_t ndacs = 3;
  const size_t nchannels = 8;
  uint16_t threshold_table[3][8]; 
  std::srand((int)std::time(0));

  I2C::device dev[3];
  I2C::command c;
  I2C::level l;
  //uint8_t DACs [3] = {0x48, 0x4A, 0x4C};
  int32_t fd;
  if (!I2C::i2c_init(fd)) {
    printf("Failed to initialize.\n");
    return 0;
  }

  for (size_t i = 0; i < ndacs; i++) {
    dev[i].channel = i;
    dev[i].addr = DACs[i];
    dev[i].devname = "/dev/i2c-0";
    dev[i].fd = fd;
  }

  // -- First read the existing contents
  printf("\nReading the existing thresholds....\n");

  c.cmd.command = I2C_RD_N;

  for (size_t i = 0; i < ndacs; i++) {
    printf("DAC %u : ",i);
    for (size_t j = 0; j < nchannels; j++) {
      printf("%u",j);
      c.cmd.channel = j;
      if (!I2C::get_dac_level(dev[i],c,l,debug)) {
        printf("Failed to read channel [%u:%u]\n",i,j);
        return 0;
      }
      printf("%04u ",l.get_int_level());
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
    printf("\n");
  }

  printf("\nInitializing random settings...\n");
  for (size_t i = 0; i < ndacs; i++) {
    if (debug) { printf("DAC %u : ",i); }
    for (size_t j = 0; j < nchannels; j++) {
      threshold_table[i][j] = (uint16_t) (rand() % 4095);
      if (debug) { printf("%04u ",threshold_table[i][j]); }
    }
    printf("\n");
  }

  printf("\nSetting the new thresholds...\n");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs; i++) {
    if (debug) { printf("DAC %u", i); }
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      l.set_int_level(threshold_table[i][j]);
      if (!I2C::set_dac_level(dev[i],c,l,debug)) {
        printf("Failed to set channel [%u:%u]\n",i,j);
        return 0;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
    printf("\n");
  }

  printf("\nPerforming consistency check...: [O : OK; X : Fail]\n");
  c.cmd.command = I2C_RD_N;
  for (size_t i = 0; i < ndacs; i++) {
    if (debug) { printf("DAC %u : ",i); }
    for (size_t j = 0; j < nchannels; j++) {
      c.cmd.channel = j;
      if (!I2C::get_dac_level(dev[i],c,l,false)) {
        printf("Failed to read channel [%u:%u]\n",i,j);
        return 0;
      }
      if(debug) { printf("%04u(%s) ",l.get_int_level(),(l.get_int_level() == threshold_table[i][j])?"O":"X"); }
      if(l.get_int_level() != threshold_table[i][j]) { 
         printf("DAC %u Channel %u failure during R/W test \n",i,j);   
         return 0;
      }
      sleep(0.1);
    }
    printf("\n");
  }

  I2C::i2c_exit(fd);

  return 1;
}

