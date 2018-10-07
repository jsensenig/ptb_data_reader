/*
			A user-space C++ program to deal with a I2C device.
*/

#include "i2conf.h"
#include <cstdio> // for printf
#include <cstring>
#include <cerrno>

extern "C" {
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
};

#include "Logger.h"

//extern int  i2c_smbus_write_word_data(const struct i2c_client * client, uint8_t command, uint16_t value);
extern int com_serial;
extern int failcount;

namespace ptb {

const uint16_t i2conf::dac_addr_[ndacs_] = {0x48,0x4A,0x4C};

uint8_t DACs [3] = {0x48, 0x4A, 0x4C};

int32_t i2conf::i2c_init( int32_t &fd,const std::string &device, bool debug) {
  int32_t file = -1;
  int32_t res = -1;
  if (debug) {
    printf("==================================================================\n");
    printf("i2c_init : Initializing I2C device %s:\n",device.c_str());
    printf("==================================================================\n");
  }
  file = open(device.c_str(),O_RDWR);
  if (file < 0) {
    Log(error,"i2c_init : Failed to open the bus.\n");
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
    Log(error,"i2c_init : Failed to query device for functionality.");
    Log(error,"i2c_init : Msg : %s",strerror(errno));
    return 0;
  } else {
    Log(info,"i2c_init : Got func word for device : %" PRIu64 " (%" PRIx64 ")",funcs,funcs);
  }
  fd = file;
  return 1;
  
}

int32_t i2conf::i2c_set_device(const device &dac) {
  // -- set the device to be operated on
  int32_t ret;
  ret = ioctl(dac.fd,I2C_SLAVE,dac.addr);
  if (ret < 0) {
    Log(error,"i2conf_set_device : Failing addressing the appropriate slave register.");
    Log(error,"message: %s",strerror(errno));
    return 0;
  }
  return 1;
}

void i2conf::i2c_exit(const int32_t &fd,  bool debug) {

  //int32_t file;
  if (fd < 0) return;
  if (debug) {
    printf("==================================================================\n");
    printf("i2c_exit : Terminating I2C device %u:\n",fd);
    printf("==================================================================\n");
  }
  close(fd);
}

int32_t i2conf::i2c_write_smbus(const device &dac, uint8_t reg, uint16_t value, bool debug) {
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

  if (!i2conf::i2c_set_device(dac)) {
    Log(error,"i2c_read_smbus : Failed to update device pointer.");
    return 0;
  }

  int32_t ret = i2c_smbus_write_word_data(dac.fd,reg,value);
  if (ret == 0) { // returns 0 on success
    if (debug) {
      printf("i2c_write_smbus : Contents written with success.\n");
    }
  } else {
    Log(error,"i2c_write_smbus : Failed to write contents to DAC.");
    Log(error,"i2c_write_smbus : Tried with DAC %x and register %x and with value %x.",dac.fd,reg,value);
    //Log(error,("message : %s",strerror(errno));
    return 0;
  }
  return 1;
}

int32_t i2conf::i2c_read_smbus(const device &dac, uint8_t reg, uint16_t &value, bool debug) {
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

  if (!i2conf::i2c_set_device(dac)) {
    Log(error,"i2c_read_smbus : Failed to update device pointer.");
    return 0;
  }

  int32_t ret = i2c_smbus_read_word_data(dac.fd,reg);
  if (ret < 0) { // Returns contents and -1 on failure
    Log(error,"i2c_read_smbus : Failed to read contents of DAC.");
    Log(error,"message : %s",strerror(errno));
    return 0;
  } else {
    value = ret & 0xFFFF;
    if (debug) {
      printf("i2c_read_smbus : Contents from 0x%X : 0x%X.",reg,value);
    }
  }
  return 1;
}


// Lower level functions, that deal with the I2C devices at a lower level 
// by writing and reading directly from the file descriptor
// Does the same as above, but using direct device memory write
int32_t i2conf::i2c_write(const device& dac, uint8_t*buffer, const ssize_t& bytes, bool debug) {
  // refuse to write to an unopen device
  if (dac.fd < 0) {
    Log(error,"i2c_write : Attempting to write to invalid FD (%i). Aborting.",dac.fd);
    return 0;
  }

  ssize_t wr_bytes;
  ssize_t idx = 0;
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
  if (!i2conf::i2c_set_device(dac)) {
    Log(error,"i2c_write : Failed to update device pointer.");
    return 0;
  }

  wr_bytes = write(dac.fd,buffer,bytes);
  if (wr_bytes != bytes) {
    Log(error,"i2c_write : Failed to write the message to DAC %i. Received %i (expected %u).",dac.channel,wr_bytes,bytes);
    return 0;
  } 
  return 1;
}

int32_t i2conf::i2c_read(const device& dac, uint8_t reg, uint8_t*&buffer, const ssize_t& bytes, bool debug) {
  if (dac.fd < 0) {
    Log(error,"i2c_write : Attempting to read from an invalid FD (%i). Aborting.",dac.fd);
    return 0;
  }

  ssize_t rd_bytes;
  ssize_t idx = 0;
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
    Log(error,"i2c_write : Failed to update device pointer.");
    return 0;
  }

  // -- First set the address
  buffer[0] = reg;
  ssize_t nbt = 1;
  if (!i2conf::i2c_write(dac,buffer,nbt,debug)) {
    Log(error,"i2c_read : Failed to set pointer.");
    return 0;
  }

  rd_bytes = read(dac.fd,buffer,bytes);
  if (rd_bytes != bytes) {
    Log(error,"i2c_read : Failed to read the register from DAC %i. Received %i (expected %u).",dac.channel,rd_bytes,bytes);
    return 0;
  }
  if (debug) {
    printf("i2c_read : Contents from register 0x%X: \n",reg);
    for (idx = 0; idx < bytes; idx++) {
      printf("[0x%X] ",buffer[idx]);
    }
    printf("\n");
  }

  return 1;
}

int32_t i2conf::set_dac_level(const device& dac, const command &cmd, const level &lvl, bool debug) {

  // refuse to write to an unopen device
  if (dac.fd < 0) {
    Log(error,"set_dac_level : Attempting to write to invalid FD (%i). Aborting.",dac.fd);
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


  if (!i2conf::i2c_write_smbus(dac,cmd.val,lvl.val,debug)) {
    Log(error,"set_dac_level : Failed to write to register 0x%X",cmd.val);
    return 0;
  }

  return 1;
}

int32_t i2conf::get_dac_level(const device& dac, const command &c, level &lvl,bool debug) {

  // refuse to write to an unopen device
  if (dac.fd < 0) {
    Log(error,"i2c_write : Attempting to write to invalid FD (%i). Aborting.",dac.fd);
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

  if (!i2conf::i2c_read_smbus(dac,c.val,lvl.val,debug)) {
    Log(error,"get_dac_level : Failed to read DAC %i.",dac.channel);
    return 0;
  }

  return 1;
}
///////////////////////////////////////////////////////////////////
int32_t i2conf::ConfigureDacs(std::vector<uint32_t> &dacs, bool debug) {

  int32_t fd;
  
  if (!i2conf::i2c_init(fd)) { //Init i2c device
    Log(error,"Failed to initialize.");
    return 0;
  }

  uint16_t threshold_table[ndacs_][nchannels_];

  i2conf::device dev[ndacs_];
  i2conf::command c;
  i2conf::level l;

  for (size_t i = 0; i < ndacs_; i++) {
    dev[i].channel = i;
    dev[i].addr = dac_addr_[i];
    dev[i].devname = "/dev/i2c-0";
    dev[i].fd = fd;
  }

  Log(verbose,"Setting %u dac thresholds to: ", dacs.size());
  //Load DAC values into a table
  int ii = 0;
  for (size_t jj = 0; jj < dacs.size(); jj++) {
     Log(verbose,"DAC %u Channel %u Value %u ", ii, (jj%nchannels_), dacs.at(jj));
    threshold_table[ii][jj%nchannels_] = dacs.at(jj);
    if (jj%nchannels_ == (nchannels_-1)) { ii++; }
  }

  if (debug) {
    printf("Printing the DAC level values. \n");
    for (size_t i = 0; i < ndacs_; i++) { //Dac settings
       printf("DAC %u : ",i);
      for (size_t j = 0; j < nchannels_; j++) { //Channel values
        printf("%04u ",threshold_table[i][j]);
      }
      printf("\n");
    }
  }

  //Reset all DAC levels to 0
  Log(info,"Resetting all DAC channels to 0");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs_; i++) {
    for (size_t j = 0; j < nchannels_; j++) {
      c.cmd.channel = j;
      l.set_int_level(0x0);
      if (!i2conf::set_dac_level(dev[i],c,l,false)) {
        Log(error,"Failed to set channel [%u:%u]",i,j);
        return 1;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }
  //Set the DAC values from config
  Log(info,"Configuring DAC thresholds");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs_; i++) {
    for (size_t j = 0; j < nchannels_; j++) {
      c.cmd.channel = j;
      l.set_int_level(threshold_table[i][j]);
      if (!i2conf::set_dac_level(dev[i],c,l,debug)) {
        Log(error,"Failed to set channel [%u:%u]",i,j);
        return 1;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }

  Log(info,"Performing DAC level consistency check.");
  Log(debug,"Debugging : Consistency check indicators [O : OK; X : Fail]");
   c.cmd.command = I2C_RD_N;
   for (size_t i = 0; i < ndacs_; i++) {
     if (debug) { printf("DAC %u : ",i); }
     for (size_t j = 0; j < nchannels_; j++) {
       c.cmd.channel = j;
       if (!i2conf::get_dac_level(dev[i],c,l,debug)) {
         Log(error,"Failed to read channel [%u:%u]",i,j);
         return 1;
       }
       if (debug) { printf("%04u(%s) ",l.get_int_level(),(l.get_int_level() == threshold_table[i][j])?"O":"X"); }
       if (l.get_int_level() != threshold_table[i][j]) { return 1; }
       sleep(0.1);
     }
     if (debug) { printf("\n"); }
   }

  Log(info,"Programmed and checked DAC levels successfully!");

  i2conf::i2c_exit(fd);
  return 0;
}
///////////////////////////////////////////////////////////////////

// -- Function to do a consistency check in the DACs. 
// -- It writes a random table, and then reads back the values 
int32_t i2conf::dac_test(bool debug) {
  //const size_t ndacs = 3;
  //const size_t nchannels = 8;
  uint16_t threshold_table[ndacs_][nchannels_];
  std::srand((int)std::time(0));

  i2conf::device dev[ndacs_];
  i2conf::command c;
  i2conf::level l;
  //uint8_t DACs [3] = {0x48, 0x4A, 0x4C};
  int32_t fd;
  if (!i2conf::i2c_init(fd)) {
    Log(error,"Failed to initialize.");
    return 0;
  }

  for (size_t i = 0; i < ndacs_; i++) {
    dev[i].channel = i;
    dev[i].addr = DACs[i];
    dev[i].devname = "/dev/i2c-0";
    dev[i].fd = fd;
  }

  // -- First read the existing contents
  Log(info,"Reading the existing thresholds ");

  c.cmd.command = I2C_RD_N;

  for (size_t i = 0; i < ndacs_; i++) {
    printf("DAC %u : ",i);
    for (size_t j = 0; j < nchannels_; j++) {
      printf("%u",j);
      c.cmd.channel = j;
      if (!i2conf::get_dac_level(dev[i],c,l,debug)) {
        Log(error,"Failed to read channel [%u:%u]",i,j);
        return 0;
      }
      Log(info,"%04u ",l.get_int_level());
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }

  Log(info,"Initializing random settings ");
  for (size_t i = 0; i < ndacs_; i++) {
    if (debug) { printf("DAC %u : ",i); }
    for (size_t j = 0; j < nchannels_; j++) {
      threshold_table[i][j] = (uint16_t) (rand() % 4095);
      if (debug) { printf("%04u ",threshold_table[i][j]); }
    }
    printf("\n");
  }

  Log(info,"Setting the new thresholds ");
  c.cmd.command = I2C_WR_UPD_N;
  for (size_t i = 0; i < ndacs_; i++) {
    if (debug) { printf("DAC %u", i); }
    for (size_t j = 0; j < nchannels_; j++) {
      c.cmd.channel = j;
      l.set_int_level(threshold_table[i][j]);
      if (!i2conf::set_dac_level(dev[i],c,l,debug)) {
        Log(error,"Failed to set channel [%u:%u]",i,j);
        return 0;
      }
      // -- sleep a short while to make sure that we are not updating this too fast
      sleep(0.1);
    }
  }

  Log(info,"Performing consistency check...: [O : OK; X : Fail]");
  c.cmd.command = I2C_RD_N;
  for (size_t i = 0; i < ndacs_; i++) {
    if (debug) { printf("DAC %u : ",i); }
    for (size_t j = 0; j < nchannels_; j++) {
      c.cmd.channel = j;
      if (!i2conf::get_dac_level(dev[i],c,l,false)) {
        Log(error,"Failed to read channel [%u:%u]",i,j);
        return 0;
      }
      if(debug) { printf("%04u(%s) ",l.get_int_level(),(l.get_int_level() == threshold_table[i][j])?"O":"X"); }
      if(l.get_int_level() != threshold_table[i][j]) { 
         Log(error,"DAC %u Channel %u failure during R/W test ",i,j);   
         return 0;
      }
      sleep(0.1);
    }
  }

  i2conf::i2c_exit(fd);

  return 1;
}

}
