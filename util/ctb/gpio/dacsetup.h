/*
 * dacsetup.h
 *
 *  Created on: Feb 1, 2018
 *      Author: 
 */

#ifndef DACSETUP_H
#define DACSETUP_H

#include <cstdint>
#include <vector>

//using namespace std;

class I2C {
  
  public:
  ////////////////////////////////////////////////
  // I2C structures for content manipulation
  ////////////////////////////////////////////////
  
  // -- Channel masks -- //
  
  #define CH0 0x0
  #define CH1 0x1
  #define CH2 0x2
  #define CH3 0x3
  #define CH4 0x4
  #define CH5 0x5
  #define CH6 0x6
  #define CH7 0x7
  #define ALL_CH 0xF
  
  // -- Commands -- //
  
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
      uint8_t     val;
    } command;
  
    // -- NFB : I am flipping these on purpose, so that I can deal with the
    // endianness without having to go insane
    typedef struct level_t {
      uint16_t lsdb : 8;
      uint16_t msdb : 8;
    } level_t;
  
    typedef union level {
      level_t  level;
      uint16_t val;
      // Utility function that given a DAC integer value, sets the appropriate bits
      // -> This is because this device is stupid and the level are set on the
      //   higher bits while the lower are ignored.
      // level : word[15:4]
      // The range is 12 bits, so max value is 4095
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
  
    //The 3 DACs on the CTB
    const size_t ndacs = 3;
    const size_t nchannels = 8;
    const uint8_t DAC[3] = {0x48,0x4A,0x4C};

    //I2C class functions
    I2C();
    int32_t dac_test(bool debug = false);
    void i2c_exit(const int32_t &fd,  bool debug = true);
    int32_t i2c_set_device(const device &dac);
    int32_t i2c_init( int32_t &fd,const std::string &device = "/dev/i2c-0", bool debug = true);
    int32_t i2c_write_smbus(const device &dac, uint8_t reg, uint16_t value, bool debug = true);
    int32_t i2c_read_smbus(const device &dac, uint8_t reg, uint16_t &value, bool debug);
    int32_t i2c_write(const device& dac, uint8_t*buffer, const size_t& bytes, bool debug = false);
    int32_t i2c_read(const device& dac, uint8_t reg, uint8_t*&buffer, const size_t& bytes, bool debug = false);
    int32_t set_dac_level(const device& dac, const command &cmd, const level &levels, bool debug = false);
    int32_t get_dac_level(const device& dac, const command &c, level &levels,bool debug = false);
    int32_t ConfigureDacs(std::vector<uint32_t> &dacs, bool debug = false);


}; //end I2C class

#endif
