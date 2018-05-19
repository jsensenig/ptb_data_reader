/*
 * i2conf.h
 *
 *  Created on: Feb 3, 2018
 *      Author: nbarros
 */

#ifndef I2CONF_H_
#define I2CONF_H_

#include <vector>
#include <string>

extern "C" {
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
}
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include "Logger.h"

using std::string;

extern int com_serial;
extern int failcount;

namespace ptb {

  class i2conf {
    public:

      enum channel {CH_0=0x0, CH_1=0x1,CH_2=0x2,CH_3=0x3,CH_4=0x4,CH_5=0x5,CH_6=0x6,CH_7=0x7,CH_ALL=0xF };
      // -- Commands

      /// Define a couple of commands

      enum dac_cmd {I2C_WR_IN_N=0x0,      // not sure of what it does
                    I2C_RD_N=0x1,         // Read one channel
                    I2C_WR_UPD_ALL=0x2,   // Write all channels
                    I2C_WR_UPD_N=0x3,     // Write channel N (mask)
                    I2C_POWER=0x4,        // Power on/off
                    I2C_CLEAR=0x5,        // Clear
                    I2C_LOAD=0x6,         // Load registers (?)
                    I2C_RESET=0x7         // Reset
                    };

      typedef struct device {
          int32_t     fd;
          uint16_t    addr;
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

      typedef struct level_t {
          uint16_t lsdb : 8;
          uint16_t msdb : 8;
      } level_t;

      typedef union level {
          level_t  level;
          uint16_t     val;
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

      /// -- Methods have been implemented

      i2conf();
      virtual ~i2conf();
      void init();
      int32_t i2c_init( int32_t &fd,const std::string &device = "/dev/i2c-0", bool debug = false);
      int32_t i2c_set_device(const device &dac);
      int32_t i2c_write_smbus(const device &dac, uint8_t reg, uint16_t value, bool debug);
      int32_t i2c_read_smbus(const device &dac, uint8_t reg, uint16_t &value, bool debug);
      int32_t i2c_write(const device& dac, uint8_t*buffer, const ssize_t& bytes, bool debug);
      int32_t i2c_read(const device& dac, uint8_t reg, uint8_t*&buffer, const ssize_t& bytes, bool debug);
      int32_t set_dac_level(const device& dac, const command &cmd, const level &lvl, bool debug);
      int32_t get_dac_level(const device& dac, const command &c, level &lvl,bool debug);
      int32_t ConfigureDacs(std::vector<uint32_t> &dacs, bool debug);
      int32_t dac_test(bool debug);
      void i2c_exit(const int32_t &fd, bool debug=false);
      void set_dacs();
      void set_dac(size_t ch, uint16_t val);
      uint16_t get_dac(size_t ch);

      void set_dev(const std::string dev);
      const std::string &get_dev() {return dev_;}


      static const uint32_t dac_nbytes_ = 3;
      static const uint16_t ndacs_ = 3;
      //static const uint16_t nchannels_ = 8;
      static const size_t nchannels_ = 8;
      static const uint16_t dac_addr_[ndacs_];// = {0x48,0x4A,0x4C};

    protected:

      // -- Variables that have really to be kept here
      std::vector<size_t> dac_threshold_;
      std::string dev_;
      bool init_done_;

      device dac_dev_[ndacs_];

  };

} /* namespace ptb */

#endif /* I2CONF_H_ */
