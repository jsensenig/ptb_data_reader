/*
 * i2conf.h
 *
 *  Created on: Feb 3, 2018
 *      Author: nbarros
 */

#ifndef I2CONF_H_
#define I2CONF_H_

#include <vector>

extern "C" {
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
}
#include <cstdint>
#include <cstdlib>
#include <ctime>

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
      } level;

      /// -- Methods have been implemented

      i2conf();
      virtual ~i2conf();
      void init();
      void set_dacs();
      void set_dac(size_t ch, uint16_t val);
      uint16_t get_dac(size_t ch);

      void set_dev(const std::string dev);
      const std::string &get_dev() {return dev_;}

    protected:

      static const uint32_t dac_nbytes_ = 3;
      const uint16_t ndacs_ = 3;
      static const uint16_t dac_addr_[ndacs_] = {0x48,0x4A,0x4C};

      // -- Variables that have really to be kept here
      std::vector<size_t> dac_threshold_;
      std::string dev_;
      bool init_done_;

      device dac_dev_[ndacs_];

  };

} /* namespace ptb */

#endif /* I2CONF_H_ */
