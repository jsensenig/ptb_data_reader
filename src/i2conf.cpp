/*
 * i2conf.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: nbarros
 */

#include "i2conf.h"

namespace ptb {

  const uint16_t i2conf::dac_addr_[ndacs_] = {0x48,0x4A,0x4C};

  i2conf::i2conf() : dev_("/dev/i2c-0"),init_done_(false) {


    // Don't initialize here
    // -- set up the internal structures
    for (size_t i = 0; i < ndacs_; i++) {
      dac_dev_[i].channel = i;
      dac_dev_[i].addr = dac_addr_[i];
      dac_dev_[i].devname = dev_;
    }

  }

  i2conf::~i2conf() {
    // TODO Auto-generated destructor stub
  }

  void i2conf::init() {
    // Initiqlization is done
    if (init_done_) return;

    // initialization is not done. Start it
    for (size_t i = 0; i < ndacs_; i++) {

    }

  }

} /* namespace ptb */
