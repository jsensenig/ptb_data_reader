/*
 * i2conf.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: nbarros
 */

#include "i2conf.h"

namespace ptb {

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
