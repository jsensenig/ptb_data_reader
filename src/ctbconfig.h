/*
 * ctbconfig.h
 *
 *  Created on: Jun 12, 2018
 *      Author: jon
 */

#ifndef SRC_CTBCONFIG_H_
#define SRC_CTBCONFIG_H_

#include "Logger.h"
#include "util.h"
#include "config.h"
#include "ptb_registers.h"
#include "boardmanager.h"

#include <map>
#include <cstdint>
#include <sstream>
#include <thread>

#include "json.hpp"

#define NBEAM_CH 9
#define NPDS_CH 32
#define NCRT_CH 24

using json = nlohmann::json;

namespace ptb {

  namespace ctbconfig {

    //Misc configs such as timing, pulser, etc
    void misc_config(json & miscconfig, json& feedback);

    void pds_config(json & pdsconfig, json& feedback);

    //Loads the Beam config into FW regs
    void beam_config(json & beamconfig, json& feedback);

    //Loads the CRT config into FW regs
    void crt_config(json & crtconfig, json& feedback);

  }; //ctbconfig namespace

}; //ptb namespace


#endif /* SRC_CTBCONFIG_H_ */
