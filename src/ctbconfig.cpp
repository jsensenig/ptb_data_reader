/*
 * ctbconfigs.cpp
 *
 *  Created on: Jun 12, 2018
 *      Author: jon
 *
 * To increase the generality of the ptbreader (Xreader?) the CTB specific
 * configurations have been moved to this file. 
 */
#include "ctbconfig.h"
#include "boardmanager.h"
#include "Logger.h"
#include "ptb_registers.h"
#include "i2conf.h"

#include <iomanip>
#include <thread>
#include <chrono>
#include <bitset>
#include <cmath>
#include <cinttypes>

#include "json.hpp"

using json = nlohmann::json;

#define __STDC_FORMAT_MACROS

extern "C" {
#include <unistd.h>
}

namespace ptb {
  namespace ctbconfig {

    //ptb::board_manager BM;
    board_manager BM;

    ////////////////////////////////////////////
    // Beam Configuration

    void beam_config(json& beamconfig, json& feedback) {

      size_t err_msgs = 8;
      std::vector<std::ostringstream> tmp(err_msgs);

      json trigs = beamconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = beamconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      if (channelmask == 0) { tmp[0] <<  "Beam channel mask set for 0x0."; }

      //Mask type triggers (inclusive & exclusive)
      bool llt3_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_llt3mask = trigs.at(0).at("mask").get<std::string>();
      uint32_t llt3mask = (int)strtol(s_llt3mask.c_str(),NULL,0);
      //if (llt3_enable) { tmpi[0] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " " << trigs.at(0).at("description").get<std::string>() << " configured"; }
      if (llt3mask == 0) { tmp[1] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; }
      bool llt4_enable = trigs.at(1).at("enable").get<bool>();
      std::string s_llt4mask = trigs.at(1).at("mask").get<std::string>();
      uint32_t llt4mask = (int)strtol(s_llt4mask.c_str(),NULL,0);
      if (llt4mask == 0) { tmp[2] << "Trigger " << trigs.at(1).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; }
      std::string s_llt5mask = trigs.at(2).at("mask").get<std::string>();
      bool llt5_enable = trigs.at(2).at("enable").get<bool>();
      uint32_t llt5mask = (int)strtol(s_llt5mask.c_str(),NULL,0);
      if (llt5mask == 0) { tmp[3] << "Trigger " << trigs.at(2).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; }
      bool llt6_enable = trigs.at(3).at("enable").get<bool>();
      std::string s_llt6mask = trigs.at(3).at("mask").get<std::string>();
      uint32_t llt6mask = (int)strtol(s_llt6mask.c_str(),NULL,0);
      if (llt6mask == 0) { tmp[4] << "Trigger " << trigs.at(3).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; }

      //Counting triggers
       bool llt12_enable = trigs.at(4).at("enable").get<bool>();
       std::string s_llt12mask = trigs.at(4).at("mask").get<std::string>();
       std::string s_llt12count = trigs.at(4).at("count").get<std::string>();
       std::string s_llt12type = trigs.at(4).at("type").get<std::string>();
       uint32_t llt12mask = (int)strtol(s_llt12mask.c_str(),NULL,0);
       uint8_t llt12count = (int)strtol(s_llt12count.c_str(),NULL,0);
       uint8_t llt12type = (int)strtol(s_llt12type.c_str(),NULL,0);
       if ((llt12count < 2 && llt12type == 0) || (llt12count < 1 && llt12type == 1)) { tmp[5] << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") will result in always asserted condition.";}
       if (llt12mask == 0) { tmp[6] << "Trigger " << trigs.at(4).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; }
       if (llt12type > 4 || llt12type == 3) {
         tmp[7] << "Trigger " << trigs.at(4).at("id").get<std::string>() << " type (" << (int)llt12type << ") logic type undefined! Setting to '==' (0x1). ";
         llt12type = 0;  //Double check this is actually "=="
       }

       //Input channel masks
       BM.set_bit_range_register(3,0,9,channelmask);
       //Trigger enables
       BM.set_bit(27,3,llt3_enable);
       BM.set_bit(27,4,llt4_enable);
       BM.set_bit(27,5,llt5_enable);
       BM.set_bit(27,6,llt6_enable);
       BM.set_bit(27,12,llt12_enable);

       uint32_t llt12conf = (llt12type<<5) + llt12count;
       //Trigger parameters
       BM.set_bit_range_register(30,0,32,llt3mask);
       BM.set_bit_range_register(31,0,32,llt4mask);
       BM.set_bit_range_register(32,0,32,llt5mask);
       BM.set_bit_range_register(33,0,32,llt6mask);
       BM.set_bit_range_register(39,0,8,llt12conf);
       //BM.set_bit_range_register(40,0,32,llt12mask);

       uint8_t dbits = 7;
       // PDS delays 24ch * 7b = 168b --> 6 regs
       for(size_t i=0; i<(size_t)(delays.size()/4); i++) {
                                                                                                                    
         int j = i * 4;
         uint32_t reg = i + 21;
         uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j]; 
                                                                                                                    
         BM.set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
       }
       
       BM.set_bit_range_register(23,0,(uint32_t)dbits,delays[delays.size()-1]);

       //Place warnings into json feedback obj
       json obj;
       for (size_t i=0; i<tmp.size(); i++) {
         if (tmp[i].str().compare("") != 0) {
           obj["type"] = "warning";
           obj["message"] = tmp[i].str();
           feedback.push_back(obj);
         }
       }

    } //Beam config

    ////////////////////////////////////////////
    // CRT configuration

    void crt_config(json& crtconfig, json& feedback) {

      size_t err_msgs = 3;
      std::vector<std::ostringstream> tmp(err_msgs);

      json trigs = crtconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = crtconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = crtconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      if (channelmask == 0) { tmp[0] << "CRT channel mask set for 0x0."; }

      //Mask type triggers (inclusive & exclusive)
      bool llt2_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_llt2mask = trigs.at(0).at("mask").get<std::string>();
      uint32_t llt2mask = (int)strtol(s_llt2mask.c_str(),NULL,0);
      if (llt2mask == 0) { tmp[1] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled!"; }
      bool llt7_enable = trigs.at(1).at("enable").get<bool>();
      std::string s_llt7mask = trigs.at(1).at("mask").get<std::string>();
      uint32_t llt7mask = (int)strtol(s_llt7mask.c_str(),NULL,0);
      if (llt7mask == 0) { tmp[2] << "Trigger " << trigs.at(1).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled!"; }

      //Input channel masks
      BM.set_bit_range_register(1,0,32,channelmask);
      //Trigger enables
      BM.set_bit(27,2,llt2_enable);
      BM.set_bit(27,7,llt7_enable);
      //Trigger parameters
      BM.set_bit_range_register(29,0,32,llt2mask);
      BM.set_bit_range_register(34,0,32,llt7mask);

      uint8_t dbits = 7;
      // PDS delays 24ch * 7b = 168b --> 6 regs
      for(size_t i=0; i<delays.size()/4; i++) {
                                                                                                                   
        int j = i * 4;
        uint32_t reg = i + 13;
        uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j]; 
                                                                                                                   
        BM.set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
      }

      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].str().compare("") != 0) {
          obj["type"] = "warning";
          obj["message"] = tmp[i].str();
          feedback.push_back(obj);
        }
      }

    } //CRT config

    ////////////////////////////////////////////
    // PDS configuration

    void pds_config(json& pdsconfig, json& feedback) {
      ///FIXME: How can this work? dacsetup is not initialized in this function
      i2conf dacsetup;

      size_t err_msgs = 3;
      std::vector<std::ostringstream> tmp1(err_msgs);

      std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
      std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = pdsconfig.at("delays").get<std::vector<uint32_t>>();
      std::string s_trigtype0 = pdsconfig.at("triggers").at(0).at("type").get<std::string>();
      std::string s_count0 = pdsconfig.at("triggers").at(0).at("count").get<std::string>();
      bool llt11_enable = pdsconfig.at("triggers").at(0).at("enable").get<bool>();
 
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      uint8_t trigtype0 = (int)strtol(s_trigtype0.c_str(),NULL,0);
      uint8_t count0 = (int)strtol(s_count0.c_str(),NULL,0);

      if (delays.size() != NBEAM_CH) {
        Log(warning, "Number of configuration values (%i) doesn't match number of DAC channels (%i)!", dac_values.size(), (i2conf::nchannels_)*(i2conf::ndacs_));
        std::ostringstream tmp;
        tmp1[0] << "Number of configuration values (" << dac_values.size() << ") doesn't match number of DAC channels (" << (i2conf::nchannels_)*(i2conf::ndacs_) << ")";
      }

       for (size_t i=0; i<delays.size(); i++) {
         if (delays[i] > std::pow(2,7)) { //Range 0 - 2^7 -1
           Log(warning, "Warning delay value out of range, will be set to max value.");
           std::ostringstream tmp;
           tmp1[1] << "Delay value out of range (" << delays.at(i) << "). Truncating to maximum (127)";
         }
       }



      if (dac_values.size() != (i2conf::nchannels_)*(i2conf::ndacs_)) {
        Log(warning, "Number of configuration values (%i) doesn't match number of DAC channels (%i)!", dac_values.size(), (i2conf::nchannels_)*(i2conf::ndacs_));
        std::ostringstream tmp;
        tmp << "Number of configuration values (" << dac_values.size() << ") doesn't match number of DAC channels (" << (i2conf::nchannels_)*(i2conf::ndacs_) << ")";

        json obj;
        obj["type"] = "warning";
        obj["message"] = tmp.str();
        feedback.push_back(obj);
      }

      Log(info,"Size of channel values vector %i", dac_values.size());
      for (size_t i=0; i<dac_values.size(); i++) {
        Log(info,"Channel %zu value %u", i, dac_values[i]);
        if (dac_values[i] > 4095) { //Range 0 - 4095
          Log(warning, "Warning DAC value out of range, will be set to max value.");
          std::ostringstream tmp;
          tmp << "DAC value out of range (" << dac_values.at(i) << "). Truncating to maximum (4095)";
          json obj;
          obj["type"] = "warning";
          obj["message"] = tmp.str();
          feedback.push_back(obj);
          dac_values[i] = 4095;
        }
      }
      //Now pass DAC configs to setup
      if (dacsetup.ConfigureDacs(dac_values,false)) {
        Log(error,"Failed to write configuration values to DACs.");
        json obj;
        obj["type"] = "error";
        obj["message"] = "Failed to write configuration values to DACs";
        feedback.push_back(obj);
      }
      Log(info,"Programmed %zu DAC channels", dac_values.size());

      //Input channel masks
      BM.set_bit_range_register(2,0,24,channelmask);

      //Configure counting trigger 0
      uint32_t trig0 = (trigtype0<<5) + count0;
      BM.set_bit_range_register(38,0,9,trig0);
      BM.set_bit(27,11,llt11_enable);

     uint8_t dbits = 7;
     // PDS delays 24ch * 7b = 168b --> 6 regs
     for(size_t i=0; i<delays.size()/4; i++) {

       int j = i * 4;
       uint32_t reg = i + 7;
       uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j]; 

       BM.set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
     }

    } //PDS config

    ////////////////////////////////////////////
    // Misc configuration

    void misc_config(json& miscconfig, json& feedback) {

      json rtrigger = miscconfig.at("randomtrigger");
      bool rtrigger_en = rtrigger.at("enable").get<bool>();
      uint32_t rtriggerfreq = rtrigger.at("frequency").get<unsigned int>();
      Log(debug,"Random Trigger Frequency [%d] (%u) [0x%X][%s]",rtriggerfreq,rtriggerfreq,rtriggerfreq, std::bitset<26>(rtriggerfreq).to_string().c_str());
      if (rtriggerfreq >= (1<<26)) {
        Log(warning,"Input value of [%u] above maximum rollover [26]. Truncating to maximum.",rtriggerfreq);
        rtriggerfreq = (1<<26)-1;
      } 
      BM.set_bit(27,0,rtrigger_en);
      BM.set_bit_range_register(25,0,26,rtriggerfreq);
      //Set pulser frequency
      json pulserconf = miscconfig.at("pulser");
      bool pulser_en = pulserconf.at("enable").get<bool>();
      uint32_t pulserfreq = pulserconf.at("frequency").get<unsigned int>();
      Log(debug,"Pulser Frequency [%d] (%u) [0x%X][%s]",pulserfreq,pulserfreq,pulserfreq, std::bitset<26>(pulserfreq).to_string().c_str());
      if (pulserfreq >= (1<<26)) {
        Log(warning,"Input value of [%u] above maximum rollover [26]. Truncating to maximum.",pulserfreq);
        pulserfreq = (1<<26)-1;
      } 
      BM.set_bit(26,31,pulser_en);
      BM.set_bit_range_register(26,0,26,pulserfreq);

    } // Misc configs

  } // ctbconfig namespace

} //ptb namepsace


