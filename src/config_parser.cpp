/*
 * config_parser.cpp
 *
 *  Created on: Oct 7, 2017
 *      Author: nbarros
 */

#include "boardmanager.h"

namespace ptb {
// FIXME: Reimplement this
void board_manager::process_config(pugi::xml_node config,std::string &answers) {
  msgs_.clear();
  msgs_.str("");

  // Only accept a new configuration if we are not running.
  // NFB: Not sure if it shouldn't always accept a config but simply place it in the cache.
  //      This could turn into a mess if we didn't know what was the latest good running configuration

  if (board_state_ == RUNNING) {
    msgs_ << "<warning>Attempted to pass a new configuration during a run. Ignoring the new configuration.</warning>";
    Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
    answers = msgs_.str();
    return;
  }

  std::stringstream strVal;

  // use this as an error catching state
  bool has_error = false;

  try{
    Log(info,"Applying a reset prior to the configuration.");
    set_reset_bit(true);

    std::this_thread::sleep_for (std::chrono::microseconds(10));
    set_reset_bit(false);

    //FIXME: Finish implementing the configuration parser

    // This is the workhorse of the configuration.
    // At this point the registers should already be mapped and things should be flowing
//    std::ostringstream document;
//    config.print(document);
//    Log(debug,"Configuration fragment [[[\n\n%s\n\n]]]",document.str().c_str());
    Log(info,"Parsing the configuration." );


    for (pugi::xml_node_iterator it = config.begin(); it != config.end(); ++it) {
      Log(verbose," Child name : %s",it->name());
      // The reader should take care of this by itself.
      ///!
      ///! DataBuffer
      ///!

      if (!strcmp(it->name(),"DataBuffer")) {
        if (!reader_) {
          Log(warning,"Don't have a valid PTBReader instance. Attempting to create a new one." );
          reader_ = new board_reader();
        }

        //-- Get the host
        data_socket_host_ = it->child("DaqHost").child_value();
        reader_->set_tcp_host(data_socket_host_);
        // -- Get the port
        strVal << it->child("DaqPort").child_value();
        strVal >> data_socket_port_;
        strVal.clear();
        strVal.str("");
        //      unsigned short port = atoi(it->child("DaqPort").child_value());
        Log(debug,"DaqPort port %hu",data_socket_port_ );
        reader_->set_tcp_port(data_socket_port_);
        Log(debug,"Setting data transmission channel to [%s:%hu]",data_socket_host_.c_str(),data_socket_port_);

        // -- Get if it is a dry run
        {
          std::string dry_run_state = it->child("DryRun").child_value();
          if (!dry_run_state.compare("true")) {
            reader_->set_dry_run(true);
            Log(warning,"Dry run mode enabled in the PTB. No data will be sent to board reader.");
          } else {
            reader_->set_dry_run(false);
            Log(debug,"Dry run mode disabled in the PTB.");
          }
        }
#ifdef ENABLE_FRAG_BLOCKS
        uint32_t rollOver;
        strVal << it->child("RollOver").child_value();
        strVal >> rollOver;
        strVal.clear();
        strVal.str("");
        Log(debug,"Packet Rollover %u",rollOver);
        reader_->setPacketRollover(rollOver);
#endif
        uint32_t duration;
        strVal <<it->child("MicroSliceDuration").child_value();
        strVal >> std::dec >> duration;

        // Microslice duration is now a full number...check if it fits into 27 bits
        Log(debug,"MicroSlice Duration [%s] (%u) [0x%X][%s]",strVal.str().c_str(),duration,duration, std::bitset<27>(duration).to_string().c_str());
        if (duration >= (1<<27)) {
          msgs_ << "<warning>Input value of [" << duration << "] above the allowed limit of 27 bits. Setting to maximum allowed.";
          Log(warning,"Input value of [%u] above maximum rollover [27]. Truncating to maximum.",duration);
          duration = (1<<27)-1;
        }

        SetBitRangeRegister(35,duration,0,27);
        Log(debug,"Register 35 : [0x%08X]", register_map_[35].value() );
        strVal.clear();
        strVal.str("");

      }
      if (!strcmp(it->name(),"ChannelMask")) {
        // Deal directly with the childs.

        uint64_t bsu;
        strVal <<std::hex << it->child("BSU").child_value();
        strVal >> bsu;
        strVal.clear();
        strVal.str("");

        // Now I want to get the first 32 bits into a register
        Log(debug,"BSU mask [0x%" PRIX64 "]",bsu );
        Log(debug,"BSU mask [%s]",std::bitset<50>(bsu).to_string().c_str());
        // Make the assignments that I had decided before:
        // Catch the lowest 32 bits of the word
        ////// ------
        ////// REG 1 (LSB OF CHANNEL MASK)
        ////// BSU[31-0]
        ////// ------

        register_map_[1].value() = bsu & 0xFFFFFFFF;
        Log(debug,"Register 1 : [0x%08X]", register_map_[1].value() );

        ////// ------
        ////// REG 2 (MSB of BSU channel mask + LSB of TSU channel mask)
        ////// TSU[13-0] + BSU[49-32]
        ////// ------
        // The remaining 18 bits (49-32) go into the lower 18 bits of the next word
        // [0-17]
        register_map_[2].value() = (bsu >> 32 ) & 0x3FFFF;
        Log(debug,"Register 2 : {17-0} : [0x%08X]",register_map_[2].value() );

        // Now grab the TSU part to complete the mask of this register
        uint64_t tsu;
        strVal << std::hex << it->child("TSU").child_value();
        strVal >> tsu;
        strVal.clear();
        strVal.str("");

        Log(debug,"TSU mask [0x%" PRIX64 "]",tsu );
        Log(debug,"TSU mask [%s]",std::bitset<48>(tsu).to_string().c_str());


        // The lowest 14 (0-13) bits go into the upper bits of the previous register
        // [18-31]
        SetBitRangeRegister(2,((tsu & 0x3FFF) << 18),18,14);
        Log(debug,"Register 2 : {31-18} : [0x%08X]", register_map_[2].value() );

        ////// ------
        ////// REG 3   (MSB of channel mask)
        ////// TSU[45-14]
        ////// ------

        // The remaining 34 (47-14) bits go into the next registers
        // (14-45)
        register_map_[3].value() = ((tsu >> 14) & 0xFFFFFFFF);
        Log(debug,"Register 3 : [0x%08X]",register_map_[3].value() );

        ////// ------
        ////// REG 4
        ////// TSU[47-46] (LSB)
        ////// ------

        // The final 2 bits go into the lsb of register 4
        SetBitRangeRegister(4,((tsu >> 45) & 0x3),0,2);
        Log(debug,"Register 4 {1-0}: [0x%08X]",register_map_[4].value() );
      }


      if (!strcmp(it->name(),"ExtTriggers")) {
        // External triggers go into register 37
        // to bits [0-3]
        uint32_t TRIGEX;
        strVal <<std::hex << it->child("mask").child_value();
        strVal >> TRIGEX;

        Log(debug,"TRIGEX (%s) [0x%X] [%s]",strVal.str().c_str(),TRIGEX,std::bitset<4>(TRIGEX).to_string().c_str());

        SetBitRangeRegister(37,(TRIGEX & 0xF),0,4);
        Log(debug,"Register 37 {3-0} : [0x%08X]",register_map_[37].value());
        strVal.clear();
        strVal.str("");

        // enable echo
        std::string enable;
        uint32_t gate  = 0;
        uint32_t prescale = 0;
        enable = it->child("echo_enabled").child_value();
        Log(debug,"ExtTrig enable : %s",enable.c_str());

        if (!enable.compare("true")) {
          SetBitRangeRegister(37,(1<<15),15,1);
        }
        Log(debug,"Register 37 {15} : [0x%08X]",register_map_[37].value());

        strVal << std::dec << it->child("gate").child_value();
        strVal >> std::dec >> gate;

        Log(debug,"TrigEx Gate (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),gate,gate,std::bitset<11>(gate).to_string().c_str());
        if (gate > ((1<<11)-1)) {
          msgs_ << "<warning>TriEx gate larger than maximum allowable size. Truncating to max value ( 2047 : 0x7FF)</warning>";
          Log(warning,"TriEx gate larger than maximum allowable size. Truncating to max value ( 2047 : 0x7FF)");
          gate = (1<<11)-1;
          Log(debug,"TrigEx Gate (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),gate,gate,std::bitset<11>(gate).to_string().c_str());
        }
        SetBitRangeRegister(37,(gate << 4),4,11);
        Log(debug,"Register 37 {14-4} : [0x%08X]",register_map_[37].value());
        strVal.clear();
        strVal.str("");


        // prescale
        strVal << std::dec << it->child("prescale").child_value();
        strVal >> std::dec >> prescale;

        Log(debug,"TrigEx Prescale (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),prescale,prescale,std::bitset<8>(prescale).to_string().c_str());
        if (prescale > ((1<<8)-1)) {
          msgs_ << "<warning>TriEx prescale larger than maximum allowable size. Truncating to max value ( 255 : 0xFF)</warning>";
          Log(warning,"TriEx prescale larger than maximum allowable size. Truncating to max value ( 255 : 0xFF)");
          prescale = (1<<8)-1;
          Log(debug,"TrigEx Prescale (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),prescale,prescale,std::bitset<8>(prescale).to_string().c_str());

        }

        SetBitRangeRegister(37,((prescale & 0xFF) << 16),16,8);
        Log(debug,"Register 37 {23-16} : [0x%08X]",register_map_[37].value());
        strVal.clear();
        strVal.str("");

      }

      if (!strcmp(it->name(),"Hardware")) {
        // M_PULSEWIDTH - Number of clock cycles to keep the any outgoing pulse (out high)
        // Width of the outgoing pulses
        // TrigOutWidth : [18-23]
        uint32_t M_PULSEWIDTH;
        strVal <<std::dec << it->child("PulseWidth").child_value();
        strVal >> std::dec >> M_PULSEWIDTH;

        Log(debug,"M_PULSEWIDTH [%s] (%u) [0x%X] [%s]",strVal.str().c_str(),M_PULSEWIDTH,M_PULSEWIDTH,std::bitset<6>(M_PULSEWIDTH).to_string().c_str());
        if (M_PULSEWIDTH > ((1<<6)-1)) {
          msgs_ << "<warning>Pulse width larger than maximum allowable size. Truncating to max value ( " << ((1<<6)-1) << " : 0x3F)</warning>";
          Log(warning,"Pulse width larger than maximum allowable size. Truncating to max value ( %u : 0x3F)",((1<<6)-1));
          M_PULSEWIDTH = (1<<6)-1;
          Log(debug,"M_PULSEWIDTH [%s] (%u) [0x%X] [%s]",strVal.str().c_str(),M_PULSEWIDTH,M_PULSEWIDTH,std::bitset<6>(M_PULSEWIDTH).to_string().c_str());
        }
        SetBitRangeRegister(4,((M_PULSEWIDTH & 0x3F) << 18),18,6);
        Log(debug,"Register 4 {23-18} : [0x%08X]", register_map_[4].value());
        strVal.clear();
        strVal.str("");

      }
      // The most troublesome part: muon triggers
      if (!strcmp(it->name(),"MuonTriggers")) {
        // This one is quite complicated to parse.
        // For the moment let's deal with it statically...

        // The lockout window goes into register 3
        // LockoutWindow : [1-5]
        uint32_t MT_LOCKDOWN;
        strVal << std::dec << it->child("LockdownWindow").child_value();
        strVal >> std::dec >> MT_LOCKDOWN;


        Log(debug,"MT_LOCKDOWN (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),MT_LOCKDOWN,MT_LOCKDOWN,std::bitset<6>(MT_LOCKDOWN).to_string().c_str());

        if (MT_LOCKDOWN > ((1<<6)-1)) {
          msgs_ << "<warning>Lockdown larger than maximum allowable size. Truncating to max value ( " << ((1<<6)-1) << " : 0x3F)</warning>";
          Log(warning,"Lockdown larger than maximum allowable size. Truncating to max value ( %u : 0x3F)",((1<<6)-1));
          MT_LOCKDOWN = (1<<6)-1;
          Log(debug,"MT_LOCKDOWN (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),MT_LOCKDOWN,MT_LOCKDOWN,std::bitset<6>(MT_LOCKDOWN).to_string().c_str());
        }

        SetBitRangeRegister(4,((MT_LOCKDOWN & 0x3F) << 10),10,6);
        Log(debug,"Register 4 {15-10} [0x%08X]",register_map_[4].value() );
        strVal.clear();
        strVal.str("");

        // The remaining configuration words go into register 5

        // M_LOCKHI - Number of clock cycles to extend the input signal
        // Width of the trigger window
        // TriggerWindow : [22-24]
        uint32_t M_LOCKHI;
        strVal << std::dec << it->child("TriggerWindow").child_value();
        strVal >> std::dec >> M_LOCKHI;

        Log(debug,"M_LOCKHI (%s) (%u)[0x%X] [%s]",strVal.str().c_str(),M_LOCKHI,M_LOCKHI,std::bitset<4>(M_LOCKHI).to_string().c_str());
        if (M_LOCKHI > ((1<<4)-1)) {
          msgs_ << "<warning>Trigger gate larger than maximum allowable size. Truncating to max value ( " << ((1<<4)-1) << " : 0xF)</warning>";
          Log(warning,"Trigger gate larger than maximum allowable size. Truncating to max value ( %u : 0xF)",((1<<4)-1));
          MT_LOCKDOWN = (1<<4)-1;
          Log(debug,"M_LOCKHI (%s) (%u)[0x%X] [%s]",strVal.str().c_str(),M_LOCKHI,M_LOCKHI,std::bitset<4>(M_LOCKHI).to_string().c_str());
        }

        SetBitRangeRegister(4,((M_LOCKHI & 0xF) << 25),25,4);
        Log(debug,"Register 4 {28-25} [0x%08X]", register_map_[4].value() );

        strVal.clear();
        strVal.str("");

        // Now it is specific trigger codes...
        // Deal with one at a time

        ///! Create an empty node to define disabled triggers
        pugi::xml_node empty_node;

        pugi::xml_node mtrigger_node = it->find_child_by_attribute("TriggerMask","id","A");
        if (mtrigger_node.empty()){
          msgs_ << "<warning>Couldn't find the configuration for trigger A. Trigger will be disabled.</warning>";
          Log(warning,"Couldn't find the configuration for trigger A.");
          // If it doesn't find, disable this trigger
          ParseMuonTrigger(empty_node,5,11);
          //continue;
        } else {
          Log(debug,"\n\nParsing configuration for trigger A\n\n");
          // -- The operations go into register 5
          ParseMuonTrigger(mtrigger_node,5,11);
        }

        mtrigger_node = it->find_child_by_attribute("TriggerMask","id","B");
        if (mtrigger_node.empty()){
          msgs_ << "<warning>Couldn't find the configuration for trigger B. Trigger will be disabled.</warning>";
          Log(warning,"Couldn't find the configuration for trigger B." );
          ParseMuonTrigger(empty_node,12,11);
        } else {
          // -- The operations go into register 6
          Log(debug,"\n\nParsing configuration for trigger B\n\n");
          ParseMuonTrigger(mtrigger_node,12,11);
        }

        mtrigger_node = it->find_child_by_attribute("TriggerMask","id","C");
        if (mtrigger_node.empty()){
          msgs_ << "<warning>Couldn't find the configuration for trigger C. Trigger C will be disabled.</warning>";
          Log(warning,"Couldn't find the configuration for trigger C." );
          ParseMuonTrigger(empty_node,18,24);
        } else {
          // -- The operations go into register 18
          Log(debug,"\n\nParsing configuration for trigger C\n\n");
          ParseMuonTrigger(mtrigger_node,18,24);
        }

        mtrigger_node = it->find_child_by_attribute("TriggerMask","id","D");
        if (mtrigger_node.empty()){
          msgs_ << "<warning>Couldn't find the configuration for trigger D. Trigger D will be disabled.</warning>";
          Log(warning,"Couldn't find the configuration for trigger D." );
          ParseMuonTrigger(empty_node,25,24);
        } else {
          // -- The operations go into register 19
          Log(debug,"\n\nParsing configuration for trigger D\n\n");
          ParseMuonTrigger(mtrigger_node,25,24);
        }

      }

      //-- Now grab the calibration masks
      if (!strcmp(it->name(),"Calibrations")) {
        std::string enable;
        uint32_t period  = 0;
        uint32_t reg = 0;
        uint32_t reg_init = 31;
        const char *channel_id[] = {"C1","C2","C3","C4"};
        // this can be hardcoded since we don't have any more than that.
        for (uint32_t i = 0; i < 4; ++i) {
          pugi::xml_node node = it->find_child_by_attribute("CalibrationMask","id",channel_id[i]);
          if (node.empty()){
            msgs_ << "<warning>Couldn't find the configuration for calibration channel " << channel_id[i] << ". Disabling (default).</warning>";
            Log(warning,"Couldn't find the configuration for calibration channel %s. Disabling it.",channel_id[i]);
            enable = "false";
            period = 0x0;
          } else {
            enable = node.child("enabled").child_value();
            Log(debug,"Calibration channel %u enable : %s",i+1,enable.c_str());
            strVal << std::dec << node.child("period").child_value();
            strVal >> std::dec >> period;
            Log(debug,"%s PERIOD (%s) (%u) [0x%X] [%s]",channel_id[i],strVal.str().c_str(),period,period,std::bitset<30>(period).to_string().c_str());
            if (period > ((1<<30)-1)) {
              msgs_ << "<warning>Period of "<< channel_id[i] << " larger than maximum allowable size. Truncating to max value ( " << ((1<<30)-1) << " : 0x3FFFFFFF)</warning>";
              Log(warning,"Period of %s larger than maximum allowable size. Truncating to max value ( %u : 0x%X)",channel_id[i],((1<<30)-1),((1<<30)-1));
              period = (1<<30)-1;
              Log(debug,"%s PERIOD (%s) (%u) [0x%X] [%s]",channel_id[i],strVal.str().c_str(),period,period,std::bitset<30>(period).to_string().c_str());
            }
          }

          reg = reg_init + i;
          if (!enable.compare("true")) {
            register_map_[reg].value() = (0x1 << 31);
            register_map_[reg].value() |= period & 0x7FFFFFFF;
          } else if (!enable.compare("false")) {
            register_map_[reg].value() = (0x0 << 31);
            register_map_[reg].value() |= period & 0x7FFFFFFF;
          } else {
            Log(error,"Unknown status of enable flag : [%s]. Possible values : (true|false)",enable.c_str());
            msgs_ << "<error>Unknown status of enable flag : ["<< enable << "]. Possible values : (true|false)</error>";
            has_error = true;
          }
          Log(debug,"Register %u {31-0} [0x%08X]", register_map_[reg].value() );

        }
      } // -- strcmp calibrations

      // Log(verbose," Content val : %s",it->value());
      // Log(verbose," Content child : %s",it->child_value() );
    } // for
  }
  // -- Only catch something deriving from std::exception and unspecified
  catch(std::exception &e) {
    msgs_ << "<error>Error processing configuration: " << e.what() << ". Not committing configuration to PTB.</error>";
    has_error = true;
  }
  catch(...) {
    msgs_ << "<error>Unknown error processing configuration. Not committing configuration to PTB.</error>";
    has_error = true;
  }
  // Check if we got an error. If so, do not commit.
  if (has_error) {
    // Don't commit. Just go back and throw the error.
    msgs_str_ = msgs_.str();
    answers = new char[msgs_str_.size()+1];
    sprintf(answers,"%s",msgs_str_.c_str());
    return;
  }

  dump_config_registers();

  // Set the bit to commit the configuration
  // into the hardware (bit 29 in register 30)
  Log(debug,"Committing configuration to the hardware.");
  Log(verbose,"Control register before config commit 0x%X", register_map_[0].value() );
  set_config_bit(true);
  Log(debug,"Control register after config commit 0x%08X", register_map_[0].value() );
  if (!get_config_bit_ACK()) {
    // If the commit is not ACK then effectively the board is not configured.
    // The best is to reset everything and fail the configuration.
    msgs_ << "<error>Failed set to set the configuration bit. ACK not received. Control register : "
        << std::hex << register_map_.at(0).value() << std::dec
        << ". Control register after reset : ";

    set_enable_bit(false);
    set_config_bit(false);
    // the hard reset also includes a complete reset of the local buffers
    set_reset_bit(true);
    // Sleep for 10 microseconds to make sure that reset has taken place
    std::this_thread::sleep_for (std::chrono::microseconds(10));
    set_reset_bit(false);
    zero_config_registers();
    set_config_bit(false);
    msgs_ << std::hex << register_map_.at(0).value() << std::dec;
    msgs_ << "</error>";
    Log(error,"Failed set to set the configuration bit. ACK not received. Resetting back.");
    has_error = true;
  }

  if (!has_error) {
    Log(debug,"Registering the committed configuration to the local cache.");
    // Store the cache in the mirror map
    register_cache_.at(0).value() = CTL_BASE_REG_VAL;
    for (uint32_t i = 1; i < num_registers_; ++i) {
      register_cache_.at(i).value() = register_map_.at(i).value();
    }

    // Check for the ACK of the configuration
    //  if ((register_map_[30].value() >> 28 & 0x1) != 0x1) {
    //    Log(error,"Configuration failed to commit. ACK not received.");
    //    throw("Configuration failed to commit. ACK not received.");
    //  }


    // After parsing everything (and making sure that all the configuration is set)
    // Store the configuration locally
    config_ = config;
    // Log(debug,"Sleeping for 1s prior to init the connection to DAQ upstream.");
    // // Tell the reader to start the connection
    // std::this_thread::sleep_for (std::chrono::seconds(1));

    // Log(verbose,"Initializing connection to DAQ upstream." );
    // //Log(verbose,"Host : " << host << " port " << tcp_port_ << endl;
    // try {
    //   reader_->InitConnection(true);
    // }
    // catch(SocketException &e) {
    //   msgs_ << "<warning>Failed to open connection to board reader.</warning>";
    //   Log(warning,"Connection failed to establish. This might cause troubles later.");
    // }
    msgs_ << "<success>true</success>";
    msgs_str_ = msgs_.str();
    answers = new char[msgs_str_.size()+1];
    sprintf(answers,"%s",msgs_str_.c_str());

  } else {
    // Don't commit. Just go back and throw the error.
    msgs_str_ = msgs_.str();
    answers = new char[msgs_str_.size()+1];
    sprintf(answers,"%s",msgs_str_.c_str());
  }
  // Most likely the connection will fail at this point. Not a big problem.
  Log(verbose,"Returning from SetConfig with answer [%s]",answers);
}

}

