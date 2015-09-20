/*
 * PTBManager.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "PTBManager.h"
#include "PTBReader.h"
#include "Logger.h"
#include "ConfigServer.h"
#ifdef ARM
#include "ptb_registers.h"
#endif /*ARM*/

// -- PugiXML includes
#include "pugixml.hpp"
#include <iomanip>
#include <thread>
#include <chrono>

extern "C" {
#include <unistd.h>
};


const char* PTBManager::default_config_ = "./config/config_default.xml";

// Init with a new reader attached
PTBManager::PTBManager(bool emu_mode) : reader_(0), cfg_srv_(0),status_(IDLE),emu_mode_(emu_mode) {
  reader_ =	new PTBReader(emu_mode);
  Log(debug,"Setting up pointers." );

  // Register the available commands
  commands_.insert(std::map<std::string, Command>::value_type("StartRun",STARTRUN));
  commands_.insert(std::map<std::string, Command>::value_type("StopRun",STOPRUN));
  commands_.insert(std::map<std::string, Command>::value_type("SoftReset",SOFTRESET));
  commands_.insert(std::map<std::string, Command>::value_type("HardReset",HARDRESET));

  // Setup the registers first, to make sure that if the data manager receives something
  // we are ready to deliver.
  SetupRegisters();

  // In fact, load also a default configuration as well.
  // Won't work...client is not ready
  //LoadDefaultConfig();

  // This had to occur after setting up the command list or we might run into trouble.
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cfg_srv_->RegisterDataManager(this);

}



PTBManager::~PTBManager() {
  Log(debug,"Destroying object." );
  //FIXME: Implement memory deallocations to free the microZed.
  FreeRegisters();
  // Clear the map
  commands_.clear();
  Log(debug,"Killing the reader");
  delete reader_;
  cfg_srv_ = NULL;
  reader_ = NULL;
  Log(debug,"Reader destroyed");
}


void PTBManager::ExecuteCommand(const char* cmd) {
  try{

    Log(debug,"Received [%s]", cmd);
    std::map<std::string, Command>::iterator it;
    for (it = commands_.begin(); it != commands_.end(); ++it) {
      Log(debug,"Key : [%s] : Value [%u]",it->first.c_str(),it->second );
      if (it->first == cmd) {
        Log(verbose,"It is the same" );
      }
    }

    Log(verbose,"Going into the switch to try %d",commands_.at(cmd) );
    switch (commands_.at(cmd)) {
      // -- Send the signal to start the run. Should be a register.
      case STARTRUN:
        // Only starts a run if the manager is in IDLE state
        // Otherwise issue a warning
        if (getStatus() != PTBManager::IDLE) {
          Log(warning,"A run is already running. Ignoring command" );
        } else {
          // Start the run
          Log(verbose,"The Run should START now" );
          StartRun();
          break;
        }
        break;
        // -- Send reset. In both cases there is a hardware reset sent
        // The soft reset is just a reset of the hardware. And let the SW remain as it is.
        // Essencially it clears the hardware, recommits the configuration and restart the run
        //
      case SOFTRESET:
        SetResetBit(true);
        // Sleep for 10 microseconds to make sure that reset has taken place
        std::this_thread::sleep_for (std::chrono::microseconds(10));
        //SetResetBit(false);
        // -- Re-commit the configuration
        RestoreConfigurationRegisters();
        SetConfigBit(true);
        SetEnableBit(true);
        break;
      case HARDRESET:
        SetEnableBit(false);
        SetConfigBit(false);
        // the hard reset also includes a complete reset of the local buffers
        SetResetBit(true);
        // Sleep for 10 microseconds to make sure that reset has taken place
        std::this_thread::sleep_for (std::chrono::microseconds(10));
        //SetResetBit(false);
        reader_->ResetBuffers();
        RestoreConfigurationRegisters();
        // Reset the configuration buggers to an enable state
        // Reassign the configuration (was kept in memory before)
        SetConfigBit(true);
        SetEnableBit(true);
        break;
      case STOPRUN:
        if (getStatus() != PTBManager::RUNNING) {
          Log(warning,"Called for STOPRUN but there is no run ongoing. Ignoring." );
          // -- Same thing.
          break;
        } else {
          Log(verbose,"The Run should STOP now" );
          StopRun();
          break;
        }
      default:
        throw std::string("Unkown command"); // command_exception("Unkown command");
    }
  }
  catch (const std::out_of_range &oor) {
    Log(error,"Out of range error: %s",oor.what() );
    throw std::string("Unkown command");
  }
  catch(std::exception &e) {
    Log(error,"STL exception caught : %s",e.what() );
    throw;
  }
  catch(std::string &str) {
    Log(error,"Caught string exception. %s",str.c_str() );
    Log(verbose,"Rethrowing" );
    throw;
  }
  catch(...) {
    Log(error,"Unknown exception caught." );
    throw std::string("Unkown command");
  }
}

void PTBManager::StartRun() {
  Log(verbose,"Starting the run" );

  // Order of execution:
  // Tell the PTBReader to start taking data (it won't receive anything but it will be ready)
  // Set the GLB_EN register to 1 (start readout in the fabric)
  // Set status flag to RUNNING
  if (!reader_) reader_ = new PTBReader(emu_mode_);

  if (!reader_->isReady()) {
    reader_->InitConnection();
  }

  if (!reader_->isReady()) {
    Log(warning,"Received call to start transmitting but reader is not ready. Refusing to run." );
    throw std::string("Data reader not ready...");
    return;
  }
  reader_->StartDataTaking();

  // The GLB_EN is located in bin 31 of register 30
  //*(volatile uint32_t*)(register_map_[30].address) |= (0x1 << 31);
  //register_map_[30].value() |= (0x1 << 31);
  SetEnableBit(true);

  Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[0].value() );

  // Check back if the ack was set
  // if (register_map_[30].value() >> 30  & 0x1 ) {
  if (GetEnableBitACK() || emu_mode_) {
    status_ = RUNNING;
    Log(info,"Run Start ACK received.");
  } else {
    status_ = IDLE;
    Log(error,"Failed to start run. ACK not received.");
    throw std::string("Start Run failed. No ACK received.");
  }
}

void PTBManager::StopRun() {
  Log(debug,"Stopping the run" );
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to stop the run. This should include:
  //        Propagate into the PTBReader to stop data collection
  //        Deallocate its streaming connection.

  // First check that it is running
  if ((status_ != RUNNING) || GetEnableBitACK() == false ) {//(((register_map_[30].value() >> 30) & 0x1) != 0x1 )) {
    Log(warning,"Stop Run requested, but device isn't running. Ignoring.");
    return;
  }

  // The order should be:
  // 1. Set the GLB_EN register to 0 (stop readout in the fabric)
  // 2. Set the status to IDLE
  // 3. Tell the PTB reader to stop streaming


  // The GLB_EN is located in bin 31 of register 30
  //  *(volatile uint32_t*)(register_map_[34].address) |= (0x0 << 31);
  //  register_map_[30].value() |= (0x0 << 31);
  SetEnableBit(false);
  Log(debug,"GLB_EN unset. Register: 0x%08x ",register_map_[0].value() );

  // Check the ACK bit
  //  if (((register_map_[30].value() >> 30 ) & 0x1)) {
  if (GetEnableBitACK() != false && !emu_mode_) {
    Log(warning,"Stop Run failed. ACK bit still high.");
    throw std::string("Stop Run failed. No ACK received.");
  }

  reader_->StopDataTaking();

  status_ = IDLE;
}

void PTBManager::SetupRegisters() {
  Log(info,"Setting up the appropriate registers to configure the board." );
  // Are we in emulator mode?

  struct LocalRegister tmp_reg;
  tmp_reg.address =NULL;
  //tmp_reg.value() = 0;
  // First set up the local map
  for (uint32_t i =0; i < num_registers_; ++i) {
    Log(verbose,"Cleaning register %u.",i);
    register_map_[i] = tmp_reg;
    register_cache_[i] = tmp_reg;
  }
  Log(debug,"Registers set up.");


  // Map the memory for each of the addresses.
  // This is different if we are emulating (use malloc), or
  // Using memory mapped registers (specific locations)

  if (emu_mode_) {
    // Most of the configuration options are passed through GPIO
    // This means everything is configured as 32bit registers

    // Allocate the memory for the registers

    for (uint32_t i =0; i < num_registers_; ++i) {
      //register_map_[i].address = malloc(sizeof(uint32_t));
      register_map_[i].address = new uint32_t();
    }
  } else {
    Log(warning,"Memory mapped registers are not yet tested. This might, or might not, fail." );
#ifdef ARM
    SetupConfRegisters();
    // First get the virtual address for the mapped physical address
    mapped_base_addr_ = MapPhysMemory(conf_reg.base_addr,conf_reg.high_addr);
    Log(debug,"Received virtual address for configuration : 0x%08X\n",reinterpret_cast<uint32_t>(mapped_base_addr_));
    // Cross check that we have at least as many offsets as registers expected
    if (conf_reg.n_registers < num_registers_) {
      Log(warning,"Have less configured registers than the ones required. (%u != %u)",conf_reg.n_registers,num_registers_);
    }
    for (uint32_t i = 0; i < num_registers_; ++i) {
      register_map_[i].address =  reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_base_addr_) + conf_reg.addr_offset[i]);
      register_map_[i].value() = 0;
    }
#endif /*ARM*/
  }
  // Allocate the memory for the register cache
  for (uint32_t i =0; i < num_registers_; ++i) {
    register_cache_[i].address = new uint32_t();
  }

}

void PTBManager::FreeRegisters() {
  Log(info,"Cleaning up the allocated registers to configure the board." );
  if (emu_mode_) {
    // Simply deallocate the memory of each register
    for (uint32_t i =0; i < num_registers_; ++i) {
      delete static_cast<uint32_t*>(register_map_[i].address);
      //      free(register_map_[i].address);
      //      register_map_[i].address = NULL;
    }
    register_map_.clear();

  } else {
    Log(warning,"Still haven't tested the unmap. Might or might not fail miserably..." );
#ifdef ARM
    munmap(mapped_base_addr_,conf_reg.high_addr-conf_reg.base_addr);
#endif /*ARM*/
  }

  // Clear and free also the cache registers
  for (uint32_t i =0; i < num_registers_; ++i) {
    delete static_cast<uint32_t*>(register_cache_[i].address);
  }
  register_cache_.clear();

}

void PTBManager::ProcessConfig(pugi::xml_node config) {
  // Only accept a new configuration if we are not running.
  if (status_ == RUNNING) {
    Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
    return;
  }

  Log(info,"Applying a reset prior to the configuration.");
  SetResetBit(true);
  std::this_thread::sleep_for (std::chrono::microseconds(10));
//      SetResetBit(false);
  Log(info,"Reset applied");

  // // Check if the reader is ready. If it is ignore the change
  // if (reader_->isReady()) {
  //   Log(warning,"FIXME: Reader is ready. Do not overwrite config.");
  //   return;
  // }

  // This is the workhorse of the configuration.
  // At this point the registers should already be mapped and things should be flowing

  Log(info,"Parsing the configuration." );

  // First zero out all registers


  // Load the childs by parts.
  // Could use a iterator but then
  ///!
  ///! DataBuffer
  ///!
  char *pEnd = NULL;

  for (pugi::xml_node_iterator it = config.begin(); it != config.end(); ++it) {
    Log(verbose," Child name : %s",it->name());
    // The reader should take care of this by itself.
    if (!strcmp(it->name(),"DataBuffer")) {
      if (!reader_) {
        Log(warning,"Don't have a valid PTBReader instance. Attempting to create a new one." );
        reader_ = new PTBReader(emu_mode_);
      }
      //-- Get the host
      std::string host = it->child("DaqHost").child_value();
      reader_->setTcpHost(host);
      //uint32_t port = strtoul(it->child("DaqPort").child_value(),&pEnd,10);
      unsigned short port = atoi(it->child("DaqPort").child_value());
      Log(debug,"Received port %hu",port );
      reader_->setTcpPort(port);
      //uint32_t rollOver = strtoul(it->child("RollOver").child_value(),&pEnd,10);
      uint32_t rollOver = atoi(it->child("RollOver").child_value());
      Log(debug,"Packet Rollover %u",rollOver );
      reader_->setPacketRollover(rollOver);
      // uint32_t duration = strtoul(it->child("MicroSliceDuration").child_value(),&pEnd,10);
      uint32_t duration = atoi(it->child("MicroSliceDuration").child_value());
      Log(debug,"MicroSlice Duration %u",duration );
      if (duration > 28) {
        Log(warning,"Input value of [%u] above maximum rollover [28]. Truncating to maximum.",duration);
        duration = 28;
      }
      uint64_t timeRollOver = (1 << duration);
      Log(debug,"MicroSlice time rollover : [%u] --> %lu clock ticks.",duration,timeRollOver);
      reader_->setTimeRollover(timeRollOver);

      Log(verbose,"Setting data transmission channel to [%s:%hu]",host.c_str(),port);
      //Log(verbose,"Packet rollover set to %u ",rollOver );
    }
    if (!strcmp(it->name(),"ChannelMask")) {
      // Deal directly with the childs.

      uint64_t bsu = strtoull(it->child("BSU").child_value(),&pEnd,16);
      // Now I want to get the first 32 bits into a register
      Log(verbose,"Started with 0x%lX",bsu );
      // Make the assignments that I had decided before:
      // Catch the lowest 32 bits of the word
      ////// ------
      ////// REG 34 (LSB OF CHANNEL MASK)
      ////// ------

      *(volatile uint32_t*)(register_map_[34].address) = bsu & 0xFFFFFFFF;
      //register_map_[0].value() = bsu & 0xFFFFFFFF;
      Log(verbose,"First register 0x%X", register_map_[0].value() );

      ////// ------
      ////// REG 1
      ////// ------
      // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
      // [0-16]
      *(volatile uint32_t*)(register_map_[1].address) = (bsu >> 32 ) & 0x1FFFF;
      //register_map_[1].value() = (bsu >> 32 ) & 0x1FFFF;
      Log(verbose,"Second register (tmp) 0x%X",register_map_[1].value() );

      // Now grab the TSU part to complete the mask of this register
      uint64_t tsu = strtoull(it->child("TSU").child_value(),&pEnd,16);

      // The lowest 15 bits go into the upper bits of the previous register
      // [17-31]
      *(volatile uint32_t*)(register_map_[1].address) |= ((tsu & 0x7FFF) << 17);
      //register_map_[1].value() |= ((tsu & 0x7FFF) << 17);
      Log(verbose,"Second register 0x%X", register_map_[1].value() );

      ////// ------
      ////// REG 2
      ////// ------

      // The remaining 33 (48-15) bits go into the next registers
      *(volatile uint32_t*)(register_map_[2].address) = ((tsu >> 15) & 0xFFFFFFFF);
      //register_map_[2].value() = ((tsu >> 15) & 0xFFFFFFFF);
      Log(verbose,"Third register 0x%X",*(volatile uint32_t*)(register_map_[2].address) );

      ////// ------
      ////// REG 3
      ////// ------

      // The final bit goes into the lsb of register 4
      *(volatile uint32_t*)(register_map_[3].address) |= ((tsu >> 47) & 0x1);
      Log(verbose,"Fourth register 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
      //      register_map_[3].value() |= ((tsu >> 47) & 0x1);
      //      Log(verbose,"Fourth register 0x%X",register_map_[3].value() );
    }
    if (!strcmp(it->name(),"ExtTriggers")) {
      // External triggers go into register 4
      // to bits [26-30]
      uint32_t TRIGEX = strtoul(it->child("Mask").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((TRIGEX & 0x1F) << 26);
      Log(verbose,"Fourth register (TRIGEX) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
      //      register_map_[3].value() |= ((TRIGEX & 0x1F) << 26);
      //      Log(verbose,"Fourth register (TRIGEX) 0x%X",register_map_[3].value() );
    }
    // -- The calibrations now have their own registers
    //    if (!strcmp(it->name(),"Calibration")) {
    //      // Calibrations go also into register 4
    //      // Period : [6-21]
    //      // Mask : [22-25]
    //      uint32_t CAL_PRD = strtoul(it->child("Period").child_value(),&pEnd,16);
    //      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_PRD & 0xFFFF) << 6);
    //      //register_map_[3].value() |= ((CAL_PRD & 0xFFFF) << 6);
    //      Log(verbose,"Fourth register (CAL_PRD) 0x%X",*(volatile uint32_t*)(register_map_[3].address));
    //
    //
    //      uint32_t CAL_MSK = strtoul(it->child("ChannelMask").child_value(),&pEnd,16);
    //      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_MSK & 0xF) << 22);
    //      //register_map_[3].value() |= ((CAL_MSK & 0xF) << 22);
    //      Log(verbose,"Fourth register (CAL_MSK) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
    //
    //    }

    if (!strcmp(it->name(),"Hardware")) {
      // M_PULSEWIDTH - Number of clock cycles to keep the any outgoing pulse (out high)
      // Width of the outgoing pulses
      // TrigOutWidth : [25-27]
      uint32_t M_PULSEWIDTH = strtoul(it->child("PulseWidth").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_PULSEWIDTH & 0x7) << 25);
      Log(verbose,"Fourth register (M_PULSEWIDTH) 0x%X", *(volatile uint32_t*)(register_map_[4].address) );
      //      register_map_[4].value() |= ((M_TRIGOUT & 0x7) << 25);
      //      Log(verbose,"Fourth register (M_PULSEWIDTH) 0x%X", register_map_[4].value() );

    }
    // The most troublesome part: muon triggers
    if (!strcmp(it->name(),"MuonTriggers")) {
      // This one is quite complicated to parse.
      // For the moment let's deal with it statically...

      // The lockout window goes into register 4
      // LockoutWindow : [1-5]
      uint32_t MT_LOCKOUT = strtoul(it->child("LockdownWindow").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((MT_LOCKOUT & 0x1F) << 1);
      Log(verbose,"Fourth register (MT_LOCKDOWN) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
      //      register_map_[3].value() |= ((MT_LOCKDOWN & 0x1F) << 1);
      //      Log(verbose,"Fourth register (MT_LOCKDOWN) 0x%X",register_map_[3].value() );

      // The remaining configuration words go into register 5

      // M_LOCKHI - Number of clock cycles to extend the input signal
      // Width of the trigger window
      // TriggerWindow : [22-24]
      uint32_t M_LOCKHI = strtoul(it->child("TriggerWindow").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_LOCKHI & 0x7) << 22);
      Log(verbose,"Fourth register (M_LOCKHI) 0x%X", *(volatile uint32_t*)(register_map_[4].address) );
      //      register_map_[4].value() |= ((M_LOCKHI & 0x7) << 22);
      //      Log(verbose,"Fourth register (M_LOCKHI) 0x%X", register_map_[4].value() );

      // Now it is specific trigger codes...
      // Deal with one at a time

      pugi::xml_node mtrigger_node = it->find_child_by_attribute("TriggerMask","id","A");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger A.");
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,4,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","B");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger B." );
        continue;
      } else {
        // -- The operations go into register 6
        ParseMuonTrigger(mtrigger_node,5,7);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","C");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger C." );
        continue;
      } else {
        // -- The operations go into register 18
        ParseMuonTrigger(mtrigger_node,17,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","D");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger D." );
        continue;
      } else {
        // -- The operations go into register 19
        ParseMuonTrigger(mtrigger_node,17,7);
      }

    }

    //-- Now grab the calibration masks
    if (!strcmp(it->name(),"Calibrations")) {
      std::string enable;
      uint32_t period  = 0;
      uint32_t reg = 30;
      pugi::xml_node node = it->find_child_by_attribute("CalibrationMask","id","C1");
      if (node.empty()){
        Log(warning,"Couldn't find the configuration for calibration channel C1.");
        continue;
      } else {
        enable = node.child("enable").child_value();
        period = strtoul(node.child("period").child_value(),&pEnd,16);
        reg = 30;
        if (period > 0x7FFFFFFF) {
          Log(warning,"Period larger than maximum allowable size. Truncating to max value ( 2147483647 : 0x7FFFFFFF)");
          period = 0x7FFFFFFF;
        }
        if (!enable.compare("true")) {
          register_map_[reg].value() = (0x1 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        } else {
          register_map_[reg].value() = (0x0 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        }
      }

      node = it->find_child_by_attribute("CalibrationMask","id","C2");
      if (node.empty()){
        Log(warning,"Couldn't find the configuration for calibration channel C2.");
        continue;
      } else {
        enable = node.child("enable").child_value();
        period = strtoul(node.child("period").child_value(),&pEnd,16);
        reg = 31;
        if (period > 0x7FFFFFFF) {
          Log(warning,"Period larger than maximum allowable size. Truncating to max value ( 2147483647 : 0x7FFFFFFF)");
          period = 0x7FFFFFFF;
        }
        if (!enable.compare("true")) {
          register_map_[reg].value() = (0x1 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        } else {
          register_map_[reg].value() = (0x0 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        }
      }

      node = it->find_child_by_attribute("CalibrationMask","id","C3");
      if (node.empty()){
        Log(warning,"Couldn't find the configuration for calibration channel C3.");
        continue;
      } else {
        enable = node.child("enable").child_value();
        period = strtoul(node.child("period").child_value(),&pEnd,16);
        reg = 32;
        if (period > 0x7FFFFFFF) {
          Log(warning,"Period larger than maximum allowable size. Truncating to max value ( 2147483647 : 0x7FFFFFFF)");
          period = 0x7FFFFFFF;
        }
        if (!enable.compare("true")) {
          register_map_[reg].value() = (0x1 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        } else {
          register_map_[reg].value() = (0x0 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        }
      }

      node = it->find_child_by_attribute("CalibrationMask","id","C4");
      if (node.empty()){
        Log(warning,"Couldn't find the configuration for calibration channel C4.");
        continue;
      } else {
        enable = node.child("enable").child_value();
        period = strtoul(node.child("period").child_value(),&pEnd,16);
        reg = 33;
        if (period > 0x7FFFFFFF) {
          Log(warning,"Period larger than maximum allowable size. Truncating to max value ( 2147483647 : 0x7FFFFFFF)");
          period = 0x7FFFFFFF;
        }
        if (!enable.compare("true")) {
          register_map_[reg].value() = (0x1 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        } else {
          register_map_[reg].value() = (0x0 << 31);
          register_map_[reg].value() |= period & 0x7FFFFFFF;
        }
      }

    } // -- strcmp calibrations

    Log(verbose," Content val : %s",it->value());
    Log(verbose," Content child : %s",it->child_value() );
  } // for

  //}

  DumpConfigurationRegisters();

  // Set the bit to commit the configuration
  // into the hardware (bit 29 in register 30)
  Log(debug,"Committing configuration to the hardware.");
  //  register_map_[34].value() |= (0x1 << 29);
  Log(verbose,"Control register before config commit 0x%X", register_map_[0].value() );
  SetConfigBit(true);
  Log(debug,"Control register after config commit 0x%X", register_map_[0].value() );
  if (!GetConfigBitACK() || emu_mode_) {
    Log(error,"Failed set to set the configuration bit. ACK not received");
    throw("Configuration failed to commit. ACK not received.");
  }

  Log(debug,"Registered committed configuration to the local cache.");
  // Store the cache in the mirror map
  // Register cache 0 is set to 0 (unsurprisingly)
  register_cache_.at(0).value() = 0;
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
  Log(debug,"Sleeping for 5s prior to init the connection to DAQ upstream.");



  // Tell the reader to start the connection
  std::this_thread::sleep_for (std::chrono::seconds(5));
  //sleep(15);
  Log(verbose,"Initializing connection to DAQ upstream." );
  //Log(verbose,"Host : " << host << " port " << tcp_port_ << endl;
  reader_->InitConnection();

}

void PTBManager::DumpConfigurationRegisters() {
  Log(info,"Dumping configuration registers.");
  Log(info,"===========================================================");
  for (size_t i = 0; i < register_map_.size(); ++i) {
    Log(info,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
  }
  Log(info,"===========================================================");

}

void PTBManager::ResetConfigurationRegisters() {
  Log(debug,"Resetting configuration registers");
  for (size_t i = 0; i < register_map_.size(); ++i) {
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    register_map_.at(i).value() = 0x0;
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
  }

}

void PTBManager::RestoreConfigurationRegisters() {
  Log(debug,"Restoring configuration registers to local cache");
  for (size_t i = 0; i < register_map_.size(); ++i) {
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    register_map_.at(i).value() = register_cache_.at(i).value();
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
  }

}



void PTBManager::ParseMuonTrigger(pugi::xml_node T, uint32_t reg, uint32_t reg_offset) {

  Log(verbose,"Processing muon trigger with conf reg %u and offset %u",reg,reg_offset );

  // First thing to do is identify the offset in the bins of the configuration register
  uint32_t word_offset = (reg_offset>1)?0:11;
  char *pEnd;
  // == First deal with the global parameters:
  // variables that are used repeatedly below
  uint32_t mask;
  uint32_t bit_offset;
  std::string field_name;
  uint32_t input_word;
  uint64_t input_longword;

  // M_?_EXT_OP - External logic to be applied between the groups
  // ExtLogic : [word_offset+5 - word_offset+6]
  mask = 0x3;
  bit_offset = 5;
  field_name = "ExtLogic";

  // For trigger A this translates into 
  // word_offset = 11
  // bit_offset = 5
  // (input_word & 0x3) << 16 [16-17]
  input_word = strtoul(T.child(field_name.c_str()).child_value(),&pEnd,16);
  *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );
  //  register_map_[reg].value() |= ((input_word & mask) << (word_offset+bit_offset));
  //  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),register_map_[reg].value() );

  // M_?_PRESC - Prescale of a given trigger
  // Prescale : [word_offset+2 - word_offset+4]
  mask = 0x7;
  bit_offset = 2;
  field_name = "Prescale";

  input_word = strtoul(T.child(field_name.c_str()).child_value(),&pEnd,16);
  // For trigger A this translates into 
  // input_word & 0x7 << 13 ==> [13-15]
  *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );
  //  register_map_[reg].value() |= ((input_word & mask) << (word_offset+bit_offset));
  //  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),register_map_[reg].value() );


  // Loop over the groups
  const char *groups[] = {"group1","group2"};

  for (uint32_t i =0; i < 2; ++i) {
    pugi::xml_node G = T.child(groups[i]);
    // Now the more messy part of using a long register
    Log(verbose,"Processing group [%s]",groups[i]);
    // Intra-group logic : [word_offset+7 - word_offset+8]
    // M_?_G[i]_OP
    mask = 0x3;
    bit_offset = 7+(i*2);
    field_name = "Logic";
    input_word = strtoul(G.child(field_name.c_str()).child_value(),&pEnd,16);
    // Trigger A:
    // i = 0 : [18-19]
    //    input_word & 0x3 << 18 ==> [18-19]
    // i = 1 : [20-21]
    //    input_word & 0x3 << 20 ==> [20-21]
    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
    Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );
    //    register_map_[reg].value() |= ((input_word & mask) << (word_offset+bit_offset));
    //    Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),register_map_[reg].value() );

    // Now get the mask into a 64 bit word
    input_longword = strtoull(G.child("BSU").child_value(),&pEnd,16);
    // These now go into the offset registers

    // -- First contains the first 32 bits of the word
    mask = 0xFFFFFFFF;
    // -- Trigger A:
    // i = 0: [Reg5 : [0-31]]
    //       map[5] = input_longword & 0xFFFFFFFF;
    // i = 1: [Reg8] : [0-31]
    //       map[8] : input_longword & 0xFFFFFFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) = (input_longword & mask);
    Log(debug,"Mask register %u (W1) %X",reg+reg_offset+(i*3),*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) );
    //    register_map_[reg+reg_offset+(i*3)].value() = (input_longword & mask);
    //    Log(debug,"Mask register %u (W1) %X",reg+reg_offset+(i*3),register_map_[reg+reg_offset+(i*3)].value() );

    // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
    // [0-16]
    mask = 0x1FFFF;


    // -- Trigger A:
    // i = 0: [Reg6 : [0-16]]
    //       map[6] = input_longword & 0x1FFFF;
    // i = 1: [Reg9] : [0-16]
    //       map[9] : input_longword & 0x1FFFF;
    // Reset the bits to zeros to avoid ghost bits
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) = 0x0;
    //register_map_[reg+reg_offset+(i*3)+1].value() = 0x0;

    // Now do the assignments
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) = (input_longword >> 32 ) & mask;
    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );
    //    register_map_[reg+reg_offset+(i*3)+1].value() = (input_longword >> 32 ) & mask;
    //    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,register_map_[reg+reg_offset+(i*3)+1].value() );



    // Now grab the TSU part to complete the mask of this register
    input_longword = strtoull(G.child("TSU").child_value(),&pEnd,16);

    // The lowest 15 bits (0-14) go into the upper bits of the previous register
    // [17-31]
    mask = 0x7FFF;
    bit_offset = 17;
    // -- Trigger A:
    // i = 0: [Reg6 : [17-31]]
    //       map[6] = input_longword & 0x1FFFF;
    // i = 1: [Reg9] : [17-31]
    //       map[9] : input_longword & 0x1FFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) |= (input_longword & mask ) << bit_offset;
    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );
    //    register_map_[reg+reg_offset+(i*3)+1].value() |= (input_longword & mask ) << bit_offset;
    //    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,register_map_[reg+reg_offset+(i*3)+1].value() );

    // The next 32 (47-15) bits go into the next register
    mask = 0xFFFFFFFF;
    bit_offset = 15;
    // -- Trigger A:
    // i = 0: [Reg7 : [0-31]]
    //       map[7] = input_longword & 0xFFFFFFFF;
    // i = 1: [Reg10] : [0-31]
    //       map[10] : input_longword & 0xFFFFFFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) = (input_longword >> bit_offset) & mask;
    Log(debug,"Mask register %u (W3) %X",reg+reg_offset+(i*3)+2,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) );
    //    register_map_[reg+reg_offset+(i*3)+2].value() = (input_longword >> bit_offset) & mask;
    //    Log(debug,"Mask register %u (W3) %X",reg+reg_offset+(i*3)+2,register_map_[reg+reg_offset+(i*3)+2].value());


    // The final bit (48) goes into the lsb of the configuration register
    mask = 0x1;
    bit_offset = i;
    // -- Trigger A:
    // i = 0: [Reg4 : [11]]
    //       map[4] = input_longword >> 47 & 0x1 << 11;
    // i = 1: [Reg4] : [12]
    //       map[10] : input_longword >> 47 & 0x1 << 12;

    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_longword >> 47) && mask) << bit_offset;
    Log(debug,"Mask register %u (MASK) %X",reg,*(volatile uint32_t*)(register_map_[reg].address) );
    //    register_map_[reg].value() |= ((input_longword >> 47) && mask) << bit_offset;
    //    Log(debug,"Mask register %u (MASK) %X",reg,register_map_[reg].value() );
  }

}


/// -- Short hand methods to be used on the higher level above

bool PTBManager::GetBit(uint32_t reg, uint32_t bit) {
  return (((register_map_[reg].value() >> bit) & 0x1) == 0x1)?true:false;
}
void PTBManager::SetBit(uint32_t reg, uint32_t bit, bool status) {
  // Things are more complicated than this. We want to set a single bit, regardless of what is around
  //register_map_[reg].value() ^= ((status?0x1:0x0) ^ register_map_[reg].value()) & ( 1 << bit);
	Log(debug,"Reading the bit %u from reg %u",bit,reg);
	uint32_t value = Xil_In32((uint32_t)register_map_[reg].address);
	uint32_t new_value = (value^(-(status?1:0) ^ value) & ( 1 << bit));

	Log(debug,"Got %08X -> %08X",value,new_value);
	Xil_Out32((uint32_t)register_map_[reg].address,new_value);
	Log(debug,"Final register value %08X",register_map_[reg].value());
  //  uint32_t tmp_reg = register_map_[reg].value();
//  (tmp_reg >> bit) = status?0x1:0x0;
//  register_map_[reg].value() = tmp_reg;
}
//

void PTBManager::SetEnableBit(bool status) {
  SetBit(0,31,status);
}

void PTBManager::SetResetBit(bool status) {
  // here true defines the reset enable...the firmware inverts this
  SetBit(0,27,status);
}

void PTBManager::SetConfigBit(bool status) {
  SetBit(0,29,status);
}

bool PTBManager::GetConfigBitACK() {
  return GetBit(0,28);
}
bool PTBManager::GetEnableBitACK() {
  return GetBit(0,30);
}

/// -- Other old and outdated methods.

// -- Outdated. This should no longer be used.
void PTBManager::LoadDefaultConfig() {
  // File opened alright. Proceed with the parsing
  // Instanciate the XML plugin
  try {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(default_config_);
    Log(verbose,"Load result : %s",result.description() );
    pugi::xml_node config = doc.child("config");
    if (config == NULL) {
      Log(error,"Failed to load the config block from input file [%s]",default_config_);
    } else {
      ProcessConfig(config);
    }
  }
  catch (pugi::xpath_exception &e) {
    Log(error,"PUGIXML exception caught: %s",e.what() );
    return;
  }
  catch (std::exception &e) {
    Log(error,"STD exception caught: %s",e.what() );
    return;
  }
  catch (...) {
    Log(error,"Unknown exception caught. This is going to mean trouble." );
    return;
  }
}

