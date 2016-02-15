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
#include "PTBexception.h"
#include "PracticalSocket.h"
#include "util.h"

#if defined(ARM_XDMA) || defined(ARM_MMAP)
#include "ptb_registers.h"
#endif /*ARM*/

// -- PugiXML includes
#include "pugixml.hpp"
#include <iomanip>
#include <thread>
#include <chrono>
#include <bitset>
#include <cmath>


#define __STDC_FORMAT_MACROS

extern "C" {
#include <unistd.h>
#include <inttypes.h>
}

/**
 * This seems to suggest that the control register is always set to 1.
 * But the config_manager IP inverts the signal so it should be set to 0 here and the
 * IP takes care of inverting it
 */

#define CTL_BASE_REG_VAL 0x00000000

mapped_register conf_reg;

// Init with a new reader attached
PTBManager::PTBManager( ) : reader_(0), cfg_srv_(0),status_(IDLE),
    mapped_conf_base_addr_(nullptr),msgs_str_(""),error_state_(false),
    data_socket_host_("192.168.100.100"),data_socket_port_(8992) {
  reader_ =	new PTBReader();
  Log(debug,"Setting up pointers." );

  // Register the available commands
  commands_.insert(std::map<std::string, Command>::value_type("StartRun",STARTRUN));
  commands_.insert(std::map<std::string, Command>::value_type("StopRun",STOPRUN));
  commands_.insert(std::map<std::string, Command>::value_type("SoftReset",SOFTRESET));
  commands_.insert(std::map<std::string, Command>::value_type("HardReset",HARDRESET));
  // Setup the registers first, to make sure that if the data 
  // manager receives something we are ready to deliver.
  SetupRegisters();

  // This had to occur after setting up the command list
  //  or we might run into trouble.
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cfg_srv_->RegisterDataManager(this);

}

PTBManager::~PTBManager() {
  Log(debug,"Destroying manager..." );
  FreeRegisters();
  // Clear the map
  commands_.clear();
  Log(debug,"Killing the reader");
  delete reader_;
  cfg_srv_ = NULL;
  reader_ = NULL;
  Log(debug,"Reader destroyed");
}


void PTBManager::ExecuteCommand(const char* cmd,char *&answers) {
  //  try{
  msgs_.clear();
  msgs_.str("");
  error_state_ = false;
  Log(debug,"Received command [%s]", cmd);
  std::map<std::string, Command>::iterator it;
#if defined(DEBUG)
  for (it = commands_.begin(); it != commands_.end(); ++it) {
    Log(debug,"Key : [%s] : Value [%u]",it->first.c_str(),it->second );
  }
#endif
  switch (commands_.at(cmd)) {
    // -- Send the signal to start the run. Should be a register.
    case STARTRUN:
      // Only starts a run if the manager is in IDLE state
      //Issue a warning and restart a run
      // Otherwise issue a warning
      if (getStatus() != PTBManager::IDLE) {
        Log(warning,"A run is already running. Starting new run." );
        msgs_ << "<warning>Board already taking data. Restarting new run.Might miss the sync pulse.</warning>";
        StopRun();
        StartRun();
      } else {
        // Start the run
        Log(verbose,"Starting a new Run." );
        StartRun();
      }
      break;
    case SOFTRESET:
      //
      SetResetBit(true);
      // Sleep for 10 microseconds to make sure that reset has taken place
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      SetResetBit(false);
      // -- Re-commit the configuration
      // Not sure if this is going to work as intended
      // TODO: Only restore if the registers actually have something in them.
      RestoreConfigurationRegisters();
      SetConfigBit(true);
      msgs_ << "<success>true</success>";
      break;
    case HARDRESET:
      // A hard reset should sent the whole thing to zeros
      // Including clearing up the configuration. A new configuration will have to be reissued
      SetEnableBit(false);
      SetConfigBit(false);
      // the hard reset also includes a complete reset of the local buffers
      SetResetBit(true);
      // Sleep for 10 microseconds to make sure that reset has taken place
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      SetResetBit(false);
      if (reader_) {
        reader_->ResetBuffers();
      }
      ResetConfigurationRegisters();
      msgs_ << "<success>true</success>";
      break;
    case STOPRUN:
      if (getStatus() != PTBManager::RUNNING) {
        msgs_ << "<warning>Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.</warning>";
        Log(warning,"Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.." );
        // The GLB_EN is located in bin 31 of register 30
        SetEnableBit(false);
        Log(debug,"GLB_EN unset. Register: 0x%08x ",register_map_[0].value() );
      } else {
        Log(verbose,"The Run should STOP now" );
        StopRun();
      }
      break;
    default:
      msgs_ << "<error>Unknown PTB command [" << cmd << "] </error>";
      break;
  }
  msgs_str_ = msgs_.str();
  answers = new char[msgs_str_.size()+1];
  sprintf(answers,"%s",msgs_str_.c_str());
}

void PTBManager::StartRun() {
  Log(verbose,"Starting the run" );

  // Order of execution:
  // Tell the PTBReader to start taking data (it won't receive anything but it will be ready)
  // Set the GLB_EN register to 1 (start readout in the fabric)
  // Set status flag to RUNNING
  if (!reader_) {
    Log(warning,"No valid reader availble. Relaunching a new one.");
    msgs_ << "<warning>No valid PTB reader availble. Relaunching a new one.</warning>";
    reader_ = new PTBReader();
    reader_->setTcpHost(data_socket_host_);
    reader_->setTcpPort(data_socket_port_);
  }

  try {
    // As of Dec-2015 the connection is supposed to be open here. No warning should now be issued.
    // Open the connection. It should not be open yet
    if (!reader_->isReady()) {
      //    msgs_ << "<warning>Connection to board reader is not opened yet. Trying to reopen.</warning>";
      //    Log(warning,"Connection is not opened yet. Trying to reopen.");
      reader_->InitConnection();
    }

    if (!reader_->isReady()) {
      Log(warning,"Received call to start transmitting but reader is not ready. Refusing to run." );
      msgs_ << "<error>PTB data reader is not ready yet.Refusing to run.</error>";
      error_state_ = true;
      return;
    }
    reader_->StartDataTaking();
  }
  catch(SocketException &e) {
    msgs_ << "<error> PTB data socket exception (socket):" << e.what() << "</error>";
    error_state_ = true;
    return;
  }
  catch(PTBexception &e) {
    msgs_ << "<error>PTB data socket exception (user):" << e.what() << "</error>";
    error_state_ = true;
    return;
  }
  catch(std::exception &e) {
    msgs_ << "<error>PTB data socket exception (std):" << e.what() << "</error>";
    error_state_ = true;
    return;
  }
  catch (...) {
    Log(error,"Unknown error starting reader thread");
    msgs_ << "<error> PTB data socket exception (unknown): error starting reader.</error>";
    error_state_ = true;
    return;
  }

  // The GLB_EN is located in bin 31 of register 30
  SetEnableBit(true);

  Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[0].value() );

  status_ = RUNNING;
  // NFB Dec-06-2015
  // This does not work. The TDU manager waits for the boardReader to give the go-ahead
  // And this waits for the TDU manager to send the sync.
  // Better to implement a specific command to get the value after the run has started
  Log(info,"Run Start requested...");

  msgs_ << "<success>true</success>";
}

void PTBManager::StopRun() {
  Log(debug,"Stopping the run" );

  // The order should be:
  // 1. Set the GLB_EN register to 0 (stop readout in the fabric)
  // 2. Set the status to IDLE
  // 3. Tell the PTB reader to stop streaming

  // The GLB_EN is located in bin 31 of register 30
  SetEnableBit(false);
  Log(debug,"GLB_EN unset. Register: 0x%08x ",register_map_[0].value() );

  // Check the ACK bit
  // Try to set the run to stop on the software
  reader_->StopDataTaking();

  if (reader_->GetErrorState()) {
    msgs_ << reader_->GetErrorMessages();
    //msgs_ << "<error> Data socket was lost while taking data. </error>";
  }

  msgs_ << "<run_statistics num_eth_packets=\"";
  msgs_ << reader_->GetNumFragmentsSent();
  msgs_ << "\" num_word_counter=\"" << reader_->GetNumCounterWords();
  msgs_ << "\" num_word_trigger=\"" << reader_->GetNumTriggerWords();
  msgs_ << "\" num_word_tstamp=\"" << reader_->GetNumTimestampWords();
  msgs_ << "\" num_word_fifo_warn=\"" << reader_->GetNumFIFOWarnings();
  msgs_ << "\" num_bytes=\"" << reader_->GetBytesSent();
  msgs_ << "\" />";

  Log(info,"End of run message: %s",msgs_.str().c_str());

  //-- Soft Reset
  //-- Also send a soft reset to make sure that when the next start run comes
  // things are just as if we were starting anew.
  // Particularly important is to clear the buffers
  SetResetBit(true);
  // Sleep for 10 microseconds to make sure that reset has taken place
  std::this_thread::sleep_for (std::chrono::microseconds(10));
  SetResetBit(false);
  // -- Re-commit the configuration
  RestoreConfigurationRegisters();
  SetConfigBit(true);

  if (GetEnableBitACK() != false ) {
    Log(warning,"Stop Run failed. ACK bit still high.");
    msgs_ << "<warning>Failed to stop run on hardware. No ACK received.</warning>";
  } else {
    Log(info,"Run stopped");
    msgs_ << "<success>true</success>";
  }
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

  Log(info,"Setting up memory mapped registers");
#if defined(ARM_XDMA) || defined(ARM_MMAP)
  SetupConfRegisters();
  // First get the virtual address for the mapped physical address
  mapped_conf_base_addr_ = MapPhysMemory(conf_reg.base_addr,conf_reg.high_addr);
  Log(debug,"Received virtual address for configuration : 0x%08X\n",reinterpret_cast<uint32_t>(mapped_conf_base_addr_));
  // Cross check that we have at least as many offsets as registers expected
  if (conf_reg.n_registers < num_registers_) {
    Log(warning,"Have less configured registers than the ones required. (%u != %u)",conf_reg.n_registers,num_registers_);
  }

  register_map_[0].address =  reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_conf_base_addr_) + conf_reg.addr_offset[0]);
  register_map_[0].value() = CTL_BASE_REG_VAL;
  for (uint32_t i = 1; i < num_registers_; ++i) {
    register_map_[i].address =  reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_conf_base_addr_) + conf_reg.addr_offset[i]);
    register_map_[i].value() = 0;
  }

#endif /*ARM_XDMA*/

  // Allocate the memory for the register cache
  for (uint32_t i =0; i < num_registers_; ++i) {
    register_cache_[i].address = new uint32_t();
  }
}

void PTBManager::FreeRegisters() {
  Log(info,"Cleaning up the allocated registers to configure the board." );
#if defined(ARM_XDMA) || defined(ARM_MMAP)
  munmap(mapped_conf_base_addr_,conf_reg.high_addr-conf_reg.base_addr);
  // Close the file
  Log(info,"Closing /dev/mem");
  close(g_mem_fd);
  //    munmap(mapped_time_base_addr_,data_reg.high_addr-data_reg.base_addr);
#endif /*ARM*/
  Log(debug,"Configuration registers unmapped.");

  Log(debug,"Deleting the cache pointers");
  // Clear and free also the cache registers
  for (uint32_t i =0; i < num_registers_; ++i) {
    delete static_cast<uint32_t*>(register_cache_[i].address);
  }
  Log(debug,"Clearing the cache");
  register_cache_.clear();
  Log(info,"Memory released!");
}

void PTBManager::ProcessConfig(pugi::xml_node config,char *&answers) {
  msgs_.clear();
  msgs_.str("");

  // Only accept a new configuration if we are not running.
  // NFB: Not sure if I shouldn't always accept a config but simply place it in the cache.

  if (status_ == RUNNING) {
    msgs_ << "<warning>Attempted to pass a new configuration during a run. Ignoring the new configuration.</warning>";
    Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
    msgs_str_ = msgs_.str();
    answers = new char[msgs_str_.size()+1];
    sprintf(answers,"%s",msgs_str_.c_str());
    return;
  }

  std::stringstream strVal;

  //char *pEnd = NULL;

  // use this as an error catching state
  bool has_error = false;

  try{
    Log(info,"Applying a reset prior to the configuration.");
    SetResetBit(true);

    std::this_thread::sleep_for (std::chrono::microseconds(10));
    SetResetBit(false);


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
          reader_ = new PTBReader();
        }

        //-- Get the host
        data_socket_host_ = it->child("DaqHost").child_value();
        reader_->setTcpHost(data_socket_host_);
        // -- Get the port
        strVal << it->child("DaqPort").child_value();
        strVal >> data_socket_port_;
        strVal.clear();
        strVal.str("");
        //      unsigned short port = atoi(it->child("DaqPort").child_value());
        Log(debug,"DaqPort port %hu",data_socket_port_ );
        reader_->setTcpPort(data_socket_port_);
        Log(debug,"Setting data transmission channel to [%s:%hu]",data_socket_host_.c_str(),data_socket_port_);

        // -- Get if it is a dry run
        {
          std::string dry_run_state = it->child("DryRun").child_value();
          if (!dry_run_state.compare("true")) {
            reader_->SetDryRun(true);
            Log(warning,"Dry run mode enabled in the PTB. No data will be sent to board reader.");
          } else {
            reader_->SetDryRun(false);
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

  DumpConfigurationRegisters();

  // Set the bit to commit the configuration
  // into the hardware (bit 29 in register 30)
  Log(debug,"Committing configuration to the hardware.");
  Log(verbose,"Control register before config commit 0x%X", register_map_[0].value() );
  SetConfigBit(true);
  Log(debug,"Control register after config commit 0x%08X", register_map_[0].value() );
  if (!GetConfigBitACK()) {
    // If the commit is not ACK then effectively the board is not configured.
    // The best is to reset everything and fail the configuration.
    msgs_ << "<error>Failed set to set the configuration bit. ACK not received. Control register : "
        << std::hex << register_map_.at(0).value() << std::dec
        << ". Control register after reset : ";

    SetEnableBit(false);
    SetConfigBit(false);
    // the hard reset also includes a complete reset of the local buffers
    SetResetBit(true);
    // Sleep for 10 microseconds to make sure that reset has taken place
    std::this_thread::sleep_for (std::chrono::microseconds(10));
    SetResetBit(false);
    ResetConfigurationRegisters();
    SetConfigBit(false);
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
  SetEnableBit(false);
  SetConfigBit(false);
  for (size_t i = 1; i < register_map_.size(); ++i) {
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    register_map_.at(i).value() = 0x0;
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
  }
  register_map_.at(0).value() = CTL_BASE_REG_VAL;
  // Reset the control registers in steps as well
  //-- Shouldn't the configuration then be committed to the board?
  Log(debug,"Recommitting the configuration to the hardware.");
  SetConfigBit(true);
}

void PTBManager::RestoreConfigurationRegisters() {
  Log(debug,"Restoring configuration registers from local cache");
  // -- Do not restore the control register
  for (size_t i = 1; i < register_map_.size(); ++i) {
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    register_map_.at(i).value() = register_cache_.at(i).value();
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
  }

}

void PTBManager::ParseMuonTrigger(pugi::xml_node T, uint32_t reg, uint32_t conf_reg) {

  Log(verbose,"Processing muon trigger starting at reg %u and conf_reg  %u",reg,conf_reg );
  // NFB - Dec-04-2015
  // Remapped the whole thing

  bool reverse_order = (conf_reg<reg)?true:false;
  //uint32_t prsc_pos;
  uint32_t lreg;
  uint32_t nbits,mask, bit_offset, input_word;
  uint64_t input_longword;
  std::string field_name;
  std::stringstream strVal;

  // -- First thing, is to grab the mask and fill it.
  // This is the same for all pieces of firmware

  ////////////////////////////////////////////////////////////////
  /// M_?_EXT_OP - External logic to be applied between the groups
  /// ExtLogic : reg_offset[bit_offset - bit_offset+1]
  ////////////////////////////////////////////////////////////////
  mask = 0x3;
  nbits = 2;
  bit_offset = (reverse_order)?8:22;
  field_name = "ExtLogic";
  if (T.empty()) {
    msgs_ << "<warning>Couldn't find external muon logic. Assuming AND.</warning>";
    Log(warning,"Couldn't find external muon logic. Assuming AND.");
    input_word = 0x0;
  } else {
    strVal << std::hex << T.child(field_name.c_str()).child_value();
    strVal >> std::hex >> input_word;
  }

  if ((input_word != 0x0) && (input_word != 0x2) && (input_word != 0x1)) {
    msgs_ << "<warning>Unknown logic found. Assuming AND.</warning>";
    Log(warning,"Unknown logic found. Assuming AND.");
    input_word = 0x0;
  }
  // Assign the value to the register
  Log(debug,"ExtLogic (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),input_word,input_word,std::bitset<2>(input_word).to_string().c_str());
  SetBitRangeRegister(conf_reg,((input_word & mask) << (bit_offset)),bit_offset,nbits);
  Log(debug,"Register %u {%u-%u} [0x%08X]",conf_reg,bit_offset+nbits-1, bit_offset, register_map_[conf_reg].value() );

  strVal.clear();
  strVal.str("");

  // The prescale is a different matter
  // All the prescales go into the same register (36)
  ////////////////////////////////////////////////////////////////
  /// M_?_PRESC - Prescale of a given trigger
  /// ExtLogic : reg_offset[bit_offset - bit_offset+7]
  ////////////////////////////////////////////////////////////////
  // First find where in the register the prescale must go:
  uint32_t prscl_reg = 36;
  if (reg < conf_reg) {
    if (reg < 18) {
      bit_offset = 0;
    } else {
      bit_offset = 8;
    }
  } else {
    if (reg < 18) {
      bit_offset = 16;
    } else {
      bit_offset = 24;
    }
  }
  mask = 0xFF;
  nbits = 8;
  field_name = "Prescale";
  if (T.empty()) {
    msgs_ << "<warning>Couldn't find trigger prescale. Assuming 0.</warning>";
    Log(warning,"Couldn't find trigger prescale. Assuming 0.");
    input_word = 0x0;
  } else {
    strVal << std::dec << T.child(field_name.c_str()).child_value();
    strVal >> std::dec >> input_word;
  }
  Log(debug,"Prescale (%s) (%u) [0x%X] [%s]",strVal.str().c_str(),input_word,input_word,std::bitset<8>(input_word).to_string().c_str());
  SetBitRangeRegister(prscl_reg,((input_word & mask) << bit_offset),bit_offset,8);
  Log(debug,"Register %u {%u-%u} [0x%08X]",prscl_reg,bit_offset+nbits-1, bit_offset, register_map_[prscl_reg].value() );

  strVal.clear();
  strVal.str("");

  ////////////////////////////////////////////////////////////////
  ///
  /// Group entries
  ///
  ////////////////////////////////////////////////////////////////

  const unsigned int ngroups = 2;
  const char*groups[] = {"group1","group2"};

  for (uint32_t i = 0; i < ngroups; ++i) {
    pugi::xml_node G;

    if (!T.empty()) {
      G = T.child(groups[i]);
    }
    // Now the more messy part of using a long register
    Log(debug,"Processing group [%s]",groups[i]);

    // Intra group logic:
    mask = 0x3;
    nbits = 2;
    // trig. A:: g1=4, g2=6
    // trig. B:: g1=24, g2=26
    bit_offset = (reverse_order)?(24+(2*i)):(4+(2*i));
    field_name = "Logic";

    if (T.empty()) {
      msgs_ << "<warning>Couldn't find internal logic for group " << groups[i]+1 << ". Assuming OR.</warning>";
      Log(warning,"Couldn't find internal logic for group %u. Assuming OR.",groups[i]+1);
      input_word = 0x1;
    } else {
      strVal << std::hex << G.child(field_name.c_str()).child_value();
      strVal >> std::hex >> input_word;
    }
    if ((input_word != 0x1) && (input_word != 0x2) && (input_word != 0x3)) {
      msgs_ << "<warning>Unknown internal logic found. Assuming OR.</warning>";
      Log(warning,"Unknown logic found. Assuming OR.");
      input_word = 0x1;
    }

    Log(debug,"Logic g%u (%s) (%u) [0x%X] [%s]",i+1,strVal.str().c_str(),input_word,input_word,std::bitset<2>(input_word).to_string().c_str());
    SetBitRangeRegister(conf_reg,((input_word & mask) << bit_offset),bit_offset,2);
    Log(debug,"Register %u {%u-%u} [0x%08X]",conf_reg,bit_offset+nbits-1, bit_offset, register_map_[conf_reg].value() );

    strVal.clear();
    strVal.str("");

    ////////////////////////////////////////
    ///
    /// -- Now the masks
    ///
    ////////////////////////////////////////
    field_name = "TSU";
    if (T.empty()) {
      msgs_ << "<warning>Couldn't find mask for TSU in group " << groups[i] << ". Assuming 0x0.</warning>";
      Log(warning,"Couldn't find mask for TSU in group %s. Assuming 0x0.",groups[i]);
      input_longword = 0x0;
    } else {
      strVal << std::hex << G.child(field_name.c_str()).child_value();
      strVal >> std::hex >> input_longword;
    }
    Log(debug,"TSU mask [0x%" PRIX64 "] [%s]",input_longword,std::bitset<48>(input_longword).to_string().c_str());

    // -- This part now could probably be made a bit more iterative
    // I know that the TSU's go into the lower registers, followed by the BSU's
    // tsu[31:0]
    // Trig. A
    // g1: 5,6,7,8    g2: 8,9,10,11
    // Trig. B
    // g1: 12,13,14,15 g2: 15,16,17,11
    mask = 0xFFFFFFFF;
    nbits = 32;
    lreg = reg+(i*3);
    register_map_[lreg].value() = (input_longword & mask);
    Log(debug,"Mask register %u (W1) {31-0} [0x%08X]",lreg,register_map_[lreg].value() );
    // The rest of the TSU's go into the next register
    //tsu[47:32] : 16 bits --> word[15:0]
    mask = 0xFFFF;
    nbits = 16;
    bit_offset = 0;
    lreg += 1; // reg = 6
    SetBitRangeRegister(lreg,((input_longword >> 32) & mask)<< bit_offset,bit_offset,nbits);
    Log(debug,"Mask register %u (W2) {%u-%u} [0x%08X]",lreg,nbits+bit_offset-1,bit_offset,register_map_[lreg].value() );

    strVal.clear();
    strVal.str("");


    // Now the BSU's
    field_name = "BSU";
    if (T.empty()) {
      msgs_ << "<warning>Couldn't find mask for BSU in group " << groups[i] << ". Assuming 0x0.</warning>";
      Log(warning,"Couldn't find mask for TSU in group %s. Assuming 0x0.",groups[i]);
      input_longword = 0x0;
    } else {
      strVal << std::hex << G.child(field_name.c_str()).child_value();
      strVal >> input_longword;
    }
    Log(debug,"BSU mask [0x%" PRIX64 "] [%s]",input_longword,std::bitset<50>(input_longword).to_string().c_str());

    // bsu[15-0] (16 bits) -> word[31:16]
    mask = 0xFFFF;
    bit_offset = 16;
    nbits = 16;
    SetBitRangeRegister(lreg,(input_longword & mask) << bit_offset,bit_offset,nbits);
    Log(debug,"Mask register %u (W1) {%u-%u} [0x%08X]",lreg,bit_offset+nbits-1,bit_offset,register_map_[lreg].value() );

    lreg += 1; // reg=7
    mask = 0xFFFFFFFF;
    nbits = 32;
    // bsu[47:16] (31 bits) --> word[31:0]
    register_map_[lreg].value() = (input_longword >> 16) & mask;
    Log(debug,"Mask register %u (W2) {31-0} [0x%08X]",lreg,register_map_[lreg].value() );

    // The last bits are a bit more special.
    // For group 1, they are the first 2 bits in the configuration register
    // For group 2, they are the next 2
    lreg = conf_reg;
    mask = 0x3;
    nbits = 2;
    if (i==0) {
      bit_offset = (reverse_order)?28:0;
    } else {
      bit_offset = (reverse_order)?30:2;
    }
    SetBitRangeRegister(lreg,((input_longword >> 48) & mask) << bit_offset,bit_offset,nbits);
    Log(debug,"Mask register %u (W3) {%u-%u} [0x%08X]",lreg,bit_offset+nbits-1,bit_offset,register_map_[lreg].value() );

    strVal.clear();
    strVal.str("");
  }
}


/// -- Short hand methods to be used on the higher level above
///
/// NOTE: The value has to be shifted to the right bit positions
/// Position is in bit number. If position is set to 16, the new value starts being inserted in the
/// 17th bit (bit 16, since bits start at 0).
void PTBManager::SetBitRangeRegister(uint32_t reg, uint32_t value, uint32_t pos, uint32_t len) {
  // Create a mask based on pos and len
  uint32_t mask = (((uint32_t)1 << len)-1) << pos;
  register_map_[reg].value() = (register_map_[reg].value() & ~mask) | (value & mask);
}


bool PTBManager::GetBit(uint32_t reg, uint32_t bit) {
  return (((register_map_[reg].value() >> bit) & 0x1) == 0x1)?true:false;
}
void PTBManager::SetBit(uint32_t reg, uint32_t bit, bool status) {
  // Things are more complicated than this. We want to set a single bit, regardless of what is around
  Log(debug,"Reading the bit %u from reg %u",bit,reg);
#if defined(ARM_XDMA) || defined(ARM_MMAP)
  uint32_t value = Xil_In32((uint32_t)register_map_[reg].address);
  uint32_t new_value = value^((-(status?1:0) ^ value) & ( 1 << bit));

  Log(debug,"Got %08X -> %08X",value,new_value);
  Xil_Out32((uint32_t)register_map_[reg].address,new_value);
#else
  // -- force making a copy
  uint32_t value = register_map_[reg].value();
  uint32_t new_value = value^((-(status?1:0) ^ value) & ( 1 << bit));
  Log(debug,"Got %08X -> %08X",value,new_value);
  register_map_[reg].value() = new_value;
#endif
  Log(debug,"Final register value %08X",register_map_[reg].value());
}

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

