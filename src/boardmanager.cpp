/*
 * PTBManager.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "boardmanager.h"

#include "boardreader.h"
#include "boardserver.h"
#include "Logger.h"
#include "opexception.h"
#include "PracticalSocket.h"
#include "util.h"


#if !defined(SIMULATION)
//defined(ARM_XDMA) || defined(ARM_MMAP)
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

//FIXME: Make sure that the default control register has this value after
// a hard reset
#define CTL_BASE_REG_VAL 0x00000000

namespace ptb {

  config::mapped_register conf_reg;

  // Init with a new reader attached
  board_manager::board_manager( ) :
		    reader_(0),
		    //cfg_srv_(0),
		    board_state_(IDLE),
		    mapped_conf_base_addr_(nullptr),
		    msgs_str_(""),
		    error_state_(false),
		    data_socket_host_("192.168.100.100"),
		    data_socket_port_(8992) {
    reader_ =	new board_reader();
    Log(debug,"Setting up pointers." );

    // Register the available commands
    commands_.insert(std::map<std::string, command>::value_type("StartRun",STARTRUN));
    commands_.insert(std::map<std::string, command>::value_type("StopRun",STOPRUN));
    commands_.insert(std::map<std::string, command>::value_type("SoftReset",SOFTRESET));
    commands_.insert(std::map<std::string, command>::value_type("HardReset",HARDRESET));
    // Setup the registers first, to make sure that if the data
    // manager receives something we are ready to deliver.
    setup_registers();

    // This had to occur after setting up the command list
    //  or we might run into trouble.
    // cfg_srv_ = socket_server::get();
    // Register to receive callbacks
    // cfg_srv_->RegisterDataManager(this);

  }

  board_manager::~board_manager() {
    Log(debug,"Destroying manager..." );
    free_registers();
    // Clear the map
    commands_.clear();
    Log(debug,"Killing the reader");
    delete reader_;
    //cfg_srv_ = NULL;
    reader_ = nullptr;
    Log(debug,"Reader destroyed");
  }


  void board_manager::exec_command(const std::string &cmd,std::string &answers) {
    //  try{
    msgs_.clear();
    msgs_.str("");
    error_state_ = false;
    Log(debug,"Received command [%s]", cmd.c_str());
    std::map<std::string, command>::iterator it;
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
      if (get_board_state() != board_manager::IDLE) {
        Log(warning,"A run is already running. Starting new run." );
        msgs_ << "<warning>Board already taking data. Restarting new run.Might miss the sync pulse.</warning>";
        stop_run();
      } else {
        // Start the run
        Log(verbose,"Starting a new Run." );
        //start_run();
      }
      start_run();
      break;
    case SOFTRESET:
      Log(info,"Applying a soft reset");
      set_reset_bit(true);
      // Sleep for 100 microseconds to make sure that reset has taken place
      std::this_thread::sleep_for (std::chrono::microseconds(100));
      set_reset_bit(false);
      // -- Re-commit the configuration
      // Not sure if this is going to work as intended
      // TODO: Only restore if the registers actually have something in them.
      restore_config_registers();
      set_config_bit(true);
      msgs_ << "<success>true</success>";
      break;
    case HARDRESET:
      // A hard reset should sent the whole thing to zeros
      // Including clearing up the configuration. A new configuration will have to be reissued
      set_enable_bit(false);
      set_config_bit(false);
      // the hard reset also includes a complete reset of the local buffers
      set_reset_bit(true);
      // Sleep for 100 microseconds to make sure that reset has taken place
      std::this_thread::sleep_for (std::chrono::microseconds(100));
      set_reset_bit(false);
      if (reader_) {
        reader_->reset_buffers();
      }
      zero_config_registers();
      msgs_ << "<success>true</success>";
      break;
    case STOPRUN:
      if (get_board_state() != board_manager::RUNNING) {
        msgs_ << "<warning>Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.</warning>";
        Log(warning,"Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.." );
        // The GLB_EN is located in bin 31 of register 30
        set_enable_bit(false);
        Log(debug,"GLB_EN unset. Register: 0x%08x ",register_map_[0].value() );
      } else {
        Log(verbose,"The Run should STOP now" );
        stop_run();
      }
      break;
    default:
      msgs_ << "<error>Unknown PTB command [" << cmd << "] </error>";
      break;
    }
    //  msgs_str_ = msgs_.str();
    answers = msgs_.str();
    //  answers = new char[msgs_str_.size()+1];
    //  sprintf(answers,"%s",msgs_str_.c_str());
  }

  void board_manager::start_run() {
    Log(verbose,"Starting the run" );

    // Order of execution:
    // Tell the PTBReader to start taking data (it won't receive anything but it will be ready)
    // Set the GLB_EN register to 1 (start readout in the fabric)
    // Set status flag to RUNNING
    if (!reader_) {
      Log(warning,"No valid reader availble. Relaunching a new one.");
      msgs_ << "<warning>No valid PTB reader availble. Relaunching a new one.</warning>";
      reader_ = new board_reader();
      reader_->set_tcp_host(data_socket_host_);
      reader_->set_tcp_port(data_socket_port_);
    }

    try {
      // Open the connection. It should not be open yet
      if (!reader_->get_ready()) {
        //    msgs_ << "<warning>Connection to board reader is not opened yet. Trying to reopen.</warning>";
        //    Log(warning,"Connection is not opened yet. Trying to reopen.");
        reader_->init_data_connection();
      }

      if (!reader_->get_ready()) {
        Log(warning,"Received call to start transmitting but reader is not ready. Refusing to run." );
        msgs_ << "<error>PTB data reader is not ready yet.Refusing to run.</error>";
        error_state_ = true;
        return;
      }
      reader_->start_data_taking();
    }
    catch(SocketException &e) {
      msgs_ << "<error> PTB data socket exception (socket):" << e.what() << "</error>";
      error_state_ = true;
      return;
    }
    catch(op_exception &e) {
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
    set_enable_bit(true);

    Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[0].value() );

    board_state_ = RUNNING;

    Log(info,"Run Start requested...");

    msgs_ << "<success>true</success>";
  }

  void board_manager::stop_run() {
    Log(debug,"Stopping the run" );

    // The order should be:
    // 1. Set the GLB_EN register to 0 (stop readout in the fabric)
    // 2. Set the status to IDLE
    // 3. Tell the PTB reader to stop streaming

    // The GLB_EN is located in bin 31 of register 30
    set_enable_bit(false);
    Log(debug,"GLB_EN unset. Register: 0x%08x ",register_map_[0].value() );

    // Check the ACK bit
    // Try to set the run to stop on the software
    reader_->stop_data_taking();

    if (reader_->get_error_state()) {
      msgs_ << reader_->get_error_msgs();
      //msgs_ << "<error> Data socket was lost while taking data. </error>";
    }

    msgs_ << "<run_statistics num_eth_packets=\"";
    msgs_ << reader_->get_n_sent_frags();
    msgs_ << "\" num_word_counter=\"" << reader_->get_n_status();
    msgs_ << "\" num_word_trigger=\"" << reader_->get_n_triggers();
    msgs_ << "\" num_word_tstamp=\"" << reader_->get_n_timestamps();
    msgs_ << "\" num_word_fifo_warn=\"" << reader_->get_n_warns();
    msgs_ << "\" num_bytes=\"" << reader_->get_sent_bytes();
    msgs_ << "\" />";

    Log(info,"End of run message: %s",msgs_.str().c_str());

    //-- Soft Reset
    //-- Also send a soft reset to make sure that when the next start run comes
    // things are just as if we were starting anew.
    // Particularly important is to clear the buffers
    set_reset_bit(true);
    // Sleep for 100 microseconds to make sure that reset has taken place
    std::this_thread::sleep_for (std::chrono::microseconds(100));
    set_reset_bit(false);
    // -- Re-commit the configuration
    restore_config_registers();
    set_config_bit(true);

    if (get_enable_bit_ACK() != false ) {
      Log(warning,"Stop Run failed. ACK bit still high.");
      msgs_ << "<warning>Failed to stop run on hardware. No ACK received.</warning>";
    } else {
      Log(info,"Run stopped");
      msgs_ << "<success>true</success>";
    }
    board_state_ = IDLE;
  }

  void board_manager::setup_registers() {
    Log(info,"Setting up the appropriate registers to configure the board." );
    // Are we in emulator mode?

    util::mem_reg tmp_reg;
    tmp_reg.addr =NULL;
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
#if !defined(SIMULATION)
    ptb::config::setup_ptb_registers();
    // First get the virtual address for the mapped physical address
    Log(debug,"Mapping physical address [0x%X 0x%X]",ptb::config::conf_reg.base_addr,ptb::config::conf_reg.high_addr);
    mapped_conf_base_addr_ = ptb::util::map_physical_memory(ptb::config::conf_reg.base_addr,ptb::config::conf_reg.high_addr);

    Log(debug,"Received virtual address for configuration : 0x%08X\n",reinterpret_cast<uint32_t>(mapped_conf_base_addr_));
    // Cross check that we have at least as many offsets as registers expected
    if (ptb::config::conf_reg.n_registers < num_registers_) {
      Log(warning,"Have less configured registers than the ones required. (%u != %u)",ptb::config::conf_reg.n_registers,num_registers_);
    }

    register_map_[0].addr =  reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_conf_base_addr_) + ptb::config::conf_reg.addr_offset[0]);
    register_map_[0].value() = CTL_BASE_REG_VAL;
    for (uint32_t i = 1; i < num_registers_; ++i) {
      register_map_[i].addr =  reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_conf_base_addr_) + ptb::config::conf_reg.addr_offset[i]);
      register_map_[i].value() = 0;
    }

#endif

    // Allocate the memory for the register cache
    for (uint32_t i =0; i < num_registers_; ++i) {
      register_cache_[i].addr = new uint32_t();
    }
  }

  void board_manager::free_registers() {
    Log(info,"Cleaning up the allocated configuration registers." );
#if defined(ARM_XDMA) || defined(ARM_MMAP)
    size_t range = ptb::config::conf_reg.high_addr-ptb::config::conf_reg.base_addr;
    munmap(mapped_conf_base_addr_,range);
    // Close the file
    Log(info,"Closing /dev/mem");
    close(ptb::util::g_mem_fd);
    //    munmap(mapped_time_base_addr_,data_reg.high_addr-data_reg.base_addr);
#endif /*ARM*/
    Log(debug,"Configuration registers unmapped.");

    Log(debug,"Deleting the cache pointers");
    // Clear and free also the cache registers
    for (uint32_t i =0; i < num_registers_; ++i) {
      delete static_cast<uint32_t*>(register_cache_[i].addr);
    }
    Log(debug,"Clearing the cache");
    register_cache_.clear();
    Log(info,"Memory released!");
  }


  void board_manager::dump_config_registers() {
    Log(info,"Dumping configuration registers.");
    Log(info,"===========================================================");
    for (size_t i = 0; i < register_map_.size(); ++i) {
      Log(info,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    }
    Log(info,"===========================================================");

  }

  // -- Set's all registers to 0x0
  // -- And the control register as well.
  void board_manager::zero_config_registers() {
    Log(debug,"Resetting configuration registers");
    stop_run();
    set_config_bit(false);
    for (size_t i = 1; i < register_map_.size(); ++i) {
      Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
      register_map_.at(i).value() = 0x0;
      Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    }
    register_map_.at(0).value() = CTL_BASE_REG_VAL;
    // Reset the control registers in steps as well
    //-- Shouldn't the configuration then be committed to the board?
    Log(debug,"Recommitting the configuration to the hardware.");
    set_config_bit(true);
  }

  void board_manager::restore_config_registers() {
    Log(debug,"Restoring configuration registers from local cache");
    // -- Do not restore the control register
    for (size_t i = 1; i < register_map_.size(); ++i) {
      Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
      register_map_.at(i).value() = register_cache_.at(i).value();
      Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value(), register_map_.at(i).value() );
    }

  }

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
      Log(warning,"Still in development. No configu registers are being set");

      // -- For the moment do nothing.

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
        }

        if (!strcmp(it->name(),"RolloverClocks")) {
          uint32_t duration;
          strVal <<it->child_value();
          strVal >> std::dec >> duration;

          // Microslice duration is now a full number...check if it fits into 27 bits
          Log(debug,"MicroSlice Duration [%s] (%u) [0x%X][%s]",strVal.str().c_str(),duration,duration, std::bitset<27>(duration).to_string().c_str());
          if (duration >= (1<<27)) {
            msgs_ << "<warning>Input value of [" << duration << "] above the allowed limit of 27 bits. Setting to maximum allowed.";
            Log(warning,"Input value of [%u] above maximum rollover [27]. Truncating to maximum.",duration);
            duration = (1<<27)-1;
          }

          // 1 ms in a 50MHz clock
          if (duration <= 50000) {
            msgs_ << "<warning>Input value of ["<< duration << "] below recommended limit of 1 ms (50000). ";
            msgs_ << "Will allow but this setting is likely to cause overload of the ethernet connection. </warning>";

          }

	  set_bit_range_register(6,duration,0,27);
	  //SetBitRangeRegister(6,duration,0,27);
	  Log(debug,"Register 6 : [0x%08X]", register_map_[6].value() );
	  strVal.clear();
	  strVal.str("");
        }
	
      }

    /**
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
       **/
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
      answers += msgs_.str();
      // msgs_str_ = msgs_.str();
      // answers = new char[msgs_str_.size()+1];
      // sprintf(answers,"%s",msgs_str_.c_str());
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
      Log(warning,"Failed to set configuration bit. ACK not received. Control register %X",register_map_.at(0).value());

      set_enable_bit(false);
      Log(debug,"Control after disable: %X", register_map_.at(0).value());
      set_config_bit(false);
      Log(debug,"Control after disable config : %X", register_map_.at(0).value());
      // the hard reset also includes a complete reset of the local buffers
      set_reset_bit(true);
      Log(debug,"Control after reset enable: %X", register_map_.at(0).value());

      // Sleep for 10 microseconds to make sure that reset has taken place
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      set_reset_bit(false);
      Log(debug,"Control after reset disable: %X", register_map_.at(0).value());

      zero_config_registers();
      Log(debug,"Control zero config registers: %X", register_map_.at(0).value());

      set_config_bit(false);
      Log(debug,"Control after disable config again: %X", register_map_.at(0).value());

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
      answers += msgs_.str();
      // msgs_str_ = msgs_.str();
      // answers = new char[msgs_str_.size()+1];
      // sprintf(answers,"%s",msgs_str_.c_str());

    } else {
      // Don't commit. Just go back and throw the error.
      answers += msgs_.str();
      // msgs_str_ = msgs_.str();
      // answers = new char[msgs_str_.size()+1];
      // sprintf(answers,"%s",msgs_str_.c_str());
    }
    // Most likely the connection will fail at this point. Not a big problem.
    Log(verbose,"Returning from SetConfig with answer [%s]",answers.c_str());
  }


}
