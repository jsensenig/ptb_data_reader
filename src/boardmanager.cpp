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
  Log(debug,"Received command [%s]", cmd);
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
        start_run();
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
  msgs_ << "\" num_bytes=\"" << reader_->GetBytesSent();
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

#endif

  // Allocate the memory for the register cache
  for (uint32_t i =0; i < num_registers_; ++i) {
    register_cache_[i].addr = new uint32_t();
  }
}

void board_manager::free_registers() {
  Log(info,"Cleaning up the allocated configuration registers." );
#if defined(ARM_XDMA) || defined(ARM_MMAP)
  size_t range = conf_reg.high_addr-conf_reg.base_addr;
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


}
