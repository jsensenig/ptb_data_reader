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

#include <iomanip>
#include <thread>
#include <chrono>
#include <bitset>
#include <cmath>
#include <cinttypes>


#define __STDC_FORMAT_MACROS

extern "C" {
#include <unistd.h>
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


  void board_manager::exec_command(const std::string &cmd,std::vector<std::string> &answers) {
    feedback_.clear();
    std::ostringstream msgs_;
    //  try{
    //    msgs_.clear();
    //    msgs_.str("");
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
          msgs_ << "Warning::Board already taking data. Restarting new run.Might miss the sync pulse.";
          feedback_.push_back(msgs_.str());
          msgs_.clear();
          msgs_.str("");
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
        msgs_ << "Result: SUCCESS";
        feedback_.push_back(msgs_.str());
        msgs_.clear();
        msgs_.str("");
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
        msgs_ << "Result: SUCCESS";
        feedback_.push_back(msgs_.str());
        msgs_.clear();
        msgs_.str("");
        break;
      case STOPRUN:
        if (get_board_state() != board_manager::RUNNING) {
          msgs_ << "Warning: Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.";
          feedback_.push_back(msgs_.str());
          msgs_.clear();
          msgs_.str("");
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
        msgs_ << "ERROR:Unknown PTB command [" << cmd << "].";
        feedback_.push_back(msgs_.str());
        msgs_.clear();
        msgs_.str("");
        break;
    }
    answers = feedback_;
    //  msgs_str_ = msgs_.str();
    //answers = msgs_.str();
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
      feedback_.push_back("WARNING: No valid PTB reader availble. Relaunching a new one.");
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
        feedback_.push_back("ERROR: PTB data reader is not ready yet.Refusing to run.");
        error_state_ = true;
        return;
      }
      reader_->start_data_taking();
    }
    catch(SocketException &e) {
      std::string msg = "ERROR: PTB data socket exception (socket): ";
      msg += e.what();
      feedback_.push_back( msg.c_str());
      error_state_ = true;
      return;
    }
    catch(op_exception &e) {
      std::string msg = "ERROR: PTB data socket exception (user): ";
      msg += e.what();
      feedback_.push_back( msg.c_str());
      error_state_ = true;
      return;
    }
    catch(std::exception &e) {
      std::string msg = "ERROR: PTB data socket exception (std): ";
      msg += e.what();
      feedback_.push_back( msg.c_str());
      error_state_ = true;
      return;
    }
    catch (...) {
      Log(error,"Unknown error starting reader thread");
      feedback_.push_back("ERROR: PTB data socket exception (unknown): error starting reader.");
      error_state_ = true;
      return;
    }

    // The GLB_EN is located in bin 31 of register 30
    set_enable_bit(true);

    Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[0].value() );

    board_state_ = RUNNING;

    Log(info,"Run Start requested...");

    feedback_.push_back("INFO: Success");
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
      std::vector<std::string> tmp = reader_->get_error_msgs();
      feedback_.insert(std::end(feedback_), std::begin(tmp), std::end(tmp));
      //msgs_ << "<error> Data socket was lost while taking data. </error>";
    }
    std::ostringstream msgs_;
    msgs_ << "\"statistics\" : {\"num_eth_packets=\"";
    msgs_ << reader_->get_n_sent_frags();
    msgs_ << ", \"num_word_counter\"=" << reader_->get_n_status();
    msgs_ << ", \"num_hlt\"=" << reader_->get_n_gtriggers();
    msgs_ << ", \"num_llt\"=" << reader_->get_n_ltriggers();
    msgs_ << ", \"num_word_tstamp\"=" << reader_->get_n_timestamps();
    msgs_ << ", \"num_word_fifo_warn\"=" << reader_->get_n_warns();
    msgs_ << ", \"num_bytes\"=" << reader_->get_sent_bytes();
    msgs_ << "}";

    feedback_.push_back(msgs_.str());
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
      feedback_.push_back("WARNING : Failed to stop run on hardware. No ACK received.");
    } else {
      Log(info,"Run stopped");
      feedback_.push_back("INFO: Success");
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


#ifndef OLD_CODE
  void board_manager::process_config(json &doc,std::vector<std::string> &answers) {
    std::ostringstream msgs_;
    msgs_.str("");


    if (board_state_ == RUNNING) {
      answers.push_back("WARNING : Attempted to pass a new configuration during a run. Ignoring the new configuration.");
      Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
      //feedback = msgs_.str();
      return;
    }
    bool has_error = false;
    try{
      Log(info,"Applying a reset prior to the configuration.");
      set_reset_bit(true);
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      set_reset_bit(false);

      Log(warning,"Still in development. Only a few configuration registers are being set");
    }
    // -- Only catch something deriving from std::exception and unspecified
    catch(std::exception &e) {
      std::string msg = "ERROR: Error processing configuration: ";
      msg += e.what();
      msg += ". Not committing configuration to PTB.";
      answers.push_back(msg);
      has_error = true;
    }
    catch(...) {
      answers.push_back("ERROR :Unknown error processing configuration. Not committing configuration to PTB.");
      has_error = true;
    }
    // Check if we got an error. If so, do not commit.
    if (has_error) {
      // Don't commit. Just go back and throw the error.
      return;
    }

    if (!reader_) {
      Log(warning,"Don't have a valid PTBReader instance. Attempting to create a new one." );
      answers.push_back("WARNING:Don't have a valid PTBReader instance. Attempting to create a new one.");
      reader_ = new board_reader();
    }

    // -- First grab the server information
    json receiver = doc.at("ctb").at("sockets").at("receiver");
    data_socket_host_ = receiver.at("host").get<std::string>();
    reader_->set_tcp_host(data_socket_host_);
    data_socket_port_ = receiver.at("port").get<unsigned short>();
    reader_->set_tcp_port(data_socket_port_);
    Log(debug,"Setting data transmission channel to [%s:%hu]",data_socket_host_.c_str(),data_socket_port_);
    
    // uint32_t duration;
    std::stringstream strVal;
    // strVal << receiver.at("rollover").get<std::string>();
    // strVal >> std::dec >> duration;
    uint32_t duration = receiver.at("rollover").get<unsigned int>();
    // Microslice duration is now a full number...check if it fits into 27 bits
    Log(debug,"MicroSlice Duration [%d] (%u) [0x%X][%s]",duration,duration,duration, std::bitset<27>(duration).to_string().c_str());
    if (duration >= (1<<27)) {
      //    answers.push_back("WARNING:Input value of [" << duration << "] above the allowed limit of 27 bits. Setting to maximum allowed.";
      Log(warning,"Input value of [%u] above maximum rollover [27]. Truncating to maximum.",duration);
      duration = (1<<27)-1;
    }

    // 1 ms in a 50MHz clock
    //  if (duration <= 50000) {
    //    msgs_ << "<warning>Input value of ["<< duration << "] below recommended limit of 1 ms (50000). ";
    //    msgs_ << "Will allow but this setting is likely to cause overload of the ethernet connection. </warning>";

    //  }

    set_bit_range_register(6,0,27,duration);
    Log(debug,"Register 6 : [0x%08X]", register_map_[6].value() );
    strVal.clear();
    strVal.str("");

    // -- Once the configuration is set, dump locally the status of the config registers
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
      msgs_ << "ERROR: Failed set to set the configuration bit. ACK not received. Control register : "
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
      answers.push_back(msgs_.str());
      msgs_.clear();
      msgs_.str("");
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
      config_ = doc;
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
      answers.push_back("INFO: Success");

    }
    // Most likely the connection will fail at this point. Not a big problem.
//    Log(verbose,"Returning from SetConfig with answer [%s]",feedback.c_str());

  }

#else
  // TODO: Implement this for the specific application.
  void board_manager::process_config(pugi::xml_node config,std::string &feedback) {
    msgs_.clear();
    msgs_.str("");

    // Only accept a new configuration if we are not running.
    // NFB: Not sure if it shouldn't always accept a config but simply place it in the cache.
    //      This could turn into a mess if we didn't know what was the latest good running configuration

    if (board_state_ == RUNNING) {
      msgs_ << "<warning>Attempted to pass a new configuration during a run. Ignoring the new configuration.</warning>";
      Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
      feedback = msgs_.str();
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

      Log(warning,"Still in development. Only a few configuration registers are being set");

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

          set_bit_range_register(6,0,27,duration);
          Log(debug,"Register 6 : [0x%08X]", register_map_[6].value() );
          strVal.clear();
          strVal.str("");
        }

      }
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
      feedback += msgs_.str();
      return;
    }

    // -- Once the configuration is set, dump locally the status of the config registers
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
      feedback += msgs_.str();
      // msgs_str_ = msgs_.str();
      // answers = new char[msgs_str_.size()+1];
      // sprintf(answers,"%s",msgs_str_.c_str());

    } else {
      // Don't commit. Just go back and throw the error.
      feedback += msgs_.str();
      // msgs_str_ = msgs_.str();
      // answers = new char[msgs_str_.size()+1];
      // sprintf(answers,"%s",msgs_str_.c_str());
    }
    // Most likely the connection will fail at this point. Not a big problem.
    Log(verbose,"Returning from SetConfig with answer [%s]",feedback.c_str());
  }

#endif
}
