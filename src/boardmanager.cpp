/*
 * PTBManager.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "boardmanager.h"
#include "boardreader.h"
#include "Logger.h"
#include "PracticalSocket.h"
#include "util.h"
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

  }

  board_manager::~board_manager() {
    Log(debug,"Destroying manager..." );
    free_registers();
    // Clear the map
    commands_.clear();
    Log(debug,"Killing the reader");
    delete reader_;
    reader_ = nullptr;
    Log(debug,"Reader destroyed");
  }


  void board_manager::exec_command(const std::string &cmd, json &answers) {
    feedback_.clear();
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
        // Issue a warning and restart a run
        // Otherwise issue a warning
        if (get_board_state() != board_manager::IDLE) {
          Log(warning,"A run is already running. Starting new run." );
          json obj;
          obj["type"] = "warning";
          obj["message"] = "Board already taking data. Restarting new run.Might miss the sync pulse.";
          feedback_.push_back(obj);
          stop_run();
        }
        // Start the run
        Log(verbose,"Starting a new Run." );
        start_run();
        break;
      case SOFTRESET:
        Log(debug,"Applying a soft reset");
        set_reset_bit(true);
        // Sleep for 100 microseconds to make sure that reset has taken place
        std::this_thread::sleep_for (std::chrono::microseconds(100));
        set_reset_bit(false);
        // -- Re-commit the configuration
        // Not sure if this is going to work as intended
        // TODO: Only restore if the registers actually have something in them.
        restore_config_registers();
        set_config_bit(true);
        break;
      case HARDRESET:
        Log(debug,"Applying a hard reset");

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
        break;
      case STOPRUN:
        if (get_board_state() != board_manager::RUNNING) {
          Log(warning,"Called for STOPRUN but there is no run ongoing. Just forcing hardware to stop.." );
          // -- we could still try to stop, no???
        }
        Log(debug,"The Run should STOP now" );
        stop_run();
        break;
      default:
        json obj;
        obj["type"] = "error";
        obj["message"] = "Unknown PTB command [" + cmd + "]";
        feedback_.push_back(obj);
        error_state_ = true;
        break;
    }

    if (error_state_) {
      json obj;
      obj["type"] = "info";
      obj["message"] = "Failed to execute command (see previous messages)";
      feedback_.push_back(obj);

    } else {
      json obj;
      obj["type"] = "info";
      obj["message"] = "Command executed.";
      feedback_.push_back(obj);
    }
    answers = feedback_;
  }

  // -- Writes everything to the feedback
  void board_manager::start_run() {
    Log(verbose,"Starting the run" );

    // Order of execution:
    // Tell the PTBReader to start taking data (it won't receive anything but it will be ready)
    // Set the GLB_EN register to 1 (start readout in the fabric)
    // Set status flag to RUNNING
    if (!reader_) {
      Log(warning,"No valid reader available. Relaunching a new one.");
      json obj;
      obj["type"] = "warning";
      obj["message"] = "No valid reader available. Relaunching a new one";
      feedback_.push_back(obj);

      reader_ = new board_reader();
      reader_->set_tcp_host(data_socket_host_);
      reader_->set_tcp_port(data_socket_port_);
    }

    try {
      // Open the connection. It should not be open yet
      if (!reader_->get_ready()) {
        reader_->init_data_connection();
        // sleep for a few ms to check for erorrs
        usleep(3000);
        bool has_error = false;
        json reader_msgs;
        reader_->get_feedback(has_error,reader_msgs,true);
        if (!reader_msgs.empty()) {

          feedback_.insert(std::end(feedback_),reader_msgs.begin(),reader_msgs.end());
        }
        if (has_error) {
          Log(error,"Got an error while initializing data socket.");
          error_state_ = true;
        }
      }

      if (!reader_->get_ready()) {
        Log(warning,"Received call to start transmitting but reader is not ready. Refusing to run." );
        json obj;
        obj["type"] = "error";
        obj["message"] = "PTB data reader is not ready yet.Refusing to run";
        feedback_.push_back(obj);
        error_state_ = true;
        return;
      }
      reader_->start_data_taking();
    }
    catch(SocketException &e) {
      json obj;
      obj["type"] = "error";
      std::string msg = "PTB data socket exception (socket): ";
      msg += e.what();
      obj["message"] = msg;
      feedback_.push_back(obj);
      error_state_ = true;
      return;
    }
    catch(std::exception &e) {
      json obj;
      obj["type"] = "error";
      std::string msg = "PTB data socket exception (std): ";
      msg += e.what();
      obj["message"] = msg;
      feedback_.push_back(obj);
      error_state_ = true;
      return;
    }
    catch (...) {
      Log(error,"Unknown error starting reader thread");
      json obj;
      obj["type"] = "error";
      obj["message"] = "PTB data socket exception (unknown): error starting reader";
      feedback_.push_back(obj);
      error_state_ = true;
      return;
    }

    /// -- Sleep a few ms to check for errors from the reader
    ///
    usleep(3000);
    bool has_error = false;
    json reader_msgs;
    reader_->get_feedback(has_error,reader_msgs,true);
    if (!reader_msgs.empty()) {
      feedback_.insert(std::end(feedback_),reader_msgs.begin(),reader_msgs.end());
    }
    if (has_error) {
      Log(error,"Received error state from the board reader");
      error_state_ = true;
      return;
    }

    // The GLB_EN is located in bin 31 of register 30
    set_enable_bit(true);

    Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[0].value() );

    board_state_ = RUNNING;

    Log(info,"Run Start requested...");

    json obj;
    obj["type"] = "info";
    obj["message"] = "Success starting the run";
    feedback_.push_back(obj);
  }

  // -- Writes everything to the feedback
  void board_manager::stop_run() {
    Log(debug,"Stopping the run" );

    // Check is the run is running

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

    usleep(3000);
    bool has_error = false;
    json reader_msgs;
    reader_->get_feedback(has_error,reader_msgs,true);
    if (!reader_msgs.empty()) {
      Log(debug,"Received %u messages from the reader",reader_msgs.size());
      feedback_.insert(std::end(feedback_),reader_msgs.begin(),reader_msgs.end());
    }
    if (has_error) {
      Log(error,"Received an error state from the board reader");
    }

    // Build a statistics object
    if (get_board_state() == board_manager::RUNNING) {
      json stat;
      stat["type"] = "statistics";
      stat["num_eth_packets"] = reader_->get_n_sent_frags();
      stat["num_eth_bytes"] = reader_->get_sent_bytes();
      stat["num_word_counter"] = reader_->get_n_status();
      stat["num_hlt"] = reader_->get_n_gtriggers();
      stat["num_llt"] = reader_->get_n_ltriggers();
      stat["num_tstamp"] = reader_->get_n_timestamps();
      stat["num_fifo_warn"] = reader_->get_n_warns();
      reader_->reset_counters();
      feedback_.push_back(stat);
      Log(info,"End of run message: %s",stat.dump(2).c_str());
    }
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
      json obj;
      obj["type"] = "warning";
      obj["message"] = "Failed to stop run on hardware. No ACK received.";
      feedback_.push_back(obj);
    } else {
      Log(info,"Run stopped");
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
    if (board_state_ == RUNNING) {
      stop_run();
    }
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


  void board_manager::process_config(json &doc,json &answers) {
    feedback_.clear();

    if (board_state_ == RUNNING) {
      json obj;
      obj["type"] = "warning";
      obj["message"] = "Attempted to pass a new configuration during a run. Ignoring the new configuration";
      answers.push_back(obj);
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
      json obj;
      obj["type"] = "warning";
      obj["message"] = "Configuration still in development. Only a few configuration registers are being set";
      answers.push_back(obj);

    }
    // -- Only catch something deriving from std::exception and unspecified
    catch(std::exception &e) {

      std::string msg = "Error processing configuration: ";
      msg += e.what();
      msg += ". Not committing configuration to PTB.";
      json obj;
      obj["type"] = "error";
      obj["message"] = msg;
      answers.push_back(obj);
      has_error = true;
    }
    catch(...) {
      json obj;
      obj["type"] = "error";
      obj["message"] = "Unknown error processing configuration. Not committing configuration to PTB";
      answers.push_back(obj);
      has_error = true;
    }
    // Check if we got an error. If so, do not commit.
    if (has_error) {
      // Don't commit. Just go back and throw the error.
      return;
    }

    if (!reader_) {
      Log(warning,"Don't have a valid PTBReader instance. Attempting to create a new one." );
      json obj;
      obj["type"] = "warning";
      obj["message"] = "Don't have a valid PTBReader instance. Attempting to create a new one";
      answers.push_back(obj);
      reader_ = new board_reader();
    }

    // -- First grab the server information
    json receiver = doc.at("ctb").at("sockets").at("receiver");
    data_socket_host_ = receiver.at("host").get<std::string>();
    reader_->set_tcp_host(data_socket_host_);
    data_socket_port_ = receiver.at("port").get<unsigned short>();
    reader_->set_tcp_port(data_socket_port_);
    Log(debug,"Setting data transmission channel to [%s:%hu]",data_socket_host_.c_str(),data_socket_port_);
    // -- Grab the subsystem configurations
    json beamconf = doc.at("ctb").at("subsystems").at("beam");
    json crtconf = doc.at("ctb").at("subsystems").at("crt");
    json pdsconf = doc.at("ctb").at("subsystems").at("pds");

    // uint32_t duration;
    std::stringstream strVal;

    json rtrigger = doc.at("ctb").at("randomtrigger");
    bool rtrigger_en = rtrigger.at("enable").get<bool>();
    uint32_t rtriggerfreq = rtrigger.at("frequency").get<unsigned int>();
    Log(debug,"Random Trigger Frequency [%d] (%u) [0x%X][%s]",rtriggerfreq,rtriggerfreq,rtriggerfreq, std::bitset<26>(rtriggerfreq).to_string().c_str());
    if (rtriggerfreq >= (1<<26)) {
      Log(warning,"Input value of [%u] above maximum rollover [26]. Truncating to maximum.",rtriggerfreq);
      rtriggerfreq = (1<<26)-1;
    }
    set_bit(27,0,rtrigger_en);
    set_bit_range_register(25,0,26,rtriggerfreq);
    //Set pulser frequency
    json pulserconf = doc.at("ctb").at("pulser");
    bool pulser_en = pulserconf.at("enable").get<bool>();
    uint32_t pulserfreq = pulserconf.at("frequency").get<unsigned int>();
    Log(debug,"Pulser Frequency [%d] (%u) [0x%X][%s]",pulserfreq,pulserfreq,pulserfreq, std::bitset<26>(pulserfreq).to_string().c_str());
    if (pulserfreq >= (1<<26)) {
      Log(warning,"Input value of [%u] above maximum rollover [26]. Truncating to maximum.",pulserfreq);
      pulserfreq = (1<<26)-1;
    }
    set_bit(26,31,pulser_en);
    set_bit_range_register(26,0,26,pulserfreq);

    uint32_t duration = receiver.at("rollover").get<unsigned int>();
    // Microslice duration is now a full number...check if it fits into 27 bits
    Log(debug,"MicroSlice Duration [%d] (%u) [0x%X][%s]",duration,duration,duration, std::bitset<29>(duration).to_string().c_str());
    if (duration >= (1<<29)) {
      //    answers.push_back("WARNING:Input value of [" << duration << "] above the allowed limit of 27 bits. Setting to maximum allowed.";
      Log(warning,"Input value of [%u] above maximum rollover [27]. Truncating to maximum.",duration);
      json obj;
      obj["type"] = "warning";
      std::ostringstream msg;
      msg << "Rollover input value for of [" << duration << "] above maximum rollover [29]. Truncating to maximum";

      obj["message"] = msg.str();
      answers.push_back(obj);

      duration = (1<<29)-1;
    }
    set_bit_range_register(6,0,29,duration);

    Log(debug,"Register 6 : [0x%08X]", register_map_[6].value() );
    strVal.clear();

    //FIXME: Introduce json parse feedback here. 
    // NFB: We should NEVER, NEVER, NEVER parse a file without properly caught exceptions
    // Otherwise we have no way to know if the configuration fails
    beam_config(beamconf);
    crt_config(crtconf);

#ifdef NO_PDS_DAC
    Log(warning,"PDS configuration block was disabled. Not configuring any PDS input");
#else


    strVal.str("");
    json feedback;
    pds_config(pdsconf,feedback);
    if (!feedback.empty()) {
      Log(debug,"Received %u messages from configuring the PDS",feedback.size());
      answers.insert(std::end(answers),feedback.begin(),feedback.end());
    }

#endif

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
      std::ostringstream msgs_;
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
      json obj;
      obj["type"] = "error";
      obj["message"] = msgs_.str();
      answers.push_back(obj);
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

      json obj;
      obj["type"] = "info";
      obj["message"] = "Successful board configuration";
      answers.push_back(obj);

    }
    // Most likely the connection will fail at this point. Not a big problem.
    //    Log(verbose,"Returning from SetConfig with answer [%s]",feedback.c_str());

  }

  void board_manager::pds_config(json &pdsconfig, json& feedback){

    ///FIXME: How can this work? dacsetup is not initialized in this function
    i2conf* dacsetup;

    std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
    std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
    std::string s_trigtype0 = pdsconfig.at("triggers").at(0).at("type").get<std::string>();
    std::string s_count0 = pdsconfig.at("triggers").at(0).at("count").get<std::string>();
    bool llt11_enable = pdsconfig.at("triggers").at(0).at("enable").get<bool>();

    uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
    uint8_t trigtype0 = (int)strtol(s_trigtype0.c_str(),NULL,0);
    uint8_t count0 = (int)strtol(s_count0.c_str(),NULL,0);

    // std::vector<uint32_t> dac_values (24, 0);

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
    if (dacsetup->ConfigureDacs(dac_values,false)) {
      Log(error,"Failed to write configuration values to DACs.");
      json obj;
      obj["type"] = "error";
      obj["message"] = "Failed to write configuration values to DACs";
      feedback.push_back(obj);
    }
    Log(info,"Programmed %zu DAC channels", dac_values.size());

    //Input channel masks
    set_bit_range_register(2,0,24,channelmask);

    //Configure counting trigger 0
    uint32_t trig0 = (trigtype0<<5) + count0;
    set_bit_range_register(38,0,9,trig0);
    set_bit(27,11,llt11_enable);

  }


  void board_manager::crt_config(json &crtconfig){

    // std::vector<uint32_t> dac_values = crtconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
    std::string s_channelmask = crtconfig.at("channel_mask").get<std::string>();
    //std::string s_trigtype0 = crtconfig.at("triggers").at(0).at("type").get<std::string>();
    //std::string s_count0 = crtconfig.at("triggers").at(0).at("count").get<std::string>();

    uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
    // uint8_t trigtype0 = (int)strtol(s_trigtype0.c_str(),NULL,0);
    // uint8_t count0 = (int)strtol(s_count0.c_str(),NULL,0);
    //Input channel masks
    set_bit_range_register(1,0,32,channelmask);
  }

  void board_manager::beam_config(json &beamconfig){

    // std::vector<uint32_t> dac_values = beamconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
    std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
    // std::string s_trigtype0 = beamconfig.at("triggers").at(0).at("type").get<std::string>();
    // std::string s_count0 = beamconfig.at("triggers").at(0).at("count").get<std::string>();

    uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
    // uint8_t trigtype0 = (int)strtol(s_trigtype0.c_str(),NULL,0);
    //  uint8_t count0 = (int)strtol(s_count0.c_str(),NULL,0);

    //Input channel masks
    set_bit_range_register(3,0,9,channelmask);}

}
