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

#include "boost/date_time/posix_time/posix_time.hpp"

#if defined(PDUNE_COMPILATION)
#include "ctb_config_funcs.hpp"
#elif defined(SBND_COMPILATION)
#include "ptbmk2_config_funcs.hpp"
#else
#error "Unknown compilation mode. Check config.h file in repository head"
#endif

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
		            mapped_gpio_base_addr_(nullptr),
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
    commands_.insert(std::map<std::string, command>::value_type("CAENReset",CAENRESET));
    // Setup the registers first, to make sure that if the data
    // manager receives something we are ready to deliver.
    setup_registers();


    // -- Force a hard reset.
    // This gives some degree of protection in case of catastrophic
    // failure. In particular protects against
    // the software thinking that the board is not running, and the board thinking otherwise

    Log(info,"Applying a hard reset to make sure that the board is not running rogue.");

    // A hard reset should sent the whole thing to zeros
    // Including clearing up the configuration. A new configuration will have to be reissued
    set_enable_bit(false);
    set_config_bit(false);
    // the hard reset also includes a complete reset of the local buffers
    set_reset_bit(true);
    // Sleep for 100 microseconds to make sure that reset has taken place
    std::this_thread::sleep_for (std::chrono::microseconds(100));
    set_reset_bit(false);

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
      case CAENRESET:   
        
#if defined(SBND_COMPILATION)
          Log(debug,"Issuing a reset to the CAEN VME crate");
          caen_reset(); 
#elif defined(PDUNE_COMPILATION)
          Log(warning, "Undefined operation for ProtoDUNE");        
          error_state_ = true;
#else
  #error "Unknown compilation mode. Check config.h file in repository head"
#endif
 
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
        obj["message"] = "PTB data reader is not ready yet. Refusing to run";
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
      Log(error,"%s",msg.c_str());
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
      Log(error,"%s",msg.c_str());
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
      Log(error,"PTB data socket exception (unknown): error starting reader");
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

    // The GLB_EN is located in bin 31 of register 0
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

    //const int num_cmd = 4;
    const int offset_reg = 81;
    std::vector<unsigned int> cmd_codes = {12, 13, 14, 15};
    // Build a statistics object
    if (get_board_state() == board_manager::RUNNING) {
     // uint32_t evtctr = register_map_[65].value();
      std::ostringstream evts; 
      evts << "num_events: " << register_map_[92].value();
      json stat;
      stat["type"] = "statistics";
      stat["message"] = evts.str();
      stat["num_eth_packets"] = reader_->get_n_sent_frags();
      stat["num_eth_bytes"] = reader_->get_sent_bytes();
      stat["num_chstatus"] = reader_->get_n_status();
      stat["num_hlt"] = reader_->get_n_gtriggers();
      stat["num_llt"] = reader_->get_n_ltriggers();
      stat["num_tstamp"] = reader_->get_n_timestamps();
      stat["num_feedback"] = reader_->get_n_feedback();
     
      stat["num_0xC"] = register_map_[12+offset_reg].value();     
      stat["num_0xD"] = register_map_[13+offset_reg].value();     
      stat["num_0xE"] = register_map_[14+offset_reg].value();     
      stat["num_0xF"] = register_map_[15+offset_reg].value();     

/*
      std::vector<std::string> keys(num_cmd);
      std::vector<std::stringstream> ssk(num_cmd);
      for(unsigned int i=0; i< num_cmd; ++i) {
	string base = "num_0x";
        ssk.at(i) << std::hex << cmd_codes.at(i);
	keys.at(i) = base + ssk.at(i).str();
        stat[keys.at(i)] = cmd_codes[i];
        stat["num_cmd_issued"] = register_map_[cmd_codes.at(i)+offset_reg].value(); 
      } 
*/    
      stat["ack_trg_ctr"] = register_map_[97].value();
 
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

  void board_manager::caen_reset() {

    if (get_board_state() == board_manager::RUNNING) {
        Log(warning, "Cannot issue CAEN reset during a run.");
        json obj;
        obj["type"] = "warning";
        obj["message"] = "Failed to issue reset to CAEN. Board is in RUNNING state";
        feedback_.push_back(obj);
    } else {
      set_caen_reset_bit(true);
      Log(debug,"VME_reset requested. Register: 0x%08x ",register_map_[0].value() );
      Log(info, "Issued reset to CAENs");
      usleep(2000); //hold reset for ~2ms
      set_caen_reset_bit(false);
    }

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

    //Follow the same process to map the GPIO address
    ptb::config::setup_ctb_gpio();
    // First get the virtual address for the mapped physical address
    Log(debug,"Mapping GPIO physical address [0x%X 0x%X]",ptb::config::gpio_reg.base_addr,ptb::config::gpio_reg.high_addr);
    mapped_gpio_base_addr_ = ptb::util::map_physical_memory(ptb::config::gpio_reg.base_addr,ptb::config::gpio_reg.high_addr);
                                                                                                                                         
    Log(debug,"Received virtual address for GPIO : 0x%08X\n",reinterpret_cast<uint32_t>(mapped_gpio_base_addr_));
    // Cross check that we have at least as many offsets as GPIO registers expected
    if (ptb::config::gpio_reg.n_registers < num_gpio_reg_) {
      Log(warning,"Have less configured GPIO registers than the ones required. (%u != %u)",ptb::config::gpio_reg.n_registers, num_gpio_reg_);
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
    // clear all messages that existed from before (since last stop?)
    feedback_.clear();
    //Keep this greeting, otherwise there will be an error when trying to insert into the empty "answers" object
    json obj;
    obj["type"] = "info";
    obj["message"] = "Beginning CTB specific configuration! ";
    answers.push_back(obj);

    if ((board_state_ == RUNNING) || get_enable_bit_ACK()) {
      json obj;
      obj["type"] = "warning";
      obj["message"] = "Attempted to pass a new configuration during a run. Ignoring the new configuration";
      answers.push_back(obj);
      Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
      //feedback = msgs_.str();
      return;
    }
    bool has_error = false;

    // -- NFB --
    // Start by sending a reset into the board, to clear out the state
    Log(info,"Applying a reset prior to the configuration.");
    set_reset_bit(true);
    std::this_thread::sleep_for (std::chrono::microseconds(10));
    set_reset_bit(false);


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

    // uint32_t duration;
    std::stringstream strVal;

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

//////////////////////////////////////
//Add preprocessor block below for CTB/PTB config use

#if defined(PDUNE_COMPILATION)

    //Program timing endpoint addr and group first using GPIO so the endpoint is ready when the config is committed
    json timingconf = doc.at("ctb").at("misc").at("timing");
    std::string s_t_addr = timingconf.at("address").get<std::string>();
    std::string s_t_group = timingconf.at("group").get<std::string>();
    uint32_t t_addr = (uint32_t)strtoul(s_t_addr.c_str(),NULL,0);
    uint32_t t_group = (uint32_t)strtoul(s_t_group.c_str(),NULL,0);
    Log(info,
        "Setting timing endpoint with address [%s] (0x%X) and group [%s] (0x%X)",
        s_t_addr.c_str(),
        t_addr,
        s_t_group.c_str(),
        t_group);
    uint32_t timing_addr = (t_group<<15) + (t_addr<<7);
    //FIXME: Careful with this kind of assignment. If there is anything in that register, it
    // will be cleared out. For example, anything in other bit ranges will be wiped clean

    util::Xil_Out32((uint32_t)(mapped_gpio_base_addr_ + ptb::config::gpio_reg.addr_offset[0]), timing_addr);

    // FIXME: There is a flaw here
    // If there was an error configuring the specific code, how do we know?
    json feedbacks;
    bool got_error = false;
    configure_ctb(doc, feedbacks,got_error);
    if (!feedbacks.empty()) {
      Log(warning,"Received %u messages from CTB configuration [error : %d]",feedbacks.size(),((got_error)?1:0));
      Log(warning,"Messages : %s",answers.dump(2).c_str());
      Log(warning,"Feedback : %s",feedbacks.dump(2).c_str());
      answers.insert(std::end(answers),feedbacks.begin(),feedbacks.end());
    }
    Log(warning,"Passed over...");
    if (got_error) has_error = true;

    //Sleep for a bit to allow the timing endpoint to cycle through it's state machine
    // --> Resetting the endpoint messes up the timing server, skip the reset
    //usleep(900000);
    //Read the timing status 
    uint32_t timing_reg = 91; //reg_out_1[31:28]
    uint32_t timing_stat = register_map_[timing_reg].value();

    Log(debug,"Received %X timing status:  and PLL LoL: %X ",(timing_stat >> 28), ((timing_stat >> 25) & 0x1));
    std::ostringstream msg;
    msg << "Recieved timing status: " << (timing_stat >> 28)
        << " [0x" << std::hex <<  (timing_stat >> 28) << std::dec
        << "] and PLL LoL: " << ((timing_stat >> 25) & 0x1) << " [0x"
        << std::hex << ((timing_stat >> 25) & 0x1) << std::dec << "]";
    json obj1;
    obj1["type"] = "info";
    obj1["message"] = msg.str().c_str();
    answers.push_back(obj1);

#if !defined(STANDALONE_TIMING)
    if ((timing_stat >> 28) != 0x8) {
      json obj;
      std::ostringstream msg2;
      msg2 << "Timing not in a good state (0x8)! Present status : " << (timing_stat >> 28)
          << " [0x" << std::hex << (timing_stat >> 28) << std::dec << "]";
      obj["type"] = "error";
      obj["message"] = msg2.str();
      Log(error,"%s",msg2.str().c_str());
      answers.push_back(obj);
      has_error = true;
    }
#endif

#elif defined(SBND_COMPILATION)

    //Used to determine the global time
    using namespace boost::gregorian;
    using namespace boost::posix_time;

    ptime t_now(second_clock::universal_time());
    ptime time_t_epoch(date(1970,1,1));
    time_duration diff = t_now - time_t_epoch;
    uint32_t t_offset_s = diff.total_seconds();
    Log(debug,"Input offset time = %s seconds since the epoch", std::bitset<32>(t_offset_s).to_string().c_str());

    set_bit_range_register(7,0,32,t_offset_s);
    Log(debug,"Register 7: [0x%8x]", register_map_[7].value() );


    json feedback;
    configure_ptbmk2(doc, feedback);
    if (!feedback.empty()) {
      Log(debug,"Received %u messages from configuring the PTB",feedback.size());
      answers.insert(std::end(answers),feedback.begin(),feedback.end());
    }

#else
#error "Unknown compilation mode. Check config.h"
#endif
    // -- Once the configuration is set, dump locally the status of the config registers
    dump_config_registers();

    // Set the bit to commit the configuration
    // into the hardware (bit 29 in register 30)
    Log(debug,"Committing configuration to the hardware.");
    Log(verbose,"Control register before config commit 0x%X", register_map_[0].value() );
    set_config_bit(true);
    Log(debug,"Control register after config commit 0x%08X", register_map_[0].value() );
    if (!get_config_bit_ACK() || has_error) {

      // If the commit is not ACK then effectively the board is not configured.
      // The best is to reset everything and fail the configuration.
      std::ostringstream msgs_;
      if (has_error) {
        msgs_ << "Failed to process configuration fragment. Resetting config. ";
        Log(error,"Error caught processing fragment");

      } else {
        msgs_ << "Failed to set configuration bit. ACK not received. ";
        Log(error,"Failed to set configuration bit. ACK not received. Control register %X",register_map_.at(0).value());
      }
      msgs_ << " Control register : [0x"
          << std::hex << register_map_.at(0).value() << std::dec
          << "]. Control register after reset : [0x";

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

      msgs_ << std::hex << register_map_.at(0).value() << std::dec << "]";
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
      Log(info,"Successful board configuration");
    } else {
      // -- there are errors. Create a message at the end
      Log(error,"Errors found while configuring the board. See preceding messages for details.");

    }

  }

//-----------------------------------------
// -- Moved configs to ctb_config_funcs.hpp and ptbmk2_config_funcs.hpp

}
