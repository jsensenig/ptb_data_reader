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

// -- PugiXML includes


// Init with a new reader attached
PTBManager::PTBManager() : reader_(), cfg_srv_(0),status_(IDLE) {
  Log(verbose) << "Setting up pointers." << endlog;

  // Register the available commands
  commands_.insert(std::map<std::string, Command>::value_type("RUNSTART",RUNSTART));
  commands_.insert(std::map<std::string, Command>::value_type("RUNSTOP",RUNSTOP));

  // This had to occur after setting up the command list or we might run into trouble.
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cfg_srv_->RegisterDataManager(this);

  SetupRegisters();
}



PTBManager::~PTBManager() {
  Log(verbose) << "Destroying object." << endlog;
  //FIXME: Implement memory deallocations to free the microZed.
}


void PTBManager::ExecuteCommand(const char* cmd) throw(std::exception) {
  try{

    Log(verbose) << "Received [" <<  cmd << "]" << endlog;
    std::map<std::string, Command>::iterator it;
    for (it = commands_.begin(); it != commands_.end(); ++it) {
      Log(verbose) << "Key : [" << it->first << "] : Value : [" << it->second << "]" << endlog;
      if (it->first == cmd) {
        Log(debug) << "It is the same" << endlog;
      }
    }

    Log(verbose) << "Going into the switch to try " <<  commands_.at(cmd) << endlog;
    switch (commands_.at(cmd)) {
    case RUNSTART:
      // Only starts a run if the manager is in IDLE state
      // Otherwise issue a warning
      if (getStatus() != PTBManager::IDLE) {
        Log(warning) << "A run is already running. Ignoring command" << endlog;
      } else {
        // Start the run
        Log(verbose) << "The Run should START now" << endlog;
        StartRun();
        break;
      }
      // -- Send the signal to start the run. Should be a register, I guess.
      break;
    case RUNSTOP:
      if (getStatus() != PTBManager::RUNNING) {
        Log(warning) << "Called for RUNSTOP but there is no run ongoing. Ignoring." << endlog;
        // -- Same thing.
        break;
      } else {
        Log(verbose) << "The Run should STOP now" << endlog;
        StopRun();
      }
    }
  }
  catch (const std::out_of_range &oor) {
    Log(error) << endlog << "Out of range error: " << oor.what() << endlog;
  }
  catch(std::exception &e) {
    Log(error) << "STL exception caught : " << e.what() << endlog;
  }
  catch(...) {
    Log(error) << "Unknown exception caught." << endlog;
  }
}

void PTBManager::StartRun() {
  Log(verbose) << "Starting the run" << endlog;
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to start the run
  status_ = RUNNING;
}

void PTBManager::StopRun() {
  Log(verbose) << "Stopping the run" << endlog;
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to stop the run. This should include:
  //        Propagate into the PTBReader to stop data collection
  //        Deallocate its streaming connection.
  status_ = IDLE;
}

void PTBManager::SetupRegisters() throw (std::exception) {
Log(debug) << "Setting up the appropriate registers to configure the board." << endlog;


}

void PTBManager::ProcessConfig(pugi::xml_node config) throw (std::exception) {
// Only accept a new configuration if we are not running.
  if (status_ == RUNNING) {
    Log(warning) << "Attempted to pass a new configuration during a run. Ignoring the new configuration." << endlog;
  }

  Log(verbose) << "Configuration should be parsed now." << endlog;

  // After parsing everything (and making sure that all the configuration is set)
  // Store the configuration locally
  config_ = config;
}
