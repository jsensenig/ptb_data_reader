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
PTBManager::PTBManager() : reader_(), cfg_srv_(0) {
  Log(verbose) << "Setting up pointers." << endlog;
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cmd_disp_ = CommandDispatcher::get();
  cmd_disp_->RegisterDataManager(this);
}



PTBManager::~PTBManager() {
  Log(verbose) << "Destroying object." << endlog;
}


