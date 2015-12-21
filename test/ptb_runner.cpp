/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"
#include "ConfigServer.h"
#include "PTBManager.h"
#include "PTBReader.h"
#include <bitset>
#include "PracticalSocket.h"
extern "C" {
#include <pthread.h>         // For POSIX threads
};
#include <csignal>
#include <thread>

using namespace std;

//FIXME: Add a manager for all the threads that are running

void handler(int signal) {
  if (signal == SIGINT) {
    Log(warning,"Found a SIGINT request. Attempting a clean stop.");
    ConfigServer*cfg = ConfigServer::get();
    cfg->Shutdown(false);
    cfg->CheckInstances();
    Log(info,"Cleaning the config server itself");
    //pthread_join(cfg->getThreadId(),NULL);
    cfg = NULL;
    //delete cfg;
    exit(0);
  }
}


void run() {

    ConfigServer*cfg = ConfigServer::get();

    //Log(debug) << "Going to sleep" <<endlog;
    //std::this_thread::sleep_for (std::chrono::seconds(10));
    Log(info,"Starting manager");
    bool emulate = false;
    // Start in emulating mode.
    PTBManager manager(emulate);
    manager.DumpConfigurationRegisters();

    // Get the reader
    const PTBReader* reader = manager.getReader();
    Log(info,"Showing the reader: %X",reader);
    //manager.StartRun();


    if (cfg->getThreadId()) {
      Log(info,"Waiting for thread %d",cfg->getThreadId());
      pthread_join(cfg->getThreadId(),NULL);
    }
}

int main() {
  Logger::SetSeverity(Logger::verbose);
  // Logger::SetSeverity(Logger::debug);
  // Logger::SetSeverity(Logger::info);
  // Logger::SetSeverity(Logger::warning);
  // Logger::SetSeverity(Logger::error);
  
  bool relaunch = true;
  while(relaunch) {
    try{
      run();
    } catch(...) {
      // Nope. This is a terrible idea. What you want to do is wait for a few seconds and relaunch
      //return -1;
      Log(warning,"An exception was caught. Waiting for 10 s and relaunching.");
      std::this_thread::sleep_for (std::chrono::seconds(10));
      relaunch = true;
    }
  }

  // Register a signal handler
  std::signal(SIGINT,handler);
  return 0;
}

