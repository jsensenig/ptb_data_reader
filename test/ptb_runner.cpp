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

int main() {
  Logger::SetSeverity(Logger::verbose);
  //Logger::SetSeverity(Logger::warning);

  // Register a signal handler
  std::signal(SIGINT,handler);
  // This doesn't work since the thread gets stuck in the constructor of ConfigServer
  // Need to make the acceptance of the client into a separate thread.
  try {
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
  catch(...) {
    return -1;
  }
  return 0;
}

