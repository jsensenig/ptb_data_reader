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

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

extern "C" {
#include <pthread.h>         // For POSIX threads
};

void* shutdown(void *arg) {

  for (int i = 0; i < 3; ++i) {
    std::this_thread::sleep_for (std::chrono::seconds(1));
    Log(info) << i+1 << " seconds elapsed." << endlog;
  }

  ConfigServer *cfg = (ConfigServer*)arg;
  cfg->Shutdown();
  return NULL;
}

int main() {
  Logger::SetSeverity(Logger::verbose);
  Log(info) << "Just a test" << endlog;

  // This doesn't work since the thread gets stuck in the constructor of ConfigServer
  // Need to make the acceptance of the client into a separate thread.

  ConfigServer*cfg = ConfigServer::get();

  //Log(debug) << "Going to sleep" <<endlog;
  //std::this_thread::sleep_for (std::chrono::seconds(10));
  Log(info) << "Starting manager" << endlog;
  bool emulate = true;
  // Start in emulating mode.
  PTBManager manager(emulate);
  manager.DumpConfigurationRegisters();

  // Get the reader
  const PTBReader* reader = manager.getReader();
  Log(info) << "Showing the reader: " << std::hex << reader << std::dec << endlog;
  manager.StartRun();

  Log(info) << "Waiting for thread [" << cfg->getThreadId() << "] to finish." << endlog;
  Log(info) << cfg->getThreadId()->__sig << endlog;
  Log(info) << "Adding a new thread to shutdown eventually..." << endlog;
  pthread_t threadID;
  if (pthread_create(&threadID, NULL, &(shutdown),(void*)cfg) != 0) {
      Log(fatal) << "Unable to create master thread." << endlog;
    }
  pthread_join(cfg->getThreadId(),NULL);
  return 0;
}

