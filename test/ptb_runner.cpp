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

using namespace std;



int main() {
  Logger::SetSeverity(Logger::verbose);

  // This doesn't work since the thread gets stuck in the constructor of ConfigServer
  // Need to make the acceptance of the client into a separate thread.

  ConfigServer*cfg = ConfigServer::get();

  //Log(debug) << "Going to sleep" <<endlog;
  //std::this_thread::sleep_for (std::chrono::seconds(10));
  Log(info,"Starting manager");
  bool emulate = true;
  // Start in emulating mode.
  PTBManager manager(emulate);
  manager.DumpConfigurationRegisters();

  // Get the reader
  const PTBReader* reader = manager.getReader();
  Log(info,"Showing the reader: %X",reader);
  //manager.StartRun();



  pthread_join(cfg->getThreadId(),NULL);
  return 0;
}

