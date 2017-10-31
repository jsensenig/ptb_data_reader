/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"
#include <bitset>

#include "../src/boardmanager.h"
#include "../src/boardreader.h"
#include "../src/boardserver.h"
#include "PracticalSocket.h"
extern "C" {
#include <pthread.h>         // For POSIX threads
}
#include <csignal>
#include <thread>
#include <chrono>

using namespace std;

bool g_relaunch;
//FIXME: Add a manager for all the threads that are running

void handler(int signal) {
  if (signal == SIGINT) {
    Log(warning,"Received a SIGINT request. Attempting a clean stop.");
    g_relaunch = false;
    socket_server*cfg = socket_server::get();
    cfg->shutdown(false);
    cfg->get_num_instances();
    Log(info,"Cleaning the config server itself");
    //pthread_join(cfg->getThreadId(),NULL);
    cfg = NULL;
    //delete cfg;
    exit(0);
  }
}


void run() {

    socket_server*cfg = socket_server::get();

    //Log(debug) << "Going to sleep" <<endlog;
    //std::this_thread::sleep_for (std::chrono::seconds(10));
    Log(info,"Starting manager");
    // Start in emulating mode.
    board_manager manager;
    manager.dump_config_registers();

    // Get the reader
    const board_reader* reader = manager.getReader();
    Log(info,"Showing the reader: %X",reader);
    //manager.StartRun();


    if (cfg->getThreadId()) {
      Log(info,"Waiting for thread %d",cfg->getThreadId());
      pthread_join(cfg->getThreadId(),NULL);
    }
}

int main() {
  // Register a signal handler
  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT,handler);
  g_relaunch = true;
  // Logger::SetSeverity(Logger::verbose);
  Logger::SetSeverity(Logger::debug);
  //Logger::SetSeverity(Logger::info);
  // Logger::SetSeverity(Logger::warning);
  // Logger::SetSeverity(Logger::error);
  
  while(g_relaunch) {
    try{
      run();
    }
    catch(std::exception &e) {
      Log(error,"STD exception caught: %s",e.what());
    }
    catch(...) {
      // Nope. This is a terrible idea. What you want to do is wait for a few seconds and relaunch
      //return -1;
      Log(warning,"An exception was caught. Waiting for 10 s and relaunching.");
      std::this_thread::sleep_for (std::chrono::seconds(10));
      g_relaunch = true;
    }
  }

  return 0;
}

