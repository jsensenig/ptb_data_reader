/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

#include "../src/boardmanager.h"
#include "../src/boardserver.h"

extern "C" {
#include <pthread.h>         // For POSIX threads
}

void* shutdown(void *arg) {

   for (int i = 0; i < 25; ++i) {
     std::this_thread::sleep_for (std::chrono::seconds(1));
     Log(info,"%d seconds elapsed.",i+1);
   }

   socket_server *cfg = (socket_server*)arg;
   cfg->shutdown();
  return NULL;
}

int main() {
  Logger::SetSeverity(Logger::verbose);
   Log(info,"Just a test");

   // This doesn't work since the thread gets stuck in the constructor of ConfigServer
   // Need to make the acceptance of the client into a separate thread.

   socket_server*cfg = socket_server::get();

   Log(debug,"Going to sleep");
   std::this_thread::sleep_for (std::chrono::seconds(10));
   Log(info,"Starting manager");
   board_manager manager;
   Log(info,"Waiting for thread [%x] to finish.",cfg->getThreadId());
   Log(info,"Adding a new thread to shutdown eventually...");
   pthread_t threadID;
   if (pthread_create(&threadID, NULL, &(shutdown),(void*)cfg) != 0) {
       Log(fatal,"Unable to create master thread.");
     }
   pthread_join(cfg->getThreadId(),NULL);
   return 0;
}

