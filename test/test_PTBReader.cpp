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
// #include <thread>         // std::this_thread::sleep_for
// #include <chrono>         // std::chrono::seconds
#include "PracticalSocket.h"
extern "C" {
#include <pthread.h>         // For POSIX threads
};

using namespace std;

void* controller(void *arg) {


  // Create a client that connects to the PTB
  try{

//   const PTBReader *reader = (const PTBReader *) arg;
//   while (!reader->isReady()) {
//     ;
//   }
//   cout << "Reader is ready... let's start the show!!!" << endlog;


  // Create a tcp server and listen to the connection
  TCPServerSocket servSock(8992);
  // Accept a client
  cout << "Entering waiting state" << endl;
  TCPSocket *sock = servSock.accept();
  std::cout << "--> Received a client request." << endlog;


  TCPSocket socksrv("localhost", 8991);

  // Tell to start the run
  const char *start_run = "<command>StartRun</command>";
  socksrv.send(start_run,27);
  char answer[1000];
  socksrv.recv(answer,1000);
  cout << "Got answer " << answer << endl;
  // Keep looping to read data and dump it
  uint32_t header;
  uint32_t body_counter[3];
  uint32_t body_trigger;
  uint32_t timestamp[2];
  uint32_t checksum;
  uint16_t size;
  uint8_t wtype;
  uint32_t *buffer = NULL;
  for (uint32_t i = 0; i < 10; ++i) {
    // First listen for a header
    sock->recv(&header, sizeof(header));
    //std::cout << "Header : " << std::hex << header << std::endl;
    printf("HEADER %x %u\n",header,header);
    // Catch the size
    size = header & 0xFFFF;
    printf("SIZE %x %u\n",(uint32_t)size,(uint32_t)size);

    // Allocate a buffer to grab the whole thing from the ethernet
    //buffer = (uint32_t*) malloc(size);

    // Grab the whole thing
    //sock->recv(buffer, size);

    // Now parse it properly.
    uint16_t i_size = (uint16_t)sizeof(uint32_t);
    while (i_size < size) {
      sock->recv(&header, sizeof(header));
      i_size += (uint16_t)sizeof(header);
      printf("PHEADER : %x \n",header);
      //std::cout << "Header : " << std::hex << header << std::dec << std::bitset<32>(header) << std::endl;
      wtype = ((header >> 29) & 0x7);
      printf("WTYPE %x %u\n",(uint32_t)wtype,(uint32_t)wtype);
      if (wtype == 0x1) { // counter word
        //pick up a counter word
        sock->recv(body_counter, sizeof(body_counter));
        printf("COUNTER   : %x %x %x \n",body_counter[0],body_counter[1],body_counter[2]);
        i_size += sizeof(body_counter);
      } else if (wtype == 0x2) { // parse a trigger word
        sock->recv(&body_trigger, sizeof(body_trigger));
        printf("TRIGGER   : %x \n",body_trigger);
        i_size += sizeof(body_trigger);
      } else if (wtype == 0x7) { // It is a timestamp
        sock->recv(timestamp, sizeof(timestamp));
        printf("TIMESTAMP : %x %x \n",timestamp[0],timestamp[1]);
        i_size += sizeof(timestamp);
      } else if (wtype == 0x4) { // checksum
        sock->recv(&checksum, sizeof(checksum));
        printf("CHECKSUM : %x \n",checksum);
        i_size += sizeof(checksum);
      }
    }
    std::cout << "== Packet printed." << endl;
  }
  printf("Sending STOP RUN\n");
  const char *stop_run = "<command>StopRun</command>";
  socksrv.send(stop_run,26);
  socksrv.recv(answer,1000);
  cout << "Got answer " << answer << endl;
  }
  catch(SocketException &e) {
    cout << "Socket exception caught." << endl;
    cout << e.what() << endl;
    ::abort();
  }
}

// void* shutdown(void *arg) {

//   for (int i = 0; i < 3; ++i) {
//     std::this_thread::sleep_for (std::chrono::seconds(1));
//     Log(info) << i+1 << " seconds elapsed." << endlog;
//   }

//   ConfigServer *cfg = (ConfigServer*)arg;
//   cfg->Shutdown();
//   return NULL;
// }

int main() {
  Logger::SetSeverity(Logger::verbose);
  Log(info) << "Just a test" << endlog;

  // This doesn't work since the thread gets stuck in the constructor of ConfigServer
  // Need to make the acceptance of the client into a separate thread.

  std::cout << "Starting the reader thread..." << endl;
  // -- Create a board reader facsimile
  pthread_t threadID_controller;
  if (pthread_create(&threadID_controller, NULL, &(controller),(void*)NULL/*reader*/) != 0) {
      Log(fatal) << "Unable to create master thread." << endlog;
    }

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
  //manager.StartRun();

//  while (!reader->isReady()) {
//    ;
//  }



  pthread_join(threadID_controller,NULL);

//  Log(info) << "Waiting for thread [" << cfg->getThreadId() << "] to finish." << endlog;
//  Log(info) << cfg->getThreadId()->__sig << endlog;
//  Log(info) << "Adding a new thread to shutdown eventually..." << endlog;
//  pthread_t threadID;
//  if (pthread_create(&threadID, NULL, &(shutdown),(void*)cfg) != 0) {
//      Log(fatal) << "Unable to create master thread." << endlog;
//    }
//  pthread_join(cfg->getThreadId(),NULL);
  return 0;
}

