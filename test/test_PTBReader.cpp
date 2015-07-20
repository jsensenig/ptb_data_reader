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
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <string>
#include <fstream>
#include "PracticalSocket.h"
extern "C" {
#include <pthread.h>         // For POSIX threads
};

using namespace std;

void control_thread() {
  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  Log(info,"#### Control thread: %s",stream.str().c_str());

  try{
  TCPSocket socksrv("localhost", 8991);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Send the configuration:
  std::ifstream config("./config/config_default.xml");
  // Read the whole file contents
  std::string config_contents((std::istreambuf_iterator<char>(config)), std::istreambuf_iterator<char>());
  //std::string config_contents;
  //config >> config_contents;
  config.close();

  char answer[1000];

  socksrv.send(config_contents.c_str(),config_contents.length());
  socksrv.recv(answer,1000);
  Log(info,"Got answer [%s]",answer);

  std::this_thread::sleep_for(std::chrono::seconds(5));
  // Tell to start the run
  Log(info,"Starting the run");
  const char *start_run = "<command>StartRun</command>";
  socksrv.send(start_run,27);
  socksrv.recv(answer,1000);
  Log(info,"Got answer [%s]",answer);


  // Sleep for 15 s and then stop the run
  std::this_thread::sleep_for(std::chrono::seconds(15));

  Log(warning,"Sending STOP RUN\n");
  const char *stop_run = "<command>StopRun</command>";
  socksrv.send(stop_run,26);
  socksrv.recv(answer,1000);
  Log(debug,"Got answer [%s]", answer);
  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught: %s",e.what());
    ::abort();
  }

}

void* reader_thread(void *arg) {

  Log(info,"Working in reader_thread!!!");

  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  Log(info,"#### Reader thread: %s",stream.str().c_str());

  // Create a client that connects to the PTB
  try{

   const PTBReader *reader = (const PTBReader *) arg;
   if (reader == NULL) {
     Log(error,"There is no reader");
     return NULL;
   }
   Log(info,"Waiting for reader to be ready...");
   while (!reader->isReady()) {
     ;
   }
   Log(info,"Reader is ready... let's start the show!!!");


  // Create a tcp server and listen to the connection
  TCPServerSocket servSock(8992);
  // Accept a client
  Log(info,"Entering waiting state");
  TCPSocket *sock = servSock.accept();
  Log(info,"--> Received a client request.");

  // Keep looping to read data and dump it
  uint32_t header;
  uint32_t body_counter[3];
  uint32_t body_trigger;
  uint32_t timestamp[2];
  uint32_t checksum;
  uint16_t size;
  uint8_t wtype;
  //  uint32_t *buffer = NULL;  // Comment this out if unused
  for (uint32_t i = 0; i < 10; ++i) {
    // First listen for a header
    sock->recv(&header, sizeof(header));
    //std::cout << "Header : " << std::hex << header << std::endl;
    Log(debug,"HEADER %x %u\n",header,header);
    // Catch the size
    size = header & 0xFFFF;
    Log(debug,"SIZE %x %u\n",(uint32_t)size,(uint32_t)size);

    // Allocate a buffer to grab the whole thing from the ethernet
    //buffer = (uint32_t*) malloc(size);

    // Grab the whole thing
    //sock->recv(buffer, size);

    // Now parse it properly.
    uint16_t i_size = (uint16_t)sizeof(uint32_t);
    while (i_size < size) {
      sock->recv(&header, sizeof(header));
      i_size += (uint16_t)sizeof(header);
      Log(info,"PHEADER : %x \n",header);
      //std::cout << "Header : " << std::hex << header << std::dec << std::bitset<32>(header) << std::endl;
      wtype = ((header >> 29) & 0x7);
      Log(info,"WTYPE %x %u\n",(uint32_t)wtype,(uint32_t)wtype);
      if (wtype == 0x1) { // counter word
        //pick up a counter word
        sock->recv(body_counter, sizeof(body_counter));
        Log(info,"COUNTER   : %x %x %x \n",body_counter[0],body_counter[1],body_counter[2]);
        i_size += sizeof(body_counter);
      } else if (wtype == 0x2) { // parse a trigger word
        sock->recv(&body_trigger, sizeof(body_trigger));
        Log(info,"TRIGGER   : %x \n",body_trigger);
        i_size += sizeof(body_trigger);
      } else if (wtype == 0x7) { // It is a timestamp
        sock->recv(timestamp, sizeof(timestamp));
        Log(info,"TIMESTAMP : %x %x \n",timestamp[0],timestamp[1]);
        i_size += sizeof(timestamp);
      } else if (wtype == 0x4) { // checksum
        sock->recv(&checksum, sizeof(checksum));
        Log(info,"CHECKSUM : %x \n",checksum);
        i_size += sizeof(checksum);
      }
    }
    std::cout << "== Packet printed." << endl;
  }
  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught: %s",e.what());
    ::abort();
  }
  catch(...) {
    Log(error,"Caught some nasty exception.");
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
  Log(info,"Just a test");

  // This doesn't work since the thread gets stuck in the constructor of ConfigServer
  // Need to make the acceptance of the client into a separate thread.


  //  ConfigServer*cfg = ConfigServer::get(); // Comment this out if unused

  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  Log(info,"#### Main thread: %s",stream.str().c_str());
  // Create a thread that passes the configuration

  //Log(debug) << "Going to sleep" <<endlog;
  //std::this_thread::sleep_for (std::chrono::seconds(10));
  Log(info,"Starting manager");
  bool emulate = true;
  // Start in emulating mode.
  PTBManager manager(emulate);

  manager.DumpConfigurationRegisters();

  Log(info,"Creating control thread");
  std::thread t1(control_thread);


  // Get the reader
  const PTBReader* reader = manager.getReader();
  Log(info,"Showing the reader: 0x%X", reader);
  //manager.StartRun();

  Log(info,"Starting the reader thread...");
  // -- Create a board reader facsimile
  pthread_t threadID_controller;
  if (pthread_create(&threadID_controller, NULL, &(reader_thread),(void*)reader) != 0) {
      Log(fatal,"Unable to create master thread.");
    }

//  while (!reader->isReady()) {
//    ;
//  }


  Log(info,"Joining the threads");
  pthread_join(threadID_controller,NULL);
  Log(info,"One joined");
  t1.join();
  Log(info,"Other joined");
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

