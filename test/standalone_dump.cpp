/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"
//#include "ConfigServer.h"
//#include "PTBManager.h"
#include <bitset>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <string>
#include <fstream>
#include "PracticalSocket.h"
//extern "C" {
//#include <pthread.h>         // For POSIX threads
//}
//
//using namespace std;

#include <csignal>
#include "../src/boardreader.h"
#include "../src/boardmanager.h"
#include "../src/boardserver.h"

using std::cout;
using std::cerr;

using ptb::board_reader;
using ptb::board_manager;
using ptb::board_server;

bool g_stop_requested;
bool g_is_configured;
FILE*outfile;

void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    //exit(signum);
    g_stop_requested = true;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    ::exit(0);
}

int send_reset(TCPSocket *sock) {
  Log(info,"Sending a reset");
  const char *start_run = "<command>HardReset</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  Log(info,"Got answer [%s]",answer);
  g_is_configured = false;
  return 0;
}

int start_run(TCPSocket *sock) {
  Log(info,"Starting the run");
  const char *start_run = "<command>StartRun</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  Log(info,"Got answer [%s]",answer);

  return 0;
}

int stop_run(TCPSocket *sock) {
  Log(info,"Starting the run");
  const char *start_run = "<command>StopRun</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  Log(info,"Got answer [%s]",answer);

  return 0;
}

void send_config(TCPSocket *socksrv) {
  if (!g_is_configured) send_reset(socksrv);

  std::ifstream config("./config/config_default.xml");
  // Read the whole file contents
  std::string config_contents((std::istreambuf_iterator<char>(config)), std::istreambuf_iterator<char>());
  //std::string config_contents;
  //config >> config_contents;
  config.close();

  char answer[1000];
  socksrv->send(config_contents.c_str(),config_contents.length());
  socksrv->recv(answer,1000);
  Log(info,"Got answer [%s]",answer);

  g_is_configured = true;
  //return 0;
}

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
  std::this_thread::sleep_for(std::chrono::seconds(10));

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

	  // Create a tcp server and listen to the connection
	  TCPServerSocket servSock(8992);
	  // Accept a client
	  Log(info,"Entering waiting state");
	  TCPSocket *sock = servSock.accept();
	  Log(info,"--> Received a client request.");


   const board_reader *reader = (const board_reader *) arg;
   if (reader == NULL) {
     Log(error,"There is no reader");
     return NULL;
   }
   Log(info,"Waiting for reader to be ready...");
   while (!reader->get_ready()) {
     ;
   }
   Log(info,"Reader is ready... let's start the show!!!");



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
      //Log(info,"PHEADER : %x \n",header);

      wtype = ((header >> 29) & 0x7);
      //Log(info,"WTYPE %x %u\n",(uint32_t)wtype,(uint32_t)wtype);
      if (wtype == 0x1) { // counter word
        //pick up a counter word
        sock->recv(body_counter, sizeof(body_counter));
        //Log(info,"COUNTER   : %x %x %x \n",body_counter[0],body_counter[1],body_counter[2]);
        i_size += sizeof(body_counter);
      } else if (wtype == 0x2) { // parse a trigger word
        sock->recv(&body_trigger, sizeof(body_trigger));
        //Log(info,"TRIGGER   : %x \n",body_trigger);
        i_size += sizeof(body_trigger);
      } else if (wtype == 0x7) { // It is a timestamp
        sock->recv(timestamp, sizeof(timestamp));
        //Log(info,"TIMESTAMP : %x %x \n",timestamp[0],timestamp[1]);
        i_size += sizeof(timestamp);
      } else if (wtype == 0x4) { // checksum
        sock->recv(&checksum, sizeof(checksum));
        //Log(info,"CHECKSUM : %x \n",checksum);
        i_size += sizeof(checksum);
      }
    }
    //std::cout << "== Packet printed." << endl;
  }
  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught: %s",e.what());
    ::abort();
  }
  catch(...) {
    Log(error,"Caught some nasty exception.");
  }
  return NULL;
}

void receive_data() {
  Log(info,"Working in reader_thread!!!");

  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  Log(info,"#### Reader thread: %s",stream.str().c_str());

  // Create a client that connects to the PTB
  try{

    // Create a tcp server and listen to the connection
    TCPServerSocket servSock(8992);
    // Accept a client
    Log(info,"Entering waiting state");
    TCPSocket *sock = servSock.accept();
    Log(info,"--> Received a client request.");

    uint16_t pckt_size = 0;
    uint16_t bytes_collected= 0;
    uint32_t header;
    uint8_t body[4096];

    bool timeout_reached = false;
    // FIXME: Receive the ethernet packets
    while ((!g_stop_requested) || (g_stop_requested && !timeout_reached)) {
      // grab a header
      sock->recv(&header,sizeof(header));
      // Cast the result into the header
      ptb::content::tcp_header_t *pkg_hdr = reinterpret_cast<ptb::content::tcp_header_t*>(&header);
      pckt_size = pkg_hdr->packet_size;
      while (bytes_collected != pckt_size) {
        sock->recv(&body[bytes_collected],(int)(pckt_size-bytes_collected));
      }
    }
  }
  catch(...) {
    ;
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
  enum commands {init=1,start=2,stop=3};
  Logger::SetSeverity(Logger::debug);
  // Set the stop condition to false to wait for the data
  g_stop_requested = false;
  std::signal(SIGINT, signalHandler);

// This doesn't work since the thread gets stuck in the constructor of ConfigServer
// Need to make the acceptance of the client into a separate thread.


//  ConfigServer*cfg = ConfigServer::get(); // Comment this out if unused

std::thread::id thread_id = std::this_thread::get_id();
std::ostringstream stream;
stream << std::hex << thread_id << " " << std::dec << thread_id;
Log(info,"#### Main thread: %s",stream.str().c_str());
// Create a thread that passes the configuration

int command = -1;
int result = 0;
try{
  Log(info,"### Establishing control connection to the PTB");
  TCPSocket socksrv("localhost", 8991);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Now make the receiving thread
  Log(info,"### Launching reader thread");
  std::thread reader(receive_data);
  // Now loop for commands
  while(!g_stop_requested) {
    cout << "### Select command : " << endl;
    cout << " 1 : Init" << endl;
    cout << " 2 : Start Run" << endl;
//    cout << " 3 : Stop Run" << endl;
//    cout << " 4 : Soft Reset" << endl;
//    cout << " 5 : Hard Reset" << endl;

    cin >> command;
    switch(command) {
      case init:
        send_config(&socksrv);
        break;
      case start:
        result = start_run(&socksrv);
        if (result) {
          cout << "Failed to issue command." << endl;
        }
        break;
//      case stop:
//        result = stop_run(&socksrv);
//        if (result) {
//          cout << "Failed to issue command." << endl;
//        }
//        break;
      default:
        cout << "Wrong option (" << command << ")." << endl;
        break;
    }
  }
  Log(warning,"### Stop requested.");
  // Wait for the reading function to return
  reader.join();

  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught: %s",e.what());
    ::abort();
  }
  return 0;
}

