/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include <bitset>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <string>
#include <fstream>
#include <iostream>

#include "PracticalSocket.h"
//extern "C" {
//#include <pthread.h>         // For POSIX threads
//}
//
//using namespace std;

#include <csignal>
#include "boardreader.h"
#include "boardmanager.h"
#include "boardserver.h"

using std::cout;
using std::cerr;
using std::endl;

using ptb::board_reader;
using ptb::board_manager;
using ptb::board_server;

bool g_stop_requested;
bool g_is_running;
bool g_is_configured;
//FILE*outfile;

TCPSocket *socket;


int send_reset(TCPSocket *sock) {
  Log(info,"Sending a reset");
  const char *start_run = "<command>HardReset</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  cout << "Got answer [" << answer << "]" << endl;

  g_is_configured = false;
  g_is_running = false;
  return 0;
}

int start_run(TCPSocket *sock) {
  Log(info,"Starting the run");
  const char *start_run = "<command>StartRun</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  cout << "Got answer [" << answer << "]" << endl;
  g_is_running = true;

  return 0;
}

int stop_run(TCPSocket *sock) {
  Log(info,"Starting the run");
  const char *start_run = "<command>StopRun</command> ";
  char answer[1000];
  sock->send(start_run,27);
  sock->recv(answer,1000);
  cout << "Got answer [" << answer << "]" << endl;
  g_is_running = false;
  return 0;
}

void send_config(TCPSocket *socksrv) {
  if (!g_is_configured) send_reset(socksrv);

  // -- For now the configuration is still minimal
  // also because the structure has not been decided yet.
  // it is also implemnted using XML yet...eventually might want to do it in
  // json or some other fancy way
  // The only parameter that is being used at the moment is really the destination of the data
  std::string config_contents = "<config><DataBuffer><DaqHost>localhost</DaqHost><DaqPort>8992</DaqPort></DataBuffer></config>";

  char answer[1000];
  socksrv->send(config_contents.c_str(),config_contents.length());
  socksrv->recv(answer,1000);
  cout << "Got answer [" << answer << "]" << endl;

  g_is_configured = true;
  //return 0;
}

void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // If it is running (taking a run), stop the run
    if (g_is_running) {
      // -- Stop the run
      stop_run(socket);
      return;
    }
    // otherwise stop the application completely
    g_stop_requested = true;

    // cleanup and close up stuff here
    // terminate program

    //exit(signum);
    //g_stop_requested = true;
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    //::exit(0);
}

/*
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
*/

void receive_data() {
  // Important to avoid mangling of the output due to race conditions to the
  // IO buffers. The trick is to use unbuffered (ie. direct) output
  std::cout.setf(std::ios::unitbuf);

  cout << "Working in reader_thread!!!" << endl;

  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  cout << "#### Receiving thread: %s" << stream.str() << endl;

  // Create a server that is meant to receive the data from the
  // PTB and keep dumping the contents into somewhere...like a binary file
  try{

    // Create a tcp server and listen to the connection
    TCPServerSocket servSock(8992);
    // Accept a client
    cout << "Opening the receiver socket" << endl;
    TCPSocket *sock = servSock.accept();
    cout << "--> Received a client request." << endl;

    // -- Use the content header to parse the received data
    // Also, might want to expand the rollover timestamps into full timestamps

    // -- open an output file.
    // -- This is an example, so just dump into some random format
    // let's say, one word/frame per line in binary format in a file
    // the frames will have the full timestamp already expanded

    // -- the idea here is to show how the parsing would normally work

    ptb::content::word::word_type wt;



    uint16_t pckt_size = 0;
    uint16_t bytes_collected= 0;
    ptb::content::tcp_header header;
    ptb::content::word::header *hdr;
    //uint32_t header;
    uint8_t tcp_body[4096];
    size_t count = 0;
    bool timeout_reached = false;
    // FIXME: Receive the ethernet packets
    while ((!g_stop_requested) || (g_stop_requested && !timeout_reached)) {
      // grab a header
      sock->recv(&header,header.word.size_bytes);
      count++;
      if (!(count%1000)) cout << "Counting " << count << "packets received..." << endl;
      // check the number of bytes in this packet
      bytes_collected = 0;
      // Cast the result into the header
//      ptb::content::tcp_header_t *pkg_hdr = reinterpret_cast<ptb::content::tcp_header_t*>(&header);
//      pckt_size = pkg_hdr->packet_size;
      while (bytes_collected != header.word.packet_size) {
        sock->recv(&tcp_body[bytes_collected],(int)(pckt_size-bytes_collected));
      }
      // -- We now should have the whole sent packet
      // parse it
      uint32_t pos = 0;
      while (pos <= header.word.packet_size) {
        // 1. Grab the header (we know what it its size)
        hdr = reinterpret_cast<ptb::content::word::header *>(&tcp_body[pos]);
        pos += hdr->word.size_bytes;

        // -- For now we are assuming that all payloads are of the same size
        switch(hdr->word.word_type) {
        case ptb::content::word::t_fback:
          cout << "Received a warning!!! This is rare! Do something smart to fix the problem" << endl;
          // advance the pointer by the size of this payload
          pos += ptb::content::word::body_t::size_bytes;
          break;
        case ptb::content::word::t_gt:
          cout << "Received a global trigger. Do something here on how to parse it" << endl;
          // advance the pointer by the size of this payload
          pos += ptb::content::word::body_t::size_bytes;
          break;
        case ptb::content::word::t_lt:
          cout << "Received a low level trigger. Do something here on how to parse it" << endl;
          // advance the pointer by the size of this payload
          pos += ptb::content::word::body_t::size_bytes;
          break;
        case ptb::content::word::t_ts:
          // Received a timestamp. Print it...why the hell not?
          ptb::content::word::payload::timestamp_t *ts = reinterpret_cast<ptb::content::word::payload::timestamp_t *>(&tcp_body[pos]);
          //cout << "Received timestamp " << ts->timestamp << endl;
          pos += ptb::content::word::body_t::size_bytes;
          break;
        }
      }

    }
  }
  catch(...) {
    cerr << "Something went wrong. Unsure what" << endl;
  }
}

int main() {
  enum commands {init=1,start=2,stop=3};
  // Set the stop condition to false to wait for the data
  g_is_running = false;
  g_stop_requested = false;
  g_is_configured = false;
  std::signal(SIGINT, signalHandler);

// This doesn't work since the thread gets stuck in the constructor of ConfigServer
// Need to make the acceptance of the client into a separate thread.


//  ConfigServer*cfg = ConfigServer::get(); // Comment this out if unused

std::thread::id thread_id = std::this_thread::get_id();
std::ostringstream stream;
stream << std::hex << thread_id << " " << std::dec << thread_id;
cout << "#### Main thread: " << stream.str() << endl;
// Create a thread that passes the configuration

int command = -1;
int result = 0;
try{
  cout << "### Establishing control connection to the PTB" << endl;
  //*socket = TCPSocket("localhost",8991);
  //socket = new TCPSocket("localhost",8991);
  TCPSocket socksrv("localhost", 8991);
  socket = &socksrv;
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Now make the receiving thread
  Log(info,"### Launching reader thread");
  std::thread reader(receive_data);
  // Now loop for commands
  while(!g_stop_requested) {
    cout << "### Select command : " << endl;
    cout << " 1 : Init" << endl;
    cout << " 2 : Start Run" << endl;
    cout << "\n\n After starting a run, you can stop the run with Ctrl+C\n\n" << endl;

//    cout << " 3 : Stop Run" << endl;
//    cout << " 4 : Soft Reset" << endl;
//    cout << " 5 : Hard Reset" << endl;

    cin >> command;
    switch(command) {
      case init:
        send_config(socket);
        break;
      case start:
        result = start_run(socket);
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
  cout << "### Stop requested." << endl;
  // Wait for the reading function to return
  reader.join();

  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught: %s",e.what());
    ::abort();
  }
  return 0;
}

