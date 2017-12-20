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
#include <csignal>
#include <cstdint>

//FIXME: Use a more robust socket library...for example boost seems sensible
#include "PracticalSocket.h"
#include "boardreader.h"
#include "boardmanager.h"
#include "boardserver.h"

// -- namespace declarations for common STL objects
using std::cout;
using std::cerr;
using std::endl;

// -- same for PTB
using ptb::board_reader;
using ptb::board_manager;
using ptb::board_server;

// -- configuration that we are running with for now
static const std::string g_config = "<config><DataBuffer><DaqHost>localhost</DaqHost><DaqPort>8992</DaqPort></DataBuffer></config>";

class ctb_robot {
public:
  ctb_robot(const std::string &host = "localhost",const uint16_t &port = 8991)
  : stop_req_(false), is_running_(false),is_conf_(false)
  {
   client_sock.connect(host,port);
   //std::signal(SIGIO, ctb_robot::static_sig_handler);
  }
  virtual ~ctb_robot() {};

  void send_reset() {
    cout << "Sending a reset" << endl;
    const char *cmd = "<command>HardReset</command>";
    client_sock.send(cmd,28);
    client_sock.recv(answer_,1024);
    cout << "Received answer [" << answer_ << "]" << endl;
    is_running_ = false;
    is_conf_ = false;
  }

  void send_start() {
    if (!is_conf_) {
      cout << "ERROR: Can't start a run without configuring first" << endl;
    }
    cout << "Sending a start run" << endl;
    const char *cmd = "<command>StartRun</command>";
    client_sock.send(cmd,27);
    client_sock.recv(answer_,1024);
    cout << "Received answer [" << answer_ << "]" << endl;
    is_running_ = true;
  }

  void send_stop() {
    cout << "Sending a stop run" << endl;
    const char *cmd = "<command>StopRun</command> ";
    client_sock.send(cmd,27);
    client_sock.recv(answer_,1024);
    cout << "Received answer [" << answer_ << "]" << endl;
    is_running_ = false;
  }

  void send_config() {
    cout << "Sending config" << endl;
    if (!is_conf_) {
      cout << "Resetting before configuring" << endl;
      send_reset();
    }
    client_sock.send(g_config.c_str(),g_config.size());
    client_sock.recv(answer_,1024);
    cout << "Received answer [" << answer_ << "]" << endl;
    is_conf_ = true;
  }


  void run() {
    enum commands {init=1,start=2,stop=3};
    // Set the stop condition to false to wait for the data
    std::signal(SIGINT, ctb_robot::static_sig_handler);
  int command = -1;
  int result = 0;
    // Now make the receiving thread
    Log(info,"### Launching reader thread");
    //std::thread reader(this->receive_data);
    std::thread reader(&ctb_robot::receive_data, this);
    // Now loop for commands
    while(!stop_req_) {
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
          send_config();
          break;
        case start:
          send_start();
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

  void receive_data() {
    // Important to avoid mangling of the output due to race conditions to the
    // IO buffers. The trick is to use unbuffered (ie. direct) output
    std::cout.setf(std::ios::unitbuf);

    cout << "Working in reader_thread!!!" << endl;

    receiver_id_ = std::this_thread::get_id();
    std::ostringstream stream;
    stream << std::hex << receiver_id_ << " " << std::dec << receiver_id_;
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

      //ptb::content::word::word_type wt;



      uint16_t pckt_size = 0;
      uint16_t bytes_collected= 0;
      ptb::content::tcp_header header;
      ptb::content::word::header *hdr;
      //uint32_t header;
      uint8_t tcp_body[4096];
      size_t count = 0;
      bool timeout_reached = false;
      // FIXME: Receive the ethernet packets
      while ((!stop_req_) || (stop_req_ && !timeout_reached)) {
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
            cout << "Received timestamp " << ts->timestamp << endl;
            pos += ptb::content::word::body_t::size_bytes;
            break;
          }
        }

      }
    }
    catch(...) {
      cerr << "Something went wrong. " << endl;
    }

  }

  void sig_handler (int signum)
  {
    if (is_running_) {
      // if we are taking data, stop it
      send_stop();
    } else {
      // If we are not, stop execution
      stop_req_ = true;
    }
  }


  static void static_sig_handler(int signum)
  {
      instance.sig_handler(signum);
  }

private:
  TCPSocket client_sock;

  // control variables
  bool stop_req_;
  bool is_running_;
  bool is_conf_;
  std::thread::id receiver_id_;
  // Aux vars
  char answer_[1024];

  // Static instance so we can play with the signal handler
  static ctb_robot instance;
};

int main() {

  ctb_robot robot("localhost",8991);
  robot.run();

  return 0;
}

