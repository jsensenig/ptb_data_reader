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
#include <sstream>

//FIXME: Use a more robust socket library...for example boost seems sensible
#include "PracticalSocket.h"
#include "content.h"
#include "json.hpp"
#include "cxxopts.hpp"

// -- namespace declarations for common STL objects
using std::cout;
using std::cerr;
using std::endl;
using json = nlohmann::json;

//////////////////////////////
// -- configuration that we are running with for now, either set the DAC values to all 0 or the operation level 1879
// EDIT here where the IP where you are running, which is where the CTB will attempt to connect to send the data


static std::string g_config; 
std::string cfile = "ctb_config.json";


//static const std::string g_config = "{\"ctb\":{\"sockets\":{\"receiver\":{\"host\":\"localhost\",\"port\":8992,\"rollover\":50000}},\"subsystems\":{\"ssp\":{\"dac_thresholds\":[2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018]}}}}";


class ctb_robot {
  public:
    ctb_robot(const std::string &host = "localhost",const uint16_t &port = 8991)
  : stop_req_(false), is_running_(false),is_conf_(false)
  {
      client_sock.connect(host,port);
  }
    virtual ~ctb_robot() {};

    void send_reset() {
      cout << "Sending a reset" << endl;
      const char *cmd = "{\"command\":\"HardReset\"}";
      client_sock.send(cmd,28);
      client_sock.recv(answer_,1024);
      cout << "Received answer [" << answer_ << "]" << endl;
      answer_[0]='\0';

      is_running_ = false;
      is_conf_ = false;
    }

    void send_start() {
      if (!is_conf_) {
        cout << "ERROR: Can't start a run without configuring first" << endl;
      }
      cout << "Sending a start run" << endl;
      const char *cmd = "{\"command\":\"StartRun\"}";
      client_sock.send(cmd,27);
      client_sock.recv(answer_,1024);
      cout << "Received answer [" << answer_ << "]" << endl;
      answer_[0]='\0';
      is_running_ = true;
    }

    void send_stop() {
      cout << "Sending a stop run" << endl;
      const char *cmd = "{\"command\":\"StopRun\"}";
      client_sock.send(cmd,27);
      client_sock.recv(answer_,1024);
      cout << "Received answer [" << answer_ << "]" << endl;
      answer_[0]='\0';
      is_running_ = false;
    }

    void send_config() {

      cout << "Sending config" << endl;
      if (!is_conf_) {
        cout << "Resetting before configuring" << endl;
        send_reset();
      }
      send_reset();
      client_sock.send(g_config.c_str(),g_config.size());
      client_sock.recv(answer_,1024);
      cout << "Received answer [" << answer_ << "]" << endl;
      answer_[0]='\0';
      is_conf_ = true;
    }

    void process_quit()
    {
      if (is_running_) {
        // if we are taking data, stop it
        send_stop();
      } else {
        // If we are not, stop execution
        stop_req_ = true;

      }
    }

    void run() {
      enum commands {init=1,start=2,stop=3,quit=4};
      // Set the stop condition to false to wait for the data
      //std::signal(SIGINT, ctb_robot::static_sig_handler);
      int command = -1;
      // Now make the receiving thread
      cout << "### Launching reader thread" << endl;
      //std::thread reader(this->receive_data);
      std::thread reader(&ctb_robot::receive_data, this);
      // Now loop for commands
      std::ifstream cfin;
      json conf;
      while(!stop_req_) {
        cout << "### Select command : " << endl;
        cout << " 1 : Init" << endl;
        cout << " 2 : Start Run" << endl;
        cout << " 3 : Stop Run" << endl;
        cout << " 4 : Quit" << endl;

        //    cout << " 4 : Soft Reset" << endl;
        //    cout << " 5 : Hard Reset" << endl;

        cin >> command;
        switch(command) {
          case init:
            // Now open the file
            cfin.exceptions ( ifstream::failbit | ifstream::badbit );
            try
            {
              cfin.open(cfile);
              cfin >> conf;
              // if (vlvl > 0) {
              // 	cout << "Dumping input configuration:"<< endl;
              // 	cout << "===============================" << endl;
              // 	cout << conf.dump(2) << endl;
              // 	cout << "===============================" << endl;
              // }
              g_config = conf.dump();
              cfin.close();
            }
            catch (const ifstream::failure& e)
            {
              cout << "** Failure opening/reading the configuration file: " << e.what() << endl;
              exit(1);
            }
            catch( const json::exception& e) {
              cout << "Caught a JSON exception: " << e.what() << endl;
              exit(1);
            }
            send_config();
            break;
          case start:
            send_start();
            break;
          case stop:
            send_stop();
            break;
          case quit:
            process_quit();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            //reader.
            //reader.join();
            break;
          default:
            cout << "Wrong option (" << command << ")." << endl;
            break;
        }
      }
      cout << "### Stop requested." << endl;
      // Wait for the reading function to return

    }

    void receive_data() {

      // Important to avoid mangling of the output due to race conditions to the
      // IO buffers. The trick is to use unbuffered (ie. direct) output
      std::cout.setf(std::ios::unitbuf);
      cout << "Working in receiving thread!!!" << endl;

      receiver_id_ = std::this_thread::get_id();
      std::ostringstream stream;
      stream << std::hex << receiver_id_ << " " << std::dec << receiver_id_;
      cout << "#### Receiving thread: %s" << stream.str() << endl;

      // Create a server that is meant to receive the data from the
      // PTB and keep dumping the contents into somewhere...like a binary file
      try{
        // Create a tcp server and listen to the connection
        TCPServerSocket servSock(8992);


        while (!stop_req_) {
          // Accept a client
          cout << "Opening the receiver socket" << endl;
          TCPSocket *sock = servSock.accept();
          cout << "--> Received a client request." << endl;

          // just wait for the run to be started
          while (!is_running_ && !stop_req_) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
          }

          cout << "Run started...." << endl;

          uint16_t bytes_collected= 0;
          ptb::content::tcp_header *header;
          ptb::content::word::word_t*word;

          //uint32_t header;
          uint8_t tcp_data[4096];
          size_t count = 0;
          bool timeout_reached = false;
          size_t tcp_body_size = 0;

          // FIXME: Receive the ethernet packets
          while (is_running_ && !stop_req_) {

            // grab a header
            //cout << "Going to receive " << header->word.size_bytes << " bytes" << endl;
            bytes_collected = 0;
            while ((bytes_collected != header->word.size_bytes) && is_running_) {
              bytes_collected += sock->recv(&tcp_data[bytes_collected],header->word.size_bytes-bytes_collected);
            }
            //printf("Received %u bytes : %X\n",bytes_collected,*(reinterpret_cast<uint32_t*>(&tcp_data[0])));
            header = reinterpret_cast<ptb::content::tcp_header *>(tcp_data);
            count++;
            if (!(count%1000)) cout << "Counting " << count << "packets received..." << endl;
            // check the number of bytes in this packet
            //cout << "Expecting to receive " << header->word.packet_size << " bytes " << endl;
            tcp_body_size = header->word.packet_size;
            bytes_collected = 0;
            while (bytes_collected != tcp_body_size) {
              bytes_collected += sock->recv(&tcp_data[bytes_collected],(int)(tcp_body_size-bytes_collected));
            }
            // cout << "Collected expected bytes. " << endl;
            // for(size_t i = 0; i < bytes_collected; i++) {
            //   printf("%02X ",tcp_data[i]);
            // }
            // printf("\n");
            // -- We now should have the whole sent packet
            // parse it
            uint8_t *data = NULL;
            uint32_t pos = 0;
            uint32_t val = 0;
            // -- Declare pointers to the different word types
            ptb::content::word::ch_status_t *chs = NULL;
            ptb::content::word::feedback_t  *fbk = NULL;
            ptb::content::word::timestamp_t *ts = NULL;
            ptb::content::word::trigger_t   *trg = NULL;
            while (pos < tcp_body_size) {
              // 1. grab the frame:
              word = reinterpret_cast<ptb::content::word::word_t*>(&tcp_data[pos]);
              // 2. Grab the header (we know what is its size)
//              hdr = &(word->wheader); //reinterpret_cast<ptb::content::word::header_t *>(tcp_data[pos]);
//              pos += hdr->size_bytes;

              // -- For now we are assuming that all payloads are of the same size
              //cout << "Word: type " << static_cast<uint32_t>(hdr->word_type)
              //<< " ts roll " << hdr->ts_rollover << endl;
              switch(word->word_type) {
                case ptb::content::word::t_fback:
                  //cout << "Received a warning!!! This is rare! Do something smart to fix the problem" << endl;
                  data = reinterpret_cast<uint8_t*>(word);
                  for (size_t i = 0; i < 16; ++i) {
                    printf("%X ",data[i]);
                  }
                  printf("\n");

                  // advance the pointer by the size of this payload
                  pos += ptb::content::word::word_t::size_bytes;
                  break;
                case ptb::content::word::t_gt:
                  cout << "Received a global trigger. Do something here on how to parse it" << endl;
                  trg = reinterpret_cast<ptb::content::word::trigger_t*>(word);
                  // advance the pointer by the size of this payload
                  pos += ptb::content::word::word_t::size_bytes;
                  break;
                case ptb::content::word::t_lt:
                  cout << "Received a low level trigger. Do something here on how to parse it" << endl;
                  trg = reinterpret_cast<ptb::content::word::trigger_t*>(word);
                  // advance the pointer by the size of this payload
                  cout << "LT: " << std::bitset<3>(trg->word_type) << " 0x"
                      << std::hex << trg->trigger_mask << std::dec << " "
                      << std::bitset<61>(trg->trigger_mask) << endl;
                  pos += ptb::content::word::word_t::size_bytes;
                  break;
                case ptb::content::word::t_ts:
                  ts = reinterpret_cast<ptb::content::word::timestamp_t *>(word);
                  //cout << "Received timestamp " << ts->timestamp << endl;
                  pos += ptb::content::word::word_t::size_bytes;
                  break;
                case ptb::content::word::t_ch:
                  chs = reinterpret_cast<ptb::content::word::ch_status_t *>(word);
                  val = chs->pds;
                  cout << "CH: " << std::bitset<3>(chs->word_type) << " " << chs->timestamp << " PDS 0x" << std::hex << val << std::dec << " " << std::bitset<24>(val)
                       << " CRT 0x" << std::hex << chs->get_crt() << std::dec
                       << " BI  0x" << std::hex << chs->get_beam() << std::dec << endl;
                  pos += ptb::content::word::word_t::size_bytes;
                  break;
                default:
                  cout << "WARNING: Unknown header" << endl;
                  break;
              }
            }
          }
          cout << "Left the loop for running..." << endl;
          delete sock;
          if (!stop_req_) {
            cout << "Going to start a new socket listening" << endl;
          }

        }
        cout << "Stop request found... should be returning here" << endl;

        // -- Don't hang in here. Disconnect the thing
        return;
      }
      catch(SocketException &e) {
        cout << "Socket exception caught : " << e.what() << endl;
      }
      catch(std::exception &e) {
        cout << "STD exception caught : " << e.what() << endl;
      }
      catch(...) {
        cerr << "Something went wrong. " << endl;
      }
      // Delete the connection
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
};

int main(int argc, char**argv) {

  std::cout.setf(std::ios::unitbuf);
  size_t vlvl = 0;

//  cout << "Some debugging..." << endl;
//  cout << "Size of header : " << sizeof(ptb::content::word::header_t) << endl;
//  cout << "Body : " << sizeof(ptb::content::word::body_t) << endl;
//  cout << "Size of channel status : " << sizeof(ptb::content::word::payload::ch_status_t) << endl;



  try {
    cxxopts::Options options(argv[0], " - command line options");
    options.add_options()
        ("c,config","Configuration file (JSON format)",cxxopts::value<std::string>())
        ("h,help", "Print help")
        ("v,verbosity","Verbosity level",cxxopts::value<size_t>(vlvl))
        ;

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      cout << options.help() << endl;
      exit(0);
    }

    if (result.count("config")) {
      cout << "--> Using " << result["config"].as<std::string>() << " config file." << endl;
      cfile = result["config"].as<std::string>();    
    } else {
      cout << "===> Using default configuration file [" << cfile << "] " << endl;
    }

    cout << "--> Verbosity level [" << vlvl << "]" << endl;

  }
  catch( const cxxopts::OptionException& e) 
  {
    cout << "** Error parsing options: " << e.what() << endl;
    exit(1);
  }

  cout << "Starting robot..." << endl;
  //ctb_robot robot("128.91.41.238",8991);
  ctb_robot robot("localhost",8991);
  cout << "Starting the loop..." << endl;
  robot.run();
  cout << "All done" << endl;

  return 0;
}

