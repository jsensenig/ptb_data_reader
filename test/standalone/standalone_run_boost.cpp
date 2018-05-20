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

#include "content.h"
#include "json.hpp"
#include "cxxopts.hpp"



// -- Boost inputs
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>


#include "ctbclient.hh"
#include "ctbdatareceiver.hh"

using boost::asio::ip::tcp;

// -- namespace declarations for common STL objects
using std::cout;
using std::cerr;
using std::endl;
using json = nlohmann::json;

//////////////////////////////
// -- configuration that we are running with for now, either set the DAC values to all 0 or the operation level 1879
// EDIT here where the IP where you are running, which is where the CTB will attempt to connect to send the data


//static std::string g_config;
//std::string cfile = "ctb_config.json";


//static const std::string g_config = "{\"ctb\":{\"sockets\":{\"receiver\":{\"host\":\"localhost\",\"port\":8992,\"rollover\":50000}},\"subsystems\":{\"ssp\":{\"dac_thresholds\":[2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018,2018]}}}}";


class ctb_robot {
  public:
    ctb_robot(const std::string &ptb_host = "localhost",const std::string &ptb_port = "8991",
        const uint16_t &reader_port = 8992, const int &debug_level = 1,
        const uint32_t &tick_period_usecs = 5 )
  : data_timeout_usecs_(100000),
    penn_data_dest_host_("localhost"),penn_data_dest_port_(reader_port),
    penn_client_host_addr_(ptb_host),penn_client_host_port_(ptb_port),
    is_running_(false), is_conf_(false), stop_req_(false),data_receiver_(nullptr)

  {
      client_ = std::unique_ptr<ctb_client>(new ctb_client(ptb_host, ptb_port, data_timeout_usecs_));
      client_->send_command("HardReset");
      if (client_->exception())
      {
        printf("ctb_robot : Caught an exception instantiating the client\n");
      }
      config_filename_ = "ctb_config.json";
      output_filename_ = "ctb_output.root";
      // -- The data receiver should only be launched when a start run is called
  }


    virtual ~ctb_robot() {};

    void send_reset() {
      printf("Sending a reset\n");
      client_->send_command("HardReset");

      is_running_ = false;
      is_conf_ = false;
    }

    void send_start() {
      if (!is_conf_) {
        printf("ERROR: Can't start a run without configuring first\n");
        return;
      }

      // -- Launching a receiver
      printf("Creating a receiver...\n");
      if (!data_receiver_ ) {
        data_receiver_ =
          std::unique_ptr<ctb_data_receiver>(new ctb_data_receiver(1,penn_data_dest_port_));

      data_receiver_->set_stop_delay(1000000); //usec
      // Sleep for a short while to give time for the DataReceiver to be ready to
      // receive connections
      usleep(500000);
      }

      printf("Sending a start run\n");
      // Start the data receiver
      data_receiver_->start();
      // Send start command to PENN

      client_->send_command("StartRun");
      if (client_->exception()) {
        printf("ERROR: Exception caught sending a start run\n");
        exit(1);
      }
      is_running_ = true;
    }

    void send_stop() {
      printf("Sending a stop run\n");
      client_->send_command("StopRun");

      sleep(1);

      // Stop the data receiver.
      data_receiver_->stop();

      is_running_ = false;
    }

    void send_config() {

      printf("Sending config\n");
      if (!is_conf_) {
        printf("Resetting before configuring\n");
        send_reset();
      }
      send_reset();

      std::ifstream cfin;
      json conf;
      cfin.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
      try
      {
        cfin.open(config_filename_);
        cfin >> conf;
        // if (vlvl > 0) {
        //  cout << "Dumping input configuration:"<< endl;
        //  cout << "===============================" << endl;
        //  cout << conf.dump(2) << endl;
        //  cout << "===============================" << endl;
        // }
        client_->send_json(conf);
        cfin.close();
      }
      catch (const std::ifstream::failure& e)
      {
        cout << "** Failure opening/reading the configuration file: " << e.what() << endl;
        exit(1);
      }
      catch( const json::exception& e) {
        cout << "Caught a JSON exception: " << e.what() << endl;
        exit(1);
      }

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
      //std::thread reader(&ctb_robot::receive_data, this);
      // Now loop for commands
      while(!stop_req_) {
        cout << "### Select command : " << endl;
        cout << " 1 : Init/Config" << endl;
        cout << " 2 : Start Run" << endl;
        cout << " 3 : Stop Run" << endl;
        cout << " 4 : Quit" << endl;

        //    cout << " 4 : Soft Reset" << endl;
        //    cout << " 5 : Hard Reset" << endl;

        std::cin >> command;
        switch(command) {
          case init:
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

    std::string output_filename_;
    std::string config_filename_;
  private:
    std::unique_ptr<ctb_data_receiver> data_receiver_;
    std::unique_ptr<ctb_client> client_;
    //bool run_receiver_;
    uint32_t data_timeout_usecs_;

    std::string penn_data_dest_host_;
    uint16_t penn_data_dest_port_;
    std::string penn_client_host_addr_;
    std::string penn_client_host_port_;

    bool is_running_;
    bool is_conf_;
    bool stop_req_;

};

int main(int argc, char**argv) {

  std::cout.setf(std::ios::unitbuf);
  size_t vlvl = 0;
  //std::string cfile = "ctb_config.json";
  std::string outfname = "ctb_output";
  std::string destination = "localhost:8991";
  std::string confname = "ctb_config.json";

  try {
    cxxopts::Options options(argv[0], " - command line options");
    options.add_options()
            ("c,config","Configuration file (JSON format)",cxxopts::value<std::string>())
            ("h,help", "Print help")
            ("v,verbosity","Verbosity level",cxxopts::value<size_t>(vlvl))
            ("o,output","Output file (ROOT)",cxxopts::value<std::string>())
            ("d,destination","CTB location in format <host>:<port> [default: localhost:8991]",cxxopts::value<std::string>())
            ;

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      printf("%s",options.help().c_str());
      exit(0);
    }

    if (result.count("config")) {
      printf("main:: --> Using %s config file\n",result["config"].as<std::string>().c_str());
      confname = result["config"].as<std::string>();
    } else {
      printf("main:: --> Using default config file [%s]\n",confname.c_str());
    }

    printf("main:: --> Verbosity level [%zu]\n",vlvl);

    if (result.count("output"))
    {
      printf("main:: Setting the output file to [%s]\n",result["output"].as<std::string>().c_str());
      outfname = result["output"].as<std::string>();
    } else {
      printf("Attempting to use the default output file [ctb_output]\n");
    }

    if (result.count("destination"))
    {
      printf("main:: Setting the CTB location to [%s]\n",result["destination"].as<std::string>().c_str());
      destination = result["destination"].as<std::string>();
    } else {
      printf("Using default CTB destination. This is likely to fail since there is no ROOT on the CTB [localhost:8991]\n");
    }


  }
  catch( const cxxopts::OptionException& e)
  {
    printf("main:: ** Error parsing options: %s\n",e.what());
    exit(1);
  }

  try {
    std::string host = "localhost";
//    uint16_t port = 8991;
    std::string port = "8991";
    size_t colon_pos = destination.find(':');
    if(colon_pos != std::string::npos) {
      host = destination.substr(0,colon_pos);
      port = destination.substr(colon_pos+1);
//      std::stringstream parser(portpart);
//      if( parser >> port )
//      {
//        printf("Port set to %hu\n",port);
//      }
//      else {
//        printf("Couldn't understand the port. Setting it to default 8991\n");
//      }

    }
    printf("Connecting to the CTB at [%s:%s]\n",host.c_str(),port.c_str());
    printf("main:: Starting robot...\n");
    //ctb_robot robot("128.91.41.238",8991);
    //    ctb_robot robot(host.c_str(),port);
    ctb_robot robot(host.c_str(),port,8992,1,5);
    robot.output_filename_ = outfname;
    robot.config_filename_ = confname;
    printf("main:: Starting the loop...\n");
    robot.run();
    printf("main:: All done...\n");


    //  try {
    //  cout << "Starting robot..." << endl;
    //  //ctb_robot robot("128.91.41.238",8991);
    //  ctb_robot robot("128.91.41.238","8991",8992,1,5);
    //  cout << "Starting the loop..." << endl;
    //  robot.run();
    //  cout << "All done" << endl;
  }
  catch(json::exception &e) {
    printf("ERROR: Caught a JSON exception: %s\n",e.what());
  }
  catch(std::exception &e) {
    printf("ERROR: Caught a STL exception: %s\n",e.what());
  }
  catch(...) {
    printf("ERROR: Caught an unknown exception\n");
  }


  return 0;
}

