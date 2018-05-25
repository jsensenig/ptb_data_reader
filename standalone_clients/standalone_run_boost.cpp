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
//#include "cxxopts.hpp"
#include "optionparser.h"
#define USE_OPTIONPARSER 1
#undef USE_CXXOPTS



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

/** \class ctb_robot Wrapper class to control the execution of the clients
 * This object does ntot communicate directly with the CTB. Instead it uses the ctb_client and
 * ctb_data_receiver objects to control and receive data from the CTB
 *
 * I use the old fashioned printf instead of cout because cout is not thread safe
 * due to caching. There are ways around it, but it ends up just being simpler
 * to use printf that is uncached and therefore far less likely to mangle the output
 *
 */
class ctb_robot {
  public:
    ctb_robot(const std::string &ptb_host = "localhost",const std::string &ptb_port = "8991",
        const uint16_t &reader_port = 8992, const int &debug_level = 1,
        const uint32_t &tick_period_usecs = 5 )
  : data_receiver_(nullptr),
    exit_req_(false), is_running_(false),is_conf_(false),
    data_timeout_usecs_(100000),
    penn_data_dest_host_("localhost"),penn_data_dest_port_(reader_port),
    penn_client_host_addr_(ptb_host),penn_client_host_port_(ptb_port)


  {
      // -- Create the client Right at the constructor, since this class is just a wrapper over the
      // client itself
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
      printf("ctb_robot::send_reset : Sending a reset\n");
      client_->send_command("HardReset");

      is_running_ = false;
      is_conf_ = false;
    }

    void send_start() {
      if (!is_conf_) {
        printf("ctb_robot::send_start : ERROR: Can't start a run without configuring first\n");
        return;
      }

      // -- Launching a receiver
      printf("ctb_robot::send_start : Creating a receiver on port %hu...\n",penn_data_dest_port_);
      if (!data_receiver_ ) {
        data_receiver_ =
            std::unique_ptr<ctb_data_receiver>(new ctb_data_receiver(penn_data_dest_port_,1));

        data_receiver_->set_output_file(output_filename_);
	data_receiver_->set_stop_delay(1000000); //usec
        // Sleep for a short while to give time for the DataReceiver to be ready to
        // receive connections
        usleep(500000);
      }

      printf("ctb_robot::send_start : Sending a start run\n");
      // Start the data receiver
      data_receiver_->start();
      // Send start command to PENN

      client_->send_command("StartRun");
      if (client_->exception()) {
        printf("ctb_robot::send_start : ERROR: Exception caught sending a start run\n");

        exit(1);
      }
      is_running_ = true;
    }

    void send_stop() {
      printf("ctb_robot::send_stop : Sending a stop run\n");
      client_->send_command("StopRun");

      sleep(1);

      // Stop the data receiver.
      data_receiver_->stop();

      is_running_ = false;
    }

    void send_config() {
      bool has_exception = false;
      printf("ctb_robot::send_config : Resetting before configuring\n");
      send_reset();

      printf("ctb_robot::send_config : Sending config\n");
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
        printf("ctb_robot::send_config : ** Failure opening/reading the configuration file: %s\n",e.what());
        has_exception = true;
      }
      catch( const json::exception& e) {
        printf("ctb_robot::send_config : Caught a JSON exception: %s\n",e.what());
        has_exception = true;
      }
      catch(const std::exception &e)
      {
        printf("ctb_robot::send_config : Caught a STL exception: %s\n",e.what());
        has_exception = true;
      }
      catch(...)
      {
        printf("ctb_robot::send_config : Caught unknown exception.Aborting and resetting.\n");
        has_exception = true;
      }
      if (has_exception) {
        printf("ctb_robot::send_config : An exception was caught. Aborting and resetting.\n");
        send_stop();
        send_reset();
        is_conf_ = false;
        return;
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
        exit_req_ = true;
      }
    }

    void run() {
      enum commands {reset=0,init=1,start=2,stop=3,quit=4};
      // Set the stop condition to false to wait for the data
      //std::signal(SIGINT, ctb_robot::static_sig_handler);
      std::string rcommand;
      std::ifstream cfin;
      json conf;
      while(!exit_req_) {
        printf("\n\n### Select command : \n 0: Reset Board\n 1 : Init\n 2 : Start Run\n 3 : Stop Run\n 4 : Quit\n\n");

        std::cin >> rcommand;
        if (!isdigit (rcommand.c_str()[0])) {
          printf("ctb_robot::run : ERROR: Wrong option!\n");
          continue;
        }
        int command = stoi(rcommand);
        switch(command) {
          case reset:
            send_reset();
            break;
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
            std::this_thread::sleep_for(std::chrono::seconds(2));
            //reader.
            //reader.join();
            break;
          default:
            printf("ctb_robot::run : Wrong option (%d)\n\n",command);
            break;
        }
      }
      printf("ctb_robot::run : ### Stop requested.\n");
      // Wait for the reading function to return

    }

    std::string output_filename_;
    std::string config_filename_;
  private:
    std::unique_ptr<ctb_data_receiver> data_receiver_;
    std::unique_ptr<ctb_client> client_;


    bool exit_req_;
    bool is_running_;
    bool is_conf_;

    uint32_t data_timeout_usecs_;

    std::string penn_data_dest_host_;
    uint16_t penn_data_dest_port_;
    std::string penn_client_host_addr_;
    std::string penn_client_host_port_;

};

int main(int argc, char**argv) {

  std::cout.setf(std::ios::unitbuf);
  size_t vlvl = 0;
  //std::string cfile = "ctb_config.json";
  std::string outfname = "ctb_output";
  std::string destination = "localhost:8991";
  std::string confname = "ctb_config.json";

#ifdef USE_CXXOPTS

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
      printf("main:: Attempting to use the default output file [ctb_output]\n");
    }

    if (result.count("destination"))
    {
      printf("main:: Setting the CTB location to [%s]\n",result["destination"].as<std::string>().c_str());
      destination = result["destination"].as<std::string>();
    } else {
      printf("main:: Using default CTB destination. This is likely to fail since there is no ROOT on the CTB [localhost:8991]\n");
    }


  }
  catch( const cxxopts::OptionException& e)
  {
    printf("main:: ** Error parsing options: %s\n",e.what());
    //printf("%s",options.help().c_str());

    exit(1);
  }
  catch(...)
  {
    printf("main:: Caught some other unexpected exception.\n");
    exit(1);
  }

#elif defined(USE_OPTIONPARSER)


  struct Arg: public option::Arg
  {
      static void printError(const char* msg1, const option::Option& opt, const char* msg2)
      {
        fprintf(stderr, "%s", msg1);
        fwrite(opt.name, opt.namelen, 1, stderr);
        fprintf(stderr, "%s", msg2);
      }

      static option::ArgStatus Unknown(const option::Option& option, bool msg)
      {
        if (msg) printError("Unknown option '", option, "'\n");
        return option::ARG_ILLEGAL;
      }

      static option::ArgStatus Required(const option::Option& option, bool msg)
      {
        if (option.arg != 0)
          return option::ARG_OK;

        if (msg) printError("Option '", option, "' requires an argument\n");
        return option::ARG_ILLEGAL;
      }

      static option::ArgStatus NonEmpty(const option::Option& option, bool msg)
      {
        if (option.arg != 0 && option.arg[0] != 0)
          return option::ARG_OK;

        if (msg) printError("Option '", option, "' requires a non-empty argument\n");
        return option::ARG_ILLEGAL;
      }

      static option::ArgStatus Numeric(const option::Option& option, bool msg)
      {
        char* endptr = 0;
        if (option.arg != 0 && strtol(option.arg, &endptr, 10)){};
        if (endptr != option.arg && *endptr == 0)
          return option::ARG_OK;

        if (msg) printError("Option '", option, "' requires a numeric argument\n");
        return option::ARG_ILLEGAL;
      }
  };


  enum  optionIndex { UNKNOWN, CONFIG, HELP, VERBOSITY, OUTPUT, DESTINATION};
  const option::Descriptor usage[] =
  {
    {UNKNOWN,     0, "","",           Arg::None,        "USAGE: standalone_run_root [options]\n\n"
    "Options:" },
    {CONFIG,      0,"c","config",     Arg::Required,    "-c <file>,       --config=<file>          \tConfiguration File."},
    {HELP,        0,"h","help",       Arg::None,        "-h,              --help                   \tPrint usage and exit." },
    {VERBOSITY,   0,"v","verbose",    Arg::Numeric,     "-v <number>,     --verbose=<number>       \tSet verbosity level." },
    {OUTPUT,      0,"o","output",     Arg::Required,    "-o <file>,       --output=<file>          \tSet ROOT output file name (without extension) [default: ctb_output]." },
    {DESTINATION, 0,"d","destination",Arg::Required,    "-d <host:port>,  --destination=<host:port>\tSet destination host/IP and port [default: localhost:8991]." },
    {UNKNOWN,     0,"", "",           option::Arg::None,"\nExamples:\n"
        "  standalone_run_root -o my_output \n"
        "  standalone_run_root -c ctb_config.json -o output.root\n" },
        {0,0,0,0,0,0}
  };

  argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present
  option::Stats  stats(usage, argc, argv);
  std::vector<option::Option> options(stats.options_max);
  std::vector<option::Option> buffer(stats.buffer_max);
  option::Parser parse(usage, argc, argv, &options[0], &buffer[0]);

  if (parse.error())
  {
    printf("main:: Failed to parse options.\n");
    return 1;
  }

  if (options[HELP] || argc == 0)
  {
    int columns = getenv("COLUMNS")? atoi(getenv("COLUMNS")) : 80;
    option::printUsage(fwrite, stdout, usage, columns);
    return 0;
  }

  for (int i = 0; i < parse.optionsCount(); ++i)
  {
    option::Option& opt = buffer[i];
    fprintf(stdout, "Argument #%d is ", i);
    switch (opt.index())
    {
      case HELP:
        // not possible, because handled further above and exits the program
      case CONFIG:
        fprintf(stdout, "main:: Using %s config file\n", opt.arg);
	confname = opt.arg;
        break;
      case VERBOSITY:
        fprintf(stdout, "main:: Setting verbosity to '%s'\n", opt.arg);
        vlvl = atoi(opt.arg);
        break;
      case OUTPUT:
        fprintf(stdout, "main:: Setting output file to '%s'\n", opt.arg);
        outfname = opt.arg;
        break;
      case DESTINATION:
        fprintf(stdout, "main:: Setting CTB host to '%s'\n", opt.arg);
        destination = opt.arg;
        break;
      case UNKNOWN:
        // not possible because Arg::Unknown returns ARG_ILLEGAL
        // which aborts the parse with an error
        break;
    }
  }

  for (option::Option* opt = options[UNKNOWN]; opt; opt = opt->next())
    std::cout << "Unknown option: " << std::string(opt->name,opt->namelen) << "\n";

  for (int i = 0; i < parse.nonOptionsCount(); ++i)
    std::cout << "Non-option #" << i << ": " << parse.nonOption(i) << "\n";

#else
#error "Couldn't find a matching option parser. Program will refuse to compile"
#endif

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
    printf("main:: Connecting to the CTB at [%s:%s]\n",host.c_str(),port.c_str());
    printf("main:: Starting robot...\n");
    //ctb_robot robot("128.91.41.238",8991);
    //    ctb_robot robot(host.c_str(),port);
    ctb_robot robot(host.c_str(),port,8992,vlvl,5);
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

