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
#include <algorithm>

//FIXME: Use a more robust socket library...for example boost seems sensible
#include "PracticalSocket.h"
#include "content.h"
#include "json.hpp"
// exclude unsupported compilers
//#if defined(__clang__)
//#if (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__) < 30400
//#include "optionparser.h"
//#define USE_OPTIONPARSER 1
//#undef USE_CXXOPTS
//#else
//#include "cxxopts.hpp"
//#define USE_CXXOPTS 1
//#undef USE_OPTIONPARSER
//#endif
//#elif defined(__GNUC__) && !(defined(__ICC) || defined(__INTEL_COMPILER))
//#if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__) < 40900
//#include "optionparser.h"
//#define USE_OPTIONPARSER 1
//#undef USE_CXXOPTS
//#else
//#include "cxxopts.hpp"
//#define USE_CXXOPTS 1
//#undef USE_OPTIONPARSER
//#endif
//#endif

#include "optionparser.h"
#define USE_OPTIONPARSER 1
#undef USE_CXXOPTS

// -- ROOT headers (for output)
#include <TFile.h>
#include <TTree.h>
#include <boost/lockfree/spsc_queue.hpp>

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
  : exit_req_(false), is_running_(false),is_conf_(false),
    outroot_(NULL),outtree_(NULL),
    outfilename_("ctb_output"), archiver_(), receiver_(),archiver_ready_(false)
  {
      client_sock.connect(host,port);
  }
    virtual ~ctb_robot() {};

    void send_reset() {
      printf("send_reset:: Sending a reset\n");
      const char *cmd = "{\"command\":\"HardReset\"}";
      client_sock.send(cmd,28);
      client_sock.recv(answer_,1024);
      try {
	json answer = json::parse(answer_);
	printf("send_reset:: Received answer [%s]\n",answer.dump(2).c_str());
      }
      catch(...){}
      std::fill(answer_, answer_+1024, 0);
      //answer_[0]='\0';

      is_running_.store(false);
      is_conf_.store(false);
    }

    void send_start() {
      if (!is_conf_.load()) {
        printf("send_start:: ERROR: Can't start a run without configuring first\n");
      }
      printf("send_start:: Launching reader thread...\n");

      is_running_.store(true);

      receiver_ = std::thread(&ctb_robot::receive_data, this);
      std::this_thread::sleep_for(std::chrono::seconds(1));

      printf("send_start:: Opening output file prior to start running.\n");
      archiver_ = std::thread(&ctb_robot::store_root, this);
      //archiver_ = archiver;
      printf("send_start:: Waiting for archiving thread to flag readyness...\n");
      while(!archiver_ready_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      printf("send_start:: Archiver ready!\n");

      printf("send_start:: Sending a start run\n");
      const char *cmd = "{\"command\":\"StartRun\"}";
      client_sock.send(cmd,27);
      client_sock.recv(answer_,1024);
      try {
	json answer = json::parse(answer_);
	printf("send_start:: Received answer [%s]\n",answer.dump(2).c_str());
      }
      catch(...){}

      //      printf("send_start:: Received answer [%s]\n",answer_);
      std::fill(answer_, answer_+1024, 0);
      //answer_[0]='\0';
    }

    void send_stop() {
      printf("send_stop:: Sending a stop run\n");
      const char *cmd = "{\"command\":\"StopRun\"}";
      client_sock.send(cmd,27);
      client_sock.recv(answer_,1024);
      try {
	json answer = json::parse(answer_);
	printf("send_stop:: Received answer [%s]\n",answer.dump(2).c_str());
      }
      catch(...){}

      //      printf("send_stop:: Received answer [%s]\n",answer_);
      std::fill(answer_, answer_+1024, 0);
      //answer_[0]='\0';
      is_running_.store(false);

      // -- kill the threads
      // -- first the receiver
      printf("send_stop:: Joining client threads\n");
      if ( receiver_.joinable() == false ) {
        printf("Thread receiver_ is not joinable?!\n");
      } else {
        printf("Joining receiver\n");
        receiver_.join();
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
      if ( archiver_.joinable() == false ) {
        printf("Thread archiver_ is not joinable?!\n");
      } else {
        printf("Joining archiver\n");
        archiver_.join();
      }
      //archiver_.join();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      printf("Threads joined.\n");
    }

    void send_config() {

      printf("send_config:: Sending config\n");
      if (!is_conf_.load()) {
        printf("send_config:: Resetting before configuring\n");
        send_reset();
      }
      send_reset();
      client_sock.send(g_config.c_str(),g_config.size());
      client_sock.recv(answer_,1024);
      try {
	json answer = json::parse(answer_);
	printf("send_config:: Received answer [%s]\n",answer.dump(2).c_str());
      }
      catch(...){}

      //      printf("send_config:: Received answer [%s]\n",answer_);
      std::fill(answer_, answer_+1024, 0);
      //answer_[0]='\0';
      is_conf_.store(true);

    }

    void process_quit()
    {
      if (is_running_.load()) {
        // if we are taking data, stop it
        send_stop();
      } else {
        // If we are not, stop execution
        exit_req_.store(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));

      }
    }

    void run() {
      enum commands {init=1,start=2,stop=3,quit=4};
      // Set the stop condition to false to wait for the data
      //std::signal(SIGINT, ctb_robot::static_sig_handler);
      //int command = -1;
      // Now make the receiving thread
      std::string rcommand;
      // Now loop for commands
      std::ifstream cfin;
      json conf;
      while(!exit_req_.load()) {
        printf("\n\n### Select command : \n 1 : Init\n 2 : Start Run\n 3 : Stop Run\n 4 : Quit\n\n");

        cin >> rcommand;
        if (!isdigit (rcommand.c_str()[0])) {
          printf("ERROR: Wrong option!\n");
          continue;
        }
        int command = stoi(rcommand);
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
              printf("** Failure opening/reading the configuration file: %s\n",e.what());
              exit(1);
            }
            catch( const json::exception& e) {
              printf("Caught a JSON exception: %s\n",e.what());;
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
            printf("Wrong option [%d]\n",command);
            break;
        }
      }
      printf("### Stop requested.\n");
      // Wait for the reading function to return

    }

    void receive_data() {

      // Important to avoid mangling of the output due to race conditions to the
      // IO buffers. The trick is to use unbuffered (ie. direct) output
      std::cout.setf(std::ios::unitbuf);
      printf("receive_data:: Starting receiving thread.\n");

      std::thread::id receiver_id_ = std::this_thread::get_id();
      std::ostringstream stream;
      stream << std::hex << receiver_id_ << " " << std::dec << receiver_id_;
      printf("receive_data:: Receiving thread: %s\n", stream.str().c_str());

      // Create a server that is meant to receive the data from the
      // PTB and keep dumping the contents into somewhere...like a binary file
      try{
        // Create a tcp server and listen to the connection
        TCPServerSocket servSock(8992);


        while (!exit_req_.load()) {
          // Accept a client
          printf("Opening the receiver socket\n");
          TCPSocket *sock = servSock.accept();
          printf("--> Received a client request.\n");

          // just wait for the run to be started
          while (!is_running_.load() && !exit_req_.load()) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
          }

          printf("receive_data:: Run started....\n");

          uint16_t bytes_collected= 0;
          ptb::content::tcp_header *header;
          ptb::content::word::word_t*word;

          //uint32_t header;
          uint8_t tcp_data[8192];
          size_t count = 0;
          bool timeout_reached = false;
          size_t tcp_body_size = 0;

          // FIXME: Receive the ethernet packets
          while (is_running_.load() && !exit_req_.load()) {

            // grab a header
            //cout << "Going to receive " << header->word.size_bytes << " bytes" << endl;
            bytes_collected = 0;
            //printf("Collecting data\n");

            while ((bytes_collected != header->word.size_bytes) && is_running_) {
              bytes_collected += sock->recv(&tcp_data[bytes_collected],header->word.size_bytes-bytes_collected);
            }
            //printf("have a header\n");

            //printf("Received %u bytes : %X\n",bytes_collected,*(reinterpret_cast<uint32_t*>(&tcp_data[0])));
            header = reinterpret_cast<ptb::content::tcp_header *>(tcp_data);
            count++;
            if (!(count%1000)) printf("receive_data:: Counting %u packets received...\n",count);

            // check the number of bytes in this packet
            //cout << "Expecting to receive " << header->word.packet_size << " bytes " << endl;
            tcp_body_size = header->word.packet_size;
            bytes_collected = 0;
            while (bytes_collected != tcp_body_size) {
              bytes_collected += sock->recv(&tcp_data[bytes_collected],(int)(tcp_body_size-bytes_collected));
            }

            uint8_t *data = NULL;
            uint32_t pos = 0;
            bool ret = false;
            while (pos < tcp_body_size) {
              buffer word;
              std::memcpy(word.data,&(tcp_data[pos]),16);

              ret = buffer_queue_.push(word);
              if (!ret) {
                printf("WARNING::: Failed to store the received data\n");
              }
              // advance pointer to the next word
              pos += 16;
            }
            /*
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
             */
          }
          printf("receive_data:: Left the receiving loop \n");
          delete sock;
          printf("receive_data:: Returning\n");
          return;
          //          if (!exit_req_.load()) {
          //            cout << "Going to start a new socket listening" << endl;
          //          }

        }

        // -- Don't hang in here. Disconnect the thing
        return;
      }
      catch(SocketException &e) {
        printf("receive_data:: Socket exception caught : %s\n",e.what());
      }
      catch(std::exception &e) {
        printf("receive_data:: STL exception caught : %s\n",e.what());
      }
      catch(...) {
        printf("receive_data:: Something went very wrong\n");
      }
      // Delete the connection
    }

    void store_root() {
      archiver_ready_.store(false);

      std::thread::id archiver_id_ = std::this_thread::get_id();
      printf("store_root:: Archiver thread spawned with ID %u\n",archiver_id_);
      // -- Check if the requested file exists
      unsigned int idx = 2;
      bool file_exists = false;
      std::string outname = outfilename_;
      outname += ".root";
      outroot_ = TFile::Open(outname.c_str());
      if (outroot_) {
        file_exists = true;
        while (outroot_ != NULL) {
          outroot_->Close();
          std::ostringstream outf;
          outf << outfilename_ << "_" << idx << ".root";
          outname = outf.str();
          idx++;
          outroot_ = TFile::Open(outname.c_str());
        }
        printf("store_root:: Found a filename that does not yet exist\n");
      }

      if (file_exists) {
        printf("store_root:: ERROR: Output file already exists. Writing output to [%s] instead\n",outname.c_str());
      } else {
        printf("store_root:: Writing contents to [%s]\n",outname.c_str());
      }
      printf("store_root:: Writing contents to [%s]\n",outname.c_str());
      outroot_ = TFile::Open(outname.c_str(),"RECREATE");
      //      if (!outroot_ || outroot_->IsZombie()) {
      //printf("store_root:: ERROR: Failed to create output file. Writing output to [%s]\n",outname.c_str());
      //      }
      // -- Build the structure for the triggers
      outroot_->cd();
      outtree_ = new TTree("data","CTB output data");
      unsigned int wtype;
      uint64_t tstamp;
      uint64_t payload;
      uint64_t crt_status;
      uint64_t beam_status;

      outtree_->Branch("type",&wtype);
      outtree_->Branch("timestamp",&tstamp,"timestamp/l");
      outtree_->Branch("payload",&payload,"payload/l");
      outtree_->Branch("beam_status",&beam_status,"beam_status/l");
      outtree_->Branch("crt_status",&crt_status,"crt_status/l");

      // -- Now enter the loop to store the data
      buffer word;
      printf("store_root:: Entering archiving loop\n");
      bool ret = true;
      archiver_ready_.store(true);

      while(is_running_.load() && !exit_req_.load())
      {

        if (buffer_queue_.empty()){
          std::this_thread::sleep_for (std::chrono::microseconds(10));
          continue;
        }
        ret = buffer_queue_.pop(word);
        if (ret == false) {
          printf("store_root:: Failed to acquire buffer...failure is imminent!\n");
          // -- should some sort of wait be put here?
          // Might hurt since it will require some sort of mutex
        }
        // -- If the word came, write it to disk
        ptb::content::word::ch_status_t *chs = NULL;
        ptb::content::word::feedback_t  *fbk = NULL;
        ptb::content::word::timestamp_t *ts = NULL;
        ptb::content::word::trigger_t   *trg = NULL;
        ptb::content::word::word_t      *wrd = NULL;
        wrd =reinterpret_cast<ptb::content::word::word_t*>(word.data);
        switch(wrd->word_type) {
          case ptb::content::word::t_fback:
            printf("store_root:: WARNING:: Feedback word. This is unexpected\n");
            break;
          case ptb::content::word::t_gt:
            trg = reinterpret_cast<ptb::content::word::trigger_t*>(word.data);
            wtype = trg->word_type;
            tstamp = trg->timestamp;
            payload = trg->trigger_word;
            outtree_->Fill();
            break;
          case ptb::content::word::t_lt:
            trg = reinterpret_cast<ptb::content::word::trigger_t*>(word.data);
            wtype = trg->word_type;
            tstamp = trg->timestamp;
            payload = trg->trigger_word;
            outtree_->Fill();
            break;
          case ptb::content::word::t_ts:
            ts = reinterpret_cast<ptb::content::word::timestamp_t*>(word.data);
            wtype = ts->word_type;
            tstamp = ts->timestamp;
            payload = 0;
            outtree_->Fill();
            break;
          case ptb::content::word::t_ch:
            chs = reinterpret_cast<ptb::content::word::ch_status_t*>(word.data);
            wtype = chs->word_type;
            tstamp = chs->timestamp;
            payload = chs->get_pds();
            crt_status = chs->get_crt();
            //crt_status = 0xffffffff;
            beam_status = chs->get_beam();
            outtree_->Fill();
            break;
          default:
            printf("store_root:: WARNING:: Unknown word type. Ignoring\n");
            break;
        }
      }
      printf("store_root::Stop request caught. Closing file.\n");
      outtree_->Write();
      outroot_->Close();
      delete outroot_;
      archiver_ready_.store(false);
    }

    std::string outfilename_;

  private:
    TCPSocket client_sock;

    // control variables
    std::atomic<bool> exit_req_;
    std::atomic<bool> is_running_;
    std::atomic<bool> is_conf_;
    std::atomic<bool> archiver_ready_;
    std::thread receiver_;
    std::thread archiver_;
    //    std::thread::id receiver_id_;
    //    std::thread::id archiver_id_;
    // Aux vars
    char answer_[1024];

    //std::string outfilename_;
    TFile *outroot_;
    TTree *outtree_;
    //uint8_t buffer[16];
    typedef struct buffer {
        uint8_t data[16];
    } buffer;
    boost::lockfree::spsc_queue<buffer, boost::lockfree::capacity<4096> > buffer_queue_;

};

int main(int argc, char**argv) {

  std::cout.setf(std::ios::unitbuf);
  size_t vlvl = 0;

  //  cout << "Some debugging..." << endl;
  //  cout << "Size of header : " << sizeof(ptb::content::word::header_t) << endl;
  //  cout << "Body : " << sizeof(ptb::content::word::body_t) << endl;
  //  cout << "Size of channel status : " << sizeof(ptb::content::word::payload::ch_status_t) << endl;

  std::string outfname = "ctb_output";
  std::string destination = "localhost:8991";

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
      cfile = result["config"].as<std::string>();    
    } else {
      printf("main:: --> Using default config file [%s]\n",cfile.c_str());
    }

    printf("main:: --> Verbosity level [%d]\n",vlvl);

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
        cfile = opt.arg;
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



  std::string host = "localhost";
  uint16_t port = 8991;
  size_t colon_pos = destination.find(':');
  if(colon_pos != std::string::npos) {
    host = destination.substr(0,colon_pos);
    std::string portpart = destination.substr(colon_pos+1);
    std::stringstream parser(portpart);
    if( parser >> port )
    {
      printf("Port set to %hu\n",port);
    }
    else {
      printf("Couldn't understand the port. Setting it to default 8991\n");
    }

  }
  printf("Connecting to the CTB at [%s:%hu]\n",host.c_str(),port);
  printf("main:: Starting robot...\n");
  //ctb_robot robot("128.91.41.238",8991);
  ctb_robot robot(host.c_str(),port);
  robot.outfilename_ = outfname;
  printf("main:: Starting the loop...\n");
  robot.run();
  printf("main:: All done...\n");

  return 0;
}

