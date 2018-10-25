/*
 * ctb_ctrl_client.cc
 *
 *  Created on: Oct 24, 2018
 *      Author: nbarros
 */
// -- STD includes
#include <ctime>
#include <algorithm>

#include "json.hpp"
#include "optionparser.h"

//FIXME: Add header to parse command line options
//#include ""

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

#include <boost/asio.hpp>

using json = nlohmann::json;
using boost::asio::ip::tcp;

//#define DEBUG 1
#undef DEBUG
//#include <boost/bind.hpp>
//#include <boost/smart_ptr.hpp>
//#include <boost/asio.hpp>
//#include <boost/thread/thread.hpp>


// -- Define the essential to run a network client
// for now play with a simple enum

// -- global variable that holds the time
char g_time[128];
std::string g_ctb_ip = "10.73.138.28";
//std::string g_ctb_ip = "128.91.41.224";
//const char *g_ctb_ip = "localhost";
const char *g_ctb_port = "8990";

char* mtime()
{
  time_t t = time(NULL);
  struct tm * p = localtime(&t);
  strftime(g_time, 128, "%c", p);
  return g_time;
}

void communicate(json &c, json &r)
{
  boost::system::error_code error;
  boost::asio::io_service io_service;
  tcp::resolver resolver(io_service);
  tcp::resolver::query query(tcp::v4(), g_ctb_ip.c_str(),g_ctb_port);
  tcp::resolver::iterator iterator = resolver.resolve(query);
  tcp::socket s(io_service);
  boost::asio::connect(s, iterator);

  boost::asio::streambuf request;
  std::ostream smsg(&request);
  smsg << c.dump(0);
  boost::asio::write(s, request,error);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  // Read the response status line. The response streambuf will automatically
  // grow to accommodate the entire line. The growth may be limited by passing
  // a maximum size to the streambuf constructor.
  boost::asio::streambuf response;
  size_t nbytes = boost::asio::read_until(s, response, '}', error);
  if (error)
  {
    printf("%s : Failed with error %u : %s\n",mtime(),error.value(),error.message().c_str());
  }
#ifdef DEBUG
  else {
    printf("%s : Received %zu bytes\n",mtime(),nbytes);
  }
#else
  (void)nbytes;
#endif

  std::istream response_stream(&response);
  std::string resp;
  std::getline(response_stream,resp,'}');
  if (resp.size())
  {
    resp += "}";
  } else {
    printf("%s : Failed to receive the response from the CTB. Please try again.\n",mtime());
    r["status"] = "ERROR";
    r["message"] = "Lost connection while processing request. Please retry.";
    return;
  }
  // -- put the delimiter there again
  json a = json::parse(resp);
#ifdef DEBUG
  printf("%s : Received answer: \n",mtime());
  printf("%s\n",a.dump(2).c_str());
#endif
  r = a;
}

void check_timing_status()
{
  printf("%s : Checking status of timing endpoint\n",mtime());
  json a;
  json c;
  c["command"] = "check_timing";
  communicate(c,a);

  if (a.empty() || a.is_null())
  {
    printf("%s : Received an invalid answer. Contact a CTB expert.\n",mtime());
  }
  // -- there is an answer. Print it
  if (a.at("status") == "OK")
  {
    // good answer
    printf("%s : Timing status register :\n\n",mtime());
    uint32_t val = a.at("message").get<uint32_t>();
    printf("\t Full register : 0x%X\n",val);
    printf("\t Timing state  : 0x%X\n\n",(val >> 28));
  } else {
    printf("%s : Failed to read timing endpoint status.\n\n",mtime());
    printf("Message : %s\n",a.at("message").get<std::string>().c_str());
    if (a.find("extra") != a.end()) {
      // there is an extra message
      printf("Additional information : %s\n",a.at("extra").get<std::string>().c_str());
    }
  }
  printf("\n\n");
}

void check_registers()
{
  printf("%s : Checking status of timing endpoint\n",mtime());
  json a;
  json c;
  c["command"] = "check_registers";
  communicate(c,a);

  if (a.empty() || a.is_null())
  {
    printf("%s : Received an invalid answer. Contact a CTB expert.\n",mtime());
  }
  // -- there is an answer. Print it
  if (a.at("status") == "OK")
  {
    printf("%s : Status of configuration registers :\n\n",mtime());
    std::vector<uint32_t> regs = a.at("message");
    std::vector<uint32_t> nzero;
    // -- do a pretty print of the registers
    // -- we know there are 256, so we can split them into 6 columns of 42 entries
    // -- alternatively we can not show the status of registers above 150
    // and reduce the output to 6 columns of 25 entries
    const size_t nregs = 150; // number of registers *TO PRINT*
    const size_t ncols = 6;
    size_t nrows = nregs/ncols;
    size_t e = 0;
    for (size_t j = 0; j < nrows; j++)
    {
      for (size_t i = 0; i < ncols; i++ )
      {
        // entry is tricky. I want to print
        // 0  25  50  75
        e = i*nrows+j;
        printf("%02zu=[%08X]  ",e,regs.at(e));
        if (regs.at(e) != 0x0)
        {
          nzero.push_back(e);
        }
      }
      printf("\n");
    }
    // sort the entries
    // Sorting the int vector
    if (nzero.size())
    {
      std::sort(nzero.begin(), nzero.end());
      printf("\n%s : List of non-zero registers:\n\n",mtime());
      for (size_t i = 0; i < nzero.size(); i++)
      {
        printf(" %u,",nzero.at(i));
      }
      printf("\n");
    }
  } else {
    printf("%s : Failed to read configuration registers.\n\n",mtime());
    printf("Message : %s\n",a.at("message").get<std::string>().c_str());
    if (a.find("extra") != a.end()) {
      // there is an extra message
      printf("Additional information : %s\n",a.at("extra").get<std::string>().c_str());
    }
  }
  printf("\n\n");

}

void reset_endpoint(bool force)
{
  printf("%s : Resetting CTB timing endpoint\n",mtime());
  json a;
  json c;
  if (force)
  {
    c["command"] = "force_reset_timing";
  } else {
    c["command"] = "reset_timing";
  }
  communicate(c,a);

  if (a.empty() || a.is_null())
  {
    printf("%s : Received an invalid answer. Contact a CTB expert.\n",mtime());
  }
  // -- there is an answer. Print it
  if (a.at("status") == "OK")
  {
    printf("%s : Timing endpoint successfully reset. Final state :\n\n",mtime());
    uint32_t val = a.at("message").get<uint32_t>();
    printf("\t Full register : 0x%X\n",val);
    printf("\t Timing state  : 0x%X\n\n",(val >> 28));
  } else {
    printf("%s : Failed to reset the CTB timing endpoint.\n\n",mtime());
    printf("Message : %s\n",a.at("message").get<std::string>().c_str());
    if (a.find("extra") != a.end()) {
      // there is an extra message
      printf("Additional information : %s\n",a.at("extra").get<std::string>().c_str());
    }
  }
  printf("\n\n");

}

int main(int argc, char**argv)
{

  printf("%s : Starting\n",mtime());

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


  enum  optionIndex { UNKNOWN, CTB, HELP, TSTATUS, RSTATUS, RESET, FORCE};
  const option::Descriptor usage[] =
  {
    {UNKNOWN,     0, "","",           Arg::None,        "USAGE: ctb_control [options]\n\n"
    "Options:" },
    {CTB,         0,"c","ctb",        Arg::Required,    "-c <CTB ip>,     --ctb=<CTB ip>          \tDestination of the CTB (optional)."},
    {HELP,        0,"h","help",       Arg::None,        "-h,              --help                   \tPrint usage and exit." },
    {TSTATUS,     0,"s","state",      Arg::None,        "-s,              --state                  \tCheck timing endpoint state." },
    {RSTATUS,     0,"d","dump",       Arg::None,        "-d,              --dump                   \tDump status of all registers." },
    {RESET,       0,"r","reset",      Arg::None,        "-r,              --reset                  \tReset the timing endpoint." },
    {FORCE,       0,"f","force",      Arg::Required,    "-f <password>,   --force=<password>       \tForce the reset even if the board *thinks* it is taking data. WARNING: Will cause CTB operation to stop. This operation requires a super secret password."},
    {UNKNOWN,     0,"", "",           option::Arg::None,"\nExamples:\n"
        "  ctb_control -c localhost -s   : print the timing status assuming the CTB is running on localhost\n"
        "  ctb_control -d                : Dump the status of all configuration registers in the production CTB\n"
        "  ctb_control -d                : Reset the CTB\n"
        "  ctb_control -d -f             : Force reset the CTB, even if the registers show it is running\n" },
        {0,0,0,0,0,0}
  };

  argc -= (argc>0); argv += (argc>0); // skip program name argv[0] if present
  option::Stats stats(usage,argc,argv);
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
    int columns = getenv("COLUMNS")? atoi(getenv("COLUMNS")) : 100;
    option::printUsage(fwrite, stdout, usage, columns);
    return 0;
  }

  enum action{none=0,dump,tstat,treset};
  action a = none;
  bool force = false;
  for (int i = 0; i < parse.optionsCount(); ++i)
  {
    option::Option& opt = buffer[i];
    printf("Argument #%d is ", i);
    switch (opt.index())
    {
      case HELP:
        // not possible, because handled further above and exits the program
      case CTB:
        printf("main:: Setting CTB host to %s\n", opt.arg);
        g_ctb_ip = opt.arg;
        break;
      case TSTATUS:
        printf("main:: Checking timing status\n");
        if (a != none) {
          printf("ERROR: Cannot request more than one action (dump, check, reset) at once!\n");
          return 0;
        }
        a = tstat;
        break;
      case RSTATUS:
        printf("main:: Checking status of all registers\n");
        if (a != none) {
          printf("ERROR: Cannot request more than one action (dump, check, reset) at once!\n");
          return 0;
        }
        a=dump;
        break;
      case RESET:
        printf("main:: Resetting the CTB timing endpoint\n");
        if (a != none) {
          printf("ERROR: Cannot request more than one action (dump, check, reset) at once!\n");
          return 0;
        }
        a=treset;
        break;
      case FORCE:
        printf("main:: Enable the use of the Force\n");
        force = true;
        break;
      case UNKNOWN:
        // not possible because Arg::Unknown returns ARG_ILLEGAL
        // which aborts the parse with an error
        break;
    }
  }

  // deal with unknown options
  for (option::Option* opt = options[UNKNOWN]; opt; opt = opt->next())
    printf("Unknown option: [%s]\n",opt->name);

  for (int i = 0; i < parse.nonOptionsCount(); ++i)
    printf("Non-option #%d : %s\n",i,parse.nonOption(i));


  printf("%s : Initiating operation.\n",mtime());

  switch(a)
  {
    case dump:
      check_registers();
            break;
    case tstat:
      check_timing_status();
      break;
    case treset:
      reset_endpoint(force);
      break;
    case none:
      printf("%s: No command was specified. One of -s, -d or -r must be used.\n");
      break;
    default:
      break;
  }
//
//  if (argc != 2)
//  {
//    printf("%s : Usage: ctb_control <option>\n",mtime());
//    return 0;
//  }
//
//  switch(atoi(argv[1]))
//  {
//    case 0: // read timing status
//      check_timing_status();
//      break;
//    case 1: //
//      check_registers();
//      break;
//    case 2:
//      reset_endpoint(false);
//      break;
//    case 3:
//      reset_endpoint(true);
//      break;
//    default:
//      printf("%s : Options : \n\t0 : Check timing status\n\t1 : Check all configuration registers\n\t2 : Reset the timing endpoint\n\t3 : Force reset the timing endpoint\n",mtime());
//  }

  return 0;
}
