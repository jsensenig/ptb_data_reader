/*
 * Logger.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: nbarros
 *  Disclaimer: This code is strongly inspired by the MaGe MGLogger.
 */

#include "Logger.h"

#include <sstream>
#include <cstdlib>
#include <unistd.h>


static struct nullstream:
    std::ostream {
      struct nullbuf: std::streambuf {
        int overflow(int c) {return traits_type::not_eof(c);}
      } m_sbuf;
      nullstream(): std::ios(&m_sbuf), std::ostream(&m_sbuf) {}
    } devnull;

Logger::severity Logger::_sev = Logger::info;
std::ostream* Logger::_ostream (&std::cout);
std::ostream* Logger::_estream (&std::cerr);
std::ostream* Logger::_nstream (&devnull);
bool Logger::_print = true;
// By default do not allow colored output
bool Logger::_color = false;

Logger::Logger() { }

Logger::~Logger() { }

std::ostream& Logger::message(Logger::severity sev, const char* where, const char*code){
  _print = true;
  if (sev >= _sev) {
    *_ostream << tostr(sev) << ":" << where << ":";
  } else {
    _print = false;
    return *_nstream;
  }

  // This part seems to be failing before the message is actually printed.
  if (sev == fatal) {
    *_ostream << ::endlog;
    ::exit(1);
    //::abort();
  }
  return *_ostream;
}

void Logger::endlog(std::ostream&){
  if(_print)*_ostream << std::endl;
}

const char* Logger::tostr(Logger::severity sev) {
  _color=isatty(fileno(stdout))?true:false;
  switch(sev) {
  case -1:
    // VERBOSE : No facyness.
    return "VERBOSE";
    break;
  case 0:
    // DEBUG (bold text)
    return (_color)?"\033[1mDEBUG\033[0m":"DEBUG";
    break;
  case 1:
    // INFO -- No fancyness either
    return "INFO";
    break;
  case 2:
    // WARNING: bold Black text on orange background
    return (_color)?"\033[1;37;43mWARNING\033[0m":"WARNING";
    break;
  case 3:
    // ERROR: white text on red
    return (_color)?"\033[1;37;41mERROR\033[0m":"ERROR";
    break;
  case 4:
    // ERROR: bold red text on white
    return (_color)?"\033[1;31mFATAL\033[0m":"FATAL";
    break;
  }
  return "INFO";
}


std::ostream& endlog(std::ostream& os) {
  Logger::endlog(os);
  return devnull;
}
