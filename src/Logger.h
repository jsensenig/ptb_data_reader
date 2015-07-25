/*
 * Logger.h
 *
 *  Created on: Jun 3, 2015
 *      Author: nbarros
 */

#ifndef MYLIB_LOGGER_H_
#define MYLIB_LOGGER_H_
// -- STL includes
#include <iostream>
#include <mutex>

//-- A few helpful defines
#define ERRLINE_SEGMENT_1(line)   #line
#define ERRLINE_SEGMENT_2(line)   ERRLINE_SEGMENT_1(line)
// Use instead a compilation macro
//#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#ifdef Log
#undef Log
#endif
#ifndef __FILENAME__
#define __FILENAME__ __FILE__
#endif
#define Log(sev,fmt,...) Logger::message(Logger::sev,__FILENAME__ "(" ERRLINE_SEGMENT_2(__LINE__) ")",fmt,##__VA_ARGS__)


class Logger {
public:
  /// \enum severity
  /// \note A message with fatal severity will make the program die more or less gracefully
  /// Further description inline
  enum severity {
    verbose=-1, ///< Highest level of verbosity. Everything possible is printed. Can be overwhelming.
    debug,      ///< Messages about the program flow are printed.
    info,       ///< Default level. Nothing wrong, Just informative messages
    warning,    ///< Something that might cause trouble later on occurred. There is a result but might not be the expected one.
    error,      ///< A serious problem occurred. Execution continues to let the code deal with the problem.
    fatal       ///< Message describes something that prevents further execution. The program is terminated.
  };


  // -- Members
  static severity GetSeverity() {return _sev;}
  static void SetSeverity(severity sev) {_sev = sev;}
  static void SetOutputFile(std::ostream &out = std::cout) { _ostream = &out;};

  static void endlog(std::ostream& s);
  static void message(Logger::severity sev, const char* where, const char* fmt,...);

private:
  Logger();
  virtual ~Logger();

  /// Severity above which messages are to be printed
  static severity _sev;

  /// Output stream
  static std::ostream* _ostream;
  // Error stream
  //static std::ostream* _estream;

  /// NULL stream
  static std::ostream* _nstream;

  /// Aux flag
  static bool _print;
  static bool _color;

  ///Private member to aux the conversion of the severity level into a string
  static const char* tostr(severity);

  static std::mutex print_mutex_;



};
//std::ostream& endlog(std::ostream& s);


#endif /* MYLIB_LOGGER_H_ */
