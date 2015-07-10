/**
 * \class ConfigServer
 * \brief Implements a configuration manager using XML to parse the options.
 * ConfigServer.hh
 *
 *  Created on: Jun 2, 2015
 *      Author: Nuno Barros
 *  Description
 */

#ifndef CONFIGSERVER_H_
#define CONFIGSERVER_H_

#include <string>
#include <map>
#include <list>
#include <stdexcept>
#include <sstream>

#include "pugixml.hpp"

// -- C headers
extern "C" {
#include <pthread.h>         // For POSIX threads
#include <stdint.h>
};

class TCPSocket;
class PTBManager;

//// Some custom exceptions
//class config_exception : public std::runtime_error
//{
//public:
//  config_exception(std::string msg) : std::runtime_error("configuration error"), message_(msg) {};
//  virtual ~config_exception() _NOEXCEPT { };
//  virtual const char* what() const throw() {
//    cnvt_.str("");
//    cnvt_ << std::runtime_error::what() << " : " << message_ ;
//    return cnvt_.str().c_str();
//  }
//private:
//  static std::ostringstream cnvt_;
//  std::string message_;
//};
//
//// Some custom exceptions
//class command_exception : public std::runtime_error
//{
//public:
//  command_exception(std::string msg) : std::runtime_error("command error"), message_(msg) {};
//  virtual ~command_exception() _NOEXCEPT { };
//  virtual const char* what() const throw() {
//    cnvt_.str("");
//    cnvt_ << std::runtime_error::what() << " : " << message_ ;
//    return cnvt_.str().c_str();
//  }
//private:
//  static std::ostringstream cnvt_;
//  std::string message_;
//};
//
//std::ostringstream config_exception::cnvt_;
//std::ostringstream command_exception::cnvt_;




class ConfigServer {
public:

	inline static ConfigServer* get();
	virtual ~ConfigServer();

	void CheckInstances() const;

	// Main client thread.
	static void HandleTCPClient(TCPSocket *sock);
	static void *ThreadMain(void *clntSock);
	static void *listen(void *arg);

	void RegisterDataManager(PTBManager*manager);

  pthread_t getThreadId() const {
    return thread_id_;
  }

  void Shutdown(bool force = false);
protected:
  static void ProcessTransmission(const char* buffer);
  static void ProcessConfig(pugi::xml_node &config);
	static void ProcessCommand(pugi::xml_node &command);

	static PTBManager *data_manager_;
private:
	// Disable constructors and assignment
	ConfigServer();
	ConfigServer(const ConfigServer &other);
	void operator=(const ConfigServer &other);



	// Self thread of the server.
	static pthread_t thread_id_;
	// parameters
	static unsigned int num_instances_;
	static unsigned int port_;
//	static std::string cfg_buffer_;
	static bool shutdown_; ///< Handler for destroying object.
	static std::string tcp_buffer_;

	static std::list<const char*> queue_;

//	static std::map<std::string, int> init_map() {
//	  std::map<std::string, int> tmp_map;
//	  tmp_map.insert(std::map<std::string, int>::value_type("RUNSTART",RUNSTART));
//    tmp_map.insert(std::map<std::string, int>::value_type("RUNSTOP",RUNSTOP));
//	  return tmp_map;
//	}

};

inline ConfigServer* ConfigServer::get() {
	static ConfigServer conf;
	num_instances_++;
	return &conf;
}

#endif /* CONFIGSERVER_H_ */
