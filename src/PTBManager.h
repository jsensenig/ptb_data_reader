/*
 * PTBManager.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBMANAGER_H_
#define PTBMANAGER_H_

#include <map>
#include "pugixml.hpp"

// -- Forward declarations
class PTBReader;
class ConfigServer;

/**
 * \class PTBManager
 * \brief Responsible for the configuration registers of the PTB.
 *
 * This class is responsible for passing the configuration into the PTB. Essentially it is
 * responsible for everything except the data readout.
 * Uses the Xilinx GPIO driver to set the configuration registers. This is more than good enough.
 */
class PTBManager {
public:

  enum Status {
    RUNNING = 0,
    IDLE = 1
  };

  enum Command {RUNSTART=0,
                RUNSTOP = 1
               };

  PTBManager();
  virtual ~PTBManager();

  const PTBReader* getReader() const {
    return reader_;
  }

  void setReader(PTBReader* reader) {
    reader_ = reader;
  }

  Status getStatus() const {
    return status_;
  }

  void setStatus(Status status) {
    this->status_ = status;
  }

  void ExecuteCommand(const char* cmd) throw(std::exception);
  // Receive the configuration as
  // Passed by copy to keep locally
  void ProcessConfig(pugi::xml_node config) throw(std::exception);

protected:
  // Commands that need to be implemented
  void StartRun();
  void StopRun();
  void SetupRegisters() throw(std::exception);

private:
  // Disallow copies
  PTBManager(const PTBManager &other);
  void operator=(const PTBManager &other);

  // The class responsible for the data reading.
  PTBReader *reader_;
  ConfigServer *cfg_srv_;
  pugi::xml_node config_;

//
//  std::map<const char*,int> fRegisterMap;
//  std::map<const char*,int> fRegisterValue;

  std::map<std::string, Command> commands_;

  Status status_;

};

#endif /* PTBMANAGER_H_ */
