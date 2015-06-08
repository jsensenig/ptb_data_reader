/*
 * PTBManager.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBMANAGER_H_
#define PTBMANAGER_H_

#include <map>

// -- Forward declarations
class PTBReader;
class ConfigServer;
class CommandDispatcher;

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
  PTBManager();
  virtual ~PTBManager();

  const PTBReader*& getReader() const {
    return reader_;
  }

  void setReader(const PTBReader*& reader) {
    reader_ = reader;
  }

  Status getStatus() const {
    return status;
  }

  void setStatus(Status status) {
    this->status = status;
  }

private:
  // Disallow copies
  PTBManager(const PTBManager &other);
  void operator=(const PTBManager &other);

  // The class responsible for the data reading.
  PTBReader *reader_;
  ConfigServer *cfg_srv_;
  CommandDispatcher *cmd_disp_;

  std::map<const char*,int> fRegisterMap;
  std::map<const char*,int> fRegisterValue;

  Status status;

};

#endif /* PTBMANAGER_H_ */
