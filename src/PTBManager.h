/*
 * PTBManager.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBMANAGER_H_
#define PTBMANAGER_H_

#include <map>
#include <cstdint>
#include "pugixml.hpp"

// -- Forward declarations
class PTBReader;
class ConfigServer;

typedef struct LocalRegister {
  void      *address;
  // Should use this method to access the value held in address
  // since it shortens the command
  volatile uint32_t& value () {return *(static_cast<volatile uint32_t*>(address));}
} LocalRegister;

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

  enum Command {STARTRUN=0,
                STOPRUN = 1,
                SOFTRESET = 2,
                HARDRESET = 3
               };

  PTBManager(bool emu_mode = false);
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

  /**
   * Loops over the registers and dum their contents, both in decimal and Hex.
   */
  void DumpConfigurationRegisters();

  /**
   * Zero the configuration registers.
   */
  void ResetConfigurationRegisters();

  void StartRun();
  void StopRun();

protected:
  // Commands that need to be implemented
  void SetupRegisters() throw(std::exception);
  void FreeRegisters() throw(std::exception);
  void LoadDefaultConfig();
  // This is a rather messy function
  /**
   * Parses a node for a muon trigger
   * @param T is the node to be parsed
   * @param reg is the configuration register. The masks are placed in consecutive registers
   * @param reg_type is the register offset to place the masks
   */
  void ParseMuonTrigger(pugi::xml_node T, uint32_t reg, uint32_t reg_offset);
private:
  // Disallow copies
  PTBManager(const PTBManager &other);
  void operator=(const PTBManager &other);

  // The class responsible for the data reading.
  PTBReader *reader_;
  ConfigServer *cfg_srv_;
  pugi::xml_node config_;

  // This is a cache of what is in each register now
  std::map<uint32_t,LocalRegister> register_map_;

  std::map<std::string, Command> commands_;

  Status status_;
  bool emu_mode_;

  static const uint8_t num_registers_ = 31;
  static const char *default_config_;
};

#endif /* PTBMANAGER_H_ */
