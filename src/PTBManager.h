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

  void ExecuteCommand(const char* cmd);
  // Receive the configuration as
  // Passed by copy to keep locally
  void ProcessConfig(pugi::xml_node config);

  /**
   * Loops over the registers and dum their contents, both in decimal and Hex.
   */
  void DumpConfigurationRegisters();

  /**
   * Zero the configuration registers.
   */
  void ResetConfigurationRegisters();
  /**
   * Restores the configuration registers form the local cache. The only exception is register 0
   * that is the commands register
   */
  void RestoreConfigurationRegisters();

  void StartRun();
  void StopRun();

  void FreeRegisters();
  void ClearCommands() {
	  commands_.clear();
  }

protected:
  // Commands that need to be implemented
  void SetupRegisters();
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

  /**
   * Returns the status of a specific bit in a register
   * @param reg
   * @param bit
   * @return
   */
  bool GetBit(uint32_t reg, uint32_t bit);

  /**
   * Generic bit setter.
   * @param reg register number
   * @param idx Bit index
   * @param status status to set
   */
  void SetBit(uint32_t reg, uint32_t bit, bool status);


    ///!
    ///! The methods below are just shorthands for certain commands.
    ///! There is no integrity check so they should be wrapped on other commands.
    ///!
  /**
   * Disables the board. Sets the enable register to status (true: on; false:off)
   * @param status
   */
  void SetEnableBit(bool status);

  /**
   * Resets the board. Sets the reset register to status (true: on; false:off)
   * @warning Keep in mind that the reset is active low, so enable the bit means effectively setting it to 0.
   * This method does not have a getter as the reset status does not matter
   * @param status
   */
  void SetResetBit(bool status);

  /**
   * Commits the configuration. Sets the config register to status (true: on; false:off)
   * @param status
   */
  void SetConfigBit(bool status);

  /**
   * Get the configuration commit status (true: committed; false:uncommited)
   * @return status of the configuration register
   */
  bool GetConfigBitACK();

  /**
   * Return the enable ack (true: enabled; false: disabled)
   * @return status of the enable bit command.
   */
  bool GetEnableBitACK();

  /**
   * Overwrites a range of bits in a register with the contents of another variable.
   * @param reg The register to be modified
   * @param pos The edge of the range to be modified (LSB)
   * @param len The size of the mask to be created in number of bits.
   * @param value The contents to be overwritten
   */
//  void SetBitRange(uint32_t reg, uint32_t pos, uint32_t len, uint32_t value);

  // The class responsible for the data reading.
  PTBReader *reader_;
  ConfigServer *cfg_srv_;
  pugi::xml_node config_;

  // This is a cache of what is in each register now
  std::map<uint32_t,LocalRegister> register_map_;
  // Use a second that is just a clone of the first (except for register 0, which is the status registers
  // When a reset is called, it is the contents of these registers that are sent to the board
  std::map<uint32_t,LocalRegister> register_cache_;

  std::map<std::string, Command> commands_;

  Status status_;
  bool emu_mode_;

  static const uint8_t num_registers_ = 40;
  static const char *default_config_;
  void *mapped_base_addr_;
};

#endif /* PTBMANAGER_H_ */
