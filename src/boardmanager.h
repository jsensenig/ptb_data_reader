/*
 * PTBManager.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBMANAGER_H_
#define PTBMANAGER_H_


#include "Logger.h"
#include "util.h"

#include <map>
#include <cstdint>
#include <sstream>
#include <thread>

#include "config.h"

#include "ptb_registers.h"

#include "json.hpp"

using json = nlohmann::json;

namespace ptb {
/**
 * \class PTBManager
 * \brief Responsible for the configuration registers of the PTB.
 *
 * This class is responsible for passing the configuration into the PTB. Essentially it is
 * responsible for everything except the data readout.
 * Uses the Xilinx GPIO driver to set the configuration registers. This is more than good enough.
 */

// -- Forward declarations
class board_reader;
//class board_server;

class board_manager {
public:

  enum state {
    RUNNING = 0,
    IDLE = 1
  };

  enum command {STARTRUN=0,
                STOPRUN = 1,
                SOFTRESET = 2,
                HARDRESET = 3,
                GETSTARTTIME = 4
               };

  board_manager();
  virtual ~board_manager();

  const board_reader* getReader() const {
    return reader_;
  }

  void setReader(board_reader* reader) {
    reader_ = reader;
  }

  state get_board_state() const {
    return board_state_;
  }

  void set_board_state(state status) {
    this->board_state_ = status;
  }

  void exec_command(const std::string &cmd,json &answers);
  // Receive the configuration as
  // Passed by copy to keep locally
  void process_config(json & config,json &answers);

  //Loads the PDS config into FW regs
  void pds_config(json & pdsconfig);

  //Loads the Beam config into FW regs
  void beam_config(json & beamconfig);

  //Loads the CRT config into FW regs
  void crt_config(json & crtconfig);


  /**
   * Loops over the registers and dum their contents, both in decimal and Hex.
   */
  void dump_config_registers();

  /**
   * Zero the configuration registers.
   */
  void zero_config_registers();
  /**
   * Restores the configuration registers form the local cache. The only exception is register 0
   * that is the commands register
   */
  void restore_config_registers();

  void free_registers();

protected:
  // Commands that need to be implemented
  void setup_registers();
  // -- Specific commands
  void start_run();
  void stop_run();
  void soft_reset();


private:
  // Disallow copies
  board_manager(const board_manager &other);
  void operator=(const board_manager &other);

  /**
   * Returns the status of a specific bit in a register
   * @param reg
   * @param bit
   * @return
   */
  bool get_bit(const uint32_t reg, const uint32_t bit);

  /**
   * Generic bit setter.
   * @param reg register number
   * @param idx Bit index
   * @param status status to set
   */
  void set_bit(const uint32_t reg, const uint32_t bit, bool status);


    ///!
    ///! The methods below are just shorthands for certain commands.
    ///! There is no integrity check so they should be wrapped on other commands.
    ///!
  /**
   * Disables the board. Sets the enable register to status (true: on; false:off)
   * @param status
   */
  void set_enable_bit(bool status);

  /**
   * Resets the board. Sets the reset register to status (true: on; false:off)
   * @warning Keep in mind that the reset is active low, so enable the bit means effectively setting it to 0.
   * This method does not have a getter as the reset status does not matter
   * @param status
   */
  void set_reset_bit(bool status);

  /**
   * Commits the configuration. Sets the config register to status (true: on; false:off)
   * @param status
   */
  void set_config_bit(bool status);

  /**
   * Get the configuration commit status (true: committed; false:uncommited)
   * @return status of the configuration register
   */
  bool get_config_bit_ACK();

  /**
   * Return the enable ack (true: enabled; false: disabled)
   * @return status of the enable bit command.
   */
  bool get_enable_bit_ACK();

  /**
   * Overwrites a range of bits in a register with the contents of another variable.
   * @param reg The register to be modified
   * @param pos The edge of the range to be modified (LSB)
   * @param len The size of the mask to be created in number of bits.
   * @param value The contents to be overwritten
   */
  void set_bit_range_register(const uint32_t reg, const uint32_t pos, const uint32_t len,const uint32_t value);


  // The class responsible for the data reading.
  board_reader *reader_;

  // This is a cache of what is in each register now
  std::map<uint32_t,ptb::util::mem_reg> register_map_;
  // Use a second that is just a clone of the first (except for register 0, which is the status registers
  // When a reset is called, it is the contents of these registers that are sent to the board
  std::map<uint32_t,ptb::util::mem_reg> register_cache_;


  std::map<std::string, command> commands_;

  state board_state_;

  static const uint8_t num_registers_ = 40;
  //static const char *default_config_;
  json config_;
  void *mapped_conf_base_addr_;

  //std::vector<std::string> feedback_;
  json feedback_;

  //std::ostringstream msgs_;
  std::string msgs_str_;
  bool error_state_;

  std::string data_socket_host_;
  unsigned short data_socket_port_;

};

// -- declare a couple of inliners, since there is no point in adding overhead
/// -- Short hand methods to be used on the higher methods
///
/// NOTE: The value has to be shifted to the right bit positions
/// Position is in bit number. If position is set to 16, the new value starts being inserted in the
/// 17th bit (bit 16, since bits start at 0).

inline void board_manager::set_bit_range_register(const uint32_t reg, const uint32_t pos, const uint32_t len,const uint32_t value) {
  // Create a mask based on pos and len
  uint32_t mask = (((uint32_t)1 << len)-1) << pos;
  register_map_[reg].value() = (register_map_[reg].value() & ~mask) | (value & mask);
}

inline void board_manager::set_bit(const uint32_t reg, const uint32_t bit, bool status) {
	  // Things are more complicated than this. We want to set a single bit, regardless of what is around
	  Log(debug,"Reading the bit %u from reg %u",bit,reg);
	  uint32_t value = util::Xil_In32((uint32_t)register_map_[reg].addr);
	  uint32_t new_value = value^((-(status?1:0) ^ value) & ( 1 << bit));

	  Log(debug,"Got %08X -> %08X",value,new_value);
	  util::Xil_Out32((uint32_t)register_map_[reg].addr,new_value);
	  // sleep for a short while to allow hardware to act on it
      std::this_thread::sleep_for (std::chrono::microseconds(10));
	  Log(debug,"Final register value %08X",register_map_[reg].value());
	}

inline bool board_manager::get_bit(const uint32_t reg, const uint32_t bit) {
	return (((register_map_[reg].value() >> bit) & 0x1) == 0x1)?true:false;
}

// Request run start
inline void board_manager::set_enable_bit(bool status) {
  set_bit(0,31,status);
}

inline void board_manager::set_reset_bit(bool status) {
  // here true defines the reset enable...the firmware inverts this
	set_bit(0,27,status);
}

// request config
inline void board_manager::set_config_bit(bool status) {
	set_bit(0,29,status);
}

inline bool board_manager::get_config_bit_ACK() {
  return get_bit(0,28);
}
inline bool board_manager::get_enable_bit_ACK() {
  return get_bit(0,30);
}

} // namespace ptb
#endif /* PTBMANAGER_H_ */
