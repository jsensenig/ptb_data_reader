/*
 * PTBManager.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "PTBManager.h"
#include "PTBReader.h"
#include "Logger.h"
#include "ConfigServer.h"

// -- PugiXML includes
#include "pugixml.hpp"
#include <iomanip>

const char* PTBManager::default_config_ = "./config/config_default.xml";

// Init with a new reader attached
PTBManager::PTBManager(bool emu_mode) : reader_(new PTBReader), cfg_srv_(0),status_(IDLE),emu_mode_(emu_mode) {
  Log(verbose) << "Setting up pointers." << endlog;

  // Register the available commands
  commands_.insert(std::map<std::string, Command>::value_type("RUNSTART",RUNSTART));
  commands_.insert(std::map<std::string, Command>::value_type("RUNSTOP",RUNSTOP));

  // Setup the registers first, to make sure that if the data manager receives something
  // we are ready to deliver.
  SetupRegisters();

  // In fact, load also a default configuration as well.
  LoadDefaultConfig();

  // This had to occur after setting up the command list or we might run into trouble.
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cfg_srv_->RegisterDataManager(this);

}



PTBManager::~PTBManager() {
  Log(verbose) << "Destroying object." << endlog;
  //FIXME: Implement memory deallocations to free the microZed.
}


void PTBManager::ExecuteCommand(const char* cmd) throw(std::exception) {
  try{

    Log(verbose) << "Received [" <<  cmd << "]" << endlog;
    std::map<std::string, Command>::iterator it;
    for (it = commands_.begin(); it != commands_.end(); ++it) {
      Log(verbose) << "Key : [" << it->first << "] : Value : [" << it->second << "]" << endlog;
      if (it->first == cmd) {
        Log(debug) << "It is the same" << endlog;
      }
    }

    Log(verbose) << "Going into the switch to try " <<  commands_.at(cmd) << endlog;
    switch (commands_.at(cmd)) {
    case RUNSTART:
      // Only starts a run if the manager is in IDLE state
      // Otherwise issue a warning
      if (getStatus() != PTBManager::IDLE) {
        Log(warning) << "A run is already running. Ignoring command" << endlog;
      } else {
        // Start the run
        Log(verbose) << "The Run should START now" << endlog;
        StartRun();
        break;
      }
      // -- Send the signal to start the run. Should be a register, I guess.
      break;
    case RUNSTOP:
      if (getStatus() != PTBManager::RUNNING) {
        Log(warning) << "Called for RUNSTOP but there is no run ongoing. Ignoring." << endlog;
        // -- Same thing.
        break;
      } else {
        Log(verbose) << "The Run should STOP now" << endlog;
        StopRun();
      }
    }
  }
  catch (const std::out_of_range &oor) {
    Log(error) << endlog << "Out of range error: " << oor.what() << endlog;
  }
  catch(std::exception &e) {
    Log(error) << "STL exception caught : " << e.what() << endlog;
  }
  catch(...) {
    Log(error) << "Unknown exception caught." << endlog;
  }
}

void PTBManager::StartRun() {
  Log(verbose) << "Starting the run" << endlog;
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to start the run

  // Order of execution:
  // Tell the PTBReader to start takign data (it won't receive anything but it will be ready)
  // Set the GLB_EN register to 1 (start readout in the fabric)
  // Set status flag to RUNNING

  if (!reader_ || !reader_->isReady()) {
    Log(error) << "Received call to start transmitting but reader is not ready. Refusing to run." << endlog;
    return;
  }
  reader_->StartDataTaking();

  // The GLB_EN is located in bin 31 of register 4
  *(volatile uint32_t*)(register_map_[3].address) |= (0x1 << 31);
  register_map_[3].value |= (0x1 << 31);
  Log(verbose) << "GLB_EN set. Register: " << std::setw(8) << std::setfill('0') << std::hex << register_map_[3].value << endlog;

  status_ = RUNNING;
}

void PTBManager::StopRun() {
  Log(verbose) << "Stopping the run" << endlog;
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to stop the run. This should include:
  //        Propagate into the PTBReader to stop data collection
  //        Deallocate its streaming connection.

  // The order should be:
  // 1. Set the GLB_EN register to 0 (stop readout in the fabric)
  // 2. Set the status to IDLE
  // 3. Tell the PTB reader to stop streaming

  // The GLB_EN is located in bin 31 of register 4
  *(volatile uint32_t*)(register_map_[3].address) |= (0x0 << 31);
  register_map_[3].value |= (0x0 << 31);
  Log(verbose) << "GLB_EN set. Register: " << std::setw(8) << std::setfill('0') << std::hex << register_map_[3].value << endlog;

  reader_->StopDataTaking();

  status_ = IDLE;
}

void PTBManager::SetupRegisters() throw (std::exception) {
  Log(debug) << "Setting up the appropriate registers to configure the board." << endlog;
  // Are we in emulator mode?

  struct LocalRegister tmp_reg;
  tmp_reg.address =NULL;
  tmp_reg.value = 0;
  // First set up the local map
  for (uint32_t i =0; i < num_registers_; ++i) {
    register_map_[i] = tmp_reg;
  }

  // Map the memory for each of the addresses.
  // This is different if we are emulating (use malloc), or
  // Using memory mapped registers (specific locations)

  if (emu_mode_) {
    // Most of the configuration options are passed through GPIO
    // This means everything is configured as 32bit registers

    // Allocate the memory for the registers

    for (uint32_t i =0; i < num_registers_; ++i) {
      register_map_[i].address = malloc(sizeof(uint32_t));
    }
  } else {
    Log(warning) << "Memory mapped registers are not yet implemented. This is most likely going to fail." << endlog;
  }
}

void PTBManager::FreeRegisters() throw (std::exception) {
  Log(debug) << "Cleaning up the allocated registers to configure the board." << endlog;
  if (emu_mode_) {
    // Simply deallocate the memory of each register
    for (uint32_t i =0; i < num_registers_; ++i) {
      free(register_map_[i].address);
      register_map_[i].address = NULL;
    }

  } else {
    Log(warning) << "Nothing to unmap as the registers are not yet implemented." << endlog;
  }

}

void PTBManager::ProcessConfig(pugi::xml_node config) throw (std::exception) {
  // Only accept a new configuration if we are not running.
  if (status_ == RUNNING) {
    Log(warning) << "Attempted to pass a new configuration during a run. Ignoring the new configuration." << endlog;
    return;
  }

  // This is the workhorse of the configuration.
  // At this point the registers should already be mapped and things should be flowing

  Log(verbose) << "Parsing the configuration." << endlog;

  // Load the childs by parts.
  // Could use a iterator but then
  ///!
  ///! DataBuffer
  ///!
  char *pEnd = NULL;

  for (pugi::xml_node_iterator it = config.begin(); it != config.end(); ++it) {
    Log(debug) << " Child name : " << it->name() << " " << endlog;
    // The reader should take care of this by itself.
    if (!strcmp(it->name(),"DataBuffer")) {
      if (!reader_) {
        Log(error) << "Don't have a valid PTBReader instance. Things are going to break." << endlog;
      }
      //-- Get the host
      std::string host = it->child("DaqHost").child_value();
      reader_->setTcpHost(host);
      uint32_t port = strtoul(it->child("DaqPort").child_value(),&pEnd,10);
      reader_->setTcpPort(port);
      uint32_t rollOver = strtoul(it->child("RollOver").child_value(),&pEnd,10);
      reader_->setTcpPort(rollOver);
      Log(info) << "Setting data transmission channel to [" << host << ":" << port << "]" << endlog;
      Log(debug) << "Packet rollover set to " << rollOver << endlog;

    }
    if (!strcmp(it->name(),"ChannelMask")) {
      // Deal directly with the childs.

      uint64_t bsu = strtoull(it->child("BSU").child_value(),&pEnd,16);
      // Now I want to get the first 32 bits into a register
      Log(verbose) << "Started with " << std::hex << bsu << endlog;
      // Make the assignments that I had decided before:
      // Catch the lowest 32 bits of the word
      *(volatile uint32_t*)(register_map_[0].address) = bsu & 0xFFFFFFFF;
      register_map_[0].value = bsu & 0xFFFFFFFF;
      Log(verbose) << "First register " << std::hex << register_map_[0].value << endlog;

      // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
      // [0-16]
      *(volatile uint32_t*)(register_map_[1].address) = (bsu >> 32 ) & 0x1FFFF;
      register_map_[1].value = (bsu >> 32 ) & 0x1FFFF;
      Log(verbose) << "Second register (tmp) " << std::hex << register_map_[1].value << endlog;

      // Now grab the TSU part to complete the mask of this register
      uint64_t tsu = strtoull(it->child("TSU").child_value(),&pEnd,16);

      // The lowest 15 bits go into the upper bits of the previous register
      // [17-31]
      *(volatile uint32_t*)(register_map_[1].address) |= ((tsu & 0x7FFF) << 17);
      register_map_[1].value |= ((tsu & 0x7FFF) << 17);
      Log(verbose) << "Second register " << std::hex << register_map_[1].value << endlog;

      // The remaining 33 (48-15) bits go into the next registers
      *(volatile uint32_t*)(register_map_[2].address) = ((tsu >> 15) & 0xFFFFFFFF);
      register_map_[2].value = ((tsu >> 15) & 0xFFFFFFFF);
      Log(verbose) << "Third register " << std::hex << *(volatile uint32_t*)(register_map_[2].address) << endlog;

      // The final bit goes into the lsb of register 4
      *(volatile uint32_t*)(register_map_[3].address) |= ((tsu >> 47) & 0x1);
      register_map_[3].value |= ((tsu >> 47) & 0x1);
      Log(verbose) << "Fourth register " << std::hex << *(volatile uint32_t*)(register_map_[3].address) << endlog;
    }
    if (!strcmp(it->name(),"ExtTriggers")) {
      // External triggers go into register 4
      // to bits [26-30]
      uint32_t TRIGEX = strtoul(it->child("Mask").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((TRIGEX & 0x1F) << 26);
      register_map_[3].value |= ((TRIGEX & 0x1F) << 26);
      Log(verbose) << "Fourth register (TRIGEX)" << std::hex << *(volatile uint32_t*)(register_map_[3].address) << endlog;
    }
    if (!strcmp(it->name(),"Calibration")) {
      // Calibrations go also into register 4
      // Period : [6-21]
      // Mask : [22-25]
      uint32_t CAL_PRD = strtoul(it->child("Period").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_PRD & 0xFFFF) << 6);
      register_map_[3].value |= ((CAL_PRD & 0xFFFF) << 6);
      Log(verbose) << "Fourth register (CAL_PRD)" << std::hex << *(volatile uint32_t*)(register_map_[3].address) << endlog;


      uint32_t CAL_MSK = strtoul(it->child("ChannelMask").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_MSK & 0xF) << 22);
      register_map_[3].value |= ((CAL_MSK & 0xF) << 22);
      Log(verbose) << "Fourth register (CAL_MSK)" << std::hex << *(volatile uint32_t*)(register_map_[3].address) << endlog;

    }
    // The most troublesome part: muon triggers
    if (!strcmp(it->name(),"MuonTriggers")) {
      // This one is quite complicated to parse.
      // For the moment let's deal with it statically...

      // The lockout window goes into register 4
      // LockoutWindow : [1-5]
      uint32_t MT_LOCKOUT = strtoul(it->child("LockoutWindow").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((MT_LOCKOUT & 0x1F) << 1);
      register_map_[3].value |= ((MT_LOCKOUT & 0x1F) << 1);
      Log(verbose) << "Fourth register (MT_LOCKOUT)" << std::hex << *(volatile uint32_t*)(register_map_[3].address) << endlog;

      // The remaining configuration words go into register 5

      // M_TRIGOUT - Number of clock cycles to keep the triger pulse (out high)
      // Width of the trigger pulse
      // TrigOutWidth : [25-27]
      uint32_t M_TRIGOUT = strtoul(it->child("TrigOutWidth").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_TRIGOUT & 0x7) << 25);
      register_map_[4].value |= ((M_TRIGOUT & 0x7) << 25);
      Log(verbose) << "Fourth register (M_TRIGOUT)" << std::hex << *(volatile uint32_t*)(register_map_[4].address) << endlog;

      // M_LOCKHI - Number of clock cycles to extend the input signal
      // Width of the trigger window
      // TriggerWindow : [22-24]
      uint32_t M_LOCKHI = strtoul(it->child("TriggerWindow").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_LOCKHI & 0x7) << 22);
      register_map_[4].value |= ((M_LOCKHI & 0x7) << 22);
      Log(verbose) << "Fourth register (M_LOCKHI)" << std::hex
          << *(volatile uint32_t*)(register_map_[4].address) << endlog;

      // Now it is specific trigger codes...
      // Deal with one at a time
      //pugi::xml_object_range<pugi::xml_named_node_iterator> mtrigger_nodes = it->children("TriggerMask");

      pugi::xml_node mtrigger_node = it->find_child_by_attribute("TriggerMask","id","A");
      if (mtrigger_node.empty()){
        Log(error) << "Couldn't find the configuration for trigger A." << endlog;
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,5,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","B");
      if (mtrigger_node.empty()){
        Log(error) << "Couldn't find the configuration for trigger B." << endlog;
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,5,7);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","C");
      if (mtrigger_node.empty()){
        Log(error) << "Couldn't find the configuration for trigger C." << endlog;
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,18,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","D");
      if (mtrigger_node.empty()){
        Log(error) << "Couldn't find the configuration for trigger D." << endlog;
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,18,7);
      }

    }

    Log(debug) << " Content val : " << it->value() << "  " << endlog;
    Log(debug) << " Content child : " << it->child_value() << endlog;
  }

  // After parsing everything (and making sure that all the configuration is set)
  // Store the configuration locally
  config_ = config;
}

void PTBManager::DumpConfigurationRegisters() {

  for (size_t i = 0; i < register_map_.size(); ++i) {
    Log(info) << "Reg " << i << " : dec=[" << std::dec << std::setw(10) << register_map_.at(i).value << "] hex=" << std::hex
        << std::setw(8) << std::setfill('0') << register_map_.at(i).value << std::dec << endlog;
  }

}

void PTBManager::ParseMuonTrigger(pugi::xml_node T, uint32_t reg, uint32_t reg_offset) {

  Log(debug) << "Processing muon trigger with conf reg " << reg << " and offset " << reg_offset << endlog;

  // First thing to do is identify the offset in the bins of the configuration register
  uint32_t word_offset = (reg_offset>1)?10:0;
  char *pEnd;
  // == First deal with the global parameters:
  // variables that are used repeatedly below
  uint32_t mask;
  uint32_t bit_offset;
  std::string field_name;
  uint32_t input_word;
  uint64_t input_longword;

  // M_?_EXT_OP - External logic to be applied between the groups
  // ExtLogic : [word_offset+5 - word_offset+6]
  mask = 0x3;
  bit_offset = 5;
  field_name = "ExtLogic";

  input_word = strtoul(T.child(field_name.c_str()).child_value(),&pEnd,16);
  *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
  register_map_[reg].value |= ((input_word & mask) << (word_offset+bit_offset));
  Log(verbose) << "Conf register " << std::dec << reg << " (" << field_name << ")" << std::hex
      << *(volatile uint32_t*)(register_map_[reg].address) << endlog;

  // M_?_PRESC - Prescale of a given trigger
  // Prescale : [word_offset+2 - word_offset+4]
  mask = 0x7;
  bit_offset = 2;
  field_name = "Prescale";

  input_word = strtoul(T.child(field_name.c_str()).child_value(),&pEnd,16);
  *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
  register_map_[reg].value |= ((input_word & mask) << (word_offset+bit_offset));
  Log(verbose) << "Conf register " << std::dec << reg << " (" << field_name << ")" << std::hex
      << *(volatile uint32_t*)(register_map_[reg].address) << endlog;


  // Loop over the groups
  const char *groups[] = {"group1","group2"};

  for (uint32_t i =0; i < 2; ++i) {
    pugi::xml_node G = T.child(groups[i]);
    // Now the more messy part of using a long register
    Log(debug) << "Processing group [" << groups[i] << "]" << endlog;
    // Intra-group logic : [word_offset+7 - word_offset+8]
    // M_?_G[i]_OP
    mask = 0x3;
    bit_offset = 7+i;
    field_name = "Logic";
    input_word = strtoul(G.child(field_name.c_str()).child_value(),&pEnd,16);
    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
    register_map_[reg].value |= ((input_word & mask) << (word_offset+bit_offset));
    Log(verbose) << "Conf register " << std::dec << reg << " (" << field_name << ")" << std::hex
        << *(volatile uint32_t*)(register_map_[reg].address) << endlog;

    // Now get the mask into a 64 bit word
    input_longword = strtoull(G.child("BSU").child_value(),&pEnd,16);
    // These now go into the offset registers

    // -- First contains the first 32 bits of the word
    mask = 0xFFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) |= (input_longword & mask);
    register_map_[reg+reg_offset+(i*3)].value |= (input_word & mask);
    Log(verbose) << "Mask register " << std::dec << reg+reg_offset+(i*3) << " (W1)" << std::hex
        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) << endlog;

    // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
    // [0-16]
    mask = 0x1FFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) |= (input_longword >> 32 ) & mask;
    register_map_[reg+reg_offset+(i*3)+1].value |= (input_longword >> 32 ) & mask;
    Log(verbose) << "Mask register " << std::dec << reg+reg_offset+(i*3)+1 << " (W2)" << std::hex
        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) << endlog;


    // Now grab the TSU part to complete the mask of this register
    input_longword = strtoull(G.child("TSU").child_value(),&pEnd,16);

    // The lowest 15 bits go into the upper bits of the previous register
    // [17-31]
    mask = 0x7FFF;
    bit_offset = 17;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) |= (input_longword & mask ) << bit_offset;
    register_map_[reg+reg_offset+(i*3)+1].value |= (input_longword & mask ) << bit_offset;
    Log(verbose) << "Mask register " << std::dec << reg+reg_offset+(i*3)+1 << " (W2)" << std::hex
        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) << endlog;

    // The next 32 (47-15) bits go into the next register
    mask = 0xFFFFFFFF;
    bit_offset = 15;

    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) |= (input_longword >> bit_offset) & mask;
    register_map_[reg+reg_offset+(i*3)+2].value |= (input_longword >> bit_offset) & mask;
    Log(verbose) << "Mask register " << std::dec << reg+reg_offset+(i*3)+2 << " (W3)" << std::hex
        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) << endlog;

    // The final bit (48) goes into the lsb of the configuration register
    mask = 0x1;
    bit_offset = i;
    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_longword >> 47) && mask) << bit_offset;
    register_map_[reg].value |= ((input_longword >> 47) && mask) << bit_offset;
    Log(verbose) << "Conf register " << std::dec << reg << " (MASK)" << std::hex
        << *(volatile uint32_t*)(register_map_[reg].address) << endlog;
  }

}

void PTBManager::LoadDefaultConfig() {
  // File opened alright. Proceed with the parsing
  // Instanciate the XML plugin
  try {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(default_config_);
    Log(verbose) << "Load result : " << result.description() << endlog;
    pugi::xml_node config = doc.child("config");
    if (config == NULL) {
      Log(error) << "Failed to load the config block from input file [" << default_config_ << "]." << endlog;
    } else {
      ProcessConfig(config);
    }
  }
  catch (pugi::xpath_exception &e) {
    Log(error) << "PUGIXML exception caught: " << e.what() << endlog;
    return;
  }
  catch (std::exception &e) {
    Log(error) << "STD exception caught: " << e.what() << endlog;
    return;
  }
  catch (...) {
    Log(error) << "Unknown exception caught. This is going to mean trouble." << endlog;
    return;
  }
}
