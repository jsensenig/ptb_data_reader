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

extern "C" {
#include <unistd.h>
};

const char* PTBManager::default_config_ = "./config/config_default.xml";

// Init with a new reader attached
PTBManager::PTBManager(bool emu_mode) : reader_(new PTBReader(emu_mode)), cfg_srv_(0),status_(IDLE),emu_mode_(emu_mode) {
  Log(debug,"Setting up pointers." );

  // Register the available commands
  commands_.insert(std::map<std::string, Command>::value_type("StartRun",STARTRUN));
  commands_.insert(std::map<std::string, Command>::value_type("StopRun",STOPRUN));
  commands_.insert(std::map<std::string, Command>::value_type("SoftReset",SOFTRESET));
  commands_.insert(std::map<std::string, Command>::value_type("HardReset",HARDRESET));

  // Setup the registers first, to make sure that if the data manager receives something
  // we are ready to deliver.
  SetupRegisters();

  // In fact, load also a default configuration as well.
  // Won't work...client is not ready
  //LoadDefaultConfig();

  // This had to occur after setting up the command list or we might run into trouble.
  cfg_srv_ = ConfigServer::get();
  // Register to receive callbacks
  cfg_srv_->RegisterDataManager(this);

}



PTBManager::~PTBManager() {
  Log(debug,"Destroying object." );
  //FIXME: Implement memory deallocations to free the microZed.
  delete reader_;
}


void PTBManager::ExecuteCommand(const char* cmd) throw(std::exception) {
  try{

    Log(debug,"Received [%s]", cmd);
    std::map<std::string, Command>::iterator it;
    for (it = commands_.begin(); it != commands_.end(); ++it) {
      Log(debug,"Key : [%s] : Value [%u]",it->first.c_str(),it->second );
      if (it->first == cmd) {
        Log(verbose,"It is the same" );
      }
    }

    Log(verbose,"Going into the switch to try %d",commands_.at(cmd) );
    switch (commands_.at(cmd)) {
    case STARTRUN:
      // Only starts a run if the manager is in IDLE state
      // Otherwise issue a warning
      if (getStatus() != PTBManager::IDLE) {
        Log(warning,"A run is already running. Ignoring command" );
      } else {
        // Start the run
        Log(verbose,"The Run should START now" );
        StartRun();
        break;
      }
      // -- Send the signal to start the run. Should be a register, I guess.
      break;
    case SOFTRESET:
    case HARDRESET:
    case STOPRUN:
      if (getStatus() != PTBManager::RUNNING) {
        Log(warning,"Called for STOPRUN but there is no run ongoing. Ignoring." );
        // -- Same thing.
        break;
      } else {
        Log(verbose,"The Run should STOP now" );
        StopRun();
        break;
      }
    default:
      throw std::string("Unkown command"); // command_exception("Unkown command");
    }
  }
  catch (const std::out_of_range &oor) {
    Log(error,"Out of range error: %s",oor.what() );
    throw std::string("Unkown command");
  }
  catch(std::exception &e) {
    Log(error,"STL exception caught : %s",e.what() );
    throw;
  }
  catch(std::string &str) {
    Log(error,"Caught exception. %s",str.c_str() );
    Log(verbose,"Rethrowing" );
    throw;
  }
  catch(...) {
    Log(error,"Unknown exception caught." );
    throw std::string("Unkown command");
  }
}

void PTBManager::StartRun() {
  Log(verbose,"Starting the run" );
  // FIXME: IMplement the specifics for setting the appropriate registers
  //        to start the run

  // Order of execution:
  // Tell the PTBReader to start takign data (it won't receive anything but it will be ready)
  // Set the GLB_EN register to 1 (start readout in the fabric)
  // Set status flag to RUNNING
  if (!reader_ || !reader_->isReady()) {
    reader_->InitConnection();
  }

  if (!reader_ || !reader_->isReady()) {
    Log(warning,"Received call to start transmitting but reader is not ready. Refusing to run." );
    throw std::string("Data reader not ready...");
    return;
  }
  reader_->StartDataTaking();

  // The GLB_EN is located in bin 31 of register 4
  *(volatile uint32_t*)(register_map_[3].address) |= (0x1 << 31);
  register_map_[3].value |= (0x1 << 31);
  Log(debug,"GLB_EN set. Register: 0x%08x ", register_map_[3].value );

  status_ = RUNNING;
}

void PTBManager::StopRun() {
  Log(debug,"Stopping the run" );
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
  Log(debug,"GLB_EN set. Register: 0x%08x ",register_map_[3].value );

  reader_->StopDataTaking();

  status_ = IDLE;
}

void PTBManager::SetupRegisters() throw (std::exception) {
  Log(info,"Setting up the appropriate registers to configure the board." );
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
    Log(warning,"Memory mapped registers are not yet implemented. This is most likely going to fail." );
  }
}

void PTBManager::FreeRegisters() throw (std::exception) {
  Log(info,"Cleaning up the allocated registers to configure the board." );
  if (emu_mode_) {
    // Simply deallocate the memory of each register
    for (uint32_t i =0; i < num_registers_; ++i) {
      free(register_map_[i].address);
      register_map_[i].address = NULL;
    }

  } else {
    Log(warning,"Nothing to unmap as the registers are not yet implemented." );
  }

}

void PTBManager::ProcessConfig(pugi::xml_node config) throw (std::exception) {
  // Only accept a new configuration if we are not running.
  if (status_ == RUNNING) {
    Log(warning,"Attempted to pass a new configuration during a run. Ignoring the new configuration." );
    return;
  }

  // // Check if the reader is ready. If it is ignore the change
  // if (reader_->isReady()) {
  //   Log(warning,"FIXME: Reader is ready. Do not overwrite config.");
  //   return;
  // }

  // This is the workhorse of the configuration.
  // At this point the registers should already be mapped and things should be flowing

  Log(verbose,"Parsing the configuration." );

  // Load the childs by parts.
  // Could use a iterator but then
  ///!
  ///! DataBuffer
  ///!
  char *pEnd = NULL;

  for (pugi::xml_node_iterator it = config.begin(); it != config.end(); ++it) {
    Log(verbose," Child name : %s",it->name());
    // The reader should take care of this by itself.
    if (!strcmp(it->name(),"DataBuffer")) {
      if (!reader_) {
        Log(error,"Don't have a valid PTBReader instance. Things are going to break." );
      }
      //-- Get the host
      std::string host = it->child("DaqHost").child_value();
      reader_->setTcpHost(host);
      //uint32_t port = strtoul(it->child("DaqPort").child_value(),&pEnd,10);
      unsigned short port = atoi(it->child("DaqPort").child_value());
      Log(debug,"Received port %hu",port );
      reader_->setTcpPort(port);
      //uint32_t rollOver = strtoul(it->child("RollOver").child_value(),&pEnd,10);
      uint32_t rollOver = atoi(it->child("RollOver").child_value());
      Log(debug,"Rollover %u",rollOver );
      reader_->setPacketRollover(rollOver);
      // uint32_t duration = strtoul(it->child("MicroSliceDuration").child_value(),&pEnd,10);
      uint32_t duration = atoi(it->child("MicroSliceDuration").child_value());
      Log(debug,"MicroSlice Duration %u",duration );
      uint64_t timeRollOver = (1 << duration);
      reader_->setTimeRollover(timeRollOver);

      Log(verbose,"Setting data transmission channel to [%s:%hu]",host.c_str(),port);
      Log(verbose,"Packet rollover set to %u ",rollOver );
    }
    if (!strcmp(it->name(),"ChannelMask")) {
      // Deal directly with the childs.

      uint64_t bsu = strtoull(it->child("BSU").child_value(),&pEnd,16);
      // Now I want to get the first 32 bits into a register
      Log(verbose,"Started with 0x%lX",bsu );
      // Make the assignments that I had decided before:
      // Catch the lowest 32 bits of the word
      *(volatile uint32_t*)(register_map_[0].address) = bsu & 0xFFFFFFFF;
      register_map_[0].value = bsu & 0xFFFFFFFF;
      Log(verbose,"First register 0x%X", register_map_[0].value );

      // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
      // [0-16]
      *(volatile uint32_t*)(register_map_[1].address) = (bsu >> 32 ) & 0x1FFFF;
      register_map_[1].value = (bsu >> 32 ) & 0x1FFFF;
      Log(verbose,"Second register (tmp) 0x%X",register_map_[1].value );

      // Now grab the TSU part to complete the mask of this register
      uint64_t tsu = strtoull(it->child("TSU").child_value(),&pEnd,16);

      // The lowest 15 bits go into the upper bits of the previous register
      // [17-31]
      *(volatile uint32_t*)(register_map_[1].address) |= ((tsu & 0x7FFF) << 17);
      register_map_[1].value |= ((tsu & 0x7FFF) << 17);
      Log(verbose,"Second register 0x%X", register_map_[1].value );

      // The remaining 33 (48-15) bits go into the next registers
      *(volatile uint32_t*)(register_map_[2].address) = ((tsu >> 15) & 0xFFFFFFFF);
      register_map_[2].value = ((tsu >> 15) & 0xFFFFFFFF);
      Log(verbose,"Third register 0x%X",*(volatile uint32_t*)(register_map_[2].address) );

      // The final bit goes into the lsb of register 4
      *(volatile uint32_t*)(register_map_[3].address) |= ((tsu >> 47) & 0x1);
      register_map_[3].value |= ((tsu >> 47) & 0x1);
      Log(verbose,"Fourth register 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
    }
    if (!strcmp(it->name(),"ExtTriggers")) {
      // External triggers go into register 4
      // to bits [26-30]
      uint32_t TRIGEX = strtoul(it->child("Mask").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((TRIGEX & 0x1F) << 26);
      register_map_[3].value |= ((TRIGEX & 0x1F) << 26);
      Log(verbose,"Fourth register (TRIGEX) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );
    }

    if (!strcmp(it->name(),"Calibration")) {
      // Calibrations go also into register 4
      // Period : [6-21]
      // Mask : [22-25]
      uint32_t CAL_PRD = strtoul(it->child("Period").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_PRD & 0xFFFF) << 6);
      register_map_[3].value |= ((CAL_PRD & 0xFFFF) << 6);
      Log(verbose,"Fourth register (CAL_PRD) 0x%X",*(volatile uint32_t*)(register_map_[3].address));


      uint32_t CAL_MSK = strtoul(it->child("ChannelMask").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[3].address) |= ((CAL_MSK & 0xF) << 22);
      register_map_[3].value |= ((CAL_MSK & 0xF) << 22);
      Log(verbose,"Fourth register (CAL_MSK) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );

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
      Log(verbose,"Fourth register (MT_LOCKOUT) 0x%X",*(volatile uint32_t*)(register_map_[3].address) );

      // The remaining configuration words go into register 5

      // M_TRIGOUT - Number of clock cycles to keep the triger pulse (out high)
      // Width of the trigger pulse
      // TrigOutWidth : [25-27]
      uint32_t M_TRIGOUT = strtoul(it->child("TrigOutWidth").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_TRIGOUT & 0x7) << 25);
      register_map_[4].value |= ((M_TRIGOUT & 0x7) << 25);
      Log(verbose,"Fourth register (M_TRIGOUT) 0x%X", *(volatile uint32_t*)(register_map_[4].address) );

      // M_LOCKHI - Number of clock cycles to extend the input signal
      // Width of the trigger window
      // TriggerWindow : [22-24]
      uint32_t M_LOCKHI = strtoul(it->child("TriggerWindow").child_value(),&pEnd,16);
      *(volatile uint32_t*)(register_map_[4].address) |= ((M_LOCKHI & 0x7) << 22);
      register_map_[4].value |= ((M_LOCKHI & 0x7) << 22);
      Log(verbose,"Fourth register (M_LOCKHI) 0x%X", *(volatile uint32_t*)(register_map_[4].address) );

      // Now it is specific trigger codes...
      // Deal with one at a time
      //pugi::xml_object_range<pugi::xml_named_node_iterator> mtrigger_nodes = it->children("TriggerMask");

      pugi::xml_node mtrigger_node = it->find_child_by_attribute("TriggerMask","id","A");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger A." );
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,5,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","B");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger B." );
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,5,7);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","C");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger C." );
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,18,1);
      }

      mtrigger_node = it->find_child_by_attribute("TriggerMask","id","D");
      if (mtrigger_node.empty()){
        Log(warning,"Couldn't find the configuration for trigger D." );
        continue;
      } else {
        // -- The operations go into register 5
        ParseMuonTrigger(mtrigger_node,18,7);
      }

    }

    Log(verbose," Content val : %s",it->value());
    Log(verbose," Content child : %s",it->child_value() );
  }

  // After parsing everything (and making sure that all the configuration is set)
  // Store the configuration locally
  config_ = config;

  // Tell the reader to start the connection
  sleep(10);
  Log(verbose,"Initializing connection to DAQ upstream." );
  //Log(verbose,"Host : " << host << " port " << tcp_port_ << endl;
  reader_->InitConnection();

}

void PTBManager::DumpConfigurationRegisters() {

  for (size_t i = 0; i < register_map_.size(); ++i) {
    Log(verbose,"Reg %u : dec=[%010u] hex=[%08X]",i, register_map_.at(i).value, register_map_.at(i).value );
  }

}

void PTBManager::ParseMuonTrigger(pugi::xml_node T, uint32_t reg, uint32_t reg_offset) {

  Log(verbose,"Processing muon trigger with conf reg %u and offset %u",reg,reg_offset );

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
  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );

  // M_?_PRESC - Prescale of a given trigger
  // Prescale : [word_offset+2 - word_offset+4]
  mask = 0x7;
  bit_offset = 2;
  field_name = "Prescale";

  input_word = strtoul(T.child(field_name.c_str()).child_value(),&pEnd,16);
  *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
  register_map_[reg].value |= ((input_word & mask) << (word_offset+bit_offset));
  Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );
//  Log(verbose,"Conf register " << std::dec << reg << " (" << field_name << ")" << std::hex
//      << *(volatile uint32_t*)(register_map_[reg].address) );


  // Loop over the groups
  const char *groups[] = {"group1","group2"};

  for (uint32_t i =0; i < 2; ++i) {
    pugi::xml_node G = T.child(groups[i]);
    // Now the more messy part of using a long register
    Log(verbose,"Processing group [%s]",groups[i]);
    // Intra-group logic : [word_offset+7 - word_offset+8]
    // M_?_G[i]_OP
    mask = 0x3;
    bit_offset = 7+i;
    field_name = "Logic";
    input_word = strtoul(G.child(field_name.c_str()).child_value(),&pEnd,16);
    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_word & mask) << (word_offset+bit_offset));
    register_map_[reg].value |= ((input_word & mask) << (word_offset+bit_offset));
    Log(debug,"Conf register %u (%s) %X",reg,field_name.c_str(),*(volatile uint32_t*)(register_map_[reg].address) );

//    Log(verbose,"Conf register " << std::dec << reg << " (" << field_name << ")" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg].address) );

    // Now get the mask into a 64 bit word
    input_longword = strtoull(G.child("BSU").child_value(),&pEnd,16);
    // These now go into the offset registers

    // -- First contains the first 32 bits of the word
    mask = 0xFFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) |= (input_longword & mask);
    register_map_[reg+reg_offset+(i*3)].value |= (input_word & mask);
    Log(debug,"Mask register %u (W1) %X",reg+reg_offset+(i*3),*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) );

//    Log(verbose,"Mask register " << std::dec << reg+reg_offset+(i*3) << " (W1)" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)].address) );

    // The remaining 17 bits (49-32) go into the lower 17 bits of the next word
    // [0-16]
    mask = 0x1FFFF;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) |= (input_longword >> 32 ) & mask;
    register_map_[reg+reg_offset+(i*3)+1].value |= (input_longword >> 32 ) & mask;
    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );

//    Log(verbose,"Mask register " << std::dec << reg+reg_offset+(i*3)+1 << " (W2)" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );


    // Now grab the TSU part to complete the mask of this register
    input_longword = strtoull(G.child("TSU").child_value(),&pEnd,16);

    // The lowest 15 bits go into the upper bits of the previous register
    // [17-31]
    mask = 0x7FFF;
    bit_offset = 17;
    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) |= (input_longword & mask ) << bit_offset;
    register_map_[reg+reg_offset+(i*3)+1].value |= (input_longword & mask ) << bit_offset;
    Log(debug,"Mask register %u (W2) %X",reg+reg_offset+(i*3)+1,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );

//    Log(verbose,"Mask register " << std::dec << reg+reg_offset+(i*3)+1 << " (W2)" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+1].address) );

    // The next 32 (47-15) bits go into the next register
    mask = 0xFFFFFFFF;
    bit_offset = 15;

    *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) |= (input_longword >> bit_offset) & mask;
    register_map_[reg+reg_offset+(i*3)+2].value |= (input_longword >> bit_offset) & mask;
    Log(debug,"Mask register %u (W3) %X",reg+reg_offset+(i*3)+2,*(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) );

//    Log(verbose,"Mask register " << std::dec << reg+reg_offset+(i*3)+2 << " (W3)" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg+reg_offset+(i*3)+2].address) );

    // The final bit (48) goes into the lsb of the configuration register
    mask = 0x1;
    bit_offset = i;
    *(volatile uint32_t*)(register_map_[reg].address) |= ((input_longword >> 47) && mask) << bit_offset;
    register_map_[reg].value |= ((input_longword >> 47) && mask) << bit_offset;
    Log(debug,"Mask register %u (MASK) %X",reg,*(volatile uint32_t*)(register_map_[reg].address) );
//
//    Log(verbose,"Conf register " << std::dec << reg << " (MASK)" << std::hex
//        << *(volatile uint32_t*)(register_map_[reg].address) );
  }

}

void PTBManager::LoadDefaultConfig() {
  // File opened alright. Proceed with the parsing
  // Instanciate the XML plugin
  try {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(default_config_);
    Log(verbose,"Load result : %s",result.description() );
    pugi::xml_node config = doc.child("config");
    if (config == NULL) {
      Log(error,"Failed to load the config block from input file [%s]",default_config_);
    } else {
      ProcessConfig(config);
    }
  }
  catch (pugi::xpath_exception &e) {
    Log(error,"PUGIXML exception caught: %s",e.what() );
    return;
  }
  catch (std::exception &e) {
    Log(error,"STD exception caught: %s",e.what() );
    return;
  }
  catch (...) {
    Log(error,"Unknown exception caught. This is going to mean trouble." );
    return;
  }
}
