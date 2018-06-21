

namespace ptb {

#define NBEAM_CH 9
#define NPDS_CH 24

    ////////////////////////////////////////////
    // Beam Configuration

    void board_manager::configure_ctb(json& doc, json& answers) {

      try { 

        //Keep this greeting, otherwise there will be an error when trying to insert into the empty "answers" object
        json obj;
        obj["type"] = "info";
        obj["message"] = "Beginning CTB configuration! ";
        answers.push_back(obj);
        
        // -- Grab the subsystem & Misc configurations
        json miscconf = doc.at("ctb").at("misc");
        json beamconf = doc.at("ctb").at("subsystems").at("beam");
        json crtconf = doc.at("ctb").at("subsystems").at("crt");
        json pdsconf = doc.at("ctb").at("subsystems").at("pds");
                                                                                                
        //FIXME: Introduce json parse feedback here.
        // NFB: We should NEVER, NEVER, NEVER parse a file without properly caught exceptions
        // Otherwise we have no way to know if the configuration fails
                                                                                                
        json mfeedback;
        misc_config(miscconf, mfeedback);
        if (!mfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the Misc Configs",mfeedback.size());
          answers.insert(answers.end(),mfeedback.begin(),mfeedback.end());
        }
                                                                                                
        json bfeedback;
        beam_config(beamconf, bfeedback);
        if (!bfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the Beam",bfeedback.size());
          answers.insert(answers.end(),bfeedback.begin(),bfeedback.end());
        }
                                                                                                
        json cfeedback;
        crt_config(crtconf, cfeedback);
        if (!cfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the CRT",cfeedback.size());
          answers.insert(answers.end(),cfeedback.begin(),cfeedback.end());
        }
                                                                                                
                                                                                                
  #ifdef NO_PDS_DAC
        Log(warning,"PDS configuration block was disabled. Not configuring any PDS input");
  #else
                                                                                                
        json feedback;
        pds_config(pdsconf,feedback);
        if (!feedback.empty()) {
          Log(debug,"Received %u messages from configuring the PDS",feedback.size());
          answers.insert(answers.end(),feedback.begin(),feedback.end());
        }
                                                                                                
  #endif
        } //try
       
        catch(std::exception &e) {

         std::string msg = "Error processing configuration: ";
         msg += e.what();
         msg += ". Not committing configuration to CTB.";
         json obj;
         obj["type"] = "error";
         obj["message"] = msg;
         answers.push_back(obj);
       }
       catch (json::exception& e) {

         std::string msg = "Error processing configuration: ";
         msg += e.what();
         msg += ". Not committing configuration to PTB.";
         json obj;
         obj["type"] = "error";
         obj["message"] = msg;
         answers.push_back(obj);
       }
       catch(...) {
         json obj;
         obj["type"] = "error";
         obj["message"] = "Unknown error processing configuration. Not committing configuration to CTB";
         answers.push_back(obj);
       }

    }

    void board_manager::beam_config(json& beamconfig, json& feedback) {

      size_t err_msgs = 8;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      json trigs = beamconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = beamconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 <<  "Beam channel mask set for 0x0.";
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());
      //Mask type triggers, inclusive & exclusive
      bool llt3_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_llt3mask = trigs.at(0).at("mask").get<std::string>();
      uint32_t llt3mask = (int)strtol(s_llt3mask.c_str(),NULL,0);
      //if (llt3_enable) { tmpi[0] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " " << trigs.at(0).at("description").get<std::string>() << " configured"; }
      std::ostringstream oss1;
      if (llt3mask == 0) { 
        oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss1.str());
      }
      tmp[1] = std::make_pair("warning", oss1.str());
      bool llt4_enable = trigs.at(1).at("enable").get<bool>();
      std::string s_llt4mask = trigs.at(1).at("mask").get<std::string>();
      uint32_t llt4mask = (int)strtol(s_llt4mask.c_str(),NULL,0);
      std::ostringstream oss2;
      if (llt4mask == 0) { 
        oss2 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());
      std::string s_llt5mask = trigs.at(2).at("mask").get<std::string>();
      bool llt5_enable = trigs.at(2).at("enable").get<bool>();
      uint32_t llt5mask = (int)strtol(s_llt5mask.c_str(),NULL,0);
      std::ostringstream oss3;
      if (llt5mask == 0) { 
        oss3 << "Trigger " << trigs.at(2).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss3.str());
      }
      tmp[3] = std::make_pair("warning", oss3.str());
      bool llt6_enable = trigs.at(3).at("enable").get<bool>();
      std::string s_llt6mask = trigs.at(3).at("mask").get<std::string>();
      uint32_t llt6mask = (int)strtol(s_llt6mask.c_str(),NULL,0);
      std::ostringstream oss4;
      if (llt6mask == 0) { 
        oss4 << "Trigger " << trigs.at(3).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss4.str());
      }
      tmp[4] = std::make_pair("warning", oss4.str());
      
      //Counting triggers
       bool llt12_enable = trigs.at(4).at("enable").get<bool>();
       std::string s_llt12mask = trigs.at(4).at("mask").get<std::string>();
       std::string s_llt12count = trigs.at(4).at("count").get<std::string>();
       std::string s_llt12type = trigs.at(4).at("type").get<std::string>();
       uint32_t llt12mask = (int)strtol(s_llt12mask.c_str(),NULL,0);
       uint8_t llt12count = (int)strtol(s_llt12count.c_str(),NULL,0);
       uint8_t llt12type = (int)strtol(s_llt12type.c_str(),NULL,0);
       std::ostringstream oss5;
       if ((llt12count < 2 && llt12type == 0) || (llt12count < 1 && llt12type == 1)) { 
         oss5 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") will result in always asserted condition.";
         Log(warning,"%s", oss5.str());
       }
       tmp[5] = std::make_pair("warning", oss5.str());
       std::ostringstream oss6;
       if (llt12mask == 0) { 
        oss6 << "Trigger " << trigs.at(4).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss6.str());
       }
       tmp[6] = std::make_pair("warning", oss6.str());
       std::ostringstream oss7;
       if (llt12type > 4 || llt12type == 3) {
         oss7 << "Trigger " << trigs.at(4).at("id").get<std::string>() << " type (" << (int)llt12type << ") logic type undefined! Setting to '==' (0x1). ";
         Log(warning,"%s", oss7.str());
         llt12type = 0;  //Double check this is actually "=="
       }
       tmp[7] = std::make_pair("warning", oss7.str());

       //Input channel masks
       set_bit_range_register(3,0,9,channelmask);
       //Trigger enables
       set_bit(27,3,llt3_enable);
       set_bit(27,4,llt4_enable);
       set_bit(27,5,llt5_enable);
       set_bit(27,6,llt6_enable);
       set_bit(27,12,llt12_enable);

       uint32_t llt12conf = (llt12type<<5) + llt12count;
       //Trigger parameters
       set_bit_range_register(30,0,32,llt3mask);
       set_bit_range_register(31,0,32,llt4mask);
       set_bit_range_register(32,0,32,llt5mask);
       set_bit_range_register(33,0,32,llt6mask);
       set_bit_range_register(39,0,8,llt12conf);
       //set_bit_range_register(40,0,32,llt12mask);

       uint8_t dbits = 7;
       // PDS delays 24ch * 7b = 168b --> 6 regs
       for(size_t i=0; i<(size_t)(delays.size()/4); i++) {

         int j = i * 4;
         uint32_t reg = i + 21;
         uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

         set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
       }

       set_bit_range_register(23,0,(uint32_t)dbits,delays[delays.size()-1]);

       //Place warnings into json feedback obj
       json obj;
       for (size_t i=0; i<tmp.size(); i++) {
         if (tmp[i].second.compare("") != 0) {
           obj["type"] = tmp[i].first;
           obj["message"] = tmp[i].second;
           feedback.push_back(obj);
         }
       }

      //Log(info,"Programmed [%d] (%u) [0x%X][%s]",rtriggerfreq,rtriggerfreq,rtriggerfreq, std::bitset<26>(rtriggerfreq).to_string().c_str());

    } //Beam config

    ////////////////////////////////////////////
    // CRT configuration

    void board_manager::crt_config(json& crtconfig, json& feedback) {

      size_t err_msgs = 3;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      json trigs = crtconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = crtconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = crtconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 << "CRT channel mask set for 0x0."; 
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());

      //Mask type triggers (inclusive & exclusive)
      bool llt2_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_llt2mask = trigs.at(0).at("mask").get<std::string>();
      uint32_t llt2mask = (int)strtol(s_llt2mask.c_str(),NULL,0);
      std::ostringstream oss1;
      if (llt2mask == 0) { 
        oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled!"; 
        Log(warning,"%s", oss1.str());
      }
      tmp[1] = std::make_pair("warning", oss1.str());
      bool llt7_enable = trigs.at(1).at("enable").get<bool>();
      std::string s_llt7mask = trigs.at(1).at("mask").get<std::string>();
      uint32_t llt7mask = (int)strtol(s_llt7mask.c_str(),NULL,0);
      std::ostringstream oss2;
      if (llt7mask == 0) { 
        oss2 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled!"; 
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());

      //Input channel masks
      set_bit_range_register(1,0,32,channelmask);
      //Trigger enables
      set_bit(27,2,llt2_enable);
      set_bit(27,7,llt7_enable);
      //Trigger parameters
      set_bit_range_register(29,0,32,llt2mask);
      set_bit_range_register(34,0,32,llt7mask);

      uint8_t dbits = 7;
      // PDS delays 24ch * 7b = 168b --> 6 regs
      for(size_t i=0; i<delays.size()/4; i++) {

        int j = i * 4;
        uint32_t reg = i + 13;
        uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

        set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
      }

      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].second.compare("") != 0) {
          obj["type"] = tmp[i].first;
          obj["message"] = tmp[i].second;
          feedback.push_back(obj);
        }
      }

    } //CRT config

    ////////////////////////////////////////////
    // PDS configuration

    void board_manager::pds_config(json& pdsconfig, json& feedback) {
      ///FIXME: How can this work? dacsetup is not initialized in this function
      i2conf dacsetup;

      size_t err_msgs = 5;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
      std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = pdsconfig.at("delays").get<std::vector<uint32_t>>();
      std::string s_trigtype0 = pdsconfig.at("triggers").at(0).at("type").get<std::string>();
      std::string s_count0 = pdsconfig.at("triggers").at(0).at("count").get<std::string>();
      bool llt11_enable = pdsconfig.at("triggers").at(0).at("enable").get<bool>();

      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      uint8_t trigtype0 = (int)strtol(s_trigtype0.c_str(),NULL,0);
      uint8_t count0 = (int)strtol(s_count0.c_str(),NULL,0);

      std::ostringstream oss0;
      if (delays.size() != NPDS_CH) {
        oss0 << "Number of configuration values " << delays.size() << " doesn't match number of PDS channels " << NPDS_CH << " !";
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());

       for (size_t i=0; i<delays.size(); i++) {
         if (delays[i] > std::pow(2,7)) { //Range 0 - 2^7 -1
           std::ostringstream oss1;
           oss1 << "Delay value out of range (" << delays.at(i) << "). Truncating to maximum (127)";
           Log(warning,"%s", oss1.str());
           tmp[1] = std::make_pair("warning", oss1.str());
         }
       }

      std::ostringstream oss2;
      if (dac_values.size() != (i2conf::nchannels_)*(i2conf::ndacs_)) {
        oss2 << "Number of configuration values (" << dac_values.size() << ") doesn't match number of DAC channels (" << (i2conf::nchannels_)*(i2conf::ndacs_) << ")";
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());

      Log(info,"Size of channel values vector %i", dac_values.size());
      for (size_t i=0; i<dac_values.size(); i++) {
        Log(info,"Channel %zu value %u", i, dac_values[i]);
        if (dac_values[i] > 4095) { //Range 0 - 4095
          std::ostringstream oss3;
          oss3 << "DAC value out of range (" << dac_values.at(i) << "). Truncating to maximum (4095)";
          tmp[3] = std::make_pair("warning", oss3.str());
          Log(warning,"%s", oss3.str());
          dac_values[i] = 4095;
        }
      }
      //Now pass DAC configs to setup
      std::ostringstream oss4;
      if (dacsetup.ConfigureDacs(dac_values,false)) {
        oss4 << "Failed to write configuration values to DACs.";
        Log(warning,"%s", oss4.str());
      }
      tmp[4] = std::make_pair("error", oss4.str());
      Log(info,"Programmed %zu DAC channels", dac_values.size());

      //Input channel masks
      set_bit_range_register(2,0,24,channelmask);

      //Configure counting trigger 0
      uint32_t trig0 = (trigtype0<<5) + count0;
      set_bit_range_register(38,0,9,trig0);
      set_bit(27,11,llt11_enable);

     uint8_t dbits = 7;
     // PDS delays 24ch * 7b = 168b --> 6 regs
     for(size_t i=0; i<delays.size()/4; i++) {

       int j = i * 4;
       uint32_t reg = i + 7;
       uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

       set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
     }

     //Place warnings into json feedback obj
     json obj;
     for (size_t i=0; i<tmp.size(); i++) {
       if (tmp[i].second.compare("") != 0) {
         obj["type"] = tmp[i].first;
         obj["message"] = tmp[i].second;
         feedback.push_back(obj);
       }
     }

    } //PDS config

    ////////////////////////////////////////////
    // Misc configuration

    void board_manager::misc_config(json& miscconfig, json& feedback) {

      size_t err_msgs = 2;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);
     
      json rtrigger = miscconfig.at("randomtrigger");
      bool rtrigger_en = rtrigger.at("enable").get<bool>();
      uint32_t rtriggerfreq = rtrigger.at("frequency").get<unsigned int>();
      Log(debug,"Random Trigger Frequency [%d] (%u) [0x%X][%s]",rtriggerfreq,rtriggerfreq,rtriggerfreq, std::bitset<26>(rtriggerfreq).to_string().c_str());
      if (rtriggerfreq >= (1<<26)) {
        std::ostringstream oss0;
        oss0 << "Random trigger value of " << rtriggerfreq << " above maximum rollover [2^26 - 1]. Truncating to maximum.";
        tmp[0] = std::make_pair("warning", oss0.str());
        Log(warning,"%s", oss0.str());
        rtriggerfreq = (1<<26)-1;
      }
      set_bit(27,0,rtrigger_en);
      set_bit_range_register(25,0,26,rtriggerfreq);
      //Set pulser frequency
      json pulserconf = miscconfig.at("pulser");
      bool pulser_en = pulserconf.at("enable").get<bool>();
      uint32_t pulserfreq = pulserconf.at("frequency").get<unsigned int>();
      Log(debug,"Pulser Frequency [%d] (%u) [0x%X][%s]",pulserfreq,pulserfreq,pulserfreq, std::bitset<26>(pulserfreq).to_string().c_str());
      if (pulserfreq >= (1<<26)) {
        pulserfreq = (1<<26)-1;
        std::ostringstream oss1;
        oss1 << "Pulser value of " << pulserfreq << " above maximum rollover [2^26 - 1]. Truncating to maximum.";
        tmp[1] = std::make_pair("warning", oss1.str());
        Log(warning,"%s", oss1.str());
      }
      set_bit(26,31,pulser_en);
      set_bit_range_register(26,0,26,pulserfreq);

      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].second.compare("") != 0) {
          obj["type"] = tmp[i].first;
          obj["message"] = tmp[i].second;
          feedback.push_back(obj);
        }
      }

    } // Misc configs

}
