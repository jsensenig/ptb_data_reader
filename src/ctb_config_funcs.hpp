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
        json hltconf = doc.at("ctb").at("HLT");
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

        json hfeedback;
        hlt_config(hltconf, hfeedback);
        if (!hfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the HLTs",hfeedback.size());
          answers.insert(answers.end(),hfeedback.begin(),hfeedback.end());
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

      size_t err_msgs = 7;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      json trigs = beamconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = beamconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);

      uint32_t reshape_len = beamconfig.at("reshape_length").get<unsigned int>();
      Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      if (reshape_len >= (1<<6)) {
        std::ostringstream oss6;
        oss6 << "Random trigger value of " << reshape_len << " above maximum rollover [2^6 - 1]. Truncating to maximum.";
        tmp[6] = std::make_pair("warning", oss6.str());
        Log(warning,"%s", oss6.str());
        reshape_len = (1<<6)-1;
      }

      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 <<  "Beam channel mask set for 0x0.";
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());
      //Mask type triggers, inclusive & exclusive
      bool llt1_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_llt1mask = trigs.at(0).at("mask").get<std::string>();
      uint32_t llt1mask = (int)strtol(s_llt1mask.c_str(),NULL,0);
      //if (llt1_enable) { tmpi[0] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " " << trigs.at(0).at("description").get<std::string>() << " configured"; }
      std::ostringstream oss1;
      if (llt1mask == 0) { 
        oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss1.str());
      }
      tmp[1] = std::make_pair("warning", oss1.str());
      bool llt3_enable = trigs.at(1).at("enable").get<bool>();
      std::string s_llt3mask = trigs.at(1).at("mask").get<std::string>();
      uint32_t llt3mask = (int)strtol(s_llt3mask.c_str(),NULL,0);
      //if (llt3_enable) { tmpi[0] << "Trigger " << trigs.at(0).at("id").get<std::string>() << " " << trigs.at(0).at("description").get<std::string>() << " configured"; }
      std::ostringstream oss2;
      if (llt3mask == 0) { 
        oss2 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());
      bool llt4_enable = trigs.at(2).at("enable").get<bool>();
      std::string s_llt4mask = trigs.at(2).at("mask").get<std::string>();
      uint32_t llt4mask = (int)strtol(s_llt4mask.c_str(),NULL,0);
      std::ostringstream oss3;
      if (llt4mask == 0) { 
        oss3 << "Trigger " << trigs.at(2).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss3.str());
      }
      tmp[3] = std::make_pair("warning", oss3.str());
      std::string s_llt5mask = trigs.at(3).at("mask").get<std::string>();
      bool llt5_enable = trigs.at(3).at("enable").get<bool>();
      uint32_t llt5mask = (int)strtol(s_llt5mask.c_str(),NULL,0);
      std::ostringstream oss4;
      if (llt5mask == 0) { 
        oss4 << "Trigger " << trigs.at(3).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss4.str());
      }
      tmp[4] = std::make_pair("warning", oss4.str());
      bool llt6_enable = trigs.at(4).at("enable").get<bool>();
      std::string s_llt6mask = trigs.at(4).at("mask").get<std::string>();
      uint32_t llt6mask = (int)strtol(s_llt6mask.c_str(),NULL,0);
      std::ostringstream oss5;
      if (llt6mask == 0) { 
        oss5 << "Trigger " << trigs.at(4).at("id").get<std::string>() << " set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss5.str());
      }
      tmp[5] = std::make_pair("warning", oss5.str());
      
      //Input channel masks
      set_bit_range_register(3,0,9,channelmask);
      set_bit_range_register(24,26,6,0);

      //Trigger enables
      set_bit(27,1,llt1_enable);
      set_bit(27,3,llt3_enable);
      set_bit(27,4,llt4_enable);
      set_bit(27,5,llt5_enable);
      set_bit(27,6,llt6_enable);

      //Trigger parameters
      set_bit_range_register(28,0,32,llt1mask);
      set_bit_range_register(30,0,32,llt3mask);
      set_bit_range_register(31,0,32,llt4mask);
      set_bit_range_register(32,0,32,llt5mask);
      set_bit_range_register(33,0,32,llt6mask);

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

      size_t err_msgs = 32;
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

     uint32_t reshape_len = crtconfig.at("reshape_length").get<unsigned int>();
      Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      if (reshape_len >= (1<<6)) {
        std::ostringstream oss31;
        oss31 << "Random trigger value of " << reshape_len << " above maximum rollover [2^6 - 1]. Truncating to maximum.";
        tmp[31] = std::make_pair("warning", oss31.str());
        Log(warning,"%s", oss31.str());
        reshape_len = (1<<6)-1;
      }

       //Counting triggers
       bool llt11_enable = trigs.at(0).at("enable").get<bool>();
       std::string s_llt11mask = trigs.at(0).at("mask").get<std::string>();
       std::string s_llt11count = trigs.at(0).at("count").get<std::string>();
       std::string s_llt11type = trigs.at(0).at("type").get<std::string>();
       uint32_t llt11mask = (int)strtol(s_llt11mask.c_str(),NULL,0);
       uint8_t llt11count = (int)strtol(s_llt11count.c_str(),NULL,0);
       uint8_t llt11type = (int)strtol(s_llt11type.c_str(),NULL,0);
                                                                                                                                                                                                         
       std::ostringstream oss1;
       if (llt11type > 4 || llt11type == 3 || llt11type == 0) {
         oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " type (" << (int)llt11type << ") logic type undefined! ";
         Log(error,"%s", oss1.str());
         //FIXME Add return, this should be an error
       }
       tmp[1] = std::make_pair("warning", oss1.str());
       std::ostringstream oss2;
       if ((llt11count == 0 && llt11type == 2)) { 
         oss2 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
         Log(warning,"%s", oss2.str());
       }
       tmp[2] = std::make_pair("warning", oss2.str());
       std::ostringstream oss3;
       if ((llt11count == 0 && llt11type == 4)) { 
         oss3 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is asking for negative count. Trigger disabled.";
         Log(warning,"%s", oss3.str());
       }
       tmp[3] = std::make_pair("warning", oss3.str());
       std::ostringstream oss4;
       if ((llt11count == 1 && llt11type == 4)) { 
         oss4 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
         Log(warning,"%s", oss4.str());
       }
       tmp[4] = std::make_pair("warning", oss4.str());
       std::ostringstream oss5;
       if ((llt11count > llt11mask) && (llt11type == 2 || llt11type == 1)) { 
         oss5 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
         Log(warning,"%s", oss5.str());
       }
       tmp[5] = std::make_pair("warning", oss5.str());
       std::ostringstream oss6;
       if (llt11count > std::pow(2,5)-1) { 
         oss6 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") is larger than the number of available bits. Configuration aborted.";
         Log(error,"%s", oss6.str());
       }
       tmp[6] = std::make_pair("warning", oss6.str());

       
       bool llt12_enable = trigs.at(1).at("enable").get<bool>();
       std::string s_llt12mask = trigs.at(1).at("mask").get<std::string>();         
       std::string s_llt12count = trigs.at(1).at("count").get<std::string>();                                                                           
       std::string s_llt12type = trigs.at(1).at("type").get<std::string>();                                                                                       
       uint32_t llt12mask = (int)strtol(s_llt12mask.c_str(),NULL,0);                                                                                                                      
       uint8_t llt12count = (int)strtol(s_llt12count.c_str(),NULL,0);                                                                                                      
       uint8_t llt12type = (int)strtol(s_llt12type.c_str(),NULL,0);                                                                                                            
                                                                                                                                                                              
       std::ostringstream oss7;                                                                                                                                             
       if (llt12type > 4 || llt12type == 3 || llt12type == 0) {                                                                                                   
         oss7 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " type (" << (int)llt12type << ") logic type undefined! ";                                          
         Log(error,"%s", oss7.str());                                                                                                                        
         //FIXME Add return, this should be an error                                                                                                                       
       }                                                                                                                                                   
       tmp[7] = std::make_pair("warning", oss7.str());                                                                                                          
       std::ostringstream oss8;
       if ((llt12count == 0 && llt12type == 2)) { 
         oss8 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
         Log(warning,"%s", oss8.str());
       }
       tmp[8] = std::make_pair("warning", oss8.str());
       std::ostringstream oss9;
       if ((llt12count == 0 && llt12type == 4)) { 
         oss9 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is asking for negative count. Trigger disabled.";
         Log(warning,"%s", oss9.str());
       }
       tmp[9] = std::make_pair("warning", oss9.str());
       std::ostringstream oss10;
       if ((llt12count == 1 && llt12type == 4)) { 
         oss10 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
         Log(warning,"%s", oss10.str());
       }
       tmp[10] = std::make_pair("warning", oss10.str());
       std::ostringstream oss11;
       if ((llt12count > llt12mask) && (llt12type == 2 || llt12type == 1)) { 
         oss11 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
         Log(warning,"%s", oss11.str());
       }
       tmp[11] = std::make_pair("warning", oss11.str());
       std::ostringstream oss12;
       if (llt12count > std::pow(2,5)-1) { 
         oss12 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") is larger than the number of available bits. Configuration aborted.";
         Log(error,"%s", oss12.str());
       }
       tmp[12] = std::make_pair("warning", oss12.str());

        bool llt13_enable = trigs.at(1).at("enable").get<bool>();                                                                                                                                          
        std::string s_llt13mask = trigs.at(2).at("mask").get<std::string>();
        std::string s_llt13count = trigs.at(2).at("count").get<std::string>();
        std::string s_llt13type = trigs.at(2).at("type").get<std::string>();
        uint32_t llt13mask = (int)strtol(s_llt13mask.c_str(),NULL,0);
        uint8_t llt13count = (int)strtol(s_llt13count.c_str(),NULL,0);
        uint8_t llt13type = (int)strtol(s_llt13type.c_str(),NULL,0);
                                                                                                                                                                                                          
        std::ostringstream oss13;
        if (llt13type > 4 || llt13type == 3 || llt13type == 0) {
          oss13 << "Trigger " << trigs.at(2).at("id").get<std::string>() << " type (" << (int)llt13type << ") logic type undefined! ";
          Log(error,"%s", oss13.str());
          //FIXME Add return, this should be an error
        }
        tmp[13] = std::make_pair("warning", oss13.str());
       std::ostringstream oss14;
       if ((llt13count == 0 && llt13type == 2)) { 
         oss14 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
         Log(warning,"%s", oss14.str());
       }
       tmp[14] = std::make_pair("warning", oss14.str());
       std::ostringstream oss15;
       if ((llt13count == 0 && llt13type == 4)) { 
         oss15 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is asking for negative count. Trigger disabled.";
         Log(warning,"%s", oss15.str());
       }
       tmp[15] = std::make_pair("warning", oss15.str());
       std::ostringstream oss16;
       if ((llt13count == 1 && llt13type == 4)) { 
         oss16 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
         Log(warning,"%s", oss16.str());
       }
       tmp[16] = std::make_pair("warning", oss16.str());
       std::ostringstream oss17;
       if ((llt13count > llt13mask) && (llt13type == 2 || llt13type == 1)) { 
         oss17 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
         Log(warning,"%s", oss17.str());
       }
       tmp[17] = std::make_pair("warning", oss17.str());
       std::ostringstream oss18;
       if (llt13count > std::pow(2,5)-1) { 
         oss18 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") is larger than the number of available bits. Configuration aborted.";
         Log(error,"%s", oss18.str());
       }
       tmp[18] = std::make_pair("warning", oss18.str());

        bool llt15_enable = trigs.at(3).at("enable").get<bool>();
        std::string s_llt15mask = trigs.at(3).at("mask").get<std::string>();
        std::string s_llt15count = trigs.at(3).at("count").get<std::string>();
        std::string s_llt15type = trigs.at(3).at("type").get<std::string>();
        uint32_t llt15mask = (int)strtol(s_llt15mask.c_str(),NULL,0);
        uint8_t llt15count = (int)strtol(s_llt15count.c_str(),NULL,0);
        uint8_t llt15type = (int)strtol(s_llt15type.c_str(),NULL,0);
                                                                                                                                                                                                          
        std::ostringstream oss19;
        if (llt15type > 4 || llt15type == 3 || llt15type == 0) {
          oss19 << "Trigger " << trigs.at(3).at("id").get<std::string>() << " type (" << (int)llt15type << ") logic type undefined! ";
          Log(error,"%s", oss19.str());
          //FIXME Add return, this should be an error
        }
        tmp[19] = std::make_pair("warning", oss19.str());
       std::ostringstream oss20;
       if ((llt15count == 0 && llt15type == 2)) { 
         oss20 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
         Log(warning,"%s", oss20.str());
       }
       tmp[20] = std::make_pair("warning", oss20.str());
       std::ostringstream oss21;
       if ((llt15count == 0 && llt15type == 4)) { 
         oss21 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is asking for negative count. Trigger disabled.";
         Log(warning,"%s", oss21.str());
       }
       tmp[21] = std::make_pair("warning", oss21.str());
       std::ostringstream oss22;
       if ((llt15count == 1 && llt15type == 4)) { 
         oss22 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
         Log(warning,"%s", oss22.str());
       }
       tmp[22] = std::make_pair("warning", oss22.str());
       std::ostringstream oss23;
       if ((llt15count > llt15mask) && (llt15type == 2 || llt15type == 1)) { 
         oss23 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
         Log(warning,"%s", oss23.str());
       }
       tmp[23] = std::make_pair("warning", oss23.str());
       std::ostringstream oss24;
       if (llt15count > std::pow(2,5)-1) { 
         oss24 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") is larger than the number of available bits. Configuration aborted.";
         Log(error,"%s", oss24.str());
       }
       tmp[24] = std::make_pair("warning", oss24.str());

        bool llt16_enable = trigs.at(4).at("enable").get<bool>();
        std::string s_llt16mask = trigs.at(4).at("mask").get<std::string>();
        std::string s_llt16count = trigs.at(4).at("count").get<std::string>();
        std::string s_llt16type = trigs.at(4).at("type").get<std::string>();
        uint32_t llt16mask = (int)strtol(s_llt16mask.c_str(),NULL,0);
        uint8_t llt16count = (int)strtol(s_llt16count.c_str(),NULL,0);
        uint8_t llt16type = (int)strtol(s_llt16type.c_str(),NULL,0);
                                                                                                                                                                                                          
        std::ostringstream oss25;
        if (llt16type > 4 || llt16type == 3 || llt16type == 0) {
          oss25 << "Trigger " << trigs.at(4).at("id").get<std::string>() << " type (" << (int)llt16type << ") logic type undefined! ";
          Log(error,"%s", oss25.str());
          //FIXME Add return, this should be an error
        }
        tmp[25] = std::make_pair("warning", oss25.str());
       std::ostringstream oss26;
       if ((llt16count == 0 && llt16type == 2)) { 
         oss26 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is equivalent to disabled.";
         Log(warning,"%s", oss26.str());
       }
       tmp[26] = std::make_pair("warning", oss26.str());
       std::ostringstream oss27;
       if ((llt16count == 0 && llt16type == 4)) { 
         oss27 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is asking for negative count. Trigger disabled.";
         Log(warning,"%s", oss27.str());
       }
       tmp[27] = std::make_pair("warning", oss27.str());
       std::ostringstream oss28;
       if ((llt16count == 1 && llt16type == 4)) { 
         oss28 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is equivalent to disabled.";
         Log(warning,"%s", oss28.str());
       }
       tmp[28] = std::make_pair("warning", oss28.str());
       std::ostringstream oss29;
       if ((llt16count > llt15mask) && (llt15type == 2 || llt15type == 1)) { 
         oss29 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
         Log(warning,"%s", oss29.str());
       }
       tmp[29] = std::make_pair("warning", oss29.str());
       std::ostringstream oss30;
       if (llt15count > std::pow(2,5)-1) { 
         oss30 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") is larger than the number of available bits. Configuration aborted.";
         Log(error,"%s", oss30.str());
       }
       tmp[30] = std::make_pair("warning", oss30.str());

      //Input channel masks
      set_bit_range_register(1,0,32,channelmask);
      set_bit_range_register(24,20,6,0);

      //Trigger enables
      set_bit(27,11,llt11_enable);
      set_bit(27,12,llt12_enable);
      set_bit(27,13,llt13_enable);
      set_bit(27,15,llt15_enable);
      set_bit(27,16,llt16_enable);

      //Trigger parameters
      uint32_t llt11 = (llt11type<<5) + llt11count;
      uint32_t llt12 = (llt12type<<5) + llt12count;
      uint32_t llt13 = (llt13type<<5) + llt13count;
      uint32_t llt15 = (llt15type<<5) + llt15count;
      uint32_t llt16 = (llt16type<<5) + llt16count;

      set_bit_range_register(38,0,9,llt11);
      set_bit_range_register(39,0,32,llt11mask);
      set_bit_range_register(40,0,9,llt12);
      set_bit_range_register(41,0,32,llt12mask);
      set_bit_range_register(42,0,9,llt13);
      set_bit_range_register(43,0,32,llt13mask);
      set_bit_range_register(46,0,9,llt15);
      set_bit_range_register(47,0,32,llt15mask);
      set_bit_range_register(48,0,9,llt16);
      set_bit_range_register(49,0,32,llt16mask);

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

      size_t err_msgs = 12;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      json trigs = pdsconfig.at("triggers"); //Array of trigger configs

      std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
      std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
      std::vector<uint32_t> delays = pdsconfig.at("delays").get<std::vector<uint32_t>>();
      std::string s_llt14type = trigs.at(0).at("type").get<std::string>();
      std::string s_llt14count = trigs.at(0).at("count").get<std::string>();
      std::string s_llt14mask = trigs.at(0).at("mask").get<std::string>();
      bool llt14_enable = trigs.at(0).at("enable").get<bool>();

      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      uint8_t llt14type = (int)strtol(s_llt14type.c_str(),NULL,0);
      uint8_t llt14count = (int)strtol(s_llt14count.c_str(),NULL,0);
      uint32_t llt14mask = (int)strtol(s_llt14mask.c_str(),NULL,0);

      uint32_t reshape_len = pdsconfig.at("reshape_length").get<unsigned int>();
      Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      if (reshape_len >= (1<<6)) {
         std::ostringstream oss11;
         oss11 << "Random trigger value of " << reshape_len << " above maximum rollover [2^6 - 1]. Truncating to maximum.";
         tmp[11] = std::make_pair("warning", oss11.str());
         Log(warning,"%s", oss11.str());
         reshape_len = (1<<6)-1;
      }



       std::ostringstream oss0;
       if (llt14type > 4 || llt14type == 3 || llt14type == 0) {
         oss0 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " type (" << (int)llt14type << ") logic type undefined! ";
         Log(error,"%s", oss0.str());
         //FIXME Add return, this should be an error
       }
       tmp[0] = std::make_pair("warning", oss0.str());
      std::ostringstream oss1;
      if ((llt14count == 0 && llt14type == 2)) { 
        oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is equivalent to disabled.";
        Log(warning,"%s", oss1.str());
      }
      tmp[1] = std::make_pair("warning", oss1.str());
      std::ostringstream oss2;
      if ((llt14count == 0 && llt14type == 4)) { 
        oss2 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is asking for negative count. Trigger disabled.";
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());
      std::ostringstream oss3;
      if ((llt14count == 1 && llt14type == 4)) { 
        oss3 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is equivalent to disabled.";
        Log(warning,"%s", oss3.str());
      }
      tmp[3] = std::make_pair("warning", oss3.str());
      std::ostringstream oss4;
      if ((llt14count > llt14mask) && (llt14type == 2 || llt14type == 1)) { 
        oss4 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is equivalent to disabled.";
        Log(warning,"%s", oss4.str());
      }
      tmp[4] = std::make_pair("warning", oss4.str());
      std::ostringstream oss5;
      if (llt14count > std::pow(2,5)-1) { 
        oss5 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") is larger than the number of available bits. Configuration aborted.";
        Log(error,"%s", oss5.str());
      }
      tmp[5] = std::make_pair("warning", oss5.str());

      std::ostringstream oss6;
      if (delays.size() != NPDS_CH) {
        oss6 << "Number of configuration values " << delays.size() << " doesn't match number of PDS channels " << NPDS_CH << " !";
        Log(warning,"%s", oss6.str());
      }
      tmp[6] = std::make_pair("warning", oss6.str());

       for (size_t i=0; i<delays.size(); i++) {
         if (delays[i] > std::pow(2,7)) { //Range 0 - 2^7 -1
           std::ostringstream oss7;
           oss7 << "Delay value out of range (" << delays.at(i) << "). Truncating to maximum (127)";
           Log(warning,"%s", oss7.str());
           tmp[7] = std::make_pair("warning", oss7.str());
         }
       }

      std::ostringstream oss8;
      if (dac_values.size() != (i2conf::nchannels_)*(i2conf::ndacs_)) {
        oss8 << "Number of configuration values (" << dac_values.size() << ") doesn't match number of DAC channels (" << (i2conf::nchannels_)*(i2conf::ndacs_) << ")";
        Log(warning,"%s", oss8.str());
      }
      tmp[8] = std::make_pair("warning", oss8.str());

      Log(info,"Size of channel values vector %i", dac_values.size());
      for (size_t i=0; i<dac_values.size(); i++) {
        Log(info,"Channel %zu value %u", i, dac_values[i]);
        if (dac_values[i] > 4095) { //Range 0 - 4095
          std::ostringstream oss9;
          oss9 << "DAC value out of range (" << dac_values.at(i) << "). Truncating to maximum (4095)";
          tmp[9] = std::make_pair("warning", oss9.str());
          Log(warning,"%s", oss9.str());
          dac_values[i] = 4095;
        }
      }
      //Now pass DAC configs to setup
      std::ostringstream oss10;
      if (dacsetup.ConfigureDacs(dac_values,false)) {
        oss10 << "Failed to write configuration values to DACs.";
        Log(warning,"%s", oss10.str());
      }
      tmp[10] = std::make_pair("error", oss10.str());
      Log(info,"Programmed %zu DAC channels", dac_values.size());

      //Input channel masks
      set_bit_range_register(2,0,24,channelmask);

      set_bit_range_register(24,14,6,0);

      //Configure counting trigger(s)
      set_bit(27,11,llt14_enable);
      uint32_t llt14 = (llt14type<<5) + llt14count;
      set_bit_range_register(44,0,9,llt14);
      set_bit_range_register(45,0,32,llt14mask);

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

      //Timing endpoint parameters
      json timingconf = miscconfig.at("timing");
      std::string s_t_addr = timingconf.at("address").get<std::string>();
      std::string s_t_group = timingconf.at("group").get<std::string>();
      uint32_t t_addr = (int)strtol(s_t_addr.c_str(),NULL,0);
      uint32_t t_group = (int)strtol(s_t_group.c_str(),NULL,0);


      set_bit(27,0,rtrigger_en);
      set_bit(26,31,pulser_en);

      set_bit_range_register(25,0,26,rtriggerfreq);
      set_bit_range_register(26,0,26,pulserfreq);

      uint32_t t_param = (t_group<<8) + t_addr;

      set_bit_range_register(61,0,10,t_param);

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

    ////////////////////////////////////////////
    // HLT configuration
                                                                                                                              
    void board_manager::hlt_config(json& hltconfig, json& feedback) {
                                                                                                                                                            
      size_t err_msgs = 5;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);
     
      json trigs = hltconfig.at("trigger");

      bool hlt0_enable = trigs.at(0).at("enable").get<bool>();
      std::string s_hlt0minc = trigs.at(0).at("minc").get<std::string>();
      std::string s_hlt0mexc = trigs.at(0).at("mexc").get<std::string>();
      uint32_t hlt0minc = (int)strtol(s_hlt0minc.c_str(),NULL,0);
      uint32_t hlt0mexc = (int)strtol(s_hlt0mexc.c_str(),NULL,0);
      std::ostringstream oss0;
      if (hlt0minc == 0) { 
        oss0 << trigs.at(0).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());
 
      bool hlt1_enable = trigs.at(1).at("enable").get<bool>();                                                         
      std::string s_hlt1minc = trigs.at(1).at("minc").get<std::string>();
      std::string s_hlt1mexc = trigs.at(1).at("mexc").get<std::string>();
      uint32_t hlt1minc = (int)strtol(s_hlt1minc.c_str(),NULL,0);
      uint32_t hlt1mexc = (int)strtol(s_hlt1mexc.c_str(),NULL,0);
      std::ostringstream oss1;
      if (hlt1minc == 0) { 
        oss1 << trigs.at(1).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss1.str());
      }
      tmp[1] = std::make_pair("warning", oss1.str());

      bool hlt2_enable = trigs.at(2).at("enable").get<bool>();
      std::string s_hlt2minc = trigs.at(2).at("minc").get<std::string>();
      std::string s_hlt2mexc = trigs.at(2).at("mexc").get<std::string>();
      uint32_t hlt2minc = (int)strtol(s_hlt2minc.c_str(),NULL,0);
      uint32_t hlt2mexc = (int)strtol(s_hlt2mexc.c_str(),NULL,0);
      std::ostringstream oss2;
      if (hlt2minc == 0) { 
        oss2 << trigs.at(2).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss2.str());
      }
      tmp[2] = std::make_pair("warning", oss2.str());

      bool hlt3_enable = trigs.at(3).at("enable").get<bool>();                                                         
      std::string s_hlt3minc = trigs.at(3).at("minc").get<std::string>();
      std::string s_hlt3mexc = trigs.at(3).at("mexc").get<std::string>();
      uint32_t hlt3minc = (int)strtol(s_hlt3minc.c_str(),NULL,0);
      uint32_t hlt3mexc = (int)strtol(s_hlt3mexc.c_str(),NULL,0);
      std::ostringstream oss3;
      if (hlt3minc == 0) { 
        oss3 << trigs.at(3).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss3.str());
      }
      tmp[3] = std::make_pair("warning", oss3.str());

      bool hlt4_enable = trigs.at(4).at("enable").get<bool>();
      std::string s_hlt4minc = trigs.at(4).at("minc").get<std::string>();
      std::string s_hlt4mexc = trigs.at(4).at("mexc").get<std::string>();
      uint32_t hlt4minc = (int)strtol(s_hlt4minc.c_str(),NULL,0);
      uint32_t hlt4mexc = (int)strtol(s_hlt4mexc.c_str(),NULL,0);
      std::ostringstream oss4;
      if (hlt4minc == 0) { 
        oss4 << trigs.at(4).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled."; 
        Log(warning,"%s", oss4.str());
      }
      tmp[4] = std::make_pair("warning", oss4.str());

      //HLT enables
      set_bit(27,19,hlt0_enable);
      set_bit(27,20,hlt1_enable);
      set_bit(27,21,hlt2_enable);
      set_bit(27,22,hlt3_enable);
      set_bit(27,23,hlt4_enable);

      //The exclusive mask goes in 16 upper bits while the inclusive mask goes in the 16 lower bits
      uint32_t hlt0 = (hlt0mexc<<16) + hlt0minc;
      uint32_t hlt1 = (hlt1mexc<<16) + hlt1minc;
      uint32_t hlt2 = (hlt2mexc<<16) + hlt2minc;
      uint32_t hlt3 = (hlt3mexc<<16) + hlt3minc;
      uint32_t hlt4 = (hlt4mexc<<16) + hlt4minc;

      //HLT parameters
      set_bit_range_register(54,0,32,hlt0);
      set_bit_range_register(55,0,32,hlt1);
      set_bit_range_register(56,0,32,hlt2);
      set_bit_range_register(57,0,32,hlt3);
      set_bit_range_register(58,0,32,hlt4);


      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].second.compare("") != 0) {
          obj["type"] = tmp[i].first;
          obj["message"] = tmp[i].second;
          feedback.push_back(obj);
        }
      }
                                                                                                                                                            
    } // HLT configs


}
