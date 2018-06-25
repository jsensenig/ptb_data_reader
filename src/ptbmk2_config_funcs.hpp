

namespace ptb {

#define NBEAM_CH 3
#define NPDS_CH 10
#define NMTCA_CH 6

    ////////////////////////////////////////////
    // Beam Configuration

    void board_manager::configure_ptbmk2(json& doc, json& answers) {

      try { 

        //Keep this greeting, otherwise there will be an error when trying to insert into the empty "answers" object
        json obj;
        obj["type"] = "info";
        obj["message"] = "Beginning PTB Mark II configuration! ";
        answers.push_back(obj);
        
        // -- Grab the subsystem & Misc configurations
        json miscconf = doc.at("ctb").at("misc");
        json beamconf = doc.at("ctb").at("subsystems").at("beam");
        json crtconf = doc.at("ctb").at("subsystems").at("crt");
        json pdsconf = doc.at("ctb").at("subsystems").at("pds");
        json mtcaconf = doc.at("ctb").at("subsystems").at("mtcas");
        json nimconf = doc.at("ctb").at("subsystems").at("nim");
        //json testconf = doc.at("ctb").at("test");
                                                                                                
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
                                                                                                
        json pfeedback;
        pds_config(pdsconf,pfeedback);
        if (!pfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the PDS",pfeedback.size());
          answers.insert(answers.end(),pfeedback.begin(),pfeedback.end());
        }
                                                                                 
        json nfeedback;
        nim_config(nimconf,nfeedback);
        if (!nfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the NIM IO",nfeedback.size());
          answers.insert(answers.end(),nfeedback.begin(),nfeedback.end());
        }


               
  #ifdef NO_MTCA
        Log(warning,"MTCA configuration block was disabled. Not configuring any PDS input");
  #else

        json feedback;
        mtca_config(mtcaconf,feedback);
        if (!feedback.empty()) {
          Log(debug,"Received %u messages from configuring the MTC/As",feedback.size());
          answers.insert(answers.end(),feedback.begin(),feedback.end());
        }
                                                                                                                                                         
  #endif

        } //try
       
        catch(std::exception &e) {

         std::string msg = "Error processing configuration: ";
         msg += e.what();
         msg += ". Not committing configuration to ptbmk2.";
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
         obj["message"] = "Unknown error processing configuration. Not committing configuration to ptbmk2";
         answers.push_back(obj);
       }

    }




    void board_manager::beam_config(json& beamconfig, json& feedback) {

      size_t err_msgs = 1;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      //json trigs = beamconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
      //std::vector<uint32_t> delays = beamconfig.at("delays").get<std::vector<uint32_t>>();
      
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 <<  "Beam channel mask set for 0x0.";
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());


       //Input channel masks
       set_bit_range_register(3,0,3,channelmask);


       //Place warnings into json feedback obj
       json obj;
       for (size_t i=0; i<tmp.size(); i++) {
         if (tmp[i].second.compare("") != 0) {
           obj["type"] = tmp[i].first;
           obj["message"] = tmp[i].second;
           feedback.push_back(obj);
         }
       }


    } //Beam config



    ////////////////////////////////////////////
    // CRT configuration

    void board_manager::crt_config(json& crtconfig, json& feedback) {

      size_t err_msgs = 1;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      //json trigs = crtconfig.at("triggers"); //Array of trigger configs

      std::string s_channelmask = crtconfig.at("channel_mask").get<std::string>();
      //std::vector<uint32_t> delays = crtconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 << "CRT channel mask set for 0x0."; 
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());


      //Input channel masks
      set_bit_range_register(1,0,32,channelmask);


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
      //i2conf dacsetup;

      size_t err_msgs = 5;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      //std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
      std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
      uint32_t channelmask = (int)strtol(s_channelmask.c_str(),NULL,0);
  

      //Input channel masks
      set_bit_range_register(2,0,24,channelmask);


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

      size_t err_msgs = 1;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);
     
      std::string s_tap = miscconfig.at("tap").get<std::string>();
      uint32_t tap_val = (int)strtol(s_tap.c_str(),NULL,0);
      
      //uint32_t tap_val = miscconfig.at("tap");
      if (tap_val >= 31) {
        std::ostringstream oss0;
        oss0 << "Tap value ( " << tap_val << " is larger than max value (31), truncating value.";
        tmp[0] = std::make_pair("warning", oss0.str());
        Log(warning, "%s", oss0.str());
        tap_val = 31; 
      }



      set_bit_range_register(10,0,5,tap_val);

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
    // MTC/A configuration

    void board_manager::mtca_config(json& mtcaconfig, json& feedback) {

      size_t err_msgs = 1;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      std::string s_channelmask = mtcaconfig.at("channel_mask").get<std::string>();
      //std::vector<uint32_t> delays = mtcaconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 << "CRT channel mask set for 0x0."; 
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());

      //Input channel masks
      set_bit_range_register(9,0,6,channelmask);


      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].second.compare("") != 0) {
          obj["type"] = tmp[i].first;
          obj["message"] = tmp[i].second;
          feedback.push_back(obj);
        }
      }

    } //MTC/A config



    ////////////////////////////////////////////
    // NIM configuration

    void board_manager::nim_config(json& nimconfig, json& feedback) {

      size_t err_msgs = 1;
      std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      std::string s_channelmask = nimconfig.at("channel_mask").get<std::string>();
      //std::vector<uint32_t> delays = nimconfig.at("delays").get<std::vector<uint32_t>>();
      uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      std::ostringstream oss0;
      if (channelmask == 0) { 
        oss0 << "CRT channel mask set for 0x0."; 
        Log(warning,"%s", oss0.str());
      }
      tmp[0] = std::make_pair("warning", oss0.str());

      //Input channel masks
      set_bit_range_register(9,0,6,channelmask);


      //Place warnings into json feedback obj
      json obj;
      for (size_t i=0; i<tmp.size(); i++) {
        if (tmp[i].second.compare("") != 0) {
          obj["type"] = tmp[i].first;
          obj["message"] = tmp[i].second;
          feedback.push_back(obj);
        }
      }

    } //NIM config


}
