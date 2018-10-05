namespace ptb {

#define NBEAM_CH 9
#define NPDS_CH 24

  void board_manager::configure_ctb(json& doc, json& answers, bool &has_error) {
    has_error = false;
    try {


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
      misc_config(miscconf, mfeedback, has_error);
      if (!mfeedback.empty()) {
        Log(debug,"Received %u messages from configuring the Misc Configs",mfeedback.size());
        answers.insert(answers.end(),mfeedback.begin(),mfeedback.end());
      }
      if (has_error) {
        Log(error,"Caught error after configuring misc. Interrupting configuration.");
        return;
      }

      json hfeedback;
      hlt_config(hltconf, hfeedback, has_error);
      if (!hfeedback.empty()) {
        Log(debug,"Received %u messages from configuring the HLTs",hfeedback.size());
        answers.insert(answers.end(),hfeedback.begin(),hfeedback.end());
      }
      if (has_error) {
        Log(error,"Caught error after configuring HLTs. Interrupting configuration.");
        return;
      }

      json bfeedback;
      beam_config(beamconf, bfeedback, has_error);
      if (!bfeedback.empty()) {
        Log(debug,"Received %u messages from configuring the Beam",bfeedback.size());
        answers.insert(answers.end(),bfeedback.begin(),bfeedback.end());
      }
      if (has_error) {
        Log(error,"Caught error after configuring the beam inputs. Interrupting configuration.");
        return;
      }

      json cfeedback;
      crt_config(crtconf, cfeedback, has_error);
      if (!cfeedback.empty()) {
        Log(debug,"Received %u messages from configuring the CRT",cfeedback.size());
        answers.insert(answers.end(),cfeedback.begin(),cfeedback.end());
      }
      if (has_error) {
        Log(error,"Caught error after configuring the CRT inputs. Interrupting configuration.");
        return;
      }


#ifdef NO_PDS_DAC
      Log(warning,"PDS configuration block was disabled. Not configuring any PDS input");
#else

      json feedback;
      pds_config(pdsconf,feedback, has_error);
      if (!feedback.empty()) {
        Log(info,"Received %u feedback messages from configuring the PDS",feedback.size());
        answers.insert(answers.end(),feedback.begin(),feedback.end());
      }
      if (has_error) {
        Log(error,"Caught error after configuring the PDS inputs. Interrupting configuration.");
        return;
      }


#endif
    } //try

    // -- NFB --
    // the order of declaration of the catch blocks is important
    catch (json::exception& e) {
      std::string msg = "Error processing configuration: ";
      msg += e.what();
      msg += ". Not committing configuration to CTB.";
      json obj;
      obj["type"] = "error";
      obj["message"] = msg;
      answers.push_back(obj);
      has_error = true;
    }
    catch(std::exception &e) {
      std::string msg = "Error processing configuration: ";
      msg += e.what();
      msg += ". Not committing configuration to CTB.";
      json obj;
      obj["type"] = "error";
      obj["message"] = msg;
      answers.push_back(obj);
      has_error = true;
    }
    catch(...) {
      json obj;
      obj["type"] = "error";
      obj["message"] = "Unknown error processing configuration. Not committing configuration to CTB";
      answers.push_back(obj);
      has_error = true;
    }

  }

  void board_manager::beam_config(json& beamconfig, json& feedback, bool &has_error) {

    // This is a static map that tells


    // -- This should not be done this way

    //size_t err_msgs = 9;
    //std::vector< std::pair< std::string, std::string > > tmp;//(err_msgs);

    json trigs = beamconfig.at("triggers"); //Array of trigger configs

    std::string s_channelmask = beamconfig.at("channel_mask").get<std::string>();
    std::vector<uint32_t> delays = beamconfig.at("delays").get<std::vector<uint32_t>>();
    uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);

    uint32_t reshape_len = beamconfig.at("reshape_length").get<unsigned int>();
    Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
    if (reshape_len >= (1<<6)) {
      std::ostringstream msg;
      msg << "Reshaping legth value of " << reshape_len << " above maximum allwed [2^6 - 1]. Truncating to maximum.";
      json fobj;
      fobj["type"] = "warning";
      fobj["message"] = msg.str();
      feedback.push_back(fobj);

      Log(warning,"%s", msg.str().c_str());
      reshape_len = (1<<6)-1;
    }

    if (channelmask == 0) {
      std::ostringstream oss;
      oss <<  "Beam channel mask set for 0x0.";
      Log(warning,"%s", oss.str());
      json fobj;
      fobj["type"] = "warning";
      fobj["message"] = oss.str();
      feedback.push_back(fobj);
    }


    //Input channel masks
    set_bit_range_register(3,0,9,channelmask);
    set_bit_range_register(24,0,6,reshape_len);


    // -- Now load the beam LLT configuration
    bool llt_en = false;
    std::string llt_id;
    uint32_t llt_mask = 0x0;

    // NFB: This is done one by one, since the registers are manually allocated
    // see https://docs.google.com/spreadsheets/d/156ZUa09TceKZn4-OJEnJtfSZLkQl1tAYATt9xLz5o20/edit?usp=sharing
    // for details about the individual register allocation

    // -- trigs contains a json object htat is a vector.
    // there is no easy way to find something that
    // -- LLT_1 : beam mask

    for (json::iterator it = trigs.begin(); it != trigs.end(); ++it) {
      llt_en = it->at("enable").get<bool>();
      llt_mask = (uint32_t)strtoul(it->at("mask").get<std::string>().c_str(),NULL,0);
      llt_id = it->at("id").get<std::string>();
      Log(debug,"Setting %s to state %u mask [%X]",llt_id.c_str(),llt_en,llt_mask);

      // now make the assignment to the correct registers
      if (llt_id == "LLT_1") {
        set_bit(27,1,llt_en);
        set_bit_range_register(28,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 28 : [0x%08X]", register_map_[28].value() );
      } else if (llt_id == "LLT_2") {
        set_bit(27,2,llt_en);
        set_bit_range_register(29,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 29 : [0x%08X]", register_map_[29].value() );
      } else if (llt_id == "LLT_3") {
        set_bit(27,3,llt_en);
        set_bit_range_register(30,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 30 : [0x%08X]", register_map_[30].value() );
      } else if (llt_id == "LLT_4") {
        set_bit(27,4,llt_en);
        set_bit_range_register(31,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 31 : [0x%08X]", register_map_[31].value() );
      } else if (llt_id == "LLT_5") {
        set_bit(27,5,llt_en);
        set_bit_range_register(32,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 32 : [0x%08X]", register_map_[32].value() );
      } else if (llt_id == "LLT_6") {
        set_bit(27,6,llt_en);
        set_bit_range_register(33,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 33 : [0x%08X]", register_map_[33].value() );
      } else if (llt_id == "LLT_7") {
        set_bit(27,7,llt_en);
        set_bit_range_register(34,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 34 : [0x%08X]", register_map_[34].value() );
      } else if (llt_id == "LLT_8") {
        set_bit(27,8,llt_en);
        set_bit_range_register(35,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 35 : [0x%08X]", register_map_[35].value() );
      } else if (llt_id == "LLT_9") {
        set_bit(27,9,llt_en);
        set_bit_range_register(36,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 36 : [0x%08X]", register_map_[36].value() );
      } else if (llt_id == "LLT_10") {
        set_bit(27,10,llt_en);
        set_bit_range_register(37,0,32,llt_mask);
        Log(debug,"Register 27 : [0x%08X]", register_map_[27].value() );
        Log(debug,"Register 37 : [0x%08X]", register_map_[37].value() );
      } else if (llt_id == "LLT_25") {
        set_bit(72,6,llt_en);
        set_bit_range_register(85,0,32,llt_mask);
        Log(debug,"Register 72 : [0x%08X]", register_map_[72].value() );
        Log(debug,"Register 85 : [0x%08X]", register_map_[85].value() );
      } else if (llt_id == "LLT_26") {
        set_bit(72,7,llt_en);
        set_bit_range_register(86,0,32,llt_mask);
        Log(debug,"Register 72 : [0x%08X]", register_map_[72].value() );
        Log(debug,"Register 86 : [0x%08X]", register_map_[86].value() );
      } else {
        std::ostringstream msg;
        msg << "Unrecognized LLT with ID [" << llt_id << "] as a beam LLT";
        Log(warning,"%s",msg.str().c_str());
        json obj;
        obj["type"]="warning";
        obj["message"] = msg.str();

        feedback.push_back(obj);
      }
    }



    //    llt_en = trigs.at(0).at("enable").get<bool>();
    //
    //
    //    // -- all this could be done with a simple array
    //    std::vector<bool> llt_en;
    //    std::vector<uint32_t> llt_id;
    //    std::vector<uint32_t> llt_mask;
    //
    //
    //    for (size_t i = 0; i < trigs.size(); i++) {
    //       bool en = trigs.at(i).at("enable").get<bool>();
    //       uint32_t mask = (uint32_t)strtoul(trigs.at(i).at("mask").get<std::string>().c_str(),NULL,0);
    //       uint32_t tid;
    //       fscanf(trigs.at(i).at("id").get<std::string>().c_str(),"LLT_%u",&tid);
    //
    //       if (mask == 0x0) {
    //         std::ostringstream msg;
    //         msg << "Beam Trigger " << trigs.at(i).at("id").get<std::string>() << " has mask set to 0x0. Equivalent to disabled.";
    //         Log(warning,"%s", msg.str());
    //         json fobj;
    //         fobj["type"] = "warning";
    //         fobj["message"] = msg.str();
    //         feedback.push_back(fobj);
    //
    //       }
    //    }
    //
    //
    //
    //
    //    //    //Trigger enables
    //    set_bit(27,1,llt1_enable);
    //    set_bit(27,2,llt2_enable);
    //    set_bit(27,3,llt3_enable);
    //    set_bit(27,4,llt4_enable);
    //    set_bit(27,5,llt5_enable);
    //    set_bit(27,6,llt6_enable);
    //    set_bit(27,7,llt7_enable);
    //    set_bit(27,8,llt8_enable);
    //
    //    //Trigger parameters
    //    set_bit_range_register(28,0,32,llt1mask);
    //    set_bit_range_register(29,0,32,llt1mask);
    //    set_bit_range_register(30,0,32,llt3mask);
    //    set_bit_range_register(31,0,32,llt4mask);
    //    set_bit_range_register(32,0,32,llt5mask);
    //    set_bit_range_register(33,0,32,llt6mask);
    //    set_bit_range_register(34,0,32,llt7mask);
    //    set_bit_range_register(35,0,32,llt8mask);
    //
    uint8_t dbits = 7;
    // Beam delays 9ch * 7b = 63b --> 3 regs
    for(size_t i=0; i<(size_t)(delays.size()/4); i++) {

      int j = i * 4;
      uint32_t reg = i + 21;
      uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

      set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
    }

    set_bit_range_register(23,0,(uint32_t)dbits,delays[delays.size()-1]);

    //    //Place warnings into json feedback obj
    //    json obj;
    //    for (size_t i=0; i<tmp.size(); i++) {
    //      if (tmp[i].second.compare("") != 0) {
    //        obj["type"] = tmp[i].first;
    //        obj["message"] = tmp[i].second;
    //        feedback.push_back(obj);
    //      }
    //    }

    //Log(info,"Programmed [%d] (%u) [0x%X][%s]",rtriggerperiod,rtriggerperiod,rtriggerperiod, std::bitset<26>(rtriggerperiod).to_string().c_str());

  } //Beam config

  ////////////////////////////////////////////
  // CRT configuration

  void board_manager::crt_config(json& crtconfig, json& feedback, bool &has_error) {


    //    size_t err_msgs = 32;
    //    std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

    json trigs = crtconfig.at("triggers"); //Array of trigger configs

    std::string s_channelmask = crtconfig.at("channel_mask").get<std::string>();
    std::vector<uint32_t> delays = crtconfig.at("delays").get<std::vector<uint32_t>>();
    uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
    bool crt_mapping_ena = crtconfig.at("pixelate").get<bool>();
    //std::ostringstream oss0;
    if (channelmask == 0x0) {
      std::ostringstream msg;
      msg << "CRT channel mask set for 0x0.";
      Log(warning,"%s", msg.str().c_str());
      json tobj;
      tobj["type"] = "warning";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
    }
    //    tmp[0] = std::make_pair("warning", oss0.str());

    uint32_t reshape_len = crtconfig.at("reshape_length").get<unsigned int>();
    Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
    if (reshape_len >= (1<<6)) {
      std::ostringstream msg;
      msg << "CRT reshape length of " << reshape_len << " above maximum [2^6 - 1]. Truncating to maximum.";
      Log(warning,"%s", msg.str().c_str());
      json tobj;
      tobj["type"] = "warning";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      //      tmp[31] = std::make_pair("warning", oss31.str());
      //      Log(warning,"%s", oss31.str());
      reshape_len = (1<<6)-1;
    }

    //Input channel masks
    set_bit_range_register(1,0,32,channelmask);
    set_bit(62,1,crt_mapping_ena);
    set_bit_range_register(25,0,6,reshape_len);


    // -- Process the triggers
    // -- All CRT LLTs are of type counter
    // WTF?

    // temp vars
    bool llt_en = false;
    uint32_t llt_mask = 0x0;
    uint32_t llt_count;
    uint32_t llt_comp;
    std::string llt_id;
    uint32_t llt_reg;

    for (json::iterator it = trigs.begin(); it != trigs.end(); ++it) {
      llt_en = it->at("enable").get<bool>();
      llt_mask = (uint32_t)strtoul(it->at("mask").get<std::string>().c_str(),NULL,0);
      llt_id = it->at("id").get<std::string>();
      llt_count = (uint32_t)strtoul(it->at("count").get<std::string>().c_str(),NULL,0);
      llt_comp = (uint32_t)strtoul(it->at("type").get<std::string>().c_str(),NULL,0);

      Log(debug,
          "Setting [%s] to state %u mask [%X] count [%u] comp [%c]",
          llt_id.c_str(),
          llt_en,
          llt_mask,
          llt_count,
          util::llt_cop_tochar(llt_comp));

      // -- Check validity of inputs
      // if it is anything but 0x1,0x2,0x4,
      if ((llt_comp == 0x0) || (llt_comp > 0x4)|| (llt_comp == 0x3)) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has an invalid comparison operation [" << std::hex <<
            llt_comp << std::dec << "]";
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        // Flag the error and return
        has_error = true;
        return;
      }
      // -- If count is 0 and operation is '=' we are basically disabling it
      if ((llt_comp == 0x2) && (llt_count == 0x0)) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has a condition of 'counts==0'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is 0 and operation is '<' we are basically disabling it
      if ((llt_comp == 0x4) && (llt_count == 0x1)) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has a condition of 'counts<1'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is 1 and operation is '<' we are basically disabling it
      if ((llt_comp == 0x4) && (llt_count == 0x0)) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has a condition of 'counts<0'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- if the count is above the allowed in the mask, we are also disabling
      // Keep in mind that by now we already checked for invalid comparison values.
      // if comparison is == or > , and count > n_bits(mask), then we are disabling it
      std::bitset<32> maskbits(llt_mask);
      // NFB: There used to be a bug here.
      if ((llt_comp != 0x4) && (llt_count > maskbits.count())) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has a condition of '> or =' and 'counts>number of asserted channels' (counts"
            << llt_count << util::llt_cop_tochar(llt_comp) << maskbits.count() << "). This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is outside of the range allowed by 6 bits we will have an issue as well
      if (llt_count > ((0x1<<5)-1)) {
        std::ostringstream msg;
        msg << "CRT " << llt_id << " has a condition with counts above number allowed by available bits ("
            << llt_count << " vs " << ((0x1<<6)-1) << "). Aborting configuration.";
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error=true;
        return;
      }


      llt_reg = ((llt_comp & 0x7) << 5) | llt_count;
      // -- Now start the register assignments
      if (llt_id == "LLT_11") {
        set_bit(27,11,llt_en);
        set_bit_range_register(38,0,8,llt_reg);
        set_bit_range_register(39,0,32,llt_mask);
      } else if (llt_id == "LLT_12") {
        set_bit(27,12,llt_en);
        set_bit_range_register(40,0,8,llt_reg);
        set_bit_range_register(41,0,32,llt_mask);
      } else if (llt_id == "LLT_13") {
        set_bit(27,13,llt_en);
        set_bit_range_register(42,0,8,llt_reg);
        set_bit_range_register(43,0,32,llt_mask);
      } else if (llt_id == "LLT_15") {
        set_bit(27,15,llt_en);
        set_bit_range_register(46,0,8,llt_reg);
        set_bit_range_register(47,0,32,llt_mask);
      } else if (llt_id == "LLT_16") {
        set_bit(27,16,llt_en);
        set_bit_range_register(48,0,8,llt_reg);
        set_bit_range_register(49,0,32,llt_mask);
      } else if (llt_id == "LLT_18") {
        set_bit(27,18,llt_en);
        set_bit_range_register(52,0,8,llt_reg);
        set_bit_range_register(53,0,32,llt_mask);
      } else if (llt_id == "LLT_19") {
        set_bit(72,0,llt_en);
        set_bit_range_register(73,0,8,llt_reg);
        set_bit_range_register(74,0,32,llt_mask);
      } else if (llt_id == "LLT_20") {
        set_bit(72,1,llt_en);
        set_bit_range_register(75,0,8,llt_reg);
        set_bit_range_register(76,0,32,llt_mask);
      } else if (llt_id == "LLT_21") {
        set_bit(72,2,llt_en);
        set_bit_range_register(77,0,8,llt_reg);
        set_bit_range_register(78,0,32,llt_mask);
      } else {
        std::ostringstream msg;
        msg << "Unrecognized LLT with ID [" << llt_id << "] as a CRT LLT";
        Log(warning,"%s",msg.str().c_str());
        json obj;
        obj["type"]="warning";
        obj["message"] = msg.str();

        feedback.push_back(obj);
      }
    }
    //    //Counting triggers
    //    bool llt11_enable = trigs.at(0).at("enable").get<bool>();
    //    std::string s_llt11mask = trigs.at(0).at("mask").get<std::string>();
    //    std::string s_llt11count = trigs.at(0).at("count").get<std::string>();
    //    std::string s_llt11type = trigs.at(0).at("type").get<std::string>();
    //    uint32_t llt11mask = (uint32_t)strtoul(s_llt11mask.c_str(),NULL,0);
    //    uint8_t llt11count = (int)strtol(s_llt11count.c_str(),NULL,0);
    //    uint8_t llt11type = (int)strtol(s_llt11type.c_str(),NULL,0);
    //
    //    std::ostringstream oss1;
    //    if (llt11type > 4 || llt11type == 3 || llt11type == 0) {
    //      oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " type (" << (int)llt11type << ") logic type undefined! ";
    //      Log(error,"%s", oss1.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[1] = std::make_pair("warning", oss1.str());
    //    std::ostringstream oss2;
    //    if ((llt11count == 0 && llt11type == 2)) {
    //      oss2 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss2.str());
    //    }
    //    tmp[2] = std::make_pair("warning", oss2.str());
    //    std::ostringstream oss3;
    //    if ((llt11count == 0 && llt11type == 4)) {
    //      oss3 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss3.str());
    //    }
    //    tmp[3] = std::make_pair("warning", oss3.str());
    //    std::ostringstream oss4;
    //    if ((llt11count == 1 && llt11type == 4)) {
    //      oss4 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss4.str());
    //    }
    //    tmp[4] = std::make_pair("warning", oss4.str());
    //    std::ostringstream oss5;
    //    if ((llt11count > llt11mask) && (llt11type == 2 || llt11type == 1)) {
    //      oss5 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") amd type (" << (int)llt11type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss5.str());
    //    }
    //    tmp[5] = std::make_pair("warning", oss5.str());
    //    std::ostringstream oss6;
    //    if (llt11count > std::pow(2,5)-1) {
    //      oss6 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt11count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss6.str());
    //    }
    //    tmp[6] = std::make_pair("warning", oss6.str());
    //
    //
    //    bool llt12_enable = trigs.at(1).at("enable").get<bool>();
    //    std::string s_llt12mask = trigs.at(1).at("mask").get<std::string>();
    //    std::string s_llt12count = trigs.at(1).at("count").get<std::string>();
    //    std::string s_llt12type = trigs.at(1).at("type").get<std::string>();
    //    uint32_t llt12mask = (uint32_t)strtoul(s_llt12mask.c_str(),NULL,0);
    //    uint8_t llt12count = (int)strtol(s_llt12count.c_str(),NULL,0);
    //    uint8_t llt12type = (int)strtol(s_llt12type.c_str(),NULL,0);
    //
    //    std::ostringstream oss7;
    //    if (llt12type > 4 || llt12type == 3 || llt12type == 0) {
    //      oss7 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " type (" << (int)llt12type << ") logic type undefined! ";
    //      Log(error,"%s", oss7.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[7] = std::make_pair("warning", oss7.str());
    //    std::ostringstream oss8;
    //    if ((llt12count == 0 && llt12type == 2)) {
    //      oss8 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss8.str());
    //    }
    //    tmp[8] = std::make_pair("warning", oss8.str());
    //    std::ostringstream oss9;
    //    if ((llt12count == 0 && llt12type == 4)) {
    //      oss9 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss9.str());
    //    }
    //    tmp[9] = std::make_pair("warning", oss9.str());
    //    std::ostringstream oss10;
    //    if ((llt12count == 1 && llt12type == 4)) {
    //      oss10 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss10.str());
    //    }
    //    tmp[10] = std::make_pair("warning", oss10.str());
    //    std::ostringstream oss11;
    //    if ((llt12count > llt12mask) && (llt12type == 2 || llt12type == 1)) {
    //      oss11 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") amd type (" << (int)llt12type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss11.str());
    //    }
    //    tmp[11] = std::make_pair("warning", oss11.str());
    //    std::ostringstream oss12;
    //    if (llt12count > std::pow(2,5)-1) {
    //      oss12 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt12count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss12.str());
    //    }
    //    tmp[12] = std::make_pair("warning", oss12.str());
    //
    //    bool llt13_enable = trigs.at(1).at("enable").get<bool>();
    //    std::string s_llt13mask = trigs.at(2).at("mask").get<std::string>();
    //    std::string s_llt13count = trigs.at(2).at("count").get<std::string>();
    //    std::string s_llt13type = trigs.at(2).at("type").get<std::string>();
    //    uint32_t llt13mask = (uint32_t)strtoul(s_llt13mask.c_str(),NULL,0);
    //    uint8_t llt13count = (int)strtol(s_llt13count.c_str(),NULL,0);
    //    uint8_t llt13type = (int)strtol(s_llt13type.c_str(),NULL,0);
    //
    //    std::ostringstream oss13;
    //    if (llt13type > 4 || llt13type == 3 || llt13type == 0) {
    //      oss13 << "Trigger " << trigs.at(2).at("id").get<std::string>() << " type (" << (int)llt13type << ") logic type undefined! ";
    //      Log(error,"%s", oss13.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[13] = std::make_pair("warning", oss13.str());
    //    std::ostringstream oss14;
    //    if ((llt13count == 0 && llt13type == 2)) {
    //      oss14 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss14.str());
    //    }
    //    tmp[14] = std::make_pair("warning", oss14.str());
    //    std::ostringstream oss15;
    //    if ((llt13count == 0 && llt13type == 4)) {
    //      oss15 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss15.str());
    //    }
    //    tmp[15] = std::make_pair("warning", oss15.str());
    //    std::ostringstream oss16;
    //    if ((llt13count == 1 && llt13type == 4)) {
    //      oss16 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss16.str());
    //    }
    //    tmp[16] = std::make_pair("warning", oss16.str());
    //    std::ostringstream oss17;
    //    if ((llt13count > llt13mask) && (llt13type == 2 || llt13type == 1)) {
    //      oss17 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") amd type (" << (int)llt13type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss17.str());
    //    }
    //    tmp[17] = std::make_pair("warning", oss17.str());
    //    std::ostringstream oss18;
    //    if (llt13count > std::pow(2,5)-1) {
    //      oss18 << "Trigger " << trigs.at(2).at("id").get<std::string>() << ", with count (" << (int)llt13count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss18.str());
    //    }
    //    tmp[18] = std::make_pair("warning", oss18.str());
    //
    //    bool llt15_enable = trigs.at(3).at("enable").get<bool>();
    //    std::string s_llt15mask = trigs.at(3).at("mask").get<std::string>();
    //    std::string s_llt15count = trigs.at(3).at("count").get<std::string>();
    //    std::string s_llt15type = trigs.at(3).at("type").get<std::string>();
    //    uint32_t llt15mask = (uint32_t)strtoul(s_llt15mask.c_str(),NULL,0);
    //    uint8_t llt15count = (int)strtol(s_llt15count.c_str(),NULL,0);
    //    uint8_t llt15type = (int)strtol(s_llt15type.c_str(),NULL,0);
    //
    //    std::ostringstream oss19;
    //    if (llt15type > 4 || llt15type == 3 || llt15type == 0) {
    //      oss19 << "Trigger " << trigs.at(3).at("id").get<std::string>() << " type (" << (int)llt15type << ") logic type undefined! ";
    //      Log(error,"%s", oss19.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[19] = std::make_pair("warning", oss19.str());
    //    std::ostringstream oss20;
    //    if ((llt15count == 0 && llt15type == 2)) {
    //      oss20 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss20.str());
    //    }
    //    tmp[20] = std::make_pair("warning", oss20.str());
    //    std::ostringstream oss21;
    //    if ((llt15count == 0 && llt15type == 4)) {
    //      oss21 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss21.str());
    //    }
    //    tmp[21] = std::make_pair("warning", oss21.str());
    //    std::ostringstream oss22;
    //    if ((llt15count == 1 && llt15type == 4)) {
    //      oss22 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss22.str());
    //    }
    //    tmp[22] = std::make_pair("warning", oss22.str());
    //    std::ostringstream oss23;
    //    if ((llt15count > llt15mask) && (llt15type == 2 || llt15type == 1)) {
    //      oss23 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss23.str());
    //    }
    //    tmp[23] = std::make_pair("warning", oss23.str());
    //    std::ostringstream oss24;
    //    if (llt15count > std::pow(2,5)-1) {
    //      oss24 << "Trigger " << trigs.at(3).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss24.str());
    //    }
    //    tmp[24] = std::make_pair("warning", oss24.str());
    //
    //    bool llt16_enable = trigs.at(4).at("enable").get<bool>();
    //    std::string s_llt16mask = trigs.at(4).at("mask").get<std::string>();
    //    std::string s_llt16count = trigs.at(4).at("count").get<std::string>();
    //    std::string s_llt16type = trigs.at(4).at("type").get<std::string>();
    //    uint32_t llt16mask = (uint32_t)strtoul(s_llt16mask.c_str(),NULL,0);
    //    uint8_t llt16count = (int)strtol(s_llt16count.c_str(),NULL,0);
    //    uint8_t llt16type = (int)strtol(s_llt16type.c_str(),NULL,0);
    //
    //    std::ostringstream oss25;
    //    if (llt16type > 4 || llt16type == 3 || llt16type == 0) {
    //      oss25 << "Trigger " << trigs.at(4).at("id").get<std::string>() << " type (" << (int)llt16type << ") logic type undefined! ";
    //      Log(error,"%s", oss25.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[25] = std::make_pair("warning", oss25.str());
    //    std::ostringstream oss26;
    //    if ((llt16count == 0 && llt16type == 2)) {
    //      oss26 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss26.str());
    //    }
    //    tmp[26] = std::make_pair("warning", oss26.str());
    //    std::ostringstream oss27;
    //    if ((llt16count == 0 && llt16type == 4)) {
    //      oss27 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss27.str());
    //    }
    //    tmp[27] = std::make_pair("warning", oss27.str());
    //    std::ostringstream oss28;
    //    if ((llt16count == 1 && llt16type == 4)) {
    //      oss28 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt16count << ") amd type (" << (int)llt16type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss28.str());
    //    }
    //    tmp[28] = std::make_pair("warning", oss28.str());
    //    std::ostringstream oss29;
    //    if ((llt16count > llt15mask) && (llt15type == 2 || llt15type == 1)) {
    //      oss29 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") amd type (" << (int)llt15type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss29.str());
    //    }
    //    tmp[29] = std::make_pair("warning", oss29.str());
    //    std::ostringstream oss30;
    //    if (llt15count > std::pow(2,5)-1) {
    //      oss30 << "Trigger " << trigs.at(4).at("id").get<std::string>() << ", with count (" << (int)llt15count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss30.str());
    //    }
    //    tmp[30] = std::make_pair("warning", oss30.str());
    //
    //
    //    //Trigger enables
    //    set_bit(27,11,llt11_enable);
    //    set_bit(27,12,llt12_enable);
    //    set_bit(27,13,llt13_enable);
    //    set_bit(27,15,llt15_enable);
    //    set_bit(27,16,llt16_enable);
    //
    //    //Trigger parameters
    //    uint32_t llt11 = (llt11type<<5) + llt11count;
    //    uint32_t llt12 = (llt12type<<5) + llt12count;
    //    uint32_t llt13 = (llt13type<<5) + llt13count;
    //    uint32_t llt15 = (llt15type<<5) + llt15count;
    //    uint32_t llt16 = (llt16type<<5) + llt16count;
    //
    //    set_bit_range_register(38,0,9,llt11);
    //    set_bit_range_register(39,0,32,llt11mask);
    //    set_bit_range_register(40,0,9,llt12);
    //    set_bit_range_register(41,0,32,llt12mask);
    //    set_bit_range_register(42,0,9,llt13);
    //    set_bit_range_register(43,0,32,llt13mask);
    //    set_bit_range_register(46,0,9,llt15);
    //    set_bit_range_register(47,0,32,llt15mask);
    //    set_bit_range_register(48,0,9,llt16);
    //    set_bit_range_register(49,0,32,llt16mask);

    uint8_t dbits = 7;
    // PDS delays 32ch * 7b = 224b --> 8 regs
    for(size_t i=0; i<delays.size()/4; i++) {

      int j = i * 4;
      uint32_t reg = i + 13;
      uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

      set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
    }

    //    //Place warnings into json feedback obj
    //    json obj;
    //    for (size_t i=0; i<tmp.size(); i++) {
    //      if (tmp[i].second.compare("") != 0) {
    //        obj["type"] = tmp[i].first;
    //        obj["message"] = tmp[i].second;
    //        feedback.push_back(obj);
    //      }
    //    }

  } //CRT config

  ////////////////////////////////////////////
  // PDS configuration

  void board_manager::pds_config(json& pdsconfig, json& feedback, bool &has_error) {
    ///FIXME: How can this work? dacsetup is not initialized in this function
    i2conf dacsetup;

    size_t err_msgs = 14;
    std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

    json trigs = pdsconfig.at("triggers"); //Array of trigger configs

    std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
    std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
    uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
    std::vector<uint32_t> delays = pdsconfig.at("delays").get<std::vector<uint32_t>>();



    uint32_t reshape_len = pdsconfig.at("reshape_length").get<unsigned int>();
    Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
    if (reshape_len > (1<<6)-1) {
      std::ostringstream msg;
      msg << "Random trigger value of " << reshape_len << " above maximum allowed [2^6 - 1]. Truncating to maximum.";
      Log(warning,"%s",msg.str().c_str());
      json tobj;
      tobj["type"] = "warning";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      reshape_len = (1<<6)-1;
    }

    // -- assign the registers
    //Input channel masks
    set_bit_range_register(2,0,24,channelmask);

    set_bit_range_register(26,0,6,reshape_len);


    // -- Delays

    if (delays.size() != NPDS_CH) {
      std::ostringstream msg;
      msg << "Number of PDS delays (" << delays.size() << ") doesn't match number of PDS channels (" << NPDS_CH << ")";
      Log(error,"%s", msg.str().c_str());
      json tobj;
      tobj["type"] = "error";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      has_error = true;
      return;
    }

    for (size_t i=0; i<delays.size(); i++) {
      if (delays.at(i) > ((0x1<<7)-1)) { //Range 0 - 2^7 -1
        std::ostringstream msg;
        msg << "PDS delay value on entry " << i << " out of range (" << delays.at(i) << "). Truncating to maximum (127)";
        Log(warning,"%s", msg.str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
    }

    // -- assign the delays
    uint8_t dbits = 7;
    // PDS delays 24ch * 7b = 168b --> 6 regs
    for(size_t i=0; i<delays.size()/4; i++) {

      int j = i * 4;
      uint32_t reg = i + 7;
      uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];

      set_bit_range_register(reg,0,(uint32_t)dbits*4,dval);
    }


    // -- Process the triggers
    // -- All CRT LLTs are of type counter
    bool llt_en;
    uint32_t llt_mask;
    uint32_t llt_comp;
    uint32_t llt_count;
    std::string llt_id;
    uint32_t llt_reg;

    for (json::iterator it = trigs.begin(); it != trigs.end(); ++it) {
      llt_en = it->at("enable").get<bool>();
      llt_mask = (uint32_t)strtoul(it->at("mask").get<std::string>().c_str(),NULL,0);
      llt_id = it->at("id").get<std::string>();
      llt_count = (uint32_t)strtoul(it->at("count").get<std::string>().c_str(),NULL,0);
      llt_comp = (uint32_t)strtoul(it->at("type").get<std::string>().c_str(),NULL,0);

      llt_reg = ((llt_comp & 0x7) << 5) | llt_count;

      // -- Now check for errors
      // NFB: THese are specific for the PDS

      // if it is anything but 0x1,0x2,0x4,
      if ((llt_comp == 0x0) || (llt_comp > 0x4)|| (llt_comp == 0x3)) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has an invalid comparison operation [" << std::hex <<
            llt_comp << std::dec << "]";
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        // Flag the error and return
        has_error = true;
        return;
      }

      // -- If count is 0 and operation is '=' we are basically disabling it
      if ((llt_comp == 0x2) && (llt_count == 0x0)) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has a condition of 'counts==0'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is 0 and operation is '<' we are basically disabling it
      if ((llt_comp == 0x4) && (llt_count == 0x1)) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has a condition of 'counts<1'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is 1 and operation is '<' we are basically disabling it
      if ((llt_comp == 0x4) && (llt_count == 0x0)) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has a condition of 'counts<0'. This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- if the count is above the allowed in the mask, we are also disabling
      // Keep in mind that by now we already checked for invalid comparison values.
      // if comparison is == or > , and count > n_bits(mask), then we are disabling it
      std::bitset<32> maskbits(llt_mask);
      // NFB: There used to be a bug here.
      if ((llt_comp != 0x4) && (llt_count > maskbits.count())) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has a condition of '> or =' and 'counts>number of asserted channels' (counts"
            << llt_count << util::llt_cop_tochar(llt_comp) << maskbits.count() << "). This is the same as being disabled.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      }
      // -- If count is outside of the range allowed by 6 bits we will have an issue as well
      if (llt_count > ((0x1<<5)-1)) {
        std::ostringstream msg;
        msg << "PDS " << llt_id << " has a condition with counts above number allowed by available bits ("
            << llt_count << " vs " << ((0x1<<6)-1) << "). Aborting configuration.";
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error=true;
        return;
      }


      // if it passed the checks, it is time to assign the registers
      if (llt_id == "LLT_14") {
        set_bit(27,14,llt_en);
        set_bit_range_register(44,0,8,llt_reg);
        set_bit_range_register(45,0,32,llt_mask);
      } else if (llt_id == "LLT_17") {
        set_bit(27,17,llt_en);
        set_bit_range_register(50,0,8,llt_reg);
        set_bit_range_register(51,0,32,llt_mask);
      } else if (llt_id == "LLT_22") {
        set_bit(72,3,llt_en);
        set_bit_range_register(79,0,8,llt_reg);
        set_bit_range_register(80,0,32,llt_mask);
      } else if (llt_id == "LLT_23") {
        set_bit(72,4,llt_en);
        set_bit_range_register(81,0,8,llt_reg);
        set_bit_range_register(82,0,32,llt_mask);
      } else if (llt_id == "LLT_24") {
        set_bit(72,5,llt_en);
        set_bit_range_register(83,0,8,llt_reg);
        set_bit_range_register(84,0,32,llt_mask);
      }  else {
        std::ostringstream msg;
        msg << "Unrecognized LLT with ID [" << llt_id << "] as a PDS LLT";
        Log(warning,"%s",msg.str().c_str());
        json obj;
        obj["type"]="warning";
        obj["message"] = msg.str();

        feedback.push_back(obj);
      }

    }


    //    std::string s_llt14type = trigs.at(0).at("type").get<std::string>();
    //    std::string s_llt14count = trigs.at(0).at("count").get<std::string>();
    //    std::string s_llt14mask = trigs.at(0).at("mask").get<std::string>();
    //    bool llt14_enable = trigs.at(0).at("enable").get<bool>();
    //    std::string s_llt17type = trigs.at(1).at("type").get<std::string>();
    //    std::string s_llt17count = trigs.at(1).at("count").get<std::string>();
    //    std::string s_llt17mask = trigs.at(1).at("mask").get<std::string>();
    //    bool llt17_enable = trigs.at(1).at("enable").get<bool>();
    //
    //    uint8_t llt14type = (int)strtol(s_llt14type.c_str(),NULL,0);
    //    uint8_t llt14count = (int)strtol(s_llt14count.c_str(),NULL,0);
    //    uint32_t llt14mask = (uint32_t)strtoul(s_llt14mask.c_str(),NULL,0);
    //    uint8_t llt17type = (int)strtol(s_llt17type.c_str(),NULL,0);
    //    uint8_t llt17count = (int)strtol(s_llt17count.c_str(),NULL,0);
    //    uint32_t llt17mask = (uint32_t)strtoul(s_llt17mask.c_str(),NULL,0);
    //
    //
    //    std::ostringstream oss0;
    //    if (llt14type > 4 || llt14type == 3 || llt14type == 0) {
    //      oss0 << "Trigger " << trigs.at(0).at("id").get<std::string>() << " type (" << (int)llt14type << ") logic type undefined! ";
    //      Log(error,"%s", oss0.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[0] = std::make_pair("warning", oss0.str());
    //    std::ostringstream oss1;
    //    if ((llt14count == 0 && llt14type == 2) ||
    //        (llt14count == 1 && llt14type == 4) ||
    //        (llt14count > llt14mask) && (llt14type == 2 || llt14type == 1)) {
    //      oss1 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss1.str());
    //    }
    //    tmp[1] = std::make_pair("warning", oss1.str());
    //    std::ostringstream oss2;
    //    if ((llt14count == 0 && llt14type == 4)) {
    //      oss2 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") amd type (" << (int)llt14type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss2.str());
    //    }
    //    tmp[2] = std::make_pair("warning", oss2.str());
    //    std::ostringstream oss3;
    //    if (llt14count > std::pow(2,5)-1) {
    //      oss3 << "Trigger " << trigs.at(0).at("id").get<std::string>() << ", with count (" << (int)llt14count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss3.str());
    //    }
    //    tmp[3] = std::make_pair("warning", oss3.str());
    //
    //    std::ostringstream oss4;
    //    if (llt17type > 4 || llt17type == 3 || llt17type == 0) {
    //      oss4 << "Trigger " << trigs.at(1).at("id").get<std::string>() << " type (" << (int)llt17type << ") logic type undefined! ";
    //      Log(error,"%s", oss4.str());
    //      //FIXME Add return, this should be an error
    //    }
    //    tmp[4] = std::make_pair("warning", oss4.str());
    //    std::ostringstream oss5;
    //    if ((llt17count == 0 && llt17type == 2) ||
    //        (llt17count == 1 && llt17type == 4) ||
    //        (llt17count > llt17mask) && (llt17type == 2 || llt17type == 1)) {
    //      oss5 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt17count << ") amd type (" << (int)llt17type << ") is equivalent to disabled.";
    //      Log(warning,"%s", oss5.str());
    //    }
    //    tmp[5] = std::make_pair("warning", oss5.str());
    //    std::ostringstream oss6;
    //    if ((llt17count == 0 && llt17type == 4)) {
    //      oss6 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt17count << ") amd type (" << (int)llt17type << ") is asking for negative count. Trigger disabled.";
    //      Log(warning,"%s", oss6.str());
    //    }
    //    tmp[6] = std::make_pair("warning", oss6.str());
    //    std::ostringstream oss7;
    //    if (llt17count > std::pow(2,5)-1) {
    //      oss7 << "Trigger " << trigs.at(1).at("id").get<std::string>() << ", with count (" << (int)llt17count << ") is larger than the number of available bits. Configuration aborted.";
    //      Log(error,"%s", oss7.str());
    //    }
    //    tmp[7] = std::make_pair("warning", oss7.str());
    //
    //    //Configure counting trigger(s)
    //    set_bit(27,14,llt14_enable);
    //    uint32_t llt14 = (llt14type<<5) + llt14count;
    //    set_bit_range_register(44,0,9,llt14);
    //    set_bit_range_register(45,0,32,llt14mask);
    //
    //    set_bit(27,17,llt17_enable);
    //    uint32_t llt17 = (llt17type<<5) + llt17count;
    //    set_bit_range_register(50,0,9,llt17);
    //    set_bit_range_register(51,0,32,llt17mask);
    //




    // -- Finally program the DACs


    //    std::ostringstream oss10;
    if (dac_values.size() != (i2conf::nchannels_)*(i2conf::ndacs_)) {
      std::ostringstream msg;
      msg << "Mismatch in number of PDS configuration channel thresholds  (" << dac_values.size() << ") and number of physical channels (" << (i2conf::nchannels_)*(i2conf::ndacs_) << ")";
      Log(error,"%s", msg.str().c_str());
      json tobj;
      tobj["type"] = "error";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      has_error = true;
      return;
    }
    //    tmp[10] = std::make_pair("warning", oss10.str());

    Log(info,"Number of PDS channel thresholds to be programmed: %i", dac_values.size());
    for (size_t i=0; i<dac_values.size(); i++) {
      Log(info,"Channel %zu value %u", i, dac_values.at(i));
      if (dac_values[i] > 4095) { //Range 0 - 4095
        std::ostringstream msg;
        msg << "DAC value for channel [" << i << "]out of range (" << dac_values.at(i) << "). Truncating to maximum (4095)";
        Log(warning,"%s", msg.str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);

        //        tmp[11] = std::make_pair("warning", oss11.str());

        dac_values[i] = 4095;
      }
    }

    //Now pass DAC configs to setup
    //std::ostringstream oss12;
    if (dacsetup.ConfigureDacs(dac_values,false)) {
      std::ostringstream msg;
      msg << "Failed to write SSP threshold configuration values to DACs.";
      Log(error,"%s", msg.str().c_str());
      json tobj;
      tobj["type"] = "error";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      has_error = true;
      return;
    }
    //    tmp[12] = std::make_pair("error", oss12.str());
    Log(info,"Programmed %zu DAC channels", dac_values.size());




    //    //Place warnings into json feedback obj
    //    json obj;
    //    for (size_t i=0; i<tmp.size(); i++) {
    //      if (tmp[i].second.compare("") != 0) {
    //        obj["type"] = tmp[i].first;
    //        obj["message"] = tmp[i].second;
    //        feedback.push_back(obj);
    //      }
    //    }

  } //PDS config

  ////////////////////////////////////////////
  // Misc configuration

  void board_manager::misc_config(json& miscconfig, json& feedback, bool &has_error) {




//    size_t err_msgs = 2;
//    std::vector< std::pair< std::string, std::string > > tmp(err_msgs);
//    std::ostringstream oss;

    bool ch_status_en = miscconfig.at("ch_status").get<bool>();
    bool standalone_en = miscconfig.at("standalone_enable").get<bool>();

    set_bit(62,0,ch_status_en);
    set_bit(70,0,standalone_en);


    // Make it compatible with the old configurations. If an exception is caught here, assume the
    // second is off
    bool rndm_tr_en = false;
    bool rndm_tr_ff = false;
    bool rndm_tr_bm = false;
    uint32_t rndm_tr_per;

    json rtrigger;
    bool legacy_rndm_tr = false;

    try {
      rtrigger = miscconfig.at("randomtrigger_1");
    }
    catch(json::exception &e) {
      legacy_rndm_tr = true;
      rtrigger = miscconfig.at("randomtrigger");
    }

    // -- Parse the trigger data
    rndm_tr_en = rtrigger.at("enable").get<bool>();
    rndm_tr_ff = rtrigger.at("fixed_freq").get<bool>();
    rndm_tr_bm = rtrigger.at("beam_mode").get<bool>();
    rndm_tr_per = rtrigger.at("period").get<unsigned int>();
    Log(debug,"Random Trigger Module 1 : Config  en %u fix_freq %u beam_mode %u",rndm_tr_en,rndm_tr_ff,rndm_tr_bm);
    Log(debug,"Random Trigger Module 1 : Period [%d] (%u) [0x%X][%s]",rndm_tr_per,rndm_tr_per,rndm_tr_per, std::bitset<26>(rndm_tr_per).to_string().c_str());

    // if (rndm_tr_per > ((0x1<<32)-1)) {
    //   std::ostringstream msg;
    //   msg << "Random trigger (1) period of " << rndm_tr_per << " above max allowed [2^32-1]. Truncating to max.";
    //   Log(warning,"%s",msg.str().c_str());
    //   json tobj;
    //   tobj["type"] = "warning";
    //   tobj["message"] = msg.str();
    //   feedback.push_back(tobj);
    //   rndm_tr_per = 0xFFFFFFFF;
    // }

    set_bit(27,0,rndm_tr_en);
    set_bit(62,2,rndm_tr_ff);
    set_bit(70,1,rndm_tr_bm);
    set_bit_range_register(4,0,32,rndm_tr_per);

    // -- Now repeat for the second module
    // only if the configuration is not in legacy mode
    if (!legacy_rndm_tr) {
      rtrigger = miscconfig.at("randomtrigger_2");
      rndm_tr_en = rtrigger.at("enable").get<bool>();
      rndm_tr_ff = rtrigger.at("fixed_freq").get<bool>();
      rndm_tr_bm = rtrigger.at("beam_mode").get<bool>();
      rndm_tr_per = rtrigger.at("period").get<unsigned int>();

      Log(debug,"Random Trigger Module 2 : Config  en %u fix_freq %u beam_mode %u",rndm_tr_en,rndm_tr_ff,rndm_tr_bm);
      Log(debug,"Random Trigger Module 2 : Period [%d] (%u) [0x%X][%s]",rndm_tr_per,rndm_tr_per,rndm_tr_per, std::bitset<26>(rndm_tr_per).to_string().c_str());

      // if (rndm_tr_per > ((0x1<<32)-1)) {
      //   std::ostringstream msg;
      //   msg << "Random trigger (2) period of " << rndm_tr_per << " above max allowed [2^32-1]. Truncating to max.";
      //   Log(warning,"%s",msg.str().c_str());
      //   json tobj;
      //   tobj["type"] = "warning";
      //   tobj["message"] = msg.str();
      //   feedback.push_back(tobj);
      //   rndm_tr_per = 0xFFFFFFFF;
      // }

      set_bit(71,31,rndm_tr_en);
      set_bit(71,30,rndm_tr_ff);
      set_bit(71,29,rndm_tr_bm);
      set_bit_range_register(72,0,32,rndm_tr_per);


    }


//      bool rtrigger_en = rtrigger.at("enable").get<bool>();
//    bool rtrig_fixed_freq = rtrigger.at("fixed_freq").get<bool>();
//    uint32_t rtriggerperiod = rtrigger.at("period").get<unsigned int>();
//    bool beam_mode = rtrigger.at("beam_mode").get<bool>();
//    Log(debug,"Random Trigger Frequency [%d] (%u) [0x%X][%s]",rtriggerperiod,rtriggerperiod,rtriggerperiod, std::bitset<26>(rtriggerperiod).to_string().c_str());
//    if (rtriggerperiod >= (1<<26)) {
//      //std::ostringstream oss0;
//      oss << "Random trigger value of " << rtriggerperiod << " above maximum rollover [2^26 - 1]. Truncating to maximum.";
//      tmp[0] = std::make_pair("warning", oss.str());
//      Log(warning,"%s", oss.str());
//      rtriggerperiod = (1<<26)-1;
//    }


    //
    // Configure the pulser
    //



    //Set pulser frequency
    json pulserconf = miscconfig.at("pulser");
    bool pulser_en = pulserconf.at("enable").get<bool>();
    uint32_t pulserfreq = pulserconf.at("frequency").get<unsigned int>();
    Log(debug,"Pulser State %u Frequency [%d] (%u) [0x%X][%s]",pulser_en,pulserfreq,pulserfreq,pulserfreq, std::bitset<26>(pulserfreq).to_string().c_str());
    if (pulserfreq > ((1<<26)-1)) {
      std::ostringstream msg;
      msg << "Pulser value of " << pulserfreq << " above maximum [2^26 - 1]. Truncating to maximum.";
      Log(warning,"%s",msg.str().c_str());
      json tobj;
      tobj["type"] = "warning";
      tobj["message"] = msg.str();
      feedback.push_back(tobj);
      pulserfreq = (1<<26)-1;
    }

    set_bit(5,31,pulser_en);
    set_bit_range_register(5,0,26,pulserfreq);


    //
    // Timing endpoint configuration
    //

    json timingconf = miscconfig.at("timing");
    //    std::string s_t_addr = timingconf.at("address").get<std::string>();
    //    std::string s_t_group = timingconf.at("group").get<std::string>();
    //std::string s_cmd_lockout = timingconf.at("lockout").get<std::string>();
    //    uint32_t t_addr = (uint32_t)strtoul(s_t_addr.c_str(),NULL,0);
    //    uint32_t t_group = (uint32_t)strtoul(s_t_group.c_str(),NULL,0);
    //uint32_t cmd_lockout = (uint32_t)strtoul(s_cmd_lockout.c_str(),NULL,0);


    uint32_t cmd_lockout = (uint32_t)strtoul(timingconf.at("lockout").get<std::string>().c_str(),NULL,0);
    bool trigger_en = timingconf.at("triggers").get<bool>();

    uint32_t lockout_reg = (cmd_lockout<<11);
    set_bit_range_register(61,0,19,lockout_reg);
    set_bit(61,10,trigger_en);

//
//
//    set_bit(62,0,ch_status_en);
//    set_bit(70,0,standalone_en);
//    set_bit(70,1,beam_mode);
//
//    set_bit(27,0,rtrigger_en);
//    set_bit(62,2,rtrig_fixed_freq);
//    set_bit(5,31,pulser_en);
//
//    set_bit_range_register(4,0,26,rtriggerperiod);
//    set_bit_range_register(5,0,26,pulserfreq);
//
//    uint32_t t_param = (cmd_lockout<<11);// + (t_group<<8) + t_addr;
//    set_bit_range_register(61,0,19,t_param);
//    set_bit(61,10,trigger_ena);
//
//    //Place warnings into json feedback obj
//    json obj;
//    for (size_t i=0; i<tmp.size(); i++) {
//      if (tmp[i].second.compare("") != 0) {
//        obj["type"] = tmp[i].first;
//        obj["message"] = tmp[i].second;
//        feedback.push_back(obj);
//      }
//    }

  } // Misc configs

  ////////////////////////////////////////////
  // HLT configuration

  void board_manager::hlt_config(json& hltconfig, json& feedback, bool &has_error) {

//    size_t err_msgs = 14;
//    std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

    Log(info,"Configuring CTB HLTs");

    json trigs = hltconfig.at("trigger");

    // -- these have all the same structure, so it is easy to
    // set up efficient code without too much repetition

    bool hlt_en;
    uint32_t hlt_minc;
    uint32_t hlt_mexc;
    uint32_t hlt_prscl;
    std::string hlt_id;

    for (json::iterator it = trigs.begin(); it != trigs.end(); ++it) {
      hlt_en = it->at("enable").get<bool>();
      hlt_id = it->at("id").get<std::string>();
      hlt_minc = (uint32_t)strtoul(it->at("minc").get<std::string>().c_str(),NULL,0);
      hlt_mexc = (uint32_t)strtoul(it->at("mexc").get<std::string>().c_str(),NULL,0);
      hlt_prscl = (uint32_t)strtoul(it->at("prescale").get<std::string>().c_str(),NULL,0);

      // -- Now make consistency checks
      if (hlt_en) {
        Log(info,"%s status is enabled.",hlt_id.c_str());
        // Inc mask is 0x0. Same as disabling it
        if (hlt_minc == 0x0) {
          std::ostringstream msg;
          msg << hlt_id << " has mask 0x0. This is the same as disabling it.";
          hlt_en = false;
          Log(warning,"%s",msg.str().c_str());
          json tobj;
          tobj["type"] = "warning";
          tobj["message"] = msg.str();
          feedback.push_back(tobj);
        }

        if (hlt_prscl == 0x0) {
          std::ostringstream msg;
          msg << hlt_id << " has prescale 0x0. Minimum is 0x1 (no prescale). Readjusting.";
          hlt_prscl = 0x1;
          Log(warning,"%s",msg.str().c_str());
          json tobj;
          tobj["type"] = "warning";
          tobj["message"] = msg.str();
          feedback.push_back(tobj);
        }

        if (hlt_prscl > ((0x1<<5)-1)) {
          std::ostringstream msg;
          msg << hlt_id << " has prescale above allowed limit (2^5-1). Truncating to maximum.";
          hlt_prscl = ((0x1<<5)-1);
          Log(warning,"%s",msg.str().c_str());
          json tobj;
          tobj["type"] = "warning";
          tobj["message"] = msg.str();
          feedback.push_back(tobj);
        }

      } else {
        Log(debug,"%s status is disabled. No checks on its contents", hlt_id.c_str());

      }

      if (hlt_id == "HLT_1") {
        set_bit(27,20,hlt_en);
        set_bit_range_register(55,0,32,hlt_minc);
        set_bit_range_register(128,0,32,hlt_mexc);
        set_bit_range_register(64,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_2") {
        set_bit(27,21,hlt_en);
        set_bit_range_register(56,0,32,hlt_minc);
        set_bit_range_register(129,0,32,hlt_mexc);
        set_bit_range_register(65,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_3") {
        set_bit(27,22,hlt_en);
        set_bit_range_register(57,0,32,hlt_minc);
        set_bit_range_register(130,0,32,hlt_mexc);
        set_bit_range_register(66,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_4") {
        set_bit(27,23,hlt_en);
        set_bit_range_register(58,0,32,hlt_minc);
        set_bit_range_register(131,0,32,hlt_mexc);
        set_bit_range_register(67,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_5") {
        set_bit(27,24,hlt_en);
        set_bit_range_register(59,0,32,hlt_minc);
        set_bit_range_register(132,0,32,hlt_mexc);
        set_bit_range_register(68,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_6") {
        set_bit(27,25,hlt_en);
        set_bit_range_register(60,0,32,hlt_minc);
        set_bit_range_register(133,0,32,hlt_mexc);
        set_bit_range_register(69,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_7") {
        set_bit(72,8,hlt_en);
        set_bit_range_register(87,0,32,hlt_minc);
        set_bit_range_register(88,0,32,hlt_mexc);
        set_bit_range_register(89,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_8") {
        set_bit(72,9,hlt_en);
        set_bit_range_register(100,0,32,hlt_minc);
        set_bit_range_register(101,0,32,hlt_mexc);
        set_bit_range_register(102,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_9") {
        set_bit(72,10,hlt_en);
        set_bit_range_register(103,0,32,hlt_minc);
        set_bit_range_register(104,0,32,hlt_mexc);
        set_bit_range_register(105,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_10") {
        set_bit(72,11,hlt_en);
        set_bit_range_register(106,0,32,hlt_minc);
        set_bit_range_register(107,0,32,hlt_mexc);
        set_bit_range_register(108,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_11") {
        set_bit(72,12,hlt_en);
        set_bit_range_register(109,0,32,hlt_minc);
        set_bit_range_register(110,0,32,hlt_mexc);
        set_bit_range_register(111,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_12") {
        set_bit(72,13,hlt_en);
        set_bit_range_register(112,0,32,hlt_minc);
        set_bit_range_register(113,0,32,hlt_mexc);
        set_bit_range_register(114,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_13") {
        set_bit(72,14,hlt_en);
        set_bit_range_register(115,0,32,hlt_minc);
        set_bit_range_register(116,0,32,hlt_mexc);
        set_bit_range_register(117,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_14") {
        set_bit(72,15,hlt_en);
        set_bit_range_register(118,0,32,hlt_minc);
        set_bit_range_register(119,0,32,hlt_mexc);
        set_bit_range_register(120,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_15") {
        set_bit(72,16,hlt_en);
        set_bit_range_register(121,0,32,hlt_minc);
        set_bit_range_register(122,0,32,hlt_mexc);
        set_bit_range_register(123,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_16") {
        set_bit(72,17,hlt_en);
        set_bit_range_register(124,0,32,hlt_minc);
        set_bit_range_register(125,0,32,hlt_mexc);
        set_bit_range_register(126,0,32,hlt_prscl);
      } else if (hlt_id == "HLT_17") {
        set_bit(27,19,hlt_en);
        set_bit_range_register(54,0,32,hlt_minc);
        set_bit_range_register(127,0,32,hlt_mexc);
        set_bit_range_register(63,0,32,hlt_prscl);
      } else {
        std::ostringstream msg;
        msg << "Unrecognized HLT with ID [" << hlt_id << "] ";
        Log(warning,"%s",msg.str().c_str());
        json obj;
        obj["type"]="warning";
        obj["message"] = msg.str();

        feedback.push_back(obj);
      }
      

    }
      Log(info,"Completed configuration of HLTs");

//
//    bool hlt1_enable = trigs.at(0).at("enable").get<bool>();
//    std::string s_hlt1minc = trigs.at(0).at("minc").get<std::string>();
//    std::string s_hlt1mexc = trigs.at(0).at("mexc").get<std::string>();
//    std::string s_hlt1prescale = trigs.at(0).at("prescale").get<std::string>();
//    uint32_t hlt1minc = (uint32_t)strtoul(s_hlt1minc.c_str(),NULL,0);
//    uint32_t hlt1mexc = (uint32_t)strtoul(s_hlt1mexc.c_str(),NULL,0);
//    uint32_t hlt1prescale = (uint32_t)strtoul(s_hlt1prescale.c_str(),NULL,0);
//    std::ostringstream oss0;
//    std::ostringstream oss7;
//    if (hlt1minc == 0) {
//      oss0 << trigs.at(0).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss0.str());
//    }
//    if (hlt1prescale == 0) {
//      oss7 << trigs.at(0).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss7.str());
//      hlt1prescale = 1;
//    }
//    tmp[0] = std::make_pair("warning", oss0.str());
//    tmp[7] = std::make_pair("warning", oss7.str());
//
//    bool hlt2_enable = trigs.at(1).at("enable").get<bool>();
//    std::string s_hlt2minc = trigs.at(1).at("minc").get<std::string>();
//    std::string s_hlt2mexc = trigs.at(1).at("mexc").get<std::string>();
//    std::string s_hlt2prescale = trigs.at(1).at("prescale").get<std::string>();
//    uint32_t hlt2minc = (uint32_t)strtoul(s_hlt2minc.c_str(),NULL,0);
//    uint32_t hlt2mexc = (uint32_t)strtoul(s_hlt2mexc.c_str(),NULL,0);
//    uint32_t hlt2prescale = (uint32_t)strtoul(s_hlt2prescale.c_str(),NULL,0);
//    std::ostringstream oss1;
//    std::ostringstream oss8;
//    if (hlt2minc == 0) {
//      oss1 << trigs.at(1).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss1.str());
//    }
//    if (hlt2prescale == 0) {
//      oss8 << trigs.at(1).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss8.str());
//      hlt2prescale = 1;
//    }
//    tmp[1] = std::make_pair("warning", oss1.str());
//    tmp[8] = std::make_pair("warning", oss8.str());
//
//    bool hlt3_enable = trigs.at(2).at("enable").get<bool>();
//    std::string s_hlt3minc = trigs.at(2).at("minc").get<std::string>();
//    std::string s_hlt3mexc = trigs.at(2).at("mexc").get<std::string>();
//    std::string s_hlt3prescale = trigs.at(2).at("prescale").get<std::string>();
//    uint32_t hlt3minc = (uint32_t)strtoul(s_hlt3minc.c_str(),NULL,0);
//    uint32_t hlt3mexc = (uint32_t)strtoul(s_hlt3mexc.c_str(),NULL,0);
//    uint32_t hlt3prescale = (uint32_t)strtoul(s_hlt3prescale.c_str(),NULL,0);
//    std::ostringstream oss2;
//    std::ostringstream oss9;
//    if (hlt3minc == 0) {
//      oss2 << trigs.at(2).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss2.str());
//    }
//    if (hlt3prescale == 0) {
//      oss9 << trigs.at(2).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss9.str());
//      hlt3prescale = 1;
//    }
//    tmp[2] = std::make_pair("warning", oss2.str());
//    tmp[9] = std::make_pair("warning", oss9.str());
//
//    bool hlt4_enable = trigs.at(3).at("enable").get<bool>();
//    std::string s_hlt4minc = trigs.at(3).at("minc").get<std::string>();
//    std::string s_hlt4mexc = trigs.at(3).at("mexc").get<std::string>();
//    std::string s_hlt4prescale = trigs.at(3).at("prescale").get<std::string>();
//    uint32_t hlt4minc = (uint32_t)strtoul(s_hlt4minc.c_str(),NULL,0);
//    uint32_t hlt4mexc = (uint32_t)strtoul(s_hlt4mexc.c_str(),NULL,0);
//    uint32_t hlt4prescale = (uint32_t)strtoul(s_hlt4prescale.c_str(),NULL,0);
//    std::ostringstream oss3;
//    std::ostringstream oss10;
//    if (hlt4minc == 0) {
//      oss3 << trigs.at(3).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss3.str());
//    }
//    if (hlt4prescale == 0) {
//      oss10 << trigs.at(3).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss10.str());
//      hlt4prescale = 1;
//    }
//    tmp[3] = std::make_pair("warning", oss3.str());
//    tmp[10] = std::make_pair("warning", oss10.str());
//
//    bool hlt5_enable = trigs.at(4).at("enable").get<bool>();
//    std::string s_hlt5minc = trigs.at(4).at("minc").get<std::string>();
//    std::string s_hlt5mexc = trigs.at(4).at("mexc").get<std::string>();
//    std::string s_hlt5prescale = trigs.at(4).at("prescale").get<std::string>();
//    uint32_t hlt5minc = (uint32_t)strtoul(s_hlt5minc.c_str(),NULL,0);
//    uint32_t hlt5mexc = (uint32_t)strtoul(s_hlt5mexc.c_str(),NULL,0);
//    uint32_t hlt5prescale = (uint32_t)strtoul(s_hlt5prescale.c_str(),NULL,0);
//    std::ostringstream oss4;
//    std::ostringstream oss11;
//    if (hlt5minc == 0) {
//      oss4 << trigs.at(4).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss4.str());
//    }
//    if (hlt5prescale == 0) {
//      oss11 << trigs.at(4).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss11.str());
//      hlt5prescale = 1;
//    }
//    tmp[4] = std::make_pair("warning", oss4.str());
//    tmp[11] = std::make_pair("warning", oss4.str());
//
//    bool hlt6_enable = trigs.at(5).at("enable").get<bool>();
//    std::string s_hlt6minc = trigs.at(5).at("minc").get<std::string>();
//    std::string s_hlt6mexc = trigs.at(5).at("mexc").get<std::string>();
//    std::string s_hlt6prescale = trigs.at(5).at("prescale").get<std::string>();
//    uint32_t hlt6minc = (uint32_t)strtoul(s_hlt6minc.c_str(),NULL,0);
//    uint32_t hlt6mexc = (uint32_t)strtoul(s_hlt6mexc.c_str(),NULL,0);
//    uint32_t hlt6prescale = (uint32_t)strtoul(s_hlt6prescale.c_str(),NULL,0);
//    std::ostringstream oss5;
//    std::ostringstream oss12;
//    if (hlt4minc == 0) {
//      oss5 << trigs.at(5).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss5.str());
//    }
//    if (hlt6prescale == 0) {
//      oss12 << trigs.at(5).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss12.str());
//      hlt6prescale = 1;
//    }
//    tmp[5] = std::make_pair("warning", oss5.str());
//    tmp[12] = std::make_pair("warning", oss12.str());
//
//    bool hlt7_enable = trigs.at(6).at("enable").get<bool>();
//    std::string s_hlt7minc = trigs.at(6).at("minc").get<std::string>();
//    std::string s_hlt7mexc = trigs.at(6).at("mexc").get<std::string>();
//    std::string s_hlt7prescale = trigs.at(6).at("prescale").get<std::string>();
//    uint32_t hlt7minc = (uint32_t)strtoul(s_hlt7minc.c_str(),NULL,0);
//    uint32_t hlt7mexc = (uint32_t)strtoul(s_hlt7mexc.c_str(),NULL,0);
//    uint32_t hlt7prescale = (uint32_t)strtoul(s_hlt7prescale.c_str(),NULL,0);
//    std::ostringstream oss6;
//    std::ostringstream oss13;
//    if (hlt7minc == 0) {
//      oss6 << trigs.at(6).at("id").get<std::string>() << " mask set to 0x0. Equivalent to disabled.";
//      Log(warning,"%s", oss6.str());
//    }
//    if (hlt7prescale == 0) {
//      oss13 << trigs.at(6).at("id").get<std::string>() << " prescale undefined (0x0). Setting to 0x1";
//      Log(warning,"%s", oss13.str());
//      hlt7prescale = 1;
//    }
//    tmp[6] = std::make_pair("warning", oss6.str());
//    tmp[13] = std::make_pair("warning", oss13.str());
//
//    //HLT enables
//    set_bit(27,19,hlt1_enable);
//    set_bit(27,20,hlt2_enable);
//    set_bit(27,21,hlt3_enable);
//    set_bit(27,22,hlt4_enable);
//    set_bit(27,23,hlt5_enable);
//    set_bit(27,24,hlt6_enable);
//    set_bit(27,25,hlt7_enable);
//
//    //The exclusive mask goes in 16 upper bits while the inclusive mask goes in the 16 lower bits
//    uint32_t hlt1 = (hlt1mexc<<16) + hlt1minc;
//    uint32_t hlt2 = (hlt2mexc<<16) + hlt2minc;
//    uint32_t hlt3 = (hlt3mexc<<16) + hlt3minc;
//    uint32_t hlt4 = (hlt4mexc<<16) + hlt4minc;
//    uint32_t hlt5 = (hlt5mexc<<16) + hlt5minc;
//    uint32_t hlt6 = (hlt6mexc<<16) + hlt6minc;
//    uint32_t hlt7 = (hlt7mexc<<16) + hlt7minc;
//
//    //HLT parameter registers
//    set_bit_range_register(54,0,32,hlt1);
//    set_bit_range_register(55,0,32,hlt2);
//    set_bit_range_register(56,0,32,hlt3);
//    set_bit_range_register(57,0,32,hlt4);
//    set_bit_range_register(58,0,32,hlt5);
//    set_bit_range_register(59,0,32,hlt6);
//    set_bit_range_register(60,0,32,hlt7);
//
//    //HLT prescale registers
//    set_bit_range_register(63,0,32,hlt1prescale);
//    set_bit_range_register(64,0,32,hlt2prescale);
//    set_bit_range_register(65,0,32,hlt3prescale);
//    set_bit_range_register(66,0,32,hlt4prescale);
//    set_bit_range_register(67,0,32,hlt5prescale);
//    set_bit_range_register(68,0,32,hlt6prescale);
//    set_bit_range_register(69,0,32,hlt7prescale);
//
//    //Place warnings into json feedback obj
//    json obj;
//    for (size_t i=0; i<tmp.size(); i++) {
//      if (tmp[i].second.compare("") != 0) {
//        obj["type"] = tmp[i].first;
//        obj["message"] = tmp[i].second;
//        feedback.push_back(obj);
//      }
//    }

  } // HLT configs


}
