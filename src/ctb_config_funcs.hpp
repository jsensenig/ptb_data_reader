  namespace ptb {

  #define NBEAM_CH 9
  #define NPDS_CH 24
  #define NCRT_CH 32

    void board_manager::configure_ctb(const json& doc, json& answers, bool &has_error) {
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
        config_misc(miscconf, mfeedback, has_error);
        if (!mfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the Misc Configs",mfeedback.size());
          answers.insert(answers.end(),mfeedback.begin(),mfeedback.end());
        }
        if (has_error) {
          Log(error,"Caught error after configuring misc. Interrupting configuration.");
          return;
        }

        json hfeedback;
        config_hlt(hltconf, hfeedback, has_error);
        if (!hfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the HLTs",hfeedback.size());
          answers.insert(answers.end(),hfeedback.begin(),hfeedback.end());
        }
        if (has_error) {
          Log(error,"Caught error after configuring HLTs. Interrupting configuration.");
          return;
        }

        json bfeedback;
        config_beam(beamconf, bfeedback, has_error);
        if (!bfeedback.empty()) {
          Log(debug,"Received %u messages from configuring the Beam",bfeedback.size());
          answers.insert(answers.end(),bfeedback.begin(),bfeedback.end());
        }
        if (has_error) {
          Log(error,"Caught error after configuring the beam inputs. Interrupting configuration.");
          return;
        }

        json cfeedback;
        config_crt(crtconf, cfeedback, has_error);
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
        config_pds(pdsconf,feedback, has_error);
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

    }// main config

    ///
    /// Beam configuration
    ///
    void board_manager::config_beam(const json& bdoc, json& feedback, bool &has_error) {

      // -- smaller and cleaner code


      // Get the delays
      std::vector<uint32_t> delays = bdoc.at("delays").get<std::vector<uint32_t>>();
      
      // -- Grab the channel mask for the beam inputs
      // std::string s_channelmask = bdoc.at("channel_mask").get<std::string>();
      // uint32_t ch_mask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      uint32_t ch_mask = static_cast<uint32_t>(strtoul(bdoc.at("channel_mask").get<std::string>().c_str(),NULL,0));

      // -- get the reshape length (aka trigger gate)
      uint32_t reshape_len = bdoc.at("reshape_length").get<unsigned int>();
      
      Log(debug,"Beam reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      // FIXME: We should think about increasing this. It is not outside the realm of possibility to attemp to do 
      //        a trigger gate of 2-3 us
      if (reshape_len > ((0x1<<6)-1)) {
        std::ostringstream msg;
        msg << "Reshaping length value of " << reshape_len << " above maximum allwed [2^6 - 1]. Truncating to maximum.";
        json fobj;
        fobj["type"] = "warning";
        fobj["message"] = msg.str();
        feedback.push_back(fobj);

        Log(warning,"%s", msg.str().c_str());
        reshape_len = (1<<6)-1;
      }

      if (ch_mask == 0x0) {
        std::ostringstream oss;
        oss <<  "Beam channel mask set for [0x0]. These inputs won't be registered.";
        Log(warning,"%s", oss.str().c_str());
        json fobj;
        fobj["type"] = "warning";
        fobj["message"] = oss.str();
        feedback.push_back(fobj);
      }

      // -- Check that the mask is not bigger than the max allowed by the number of channels
      if (((0x1 <<NBEAM_CH)-1) < ch_mask) {
        std::ostringstream msg;
        msg << "Beam channel mask larger than maximum possible (" << std::hex << ch_mask << std::dec
        << "vs " << std::hex << ((0x1 <<NBEAM_CH)-1) << std::dec << ").";
        Log(error,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;
      }

      if (delays.size() != NBEAM_CH) {
        std::ostringstream msg;
        msg << "Number of beam delays (" << delays.size() << ") doesn't match number of beam channels (" << NPDS_CH << ")";
        Log(error,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;
      }



      // Commit the masks to the registers
      set_bit_range_register(3,0,9,ch_mask);
      set_bit_range_register(24,0,6,reshape_len);

      // -- Now load the beam LLT configuration
      bool llt_en = false;
      std::string llt_id;
      uint32_t llt_mask = 0x0;

      // -- Grab the triggers (it comes in the shape of an array)
      json trigs = bdoc.at("triggers"); //Array of trigger configs

      // NFB: This is done one by one, since the registers are manually allocated
      // see https://docs.google.com/spreadsheets/d/156ZUa09TceKZn4-OJEnJtfSZLkQl1tAYATt9xLz5o20/edit?usp=sharing
      // for details about the individual register allocation

      for (json::iterator it = trigs.begin(); it != trigs.end(); ++it) {
        // -- LOOP over the LLTs
        llt_en = it->at("enable").get<bool>();
        llt_mask = (uint32_t)strtoul(it->at("mask").get<std::string>().c_str(),NULL,0);
        llt_id = it->at("id").get<std::string>();
        Log(info,"Processing %s ...",llt_id.c_str());
        Log(debug,"Setting %s to state %u mask [%X]",llt_id.c_str(),llt_en,llt_mask);

        // -- Are there verifications to be made?
        if (llt_en) {
          // If a mask is 0x0 it is the same as disabling
          if (llt_mask == 0x0) {
            std::ostringstream msg;
            msg << "Beam " << llt_id << " has a mask [0x0]. This effectively disables it.";
            Log(warning,"%s",msg.str().c_str());
            json tobj;
            tobj["type"] = "warning";
            tobj["message"] = msg.str();
            feedback.push_back(tobj);        
          }

        } else {
          Log(debug,"%s is disabled. Skipping consistency checks.",llt_id.c_str());
        }

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

        Log(info,"%s summary: en %u mask [0x%X]",llt_id.c_str(),llt_en,llt_mask);

      }

      // -- Process delays
      // NFB : Careful with the use of uint8_t. This has a very short range and can 
      //       cause troubles down the line if one wants to expand
      Log(debug,"Setting beam delays...");    
      const uint32_t dbits = 7;
      const size_t n_regs = static_cast<size_t>(delays.size()/4);
      // Beam delays 9ch * 7b = 63b --> 3 regs
      for(size_t i=0; i<n_regs; i++) {

        int j = i * 4;
        uint32_t reg = i + 21;
        //uint32_t dval = (delays[j+3] << dbits*3) | (delays[j+2] << dbits*2) | (delays[j+1] << dbits) | delays[j];
        uint32_t dval = ((delays[j+3] & 0x7F) << dbits*3) | ((delays[j+2] & 0x7F) << dbits*2) | ((delays[j+1] & 0x7F) << dbits) | (delays[j] & 0x7F);
        Log(debug,"Setting reg %u to [%08X]",reg,dval);
        set_bit_range_register(reg,0,dbits*4,dval);
      }
      // -- Set the last register that sits alone 
      set_bit_range_register(23,0,dbits,delays[delays.size()-1]);
      Log(debug,"Done with the beam config.");
    } //Beam config

    ///
    /// CRT configuration
    ///
    void board_manager::config_crt(const json& crtconfig, json& feedback, bool &has_error) {

      json trigs = crtconfig.at("triggers"); //Array of trigger configs

      std::vector<uint32_t> delays = crtconfig.at("delays").get<std::vector<uint32_t>>();

      // std::string ch_mask = crtconfig.at("channel_mask").get<std::string>();
      // uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);

      uint32_t ch_mask = static_cast<uint32_t>(strtoul(crtconfig.at("channel_mask").get<std::string>().c_str(),NULL,0));

      bool crt_pixel_en = crtconfig.at("pixelate").get<bool>();

      if (ch_mask == 0x0) {
        std::ostringstream msg;
        msg << "CRT channel mask set for [0x0]. No channel changes will record and CRT won't contribute to triggers.";
        Log(warning,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      } 

      // -- Check that the mask is not bigger than the max allowed by the number of channels
      // if (((0x1 <<NCRT_CH)-1) < ch_mask) {
      //   std::ostringstream msg;
      //   msg << "CRT channel mask larger than maximum possible (" << std::hex << ch_mask << std::dec
      //   << "vs " << std::hex << ((0x1 <<NCRT_CH)-1) << std::dec << ").";
      //   Log(error,"%s", msg.str().c_str());
      //   json tobj;
      //   tobj["type"] = "error";
      //   tobj["message"] = msg.str();
      //   feedback.push_back(tobj);
      //   has_error = true;
      //   return;
      // }

      if (delays.size() != NCRT_CH) {
        std::ostringstream msg;
        msg << "Number of CRT delays (" << delays.size() << ") doesn't match number of CRT channels (" << NPDS_CH << ")";
        Log(error,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;
      }
      // If pixelate is off, the masks have to be very different
      if (!crt_pixel_en) {
        std::ostringstream msg;
        msg << "CRT channel pixelation is **disabled**. Different trigger masks are likely necessary.";
        Log(warning,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);        
      }

      uint32_t reshape_len = crtconfig.at("reshape_length").get<unsigned int>();
      Log(debug,"CRT Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      if (reshape_len > ((1<<6)-1)) {
        std::ostringstream msg;
        msg << "CRT reshape length of " << reshape_len << " above maximum [2^6 - 1]. Truncating to maximum.";
        Log(warning,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        reshape_len = (1<<6)-1;
      }

      //Input channel masks
      set_bit_range_register(1,0,32,ch_mask);
      set_bit(62,1,crt_pixel_en);
      set_bit_range_register(25,0,6,reshape_len);


      // -- Process the triggers
      // -- All CRT LLTs are of type counter

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

        Log(info,"Processing %s...",llt_id.c_str());
        Log(debug,
            "Setting [%s] to state %u mask [%X] count [%u] comp [%c]",
            llt_id.c_str(),
            llt_en,
            llt_mask,
            llt_count,
            util::llt_cop_tochar(llt_comp));

        if (llt_en) {

          // -- Check validity of inputs
          // the operation can only be  0x1,0x2,0x4,
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
          // FIXME: Reevaluate if this check makes sense
          // // -- If count is 0 and operation is '=' we are basically disabling it
          // // -- This could be valid
          // if ((llt_comp == 0x2) && (llt_count == 0x0)) {
          //   std::ostringstream msg;
          //   msg << "CRT " << llt_id << " has a condition of 'counts==0'. This is the same as being disabled.";
          //   Log(warning,"%s",msg.str().c_str());
          //   json tobj;
          //   tobj["type"] = "warning";
          //   tobj["message"] = msg.str();
          //   feedback.push_back(tobj);
          // }
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
          // note that == and count = n_bits(mask) is still valid
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
                << llt_count << " vs " << ((0x1<<5)-1) << "). Aborting configuration.";
            Log(error,"%s",msg.str().c_str());
            json tobj;
            tobj["type"] = "error";
            tobj["message"] = msg.str();
            feedback.push_back(tobj);
            has_error=true;
            return;
          }
        } else {
          Log(debug,"%s is disabled. Skipping consistency checks.",llt_id.c_str());
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
        Log(info,
          "%s summary: en %u mask [0x%X] comp [0x%X] count [0x%X]",
          llt_id.c_str(),llt_en,llt_mask,llt_comp,llt_count);
      }

      // -- Process delays
      Log(debug,"Setting CRT delays...");
      const uint32_t dbits = 7;
      const size_t n_regs = static_cast<size_t>(delays.size()/4);
      // PDS delays 32ch * 7b = 224b --> 8 regs
      for(size_t i=0; i<n_regs; i++) {

        int j = i * 4;
        uint32_t reg = i + 13;
        //uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];
        uint32_t dval = ((delays[j+3] & 0x7F) << dbits*3) | ((delays[j+2] & 0x7F) << dbits*2) | ((delays[j+1] & 0x7F) << dbits) | (delays[j] & 0x7F);
        Log(debug,"Setting reg %u to [%08X]",reg,dval);
        set_bit_range_register(reg,0,dbits*4,dval);
      }
      Log(debug,"Done with the CRT config.");

    } //CRT config

    ///
    /// PDS configuration
    ///
    void board_manager::config_pds(const json& pdsconfig, json& feedback, bool &has_error) {

      // size_t err_msgs = 14;
      // std::vector< std::pair< std::string, std::string > > tmp(err_msgs);

      json trigs = pdsconfig.at("triggers"); //Array of trigger configs

      std::vector<uint32_t> dac_values = pdsconfig.at("dac_thresholds").get<std::vector<uint32_t>>();
      // get the channel mask
      uint32_t ch_mask = static_cast<uint32_t>(strtoul(pdsconfig.at("channel_mask").get<std::string>().c_str(),NULL,0));

      // std::string s_channelmask = pdsconfig.at("channel_mask").get<std::string>();
      // uint32_t channelmask = (uint32_t)strtoul(s_channelmask.c_str(),NULL,0);
      std::vector<uint32_t> delays = pdsconfig.at("delays").get<std::vector<uint32_t>>();

      if (ch_mask == 0x0) {
        std::ostringstream msg;
        msg << "PDS channel mask set for [0x0]. No channel changes will record and PDS won't contribute to triggers.";
        Log(warning,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
      } 

      // -- Check that the mask is not bigger than the max allowed by the number of channels
      if (((0x1 <<NPDS_CH)-1) < ch_mask) {
        std::ostringstream msg;
        msg << "PDS channel mask larger than maximum possible (" << std::hex << ch_mask << std::dec
        << "vs " << std::hex << ((0x1 <<NPDS_CH)-1) << std::dec << ").";
        Log(error,"%s", msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;
      }


      uint32_t reshape_len = pdsconfig.at("reshape_length").get<unsigned int>();
      Log(debug,"Reshape length [%d] (%u) [0x%X][%s]",reshape_len,reshape_len,reshape_len, std::bitset<6>(reshape_len).to_string().c_str());
      if (reshape_len > ((1<<6)-1)) {
        std::ostringstream msg;
        msg << "PDS reshape length of " << reshape_len << " above maximum [2^6 - 1]. Truncating to maximum.";
        Log(warning,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "warning";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        reshape_len = (1<<6)-1;
      }

      // -- assign the registers
      //Input channel masks
      set_bit_range_register(2,0,24,ch_mask);
      // reshape length
      set_bit_range_register(26,0,6,reshape_len);


      // -- Delays
      Log(debug,"Setting PDS delays...");

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
          msg << "PDS delay value on ch " << i << " out of range (" << delays.at(i) << "). Truncating to maximum (127)";
          Log(warning,"%s", msg.str());
          json tobj;
          tobj["type"] = "warning";
          tobj["message"] = msg.str();
          feedback.push_back(tobj);
        }
      }

      // -- assign the delays
      const uint32_t dbits = 7;
      const size_t n_regs = static_cast<size_t>(delays.size()/4);

      // PDS delays 24ch * 7b = 168b --> 6 regs
      for(size_t i=0; i<n_regs; i++) {

        int j = i * 4;
        uint32_t reg = i + 7;
        // uint32_t dval = (delays[j+3] << dbits*3) + (delays[j+2] << dbits*2) + (delays[j+1] << dbits) + delays[j];
        uint32_t dval = ((delays[j+3] & 0x7F) << dbits*3) | ((delays[j+2] & 0x7F) << dbits*2) | ((delays[j+1] & 0x7F) << dbits) | (delays[j] & 0x7F);
        set_bit_range_register(reg,0,dbits*4,dval);
      }

      Log(debug,"Processing PDS triggers.");


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


        Log(info,"Processing %s...",llt_id.c_str());
        Log(debug,
            "Setting [%s] to state %u mask [%X] count [%u] comp [%c]",
            llt_id.c_str(),
            llt_en,
            llt_mask,
            llt_count,
            util::llt_cop_tochar(llt_comp));

        // -- Now check for errors
        // only check if the LLT is enabled
        if (llt_en) {

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
          // this is not true. Asking for == 0x0 is valid, even if stupid
          // if ((llt_comp == 0x2) && (llt_count == 0x0)) {
          //   std::ostringstream msg;
          //   msg << "PDS " << llt_id << " has a condition of 'counts==0'. This is the same as being disabled.";
          //   Log(warning,"%s",msg.str().c_str());
          //   json tobj;
          //   tobj["type"] = "warning";
          //   tobj["message"] = msg.str();
          //   feedback.push_back(tobj);
          // }

          // -- If count is 0 and operation is '<' we are basically disabling it
          if ((llt_comp == 0x4) && (llt_count == 0x0)) {
            std::ostringstream msg;
            msg << "PDS " << llt_id << " has a condition of 'counts<0'. This is impossible. Considering it as disabled.";
            llt_en = false;
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
            msg << "PDS " << llt_id << " has a condition of '> or =' and 'counts>number of masked channels' (counts"
                << llt_count << util::llt_cop_tochar(llt_comp) << maskbits.count() << "). This is the same as being disabled.";
            Log(warning,"%s",msg.str().c_str());
            llt_en = false;
            json tobj;
            tobj["type"] = "warning";
            tobj["message"] = msg.str();
            feedback.push_back(tobj);
          }
          // -- If count is outside of the range allowed by 5 bits we will have an issue as well
          if (llt_count > ((0x1<<5)-1)) {
            std::ostringstream msg;
            msg << "PDS " << llt_id << " has a condition with counts above number allowed by available bits ("
                << llt_count << " vs " << ((0x1<<5)-1) << "). Aborting configuration.";
            Log(error,"%s",msg.str().c_str());
            json tobj;
            tobj["type"] = "error";
            tobj["message"] = msg.str();
            feedback.push_back(tobj);
            has_error=true;
            return;
          }
        } else {
          Log(debug,"%s is disabled. Skipping consistency checks.",llt_id.c_str());
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
        Log(info,
          "%s summary: en %u mask [0x%X] comp [0x%X] count [0x%X]",
          llt_id.c_str(),llt_en,llt_mask,llt_comp,llt_count);
      }


      // -- Finally program the DACs
      ///FIXME: How can this work? dacsetup is not initialized in this function
      i2conf dacsetup;

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

      Log(info,"Number of PDS channel thresholds to be programmed: %i", dac_values.size());
      for (size_t i=0; i<dac_values.size(); i++) {
        Log(info,"Channel %zu value %u", i, dac_values.at(i));
        if (dac_values[i] > 4095) { //Range 0 - 4095
          std::ostringstream msg;
          msg << "DAC value for channel [" << i << "] out of range (" << dac_values.at(i) << "). Truncating to maximum (4095)";
          Log(warning,"%s", msg.str());
          json tobj;
          tobj["type"] = "warning";
          tobj["message"] = msg.str();
          feedback.push_back(tobj);

          dac_values[i] = 4095;
        }
      }

      //Now pass DAC configs to setup
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
      Log(info,"Programmed %zu DAC channels", dac_values.size());

      Log(debug,"Done with the PDS config.");

    } // PDS config

    ///
    /// Misc configuration
    ///
    void board_manager::config_misc(const json& miscconfig, json& feedback, bool &has_error) {

      Log(debug,"Processing miscelania parameters.");

      bool ch_status_en = miscconfig.at("ch_status").get<bool>();
      bool standalone_en = miscconfig.at("standalone_enable").get<bool>();

      set_bit(62,0,ch_status_en);
      set_bit(70,0,standalone_en);

      // Make it compatible with the old configurations. 
      // If an exception is caught here, assume the second random trigger is off
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

        set_bit(71,31,rndm_tr_en);
        set_bit(71,30,rndm_tr_ff);
        set_bit(71,29,rndm_tr_bm);
        set_bit_range_register(138,0,32,rndm_tr_per);


      }

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
      // FIXME: These parameters are currently being passed through GPIO
      //    std::string s_t_addr = timingconf.at("address").get<std::string>();
      //    std::string s_t_group = timingconf.at("group").get<std::string>();
      //    uint32_t t_addr = (uint32_t)strtoul(s_t_addr.c_str(),NULL,0);
      //    uint32_t t_group = (uint32_t)strtoul(s_t_group.c_str(),NULL,0);


      uint32_t cmd_lockout = (uint32_t)strtoul(timingconf.at("lockout").get<std::string>().c_str(),NULL,0);
      bool trigger_en = timingconf.at("triggers").get<bool>();

      uint32_t lockout_reg = (cmd_lockout<<11);
      set_bit_range_register(61,0,19,lockout_reg);
      set_bit(61,10,trigger_en);

      Log(debug,"Done with miscelania settings.");

    } // Misc configs

    ///
    /// HLT configuration
    /// 
    void board_manager::config_hlt(const json& hltconfig, json& feedback, bool &has_error) {

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
        hlt_minc = static_cast<uint32_t>(strtoul(it->at("minc").get<std::string>().c_str(),NULL,0));
        hlt_mexc = static_cast<uint32_t>(strtoul(it->at("mexc").get<std::string>().c_str(),NULL,0));
        hlt_prscl = static_cast<uint32_t>(strtoul(it->at("prescale").get<std::string>().c_str(),NULL,0));

        Log(info,"Processing %s...",hlt_id.c_str());
        Log(debug,
            "Setting [%s] to state %u mask inc [%X] mask exc [%X] prescale [%u]",
            hlt_id.c_str(),hlt_en,hlt_minc,hlt_mexc,hlt_prscl);

        // -- Now make consistency checks
        if (hlt_en) {
          Log(debug,"%s is enabled.",hlt_id.c_str());
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
          // -- If there are overlapping bits in the include and exclude masks, there is an error 
          // in the HLT configuration
          if (hlt_minc & hlt_mexc) {

            std::ostringstream msg;
            msg << hlt_id << " has the same LLT masked in the inclusion and exclusion masks. This is not possible. Disabling HLT.";
            hlt_en = false;
            Log(error,"%s",msg.str().c_str());
            json tobj;
            tobj["type"] = "error";
            tobj["message"] = msg.str();
            feedback.push_back(tobj);
            has_error = true;
            return;
          }          

        } else {
          Log(debug,"%s is disabled. No checks on its contents", hlt_id.c_str());

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
        Log(info,
          "%s summary: en %u mask inc [0x%X] mask exc [0x%X] prescale count [%u]",
          hlt_id.c_str(),hlt_en,hlt_minc,hlt_mexc,hlt_prscl);
      }

      Log(info,"Processing HLT masks into each timing command.");
      // -- map HLTs into the different commands
	// hltconfig.at
      json cmd_masks = hltconfig.at("command_mask");
      // this is a json doc on its own, mapping the letter of the command
      // to the HLT mask
      uint32_t mask_C = static_cast<uint32_t>(strtoul(cmd_masks.at("C").get<std::string>().c_str(),NULL,0));
      uint32_t mask_D = static_cast<uint32_t>(strtoul(cmd_masks.at("D").get<std::string>().c_str(),NULL,0));
      uint32_t mask_E = static_cast<uint32_t>(strtoul(cmd_masks.at("E").get<std::string>().c_str(),NULL,0));
      uint32_t mask_F = static_cast<uint32_t>(strtoul(cmd_masks.at("F").get<std::string>().c_str(),NULL,0));

      Log(info,"Command C HLT mask : [0x%X]",mask_C);
      Log(info,"Command D HLT mask : [0x%X]",mask_D);
      Log(info,"Command E HLT mask : [0x%X]",mask_E);
      Log(info,"Command F HLT mask : [0x%X]",mask_F);

      // -- Check that the same trigger is not on more than one mask
      if (mask_C & mask_D) {
        std::ostringstream msg;
        msg << " Masks C and D have HLTs masked in both. This cannot happen. [C:" 
        << std::hex << mask_C << std::dec << " | D:" 
        << std::hex << mask_D << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_C & mask_D) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }
      if (mask_C & mask_E) {
        std::ostringstream msg;
        msg << " Masks C and E have HLTs masked in both. This cannot happen. [C:" 
        << std::hex << mask_C << std::dec << " | E:" 
        << std::hex << mask_E << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_C & mask_E) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }
      if (mask_C & mask_F) {
        std::ostringstream msg;
        msg << " Masks C and F have HLTs masked in both. This cannot happen. [C:" 
        << std::hex << mask_C << std::dec << " | F:" 
        << std::hex << mask_F << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_C & mask_F) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }
      if (mask_D & mask_F) {
        std::ostringstream msg;
        msg << " Masks D and F have HLTs masked in both. This cannot happen. [D:" 
        << std::hex << mask_D << std::dec << " | F:" 
        << std::hex << mask_F << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_D & mask_F) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }
      if (mask_D & mask_E) {
        std::ostringstream msg;
        msg << " Masks D and E have HLTs masked in both. This cannot happen. [D:" 
        << std::hex << mask_D << std::dec << " | E:" 
        << std::hex << mask_E << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_D & mask_E) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }
      if (mask_F & mask_E) {
        std::ostringstream msg;
        msg << " Masks E and F have HLTs masked in both. This cannot happen. [E:" 
        << std::hex << mask_E << std::dec << " | F:" 
        << std::hex << mask_F << std::dec << " | common:"
        << std::hex << static_cast<uint32_t>(mask_E & mask_F) << std::dec << "]. This is not possible. Disabling HLT.";
        hlt_en = false;
        Log(error,"%s",msg.str().c_str());
        json tobj;
        tobj["type"] = "error";
        tobj["message"] = msg.str();
        feedback.push_back(tobj);
        has_error = true;
        return;        
      }

      // attach the masks to the registers
      set_bit_range_register(134,0,32,mask_C);
      set_bit_range_register(135,0,32,mask_D);
      set_bit_range_register(136,0,32,mask_E);
      set_bit_range_register(137,0,32,mask_F);



      Log(info,"Completed configuration of HLTs");

    } // HLT configs


  }
