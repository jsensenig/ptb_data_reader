#include <iostream>
#include <fstream>
#include <cinttypes>
#include <regex>
#include "json.hpp"

using json = nlohmann::json;
using std::cout;
using std::endl;

int main() {

try {

  std::ifstream t("trigger_standard.fcl");
  std::string str;
  std::string baseconf, newconf;
  size_t conf_begin, conf_end;
  std::vector<unsigned int> triggers_enabled;
  std::vector<unsigned int> prescales;
  
  triggers_enabled.push_back(2);
  prescales.push_back(1);

  std::string key1, key2, key3;

  t.seekg(0, std::ios::end);   
  str.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  str.assign((std::istreambuf_iterator<char>(t)),
              std::istreambuf_iterator<char>());
  t.close();

  // Now find the 'board_config' entry
  if (str.find("board_config") != std::string::npos) {
    conf_begin = str.find("board_config");
    cout << "Begin position " << conf_begin << endl;
    // Now trim by finding the commas
    conf_begin = str.find("'",conf_begin);
    if (conf_begin == std::string::npos) {
      cout << "Failed to find the configuration" << endl;
      return 0;
    }
    //baseconf = baseconf.substr(conf_begin+1);
    conf_end = str.find("'",conf_begin+1);
    cout << "End position " << conf_end << endl;
    if (conf_end == std::string::npos) {
      cout << "Failed to find end of the configuration" << endl;
      return 0;
    }
    baseconf = str.substr(conf_begin+1,conf_end-conf_begin-1);
    cout << "Configuration : [" << baseconf << "]" << endl;
  }
  // by now have the base configuration
  // -- now parse it as JSON

  json config = json::parse(baseconf);
  cout << "Configuration parsed. Ready to customize" << endl;

  // -- Enable the triggers that are specified
  cout << "Enabling triggers with set prescales : " << endl;
  printf("Trigger : Prescale\n");
  // Assume that they are organized in an array
  for (size_t i = 0; i < triggers_enabled.size();i++ ) {
    printf("HLT_%u  : %u\n",triggers_enabled.at(i),prescales.at(i));
  }
  cout << "...disabling all other triggers!" << endl;

  json triggers = config.at("ctb").at("HLT").at("trigger");
  for (json::iterator it = triggers.begin(); it != triggers.end(); ++it) {
    key1 = it->at("id");
    unsigned int tid = 1;
    std::regex rx("HLT_([0-9]+)");
    std::smatch match;

    if (std::regex_match(key1, match, rx)) {
      std::ssub_match sub_match = match[1];
      std::cout << sub_match.str() << endl;
      std::stringstream(sub_match.str()) >>  tid;      
    } else {
      cout << "Failed to determine HLT index : " << key1 << endl;
      return 0;
    }
 
    // std::string s = boost::regex_replace(
    // key1,
    // boost::regex("[^0-9]*([0-9]+).*"),
    // std::string("\\1")
    // );

    //std::stringstream(s) >>  tid;
    //cout << "TID " << tid << endl;
    
    if (std::find(triggers_enabled.begin(), triggers_enabled.end(), tid) != triggers_enabled.end()) {
      ptrdiff_t pos = std::distance(triggers_enabled.begin(), find(triggers_enabled.begin(), triggers_enabled.end(), tid));
      cout << "Enabling trigger " << tid << endl;
      it->at("enable") = true;
      std::stringstream prescale;
      prescale << "0x" << std::hex << prescales.at(pos) << std::dec;
      it->at("prescale") = prescale.str();
      cout << it->dump(2) << endl;
    } else {
      it->at("enable") = false;
    }
  }
  //reassign to update the changes
  config.at("ctb").at("HLT").at("trigger") = triggers;
  // for (size_t i = 1; i < 8; i++) {
  //   std::ostringstream key;
  //   key << "HLT_" << i;

  //   if (std::find(triggers_enabled.begin(), triggers_enabled.end(), i) != triggers_enabled.end()) {
  //     triggers.at(key.str()).at("enable") = true;
  //     prescale << "0x" << std::hex << prescales.at(i) << std::dec;
  //     triggers.at(key.str()).at("prescale") = prescale.str();

  //   } else {
  //     triggers.at(key.str()).at("enable") = false;
  //   }
  // }

  cout << "Dumping the new configuration..." << endl;

  newconf = str.replace(conf_begin+1,conf_end-conf_begin-1,config.dump(2));

  std::ofstream newfile("trigger_standard.fcl.new");
  newfile << newconf;
  newfile.close();

  cout << "All done. File 'trigger_standard.fcl.new' contains the new configuration. Please check its contents before deploying..." << endl; 
// Setting prescales


}
catch(json::out_of_range &e) {
  std::cerr << "JSON oor exception : " << e.what() << std::endl;
}

catch(json::exception &e) {
  std::cerr << "JSON exception : " << e.what() << std::endl;
}
catch(std::out_of_range &e ) {
  std::cerr << "Out of range : " << e.what() << std::endl;
}
catch(std::exception &e) {
  std::cerr << "Some other STL exception : " << e.what() << std::endl;
}

  return 0;
}
