#include <iostream>
#include <fstream>
#include <cinttypes>
#include "json.hpp"

using json = nlohmann::json;

int main() {

std::ifstream i;
try {
  i.open("ptb_config.json");
  json j;
  std::cout << "Parsing document" << std::endl;
  i >> j;

  std::cout << "JSON object filled..." << std::endl;
  //  json jtest = j.at("test");
  json ctbconf = j.at("ctb");

  json sockets = ctbconf.at("sockets");
  json receiver = sockets.at("receiver");
  std::string rhost = receiver.at("host").get<std::string>();
  uint16_t rport = receiver.at("port").get<uint16_t>();
  uint32_t rollover = receiver.at("rollover").get<uint32_t>();

  // Grab the information about the calibration stream
  json calib = sockets.at("calibration");
  bool calib_en = calib.at("enable").get<bool>();
  if (calib_en) {
  	std::cout << "Setting up the calibration stream." << std::endl;
  	std::string host = calib.at("host").get<std::string>();
  	uint16_t port = calib.at("port").get<uint16_t>();
  	std::cout << "Expecting calibration listener at " << host << ":" << port << std::endl;
  }


  uint32_t coinc = ctbconf.at("coincidence").get<uint32_t>();
  std::cout << "Coincidence gate set to " << coinc << std::endl;

  bool random = ctbconf.at("random_trigger").at("enable").get<bool>();
  if (random) {
  	std::cout << "==> Enabling random trigger with a period of " << ctbconf.at("random_trigger").at("period").get<uint32_t>() << std::endl;
  }

  // -- Now loop over each system and 
  json subsys = ctbconf.at("subsystems");


// special iterator member functions for objects
for (json::iterator it = subsys.begin(); it != subsys.end(); ++it) {
  	std::cout << it.key() << " : " << it.value() << "\n";
  	if (it.key() == "ssp") {
  		json ssp = it.value();
  		uint32_t ssp_mask = std::strtol(ssp.at("mask").get<std::string>().c_str(), NULL, 16);
  		std::cout << "SSP channel mask : " << ssp.at("mask").get<std::string>() << " " << std::hex << ssp_mask << std::dec << " " << ssp_mask << std::endl;
  		std::vector<uint32_t> dacs = ssp.at("dac_thresholds");

  		std::cout << "Setting " << dacs.size() << " dac thresholds to:" << std::endl;
  		for (size_t i = 0; i < dacs.size(); i++) {
  			std::cout << dacs.at(i) << " ";
  		}
  		std::cout << std::endl;
  	}
}
 
  i.close();
//   std::cout << "Listening port : " << lport << std::endl;
//   std::cout << "Receiver : " << rhost << ":" << rport << std::endl;
//   std::cout << "Monitoring stream : " << monconn << std::endl;

//   // -- Now grab the channel masks
//   json masks = j.at("setup").at("channel_mask");
//   for (json::iterator it = masks.begin(); it != masks.end(); ++it) {
//     std::cout << "Mask " << it.key() << " : " << it.value() << " (" << std::strtol(it.value().get<std::string>().c_str(), NULL, 16) << ")" << std::endl;
//   }

//   json jdacs = j.at("setup").at("ssp");
//   std::vector<int> dacs = jdacs["dac_values"];

//   if (dacs.size()) {
//     std::cout << "Dacs : "; 
//     for (auto &element : dacs) {
//       std::cout << element << ", ";
//     }
//     std::cout << std::endl;
//   }
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
i.close();
  // -- Now attempt to parse the expected setup

std::string teststr = "{ \"something\":[1,2,3],\"a1\":3.14,\"b1\":\"ssddsd\",\"c1\":{ \"something\": \"else\" },\"d1\":\"1233\"}";
json obj = json::parse(teststr);
for (json::iterator it = obj.begin(); it != obj.end(); ++it) {
    std::cout << it.key() << " : " << it.value() << "\n";
}


try{
  // -- find if there is a "command" key
  std::cout << "Command : " << obj.count("command") << std::endl;
  // -- find if there is a "something" key
  std::cout << "Something : " << obj.count("something") << std::endl;
  obj.erase("something");
  for (json::iterator it = obj.begin(); it != obj.end(); ++it) {
      std::cout << it.key() << " : " << it.value() << "\n";
  }

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
