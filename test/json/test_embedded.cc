/*
 * test_embedded.cc
 *
 *  Created on: May 7, 2018
 *      Author: nbarros
 */




#include "json.hpp"
#include <iostream>
#include <vector>
#include <string>

using json = nlohmann::json;

int main() {

  json doc = json::parse("{\"feedback\": [{\"type\":\"info\",\"message\":\"This is a message\"},"
      "{\"type\":\"error\",\"message\":\"This is an error message\"},"
      "{\"type\":\"info\",\"message\":\"This is a message\"},"
      "{\"type\":\"statistics\",\"num_words\":20, \"num_triggers\":1234}]}");

  std::vector<json> entries = doc.at("feedback");

  size_t pos;
  for (size_t i = 0; i < entries.size(); i++)
  {
    if (entries[i].at("type").get<std::string>() == std::string("info"))
    {
      printf("INFO Message : %s\n",(entries[i].at("message").get<std::string>()).c_str());
    }
    if (entries[i].at("type").get<std::string>() == std::string("error"))
    {
      printf("ERROR Message : %s\n",(entries[i].at("message").get<std::string>()).c_str());
    }
    if (entries[i].at("type").get<std::string>() == std::string("statistics"))
    {
      pos = i;
      for (json::iterator it = entries[i].begin(); it != entries[i].end(); ++it) {
        std::cout << it.key() << " : " << it.value() << std::endl;
      }
//      printf("Statistics:\n\n # words : %d\n # triggers : %d\n\n",
//          entries[i].at("num_words").get<unsigned int>(),entries[i].at("num_triggers").get<unsigned int>());
    }

  }

  json test;
  test["type"] = "warning";
  test["message"] = "This is a warning message that was appended.";
  doc["feedback"].push_back(test);
  std::cout << doc.dump(2) << std::endl;

//  std::vector<json> entries2 = doc.at("feedback");
  for (json::iterator it = doc.at("feedback").begin(); it != doc.at("feedback").end(); ++it) {
    std::cout << it->dump(2) << std::endl;
    //std::cout << it.key() << " : " << it.value() << std::endl;
  }


  json obj;
  json tmp1;
  tmp1["type"] = "warning";
  tmp1["message"] = "Some message 1";
  obj.push_back(tmp1);
  json tmp2;
  tmp2["type"] = "info";
  tmp2["message"] = "Some message 2";
  obj.push_back(tmp2);

  std::cout << obj.dump() << std::endl;

  json gobj;
  gobj["feedback"] = obj;
  std::cout << gobj.dump() << std::endl;

  gobj["feedback"].insert(std::end(gobj["feedback"]),obj.begin(),obj.end());
  std::cout << gobj.dump() << std::endl;

  // Now print it again


  // now try to update the
  return 0;

}
