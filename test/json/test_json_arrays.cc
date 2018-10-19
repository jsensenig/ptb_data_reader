#include <iostream>
#include <fstream>
#include <cinttypes>
#include "json.hpp"

using json = nlohmann::json;
using std::cout;
using std::endl;

int main() {





  json j;
  json k;


try {
  cout << "Appending 1....going to work" << endl;
  json tmp;
  tmp["type"]="debug";
  tmp["message"] = "Some message";
  j.push_back(tmp);

  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;
  

  // push another
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

try {
  cout << " Appending another...will work" << endl;
  json tmp;
  tmp["type"]="info";
  tmp["message"] = "Some other message";
  j.push_back(tmp);

  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;

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

try {
  cout << " Using insert... will crash" << endl;

  if (!j.empty()) {
    k.insert(std::end(k),j.begin(),j.end());
  } else {
    cout << "Thinks J is empty: " << endl;
  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;
  }

  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;

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


try {
  cout << "Data is being prepended. Should work" << endl;
json tmp;
  tmp["type"]="debug";
  tmp["message"] = "Some debug message";
  k.push_back(tmp);


  if (!j.empty()) {
    k.insert(std::end(k),j.begin(),j.end());
  } else {
    cout << "Thinks J is empty: " << endl;
  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;
  }

  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;

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


try {
  cout << "Setting type to array first. Should work" << endl;
  std::vector<std::string> jj;
  k = json(jj);

  if (!j.empty()) {
    k.insert(std::end(k),j.begin(),j.end());
  } else {
    cout << "Thinks J is empty: " << endl;
  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;
  }

  std::cout << "j :" << j.dump(2) << std::endl;
  std::cout << "k :" << k.dump(2) << std::endl;

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
