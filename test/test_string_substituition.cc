/*
 * test_string.cc
 *
 *  Created on: Feb 7, 2018
 *      Author: nbarros
 */




#include <iostream>
#include <string>

int main() {

  std::string msg = "{ \"something\":[1,2,3],\"a1\":3.14,\"b1\":\"ssddsd\",\"c1\":{ something else },\"d1\":\"1233\"} {\"x1\":[]}";
  int b = 0;
  bool f = true;
  size_t pb,pe;
  std::string sub_msg;
  for (size_t pos = 0; pos < msg.size(); pos++) {
    if (msg.at(pos) == '{') {
      if (f){
        f = false;
        pb = pos;
      }
       b++;
    }
    if (msg.at(pos) == '}') {
      b--;
      if (b == 0) {
        pe = pos;
        sub_msg = msg.substr(pb,pe-pb+1);
        msg = msg.substr(pe+1);
        std::cout << "Content : " << sub_msg << std::endl;
        std::cout << "Remainder : " << msg << std::endl;
        pos = 0;
        pb = pos;
        f = true;
      }
    }
  }
  std::cout << "Remainer at end : "<< msg << std::endl;


  return 0;
}
