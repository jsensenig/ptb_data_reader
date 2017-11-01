#include <iostream>
#include <sstream>
#include <cstdint>

int main() {
  std::stringstream strVal;
  strVal << "1230221";
  uint32_t val1;
  strVal >> val1;
  strVal.clear();
  strVal.str("");
  strVal << "1442";
  uint32_t val2;
  strVal >> val2;
  strVal.clear();
  strVal.str("");

  strVal << "12";
  uint32_t val3;
  strVal >> val3;
  strVal.clear();
  strVal.str("");

  strVal << std::hex << "0x1111111111111111";
  uint64_t val4;
  strVal >> val4;
  
  std::cout << val1 << " " << val2 << " " << val3 << std::endl;
  std::cout << val4 << " " << std::endl;
  
  return 0;
}
