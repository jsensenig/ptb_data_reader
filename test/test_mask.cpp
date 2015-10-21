#include <stdint.h>
#include <iostream>
#include <bitset>

using std::cout;
using std::endl;
using std::hex;
using std::dec;

int main() {

  uint32_t x = 0xFFFFFFFF;
  cout << "Original value: " << x << " " << hex << x << " " << dec << std::bitset<32>(x) << endl;
  
  x  = 0x0 << 16;// | x;
  cout << "New value: " << x << " " << hex << x << " " << dec << std::bitset<32>(x) << endl;

  uint32_t n = 16;
  x = 0xFFFFFFFF;
  uint32_t status = 0x0;
  x ^= (-status ^ x) & (1 << n);
  cout << "New value: " << x << " " << hex << x << " " << dec << std::bitset<32>(x) << endl;


  bool stat = false;
  uint32_t tmp =  x^((-(stat?1:0) ^ x) & ( 1 << n));

  cout << "New value (CC): " << tmp << " " << hex << tmp << " " << dec << std::bitset<32>(tmp) << endl;

  // -- Now try to do the same with a full range
  uint32_t reg = 0xFFFFFFFF;
  uint32_t pos = 15;
  uint32_t val = 0xAA << pos;
  uint32_t len = 9;

  uint32_t mask = (((uint32_t)1 << len)-1) << pos;
  reg = (reg & ~mask) | (val & mask);

  cout << " Mask (CC): " << mask << " " << hex << mask << " " << dec << std::bitset<32>(mask) << endl;
  cout << "~mask (CC): " << ~mask << " " << hex << ~mask << " " << dec << std::bitset<32>(~mask) << endl;
  cout << "Range (CC): " << reg << " " << hex << reg << " " << dec << std::bitset<32>(reg) << endl;



  return 0;
}
