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

  return 0;
}
