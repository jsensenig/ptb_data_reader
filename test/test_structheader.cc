#include <iostream>

typedef struct mystruct {
  typedef uint32_t ts_type;
  typedef uint8_t  word_type;

  ts_type timestamp : 28;
  word_type xeader  : 4;
} mystruct;

int main() {

uint8_t x = 0;
 
 uint32_t word = (0xF << 28 ) | (0xAAAA);
 mystruct *wtp = reinterpret_cast<mystruct*>(&word);
 std::cout << "Word : " << word << " H " << std::hex << word << std::dec
	   << " B " << std::bitset<32>(word) << std::endl;

 std::cout << "Manual : " << std::bitset<4>((word >> 28) & 0xF) 
	   << "  " << std::bitset<28>(word & 0xFFFFFFFF) << std::endl;

 std::cout << "Auto   : " << std::bitset<4>(wtp->xeader) << "  " << std::bitset<28>(wtp->timestamp) << std::endl;

return 0;
}
