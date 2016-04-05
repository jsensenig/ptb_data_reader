#include <iostream>
#include <bitset>

struct read_status {
  typedef uint8_t state_t;
  typedef uint8_t index_t;
  index_t idx : 1;
  state_t state :2;
  static const state_t  states[2]; 
  uint32_t get_next_read() {
    return static_cast<uint32_t>(states[++idx]);
  }
};

const read_status::state_t  read_status::states[2] = {0x1,0x2};

int main () {
  read_status stat;// = {0x0,0x2};
  stat.idx = 0x0;
  std::cout << "size of struct " << sizeof(stat) << std::endl;
  for (uint32_t i = 0; i < 100; ++i) {
    uint32_t val = stat.get_next_read();
    std::cout << "Next state : " << static_cast<uint32_t>(val) << " " << std::hex << static_cast<uint32_t>(val) << std::dec << " " << std::bitset<2>(val) << std::endl;
  }
  return 0;
  
}
