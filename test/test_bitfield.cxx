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

typedef uint16_t trigger_code_t;

typedef struct global_trigger_t {
    //uint32_t word : 32;
    trigger_code_t trigger_id : 16;
    uint32_t padding    : 16;
} global_trigger_t;

typedef union global_trigger {
    global_trigger_t word;
    uint32_t       value;
} global_trigger;

const read_status::state_t  read_status::states[2] = {0x1,0x2};

int main () {
  read_status stat;// = {0x0,0x2};
  stat.idx = 0x0;
  std::cout << "size of struct " << sizeof(stat) << std::endl;
  for (uint32_t i = 0; i < 100; ++i) {
    uint32_t val = stat.get_next_read();
    std::cout << "Next state : " << static_cast<uint32_t>(val) << " " << std::hex << static_cast<uint32_t>(val) << std::dec << " " << std::bitset<2>(val) << std::endl;
  }

global_trigger gt;

std::cout << "Size of union " << sizeof(gt) << std::endl;
std::cout << "Size of struct " << sizeof(gt.word) << std::endl;
//std::cout << "Size of id " << sizeof(gt.word.trigger_id) << std::endl;

for (size_t i = 0; i < 50; i++) {
  gt.word.trigger_id = i;
  std::cout << "base : " << i << " value " << gt.value << /*" word " << static_cast<uint32_t>(gt.word) <<*/ " trigger " << gt.word.trigger_id << std::endl;
}


  return 0;
  
}
