#include <iostream>
#include <cstdlib>


void build_header(void**buffer,uint32_t &seq_num, uint8_t &version) {
  
  void *p = *buffer;

  *(volatile uint32_t*)(p) = (seq_num & 0xFF) << 24 | version;

}

int main() {
  
  uint32_t rollover = 80;
  // Size of header in 32 bit increments
  uint32_t header_size = 
  void* buffer = calloc(rollover+5,sizeof(uint32_t));

  // Block of 32-bits after which the data should be inserted
  uint32_t offset_data = 1; 
  uint32_t seq_num = 0;
  uint8_t version = 0x4;

  for (size_t  i=0; i < 100; ++i) {
    
    // First construct the header
    build_header(&buffer,i,version);
  
    std::cout << " Buffer as of now: " << (uint64_t) (*buffer) << std::endl;
  }


  return 0;

}
