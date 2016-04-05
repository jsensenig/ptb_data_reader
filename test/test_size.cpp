#include <iostream>
#include <sstream>
#include <bitset>

  struct TimestampPayload {
    typedef uint64_t timestamp_t;
    typedef uint16_t data_size_t;
    timestamp_t nova_timestamp : 64;

    static data_size_t const num_bits_timestamp = 64;
    static size_t const size_words = sizeof(uint64_t);

    static size_t const ptb_offset = sizeof(uint32_t);

  };

int main () {

  std::cout << sizeof(TimestampPayload) << std::endl;

  return 0;
}
