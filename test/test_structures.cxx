#include <iostream>
#include <bitset>
#include <cstdint>

    struct Word_warning {
      typedef uint32_t data_t;
      typedef uint16_t data_size_t;

      typedef uint8_t  warning_type_t;
      typedef uint8_t  data_packet_type_t;
      typedef uint32_t short_nova_timestamp_t;

      // The order of the data packet type and the timestamp have been
      // swapped to reflect that it's the MOST significant three bits in
      // the payload header which contain the type. I've also added a
      // 1-bit pad to reflect that the least significant bit is unused.

      uint32_t padding : 24;
      warning_type_t warning_type : 5;
      data_packet_type_t     data_packet_type     : 3;

      static size_t const size_words = sizeof(data_t);
      static data_size_t const num_bits_padding     = 24;
      static data_size_t const num_bits_warning  = 5;
      static data_size_t const num_bits_packet_type = 3;
    };

// I would like to have this payload with the same 
// 13 bytes that the word is transformed into
struct CounterPayload {
  typedef uint64_t counter_set_t;

  typedef uint16_t data_size_t;

  
  // -- Must be careful to follow the right order
  // from lsb to msb it is
  // -- TSU mappings
  counter_set_t tsu_wu     : 10;
  counter_set_t tsu_el     : 10;
  counter_set_t tsu_extra  :  4;
  counter_set_t tsu_nu     :  6;
  counter_set_t tsu_sl     :  6;
  counter_set_t tsu_nl     :  6;
  counter_set_t tsu_su     :  6;
  // -- BSU mappings
  counter_set_t bsu_rm     : 16;//end of first counter_set_t==uint64_t

  counter_set_t bsu_cu     : 10;
  counter_set_t bsu_cl1    : 6;
  counter_set_t extra      : 1;
  counter_set_t bsu_cl2    : 7;
  counter_set_t bsu_rl     : 10;
  // Just ignore the rest of the word
  counter_set_t padding    : 30;

  static data_size_t const num_bits_tsu_wu = 10;
  static data_size_t const num_bits_tsu_el     = 10;
  static data_size_t const num_bits_tsu_extra  =  4;
  static data_size_t const num_bits_tsu_nu     =  6;
  // static data_size_t const num_bits_tsu_sl     =  6;
  // static data_size_t const num_bits_tsu_nl     =  6;
  // static data_size_t const num_bits_tsu_su     =  6;
  // static data_size_t const num_bits_bsu_rm     = 16;
  // static data_size_t const num_bits_bsu_cu     = 10;
  // static data_size_t const num_bits_bsu_cl1    = 6;
  // static data_size_t const num_bits_bsu_extra = 1;
  // static data_size_t const num_bits_bsu_cl2    = 7;
  // static data_size_t const num_bits_bsu_rl     = 10;
  //  static data_size_t const num_bits_padding   = 30;
};

    struct TimestampPayload {
        typedef uint64_t timestamp_t;
        typedef uint16_t timestamp_bits_t;
        timestamp_t nova_timestamp : 64;

        static timestamp_bits_t const num_bits_timestamp = 64;
    };


int main() {

  const int size = 13;
  uint8_t data_raw[size] = {0x18, 0xAA, 0x70, 0xFF ,0x00,0x00,0x00 ,0x00 ,0x4C ,0x55 ,0x38 ,0x55 ,0x00};//,0x00,0x00,0x00};
  std::cout << "Orginal data:" << std::endl;
  // First print the bit stream to make sure that I am getting what I thought
  for (uint32_t i = 0; i < size;++i) {
    std::cout << std::bitset<8>(data_raw[i]) << " ";
  }
  std::cout << std::endl;
  
  // Size of the structure:
  std::cout << "The size of the CounterPayload is: " << sizeof(CounterPayload) << std::endl;
  
  uint8_t *data = reinterpret_cast<uint8_t*>(data_raw);
  
  // Repeat:
  // First print the bit stream to make sure that I am getting what I thought
  for (uint32_t i = 0; i < size;++i) {
    std::cout << std::bitset<8>(data[i]) << " ";
  }
  std::cout << std::endl;
  
  // -- Cast the structure
  CounterPayload *counter_data = reinterpret_cast<CounterPayload*>(data);
  
  // Start printing information:
  if (counter_data->tsu_wu != 0) {
    std::cout << "There's some data in the TSU WU" << std::endl;
    // std::cout << std::bitset<counter_data->num_bits_tsu_wu>(counter_data->tsu_wu) << std::endl;
  }

  // Test grabbing a timestamp to make sure that it is working properly
  TimestampPayload *ts = reinterpret_cast<TimestampPayload*>(data);
  
  std::cout << "The payload is : " << ts->nova_timestamp << std::endl;
  std::cout << "Bits " << std::bitset<64>(ts->nova_timestamp) << std::endl;

  Word_warning *warn = reinterpret_cast<Word_warning*>(data);
  std::cout << "The size of the payload is " << sizeof(Word_warning) << std::endl;
  std::cout << "The warning type is " << std::bitset<5>(warn->warning_type) << std::endl;
  return 0;

}
