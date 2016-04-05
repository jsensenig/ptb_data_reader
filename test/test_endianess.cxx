#include <iostream>
#include <cstdint>
#include <bitset>

struct header {
  typedef uint32_t data_t;
  typedef uint8_t  data_packet_type_t;
  typedef uint32_t short_nova_timestamp_t;

  // The order of the data packet type and the timestamp have been
  // swapped to reflect that it's the MOST significant three bits in
  // the payload header which contain the type. I've also added a
  // 1-bit pad to reflect that the least significant bit is unused.

  // byte[3] : data_packet_type[2-0],ts[27-23]
  // byte[2] : ts[22-15]
  // byte[1] : ts[14-7]
  // byte[0] : ts[6-0],padding

  uint8_t padding : 1;
  short_nova_timestamp_t short_nova_timestamp : 28;
  data_packet_type_t     data_packet_type     : 3;

  static size_t const size_words = sizeof(data_t);
};

// Create a struct that would work properly with the data coming in
// keep in mind that we want to look at the TSUs and BSUs
// BSUs are the msb's
struct CounterPayload {
  typedef uint64_t counter_set_t;
  typedef uint16_t set_size_t;
    counter_set_t tsu_wu     : 10;
    counter_set_t tsu_el     : 10;
    counter_set_t tsu_extra  :  4;    
    counter_set_t tsu_nu     :  6;
    counter_set_t tsu_sl     :  6;
    counter_set_t tsu_nl     :  6;
    counter_set_t tsu_su     :  6;
    counter_set_t bsu_rm     : 16;//end of first counter_set_t==uint64_t
    counter_set_t bsu_cu     : 10;
    counter_set_t bsu_cl     : 13;
    counter_set_t bsu_rl     : 10;
    counter_set_t ts_rollover: 28;
    counter_set_t header     :  3;//end of second counter_set_t==uint64_t
    static set_size_t const num_bits_tsu_wu     = 10;
    static set_size_t const num_bits_tsu_el     = 10;
    static set_size_t const num_bits_tsu_extra  =  4;
    static set_size_t const num_bits_tsu_nu     =  6;
    static set_size_t const num_bits_tsu_sl     =  6;
    static set_size_t const num_bits_tsu_nl     =  6;
    static set_size_t const num_bits_tsu_su     =  6;
    static set_size_t const num_bits_bsu_rm     = 16;
    static set_size_t const num_bits_bsu_cu     = 10;
    static set_size_t const num_bits_bsu_cl     = 13;
    static set_size_t const num_bits_bsu_rl     = 10;
    static set_size_t const num_bits_ts_rollover= 28;
    static set_size_t const num_bits_header     =  3;
};

int main() {

  // The bytes are loaded from the DMA in the reverse order from msB to lsB
  // i.e., data[0] ==> lsB
  // 
  //uint8_t data[4] = {225,235,71,141}; // 11100001 11101011 01000111 10001101
  uint8_t data[4] = {141,71,235,225}; // 

  // -- print it to make sure that it is what I see coming out of the DMA
  std::cout << "Original : ";
  for (uint32_t i =0; i < 4; ++i) {
    std::cout << std::bitset<8>(data[i])<< " ";
   }
  std::cout <<std::endl;

  // Now cast into an integer and print it
  uint32_t *datai = reinterpret_cast<uint32_t*>(data);
  printf("Value is : %u (%08X)\n",datai[0],datai[0]);
  
  // Now use the struct to look at the contents
  // Now check the bitfield:
  struct header *data2 = reinterpret_cast<struct header*>(data);
  uint32_t type = (uint32_t) data2->data_packet_type;
  uint32_t timestamp = (uint32_t) data2->short_nova_timestamp;
  uint32_t pad = (uint32_t) data2->padding;

  std::cout << " Type (should be 7) " << type << std::endl;
  std::cout << " TS (should be 16098246) " << timestamp << std::endl;
  std::cout << " Pad (should be 1)" << pad << std::endl;
  
  return 0;
}

// Let's this some other way

/**
int main() {


  uint8_t *bytes = NULL;
  uint32_t *int_bytes = NULL;
  
  uint32_t data = std::bitset<32>("11100001111010110100011110001101").to_ulong();

  printf("Value is : %u (%08X)\n",data,data);
  std::cout << "Original: " << std::bitset<32>("11100001111010110100011110001101") << std::endl;
  std::cout << "New representation: " << std::bitset<32>(data) << std::endl;
  // -- Now print the individual bytes
  bytes = reinterpret_cast<uint8_t*>(&data);
  
  for (uint32_t i =0; i < sizeof(data); ++i) {
    std::cout << std::bitset<8>(bytes[i])<< " ";
   }
  std::cout <<std::endl;
 
  // Now check the bitfield:
  struct header *data2 = reinterpret_cast<struct header*>(&data);
  uint32_t type = (uint32_t) data2->data_packet_type;
  uint32_t timestamp = (uint32_t) data2->short_nova_timestamp;
  uint32_t pad = (uint32_t) data2->padding;

  std::cout << " Type (should be 7) " << type << std::endl;
  std::cout << " TS (should be 16098246) " << timestamp << std::endl;
  std::cout << " Pad (should be 1)" << pad << std::endl;
  
  return 0;
}

**/
  // Usual output data[31-0],data[63-32],data[95-64],data[127-96]
  // In little endian this means:
  // data[7-0],data[15-8],data[23-16],data[31-24],
  // data[39-32],data[47-40],data[55-48],data[63-56]
  // data[71-64],data[79-72],data[87-80],data[95-88]
  // data[103-96],data[111-104],data[119-112],data[127-120]
          
  // If I reverse completely the byte assignment then I would be getting
  /**
          data[103-96],data[111-104],data[119-112],data[127-120]
          data[71-64],data[79-72],data[87-80],data[95-88]
          data[39-32],data[47-40],data[55-48],data[63-56]
          data[7-0],data[15-8],data[23-16],data[31-24],
          in integers this means:
          data[127-96],data[95-64],data[63-32],data[31-24]
  **/
