/*
 * test_bitshift.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: nbarros
 */

#include <cstdint>
#include <iostream>
#include <sstream>
#include <bitset>
#include <cstring>

void display_bits(void* memstart, size_t nbytes, std::string sourcename = "") {

  std::stringstream bitstr;
  bitstr << "The " << nbytes << "-byte chunk of memory beginning at " << static_cast<void*>(memstart) << " is : ";

  // -- NFB : Nov-20-2015
  // reversed the order of the bits when printing, so that the msb remain on the left
  for(unsigned int i = 0; i < nbytes; i++) {
    bitstr << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
  }

  std::cout << "Field [" << bitstr.str() << "]" << std::endl;
//  mf::LogInfo(sourcename.c_str()) << bitstr.str();
}

int main() {

  uint32_t val[4];

  val[0] = (0x7 << 29) | (0x0FFFFFE << 1) | 0x1 ;
  val[1] = 0x55555555;
  val[2] = 0x55555555;
  val[3] = 0x55555555;

  uint8_t *vval = reinterpret_cast<uint8_t*>(val);
  std::cout << "Original word: " << std::endl;
  display_bits(val,16);
  std::cout << "Reinterpreted word:" << std::endl;
  display_bits(vval,16);

  uint32_t *rval = reinterpret_cast<uint32_t*>(vval);
  std::cout << "re-recasted word: " << std::endl;
  display_bits(rval,16);

  // Try using memcpy to reorder things properly
  uint8_t*data = new uint8_t(4*sizeof(uint32_t));
  uint8_t*data2 = new uint8_t(4*sizeof(uint32_t));

  //std::memcpy(data,val)
  for (uint32_t i = 0; i < 4; ++i) {
    std::memcpy(&data[i*4],&val[i],4);
    uint32_t bla = htonl(val[i]);
    std::memcpy(&data2[i*4],&bla,4);
  }
  std::cout << "re-recasted word: " << std::endl;
  display_bits(data,16);
  std::cout << "re-recasted word: " << std::endl;
  display_bits(data2,16);


  //
  printf("Original: %08X %08X %08X %08X\n",val[0],val[1],val[2],val[3]);
  printf("Secondary: %08X %08X %08X %08X\n",reinterpret_cast<uint32_t*>(&(data2[0]))[0],data2[4],data2[8],data2[12]);


//  // Now reassign the whole thing and check it still works
//  uint32_t newv[5];
//  newv[0] = val[3];
//  for (size_t k = 0; k < 3; ++k) {
//    // k: 0 , k+1 : 1, k+2 = 2, 4-(k+1): 3
//    // k: 1 , k+1 : 2, k+2 = 3, 4-(k+1): 2
//    // k: 2 , k+1 : 3, k+2 = 4, 4-(k+1): 1
//    // k: 3 , k+1 : 4, k+2 = 5, 4-(k+1): 0
//    // -- The pattern is to pick up the lsb from previous word and the rest from the follow-up
//    // except for the last one
//    //          eth_buffer[ipck+k] = frame[3-(k+1)];
//
//    /**
//     newv[1] = (val[3] & 0x1) << 31 | ((val[2] & 0xFFFFFFFE) >> 1);
//     newv[2] = (val[2] & 0x1) << 31 | ((val[1] & 0xFFFFFFFE) >> 1);
//     newv[3] = (val[1] & 0x1) << 31 | ((val[0] & 0xFFFFFFFE) >> 1);
//     newv[4] = (val[0] & 0x1) << 31 | 0x0;
//
//     */
//    newv[k+1] = ((val[4-(k+1)] & 0x1) << 31) | ((val[4-(k+2)] & 0xFFFFFFFE) >> 1);
//  }
//  newv[4] = ((val[0] & 0x1) << 31) | 0x0;
//
//  std::cout << "New word: " << std::endl;
//  display_bits(newv,20);
//  for (uint32_t i = 0; i < 5; ++i) {
//    std::cout << std::bitset<32>(newv[i]) << std::endl;
//  }
//  //std::cout << "Bitset : " << std::bitset<160>((unsigned char*)newv,20) << std::endl;
//
//
//  uint8_t *field = reinterpret_cast<uint8_t*>(newv);
//
//
//  std::cout << "Recast :" << std::endl;
//    for (uint32_t i = 0; i < 20; ++i) {
//    std::cout << std::bitset<8>(field[i]) << " ";
//  }
//  std::cout << std::endl;
  return 0;
}
