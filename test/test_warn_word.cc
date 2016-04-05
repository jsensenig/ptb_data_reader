#include <iostream>
#include <bitset>
#include <cstdint>

int main() {
  std::bitset<32> bit_word("00000100000000000000000000000000");
  uint32_t data = bit_word.to_ulong();
  std::cout << "Number " << data << " " << std::hex << data << std::endl;
  return 0;
}
