#include <iostream>

int main() {

uint8_t x = 0;

for (uint32_t i = 0; i < 1000; ++i) {
	x+= 1;
	std::cout << (uint32_t)x << " " << std::hex << (uint32_t)x << std::dec << " " << std::bitset<8>(x) << std::endl;
	}
return 0;
}
