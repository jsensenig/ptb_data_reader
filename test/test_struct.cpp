#include <iostream>
#include <cstdint>

typedef struct mys {
 void*address;
 uint32_t value() { return *(volatile uint32_t*)address;}

 uint32_t &val() {return *(static_cast<uint32_t*>(address));}
 volatile uint32_t&val2() {return *(volatile uint32_t*)address;}
 
} mys;

int main() {
mys bla;
bla.address = malloc(sizeof(int));
*(volatile uint32_t*)bla.address = 1234;

std::cout << "Address " << std::hex << &(bla.address) << " " << bla.address << std::dec << " " << &(bla.address) << " " << bla.address << " Value orig " << *(volatile uint32_t*)bla.address << " method " << bla.value() << std::endl;
bla.val2() = 5678;
std::cout << "Address " << std::hex << &(bla.address) << " " << bla.address << std::dec << " " << &(bla.address) << " " << bla.address << " Value orig " << *(volatile uint32_t*)bla.address << " method " << bla.value() << std::endl;

std::cout << " " << bla.value() << " " << bla.val2() << std::endl;

bla.val2() = 0x0;
std::cout << "Address " << &(bla.address) << " " << bla.address << " Value orig " << *(volatile uint32_t*)bla.address << " method " << bla.value()  << " " << bla.val2() << std::endl;

return 0;
}
