#include <iostream>
#include <cstdint>

using namespace std;
int main () {

void *addr = new uint32_t(123213);
cout << " Addr " << addr << " " << static_cast<uint32_t*>(addr) << " " << *static_cast<uint32_t*>(addr) << endl;
uint32_t val = 1;
return 0;
}
