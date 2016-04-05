#include <iostream>
#include <cstring>

int main() {
  int pos = 0;
  int *bla = new int[100]();
  int *ble = nullptr;

  for (int k = 0; k < 100; ++k) {
    std::cout << bla[k] << " ";
  }
  std::cout << std::endl;

 for (int i =0; i < 100;) {
    ble = &bla[i];
    for (int j = 0; j < 10; ++j){
      std::memcpy(&ble[j],&j,4);
      //ble[j] = j;
    }
    i += 10;
  } 
 
  // Now print 10 lines of numbers. 
  // The array should be filled with equal entries
  for (int k = 0; k < 100; ++k) {
    std::cout << bla[k] << " ";
  }
  std::cout << std::endl;
  
  return 0;
}
