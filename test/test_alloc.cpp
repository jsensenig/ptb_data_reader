#include <iostream>
#include <queue>
extern "C" {
#include <stdint.h>
};


int main() {
  std::queue<uint32_t*> queue;
  uint32_t* frame = NULL;
  uint32_t* packet = new uint32_t[40];
  for (uint32_t i = 0; i < 10; ++i) {
    frame = new uint32_t[4];
    for (uint32_t j = 0; j < 4; ++j) {
      frame[j] = (j+1)*i;
      printf("%u ",frame[j]);
    }
    printf("\n");
    queue.push(frame);
  }
  std::cout << "Set the queue. Printing." << std::endl;
  // for (uint32_t i = 0; i < 10; ++i) {
  //   for (uint32_t j = 0; j < 4; ++j) {
  //     printf("%u ",)frame[j] = j*i;
  //   }
  // }
  uint32_t x = 0;
  for (uint32_t i = 0; i < 10; ++i) {
    frame = queue.front();
    queue.pop();
    for (uint32_t j = 0; j < 4; ++j) {
      printf("%u ",frame[j]);
      packet[x] = frame[j];
      x++;
    }
    delete[] frame;
    printf("\n");
  }

  for (  uint32_t x = 0, y = 0; x < 40; ++x,++y) {
    printf("%u ",packet[x]);
    if (y == 3) {
      printf("\n");
      y = -1;
    }
  }
  delete [] packet;

  
  return 0;
}
