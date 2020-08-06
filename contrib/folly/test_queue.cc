#include <iostream>
#include <vector>
#include <thread>
#include "ProducerConsumerQueue.h"

  namespace tq {

    struct tpset {
      int count;
      int detid;
    };

    void producer();
    void consumerptr();
    void consumercopy();
    folly::ProducerConsumerQueue<tpset> queue_{100000};
    bool stop_flag;
  }

   int main() {
     
     std::cout << "Beginning test." << std::endl;

     tq::stop_flag = false;

     std::thread prod = std::thread(&tq::producer);
     std::thread cons = std::thread(&tq::consumerptr);

     prod.join();
     cons.join();

     std::cout << "End test." << std::endl;

   	 return 0;
   }
    
  // Producer function
   void tq::producer() {

     tpset send;
     send.detid = 0x4;
     std::cout << "Begin filling queue" << std::endl;

     for(int i=0; i<1000; i++) {
       send.count = i;
       if (!queue_.isFull()) queue_.write(send); 
       std::this_thread::sleep_for(std::chrono::microseconds(1000));
     }
     std::this_thread::sleep_for(std::chrono::microseconds(100000));
     stop_flag = true;
     return;
	}

  // Consumer function, uses data in place
  void tq::consumerptr() {

    tpset *recv = nullptr;

    std::cout << "Beginning read loop" << std::endl;

    while (!stop_flag) {
      do {
        recv = queue_.frontPtr();
        std::cout << recv << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      } while(!recv && !stop_flag);
      if (!queue_.isEmpty()) queue_.popFront();
      std::cout << "Queue value " << recv->count << "  " << recv->detid << std::endl;
    }
     return;
  }

  // Consumer function, copies the data
  void tq::consumercopy() {
                                                                    
    tpset recv;
                                                                    
    std::cout << "Beginning read loop" << std::endl;
                                                                    
    while (!stop_flag) {
      while(!queue_.read(recv)){
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      std::cout << "Queue value " << recv.count << std::endl;
    }
    return;
  }
