//dma_test
// Created on: Aug 5, 2020
//     Author: jon
//


#include "../../src/board_reader_interface.h"
#include "zmq.hpp"
#include "dma_test.h"

extern "C" {
#include <arpa/inet.h>
#include <unistd.h>
#include <inttypes.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

}

#include <cstdint>
#include <bitset>
#include <sstream>
#include <thread>
#include <chrono>
#include <map>
#include <fstream>
#include <cstring>
#include <iostream>

int main() {
	DMATest test;
	test.Start();
}

DMATest::DMATest() :
		rcvr_thread_(0),
		run_(false)
{ }

DMATest::~DMATest()
{
  delete rcvr_thread_;
  rcvr_thread_ = nullptr;
}

void DMATest::Start() {

    run_ = true;

    //DataReaderI brI;
    brI.init_data_connectionI();

    if(!brI.get_readyI()) {
    	brI.set_readyI(true);
    }
    //sock_(ctx_, zmq::socket_type::sub);
    sock_.connect("inproc://test");
    // Subscribe to ALL messages
    sock_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    std::cout << "Starting data taking" << std::endl;

    rcvr_thread_ = new std::thread(&DMATest::data_receiver,this);

    if (!rcvr_thread_) {
    	std::cout << "Error data thread not started!" << std::endl;
    }

    sleep(5);
    std::cout << "Ending data taking" << std::endl;

    // stop_data_taking() does 3 things in the board reader
    // 1. Clears running threads
    // 2. Cleans and shuts down the DMA
    // 3. Resets buffers
    brI.stop_data_takingI();
    run_ = false;
    rcvr_thread_->join();

    std::cout << "Timestamp words received " << brI.get_n_timestampsI() << std::endl;
    // Clear the counters
    brI.reset_countersI();

}

void DMATest::data_receiver() {

  // Loop over the data from the ZMQ connection
  zmq::message_t rmsg;

  while (run_) {
    auto recvd = sock_.recv(rmsg, zmq::recv_flags::none);

	if(true) {
	  std::cout << "Received message size: " << rmsg.size() << std::endl;
	  // Recast data into original type
	  const uint32_t *iptr = rmsg.data<uint32_t>();
	  std::cout << "Received message: " << *iptr << std::endl;
	}
  }
}

