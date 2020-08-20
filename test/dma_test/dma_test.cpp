//dma_test
// Created on: Aug 5, 2020
//     Author: jon
// Test/example of how to control and receive data from the
// boardreader.
//


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

DMATest::DMATest():
		rcvr_thread_(0),
		run_(false),
	    sock_(ctx_, zmq::socket_type::sub)
{ }

DMATest::~DMATest()
{
  delete rcvr_thread_;
  rcvr_thread_ = nullptr;
}

void DMATest::Start() {

    run_ = true;

    //DataReaderI2 brI;
    brI.init_data_connection();

    if(!brI.get_ready()) {
    	brI.set_ready(true);
    }

    // Set to fake data mode
    brI.set_fake_data(true);

    // Start the run
    brI.start_data_taking();

    std::cout << "Starting data taking" << std::endl;

    rcvr_thread_ = new std::thread(&DMATest::data_receiver,this);

    if (!rcvr_thread_) {
    	std::cout << "Error data thread not started!" << std::endl;
    }

    sleep(15);  // Receive data for a few seconds
    std::cout << "Ending data taking" << std::endl;

    // Don't actually shutdown the receiver first as you might lose data
    run_ = false;
    rcvr_thread_->join();

    // stop_data_taking() does 3 things in the board reader
    // 1. Clears running threads
    // 2. Cleans and shuts down the DMA
    // 3. Resets buffers
    brI.stop_data_taking();
    // Grab some of the run statistics
    std::cout << "Timestamp words received " << brI.get_n_timestamps() << std::endl;
    std::cout << "Bytes sent " << brI.get_sent_bytes() << std::endl;

    // Clear the counters
    brI.reset_counters();

}

void DMATest::data_receiver() {

  // Connect to the ZMQ socket
  sock_.connect("tcp://127.0.0.1:3855");
  // Subscribe to ALL messages
  sock_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  // Loop over the data from the ZMQ connection
  zmq::message_t rmsg;

  while (run_) {
    auto recvd = sock_.recv(rmsg, zmq::recv_flags::none);

	// Recast data into original type
	const uint32_t *iptr = rmsg.data<uint32_t>();
	std::cout << "Received message value: " << *iptr << " size:" << rmsg.size() << std::endl;
  }
}

