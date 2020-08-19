//zmq_test
// Created on: Aug 5, 2020
//     Author: jon
//

#include <zmq.hpp>

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
#include <atomic>
#include <fstream>
#include <cstring>
#include <iostream>
#include <string>
#include <memory>

using namespace std;

// Set up the ZMQ sockets
const std::string addr_ = "inproc://test";
zmq::context_t ctx_ (1);
zmq::socket_t send_sock_(ctx_, ZMQ_PUB);
zmq::socket_t recv_sock_(ctx_, ZMQ_SUB);

const size_t size = 5;
std::atomic<bool> run_ (false);

void data_sender();
void data_receiver();


void data_sender() {

	// Set up socket to send data
	send_sock_.bind(addr_);


	// Create and fill buffer with data
	void* buf_ptr = malloc(sizeof(uint32_t));
	const void* cptr = buf_ptr;

	// Recast so we can fill with data
	uint32_t* buff = static_cast<uint32_t*>(buf_ptr);

	for (size_t i = 0; i < 1; i++) {
		buff[i] = (uint32_t)(65001);
	}

	zmq::message_t send_msg(sizeof(uint32_t));

	while (run_) {

	  std::memcpy (send_msg.data(), buf_ptr, sizeof(uint32_t));

	// Send buffer over ZMQ (implicit send_flag = none)
	  auto res = send_sock_.send(send_msg, zmq::send_flags::none);

	  if(res) cout << "Sent message size " << send_msg.size() << endl;
	  sleep(1);
	}
    // Send one last message to allow the receiver thread to join
	auto res = send_sock_.send(send_msg, zmq::send_flags::none);
}

void data_receiver() {

  sleep(1);
  // Connect to port
  recv_sock_.connect(addr_);
  // Subscribe to ALL messages
  recv_sock_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  zmq::message_t rmsg;

  while (run_) {
	// Implicit recv flag = none
    auto recvd = recv_sock_.recv(rmsg, zmq::recv_flags::none);

	if(true) {
	  cout << "Received message size: " << rmsg.size() << endl;
	  // Recast data into original type
	  const uint32_t *iptr = rmsg.data<uint32_t>();
	  cout << "Received message: " << *iptr << endl;
	}
  }
}

int main() {

  cout << "Starting test " << endl;

  // Set run flag true
  run_ = true;

  // Start a thread for both sender and receiver
  std::thread *sendr_thread = new std::thread(&data_sender);
  std::thread *rcvr_thread = new std::thread(&data_receiver);

  cout << "Sockets opened and threads started " << endl;

  sleep(20);
  cout << "Ending test " << endl;
  run_ = false;

  rcvr_thread->join();
  sendr_thread->join();
  cout << "Joined threads" << endl;

  delete sendr_thread;
  delete rcvr_thread;

}

