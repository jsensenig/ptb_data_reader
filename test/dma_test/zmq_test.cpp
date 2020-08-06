//zmq_test
// Created on: Aug 5, 2020
//     Author: jon
//

#include "zmq.hpp"

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

using namespace std;

// Set up the ZMQ sockets
zmq::context_t ctx_;
zmq::socket_t send_sock_(ctx_, zmq::socket_type::sub);
zmq::socket_t recv_sock_(ctx_, zmq::socket_type::sub);
const std::string addr_ = "inproc://5678";
const size_t size = 1024;

bool run_ = false;

void data_sender();
void data_receiver();
//int main();

void data_sender() {

	// Set up socket to send data
	//send_sock_.bind(addr_);

	// Create and fill buffer with data
	void* buf_ptr = malloc(size*sizeof(uint32_t));
	const void* cptr = buf_ptr;
	// Recast so we can fill with data
	uint32_t* buff = static_cast<uint32_t*>(buf_ptr);

	for (size_t i = 0; i < size; i++) {
		buff[i] = (uint32_t)i;
	}
	cout << "Allocated and filled send buffer" << endl;
	// Basic construction
	zmq::const_buffer cbuf = zmq::buffer(cptr, size);
	cout << "Created ZMQ buffer send" << endl;
	sleep(1);
	// Send buffer over ZMQ
	auto res = send_sock_.send(cbuf, zmq::send_flags::none);
	if(res) {
      cout << "Sent bytes" << endl;
	}
}

void data_receiver() {
  // Connect to port
  //recv_sock_.connect(addr_);

  // Create receiving buffer
  void* buf_ptr = malloc(size*sizeof(uint32_t));
  cout << "Allocated buffer recv" << endl;

  zmq::mutable_buffer recv_mbuf = zmq::buffer(buf_ptr, size);
  cout << "Created ZMQ buffer recv" << endl;
  while (run_) {
	auto recv_bytes = recv_sock_.recv(recv_mbuf, zmq::recv_flags::none);
	if (recv_bytes) {
      cout << "Bytes received " << endl;
	}
  }
}

int main() {

  cout << "Starting test " << endl;
  // Create receiving buffer
  void* buf_ptr = malloc(size);
  cout << "Allocated buffer recv" << endl;

  zmq::mutable_buffer recv_mbuf = zmq::buffer(buf_ptr, size);
  cout << "ZMQ buffer main" << endl;
  // Set run flag true
  run_ = true;
  send_sock_.bind(addr_);
  recv_sock_.connect(addr_);

  // Start a thread for both sender and receiver
  std::thread *sendr_thread = new std::thread(&data_sender);
  std::thread *rcvr_thread = new std::thread(&data_receiver);

  cout << "Sockets opened and threads started " << endl;

  sleep(20);
  cout << "Ending test " << endl;
  run_ = false;

  sendr_thread->join();
  rcvr_thread->join();

}

