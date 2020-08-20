/*
 * dma_test.h
 *
 *  Created on: Aug 5, 2020
 *      Author: jon
 */

#ifndef TEST_DMA_TEST_DMA_TEST_H_
#define TEST_DMA_TEST_DMA_TEST_H_

#include "zmq.hpp"
#include <thread>

#include "../../src/interface.hpp"

using json = nlohmann::json;

class DMATest {
  public:

	DMATest();
	  virtual ~DMATest();

	void Start();
	void data_receiver();

	DataReaderI brI;
	//ptb::board_reader brI;

  protected :

	zmq::context_t ctx_;
	zmq::socket_t sock_;
	bool run_;

	std::thread *rcvr_thread_;

  private :

};

#endif /* TEST_DMA_TEST_DMA_TEST_H_ */
