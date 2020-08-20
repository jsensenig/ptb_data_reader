/*
 * interface.hpp
 *
 *  Created on: Aug 19, 2020
 *      Author: jon
 *
 *  Wrapper for the board_reader class.
 */

#include "boardreader.h"
#include <cstdint>
#include "json.hpp"

using json = nlohmann::json;


class DataReaderI {
  public:

	 // Initialize the ZMQ connections
	 void init_data_connection()  { br_.init_data_connection(); }

	 // Clear all threads, this is called by "stop_data_taking" so
	 // shouldn't be necessary except in an unusual case
	 void clear_threads()         { br_.clear_threads(); }

	 // Shutdown threads, DMA, empty buffers and close ZMQ socket
	 void stop_data_taking()      { br_.stop_data_taking(); }

	 // Start the data threads and initialize/start DMA engine.
	 // "init_data_connection" should be called  before this.
	 void start_data_taking()     { br_.start_data_taking(); }

	 // Clears the buffers. Called in the "stop_data_taking" process
	 // so only necessary in a special case.
	 void reset_buffers()         { br_.reset_buffers(); }

	 // Reset the word counters. Be sure to read them before calling this.
	 void reset_counters()        { br_.reset_counters(); }

	 // Query the status of the initialization. Should be called as
	 // a check before starting data taking.
	 bool get_ready()         { return br_.get_ready(); }

	 // Externally set ready flag if needed.
	 void set_ready(bool rdy) { br_.set_ready( rdy ); }

	 // Set to true to stream fake data over the ZMQ socket for testing. Default is false.
	 void set_fake_data(bool fake_data) { br_.set_fake_data( fake_data ); }

	 // Query the counter for the number of fragments sent during the run.
	 uint32_t get_n_sent_frags()  { return br_.get_n_sent_frags(); }

	 // Query the counter for the number of status words were received during the run
	 uint32_t get_n_status()      { return br_.get_n_status(); }

	 // Query the counter for the number of global triggers received during the run.
	 uint32_t get_n_gtriggers()   { return br_.get_n_gtriggers(); }

	 // Query the counter for the number of low level triggers received during the run.
	 uint32_t get_n_ltriggers()   { return br_.get_n_ltriggers(); }

	 // Query the counter for the number of feedback words received during the run.
	 uint32_t get_n_feedback()    { return br_.get_n_feedback(); }

	 // Query the counter for the number of timestamp words received during the run.
	 uint32_t get_n_timestamps()  { return br_.get_n_timestamps(); }

	 // Query the counter for the number of bytes sent during the run.
	 uint32_t get_sent_bytes()    { return br_.get_sent_bytes(); }

	 // Query messages generated during running in json format.
	 void get_feedback(bool &error, json&msgs,const bool reset) {
		                   br_.get_feedback(error, msgs, reset); }

  protected :

  private :

	ptb::board_reader br_;

};
