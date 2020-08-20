/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "boardreader.h"

#include "Logger.h"
#include "util.h"
#include "ptb_registers.h"

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

#ifdef ARM_SG_DMA
#include "pothos_zynq_dma_driver.h"
#endif

using namespace ptb;

//TODO: Get rid of the exceptions... convert everything into return states
//      and use the respective

board_reader::board_reader() :
    client_thread_collector_(0),
    client_thread_transmitter_(0),
    ready_(false),
	fake_data_(false),
    s2mm(nullptr),
    dma_initialized_(false),
    error_state_(false),
	sock_(ctx_, zmq::socket_type::pub),
    reset_dma_engine_(false),
    keep_transmitting_(true),
    keep_collecting_(true),
    error_dma_timeout_(false),
    error_dma_claimed_(false),
    // below this point all these are debugging variables
    num_eth_fragments_(0),
    num_word_chstatus_(0),num_word_gtrigger_(0),num_word_ltrigger_(0),num_word_feedback_(0),
    num_word_tstamp_(0),bytes_sent_(0)
{

  // this does nothing but gets rid of a compilation warning
  (void)ptb::content::format_version;

}

board_reader::~board_reader() {
  Log(debug,"Destroying the reader." );
  ready_ = false;
}


void board_reader::get_feedback(bool &error, json &msgs, const bool reset)
{
  error = error_state_.load(std::memory_order_acquire);
  msgs = feedback_messages_;

  if (reset) {
    error_state_.store(false,std::memory_order_relaxed);
    error_dma_timeout_.store(false,std::memory_order_relaxed);
    error_dma_claimed_.store(false,std::memory_order_relaxed);
    feedback_messages_.clear();
  }
}


void board_reader::reset_counters() {
  Log(info,"Resetting reader counters");
  num_eth_fragments_ = 0;
  num_word_chstatus_ = 0;
  num_word_feedback_ = 0;
  num_word_gtrigger_ = 0;
  num_word_ltrigger_ = 0;
  num_word_tstamp_ = 0;
  bytes_sent_ = 0;
}


void board_reader::clear_threads() {
  Log(debug,"Killing the daughter threads.\n");
  // First stop the loops
  keep_collecting_.store(false,std::memory_order_relaxed);
  std::this_thread::sleep_for (std::chrono::milliseconds(100));
  Log(info,"Joining collector thread.");
  // Kill the collector thread first
  // Ideally we would prefer to join the thread
  if (client_thread_collector_ != nullptr) {
    client_thread_collector_->join();
    Log(debug,"Collector thread just joined");
    delete client_thread_collector_;
    client_thread_collector_ = nullptr;
  }
  // Kill the transmitter thread.

  // Occasionally there seems to be memory corruption between these two steps
  Log(info,"Telling transmitter thread to stop.");
  keep_transmitting_.store(false,std::memory_order_relaxed);
  std::this_thread::sleep_for (std::chrono::milliseconds(200));
  // -- Apparently this is a bit of a problem since the transmitting thread never leaves cleanly
  Log(info,"Killing transmitter thread.");
  if (client_thread_transmitter_ != nullptr) {
    client_thread_transmitter_->join();
    Log(debug,"Transmitter thread just joined");
    delete client_thread_transmitter_;
    client_thread_transmitter_ = nullptr;
  }
  Log(info,"Threads cleared");
}


void board_reader::stop_data_taking() {
  // This stops the data taking completely.

  // Clean up the threads
  Log(info,"Clearing the threads");
  clear_threads();
  // Prepare everything so that the PTB gets ready again

  Log(info,"Shutting down the DMA engine.");
  clean_and_shutdown_dma();

  Log(info,"Resetting existing buffers");
  reset_buffers();

  // Close zmq socket
  sock_.close();

  Log(info,"Data taking stopped.");
}

/// -- This lives in the main thread (same as manager)
/// NOTE: Any state reset should be done here
void board_reader::start_data_taking() {

  if (ready_) {
	  if(!fake_data_) {
		init_dma();

		// We are ready to do business. Start reading the DMA into the queue.
		Log(verbose, "==> Creating collector thread\n" );
		keep_collecting_.store(true,std::memory_order_relaxed);
		client_thread_collector_ = new std::thread(&board_reader::data_collector,this);
		if (client_thread_collector_ == nullptr) {
		  Log(error,"Unable to create collector thread . Failing..." );
		  json obj;
		  obj["type"] = "error";
		  obj["message"] = "Unable to create collector thread";
		  feedback_messages_.push_back(obj);
		  error_state_.store(true,std::memory_order_relaxed);
		}
		// We are ready to do business. Start reading the DMA into the queue.
		Log(verbose, "==> Creating transmitter\n" );
		keep_transmitting_ = true;
		client_thread_transmitter_ = new std::thread(&board_reader::data_transmitter,this);
		if (client_thread_transmitter_ == nullptr) {
		  Log(error,"Unable to create transmitter thread . Failing..." );
		  json obj;
		  obj["type"] = "error";
		  obj["message"] = "Unable to create transmitter thread";
		  feedback_messages_.push_back(obj);
		  error_state_.store(true,std::memory_order_relaxed);
		}
	  } else {
		// Only sends an incrementing integer over zmq for testing
		keep_transmitting_ = true;
		client_thread_transmitter_ = new std::thread(&board_reader::fake_data_sender,this);
		Log(info,"Started fake data transmission");
	  }
  } else {
	Log(error,"Calling to start data taking but connection is not ready yet" );
	json obj;
	obj["type"] = "error";
	obj["message"] = "Calling to start data taking but data connection is not ready yet";
	feedback_messages_.push_back(obj);
	error_state_.store(true,std::memory_order_relaxed);
  }
}

void board_reader::reset_buffers() {
  Log(warning,"Resetting the software buffers.");
  uint32_t counter = 0;

  while(!buffer_queue_.isEmpty()) {
	buffer_queue_.popFront();
    counter++;
  }
  Log(info,"Popped %u entries from the buffer queue.",counter);
}

/// -- This is one of the main calls from the manager.
/// NOTE: Any state reset should the done here
void board_reader::init_data_connection() {
  // -- reset the state machine

  //-- If a connection exists, assume it is correct and continue to use it
  // NOTE: A ghost connection can exist
  // Try first if the connection is good
	//ZMQ connection should be made in thread where it is used
//  try {
//    sock_.bind("inproc://test");
//  }
//  catch(...) {
//	  ready_ = false;
//  }
//  ready_ = true;
}

void board_reader::data_collector() {
  Log(info, "Starting data collector");

  static size_t len;
  static ptb::content::buffer_t dma_buffer;
  size_t counter = 0;
  while(keep_collecting_.load(std::memory_order_acquire)) {
    if (!keep_collecting_.load(std::memory_order_acquire)) {
      Log(warning,"Received signal to stop acquiring data. Cleaning out...");
      break;
    }
    counter++;
    dma_buffer.handle = pzdud_acquire(s2mm, &len);
    // Deal with the case that there is no data transferred yet
    if (dma_buffer.handle < 0)
    {
      // Failed because there are no complete transactions in RAM yet
      // -- This is not really a failure
      // Could avoid it by implementing a kernel signaling to be caught here
      if (dma_buffer.handle == PZDUD_ERROR_COMPLETE) {
        // -- Set the system to sit for a few usec
        // -- This should be configurable, but for now it is alright
        pzdud_wait(s2mm,5);
        continue;
      }
      // Failed because it timed out (meaning that the DMA didn't answer back)
      // This is dangerous and usually means that something crapped out
      if (dma_buffer.handle == PZDUD_ERROR_TIMEOUT) {
        Log(error,"Failed to acquire data with timeout on iteration %u . Returned %i",counter,dma_buffer.handle);
        error_dma_timeout_.store(true,std::memory_order_relaxed);
        json obj;
        obj["type"] = "error";
        obj["message"] = "Failed to acquire data with timeout from DMA";
        feedback_messages_.push_back(obj);
      }
      if (dma_buffer.handle == PZDUD_ERROR_CLAIMED) {
        Log(error,"Failed to acquire data due to claimed buffers [iteration %u].",counter);
        error_dma_claimed_.store(true,std::memory_order_relaxed);
        json obj;
        obj["type"] = "error";
        obj["message"] = "Failed to acquire data due to claimed DMA buffers";
        feedback_messages_.push_back(obj);
        //Create feedback word and inject into data stream
        // -- not sure how to grab the most recent TS
        // -- for consistency these words should be big endian
        //uint64_t feedback_up = (code<<?) + (code<<?);
        //uint64_t feedback_down = TS;
        //buffer_queue_.push(feedback_up,8);
        //buffer_queue_.push(feedback_down,8);
      }
      Log(error,"Failed to acquire data. Returned %i [iteration %u]",dma_buffer.handle,counter);
      std::ostringstream err_msg;
      err_msg  << "Failed to acquire data. Returned " << dma_buffer.handle
               << " [iteration " << counter << "]" ;
      json obj;
      obj["type"] = "error";
      obj["message"] = err_msg.str();
      feedback_messages_.push_back(obj);
      error_state_.store(true,std::memory_order_relaxed);
      keep_collecting_ = false;
      break;
    }
    // -- Push the transfer to the queue

    //Log(verbose,"Acquired %u (%u bytes) on %u iterations",dma_buffer.handle,dma_buffer.len,counter);
    dma_buffer.len = len;
    buffer_queue_.write(dma_buffer);
  }
  Log(info,"Stopping the data collection");
}


void board_reader::clean_and_shutdown_dma() {
  if (!dma_initialized_) {
    Log(warning,"Asking to shutdown an uninitialized DMA");
    json obj;
    obj["type"] = "warning";
    obj["message"] = "Received DMA shutdown request on uninitialized DMA";
    feedback_messages_.push_back(obj);
    return;
  }
  Log(debug,"Halting the S2MM transition stream");
  static int ret = 0;
  ret = pzdud_halt(s2mm);
  if (ret != PZDUD_OK) {
    Log(warning,"Failed to halt the DMA. Forcing a reset");
    json obj;
    obj["type"] = "warning";
    obj["message"] = "Failed to halt the DMA. Forcing a reset";
    feedback_messages_.push_back(obj);

  }
  Log(debug,"Freeing allocated buffers associated with the S2MM transition stream");
  pzdud_free(s2mm);

  if (reset_dma_engine_.load(std::memory_order_acquire)) {
    pzdud_reset(s2mm);
    reset_dma_engine_.store(false,std::memory_order_relaxed);
  }

  Log(debug,"Destroying the handle over the S2MM channel of the DMA engine");
  pzdud_destroy(s2mm);

  dma_initialized_ = false;
  Log(info,"DMA engine shut down");
}



void board_reader::init_dma() {
  if (dma_initialized_) {
    Log(warning,"DMA already initialized. This call should not happen.");
    json obj;
    obj["type"] = "warning";
    obj["message"] = "DMA already initialized. This will likely lead to trouble!";
    feedback_messages_.push_back(obj);
    return;
  }
  int retstat = 0;
  Log(debug,"Initializing DMA engine");
  // -- NFB: Here 0 is the engine number
  s2mm = pzdud_create(0, PZDUD_S2MM);

  if (s2mm == nullptr) {
    Log(error,"Failed to establish connection to device");
    json obj;
    obj["type"] = "error";
    obj["message"] = "Failed to establish connection to DMA device";
    feedback_messages_.push_back(obj);
    error_state_.store(true, std::memory_order_relaxed);
    return;
  }
  Log(debug,"Allocating DMA buffers");
  retstat = pzdud_alloc(s2mm, num_buffs_, buff_size_);
  Log(debug,"Received answer %d\n", retstat);
  if (retstat != PZDUD_OK) {
    Log(error,"Failed to allocate DMA buffers");
    json obj;
    obj["type"] = "error";
    obj["message"] = "Failed to allocate DMA buffers";
    feedback_messages_.push_back(obj);
    error_state_.store(true, std::memory_order_relaxed);

    retstat = pzdud_destroy(s2mm);
    if (retstat != PZDUD_OK) {
      Log(error,"Failed to destroy DMA engine");
      json obj;
      obj["type"] = "error";
      obj["message"] = "Failed to destroy DMA engine";
      feedback_messages_.push_back(obj);
    }
    return;
  }
  Log(debug,"Initializing buffer table");
  for (size_t i = 0; i < num_buffs_; i++) {
    buff_addr_[i] = (uint32_t)pzdud_addr(s2mm,i);
    //Log(verbose,"Buffer %u : 0x%p",i,buff_addr_[i]);
  }
  Log(debug,"Initializing the DMA engine...");
  retstat = pzdud_init(s2mm, true);
  Log(debug,"Received answer %d\n", retstat);
  if (retstat != PZDUD_OK) {
    Log(error,"Failed to initialize DMA engine");
    json obj;
    obj["type"] = "error";
    obj["message"] = "Failed to initialize DMA engine";
    feedback_messages_.push_back(obj);
    error_state_.store(true, std::memory_order_relaxed);

    retstat = pzdud_free(s2mm);
    if (retstat != PZDUD_OK) {
      Log(error,"Failed to free DMA buffers");
      json obj;
      obj["type"] = "error";
      obj["message"] = "Failed to destroy DMA engine";
      feedback_messages_.push_back(obj);
      reset_dma_engine_.store(true, std::memory_order_relaxed);
    }

    retstat = pzdud_destroy(s2mm);
    if (retstat != PZDUD_OK) {
      Log(error,"Failed to destroy DMA engine");
      json obj;
      obj["type"] = "error";
      obj["message"] = "Failed to destroy DMA engine";
      feedback_messages_.push_back(obj);
      reset_dma_engine_.store(true, std::memory_order_relaxed);
    }

    return;
  }
  dma_initialized_ = true;
  Log(info,"DMA engine ready to take data");
}

void board_reader::data_transmitter() {
  Log(debug, "Starting data transmitter\n");

  //FIXME: Implement fragmentation
  // this builds an array of 128k ints = 512 kbytes
  // Should correspond to several ethernet packets in worst case scenario
  //uint32_t *global_eth_buffer = new uint32_t[eth_buffer_size_u32_]();
  static uint32_t global_eth_buffer[eth_buffer_size_u32_];
  static eth_packet debug_eth_buffer;

  // define constant feedback words for these purposes
  static ptb::content::word::feedback_t feedback_dma;
  feedback_dma.source = 0x3;
  feedback_dma.word_type = 0x0;
  feedback_dma.payload1 = 0x0;
  feedback_dma.payload2 = 0x0;

  static uint64_t last_timestamp = 0;

  // Local pointer that is effectively used to build the eth packet
  // It is just an auxiliary moving pointer
  uint32_t *eth_buffer = nullptr;
  // pointer that keeps track of the offset within the global buffer
  //FIXME: Is this needed?
  uint32_t global_eth_pos = 0;

  /// Statistics collection variables
  /// FIXME: SHould the counters be reset here?
  // Debugging information that is passed down in the end of the run
  num_eth_fragments_ = 0;
  num_word_chstatus_ = 0;
  num_word_gtrigger_ = 0;
  num_word_ltrigger_ = 0;
  num_word_tstamp_ = 0;
  num_word_feedback_ = 0;
  bytes_sent_ = 0;

  // This will keep track on the number of u32 words
  // In the end the size of the buffer will be ipck*sizeof(uint32_t);
  static uint32_t n_u32_words = 0;
  static uint32_t seq_num_ = 0;
  static bool first_message = true;
  // Temporary variables that will end up making part of the eth packet
  //  static uint32_t packet_size = 0;
  last_timestamp = 0;
  // pointer to a new word
  ptb::content::buffer_t dma_buffer;


  /// -- Initialize the sequence number to 1 (first packet)
  seq_num_ = 1;
  first_message = true;

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  ptb::content::word::word_t *frame;
  ptb::content::word::feedback_t*fbk;

  //Open up the ZMQ conection
  try {
    sock_.bind("inproc://test");
  }
  catch(...) {
	  ready_ = false;
  }

  while(keep_transmitting_.load(std::memory_order_acquire)) {

    // Set the local pointer to the place pointed by the global pointer

    zmq::message_t zmq_send((uint32_t)dma_buffer.len);

    /**
    !!!! Start by not doing any size modifications into the memory
    simply wrap the whole thing with a header and ship.
    This will allow to do zero-copy transfer

    !!! Caveat: we cannot allow the full buffer to fill. Add a provision for that...

    FW decides when the packet is ready to be sent.
    The TS word marks that a packet has to be sent.

    2 conditions to be careful of:

    1. If time time based rollover is reached.
    2. If the size based rollover is reached. (not implemented)
     **/

    /// -- Start by generating a header

    n_u32_words = 0;

    // FIXME: Finish implementing feedback word injection here
    //if ()


    // -- Pop a buffer
    // FIXME replace with folly queue
    //if (!buffer_queue_.pop(dma_buffer)) {
    if(buffer_queue_.isEmpty()) {
      // -- should some sort of wait be put here?
      // Might hurt since it will require some sort of mutex
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      continue;
    }
    // Copies data from buffer, but since it is only a handle
    // to data in memory not much overhead is created
    buffer_queue_.read(dma_buffer);

    /// -- NFB : Using memcpy as I am sure that there is no memory
    ///          overlap, therefore can use the faster version


    // -- If there are DMA feedbacks, inject them here
    if (error_state_.load(std::memory_order_acquire)) {
      feedback_dma.timestamp = last_timestamp;
      if (error_dma_timeout_.load(std::memory_order_acquire)) feedback_dma.code = 0x1;
      else if (error_dma_claimed_.load(std::memory_order_acquire)) feedback_dma.code = 0x2;
      else feedback_dma.code = 0x3;
      Log(warning,
          "DMA error caught. Sneaking in feedback word : \nTS : [%" PRIu64 "]\nCode : [%X]\nSource : [%X]\nPayload : [%" PRIX64 "]\nType : [%X]",
          feedback_dma.timestamp,feedback_dma.code,feedback_dma.source,feedback_dma.get_payload(),(uint32_t)feedback_dma.word_type);
    }

    // -- copy the whole buffer into the zmq message
    std::memcpy(zmq_send.data(), (void*)buff_addr_[dma_buffer.handle], dma_buffer.len);

    // -- loop over the buffer to collect word statistics
    //FIXME: Might want to do this at the FPGA level
    size_t tpos = 0;
    while (tpos < dma_buffer.len) {
       frame = reinterpret_cast<ptb::content::word::word_t*>(buff_addr_[dma_buffer.handle]+tpos);
       switch(frame->word_type) {
         case ptb::content::word::t_fback:
           fbk = reinterpret_cast<ptb::content::word::feedback_t*>(frame);
           Log(warning,
               "Feedback word caught : \nTS : [%" PRIu64 "]\nCode : [%X]\nSource : [%X]\nPayload : [%" PRIX64 "]\nType : [%X]\n",
               fbk->timestamp,fbk->source,fbk->code,fbk->get_payload(),(uint32_t)fbk->word_type);
           num_word_feedback_++;
           break;
         case ptb::content::word::t_gt:
           num_word_gtrigger_++;
           break;
         case ptb::content::word::t_lt:
           num_word_ltrigger_++;
           break;
         case ptb::content::word::t_ts:
           last_timestamp = frame->timestamp;
           num_word_tstamp_++;
           break;
         case ptb::content::word::t_ch:
           num_word_chstatus_++;
           break;
         default:
           // -- A dangerous situation was found...the header has an unexpected type
           if (first_message) {
             first_message = false;
             std::ostringstream msg;
             msg << "Found a word header with an unexpected type : "
                 << std::bitset<3>(frame->word_type);
             Log(error,"%s",msg.str().c_str());
             json obj;
             obj["type"]="error";
             obj["message"]=msg.str();
             feedback_messages_.push_back(obj);
           }
           break;
       }
       // Advance the pointer
       tpos += ptb::content::word::word_t::size_bytes;
    }

    // Send/publish the data
    auto sent = sock_.send(zmq_send, zmq::send_flags::none);

    n_u32_words = dma_buffer.len/sizeof(uint32_t);
      //Log(debug,"Releasing buffer %u",dma_buffer.handle);
      // -- Do a copy of the data
      // -- release the memory buffer
    pzdud_release(s2mm, dma_buffer.handle, 0);

    bytes_sent_ += dma_buffer.len;
    num_eth_fragments_++;

      //TODO implement error handling

    // increment the sequence number
    seq_num_++;

  } // -- while(keep_transmitting_)
  // Exited the  run loop. Return.

  Log(debug,"Left transmission loop.");
  if (error_state_) {
    Log(warning,"Transmission loop exited with error state.");
  }

  // wait for a few moments before deallocating the memory so that the kernel does not go ballistic
  // in case it hans't yet committed all the buffers
  // FIXME: Keep an eye for problems by commenting this out.
  // this assumes that the send operation gains ownership over the memory of the data
  // being sent. This is probably tru, but should be checked
  //  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Finish the connection
  Log(info,"Closing data socket.");
  Log(info,"Data socket closed.");

}

void board_reader::fake_data_sender() {

	// Set up socket to send data
	sock_.bind("tcp://127.0.0.1:3855");

	// Create and fill buffer with data
	void* buf_ptr = malloc(sizeof(uint32_t));
	const void* cptr = buf_ptr;

	// Recast so we can fill with data
	uint32_t* buff = static_cast<uint32_t*>(buf_ptr);

	uint32_t iter = 0;
	Log(info,"Starting transmission loop");

    while(keep_transmitting_.load(std::memory_order_acquire)) {
      zmq::message_t send_msg(sizeof(uint32_t));
	  buff[0] = iter;
	  std::memcpy (send_msg.data(), buf_ptr, sizeof(uint32_t));

	  // Send buffer over ZMQ (implicit send_flag = none)
	  auto res = sock_.send(send_msg, zmq::send_flags::none);

      // Unsuccessful send attempts return -1
	  if (res.value() < 0) {
		Log(info,"Error message not sent!");
	  } else {
	    bytes_sent_ += res.value();
	  }

	  sleep(1);
	  iter++;
	}
    // Send one last message to allow the receiver thread to join
	auto res = sock_.send(zmq::message_t(sizeof(uint32_t)), zmq::send_flags::none);
}


