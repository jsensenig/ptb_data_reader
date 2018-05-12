/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "boardreader.h"

#include "Logger.h"
#include "opexception.h"
#include "PracticalSocket.h"
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

#include <cstring>

#ifdef ARM_SG_DMA
#include "pothos_zynq_dma_driver.h"
#endif

using namespace ptb;

//TODO: Get rid of the exceptions... convert everything into return states
//      and use the respective

board_reader::board_reader() : tcp_port_(0), tcp_host_(""),
    data_socket_(nullptr),
#if defined(ENABLE_FRAG_BLOCKS)
    packet_rollover_(0),fragmented_(false),
#endif
    client_thread_collector_(0),client_thread_transmitter_(0),ready_(false),
    s2mm(nullptr),
    seq_num_(0),
    keep_transmitting_(true),
    keep_collecting_(true),time_rollover_(0),dma_initialized_(false),
    // below this point all these are debugging variables
    error_state_(false),
    num_eth_fragments_(0),
    num_word_counter_(0),num_word_gtrigger_(0),num_word_ltrigger_(0),num_word_feedback_(0),
    num_word_tstamp_(0),bytes_sent_(0)
{



}

board_reader::~board_reader() {
  Log(debug,"Destroying the reader." );
  ready_ = false;
}


void board_reader::get_feedback(bool &error, json &msgs, const bool reset)
{
  error = error_state_.load();
  msgs = feedback_messages_;

  if (reset) {
    error_state_.store(false);
    feedback_messages_.clear();
  }
}


void board_reader::reset_counters() {
  Log(info,"Resetting reader counters");
  num_eth_fragments_ = 0;
  num_word_counter_ = 0;
  num_word_feedback_ = 0;
  num_word_gtrigger_ = 0;
  num_word_ltrigger_ = 0;
  num_word_tstamp_ = 0;
  bytes_sent_ = 0;
}


void board_reader::clear_threads() {
  Log(debug,"Killing the daughter threads.\n");
  // First stop the loops
  keep_collecting_ = false;
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
  keep_transmitting_ = false;
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

  Log(warning,"Shutting down the DMA engine.");
  clean_and_shutdown_dma();

  Log(info,"Resetting existing buffers");
  reset_buffers();

  Log(info,"Data taking stopped.");
}

/// -- This lives in the main thread (same as manager)
/// NOTE: Any state reset should be done here
void board_reader::start_data_taking() {
//  error_state_.store(false);
//  error_messages_.clear();
  if (ready_) {

    init_dma();



    // We are ready to do business. Start reading the DMA into the queue.
    Log(verbose, "==> Creating collector thread\n" );
    keep_collecting_ = true;
    client_thread_collector_ = new std::thread(&board_reader::data_collector,this);
    if (client_thread_collector_ == nullptr) {
      Log(error,"Unable to create collector thread . Failing..." );
      json obj;
      obj["type"] = "error";
      obj["message"] = "Unable to create collector thread";
      feedback_messages_.push_back(obj);
      error_state_.store(true);
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
      error_state_.store(true);
    }


  } else {
    Log(error,"Calling to start data taking but connection is not ready yet." );
    json obj;
    obj["type"] = "error";
    obj["message"] = "Calling to start data taking but data connection is not ready yet";
    feedback_messages_.push_back(obj);
    error_state_.store(true);
  }
}

void board_reader::reset_buffers() {
  Log(warning,"Resetting the software buffers.");
  uint32_t counter = 0;
  //bool has_data = true;
  while(buffer_queue_.pop()) {
    counter++;
  }
  Log(info,"Popped %u entries from the buffer queue.",counter);
}

// Only does one thing and that is killing the data connection.
void board_reader::close_data_connection() {
  bool socket_good = test_socket();
  // only call for delete if the socket is good
  if ((data_socket_ != nullptr) && socket_good) delete data_socket_;
  data_socket_ = nullptr;
  ready_ = false;
}

bool board_reader::test_socket() {
  // -- test if the socket yields something usable
  bool socket_good = true;
  try {
    data_socket_->getForeignPort();
  }
  catch(...) {
    // if an exception is caught here, then the socket is no longer good.
    socket_good = false;
  }
  return socket_good;
}

/// -- This is one of the main calls from the manager.
/// NOTE: Any state reset should the done here
void board_reader::init_data_connection(bool force) {
  // -- reset the state machine

  //-- If a connection exists, assume it is correct and continue to use it
  // FIXME: A ghost connection can exist
  // Try first if the connection is good
  if (data_socket_ != nullptr) {
    bool socket_good = test_socket();

    if (socket_good) {
      if (force) {
        Log(warning,"Destroying existing socket!");
        json obj;
        obj["type"] = "warning";
        obj["message"] = "Stale data socket found. Destroying existing socket!";
        feedback_messages_.push_back(obj);
        delete data_socket_;
        data_socket_ = nullptr;
      } else {
        Log(info,"Reusing existing connection.");
        return;
      }
    }
  }

  // Check if the server data is set up, and compute
  if (tcp_host_ != "" && tcp_port_ != 0) {

    try{
      Log(debug, "Opening socket connection : %s : %hu",tcp_host_.c_str(),tcp_port_ );
      data_socket_ = new TCPSocket(tcp_host_.c_str(),tcp_port_);

      if (data_socket_ == nullptr) {
        Log(error,"Unable to establish the client socket. Failing." );
        json obj;
        obj["type"] = "error";
        obj["message"] = "Unable to establish the client socket";
        feedback_messages_.push_back(obj);
        error_state_.store(true);
        ready_ = false;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    // -- Catch and rethrow the exceptions so that they can be dealt with at higher level.
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      json obj;
      obj["type"] = "error";
      std::string emsg = "Socket exception caught creating data connection : ";
      emsg += e.what();
      obj["message"] =  emsg;
      feedback_messages_.push_back(obj);
      error_state_.store(true);
      ready_ = false;
      if (data_socket_) delete data_socket_;
      data_socket_ = nullptr;
    }
    catch(std::exception &e) {
      ready_ = false;
      if (data_socket_) delete data_socket_;
      data_socket_ = nullptr;
      Log(error,"STD exception caught : %s",e.what() );
      json obj;
      obj["type"] = "error";
      std::string emsg = "STL exception caught creating data connection : ";
      emsg += e.what();
      obj["message"] =  emsg;
      feedback_messages_.push_back(obj);
      error_state_.store(true);
    }
    catch(...) {
      ready_ = false;
      if (data_socket_) delete data_socket_;
      data_socket_ = nullptr;
      Log(error,"Unknown exception caught." );
      json obj;
      obj["type"] = "error";
      obj["message"] = "Unknown exception caught creating data connection";
      feedback_messages_.push_back(obj);
      error_state_.store(true);
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
    json obj;
    obj["type"] = "error";
    obj["message"] = "Calling to start connection without defining connection parameters";
    feedback_messages_.push_back(obj);
    error_state_.store(true);

    ready_ = false;
    if (data_socket_) delete data_socket_;
    data_socket_ = nullptr;
  }

  if (error_state_.load()) {
    Log(error,"Data connection failed to be established. See previous messages.");
  } else {
    Log(debug,"Connection opened successfully");
  }

}


void board_reader::data_collector() {
  Log(info, "Starting data collector\n" );


  static size_t len;
  //static int handle;
  ptb::content::buffer_t dma_buffer;
  std::ostringstream err_msg;
  size_t counter = 0;
  while(keep_collecting_) {
    if (!keep_collecting_) {
      Log(warning,"Received signal to stop acquiring data. Cleaning out...");
      break;
    }
    counter++;
    dma_buffer.handle = pzdud_acquire(s2mm, &len);
    // Deal with the case that there is no data transferred yet
    if (dma_buffer.handle < 0)
    {
      // Failed because there are no complete
      // transactions in RAM yet
      // -- This is not really a failure
      // Could avoid it by implementing a kernel
      // signaling to be caught here
      if (dma_buffer.handle == PZDUD_ERROR_COMPLETE) {
        // -- Set the system to sit for a few usec
        pzdud_wait(s2mm,5);
        continue;
      }
      // Failed because it timed out
      // (meaning that the DMA didn't answer back)
      // This is dangerous and usually means that
      // something crapped out
      if (dma_buffer.handle == PZDUD_ERROR_TIMEOUT) {
        Log(error,"Failed to acquire data with timeout on iteration %u . Returned %i",counter,dma_buffer.handle);
        json obj;
        obj["type"] = "error";
        obj["message"] = "Failed to acquire data with timeout from DMA";
        feedback_messages_.push_back(obj);
      }
      if (dma_buffer.handle == PZDUD_ERROR_CLAIMED) {
        Log(error,"Failed to acquire data due to claimed buffers [iteration %u].",counter);
        json obj;
        obj["type"] = "error";
        obj["message"] = "Failed to acquire data due to claimed DMA buffers";
        feedback_messages_.push_back(obj);
      }
      Log(error,"Failed to acquire data. Returned %i [iteration %u]",dma_buffer.handle,counter);
      err_msg  << "Failed to acquire data. Returned " << dma_buffer.handle;
      json obj;
      obj["type"] = "error";
      obj["message"] = err_msg.str();
      feedback_messages_.push_back(obj);
      error_state_.store(true);
      keep_collecting_ = false;
      break;
    }
    // -- Push the transfer to the queue
    //dma_buffer.handle = handle;
    Log(verbose,"Acquired %u (%u bytes) on %u iterations",dma_buffer.handle,dma_buffer.len,counter);
    dma_buffer.len = len;
    buffer_queue_.push(dma_buffer);
  }
  Log(info,"Stopping the data collection");

}


void board_reader::clean_and_shutdown_dma() {
  if (!dma_initialized_) {
    Log(warning,"Asking to shutdown an uninitialized DMA ");
    return;
  }
  Log(debug,"Halting the S2MM transition stream");
  pzdud_halt(s2mm);
  Log(debug,"Freeing allocated buffers associated with the S2MM transition stream");
  pzdud_free(s2mm);
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
    error_state_.store(true);
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
    error_state_.store(true);

    pzdud_destroy(s2mm);
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
    error_state_.store(true);

    pzdud_free(s2mm);
    pzdud_destroy(s2mm);
    return;
  }
  dma_initialized_ = true;
  Log(info,"DMA engine ready to take data");
}

void board_reader::data_transmitter() {
  Log(debug, "Starting data transmitter\n");

  // Allocate a memory buffer to build eth packets.

  //FIXME: Implement fragmentation
  // this builds an array of 64k ints = 256 kbytes
  // Should correspond to several ethernet packets in worst case scenario
  uint32_t *global_eth_buffer = new uint32_t[eth_buffer_size_u32]();

  // Local pointer that is effectively used to build the eth packet
  uint32_t *eth_buffer = nullptr;
  // pointer that keeps track of the offset within the global buffer
  uint32_t global_eth_pos = 0;

  /** Statistics collection variables
   */
  error_state_ = false;
  feedback_messages_.clear();
  // Debugging information that is passed down in the end of the run
  num_eth_fragments_ = 0;
  num_word_counter_ = 0;
  num_word_gtrigger_ = 0;
  num_word_ltrigger_ = 0;
  num_word_tstamp_ = 0;
  num_word_feedback_ = 0;
  bytes_sent_ = 0;

  // This will keep track on the number of u32 words
  // In the end the size of the buffer will be ipck*sizeof(uint32_t);
  static uint32_t n_u32_words = 0;
  static uint32_t n_bytes_sent = 0;

  // Temporary variables that will end up making part of the eth packet
  //  static uint32_t packet_size = 0;

  // pointer to a new word
  ptb::content::buffer_t dma_buffer;
  seq_num_ = 1;

  // Assign the skelleton packet header
  // This would be nice to go out of the loop but it
  ptb::content::tcp_header eth_header;
  eth_header.word.format_version = (ptb::content::format_version << 4) | ((~ptb::content::format_version) & 0xF);

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  ptb::content::word::word_t *frame;
  while(keep_transmitting_) {

    // Set the local pointer to the place pointer by the global pointer
    eth_buffer = &global_eth_buffer[global_eth_pos];

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
    n_bytes_sent = 0;

    eth_header.word.sequence_id = seq_num_;

    // -- Pop a buffer
    if (!buffer_queue_.pop(dma_buffer)) {
      // -- should some sort of wait be put here?
      // Might hurt since it will require some sort of mutex
      std::this_thread::sleep_for (std::chrono::microseconds(1000));
      continue;
    }

    // -- at this point there is a buffer available
    // Assign the size to the header
    eth_header.word.packet_size = dma_buffer.len & 0xFFFF;
    //Log(debug,"Header : [%X]",eth_header.value);
    std::memcpy(&(eth_buffer[0]),&eth_header,sizeof(eth_header));
    //Log(debug,"Header (xcheck) : [%X]",*(&eth_buffer[0]));
    // -- copy the whole buffer
    std::memcpy(&(eth_buffer[1]),(void*)buff_addr_[dma_buffer.handle],dma_buffer.len);


    // -- loop over the buffer to collect word statistics
    size_t tpos = 0;
    while (tpos < dma_buffer.len) {
       frame = reinterpret_cast<ptb::content::word::word_t*>(buff_addr_[dma_buffer.handle]+tpos);
       switch(frame->word_type) {
         case ptb::content::word::t_fback:
           num_word_feedback_++;
           break;
         case ptb::content::word::t_gt:
           num_word_gtrigger_++;
           break;
         case ptb::content::word::t_lt:
           num_word_ltrigger_++;
           break;
         case ptb::content::word::t_ts:
           num_word_tstamp_++;
           break;
         case ptb::content::word::t_ch:
           num_word_counter_++;
           break;

       }
       // Advance the pointer
       tpos += ptb::content::word::word_t::size_bytes;
    }

    // -- Send the data
    try {
      //Log(debug,"%X",eth_buffer[0]);
      n_bytes_sent = sizeof(eth_header)+dma_buffer.len;
      // -- release the memory buffer

      n_u32_words = n_bytes_sent/sizeof(uint32_t);
      data_socket_->send(eth_buffer,n_bytes_sent);
      //Log(debug,"Releasing buffer %u",dma_buffer.handle);
      pzdud_release(s2mm, dma_buffer.handle, 0);

      global_eth_pos += (n_u32_words+4);
      // add 4 bytes of padding just to make sure that there are no overlaps
      // for occasional small packets troubles could happen.
      if ((global_eth_pos+(3*n_u32_words)) > eth_buffer_size_u32) {
        // reset the pointer to the beginning
        global_eth_pos = 0;
      }

      bytes_sent_ += n_bytes_sent;
      num_eth_fragments_++;
    }
    catch(SocketException &e) {
      Log(error,"Socket exception : %s",e.what() );
      error_state_.store(true);
      std::string err_msg;
      err_msg = "ERROR: Data socket exception [";
      std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
      std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      std::tm now_tm = *std::localtime(&now_c);
      char date[128];
      strftime(date, sizeof(date), "%A %c", &now_tm);
      err_msg += date;
      err_msg += "] : ";
      err_msg += e.what();
      json obj;
      obj["type"] = "error";
      obj["message"] = err_msg;
      feedback_messages_.push_back(obj);
      keep_collecting_ = false;
      keep_transmitting_ = false;
      error_state_.store(true);
    }

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
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // Deallocate the memory
  delete [] global_eth_buffer;
  // Finish the connection
  Log(info,"Closing data socket.");
  close_data_connection();
  Log(info,"Data socket closed.");
}



