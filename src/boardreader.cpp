/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "boardreader.h"

#include "Logger.h"
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

board_reader::board_reader() :
    tcp_port_(0), tcp_host_(""),
    data_socket_(nullptr),
    client_thread_collector_(0),
    client_thread_transmitter_(0),
    ready_(false),
    s2mm(nullptr),
    dma_initialized_(false),
    error_state_(false),

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

  Log(info,"Data taking stopped.");
}

/// -- This lives in the main thread (same as manager)
/// NOTE: Any state reset should be done here
void board_reader::start_data_taking() {

  if (ready_) {

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
  // NOTE: A ghost connection can exist
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
    // -- Catch and convert the exceptions into meaningful returnable messages
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      json obj;
      obj["type"] = "error";
      std::string emsg = "Socket exception caught creating data connection : ";
      emsg += e.what();
      obj["message"] =  emsg;
      feedback_messages_.push_back(obj);
      error_state_.store(true,std::memory_order_relaxed);
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
      error_state_.store(true,std::memory_order_relaxed);
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
      error_state_.store(true,std::memory_order_relaxed);
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
    json obj;
    obj["type"] = "error";
    obj["message"] = "Calling to start connection without defining connection parameters";
    feedback_messages_.push_back(obj);
    error_state_.store(true,std::memory_order_relaxed);

    ready_ = false;
    if (data_socket_) delete data_socket_;
    data_socket_ = nullptr;
  }

  if (error_state_.load(std::memory_order_acquire)) {
    Log(error,"Data connection failed to be established. See previous messages.");
  } else {
    Log(debug,"Connection opened successfully");
  }

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
    buffer_queue_.push(dma_buffer);
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
  // this builds an array of 64k ints = 256 kbytes
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
  static uint32_t n_bytes_sent = 0;
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
  /// Assign the skeleton packet header
  /// This would be nice to go out of the loop but it
  ptb::content::tcp_header eth_header;
  eth_header.word.format_version = (ptb::content::format_version << 4) | ((~ptb::content::format_version) & 0xF);

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  ptb::content::word::word_t *frame;
  ptb::content::word::feedback_t*fbk;
  while(keep_transmitting_.load(std::memory_order_acquire)) {

    // Set the local pointer to the place pointed by the global pointer
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

    // FIXME: Finish implementing feedback word injection here
    //if ()


    // -- Pop a buffer
    if (!buffer_queue_.pop(dma_buffer)) {
      // -- should some sort of wait be put here?
      // Might hurt since it will require some sort of mutex
      std::this_thread::sleep_for (std::chrono::microseconds(10));
      continue;
    }

    // -- at this point there is a buffer available
    // Assign the size to the header
    eth_header.word.packet_size = dma_buffer.len & 0xFFFF;

    /// -- NFB : Using memcpy as I am sure that there is no memory
    ///          overlap, therefore can use the faster version


    size_t wpos = 1;
    // -- If there are DMA feedbacks, inject them here
    if (error_state_.load(std::memory_order_acquire)) {
      feedback_dma.timestamp = last_timestamp;
      if (error_dma_timeout_.load(std::memory_order_acquire)) feedback_dma.code = 0x1;
      else if (error_dma_claimed_.load(std::memory_order_acquire)) feedback_dma.code = 0x2;
      else feedback_dma.code = 0x3;
      Log(warning,
          "DMA error caught. Sneaking in feedback word : \nTS : [%" PRIu64 "]\nCode : [%X]\nSource : [%X]\nPayload : [%" PRIX64 "]\nType : [%X]",
          feedback_dma.timestamp,feedback_dma.code,feedback_dma.source,feedback_dma.get_payload(),(uint32_t)feedback_dma.word_type);
      //Log(warning,"Caught a DMA error. Sneaking in a feedback word");

      std::memcpy(&(eth_buffer[1]),(void*)&feedback_dma,sizeof(feedback_dma));
      wpos = 5;
      num_word_feedback_++;
      eth_header.word.packet_size = (dma_buffer.len + sizeof(feedback_dma)) & 0xFFFF;

    }

    // -- Now copy the header. The reason it is done afterwards is because we
    // want to first sneak in the feedback word from the DMA error
    std::memcpy(&(eth_buffer[0]),&eth_header,sizeof(eth_header));

    // -- copy the whole buffer
    std::memcpy(&(eth_buffer[wpos]),(void*)buff_addr_[dma_buffer.handle],dma_buffer.len);

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

    // -- Send the data
    try {
      //Log(debug,"%X",eth_buffer[0]);
      n_bytes_sent = sizeof(eth_header)+dma_buffer.len + (wpos==5)?16:0;
      n_u32_words = n_bytes_sent/sizeof(uint32_t);

      data_socket_->send(eth_buffer,n_bytes_sent);

      //Log(debug,"Releasing buffer %u",dma_buffer.handle);
      // -- Do a copy of the data
      // -- release the memory buffer
      pzdud_release(s2mm, dma_buffer.handle, 0);

      // -- NFB -- Copy the sent buffer to local cache
      // FIXME Remove this once the crash on BR is debugged
      debug_eth_buffer.nbytes = n_bytes_sent;
      debug_eth_buffer.nentries = n_u32_words;
      std::memcpy(&(debug_eth_buffer.data[0]),&eth_buffer,n_bytes_sent);

      global_eth_pos += (n_u32_words+4);
      // add 4 bytes of padding just to make sure that there are no overlaps
      // for occasional small packets troubles could happen.
      if ((global_eth_pos+(4*n_u32_words)) > eth_buffer_size_u32_) {
        // reset the pointer to the beginning
        global_eth_pos = 0;
      }

      bytes_sent_ += n_bytes_sent;
      num_eth_fragments_++;
    }
    catch(SocketException &e) {
      Log(error,"Socket exception : %s",e.what() );
      error_state_.store(true, std::memory_order_relaxed);
      std::string err_msg;
      err_msg = "Data socket exception [";
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
      keep_collecting_.store(false,std::memory_order_relaxed);
      keep_transmitting_.store(false,std::memory_order_relaxed);
      // -- dump the previous eth package to disk
      //FIXME Remove this once the issue with the BR is sorted out
      char fname[128];
      strftime(fname, sizeof(fname), "ctb_eth_dump_%Y%m%d_%H%M%S.txt", &now_tm);

      std::ofstream dfout(fname);
      for (size_t i = 0; i < debug_eth_buffer.nentries; i++) {
        if (i==0) dfout << std::hex << debug_eth_buffer.data[i] ;
        dfout << debug_eth_buffer.data[i];
        if (!(i%4)) dfout << std::dec << "\n";
      }
      dfout.close();
    }
    catch(...) {
      Log(error,"Unknown exception caught sending data");
      error_state_.store(true, std::memory_order_relaxed);
      std::string err_msg;
      err_msg = "Unknown exception [";
      std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
      std::time_t now_c = std::chrono::system_clock::to_time_t(now);
      std::tm now_tm = *std::localtime(&now_c);
      char date[128];
      strftime(date, sizeof(date), "%A %c", &now_tm);
      err_msg += date;
      err_msg += "]";
      json obj;
      obj["type"] = "error";
      obj["message"] = err_msg;
      feedback_messages_.push_back(obj);
      keep_collecting_.store(false,std::memory_order_relaxed);
      keep_transmitting_.store(false,std::memory_order_relaxed);

      // -- dump the previous eth package to disk
      //FIXME Remove this once the issue with the BR is sorted out
      char fname[128];
      strftime(fname, sizeof(fname), "ctb_eth_dump_%Y%m%d_%H%M%S.txt", &now_tm);

      std::ofstream dfout(fname);
      for (size_t i = 0; i < debug_eth_buffer.nentries; i++) {
        if (i==0) dfout << std::hex << debug_eth_buffer.data[i] ;
        dfout << debug_eth_buffer.data[i];
        if (!(i%4)) dfout << std::dec << "\n";
      }
      dfout.close();


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
  // FIXME: Keep an eye for problems by commenting this out.
  // this assumes that the send operation gains ownership over the memory of the data
  // being sent. This is probably tru, but should be checked
  //  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Finish the connection
  Log(info,"Closing data socket.");
  close_data_connection();
  Log(info,"Data socket closed.");
}



