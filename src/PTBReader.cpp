/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "PTBReader.h"
#include "Logger.h"
#include "PracticalSocket.h"
#include "PTBexception.h"
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


#ifdef ARM_XDMA
// declare a bunch of variables that are used by the DMA driver

static int g_dma_fd;              // mmapped file descriptor
static uint8_t *g_dma_mem_map;    /* mmapped array of bytes representing the transferred memory */
#define DMA_BUFFER_SIZE 33554432  // 32 MB buffer for the DMA trasaction. Should give plenty of
// contigency for the CPU
#endif /*ARM_XDMA*/

#ifdef ARM_MMAP
// This contains the data register
mapped_register data_reg;

struct read_status {
  typedef uint8_t state_t;
  typedef uint8_t index_t;
  index_t idx : 1;
  state_t state :2;
  static const state_t  states[2];
  uint32_t get_next_read() {
    state = states[++idx];
    return static_cast<uint32_t>(state);
  }
};

const read_status::state_t  read_status::states[2] = {0x1,0x2};

#endif
void print_bits(void* memstart, size_t nbytes) {

  std::cout << "The " << nbytes << "-byte chunk of memory beginning at " 
      << static_cast<void*>(memstart) << " is : [" << std::endl;

  for(unsigned int i = 0; i < nbytes; i++) {
    std::cout << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
  }
  std::cout << "\n]" << std::endl;
}

std::string display_bits(void* memstart, size_t nbytes) {

  std::stringstream bitstr;
  bitstr << "The " << nbytes << "-byte chunk of memory beginning at " 
      << static_cast<void*>(memstart) << " is : [";

  for(unsigned int i = 0; i < nbytes; i++) {
    bitstr << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
  }
  bitstr << "]";
  return bitstr.str();
}



PTBReader::PTBReader() : tcp_port_(0), tcp_host_(""),
    socket_(nullptr),
    #ifdef ENABLE_FRAG_BLOCKS
    packet_rollover_(0),fragmented_(false),
#endif
    client_thread_collector_(0),client_thread_transmitter_(0),ready_(false),
#ifdef ARM_MMAP
    memory_pool_(nullptr),mapped_data_base_addr_(0),
#endif
#ifdef ARM_XDMA
    dma_buffer_(nullptr),
#endif
    seq_num_(0),
    keep_transmitting_(true),
    keep_collecting_(true),time_rollover_(0),
    // below this point all these are debugging variables
    dry_run_(false), error_state_(false),error_messages_(""),
    num_eth_fragments_(0),
    num_word_counter_(0),num_word_trigger_(0),num_word_fifo_warning_(0),
    num_word_tstamp_(0),bytes_sent_(0)
{


#ifndef LOCKFREE
  // Init the mutex for the queue
  if (pthread_mutex_init(&lock_, NULL) != 0)
  {
    Log(error,"\n Failed to create the mutex for the data queue\n" );
    throw PTBexception("Failed to create the mutex for the data queue.");
  }
#endif

}

PTBReader::~PTBReader() {
  Log(debug,"Destroying the reader." );
#ifndef LOCKFREE
  pthread_mutex_destroy(&lock_);
#endif
  ready_ = false;
}

void PTBReader::ClearThreads() {
  if (dry_run_) {
    // the threads only exist in transmission mode. 
    return;
  } else {
    Log(debug,"Killing the daughter threads.\n");
    // First stop the loops
    keep_collecting_ = false;
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    Log(info,"Joining collector thread.");
    // Kill the collector thread first
    // Ideally we would prefer to join the thread
    if (client_thread_collector_ != 0) {
      pthread_join(client_thread_collector_,NULL);
      client_thread_collector_ = 0;
    }
    // Kill the transmitter thread.

    // Occasionally there seems to be memory corruption between these two steps
    Log(info,"Telling transmitter thread to stop.");
    keep_transmitting_ = false;
    std::this_thread::sleep_for (std::chrono::milliseconds(200));
    // -- Apparently this is a bit of a problem since the transmitting thread never leaves cleanly
    Log(info,"Killing transmitter thread.");
    if (client_thread_transmitter_) {
      pthread_join(client_thread_transmitter_,NULL);
      client_thread_transmitter_ = 0;
    }
    Log(info,"Threads cleared");
  }
}

void PTBReader::StopDataTaking() {
  // This should be a full stop. 
  // However there should aso be a PauseDataTaking, that does not destroy the threads
  // and simply waits for a StartRun to come again.
  // Clean up the threads
  ClearThreads();

  // Prepare everything so that the PTB gets ready again

#if defined(ARM_XDMA)
  //FIXME: There is a risk that the thread is killed before
  // the execution reaches this point.
  Log(warning,"Shutting down the DMA engine.");
  if (munmap(g_dma_mem_map, DMA_BUFFER_SIZE) == -1) {
    Log(error,"Error un-mmapping the DMA file");
  }
  // Un-mmaping doesn't close the file.
  close(g_dma_fd);

#elif defined(ARM_MMAP)
  Log(info,"Unmapping data registers...");
  // Release the mmapped memory for the data
  UnmapPhysMemory(mapped_data_base_addr_,(data_reg.high_addr-data_reg.base_addr));
  Log(info,"Emptying the buffer queue");
  bool has_entries;
  do {
    // TODO: Check that calling a destructor of a partial memory address
    // does not cause trouble.
    has_entries = buffer_queue_.pop();
  } while(has_entries);

  Log(info,"Deleting memory pool...");
  delete [] memory_pool_;
  // The file is closed when the configuration registers are unmapped
#endif /*ARM_MMAP*/

  Log(info,"Data taking stopped.");
}

void PTBReader::StartDataTaking() {
  if (dry_run_) { // in dry run don't launch anything
    return;
  } else {
    if (ready_) {
#ifdef ARM_XDMA
      int status = 0;

      g_dma_fd = open("/dev/xdma", O_RDWR | O_CREAT | O_TRUNC, (mode_t) 0600);
      if (g_dma_fd == -1) {
        Log(error,"Error opening DMA mem file for writing");
        status = -1;
      }

      // mmap the file to get access to the memory area.
      // Map 32 MB (to allow for a circular buffer of 8192 pages
      g_dma_mem_map = (uint8_t*)mmap(0, DMA_BUFFER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, g_dma_fd, 0);
      if (g_dma_mem_map == MAP_FAILED) {
        close(g_dma_fd);
        Log(error,"Error mmapping the DMA file");
        status = -2;
      }
      // get number of DMA devices
      int num_devices = 0;
      if (ioctl(g_dma_fd, XDMA_GET_NUM_DEVICES, &num_devices) < 0) {
        Log(error,"Error ioctl getting device num");
        status = -3;
      }
      // I know that we only have 1 device
      if (num_devices <= 0) {
        Log(error,"Couldn't find DMA devices (expected 1).");
        status = -4;
      } else {
        Log(info,"Found %d DMA devices (1 expected)",num_devices);
      }

      // Fill up the structure
      xdma_device.tx_chan = 0;
      xdma_device.rx_chan = 0;
      xdma_device.tx_cmp = 0;
      xdma_device.rx_cmp = 0;
      xdma_device.device_id = 0;

      // Get device info
      if (ioctl(g_dma_fd, XDMA_GET_DEV_INFO, &xdma_device) < 0) {
        Log(error,"Error ioctl getting device info");
        status = -5;
      }
      // Print the addresses
      Log(info,"Device info: tx chan: %x, tx cmp:%x, rx chan: %x, rx cmp: %x",
          xdma_device.tx_chan, xdma_device.tx_cmp, xdma_device.rx_chan, xdma_device.rx_cmp);

      // Set up the destination channel
      xdma_dst_cfg.chan = xdma_device.rx_chan;
      xdma_dst_cfg.dir = XDMA_DEV_TO_MEM;
      xdma_dst_cfg.coalesc = 1;
      xdma_dst_cfg.delay = 0;
      xdma_dst_cfg.reset = 0;

      if (ioctl(g_dma_fd, XDMA_DEVICE_CONTROL, &xdma_dst_cfg) < 0) {
        Log(error,"Error ioctl config dst (rx) chan");
        status = -6;
      }

      if (status < 0) {
        Log(error,"Failed to initialize the DMA engine for data collection (status = %d).",status);
        // FIXME: Get rid of the exceptions. Too slow.
        throw PTBexception("Failed to initialize the DMA engine for PTB data collection.");
        return;
      }
#elif defined(ARM_MMAP)

      Log(debug,"Mapping the data registers");
      // Map the physical addresses
      SetupDataRegisters();
      mapped_data_base_addr_ = MapPhysMemory(data_reg.base_addr,data_reg.high_addr);
      control_register_.address = reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_data_base_addr_) + data_reg.addr_offset[0]);
      data_register_.address = reinterpret_cast<void*>(reinterpret_cast<uint32_t>(mapped_data_base_addr_) + data_reg.addr_offset[1]);
#endif /*ARM_MMAP*/

      // We are ready to do business. Start reading the DMA into the queue.
      Log(verbose, "==> Creating collector\n" );
      keep_collecting_ = true;
      if (pthread_create(&client_thread_collector_,NULL,&(PTBReader::ClientCollectorFunc),this) != 0) {
        Log(error,"Unable to create client thread. Failing..." );
        throw PTBexception("Unable to create data collector.");
        return;
      }
      // We are ready to do business. Start reading the DMA into the queue.
      Log(verbose, "==> Creating transmitter\n" );
      keep_transmitting_ = true;
      if (pthread_create(&client_thread_transmitter_,NULL,&(PTBReader::ClientTransmitterFunc),this) != 0) {
        Log(error,"Unable to create client thread. Failing..." );
        throw PTBexception("Unable to create transmitter thread.");
        return;
      }

    } else {
      Log(error,"Calling to start data taking but connection is not ready yet." );
      throw PTBexception("Connection not available.");
    }
  }
}

void PTBReader::ResetBuffers() {
  Log(warning,"Resetting the software buffers.");
  uint32_t counter = 0;
#if defined(LOCKFREE)
  bool has_data = true;
  do {
    has_data = buffer_queue_.pop();
    counter++;
  } while (has_data);
#else
  pthread_mutex_lock(&lock_);

  while (!buffer_queue_.empty()) {
    buffer_queue_.pop();
    counter++;
  }
  pthread_mutex_unlock(&lock_);
#endif
  Log(info,"Popped %u entries from the buffer queue.",counter);
}

// Only does one thing and that is killing the data connection.
void PTBReader::CloseConnection() {
  bool socket_good = TestSocket();
  // only call for delete if the socket is good
  if ((socket_ != nullptr) && socket_good) delete socket_;
  socket_ = nullptr;
  ready_ = false;
}

bool PTBReader::TestSocket() {
  // -- test if the socket yields something usable
  bool socket_good = true;
  try {
    socket_->getForeignPort();
  }
  catch(...) {
    // if an exception is caught here, then the socket is no longer good.
    socket_good = false;
  }
  return socket_good;
}

void PTBReader::InitConnection(bool force) {
  //-- If a connection exists, assume it is correct and continue to use it
  // FIXME: A ghost connection can exist
  // Try first if the connection is good
  if (socket_ != nullptr) {
    bool socket_good = TestSocket();

    if (socket_good) {
      if (force) {
        Log(warning,"Destroying existing socket!");
        delete socket_;
        socket_ = nullptr;
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
      socket_ = new TCPSocket(tcp_host_.c_str(),tcp_port_);

      if (socket_ == nullptr) {
        Log(error,"Unable to establish the client socket. Failing." );
        ready_ = false;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    // -- Catch and rethrow the exceptions so that they can be dealt with at higher level.
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      ready_ = false;
      if (socket_) delete socket_;
      socket_ = nullptr;
      throw;
    }
    catch(std::exception &e) {
      ready_ = false;
      if (socket_) delete socket_;
      socket_ = nullptr;
      Log(error,"STD exception caught : %s",e.what() );
      throw;
    }
    catch(...) {
      ready_ = false;
      if (socket_) delete socket_;
      socket_ = nullptr;
      Log(error,"Unknown exception caught." );
      throw;
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
    ready_ = false;
    if (socket_) delete socket_;
    socket_ = nullptr;
    throw PTBexception("Calling to start connection without defining connection parameters.");
  }
  Log(verbose,"Connection opened successfully");
}


void PTBReader::ClientCollector() {
  Log(info, "Starting data collector\n" );

#ifdef ARM_XDMA
  // Registers should be setup already.
  // First setup the DMA:
  ///FIXME: Maybe this code could be moved elsewhere to speed up initialization
  //static int status = 0;
  //    Log(debug,"Allocating the DMA buffer.");
  //    static const buffer_end = buffer_size*frame_size_u32
  dma_buffer_ = nullptr;
  // the end of the buffer is the size in number uint32_t
  static const uint32_t buffer_end = DMA_BUFFER_SIZE/sizeof(uint32_t);

  static uint32_t pos = 0;

  int ret = 0;
  pos = 0; // pos is in units of uint32_t

  // Setup parts of the structure that are common to all transfers
  xdma_buf.chan = xdma_device.rx_chan;
  xdma_buf.completion = xdma_device.rx_cmp;
  xdma_buf.cookie = 0;
  xdma_buf.dir = XDMA_DEV_TO_MEM;

  xdma_trans.chan = xdma_device.rx_chan;
  xdma_trans.completion =  xdma_device.rx_cmp;

  dma_buffer_ = reinterpret_cast<uint32_t*>(g_dma_mem_map);

  uint64_t total_duration = 0;
  uint32_t iterations = 0;
  Log(debug,"Starting data collection");
  auto begin = std::chrono::high_resolution_clock::now();

  while (keep_collecting_) {
    auto begin_trf = std::chrono::high_resolution_clock::now();
    // Configure the receiving buffer
    // offset in bytes
    xdma_buf.buf_offset = pos*sizeof(uint32_t);
    xdma_buf.buf_size = 16*sizeof(uint8_t); // Grab a data frame

    // Prepare the buffer
    ret = (int)ioctl(g_dma_fd, XDMA_PREP_BUF, &xdma_buf);

    // Now perform the transation
    xdma_trans.cookie = xdma_buf.cookie;
    // should implement a wait or not?
    // Try with a wait fpr now.
    //FIXME: Try without waiting period (implying that the transfer occurs faster than
    // the time it takes to go there fetch the memory
    xdma_trans.wait = 1;
    ret = (int) ioctl(g_dma_fd, XDMA_START_TRANSFER, &xdma_trans);

    auto end_trf = std::chrono::high_resolution_clock::now();
    total_duration += std::chrono::duration_cast<std::chrono::microseconds>(end_trf-begin_trf).count();
    iterations++;
    if (ret == -1) {
      //#ifdef DEBUG
      //        Log(warning,"Reached a timeout in the DMA transfer.");
      //#endif
      timeout_cnt_++;
      if (timeout_cnt_ > timeout_cnt_threshold_*10) {
        Log(error,"Received too many timeouts. Failing the run.");
        // Artificially generate a warning packet
        dma_buffer_[pos] = WARN_TIMEOUT;
      }
    }

    // DMA transaction was successful.
    // FIXME: Replace this by lock free queue
  #if defined(LOCKFREE)
    buffer_queue_.enqueue(&dma_buffer_[pos]);
  #else
    pthread_mutex_lock(&lock_);
    buffer_queue_.push(&dma_buffer_[pos]);
    //buffer_queue_.push(&frame[pos]);
    pthread_mutex_unlock(&lock_);
  #endif
    //Log(debug,"Done storing the buffer");
    pos += frame_size_u32;
    //pos_pool += frame_size_bytes; 
    if (pos+frame_size_u32 >= buffer_end) {
      //#ifdef DEBUG
      //        Log(info,"Reached buffer size.Resetting to start.\n");
      //#endif
      pos = 0;
    } // pos check
  } // keep_collecting_

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
  Log(warning,"Stopped collecting data.");
  std::cout << "Duration " << duration << " us Iterations : "
      << iterations << ". Flow: "
      << duration/(double)iterations << " us/iteration" << std::endl;
  std::cout << "Transfer Duration " << total_duration
      << " us Iterations : " << iterations << ". Flow: "
      << total_duration/(double)iterations << " us/iteration" << std::endl;

  //#ifdef DEBUG
  //    Log(warning,"Stopped collecting data.");
  //#endif
  /// We can't exit the engine here otherwise the memory mapped registers disappear and the queue vanishes causing a crash
  // Log(debug,"Shutting down the DMA engine.");
  // xdma_exit();
  // Log(debug,"DMA engine done.");

#elif defined(ARM_MMAP)

  // In this case we want to have a local memory buffer and it is there that we
  // do any manipulations
  // Also allocate 32 MB
  memory_pool_ = new uint32_t[buffer_size_frames*frame_size_u32];
  static uint32_t pos = 0; // position pointer in the memory_pool_ variable
  static uint32_t test_val = 0x0;
  // Some memory testing
#if defined(MEASURE_PERFORMANCE)
  uint64_t total_duration = 0;
  uint32_t iterations = 0;
  // this should only be done after the first frame actually comes in
  // otherwise most of the time is spent in waiting
  auto begin = std::chrono::high_resolution_clock::now();
  bool first = true;
#ifdef DEBUG
  static uint32_t loop_counter = 0x0;
  static uint32_t test_val2 = 0x0;
  static uint32_t new_state = 0x0;
#endif
#endif

   // force position to always start at 0
  pos = 0;
  read_status stat;
  // Initialize the index
  stat.idx = 0x0;
  while(keep_collecting_) {
#if defined(MEASURE_PERFORMANCE)
    auto begin_trf = std::chrono::high_resolution_clock::now();
#endif

#if defined(DEBUG)
    // Set the read_ready bit
    // enable read_ready to trigger a transaction
    // There is an issue here. If the transaction is not done by the time
    // that the test of the bit is done, we will miss a transaction and another
    // one will happen. The idea is to keep polling until a frame comes
    Log(debug,"Initial control : %08X",control_register_.value());
    new_state = stat.get_next_read();
    control_register_.value() = new_state;
    Log(debug,"new_state 0x%X reg 0x%X idx %u state 0x%X",new_state,control_register_.value(),static_cast<uint32_t>(stat.idx),static_cast<uint32_t>(stat.state));
#else
    control_register_.value() = stat.get_next_read();
#endif
    // poll for the data to be released
    do {
#if defined(DEBUG)
      test_val2 = test_val;
      test_val = control_register_.value();
      if (test_val != test_val2) {
        Log(debug,"Changed status (%u) : 0x%X -> 0x%X",loop_counter,test_val2,test_val);
      }
      loop_counter++;
#else
      test_val = control_register_.value();
#endif
    } while ((((test_val >> 2) & 0x1) == 0x0) && keep_collecting_);
    // -- Data arrived or cancel request arrived
    if (!keep_collecting_) {
      break;
    }

#if defined(MEASURE_PERFORMANCE)
    //    // Check if there is valid data in the register
    //    if (((control_register_.value() >> 1 ) & 0x1) == 0x1) {
      if (first) {
        begin = std::chrono::high_resolution_clock::now();
        begin_trf = std::chrono::high_resolution_clock::now();
        first = false;
      }
#endif
      // A transation was completed
      // there is data to be collected
      std::memcpy(&memory_pool_[pos],data_register_.address,frame_size_bytes);
#if defined(MEASURE_PERFORMANCE)
      auto end_trf = std::chrono::high_resolution_clock::now();
      total_duration += std::chrono::duration_cast<std::chrono::microseconds>(end_trf-begin_trf).count();
      iterations++;

#endif
#ifdef DEBUG

      printf("%08X %08X %08X %08X\n",memory_pool_[pos],memory_pool_[pos+1],memory_pool_[pos+2],memory_pool_[pos+3]);
#endif
      // DMA transaction was successful.
#if defined(LOCKFREE)
      buffer_queue_.enqueue(&memory_pool_[pos]);
#else
      pthread_mutex_lock(&lock_);
      buffer_queue_.push(&memory_pool_[pos]);
      pthread_mutex_unlock(&lock_);
#endif
      pos += frame_size_u32;

      if (pos+frame_size_u32 >= buffer_size_frames) {
      pos = 0;
    }
  } // while keep_collecting
#if defined(MEASURE_PERFORMANCE)
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
  Log(warning,"Stopped collecting data.");
  std::cout << "Duration " << duration << " us Iterations : "
      << iterations << ". Flow: "
      << duration/(double)iterations << " us/iteration" << std::endl;
  std::cout << "Transfer Duration " << total_duration
      << " us Iterations : " << iterations << ". Flow: "
      << total_duration/(double)iterations << " us/iteration" << std::endl;
#else
  Log(info,"Stopped collecting data.");
  #endif

#else
#error No DMA mode defined.

#endif /*ARM_MMAP*/
}

#if defined(ARM_XDMA) || defined(ARM_MMAP)
void PTBReader::ClientTransmitter() {
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


  error_state_ = false;
  error_messages_ = "";
  // Debugging information that is passed down in the end of the run
  num_eth_fragments_ = 0;
  num_word_counter_ = 0;
  num_word_trigger_ = 0;
  num_word_tstamp_ = 0;
  bytes_sent_ = 0;

  // This will keep track on the number of u32 words
  // In the end the size of the buffer will be ipck*sizeof(uint32_t);
  static uint32_t ipck = 0;
  static bool carry_on = true;
  static bool ts_arrived = false;

  // Temporary variables that will end up making part of the eth packet
  static uint32_t packet_size = 0;
  uint32_t eth_header = 0;
  uint32_t eth_checksum = 0;

  // pointer to a new word
  uint32_t* frame = nullptr;
  ts_arrived = false;
  seq_num_ = 1;

  // Assign the skelleton packet header
  // This would be nice to go out of the loop but it
  eth_header = (fw_version_ << 28 ) | (((~fw_version_) & 0xF) << 24) | ((seq_num_  << 16) & 0xFF0000);

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  // Wonder if sometimes there could be a conflict when both a write and a read are issued over the variable.
  while(keep_transmitting_) {

    // Set the local pointer to the place pointer by the global pointer
    eth_buffer = &global_eth_buffer[global_eth_pos];

    // FW decides when the packet is ready to be sent.
    // The TS word marks that a packet has to be sent.

    // 2 conditions to be careful of:
    //
    // 1. If time time based rollover is reached.
    // 2. If the size based rollover is reached. (not implemented)


    /// -- Start by generating a header

    // ipck   : Counter of u32
    ipck = 0;
    carry_on = true;

    eth_header = SetBitRange(eth_header,(seq_num_  << 16) & 0xFF0000,16,8);
    ipck += 1;

    // Log(verbose, "Temp HEADER : %x (%x [%u] %x [%u])\n",eth_buffer[0],fw_version_,fw_version_,seq_num_,seq_num_);

    // while a packet_sending_condition is not reached
    // keep the loop going...
    // Needs some extra protection for when the StopRun is called, 
    // otherwise the loop does not
    // really stop since it will waiting forever for a 
    // timestamp word that will never arrive.
    while(carry_on and keep_transmitting_) {

      // Grab a frame...if there is one to grab
#if defined(LOCKFREE)
      while (!buffer_queue_.try_dequeue(frame) && keep_transmitting_) {
        continue;
      }
      // break out if a stop was requested while waiting for data to be available.
      if (!keep_transmitting_) {
	Log(info,"Received request to stop transmitting.");
	break;
      }
#else
      if (buffer_queue_.size() == 0) {
        continue;
      }
      // FIXME: Replace this by a lock free queue
      //Log(debug,"Grabbing from queue");
      pthread_mutex_lock(&lock_);
      frame = buffer_queue_.front();
      buffer_queue_.pop();
      pthread_mutex_unlock(&lock_);
#endif

      // TODO: If the PTBreader is still slow after these changes modify the
      // software to simply send the whole thing at once without resizing the packets
      // All packets become 16 bytes long and send everything as collected

#ifdef DEBUG
      Log(debug,"Grabbed : %08X %08X %08X %08X",frame[0],frame[1],frame[2],frame[3]);
#endif

      // Grab the header, that now should be at the 4 lsB
      Word_Header *frame_header = reinterpret_cast_checked<Word_Header *>(frame);
      //Word_Header *frame_header = reinterpret_cast<Word_Header *>(frame);

      // Very first check to discard "ghost frames"
      // -- Ghost frames are caused by the NOvA timing not being fully initialized by
      // the time the word was generated. These usually occur because there
      // is a delay between the sync pulse and the timestamps starting
      // to be populated in the NOvA firmware.
      // Unfortunately it doesn't seem that anything can be done about it

      // Conditions that must be checked before the first timestamp arrives
      if (!ts_arrived) {
        // FIXME: Get rid of ghost frames on the FW side
        // Ghost frames are a mess. For the moment keep them but need to set
        // up something more reliable.
        // Most likely on the firmware side
        // after the first TS arrives ignore ghost frames

        if ((frame_header->short_nova_timestamp & 0x7FFFFFF) == 0x0) {
          //#ifdef DEBUG
          Log(warning,"Dropping ghost frame");
          //#endif
          continue;
        }

        // FIXME: This is just temporary. Have to fix the firmware to grab
        // the proper data
        // This problem is strongly related with the previous.
        // Will have to fix it on the firmware side to ensure that the very
        // first word is a timestamp
        if ((frame_header->data_packet_type != DataTypeTimestamp)) {
          //#ifdef DEBUG
          Log(warning,"Dropping frames until a timestamp shows up.");
          //#endif
          continue;
        }
      } // ts_arrived


      /// -- Check if the packet is a TS frame
      if(frame_header->data_packet_type == DataTypeTimestamp) {
        ts_arrived = true;
        // This is a timestamp word.
        // Check if this is the first TS after StartRun
        // Log(verbose, "Timestamp frame...\n");
#ifdef DEBUG
        TimestampPayload *t = reinterpret_cast<TimestampPayload *>(&(frame[TimestampPayload::payload_offset_u32]));
        Log(verbose,"Timestamp first estimate  %" PRIX64 " (%" PRIu64 ")",t->nova_timestamp,t->nova_timestamp);
        std::cout << "Just to make sure: " << t->nova_timestamp << std::endl;

        // Log(verbose,"Timestamp full packet:");
        // print_bits(frame,16);
        // Log(verbose,"Timestamp estimation:");
        // print_bits(&frame[Word_Header::size_words+TimestampPayload::ptb_offset],16-(Word_Header::size_words+TimestampPayload::ptb_offset));
        // print_bits(t,8);
#endif
        std::memcpy(&eth_buffer[ipck],frame,Word_Header::size_bytes);
        ipck += Word_Header::size_u32;

        // Don't forget to apply the ptb_offset!!
        std::memcpy(&eth_buffer[ipck],
                    &(frame[TimestampPayload::payload_offset_u32]),
                    TimestampPayload::size_bytes);
        ipck += TimestampPayload::size_u32;
#ifdef DEBUG
        //Log(verbose,"Intermediate packet:");
        //print_bits(eth_buffer,ipck);
        // No support for
#ifdef ENABLE_FRAG_BLOCKS
        fragmented_ = false;
#endif
#endif

#ifdef DATA_STATISTICS
        num_word_tstamp_++;
#endif
        // break out of the cycle to send the packet
        carry_on = false;
        break;
      }

      /// --  Not a TS packet...just accumulate it
      // -- Grab the header (valid for all situations
      // Log(verbose, "Grabbing the header\n");

      std::memcpy(&eth_buffer[ipck],frame,Word_Header::size_bytes);
      ipck += Word_Header::size_u32;

      switch(frame_header->data_packet_type) {
      case DataTypeCounter:
#ifdef DEBUG
        // The counter words now require even more special handling
        Log(verbose,"Counter word");
#endif
        // The data that is in the header is from the BSU's, so we can simply copy the payload
        // and then just add the 2 additional bits from the header at the end
        // I put it hardcoded so that I don't end up messing up with it
        // Since the firmware sends little endian data, the bytes are reversed
        std::memcpy(&eth_buffer[ipck],
            &(frame[CounterPayload::payload_offset_u32]),
            CounterPayload::size_words_ptb_bytes);
        // Now add the remaining 2 bits to the lsb of the next u32
        // and pad the rest with zeros
        ipck += CounterPayload::size_words_ptb_u32;
        eth_buffer[ipck] = 0x2 & frame_header->padding;
        ipck+=1;
#ifdef DATA_STATISTICS
        // Log(verbose,"Intermediate packet:");
        // print_bits(eth_buffer,ipck);

        num_word_counter_++;
#endif
        break;
      case DataTypeTrigger:
        // Log(verbose, "Trigger word\n");
        std::memcpy(&eth_buffer[ipck],
                    &frame[TriggerPayload::payload_offset_u32],
                    TriggerPayload::size_bytes);
        ipck += TriggerPayload::size_u32;
#ifdef DATA_STATISTICS
        // Log(verbose,"Intermediate packet:");
        // print_bits(eth_buffer,ipck);
        num_word_trigger_++;
#endif
        break;
      case DataTypeWarning:
        Log(warning,
            "+++ Received a FIFO warning of type %08X .+++",
            (reinterpret_cast_checked<uint32_t*>(frame))[0]);
#ifdef DATA_STATISTICS
        num_word_fifo_warning_++;
#endif
        break;
      default:
        eth_buffer[ipck] = WARN_UNKNOWN_DATA;
        ipck += 1;
        Log(warning,"Unknown data type [%X]",frame[0] & 0xFF);
        Log(warning,"Bits : %s",display_bits(&frame_header,sizeof(frame_header)).c_str());
      } // case


#ifdef ENABLE_FRAG_BLOCKS
      // Size Rollover reached. Produce a fragmented block;
      // Keep collecting only until the next TS word
      if ((ipck+frame_size_u32) >= packet_rollover_) {
        Log(warning,"Issuing a fragmented packet. THis is going to cause trouble.");
        fragmented_ = true;
        break;
      }
#endif
    } // -- carry_on && keep transmitting

    // -- if keep_transmitting was called out (eg. run ended),
    // don't send this data. The board reader won't like to receive an incomplete packet without
    // timestamp word
    if (!keep_transmitting_) {
      Log(info,"Stopping transmission requested. Breaking out.");
      break;
    }
    // Transmit the data.
    // Log(verbose, "Packet completed. Calculating the checksum.\n");

    // Add one word to account for the checksum
    packet_size = (ipck+1)*sizeof(uint32_t);
    // Log(verbose,"Size was calculated to be %u",packet_size);


    eth_header = SetBitRange(eth_header,packet_size,0,16);
    std::memcpy(&(eth_buffer[0]),&eth_header,sizeof(eth_header));

    // Calculate the checksum
    //
    // -- Followed the recipe in
    //    https://en.wikipedia.org/wiki/BSD_checksum
    static uint16_t checksum = 0x0;
#ifdef DEBUG
    checksum = 0x0;
    //    uint8_t*ch_buff = reinterpret_cast<uint8_t*>(eth_buffer);
    uint8_t*ch_buff = eth_buffer;
    for (uint32_t i = 0; i < ipck; ++i) {

      checksum = (checksum >> 1) + ((checksum & 0x1) << 15) ;
      checksum += ch_buff[i];
      checksum &= 0xFFFF;
      //Log(debug," Byte %u : %08X Chksum %08X (%hu)",i,ch_buff[i],checksum,checksum);
    }
    // Log(verbose,"Checksum : %hu",checksum);
    // Calculate 16 bit checksum of the packet
#endif
    // Add the checksum to the last packet
    eth_checksum = (0x4 << 29) | checksum;
    std::memcpy(&eth_buffer[ipck],&eth_checksum,sizeof(eth_checksum));
    ipck += 1;
    //packet_size = ipck*sizeof(uint32_t);
#ifdef DEBUG
    Log(debug,"Sending packet with %u bytes (including header)",packet_size);
    print_bits(eth_buffer,packet_size);

#endif
    try {
      socket_->send(eth_buffer,packet_size);
      global_eth_pos += (ipck+4);
      // add 4 bytes of padding just to make sure that there are no overlaps
      // for occasional small packets troubles could happen.
      if ((global_eth_pos+(4*ipck)) > eth_buffer_size_u32) {
        // reset the pointer to the beginning
        global_eth_pos = 0;
      }
#ifdef DATA_STATISTICS
      num_eth_fragments_++;
      bytes_sent_ += packet_size;
#endif
      }
    catch(SocketException &e) {
      Log(error,"Socket exception : %s",e.what() );
      // Stop collecting and transmitting
      //
      error_state_ = true;
      error_messages_ += "<error>Data socket exception [";
      error_messages_ += currentDateTime();
      error_messages_ += "] : ";
      error_messages_ += e.what();
      error_messages_ += "</error>";
      keep_collecting_ = false;
      keep_transmitting_ = false;
      //
    }

    // FIXME: Add support for fragmented blocks. 
    // Require the board reader to hold the data 
    // in a buffer until the whole microslice is parsed
#ifdef ENABLE_FRAG_BLOCKS
    if (!fragmented_) {
      seq_num_++;
      fragmented_ =false;
    } else {
      seq_num_ = seq_num_;
      Log(error,"Fragmented packets are not currently supported.");
      //throw PTBexception("Fragmented packets are not supported.");
      keep_transmitting_ = false;
      keep_collecting_ = false;
    }
#else 
    seq_num_++;
#endif
    // if (seq_num_ >= 5) {
    //   Log(warning,"FIXME: Forcing to stop after the first packet being generated");
    //   keep_collecting_ = false;
    //   keep_transmitting_ = false;
    //   // sleep for a while waiting for the generators to stop
    //   std::this_thread::sleep_for (std::chrono::seconds(5));

  } // -- while(keep_transmitting_)
  // Exited the  run loop. Return.

  Log(info,"Exited transmission loop.");
  if (error_state_) {
    Log(warning,"Transmission loop exited with error state.");
  }
  // wait for a few moments before deallocating the memory so that the kernel does not go ballistic
  // in case it hans't yet committed all the buffers
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // Deallocate the memory
  delete [] global_eth_buffer;
  // Finish the connection
  Log(warning,"Closing data socket.");
  CloseConnection();
  Log(info,"Data socket closed.");
  // Should be safe to delete the memory_pool here as well.
  // Close the socket as well
}

#else
#error No DMA mode defined.
void PTBReader::ClientTransmitter() {
}
#endif

