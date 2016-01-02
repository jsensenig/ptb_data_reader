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
#include <sys/time.h>
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
    dry_run_(false),
    num_eth_fragments_(0),
    num_word_counter_(0),num_word_trigger_(0),num_word_fifo_warning_(0),
    num_word_tstamp_(0),bytes_sent_(0),
    first_timestamp_(0),last_timestamp_(0)
{


#ifndef LOCKFREE
  // Init the mutex for the queue
  // FIXME: Replace mutex by lock free queue
  if (pthread_mutex_init(&lock_, NULL) != 0)
  {
    Log(error,"\n Failed to create the mutex for the data queue\n" );
    throw std::string("Failed to create the mutex for the data queue.");
  }
#endif

}

PTBReader::~PTBReader() {
  Log(debug,"Destroying the reader." );
#ifndef LOCKFREE
  //FIXME: Replace mutex by lock free queue
  pthread_mutex_destroy(&lock_);
#endif
  ready_ = false;
}

void PTBReader::ClearThreads() {
  if (dry_run_) {
    // the threads only exist in transmission mode. 
    return;
  } else {
    // FIXME: Migrate this to C++11 threads.
    Log(debug,"Killing the daughter threads.\n");
    // First stop the loops
    keep_collecting_ = false;
    std::this_thread::sleep_for (std::chrono::milliseconds(100));
    Log(info,"Killing collector thread.");
    // Kill the collector thread first
    // Ideally we would prefer to join the thread
    if (client_thread_collector_ != 0) {
      pthread_join(client_thread_collector_,NULL);
    }
    // Kill the transmittor thread.

    keep_transmitting_ = false;
    std::this_thread::sleep_for (std::chrono::seconds(2));
    // -- Apparently this is a bit of a problem since the transmitting thread never leaves cleanly
    Log(info,"Killing transmitter thread.");
    if (client_thread_transmitter_) {
      pthread_join(client_thread_transmitter_,NULL);
    }
  }
}

void PTBReader::StopDataTaking() {
  // This should be a full stop. 
  // However there should aso be a PauseDataTaking, that does not destroy the threads
  // and simply waits for a StartRun to come again.
  // Clean up the threads
  ClearThreads();

  // Prepare everything so that the PTB gets ready again

#ifdef ARM_XDMA
  //FIXME: There is a risk that the thread is killed before
  // the execution reaches this point.
  Log(warning,"Shutting down the DMA engine.");
  if (munmap(g_dma_mem_map, DMA_BUFFER_SIZE) == -1) {
    Log(error,"Error un-mmapping the DMA file");
  }
  // Un-mmaping doesn't close the file.
  close(g_dma_fd);
#endif /*ARM_XDMA*/

#ifdef ARM_MMAP
  // Release the mmapped memory for the data
  UnmapPhysMemory(mapped_data_base_addr_,(data_reg.high_addr-data_reg.base_addr));
  delete [] memory_pool_;
  // The file is closed when the configuration registers are unmapped
#endif /*ARM_MMAP*/

  //#ifdef ARM_POTHOS
  //  // All this below should only be done after not only the DMA  is finished
  //  // but the waiting buffers are done as well.
  //  int ret = pzdud_halt(s2mm_);
  //  if (ret != PZDUD_OK) {
  //    Log(warning,"Failed to halt the DMA engine");
  //  }
  //
  //  // Free the DMA channel
  //  pzdud_free(s2mm_);
  //
  //  // cleanup the DMA channel
  //  pzdud_destroy(s2mm_);
  //#endif

  Log(info,"Data taking stopped.");
}

void PTBReader::StartDataTaking() {
  //FIXME: Migrate this to C++11 constructs
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
#endif

#ifdef ARM_MMAP
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
      Log(error,"Calling to start taking data connection is not ready yet." );
      throw PTBexception("Connection not available.");
    }
  }
}

void PTBReader::ResetBuffers() {
  // FIXME: Migrate this to C++11 constructs (std::thread::mutex)
  // FIXME: Replace mutex by lock free queue
  Log(warning,"Resetting the software buffers.");
  uint32_t counter = 0;
#if defined(LOCKFREE)
  buffer_queue_.pop();
  counter++;
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
  delete socket_;
}

void PTBReader::InitConnection(bool force) {
  //-- If a connection exists, assume it is correct and continue to use it
  // FIXME: A ghost connection can exist
  if (socket_ != NULL) {
    if (force) {
      Log(warning,"Destroying existing socket!");
      delete socket_;
    } else {
      Log(info,"Reusing existing connection.");
      return;
    }
  }

  // Check if the server data is set up, and compute
  if (tcp_host_ != "" && tcp_port_ != 0) {

    try{
      Log(debug, "Opening socket connection : %s : %hu",tcp_host_.c_str(),tcp_port_ );
      socket_ = new TCPSocket(tcp_host_.c_str(),tcp_port_);

      if (socket_ == NULL) {
        Log(error,"Unable to establish the client socket. Failing." );
        ready_ = false;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    // -- Catch and rethrow the exceptions sho that they can be dealt with at higher level.
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      ready_ = false;
      delete socket_;
      socket_ = NULL;
      throw;
    }
    catch(std::exception &e) {
      ready_ = false;
      delete socket_;
      socket_ = NULL;
      Log(error,"STD exception caught : %s",e.what() );
      throw;
    }
    catch(...) {
      ready_ = false;
      delete socket_;
      socket_ = NULL;
      Log(error,"Unknown exception caught." );
      throw;
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
    ready_ = false;
    delete socket_;
    socket_ = NULL;
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
  memory_pool_ = new uint32_t[buffer_size*frame_size_u32];
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
    //}
    //  else {
    //      printf("There is nothing\n");
    //      control_register_.value() = 0x0;
    //      continue;
    //    }
    if (pos+frame_size_u32 >= buffer_size) {
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

#elif defined(ARM_POTHOS)
  // This should be done out of this part
  // Create DMA channel
  s2mm_ = pzdud_create(0, PZDUD_S2MM);
  static int ret = 0;

  if (s2mm_ == NULL) {
    Log(error,"Failed to create DMA instance.");
    return;
  }

  // Allocate DMA channel
  // num_buffs = 2048
  // buff_size = 4096 // This is a full page. Sounds appropriate although
  // I am not totally sure it won't be better to use more smaller pages
  ret = pzdud_alloc(s2mm_, 2048, 4096);
  if (ret != PZDUD_OK) {
    Log(error,"Failed to allocate buffers.");
    return ;
  }

  // Init the DMA
  ret = pzdud_init(s2mm_, true);
  if (ret != PZDUD_OK) {
    Log(error,"Failed to initialize the DMA");
    return ;
  }

  // Create all the buffer containers

  std::g_dma_mem_map<int,DMABuffer *> buffers;
  for (int i = 0; i < 2048; ++i) {
    buffers[i] = new DMABuffer;
    buffers[i]->data_ptr  = static_cast<uint32_t*>(pzdud_addr(s2mm_, i));
    buffers[i]->size = 0;
    buffers[i]->handle = i;
  }

  keep_collecting_ = true;
  size_t len;
  int handle;
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  while (keep_collecting_) {
    // Now enter the loop to do real work
    // Wait for a second
    ret = pzdud_wait(s2mm_, 1000000);
    if (ret != PZDUD_OK) {
      Log(warning,"Timeout waiting for data from the MicroZed");
    }

    handle = pzdud_acquire(s2mm_, &len);
    if (handle < 0) {
      Log(error,"Failed to acquire handle %d", handle);
      return ;
    }

    // I can make this much more efficient by pre-creating the references to the handles
    // The allocation table remains always the same until a reset or an alloc are called

    if (buffers[handle]->size != 0) {
      Log(error,"Allocation error. Returned buffer still in use [%d]",handle);
      throw PTBexception("Returned a buffer still in use");
    }

    buffers[handle]->size = len;
#if defined(LOCKFREE)
    buffer_queue_.enqueue(buffers[handle]);
    #else
    // DMA transaction was successful.
    pthread_mutex_lock(&lock_);
    buffer_queue_.push(buffers[handle]);
    pthread_mutex_unlock(&lock_);
#endif
  }

  // Loop is done



  //pzdud_release(s2mm_, handle, 0);

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout << "WARNING: Elapsed time : " << duration << std::endl;

#endif /*ARM_POTHOS*/
}

#if defined(ARM_XDMA) || defined(ARM_MMAP)
void PTBReader::ClientTransmitter() {
  Log(debug, "Starting data transmitter\n");


  // Allocate the maximum memory that an eth packet can have.
  // FIXME: Change this to zero memory copy. If using MMAP, it should be trivial
  // as the position should never be overrun
  // Ideally one would want to not allocate the eth packet and simply use the
  // allocated RAM directly. Doable, but must be careful because of the circular buffer.
  // For the moment copy the contents and see if performance is good enough.
  // use std::memcpy for better performance

  // This should never happen. Sending a 64kB packet would be way overkill
  // It would be preferable to fragment the packet, b
  //FIXME: Implement fragmentation
  uint32_t *eth_buffer = new uint32_t[max_packet_size]();

  num_eth_fragments_ = 0;
  num_word_counter_ = 0;
  num_word_trigger_ = 0;
  num_word_tstamp_ = 0;
  bytes_sent_ = 0;
  first_timestamp_ = 0;
  last_timestamp_ = 0;

  // This will keep track on the number of u32 words
  // In the end the size of the buffer will be ipck*sizeof(uint32_t);
  static uint32_t ipck = 0;
  static bool carry_on = true;
  static bool ts_arrived = false;
  static uint32_t packet_size = 0;
  uint32_t* frame = NULL;
  uint32_t eth_header = 0;
  uint32_t eth_checksum = 0;

  ts_arrived = false;
  seq_num_ = 1;

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  while(keep_transmitting_) {

    // FW decides when the packet is ready to be sent.
    // When DMA brings the TS word it is sign that packet has to be sent

    // 2 conditions to be careful of:
    //
    // 1. If time time based rollover is reached.
    // 2. If the size based rollover is reached.


    /// -- Start by generating a header

    /// -- The endianess issue:
    // The data is collected little endian. However, for this kind of applications
    // little endian is a bit silly in the sense that the most significant byte 
    // is on the
    // last position of an array. This often causes troubles of the 
    // bytes being swapped in groups
    // of 4 (a uint32_t is 4 bytes long...har har)

    // ipck   : Counter of u32
    // iframe : Frame counter.
    ipck = 0;
    carry_on = true;

    // FIXME: Move part of this assigment out of the loop.
    eth_header = (fw_version_ << 28 ) | (((~fw_version_) & 0xF) << 24) | ((seq_num_  << 16) & 0xFF0000);

    //    eth_buffer[0] = header;
    // Log(verbose, "Temp HEADER : %x (%x [%u] %x [%u])\n",eth_buffer[0],fw_version_,fw_version_,seq_num_,seq_num_);
    // ipck should not include the header
    // But it also works as an index counter, so the manipulation is done at the end
    ipck += 1;

    // while a packet_sending_condition is not reached
    // keep the loop going...
    // Needs some extra protection for when the StopRun is called, 
    // otherwise the loop does not
    // really stop since it will waiting forever for a 
    // timestamp word that will never arrive.
    while(carry_on and keep_transmitting_) {

      // Grab a frame...if there is one to grab
      // There should be a better way to set up this wait
      // Ideally we would love to have a inerrupt sending a 
      // callback the moment that there
#if defined(LOCKFREE)
      while (!buffer_queue_.try_dequeue(frame) && keep_transmitting_) {
        continue;
      }
      // break out if a stop was requested while waiting for data to be available.
      if (!keep_transmitting_) break;
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

      // FIXME: If the PTBreader is still slow after these changes modify the
      // software to simply send the while thing at once without resizing the packets
      // All packets become 16 bytes long and send everything as collected

#ifdef DEBUG
      Log(debug,"Grabbed : %08X %08X %08X %08X",frame[0],frame[1],frame[2],frame[3]);
#endif

      // Grab the header, that now should be at the 4 lsB
      //Word_Header *frame_header = reinterpret_cast_checked<Word_Header *>(frame);
      Word_Header *frame_header = reinterpret_cast<Word_Header *>(frame);

      // Very first check to discard "ghost frames"
      // -- Ghost frames are caused by the NOvA timing not being fully initialized by
      // the time the word was generated. These usually occur because there
      // is a delay between the sync pulse and the timestamps starting 
      // to be populated in the NOvA firmware.
      // Unfortunately it doesn't seem that anything can be done about it

      // FIXME: Get rid of ghost frames on the FW side
      // Ghost frames are a mess. For the moment keep them but need to set 
      // up something more reliable.
      // Most likely on the firmware side
      // Wouldn't there be a possibility of randomly reaching a state with a full zero nova timestamp?
      if ((seq_num_ == 1) && (frame_header->short_nova_timestamp & 0x7FFFFFF) == 0x0) {
        //#ifdef DEBUG
        Log(warning,"Dropping ghost frame");
        //#endif
        continue;
      }

      // FIXME: THis is just temporary. Have to fix the firmware to grab 
      // the proper data
      // This problem is strongly related with the previous. 
      // Will have to fix it on the firmware side
      if (!ts_arrived && (frame_header->data_packet_type != DataTypeTimestamp)) {
        //#ifdef DEBUG
        Log(warning,"Dropping frames until a timestamp shows up.");
        //#endif
        continue;
      }

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

#ifdef MEASURE_PERFORMANCE
        num_word_tstamp_++;
#endif
        // break out of the cycle
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
        std::memcpy(&eth_buffer[ipck],
            &(frame[CounterPayload::payload_offset_u32]),
            CounterPayload::size_words_ptb_bytes);
        // Now add the remaining 2 bits to the lsb of the next u32
        // and pad the rest with zeros
        ipck += CounterPayload::size_words_ptb_u32;
        eth_buffer[ipck] = 0x2 & frame_header->padding;
        ipck+=1;
#ifdef MEASURE_PERFORMANCE
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
#ifdef MEASURE_PERFORMANCE
        // Log(verbose,"Intermediate packet:");
        // print_bits(eth_buffer,ipck);
        num_word_trigger_++;
#endif
        break;
      case DataTypeWarning:
        Log(warning,
            "+++ Received a FIFO warning of type %08X .+++",
            (reinterpret_cast_checked<uint32_t*>(frame))[0]);
#ifdef MEASURE_PERFORMANCE
        num_word_fifo_warning_++;
#endif
        break;
      default:
        eth_buffer[ipck] = WARN_UNKNOWN_DATA;
        ipck += 1;
        Log(warning,"Unknown data type [%X] (%s)",frame[0] & 0xFF, display_bits(&frame_header,sizeof(frame_header)).c_str());

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
    }

    // Transmit the data.
    // Log(verbose, "Packet completed. Calculating the checksum.\n");


    // FIXME: Verify that the size is correct. 
    // The size that is written in the header should not include the checksum
    packet_size = ipck*sizeof(uint32_t);
    // Log(verbose,"Size was calculated to be %u",packet_size);


    eth_header = SetBitRange(eth_header,packet_size,0,16);
    std::memcpy(&(eth_buffer[0]),&eth_header,sizeof(eth_header));
    // // eth_buffer[0] = SetBitRange(eth_buffer[0],packet_size,0,16);
    // bitdump.str(""); bitdump << std::bitset<4>(eth_buffer[0] >> 28)
    // 			     << " " << std::bitset<4>(eth_buffer[0] >> 24 & 0xF)
    // 			     << " " << std::bitset<8>(eth_buffer[0] >> 16 & 0xFF)
    // 			     << " " << std::bitset<16>(eth_buffer[0] & 0xFFFF);
    // Log(verbose,"Temp header : %X [%s]",eth_buffer[0], bitdump.str().c_str());

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
    packet_size = ipck*sizeof(uint32_t);
#ifdef DEBUG
    Log(debug,"Sending packet with %u bytes (including header)",packet_size);
    print_bits(eth_buffer,packet_size);

#endif
    try {
      socket_->send(eth_buffer,packet_size);
      num_eth_fragments_++;
    }
    catch(SocketException &e) {
      Log(error,"Socket exception : %s",e.what() );
      // Try again
      socket_->send(eth_buffer,packet_size);
      num_eth_fragments_++;
      // rethrow the exception so that is caught and run is stopped properly
      //FIXME: Not sure if the exception will be caught in the other thread.
      throw;
      // Set the run to be stopped
//      keep_transmitting_ = false;
//      keep_collecting_ = false;
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
  // Deallocate the memory
  delete [] eth_buffer;
  // Should be safe to delete the memory_pool here as well.
}
#elif defined(ARM_POTHOS)
void PTBReader::ClientTransmitter() {
  Log(verbose, "Starting transmitter\n");

  seq_num_ = 1;
  uint32_t header = 0;

  // For the sake of the argument let's assume a packet should never be bigger than a page
  // If it is then will have to implement bigger pages
  // Even better, could use a quasi-zero copy method
  uint32_t *eth_buffer = new uint32_t[4096]();

  // This will keep track on the number of u32 words
  // In the end the size of the buffer will be ipck*sizeof(uint32_t);
  static size_t parsed_bytes = 0; // holds how many bytes of the present page have been parsed
  // to find out when to release the page and grab a new one
  DMABuffer *buffer_page = NULL;
  static uint32_t ipck = 0;
  static bool building_eth_pkt = true;
  static bool ts_arrived = false;

  uint32_t* frame = NULL;
  // This is no longer used

  ts_arrived = false;

  // The whole method runs on an infinite loop with control variables
  while(keep_transmitting_) {

    // FW decides when the packet is ready to be sent.
    // When DMA brings the TS word it is sign that packet has to be sent downstream

    // The data is collected little endian. However, for this kind of applications
    // little endian is a bit silly in the sense that the most significant byte is on the
    // last position of an array. This often causes troubles of the bytes being swapped in groups
    // of 4 (a uint32_t is 4 bytes long...har har)

    /*
        Since the board reader is a bunch of spaggetti, it is better to use a hybrid
       approach here:
         - The header's are keps as loaded from the DMA and therefore are passed in little endian. The logic on
          the board reader is not trivial and it was tested as working.
         - The payloads are byte-swapped (using htonl), becoming effectively big-endian and therefore a reflection
          of what the board actually produces.
     */

    ipck = 0;
    building_eth_pkt = true;

    // FIXME: Move part of this assigment out of the loop.
    header = (fw_version_ << 28 ) | (((~fw_version_) & 0xF) << 24) | ((seq_num_  << 16) & 0xFF0000);



    //    eth_buffer[0] = header;
    // Log(verbose, "Temp HEADER : %x (%x [%u] %x [%u])\n",eth_buffer[0],fw_version_,fw_version_,seq_num_,seq_num_);
    // ipck should not include the header
    // But it also works as an index counter, so the manipulation is done at the end
    ipck += sizeof(header);

    // while a packet_sending_condition is not reached
    // keep the loop going...
    // Needs some extra protection for when the StopRun is called, otherwise the loop does not
    //really stop since it will waiting forever for a timestamp word that will never arrive.
    while(building_eth_pkt and keep_transmitting_) {


      // Grab a frame...if there is one to grab
      // There should be a better way to set up this wait
      // Ideally we would love to have a inerrupt sending a callback the moment that there
      // buffer_queue_ is not empty

      if (buffer_page == NULL) {

        if (buffer_queue_.size() == 0) {
          continue;
        }

        // grab a new frame
        pthread_mutex_lock(&lock_);
        buffer_page = buffer_queue_.front();
        parsed_bytes = 0;
        buffer_queue_.pop(); // remove the page from the queue
        pthread_mutex_unlock(&lock_);

      }
      // FIXME: If the PTBreader is still slow after these changes modify the
      // software to simply send the while thing at once without resizing the packets
      // Enter a new LOOP to process the current buffer

      frame = &((buffer_page->data_ptr)[parsed_bytes]);
#ifdef DEBUG
      Log(debug,"Grabbed : %08X %08X %08X %08X",frame[0],frame[1],frame[2],frame[3]);
#endif


      // Grab the header, that now should be at the 4 lsB
      Word_Header *frame_header = reinterpret_cast<Word_Header *>(frame);

      // Very first check to discard "ghost frames"
      // -- Ghost frames are caused by the NOvA timing not being fully initialized by
      // the time the word was generated. These usually occur because there is a delay
      // between the sync pulse and the timestamps starting to be populated in the NOvA firmware.
      // Unfortunately it doesn't seem that anything can be done about it

      // Ghost frames are a mess. For the moment keep them but need to set up something more reliable.
      // Most likely on the firmware side
      if ((frame_header->short_nova_timestamp & 0x7FFFFFF) == 0x0) {
        //#ifdef DEBUG
        Log(warning,"Dropping ghost frame");
        //#endif
        continue;
      } // --

      // FIXME: THis is just temporary. Have to fix the firmware to grab the proper data
      // This problem is strongly related with the previous. Will have to fix it on the firmware side
      if (!ts_arrived && (frame_header->data_packet_type != DataTypeTimestamp)) {
        //#ifdef DEBUG
        //if ((first_timestamp_ == 0) && (frame_header->data_packet_type != DataTypeTimestamp)) {
        Log(warning,"Dropping frames until a timestamp shows up.");
        //}
        //#endif
        continue;
      }


      /// -- Check if it is a TS frame
      if(frame_header->data_packet_type == DataTypeTimestamp) {
        ts_arrived = true;
        // This is a timestamp word.
        // Check if this is the first TS after StartRun
        // Log(verbose, "Timestamp frame...\n");
        // Log(verbose, "Sending the packet\n");
        // Log(verbose,"Offset %u ",offset);
#ifdef DEBUG
        TimestampPayload *t = reinterpret_cast<TimestampPayload *>(&(frame[TimestampPayload::payload_offset_u32]));
        Log(verbose,"Timestamp first estimate  %" PRIX64 " (%" PRIu64 ")",t->nova_timestamp,t->nova_timestamp);
        std::cout << "Just to make sure: " << t->nova_timestamp << std::endl;

        // Log(verbose,"Timestamp full packet:");
        // print_bits(frame,16);
        // Log(verbose,"Timestamp estimation:");
        // print_bits(&frame[Word_Header::size_words+TimestampPayload::ptb_offset],16-(Word_Header::size_words+TimestampPayload::ptb_offset));
        // print_bits(t,8);



        if (first_timestamp_ == 0) {
          first_timestamp_ = t->nova_timestamp;

          last_timestamp_ = first_timestamp_;
          Log(verbose,"Timestamp estimated to be %" PRIX64 " (%" PRIu64 ")",first_timestamp_);
        } else {
          last_timestamp_ = t->nova_timestamp;
        }
#endif
        std::memcpy(&eth_buffer[ipck],frame,Word_Header::size_bytes);
        ipck += Word_Header::size_u32;

        // Don't forget to apply the ptb_offset!!
        std::memcpy(&eth_buffer[ipck],&(frame[TimestampPayload::payload_offset_u32]),TimestampPayload::size_bytes);
        ipck += TimestampPayload::size_u32;
#ifdef DEBUG
        //Log(verbose,"Intermediate packet:");
        //print_bits(eth_buffer,ipck);
        // No support for
        fragmented_ = false;
        num_word_tstamp_++;
#endif
        // break out of the cycle
        building_eth_pkt = false;
        break;
      } // timestamp word

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
        std::memcpy(&eth_buffer[ipck],&(frame[CounterPayload::payload_offset_u32]),CounterPayload::size_words_ptb_bytes);
        // Now add the remaining 2 bits to the lsb of the next byte and pad the rest with zeros
        ipck += CounterPayload::size_words_ptb_u32;
        eth_buffer[ipck] = 0x2 & frame_header->padding;
        ipck+=1;

#ifdef DEBUG
        //  for (uint32_t i = 0; i < 3; ++i) {
        //   eth_buffer[ipck] = 0x00;
        //   ipck += 1;
        // }
        // Log(verbose,"Intermediate packet:");
        // print_bits(eth_buffer,ipck);

        num_word_counter_++;
#endif
        break;
      case DataTypeTrigger:
        // Log(verbose, "Trigger word\n");
        std::memcpy(&eth_buffer[ipck],&frame[TriggerPayload::payload_offset_u32],TriggerPayload::size_bytes);
        ipck += TriggerPayload::size_u32;
        // Log(verbose,"Intermediate packet:");
        // print_bits(eth_buffer,ipck);
#ifdef DEBUG
        num_word_trigger_++;
#endif
        break;
      case DataTypeWarning:
        //#ifdef DEBUG
        Log(warning,
            "+++ Received a FIFO warning of type %08X .+++",
            (reinterpret_cast_checked<uint32_t*>(frame))[0]);
        // We only really care for the header. Ignore everything else
        // Log(verbose, "Selftest word\n");
        num_word_fifo_warning_++;
        //#endif
        break;
      default:
        eth_buffer[ipck] = WARN_UNKNOWN_DATA;
        ipck += 1;
#ifdef DEBUG
        Log(warning,"Unknown data type [%X] (%s)",frame[0] & 0xFF, display_bits(&frame_header,sizeof(frame_header)).c_str());
#endif

      } // -- end case


      // Size Rollover reached. Produce a fragmented block;
      // Keep collecting only until the next TS word
#ifdef DEBUG
      if ((ipck+20) >= packet_rollover_) {
        Log(warning,"Issuing a fragmented packet. THis is going to cause trouble.");
        fragmented_ = true;
        break;
      }
#endif

      // Out of the loop. Send the packet
    }

    // Transmit the data.
    // Log(verbose, "Packet completed. Calculating the checksum.\n");

    // Write the size (in bytes)
    // keep in mind that ipck is summed 1 for the checksum
    //    uint16_t packet_size = ((ipck+1)*sizeof(uint32_t));

    // This discounts the headers and the checksum, only keeps useful data
    // to be directly comparable to the value reported by the board reader
#ifdef DEBUG
    bytes_sent_ += ipck*sizeof(uint32_t);
#endif
    // Is this correct? I would have thought that it already has the size
    // needed. The packet_size should not include the header, since that
    // is a fixed part of it
    uint32_t packet_size = (ipck+1)*sizeof(uint32_t);

    // Log(verbose,"Size was calculated to be %u",packet_size);

    // eth_buffer is a uint8_t. Better to reassign to the header and then
    // memcopy the header into the buffer

    header = SetBitRange(header,packet_size,0,16);
    std::memcpy(&(eth_buffer[0]),&header,sizeof(header));

    // // eth_buffer[0] = SetBitRange(eth_buffer[0],packet_size,0,16);
    // bitdump.str(""); bitdump << std::bitset<4>(eth_buffer[0] >> 28)
    //           << " " << std::bitset<4>(eth_buffer[0] >> 24 & 0xF)
    //           << " " << std::bitset<8>(eth_buffer[0] >> 16 & 0xFF)
    //           << " " << std::bitset<16>(eth_buffer[0] & 0xFFFF);
    // Log(verbose,"Temp header : %X [%s]",eth_buffer[0], bitdump.str().c_str());
    // Calculate the checksum
    //
    // -- Followed the recipe in
    //    https://en.wikipedia.org/wiki/BSD_checksum

#ifdef DEBUG
    static uint16_t checksum = 0x0;
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
    // Add the checksum to the last packet
    //eth_buffer[ipck] = (0x4 << 29) | checksum;
    static uint32_t checksum_word = (0x4 << 29) | checksum;
    std::memcpy(&eth_buffer[ipck],&checksum_word,sizeof(uint32_t));
    // JCF, Jul-18-2015
    ipck += sizeof(checksum_word);
#endif
    // I'm switching "ipck+1" in place of "ipck+2", below - not sure
    // why there was an extra uint32_t (above and beyond the
    // microslice header) being sent
#ifdef DEBUG
    Log(debug,"Sending packet with %u bytes (including header)",ipck);
    print_bits(eth_buffer,ipck);

#endif
    try {
      socket_->send(eth_buffer,ipck);
    }
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );

      // Set the run to be stopped
    }

    // if we didn't have a fragmented block, update the sequence number
    // and update the timestamp to the latest one
#ifdef DEBUG
    if (!fragmented_) {
#endif
      seq_num_++;
      fragmented_ =false;

#ifdef DEBUG
    } else {
      seq_num_ = seq_num_;
      Log(error,"Fragmented packets are not currently supported.");
      //throw PTBexception("Fragmented packets are not supported.");
      keep_transmitting_ = false;
      keep_collecting_ = false;
    }
#endif
    // if (seq_num_ >= 5) {
    //   Log(warning,"FIXME: Forcing to stop after the first packet being generated");
    //   keep_collecting_ = false;
    //   keep_transmitting_ = false;
    //   // sleep for a while waiting for the generators to stop
    //   std::this_thread::sleep_for (std::chrono::seconds(5));
    // }

    // Update the buffer page pointer
    parsed_bytes += frame_size_bytes;
    if (parsed_bytes == buffer_page->size) {
      // Reached the end of this page. Close, release it and grab a new one
      // This should only be released when the data has been sent
      pzdud_release(s2mm_, buffer_page->handle, 0);
      buffer_page->size = 0;
      buffer_page = NULL;
    }


  } // -- while(keep_transmitting_)
  // Exited the  run loop. Return.
  Log(info,"Exited transmission loop.");
  // Deallocate the memory
  delete [] eth_buffer;
}



#endif

// Completely auxiliary clock function
uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}

