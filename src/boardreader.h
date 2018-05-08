/*
 * PTBReadefr.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

#include "config.h"

#include <string>
#include <iostream>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <thread>
#if defined(BOOST)
#include <boost/lockfree/spsc_queue.hpp>
#elif defined(LOCKFREE)
#include "atomicops.h"
#include "readerwriterqueue.h"
#else
#include <queue>
#endif

#include "util.h"
#include "pothos_zynq_dma_driver.h"

#include "content.h"
#include "json.hpp"
#include <atomic>

using json = nlohmann::json;
class TCPSocket;

namespace ptb {

/**
 * This class is responsible for reading the DMA and transferring the data to
 *  the server.
 * \class PTBReader
 *
 * \details This class is kept simple on purpose. 
 * It does not talk with the [ConfigServer]
 * by design and all it does is really collect the data from the memory 
 * assembles the TCP packets and sends them to the board reader.
 *
 */
class board_reader {
 public:
  
    // NFB : The data structure specification had been moved outside into content.h

  /// Bunch of structures that help manipulating the data.
  /// These *must* be kept in sync with lbne-raw-data

  /** 
   * Implementation of the PTBReader class
   **/

  board_reader();
  
  virtual ~board_reader();
  
  bool get_ready() const {
    return ready_;
  }
  
  void set_ready(bool ready) {
    ready_ = ready;
  }

  const std::string& get_tcp_host() const {
    return tcp_host_;
  }

  void set_tcp_host(const std::string& tcpHost) {
    tcp_host_ = tcpHost;
  }

  unsigned short get_tcp_port() const {
    return tcp_port_;
  }

  void set_tcp_port(unsigned short tcpPort) {
    tcp_port_ = tcpPort;
  }

  // FIXME: Implement this along with the fragmented blocks
#ifdef ENABLE_FRAG_BLOCKS
  uint32_t get_packet_rollover() const {
    return packet_rollover_;
  }

  void set_packet_rollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }
#endif
  /**
   * Stop the client threads without touching the mutex.
   * Important in the case that one wants to delete the object 
   * without having started a run.
   */
  void clear_threads();

  /** 
   * Function called by the manager to cleanly stop 
   * the data transmission and connection
   */
  void stop_data_taking();

  /** Starts the collection and transmission threads
   * if the class was instanciated in "dry run" mode, 
   * it does nothing. 
   **/
  void start_data_taking();

  /**
   * Resets the buffers. The run should already be stopped.
   */
  void reset_buffers();
  
  /** Start the connection to the board reader **/
  void init_data_connection(bool force = false);
  void close_data_connection();

  bool test_socket();

  bool get_data_connection_valid() const {
    // This might not be necessarily the best option
    return (data_socket_ != NULL);
  }
  
  // -- Statistics methods:
  uint32_t get_n_sent_frags() {return num_eth_fragments_;};
  uint32_t get_n_status() {return num_word_counter_;};
  uint32_t get_n_gtriggers() {return num_word_gtrigger_;};
  uint32_t get_n_ltriggers() {return num_word_ltrigger_;};
  uint32_t get_n_warns() {return num_word_feedback_;};
  uint32_t get_n_timestamps() {return num_word_tstamp_;};
  uint32_t get_sent_bytes() {return bytes_sent_;};


  void set_dry_run(bool status) {dry_run_ = status;};

  bool get_dry_run() {return dry_run_;};

  bool get_error_state() {return error_state_;};

  json & get_error_msgs() {return error_messages_;};

protected:
  /** Data collector function into the queue. Runs on it's own thread**/
  void data_collector();

  /** Data transmitter **/
  void data_transmitter();

  void dump_packet(uint32_t* buffer, uint32_t tot_size);
  void clean_and_shutdown_dma();
  void init_dma();
private:

  // -- Structures for data socket connection

  unsigned short  tcp_port_;
  std::string     tcp_host_;

  TCPSocket *     data_socket_;

  std::thread     *client_thread_collector_;
  std::thread     *client_thread_transmitter_;

#ifdef ENABLE_FRAG_BLOCKS
  uint32_t packet_rollover_;
#endif
  bool ready_;



  pzdud_t *s2mm;
  static const size_t num_buffs_ = 4096; // number of allocated buffers for the DMA
  static const size_t buff_size_ = 4096; // size of each individual buffer (1 memory page) in bytes
  uint32_t buff_addr_[num_buffs_];

#if defined(BOOST)
  boost::lockfree::spsc_queue<ptb::content::buffer_t, boost::lockfree::capacity<num_buffs_> >buffer_queue_;
#elif defined(LOCKFREE)
  moodycamel::ReaderWriterQueue<uint32_t*> buffer_queue_;
#else
  std::queue<uint32_t*> buffer_queue_;
#endif



  //FIXME: Revise the need for these words
  // It may make more sense to have them hardcoded in content.h
  //  // Warning pre_computed words
  //  static const uint32_t WARN_TIMEOUT = 0x04000000;
  //  static const uint32_t WARN_UNKNOWN_DATA = 0x02000000;

  // A few auxiliary constants
  static const uint32_t eth_buffer_size_u32 = 0xFFFF; // Max possible ethernet packet
  static const uint32_t frame_size_bits   = 0x80;   // the buffer is 128 bits
  static const uint32_t frame_size_bytes  = 0x10;   // 16 bytes
  static const uint32_t frame_size_u32    = 0x4;    // 4xuint32_t

  // A few more constants that are important
  static const uint32_t fw_version_ = FIRMWARE_REVISION;

  // Frame sequence number
  uint32_t seq_num_;
#ifdef ENABLE_FRAG_BLOCKS
  bool fragmented_;
#endif
  bool keep_transmitting_;
  bool keep_collecting_;
  uint64_t time_rollover_;
  bool dma_initialized_;
  // -- Debugging and control variables
  // timeouts don't make sense with MMAP
  // neither does in SG_DMA, as it depends on the frequency of the data

bool dry_run_; // Run the reader without collecting data
  // FIXME: Finish implementing the atomicity
std::atomic<bool> error_state_;
  json error_messages_;
//  std::vector<std::string> error_messages_;

  // -- Internal run statistics
  uint32_t num_eth_fragments_;
  uint32_t num_word_counter_;
  uint32_t num_word_gtrigger_;
  uint32_t num_word_ltrigger_;
  uint32_t num_word_feedback_;
  uint32_t num_word_tstamp_;
  uint32_t bytes_sent_;
  std::map<int,int> counter_stats_;
  std::map<ptb::content::word::trigger_code_t,int> trigger_stats_;

};

}

#endif /* PTBREADER_H_ */
