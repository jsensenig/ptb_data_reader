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
//FIXME replace with folly queue
#elif defined(LOCKFREE)
#include "atomicops.h"
#include "readerwriterqueue.h"
#else
#include <queue>
#endif

#include "util.h"
#include "pothos_zynq_dma_driver.h"
#include "ProducerConsumerQueue.h"

#include "zmq.hpp"
#include "content.h"
#include "json.hpp"
#include <atomic>

using json = nlohmann::json;
class TCPSocket;

namespace ptb {


  typedef struct eth_packet {
      size_t nbytes;
      size_t nentries;
    uint32_t data[0x20000]; // 512 kb 
  } eth_packet;

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

      board_reader();

      virtual ~board_reader();

      bool get_ready() const {
        return ready_;
      }

      void set_ready(bool ready) {
        ready_ = ready;
      }

      void set_fake_data(bool fake_data) {
    	fake_data_ = fake_data;
      }

      void init_data_connection();
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

      // -- Statistics methods:
      uint32_t get_n_sent_frags() {return num_eth_fragments_;};
      uint32_t get_n_status() {return num_word_chstatus_;};
      uint32_t get_n_gtriggers() {return num_word_gtrigger_;};
      uint32_t get_n_ltriggers() {return num_word_ltrigger_;};
      uint32_t get_n_feedback() {return num_word_feedback_;};
      uint32_t get_n_timestamps() {return num_word_tstamp_;};
      uint32_t get_sent_bytes() {return bytes_sent_;};

      void reset_counters();

      void get_feedback(bool &error, json&msgs,const bool reset = true);

    protected:
      /** Data collector function into the queue. Runs on it's own thread**/
      void data_collector();

      /** Data transmitter **/
      void data_transmitter();

      void dump_packet(uint32_t* buffer, uint32_t tot_size);
      void clean_and_shutdown_dma();
      void init_dma();
      void fake_data_sender();
    private:

      std::thread     *client_thread_collector_;
      std::thread     *client_thread_transmitter_;

      bool ready_;
      bool fake_data_;


      /// -- DMA parameters

      pzdud_t *s2mm;
      static const size_t eth_buffer_size_bytes_ = 0x80000;// 512 kb //0x40000;// 256 kB
      static const size_t eth_buffer_size_u32_ = 0x20000; // 512 kb in 4 byte words // 0xFFFF; // 256 kB

      //static uint32_[eth_buffer_size_u32_];
      static const size_t num_buffs_ = 4096; // number of allocated buffers for the DMA
      static const size_t buff_size_ = 4096; // size of each individual buffer (1 memory page) in bytes
      uint32_t buff_addr_[num_buffs_];

#if defined(BOOST)
      //boost::lockfree::spsc_queue<ptb::content::buffer_t, boost::lockfree::capacity<num_buffs_> > buffer_queue_;
#elif defined(LOCKFREE)
      moodycamel::ReaderWriterQueue<uint32_t*> buffer_queue_;
#else
      std::queue<uint32_t*> buffer_queue_;
#endif

      folly::ProducerConsumerQueue<ptb::content::buffer_t> buffer_queue_{100000};

      bool dma_initialized_;

      zmq::context_t ctx_;
      zmq::socket_t sock_;

      /// -- these are controlled through the weak locks
      /// for more information see:
      /// https://www.arangodb.com/2015/02/comparing-atomic-mutex-rwlocks/
      /// https://gist.github.com/13abylon/523685ef6af5bde24dc3
      ///
      /// Technique 10 on that example is used:
      /// atomic_uint.store(i, std::memory_order_relaxed);
      /// atomic_uint.load(std::memory_order_acquire);
      std::atomic<bool> error_state_;
      std::atomic<bool> reset_dma_engine_;
      std::atomic<bool> keep_transmitting_;
      std::atomic<bool> keep_collecting_;

      // -- Error states for the various DMA errors
      std::atomic<bool> error_dma_timeout_;
      std::atomic<bool> error_dma_claimed_;


      json feedback_messages_;

      /// -- Internal run statistics
      uint32_t num_eth_fragments_;
      uint32_t num_word_chstatus_;
      uint32_t num_word_gtrigger_;
      uint32_t num_word_ltrigger_;
      uint32_t num_word_feedback_;
      uint32_t num_word_tstamp_;
      uint32_t bytes_sent_;

  };

}

#endif /* PTBREADER_H_ */
