/*
 * PTBReader.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

#include <string>
#include <iostream>
#include <queue>

extern "C" {
#include<pthread.h>
#include <stdint.h>
#include <stdio.h>
}

#include <cstdlib>
#include <cstring>

#ifdef ARM_POTHOS
#include "pothos_zynq_dma_driver.h"

struct DMABuffer {
  uint32_t *data_ptr;
  size_t size;
  int handle;
};

#endif
#ifdef ARM_XDMA
#include "xdma.h"
#endif
#ifdef ARM_MMAP // why?
#include "util.h"
#endif

class TCPSocket;



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
class PTBReader {
 public:
  
  /// Bunch of structures that help manipulating the data
  // ethernet packet header
  struct Header {
    
    typedef uint32_t data_t;
    
    typedef uint8_t  format_version_t;
    typedef uint8_t  sequence_id_t;
    typedef uint16_t block_size_t;
    
    block_size_t     block_size     : 16;
    sequence_id_t    sequence_id    : 8;
    format_version_t format_version : 8;
    
    static size_t const size_words = sizeof(data_t);
    
    //static const size_t raw_header_words = 1;
    //data_t raw_header_data[raw_header_words];
  };
  
  struct Word_warning {
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;
    
    typedef uint8_t  warning_type_t;
    typedef uint8_t  data_packet_type_t;
    
    // The order of the data packet type and the timestamp have been
    // swapped to reflect that it's the MOST significant three bits in
    // the payload header which contain the type. I've also added a
    // 1-bit pad to reflect that the least significant bit is unused.
    
    uint32_t padding                        : 24;
    warning_type_t warning_type             : 5;
    data_packet_type_t     data_packet_type : 3;
    
    static size_t const size_words = sizeof(data_t);
    static data_size_t const num_bits_padding     = 24;
    static data_size_t const num_bits_warning     = 5;
    static data_size_t const num_bits_packet_type = 3;
  };

  // Common header to everything except warning words
  struct Word_Header {
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;
    
    typedef uint8_t  data_packet_type_t;
    typedef uint32_t short_nova_timestamp_t;
    
    // The order of the data packet type and the timestamp have been
    // swapped to reflect that it's the MOST significant three bits in
    // the payload header which contain the type. I've also added a
    // 2-bit pad to reflect that the 2 least significant bits are unused.
    
    uint8_t padding : 2;
    short_nova_timestamp_t short_nova_timestamp : 27;
    data_packet_type_t     data_packet_type     : 3;
    
    static size_t const size_bytes = sizeof(data_t);
    static size_t const size_u32 = sizeof(data_t)/sizeof(uint32_t);
    
    static data_size_t const num_bits_padding 	= 2;
    static data_size_t const num_bits_short_tstamp	= 27;
    static data_size_t const num_bits_packet_type	= 3;
  };

  struct CounterPayload {
    typedef uint64_t counter_set_t;
    typedef uint16_t data_size_t;
    // -- Must be careful to follow the right order
    // from lsb to msb it is
    // -- TSU mappings
    counter_set_t tsu_wu     : 10;
    counter_set_t tsu_el     : 10;
    counter_set_t tsu_extra  :  4;
    counter_set_t tsu_nu     :  6;
    counter_set_t tsu_sl     :  6;
    counter_set_t tsu_nl     :  6;
    counter_set_t tsu_su     :  6;
    // -- BSU mappings
    counter_set_t bsu_rm     : 16;//end of first counter_set_t==uint64_t
    counter_set_t bsu_cu     : 10;
    counter_set_t bsu_cl1    : 6;
    counter_set_t extra      : 1;
    counter_set_t bsu_cl2    : 7;
    counter_set_t bsu_rl     : 10;
    // Just ignore the rest of the word
    counter_set_t padding    : 30;
    
    // Not sure of what is this exactly
    //static data_size_t num_bytes_padding_ptb = 2*sizeof(uint32_t);
    
    static data_size_t const num_bits_tsu_wu    = 10;
    static data_size_t const num_bits_tsu_el    = 10;
    static data_size_t const num_bits_tsu_extra =  4;
    static data_size_t const num_bits_tsu_nu    =  6;
    static data_size_t const num_bits_tsu_sl    =  6;
    static data_size_t const num_bits_tsu_nl    =  6;
    static data_size_t const num_bits_tsu_su    =  6;
    static data_size_t const num_bits_bsu_rm    = 16;
    static data_size_t const num_bits_bsu_cu    = 10;
    static data_size_t const num_bits_bsu_cl1   = 6;
    static data_size_t const num_bits_bsu_extra = 1; // -- unused channel (32)
    static data_size_t const num_bits_bsu_cl2   = 7;
    static data_size_t const num_bits_bsu_rl    = 10;
    static data_size_t const num_bits_padding   = 30;
    
    static size_t const size_bytes = 2*sizeof(uint64_t);
    static size_t const size_u32 = size_bytes/sizeof(uint32_t);
    
    // The size that arrives from the PTB
    // NOTE: Not sure what this is for.
    static size_t const size_words_ptb_u32 = 3;
    static size_t const size_words_ptb_bytes = size_words_ptb_u32*sizeof(uint32_t);
    // The offset to grab the data
    static size_t const ptb_offset_u32 = 0;
    static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
    
    // The payload position offset from the top of the frame (header + discard)
    static size_t const payload_offset_u32 = 1+ptb_offset_u32;
    static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);
  };

  struct TriggerPayload {
    typedef uint32_t trigger_type_t;
    typedef uint16_t data_size_t;

    // The 23 lsb are padding. No information is passed there
    trigger_type_t padding : 19; 

    trigger_type_t trigger_id_calib: 4; // which of the calibration channels 
    trigger_type_t trigger_id_muon : 4;
    trigger_type_t trigger_type    : 5; // the 5 msb are the trigger type
    
    static data_size_t const num_bits_padding = 23;
    static data_size_t const num_bits_trigger_id = 4;
    static data_size_t const num_bits_trigger_type = 5;
    
    // ID the trigger types
    static trigger_type_t const calibration = 0x00;
    static trigger_type_t const muon        = 0x10;
    static trigger_type_t const ssp         = 0x08;
    // -- This should probably be split into RCE and then RCE types
    static trigger_type_t const rce_a       = 0x01;
    static trigger_type_t const rce_b       = 0x02;
    static trigger_type_t const rce_ab      = 0x03;
    static trigger_type_t const rce_c       = 0x04;
    static trigger_type_t const rce_ac      = 0x05;
    static trigger_type_t const rce_bc      = 0x06;
    static trigger_type_t const rce_abc     = 0x07;
    
    static size_t const size_bytes = sizeof(uint32_t);
    static size_t const size_u32 = size_bytes/sizeof(uint32_t);

    // The number of u32 that have to offset from the PTB packet to grab the data that matters
    // In this case the two lsInts are empty and can be offset
    static size_t const ptb_offset_u32 = 2;
    static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
    
    // The payload position offset from the top of the frame (header + discard)
    static size_t const payload_offset_u32 = 1;
    static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);
    
    // Add a function that can be used to parse the trigger payload
    static std::string getTriggerName(trigger_type_t trigger_type) {
      switch (trigger_type) {
      case calibration:
	return "calibration";
	break;
      case rce_a:
	return "rce_a";
	break;
      case rce_b:
	return "rce_b";
	break;
      case rce_c:
	return "rce_c";
	break;
      case rce_ab:
	return "rce_ab";
	break;
      case rce_ac:
	return "rce_ac";
	break;
      case rce_bc:
	return "rce_bc";
	break;
      case rce_abc:
	return "rce_abc";
	break;
      case ssp  :
	return "ssp";
	break;
      case muon :
	return "muon";
	break;
      default:
	return "unknown";
	break;
      }
      return "";
    }
    
  };

  struct TimestampPayload {
    typedef uint64_t timestamp_t;
    typedef uint16_t data_size_t;
    timestamp_t nova_timestamp : 64;
    
    static data_size_t const num_bits_timestamp = 64;
    static size_t const size_bytes = sizeof(uint64_t);
    static size_t const size_u32 = size_bytes/sizeof(uint32_t);
    // drop 1 int
    static size_t const ptb_offset_u32 = 1;
    static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
    // The payload position offset from the top of the frame (header + discard)
    
    static size_t const payload_offset_u32 = 1+ptb_offset_u32;
    static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);
  
  };

  typedef Header::block_size_t microslice_size_t;
  
  //the size of the payloads (neglecting the Payload_Header)
  // FIXME: Since everythig is being worked in bytes the sizes can be trimmed
  static microslice_size_t const payload_size_counter   = 4*sizeof(uint32_t); //128 bit payload.
  static microslice_size_t const payload_size_trigger   = 1 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_timestamp = 2 * sizeof(uint32_t); //64-bit payload
  static microslice_size_t const payload_size_selftest  = 1 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_checksum  = 0 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_warning   = 0 * sizeof(uint32_t); //32-bit payload
  
  //The types of data words
  static const Word_Header::data_packet_type_t DataTypeWarning   = 0x0; //0b000
  static const Word_Header::data_packet_type_t DataTypeCounter   = 0x1; //0b001
  static const Word_Header::data_packet_type_t DataTypeTrigger   = 0x2; //0b010
  static const Word_Header::data_packet_type_t DataTypeChecksum  = 0x4; //0b100
  static const Word_Header::data_packet_type_t DataTypeTimestamp = 0x7; //0b111

  /** 
   * Implementation of the PTBReader class
   **/

  PTBReader();
  
  virtual ~PTBReader();
  
  bool isReady() const {
    return ready_;
  }
  
  void setReady(bool ready) {
    ready_ = ready;
  }

  const std::string& getTcpHost() const {
    return tcp_host_;
  }

  void setTcpHost(const std::string& tcpHost) {
    tcp_host_ = tcpHost;
  }

  unsigned short getTcpPort() const {
    return tcp_port_;
  }

  void setTcpPort(unsigned short tcpPort) {
    tcp_port_ = tcpPort;
  }

  // FIXME: Implement this along with the fragmented blocks
  uint32_t getPacketRollover() const {
    return packet_rollover_;
  }

  void setPacketRollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }
  
  /**
   * Stop the client threads without touching the mutex.
   * Important in the case that one wants to delete the object 
   * without having started a run.
   */
  void ClearThreads();

  /** 
   * Function called by the manager to cleanly stop 
   * the data transmission and connection
   */
  void StopDataTaking();

  /** Starts the collection and transmission threads
   * if the class was instanciated in "dry run" mode, 
   * it does nothing. 
   **/
  void StartDataTaking();

  /**
   * Resets the buffers. The run should already be stopped.
   */
  void ResetBuffers();
  
  /** Start the connection to the board reader **/
  void InitConnection(bool force = false);

  bool getConnectionValid() const {
    // This might not be necessarily the best option
    return (socket_ != NULL);
  }
  
  // -- Statistics methods:
  uint32_t GetNumMicroslices() {return num_microslices_;};
  uint32_t GetNumCounterWords() {return num_word_counter_;};
  uint32_t GetNumTriggerWords() {return num_word_trigger_;};
  uint32_t GetNumFIFOWarnings() {return num_word_fifo_warning_;};
  uint32_t GetNumTimestampWords() {return num_word_tstamp_;};
  uint32_t GetBytesSent() {return bytes_sent_;};
  uint64_t GetRunTime() {return (last_timestamp_ - first_timestamp_);};


  void SetDryRun(bool status) {dry_run_ = status;};

  bool GetDryRun() {return dry_run_;};

protected:
  /** Data collector function into the queue. Runs on it's own thread**/
  void ClientCollector();

  /** Data transmitter **/
  void ClientTransmitter();

  void DumpPacket(uint32_t* buffer, uint32_t tot_size);

private:
  static void * ClientCollectorFunc(void * This) {((PTBReader *)This)->ClientCollector(); return NULL;}
  static void * ClientTransmitterFunc(void * This) {((PTBReader *)This)->ClientTransmitter(); return NULL;}

  unsigned short tcp_port_;
  std::string tcp_host_;
  uint32_t packet_rollover_;

  TCPSocket *socket_;
  pthread_t client_thread_collector_;
  pthread_t client_thread_transmitter_;
  pthread_mutex_t lock_;

  bool ready_;


//#ifdef ARM_POTHOS
//  std::queue<DMABuffer*> buffer_queue_;
//endif
#ifdef ARM_XDMA
// declare a bunch of variables that are common to the program
  std::queue<uint32_t*> buffer_queue_;
  struct xdma_dev xdma_device;
  struct xdma_chan_cfg xdma_dst_cfg;
  struct xdma_buf_info xdma_buf;
  struct xdma_transfer xdma_trans;
  uint32_t *dma_buffer_;

#endif

#ifdef ARM_MMAP
  // Keeps frames stored
  LocalRegister control_register_;
  LocalRegister data_register_;
  std::queue<uint32_t*> buffer_queue_;
  uint32_t * memory_pool_;
  void * mapped_data_base_addr_;
#endif

  // A few auxiliary constants
  static const uint32_t max_packet_size = 0xFFFF;
  static const uint32_t frame_size_bits = 0x80; // the buffer is 128 bits
  static const uint32_t frame_size_bytes = 0x10; // 16 bytes
  static const uint32_t frame_size_u32 = 0x4; // 4xuint32_t
  // This is the buffer size in number of frames
  static const uint32_t buffer_size = 2*1024*1024;//1024*1024;


  // Warning pre_computed words
  static const uint32_t WARN_TIMEOUT = 0x04000000;
  static const uint32_t WARN_UNKNOWN_DATA = 0x02000000;


  // A few more constants that are important
  // This is actually
  static const uint32_t fw_version_ = 0x4;

  // Frame sequence number
  uint32_t seq_num_;

  bool fragmented_;
  bool keep_transmitting_;
  //bool ready_to_send_;
  //bool first_ts_;
  bool keep_collecting_;
  uint64_t time_rollover_;

  // Debugging and control variables
uint32_t timeout_cnt_;
const uint32_t timeout_cnt_threshold_ = 10000;

bool dry_run_; // Run the PTB without collecting data

  // -- Internal run statistics
  uint32_t num_microslices_;
  uint32_t num_word_counter_;
  uint32_t num_word_trigger_;
  uint32_t num_word_fifo_warning_;
  uint32_t num_word_tstamp_;
  uint32_t bytes_sent_;
  uint32_t first_timestamp_;
  uint32_t last_timestamp_;
  

  //#ifdef ARM_POTHOS
  //  pzdud_t *s2mm_;
  //#endif
};

/// 
/// This should be obsolete now.
/// 
/** Auxiliary classes
 *
 */
class evtType {
 public:
  uint8_t type;
  uint8_t trigger;
  uint64_t next;
  
};

class closer {
public:
  bool operator() (const evtType &a, const evtType &b) {
    return (a.next>b.next);
  };
};



#endif /* PTBREADER_H_ */
