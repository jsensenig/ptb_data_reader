/*
 * PTBReader.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

#include "CompilerOptions.h"

#include <string>
#include <iostream>

#if defined(LOCKFREE)
#include "atomicops.h"
#include "readerwriterqueue.h"
#else
#include <queue>
#endif

extern "C" {
#include<pthread.h>
#include <stdio.h>
}

#include <cstdlib>
#include <cstring>
#include <cstdint>

#if defined(ARM_XDMA)
#include "xdma.h"
#elif defined(ARM_MMAP) // needed to use the LocalRegister structure
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
  
  /// Bunch of structures that help manipulating the data.
  /// These *must* be kept in sync with lbne-raw-data

    /// Header of ethernet packet
    struct Header {
    
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;

    typedef uint8_t  format_version_t;
    typedef uint8_t  sequence_id_t;
    typedef uint16_t block_size_t;
    
    block_size_t     block_size     : 16;
    sequence_id_t    sequence_id    : 8;
    format_version_t format_version : 8;
    
    static size_t const size_words = sizeof(data_t);
    
    static data_size_t const num_bits_size        = 16;
    static data_size_t const num_bits_sequence_id = 8;
    static data_size_t const num_bits_format      = 8;
  };

  /// Internal warning word
  struct Word_Warning {
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;
    
    typedef uint8_t  warning_type_t;
    typedef uint8_t  data_packet_type_t;

    uint32_t padding                        : 24;
    warning_type_t warning_type             : 5;
    data_packet_type_t data_packet_type     : 3;
    
    static size_t const size_words = sizeof(data_t);
    static data_size_t const num_bits_padding     = 24;
    static data_size_t const num_bits_warning     = 5;
    static data_size_t const num_bits_packet_type = 3;
  };

  /// Frame header : Common header to everything except warning words
  struct Word_Header {
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;
    
    typedef uint8_t  data_packet_type_t;
    typedef uint32_t short_nova_timestamp_t;
    
    // The order of the data packet type and the timestamp have been
    // swapped to reflect that it's the MOST significant three bits in
    // the payload header which contain the type. I've also added a
    // 2-bit pad to reflect that the 2 least significant bits are unused.
    
    uint8_t padding                             : 2;
    short_nova_timestamp_t short_nova_timestamp : 27;
    data_packet_type_t     data_packet_type     : 3;
    
    static size_t const size_bytes = sizeof(data_t);
    static size_t const size_u32 = sizeof(data_t)/sizeof(uint32_t);
    
    static data_size_t const num_bits_padding       = 2;
    static data_size_t const num_bits_short_tstamp  = 27;
    static data_size_t const num_bits_packet_type   = 3;

    // This function converts the TS rollover into a full TS
    // it takes the payload of the timestamp word that came after this word
    // (the microslice border)
    // and calculates the offset
    // to use the full timestamp received before use the other method (pre)
    uint64_t get_full_timestamp_post(uint64_t ts_ref) {
      if ((ts_ref & 0x7FFFFFF) == short_nova_timestamp) {
        // they are equal. This word is right on the border of the microslice
        return ts_ref;
      } else if ((ts_ref & 0x7FFFFFF) > short_nova_timestamp) {
        // there was no bit rollover in between
        return ts_ref - ((ts_ref & 0x7FFFFFF) - short_nova_timestamp);
      } else {
        // it rolled over.
        // Be sure of the values being set
        return ts_ref - ((ts_ref & 0x7FFFFFF) + (0x7FFFFFF - short_nova_timestamp));
      }
    }
    // Does the same as the previous but with the timestamp that came before
    // They should be equivalent but need to check to confirm
    uint64_t get_full_timestamp_pre(uint64_t ts_ref) {
      if ((ts_ref & 0x7FFFFFF) == short_nova_timestamp) {
        // they are equal. This word is right on the border of the microslice
        return ts_ref;
      } else if ((ts_ref & 0x7FFFFFF) > short_nova_timestamp) {
        // it rolled over. Has to sum the short and the difference
        // that takes for the reference to roll over
        return ts_ref + (0x7FFFFFF - ((ts_ref & 0x7FFFFFF))) + short_nova_timestamp;
      } else {
        // it didn't roll over. Just add the difference of the rollovers
        return ts_ref + (short_nova_timestamp - (ts_ref & 0x7FFFFFF));
      }
    }


  };

  /// Counter payload description
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
    //FIXME: The panels should be remapped so that bsu_cl would be all together
    counter_set_t bsu_cl1    : 6;
    counter_set_t bsu_extra  : 1;
    counter_set_t bsu_cl2    : 7;
    counter_set_t bsu_rl     : 10;
    // Just ignore the rest of the word
    counter_set_t padding    : 30;
    
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
    static size_t const size_words_ptb_u32 = 3;
    static size_t const size_words_ptb_bytes = size_words_ptb_u32*sizeof(uint32_t);
    // The offset to grab the data
    static size_t const ptb_offset_u32 = 0;
    static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
    
    // The payload position offset from the top of the frame (header + discard)
    static size_t const payload_offset_u32 = 1+ptb_offset_u32;
    static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);

    counter_set_t get_bsu_cl() {return ((bsu_cl2 << 10) | (bsu_cl1));};
  };

  /// Trigger description
  struct TriggerPayload {
    typedef uint32_t trigger_type_t;
    typedef uint16_t data_size_t;

    // The 16 lsb are padding. No information is passed there
    trigger_type_t padding_low : 16;

    // This is to be remapped so that calib words can be OR-ed with
    // calibration words
    // [8 : t_type][4 : muon_type][4 : calib type]
    trigger_type_t trigger_id_calib: 4; // which of the calibration channels 
    trigger_type_t trigger_id_muon : 4; // which of the muon triggers
    trigger_type_t trigger_type    : 5; // the 5 msb are the trigger type
    trigger_type_t padding_high    : 3; // this makes the information byte aligned
    
    static data_size_t const num_bits_padding_low     = 16;
    static data_size_t const num_bits_trigger_id_calib= 4;
    static data_size_t const num_bits_trigger_id_muon = 4;
    static data_size_t const num_bits_trigger_type    = 5;
    static data_size_t const num_bits_padding_high    = 3;
    
    // The logic below this point is a bit different since now multiple
    // triggers can be ID-ed in the same word
    // Better to ask for specific types


    // ID the trigger types for fugure reference
    static trigger_type_t const calibration = 0x00;
    static trigger_type_t const muon        = 0x10;
    static trigger_type_t const ssp         = 0x08;
    // -- This should probably be split into RCE and then RCE types
    static trigger_type_t const rce_1       = 0x01;
    static trigger_type_t const rce_2       = 0x02;
    static trigger_type_t const rce_12      = 0x03;
    static trigger_type_t const rce_3       = 0x04;
    static trigger_type_t const rce_13      = 0x05;
    static trigger_type_t const rce_23      = 0x06;
    static trigger_type_t const rce_123     = 0x07;
    
    // C1 : 1000 : 0x8
    // C2 : 0100 : 0x4
    // C3 : 0010 : 0x2
    // C4 : 0001 : 0x1
    static trigger_type_t const C1 = 0x8;
    static trigger_type_t const C2 = 0x4;
    static trigger_type_t const C3 = 0x2;
    static trigger_type_t const C4 = 0x1;

    // TA : 1000
    // TB : 0100
    // TC : 0010
    // TD : 0001
    static trigger_type_t const TA = 0x8;
    static trigger_type_t const TB = 0x4;
    static trigger_type_t const TC = 0x2;
    static trigger_type_t const TD = 0x1;


    static size_t const size_bytes = sizeof(uint32_t);
    static size_t const size_u32 = size_bytes/sizeof(uint32_t);

    // The number of u32 that have to offset from the PTB packet to grab the data that matters
    // In this case the two lsInts are empty and can be offset
    static size_t const ptb_offset_u32 = 2;
    static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
    
    // The payload position offset from the top of the frame (header + discard)
    static size_t const payload_offset_u32 = 1+ptb_offset_u32;
    static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);
    


    ///Bunch of auxiliary functions to help parse the word
    ///
    bool has_muon_trigger() {
      return ((trigger_type & muon) != 0x0);
    }
    bool has_rce_trigger() {
      return ((trigger_type & rce_123) != 0x0);
    }
    bool has_ssp_trigger() {
      return ((trigger_type & ssp) != 0x0);
    }
    bool has_calibration() {
      return ((trigger_type & calibration) != 0x0);
    }


    /// Test for the different calibration types
    ///
    bool has_C1() {
      return ((trigger_id_calib & C1) != 0);
    }
    bool has_C2() {
      return ((trigger_id_calib & C2) != 0);
    }
    bool has_C3() {
      return ((trigger_id_calib & C3) != 0);
    }
    bool has_C4() {
      return ((trigger_id_calib & C4) != 0);
    }

    /// Test the different muon trigger types
    ///
    bool has_muon_TA() {
      return ((trigger_id_muon & TA) != 0);
    }
    bool has_muon_TB() {
      return ((trigger_id_muon & TB) != 0);
    }
    bool has_muon_TC() {
      return ((trigger_id_muon & TC) != 0);
    }
    bool has_muon_TD() {
      return ((trigger_id_muon & TD) != 0);
    }


    // Add a function that can be used to parse the trigger payload
    //FIXME: This should be considered obsolete and removed
    // in a near future

    static std::string getTriggerName(trigger_type_t trigger_type) {
      switch (trigger_type) {
      case calibration:
	return "calibration";
	break;
      case rce_1:
	return "rce_1";
	break;
      case rce_2:
	return "rce_2";
	break;
      case rce_3:
	return "rce_3";
	break;
      case rce_12:
	return "rce_12";
	break;
      case rce_13:
	return "rce_13";
	break;
      case rce_23:
	return "rce_23";
	break;
      case rce_123:
	return "rce_123";
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
#ifdef ENABLE_FRAG_BLOCKS
  uint32_t getPacketRollover() const {
    return packet_rollover_;
  }

  void setPacketRollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }
#endif
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
  void CloseConnection();

  bool TestSocket();

  bool getConnectionValid() const {
    // This might not be necessarily the best option
    return (socket_ != NULL);
  }
  
  // -- Statistics methods:
  uint32_t GetNumFragmentsSent() {return num_eth_fragments_;};
  uint32_t GetNumCounterWords() {return num_word_counter_;};
  uint32_t GetNumTriggerWords() {return num_word_trigger_;};
  uint32_t GetNumFIFOWarnings() {return num_word_fifo_warning_;};
  uint32_t GetNumTimestampWords() {return num_word_tstamp_;};
  uint32_t GetBytesSent() {return bytes_sent_;};


  void SetDryRun(bool status) {dry_run_ = status;};

  bool GetDryRun() {return dry_run_;};

  bool GetErrorState() {return error_state_;};

  std::string GetErrorMessages() {return error_messages_;};

protected:
  /** Data collector function into the queue. Runs on it's own thread**/
  void ClientCollector();

  /** Data transmitter **/
  void ClientTransmitter();

  void DumpPacket(uint32_t* buffer, uint32_t tot_size);

private:
  static void * ClientCollectorFunc(void * This) {((PTBReader *)This)->ClientCollector(); return NULL;}
  static void * ClientTransmitterFunc(void * This) {((PTBReader *)This)->ClientTransmitter(); return NULL;}

  // -- Structures for data socket connection

  unsigned short tcp_port_;
  std::string tcp_host_;

  TCPSocket *socket_;

  pthread_t client_thread_collector_;
  pthread_t client_thread_transmitter_;

#ifndef LOCKFREE
  pthread_mutex_t lock_;
#endif

#ifdef ENABLE_FRAG_BLOCKS
  uint32_t packet_rollover_;
#endif
  bool ready_;


#if defined(ARM_XDMA)
// declare a bunch of variables that are common to the program
#if defined(LOCKFREE)
  moodycamel::ReaderWriterQueue<uint32_t*> buffer_queue_;
  #else
  std::queue<uint32_t*> buffer_queue_;
#endif
  struct xdma_dev xdma_device;
  struct xdma_chan_cfg xdma_dst_cfg;
  struct xdma_buf_info xdma_buf;
  struct xdma_transfer xdma_trans;
  uint32_t *dma_buffer_;

#elif defined(ARM_MMAP)

  // Keeps frames stored
  LocalRegister control_register_;
  LocalRegister data_register_;
#if defined(LOCKFREE)
  moodycamel::ReaderWriterQueue<uint32_t*> buffer_queue_;
  #else
  std::queue<uint32_t*> buffer_queue_;
#endif
  uint32_t * memory_pool_;
  void * mapped_data_base_addr_;
#else
#error DMA mode not specified
#endif


  // A few auxiliary constants

  static const uint32_t eth_buffer_size_u32   = 0xFFFF; // Max possible ethernet packet
  static const uint32_t frame_size_bits   = 0x80;   // the buffer is 128 bits
  static const uint32_t frame_size_bytes  = 0x10;   // 16 bytes
  static const uint32_t frame_size_u32    = 0x4;    // 4xuint32_t
  // This is the buffer size in number of frames
  // Could easily do something else, or even set this as a configuration
  // parameter
  // FIXME: Change to a fhicl param
  ///! Number of frames possible to buffer in the RAM memory
  /// Increased from 2M to 8M => 32MB
  static const uint32_t buffer_size_frames = 8*1024*1024;


  // Warning pre_computed words
  static const uint32_t WARN_TIMEOUT = 0x04000000;
  static const uint32_t WARN_UNKNOWN_DATA = 0x02000000;


  // A few more constants that are important
  static const uint32_t fw_version_ = 0x5;

  // Frame sequence number
  uint32_t seq_num_;
#ifdef ENABLE_FRAG_BLOCKS
  bool fragmented_;
#endif
  bool keep_transmitting_;
  bool keep_collecting_;
  uint64_t time_rollover_;

  // -- Debugging and control variables
  // timeouts don't make sense with MMAP
#if defined(ARM_XDMA)
uint32_t timeout_cnt_;
const uint32_t timeout_cnt_threshold_ = 10000;
#endif

bool dry_run_; // Run the PTB without collecting data
bool error_state_;
std::string error_messages_;

  // -- Internal run statistics
  uint32_t num_eth_fragments_;
  uint32_t num_word_counter_;
  uint32_t num_word_trigger_;
  uint32_t num_word_fifo_warning_;
  uint32_t num_word_tstamp_;
  uint32_t bytes_sent_;
  
};


#endif /* PTBREADER_H_ */
