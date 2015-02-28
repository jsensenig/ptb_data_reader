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
};

#include <cstdlib>
#include <cstring>
class TCPSocket;

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


/**
 * This class is responsible for reading the DMA and transferring the data to the server.
 * \class PTBReader
 *
 * \details This class is kept simple on purpose. It does not talk with the [ConfigServer]
 * by design and all it does is really collect the data from the memory map of the DMA,
 * assembles the TCP packets and sends them to the server.
 *
 */
class PTBReader {
public:

    struct Header {

      typedef uint32_t data_t;

      typedef uint8_t  format_version_t;
      typedef uint8_t  sequence_id_t;
      typedef uint16_t block_size_t;

      // JCF, Jul-28-15

      // The order of these variables have been reversed to reflect that
      // the block size takes up the two least significant bytes, the
      // sequence ID the second most significant byte, and the format
      // version the most significant byte, of the four-byte microslice header

      block_size_t     block_size     : 16;
      sequence_id_t    sequence_id    : 8;
      format_version_t format_version : 8;

      static size_t const size_words = sizeof(data_t);

      //static constexpr size_t raw_header_words = 1;
      //data_t raw_header_data[raw_header_words];
    };

    struct Payload_Header {
        typedef uint32_t data_t;

        typedef uint8_t  data_packet_type_t;
        typedef uint32_t short_nova_timestamp_t;

        // The order of the data packet type and the timestamp have been
        // swapped to reflect that it's the MOST significant three bits in
        // the payload header which contain the type. I've also added a
        // 1-bit pad to reflect that the least significant bit is unused.

        uint8_t padding : 1;
        short_nova_timestamp_t short_nova_timestamp : 28;
        data_packet_type_t     data_packet_type     : 3;

        static size_t const size_words = sizeof(data_t);
    };

    typedef Header::block_size_t microslice_size_t;

    //the size of the payloads (neglecting the Payload_Header)
    // FIXME: Since everythig is being worked in bytes the sizes can be trimmed
    static microslice_size_t const payload_size_counter   = 13; //104 bit payload. The smallest that contains all 97 counters
    static microslice_size_t const payload_size_trigger   = 1 * sizeof(uint32_t); //32-bit payload
    static microslice_size_t const payload_size_timestamp = 2 * sizeof(uint32_t); //64-bit payload
    static microslice_size_t const payload_size_selftest  = 1 * sizeof(uint32_t); //32-bit payload
    static microslice_size_t const payload_size_checksum  = 0 * sizeof(uint32_t); //32-bit payload
    /* static microslice_size_t const payload_size_counter   = 4 * sizeof(uint32_t); //96-bit payload */
    /* static microslice_size_t const payload_size_trigger   = 1 * sizeof(uint32_t); //32-bit payload */
    /* static microslice_size_t const payload_size_timestamp = 2 * sizeof(uint32_t); //64-bit payload */
    /* static microslice_size_t const payload_size_selftest  = 1 * sizeof(uint32_t); //32-bit payload */
    /* static microslice_size_t const payload_size_checksum  = 0 * sizeof(uint32_t); //32-bit payload */

    //The types of data words
    static const Payload_Header::data_packet_type_t DataTypeSelftest  = 0x0; //0b000
    static const Payload_Header::data_packet_type_t DataTypeCounter   = 0x1; //0b001
    static const Payload_Header::data_packet_type_t DataTypeTrigger   = 0x2; //0b010
    static const Payload_Header::data_packet_type_t DataTypeChecksum  = 0x4; //0b100
    static const Payload_Header::data_packet_type_t DataTypeTimestamp = 0x7; //0b111


  PTBReader(bool emu = false);
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
    //std::cout << "====> Received port " << tcpPort << std::endl;
    tcp_port_ = tcpPort;
    //std::cout << "====> Set port " << tcp_port_ << std::endl;
  }

  uint32_t getPacketRollover() const {
    return packet_rollover_;
  }

  void setPacketRollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }

  /**
   * Stop the client threads without touching the mutex.
   * Important in the case that one wants to delete the object without having start a run.
   */
  void ClearThreads();
  /** Function called by the manager to cleanly stop the data transmission and connection
   *
   */
  void StopDataTaking();

  /** Starts a new thread that is continuously polling the memory **/
  void StartDataTaking();

  /**
   * Resets the buffers. The run should already be stopped.
   */
  void ResetBuffers();

  void InitConnection(bool force = false);

  bool isEmuMode() const {
    return emu_mode_;
  }

  void setEmuMode(bool emuMode) {
    emu_mode_ = emuMode;
  }

  uint64_t getTimeRollover() const {
    return time_rollover_;
  }

  void setTimeRollover(uint64_t timeRollover) {
    time_rollover_ = timeRollover;
  }

  bool getConnectionValid() const {
    return (socket_ != NULL);
  }

protected:
  /** DMA data collector function into the queue. Runs on it's own thread**/
  //static void* ClientCollectorFunc(void *args);
  void ClientCollector();

  /** Reader from the queue that is responsible for packet construction and transmission **/
  //static void* ClientTransmitor(void *args);
  void ClientTransmiter();

  void DumpPacket(uint32_t* buffer, uint32_t tot_size);

private:
  static void * ClientCollectorFunc(void * This) {((PTBReader *)This)->ClientCollector(); return NULL;}
  static void * ClientTransmitorFunc(void * This) {((PTBReader *)This)->ClientTransmiter(); return NULL;}

  unsigned short tcp_port_;
  std::string tcp_host_;
  uint32_t packet_rollover_;

  TCPSocket *socket_;
  pthread_t client_thread_collector_;
  pthread_t client_thread_transmitor_;
  pthread_mutex_t lock_;

  bool ready_;

  // Keeps frames stored
  std::queue<uint8_t*> buffer_queue_;

  // A few auxiliary constants
  static const uint32_t max_packet_size = 0xFFFF;
  static const uint32_t frame_size_bits = 0x80; // the buffer is 128 bits
  static const uint32_t frame_size_bytes = 0x10; // 16 bytes
  static const uint32_t frame_size_uint = 0x4; // 4xuint32_t
  
  // A few more constants that are important
  // This is actually
  static const uint32_t fw_version_ = 0x3;

  // Frame sequence number
  uint32_t seq_num_;

  bool emu_mode_;
  bool fragmented_;
  bool keep_transmitting_;
  //bool ready_to_send_;
  bool first_ts_;
  bool keep_collecting_;
  // Previous timestamp
  uint64_t previous_ts_;
  uint64_t time_rollover_;
  uint64_t current_ts_;


  // Debugging and control variables
uint32_t timeout_cnt_;
const uint32_t timeout_cnt_threshold_ = 1000;

};

#endif /* PTBREADER_H_ */
