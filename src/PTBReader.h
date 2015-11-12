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

  void GenerateFrame(uint32_t **buffer);

  void DumpPacket(uint32_t* buffer, uint32_t tot_size);
  /** Method used to initialize the sampler for the simulations **/
  void InitEmuSampler();
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
  std::queue<uint32_t*> buffer_queue_;

  // A few auxiliary constants
  static const uint32_t max_packet_size = 0xFFFF;
  static const uint32_t frame_size = 0x80; // the buffer is 128 bits

  // A few more constants that are important
  // This is actually
  static const uint32_t fw_version_ = 0x1;

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

  // -- Simulator parameters
  uint32_t freq_counter;
  uint32_t freq_trigA;
  uint32_t freq_trigB;
  uint32_t freq_trigC;
  uint32_t freq_trigD;
//  uint32_t freq_extTig;

  // Debugging and control variables
uint32_t timeout_cnt_;
const uint32_t timeout_cnt_threshold_ = 1000;

  std::priority_queue<evtType,std::vector<evtType>,closer> evt_queue_;
};

#endif /* PTBREADER_H_ */
