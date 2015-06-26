/*
 * PTBReader.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

#include <string>

class ConfigServer;

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

  uint32_t getTcpPort() const {
    return tcp_port_;
  }

  void setTcpPort(uint32_t tcpPort) {
    tcp_port_ = tcpPort;
  }

  uint32_t getPacketRollover() const {
    return packet_rollover_;
  }

  void setPacketRollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }

  /** Function called by the manager to cleanly stop the data transmission and connection
   *
   */
  void StopDataTaking() { };

  void StartDataTaking() { };

private:

  uint32_t tcp_port_;
  std::string tcp_host_;
  uint32_t packet_rollover_;


  bool ready_;
};

#endif /* PTBREADER_H_ */
