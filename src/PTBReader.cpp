/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "PTBReader.h"
#include "Logger.h"
#include "PracticalSocket.h"

#include <sys/time.h>

// Completely auxiliary clock function
uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}




PTBReader::PTBReader(bool emu) : tcp_port_(0), tcp_host_(""),
    packet_rollover_(0),socket_(0),
    client_thread_collector_(0),client_thread_transmitor_(0),ready_(false), emu_mode_(emu)
{
  Log(debug) << "Creating an instance of the reader." << endlog;

  if (emu_mode_) {
    void InitEmuSampler();
  }
}

PTBReader::~PTBReader() {
  Log(debug) << "Destroying the reader." << endlog;

}

void PTBReader::StopDataTaking() {

  pthread_mutex_destroy(&lock_);


}

void PTBReader::StartDataTaking() {
  if (ready_) {
    // Init the mutex for the queue
    if (pthread_mutex_init(&lock_, NULL) != 0)
    {
      Log(error) << "\n Failed to create the mutex for the data queue\n" << endlog;
      return;
    }

    // We are ready to do business. Start reading the DMA into the queue.
    if (pthread_create(&client_thread_collector_,NULL,&(PTBReader::ClientCollectorFunc),this) != 0) {
      Log(error) << "Unable to create client thread. Failing..." << endlog;
      return;
    }
    // We are ready to do business. Start reading the DMA into the queue.
    if (pthread_create(&client_thread_transmitor_,NULL,&(PTBReader::ClientTransmitorFunc),this) != 0) {
      Log(error) << "Unable to create client thread. Failing..." << endlog;
      return;
    }

  } else {
    Log(error) << "Calling to start taking data connection is not ready yet." << endlog;

  }
}

void PTBReader::InitConnection() {

  // Check if the server data is set up, and compute
  if (tcp_host_ != "" && tcp_port_ != 0) {

    try{
      socket_ = new TCPSocket(tcp_host_,tcp_port_);

      if (socket_ == NULL) {
        Log(error) << "Unable to establish the client socket. Failing." << endlog;
        throw;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    catch(SocketException &e) {
      Log(error) << "Socket exception caught : " << e.what() << endlog;
    }
    catch(std::exception &e) {
      Log(error) << "STD exception caught : " << e.what() << endlog;
    }
    catch(...) {
      Log(error) << "Unknown exception caught." << endlog;
    }
  } else {
    Log(error) << "Calling to start connection without defining connection parameters." << endlog;
  }

}

void PTBReader::ClientCollector() {

  if (emu_mode_) {
    // Reserve the memory for one frame
    uint32_t* frame = (uint32_t*)calloc(4,sizeof(uint32_t));
    GenerateFrame(&frame);
    pthread_mutex_lock(&lock_);
    buffer_queue_.push(frame);
    pthread_mutex_unlock(&lock_);

  } else {
    Log(warning) << "Real data taking mode is not ready yet. " << endlog;
  }

}


void PTBReader::ClientTransmiter() {
  // FIXME: Finish implementation
  // Fetch a frame from the queue


  // The whole methd runs on an infinite loop with a control variable
  // That is set from the main thread.
  while(keep_transmitting_) {

    // There are 3 conditions to complete a tramission packet:
    //
    // 1. The rollover is reached (full packet)
    // --    j == rollover
    // 2. The 64kb size is reached (fragmented block)
    // -- j ==  0xFFFFFFFF/(dma_pkt_size)
    // 3. The rollover time is reached (full packet)
    // tn == (t0-1)

    // The size should *never* go beyond the 64 kb
    uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);

    // Start by generating a header


    // Primary loop is at 32bit word packets
    static const uint32_t max_packet_num = max_packet_size/sizeof(uint32_t);
    uint32_t ipck = 0,iframe = 0;
    bool carry_on = true;
    eth_buffer[0] = (fw_version << 28 ) | (seq_num_  << 20);
    ipck += 1;
    while(carry_on) {

      // Grab a frame
      pthread_mutex_lock(&lock_);
      uint32_t *frame = buffer_queue_.front();
      buffer_queue_.pop();
      pthread_mutex_unlock(&lock_);

      // Check if it is a TS frame
      if ((frame[0] >> 27 | 0x7) == 0x7) {
        // This is a timestamp word.
        // Close the packet
        eth_buffer[ipck] = frame[0];
        eth_buffer[ipck+1] = frame[1];
        eth_buffer[ipck+2] = frame[2];
        ipck +=3;
        carry_on = false;
        fragmented_ = false;
        // break out of the cycle
        break;
      }

      // Start another index to move in words of 32 bits
      // These can loop all the
      eth_buffer[ipck] = frame[0];
      ipck += 1;
      // trim the data depending on the type
      if ((frame[0] >> 27 | 0x7) == 0x1) { // counter word. Just assign as it is
        for (size_t k = 0; k < 3; ++k) {
          eth_buffer[ipck+k] = frame[k+1];
        }
        ipck += 3;
        // Refresh ipck to the latest position
      } else if ((frame[0] >> 27 | 0x7) == 0x2) {
        // trigger word: Only the first 32 bits are actually needed at all
        eth_buffer[ipck+1] = frame[1];
        // The rest of the buffer is crap
        ipck += 1;
      }

      // Frame completed. check if we can wait for another or keep collecting
      iframe += 1;

      // If we are 128 bits away from the limit close it due to reaching the size


      // Rollover reached
      if (iframe == packet_rollover_) {
        carry_on = false;
        fragmented_ = false;
        break;
      }


      // Reached the max size without having yet a timestamp
      if (ipck == max_packet_num - 8) {
        carry_on = false;
        fragmented_ = true;
        break;
      }

    }


    // Exited the packet loop.
    // Check if the first packet is a timestamp word
    if ((eth_buffer[1] >> 29 & 0x7) == 0x7) {
      Log(warning) << "Empty packet found. Dropping it without sending." << endlog;
      continue;
    }

    // Transmit the data.
    Log(verbose) << "Packet completed. Calculating the checksum." << endlog;

    // Write the size (in bytes)
    uint16_t packet_size = (ipck*sizeof(uint32_t));
    eth_buffer[1] |= packet_size ;
    // Calculate the checksum
    //
    // -- Followed the recipe in
    //    https://en.wikipedia.org/wiki/BSD_checksum
    uint16_t checksum;
    uint8_t*ch_buff = reinterpret_cast<uint8_t*>(eth_buffer);
    for (uint32_t i = 0; i < ipck*4; ++i) {
      // Recast as uint8_t
      checksum = (checksum >> 1) + ((checksum & 0x1) << 15) ;
      checksum += ch_buff[i];
      checksum &= 0xFFFF;
    }
    // Calculate 16 bit checksum of the packet
    // Add the checksum to the last packet
    eth_buffer[ipck] = (0x4 << 29) | checksum;

    // Send the packet:
    try {
    socket_->send(eth_buffer,ipck);
    }
    catch(SocketException &e) {
      Log(error) << "Cocket exception caught : " << e.what() << endlog;
      // Retry
      socket_->send(eth_buffer,ipck);
    }
    // Clear out the memory
    free(eth_buffer);
  }
  // Exited the  run loop. Return.
  Log(info) << "Exited transmission loop. Checking for queued packets." << endlog;
}

/// Everything that is under this part is mostly for simulation




void PTBReader::GenerateFrame(uint32_t **buffer) {
  uint32_t*lbuf = buffer[0];
  uint64_t now = ClockGetTime();

  // Here we decide which type of packet I am going to generate
  // It should be dependent on the frequencies that I attach to each type
  // (not using calibrations for now)

  evtType nextEvt = evt_queue_.top();
  evt_queue_.pop();

  // Wait for the time to be ready
  while (now < nextEvt.next) {
    now = ClockGetTime();
  }

  // Write the frame
  lbuf[0] = ((nextEvt.type & 0x7) << 29) | ((now & 0xFFFFFFF) << 1) | 0x0;
  if (nextEvt.type == 0x1) {
    // Generate 3 random numbers and assign them
    uint32_t val = rand();
    lbuf[0] |= ((val >> 30) & 0x1);
    lbuf[1] = val;
    lbuf[2] = rand();
    lbuf[3] = rand();
  } else {
    // The 5 msb are the trigger word mask (10000 for muon counters)
    lbuf[1] = (0x10 << 27) | ((nextEvt.trigger & 0xF) << 23) | 0x0;
  }

  // Put it back again in the queue
  if (nextEvt.type == 0x1) {
    nextEvt.next =now + (1.0/freq_counter)*1000000UL;;
  }
  switch(nextEvt.trigger) {

  case 0x8:
    nextEvt.next = now + (1.0/freq_trigA)*1000000UL;
    break;
  case 0x4:
    nextEvt.next = now + (1.0/freq_trigB)*1000000UL;
    break;
  case 0x2:
    nextEvt.next = now + (1.0/freq_trigC)*1000000UL;
    break;
  case 0x1:
    nextEvt.next = now + (1.0/freq_trigD)*1000000UL;
    break;
  default:
    Log(error) << "Failed to figure out the evt type to requeue." << endlog;
  }
  evt_queue_.push(nextEvt);
}


void PTBReader::InitEmuSampler() {
  // Attach the evt frequencies -- in Hz

  freq_counter = 1000;
  freq_trigA = 100;
  freq_trigB = 20;
  freq_trigC = 30;
  freq_trigD = 10;

  uint64_t now = ClockGetTime();
  evtType evt;
  // Counter word
  evt.type = 0x1;
  evt.trigger = 0x0;
  evt.next = now + (1.0/freq_counter)*1000000UL;
  evt_queue_.push(evt);

  // Trigger word
  evt.type = 0x2;
  evt.trigger = 0x8;
  evt.next = now + (1.0/freq_trigA)*1000000UL;
  evt_queue_.push(evt);

  evt.type = 0x2;
  evt.trigger = 0x4;
  evt.next = now + (1.0/freq_trigB)*1000000UL;
  evt_queue_.push(evt);

  evt.type = 0x2;
  evt.trigger = 0x2;
  evt.next = now + (1.0/freq_trigC)*1000000UL;
  evt_queue_.push(evt);

  evt.type = 0x2;
  evt.trigger = 0x8;
  evt.next = now + (1.0/freq_trigD)*1000000UL;
  evt_queue_.push(evt);

}
