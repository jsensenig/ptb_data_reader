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
#include <stdint.h>

// Completely auxiliary clock function
uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}




PTBReader::PTBReader(bool emu) : tcp_port_(0), tcp_host_(""),
    packet_rollover_(0),socket_(0),
    client_thread_collector_(0),client_thread_transmitor_(0),ready_(false), emu_mode_(emu),
    fragmented_(false),keep_transmitting_(true),ready_to_send_(false),previous_ts_(0)

{
  printf("Creating an instance of the reader with emu %s\n",emu_mode_?"true":"false");

  if (emu_mode_) {
    printf("Filling emulator queue\n");
    InitEmuSampler();
  } else {
    printf("Not filling emulator queue\n");
  }
}

PTBReader::~PTBReader() {
  Log(debug) << "Destroying the reader." << endlog;

}

void PTBReader::StopDataTaking() {

  printf("Killing the daughter threads.\n");
  // Kill the collector thread first
  pthread_cancel(client_thread_collector_);
  // Kill the transmittor thread.
  pthread_cancel(client_thread_transmitor_);

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
    std::cout << "==> Creating collector\n" << std::endl;
    if (pthread_create(&client_thread_collector_,NULL,&(PTBReader::ClientCollectorFunc),this) != 0) {
      Log(error) << "Unable to create client thread. Failing..." << endlog;
      return;
    }
    // We are ready to do business. Start reading the DMA into the queue.
    std::cout << "==> Creating transmitter\n" << std::endl;
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

    if (tcp_port_ == 2320) {
     std::cout << "JCF: the true tcp_port_ value is 2320" << std::endl;
    } else if (tcp_port_ == 8992) {
      std::cout << "JCF: the true tcp_port_ value is 8992" << std::endl;
    } else {
      std::cout << "JCF: the true tcp_port_ value is neither 2320 nor 8992, cout prints it as " << tcp_port_ << std::endl;
    }

    try{
      std::cout << "Opening socket connection : " << tcp_host_ << " " << (unsigned short)tcp_port_ << std::endl;
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
      throw;
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
  std::cout << "Starting collector\n" << std::endl;
  while(true) {
    if (emu_mode_) {
      // Reserve the memory for one frame (128 bits long)
      uint32_t* frame = (uint32_t*)calloc(4,sizeof(uint32_t));
      printf("Generating the frame\n");
      GenerateFrame(&frame);
      printf("Frame generated\n");
      pthread_mutex_lock(&lock_);
      buffer_queue_.push(frame);
      pthread_mutex_unlock(&lock_);

    } else {
      Log(warning) << "Real data taking mode is not ready yet. " << endlog;
    }
  }

}


void PTBReader::ClientTransmiter() {
  printf("Starting transmitter\n");
  printf("previous : %u roll %u \n",previous_ts_,time_rollover_);
  // FIXME: Finish implementation
  // Fetch a frame from the queue
  seq_num_ = 1;

  // The whole methd runs on an infinite loop with a control variable
  // That is set from the main thread.
  while(keep_transmitting_) {

    // Logic was dramatically changed.
    // FW decides when the packet is ready.
    // When DMA brings the TS word it is sign that packet has to be sent

    // 2 conditions to be careful of:
    //
    // 1. If time time based rollover is reached.
    // 2. If the size based rollover is reached.


    // The size should *never* go beyond the 64 kb
    uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);

    // Start by generating a header


    // Primary loop is at 32bit word packets
    static const uint32_t max_packet_num = max_packet_size/sizeof(uint32_t);
    uint32_t ipck = 0,iframe = 0;
    bool carry_on = true;
    eth_buffer[0] = (fw_version << 24 ) | ((seq_num_  << 16) & 0xFF);
    printf("Temp HEADER : %x (%x %x)\n",eth_buffer[0],(uint32_t)fw_version,(uint32_t)seq_num_);
    ipck += 1;
    // while a packet_sending_condition is not reached
    // keep the loop going...
    while(carry_on) {


      // Grab a frame
      if (buffer_queue_.size() == 0) {
        continue;
      }
      printf("Transmitting data...\n");
      pthread_mutex_lock(&lock_);
      uint32_t *frame = buffer_queue_.front();
      buffer_queue_.pop();
      pthread_mutex_unlock(&lock_);

      // If we still didn't get the first timestamp just drop the packages
      printf("--> Frame1 %x \n",frame[0]);

      /// -- Check if it is a TS frame
      if ((frame[0] >> 29 & 0x7) == 0x7) {
        // This is a timestamp word.
        // Check if this is the first TS after StartRun
        printf("Timestamp frame...\n");
        current_ts_ = (frame[1] << 32) | frame[2];

        if (first_ts_) {
          // First TS after Run start
          previous_ts_ = (frame[1] << 31) | frame[2];
          first_ts_ = false;
          continue;
        } else if (current_ts_ >= (previous_ts_ + time_rollover_)) {
          printf("Sending the packet\n");
          // Check if we are ready to send the packet (reached the time rollover).
          // Close the packet
          eth_buffer[ipck] = frame[0];
          eth_buffer[ipck+1] = frame[1];
          eth_buffer[ipck+2] = frame[2];
          ipck +=3;
          carry_on = false;
          fragmented_ = false;
          // break out of the cycle
          break;
          // FIXME: Add the situation in which we arrive to the time rollover.

        } else {
          // Not ready to send yet. Drop this frame and get another
          continue;
        }
      }

      /// --  Not a TS packet...just accumulate it
      // Start another index to move in words of 32 bits
      // These can loop all the
      printf("--> Frame2 %x \n",frame[0]);
      eth_buffer[ipck] = frame[0];
      ipck += 1;
      // trim the data depending on the type
      if ((frame[0] >> 29 & 0x7) == 0x1) { // counter word. Just assign as it is
        for (size_t k = 0; k < 3; ++k) {
          eth_buffer[ipck+k] = frame[k+1];
        }
        // Refresh ipck to the latest position
        ipck += 3;
      } else if ((frame[0] >> 29 & 0x7) == 0x2) {
        // trigger word: Only the first 32 bits of the payload
        // are actually needed
        eth_buffer[ipck+1] = frame[1];
        // The rest of the buffer is crap
        ipck += 1;
      }

      // Frame completed. check if we can wait for another or keep collecting
      iframe += 1;

      // Size Rollover reached. Produce a fragmented block;
      // Keep collecting only until the next TS word
      if (iframe == packet_rollover_) {
        fragmented_ = true;
        break;
      }

    }
    printf("Broke out of the loop.\n");



    //    // Exited the packet loop.
    //    // Check if the first packet is a timestamp word
    //    if ((eth_buffer[1] >> 29 & 0x7) == 0x7) {
    //      Log(warning) << "Empty packet found. Dropping it without sending." << endlog;
    //      continue;
    //    }

    // Transmit the data.
    printf("Packet completed. Calculating the checksum.\n");

    // Write the size (in bytes)
    uint16_t packet_size = (ipck*sizeof(uint32_t));
    eth_buffer[0] |= packet_size ;

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

    /// -- Send the packet:
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

    // if we didn't have a fragmented block, update the sequence number
    // and update the timestamp to the latest one
    if (!fragmented_) {
      seq_num_++;
      previous_ts_ = current_ts_;
      fragmented_ =false;
    }


  }
  // Exited the  run loop. Return.
  Log(info) << "Exited transmission loop. Checking for queued packets." << endlog;
}

/// Everything that is under this part is mostly for simulation




void PTBReader::GenerateFrame(uint32_t **buffer) {

  printf("In GenerateFrame\n");
  printf("== [%x] \n",buffer[0][0]);
  printf("== [%x] \n",buffer[0][1]);
  uint32_t*lbuf = buffer[0];
  printf("Getting time\n");
  uint64_t now = ClockGetTime();
  std::cout << "Clock " << now << std::endl;

  // Here we decide which type of packet I am going to generate
  // It should be dependent on the frequencies that I attach to each type
  // (not using calibrations for now)

  evtType nextEvt = evt_queue_.top();
  printf("Got an event.\n");
  evt_queue_.pop();

  // If we haven't reached the time for the next event just send a TS packet.
  if (now < nextEvt.next) {
    printf("Creating a TS packet\n");
    lbuf[0] = (0x7 << 29) | (now & 0xFFFFFFF);
    lbuf[1] = (now >> 32 & 0xFFFFFFF);
    lbuf[2] = (now & 0xFFFFFFF);

    return;
  }
  printf("Creating something else\n");

  //  // Wait for the time to be ready
  //  while (now < nextEvt.next) {
  //    now = ClockGetTime();
  //  }

  // Time for an event

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
  } else {

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
  }
  evt_queue_.push(nextEvt);
}


void PTBReader::InitEmuSampler() {
  // Attach the evt frequencies -- in Hz
  printf("Starting the Sampler\n");

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

  printf("Finished the Sampler\n");

}
