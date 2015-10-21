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
#include <bitset>
#include <sstream>
#include <thread>
#include <chrono>

#ifdef ARM
#include "libxdma.h"
#endif /*ARM*/

// Completely auxiliary clock function
uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}




PTBReader::PTBReader(bool emu) : tcp_port_(0), tcp_host_(""),
    packet_rollover_(0),socket_(0),
    client_thread_collector_(0),client_thread_transmitor_(0),ready_(false), emu_mode_(emu),
				 fragmented_(false),keep_transmitting_(true),/*ready_to_send_(false),*/first_ts_(true),keep_collecting_(true),previous_ts_(0),timeout_cnt_(0)

{
  printf("Here\n");
  Log(verbose,"Creating an instance of the reader with emu %s\n",emu_mode_?"true":"false");

  if (emu_mode_) {
    Log(verbose,"Filling emulator queue");
    InitEmuSampler();
  } else {
    Log(verbose,"Not filling emulator queue");
    // Nothing should need to be done here.
  }

  // Init the mutex for the queue
  if (pthread_mutex_init(&lock_, NULL) != 0)
  {
    Log(error,"\n Failed to create the mutex for the data queue\n" );
    return;
  }

}

PTBReader::~PTBReader() {
  Log(debug,"Destroying the reader." );
  ready_ = false;

}
void PTBReader::ClearThreads() {
	Log(debug,"Killing the daughter threads.\n");
	// First stop the loops
	keep_collecting_ = false;
	std::this_thread::sleep_for (std::chrono::seconds(2));
	Log(info,"Killing collector thread.");
	// Kill the collector thread first
	pthread_cancel(client_thread_collector_);
	// Kill the transmittor thread.
	keep_transmitting_ = false;
	std::this_thread::sleep_for (std::chrono::seconds(2));
	Log(info,"Killing transmiter thread.");
	pthread_cancel(client_thread_transmitor_);

}

void PTBReader::StopDataTaking() {

  ClearThreads();
  pthread_mutex_destroy(&lock_);

  ready_ = false;
}

void PTBReader::StartDataTaking() {
  if (ready_) {

    // We are ready to do business. Start reading the DMA into the queue.
    Log(verbose, "==> Creating collector\n" );
    keep_collecting_ = true;
    if (pthread_create(&client_thread_collector_,NULL,&(PTBReader::ClientCollectorFunc),this) != 0) {
      Log(error,"Unable to create client thread. Failing..." );
      return;
    }
    // We are ready to do business. Start reading the DMA into the queue.
    Log(verbose, "==> Creating transmitter\n" );
    keep_transmitting_ = true;
    if (pthread_create(&client_thread_transmitor_,NULL,&(PTBReader::ClientTransmitorFunc),this) != 0) {
      Log(error,"Unable to create client thread. Failing..." );
      return;
    }

  } else {
    Log(error,"Calling to start taking data connection is not ready yet." );

  }
}

void PTBReader::ResetBuffers() {
  Log(warning,"Resetting the software buffers.");
  pthread_mutex_lock(&lock_);
  while (!buffer_queue_.empty()) {
    uint32_t*frame = buffer_queue_.front();
    buffer_queue_.pop();
    //delete [] frame;
  }
  pthread_mutex_unlock(&lock_);
}

void PTBReader::InitConnection() {

  // First thing is. If a connection exists. Close it
  if (socket_ != NULL) {
    Log(warning,"Destroying existing socket!");
    delete socket_;
  }

  // Check if the server data is set up, and compute
  if (tcp_host_ != "" && tcp_port_ != 0) {

    if (tcp_port_ == 2320) {
     Log(verbose, "JCF: the true tcp_port_ value is 2320" );
    } else if (tcp_port_ == 8992) {
      Log(verbose, "JCF: the true tcp_port_ value is 8992" );
    } else {
      Log(warning, "JCF: the true tcp_port_ value is neither 2320 nor 8992, cout prints it as %hu", tcp_port_ );
    }

    try{
      Log(debug, "Opening socket connection : %s : %hu",tcp_host_.c_str(),tcp_port_ );
      socket_ = new TCPSocket(tcp_host_.c_str(),tcp_port_);

      if (socket_ == NULL) {
        Log(error,"Unable to establish the client socket. Failing." );
        throw;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      throw;
    }
    catch(std::exception &e) {
      Log(error,"STD exception caught : %s",e.what() );
    }
    catch(...) {
      Log(error,"Unknown exception caught." );
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
  }

}

void PTBReader::ClientCollector() {
  Log(verbose, "Starting data collector\n" );
    if (emu_mode_) {
      while(keep_collecting_) {
      // Reserve the memory for one frame (128 bits long)
      uint32_t* frame = (uint32_t*)calloc(4,sizeof(uint32_t));
      Log(verbose, "Generating the frame\n");
      GenerateFrame(&frame);
      // If the header is something else do nothing.
      if (((frame[0] >> 29) & 0x7) == 0x0) continue;
      Log(verbose, "Frame generated\n");
      pthread_mutex_lock(&lock_);
      buffer_queue_.push(frame);
      pthread_mutex_unlock(&lock_);
      }
    } else {
      Log(warning,"Real data taking mode is not tested yet. " );
      // Registers should be setup already.
      // TODO: Implement the DMA data taking code here.
      // First setup the DMA:
#ifdef ARM
      int status = xdma_init();
      if (status < 0) {
        Log(error,"Failed to initialize the DMA engine for data collection.");
        return;
      }
      Log(debug,"Allocating the DMA buffer.");

      uint32_t *frame = NULL;
      //      const uint32_t nbytes_to_collect = 16; //128 bit frame
      const uint32_t buffer_size = 1024;//4096;
      uint32_t pos = 0;
      timeout_cnt_ = 0;
      // Allocate the memory necessary for a packet.
      //frame = reinterpret_cast<uint32_t*>(xdma_alloc(4,sizeof(uint32_t)));
      // This should build a 1000 value circular buffer
      frame = reinterpret_cast<uint32_t*>(xdma_alloc(buffer_size,4*sizeof(uint32_t)));
      for (size_t i = 0; i < 4*buffer_size; ++i) {
    	  frame[i] = 0;
        }
        //FIXME: Verify that the numbers are correct&(dst[pos])
      while (keep_collecting_) {
        Log(debug,"Calling for a transaction on position %d %08X",pos,&(frame[pos]));
        status = xdma_perform_transaction(0,XDMA_WAIT_DST,NULL,0,&(frame[pos]),4);
	//printf("Transaction done with return %d\n",status);
        if (status == -1) {
          Log(warning,"Reached a timeout in the DMA transfer.");
          timeout_cnt_++;
          if (timeout_cnt_ > timeout_cnt_threshold_) {
            Log(error,"Received too many timeouts. Failing the run.");
            throw std::string("Reached timeout counter limit.");
          }
        } else {
          if (timeout_cnt_ > 0){
            timeout_cnt_ = 0;
          }
          Log(debug,"Received %08X %08X %08X %08X",frame[pos],frame[pos+1],frame[pos+2],frame[pos+3]);
        }
        // DMA transaction was successful.
        // -- add the data to the queue;
        Log(debug,"Storing the buffer");
        pthread_mutex_lock(&lock_);
        buffer_queue_.push(&frame[pos]);
        pthread_mutex_unlock(&lock_);
        Log(debug,"Done storing the buffer");
        pos += 4;
        if (pos+4 >= buffer_size) {
        	Log(warning,"Reached buffer size.Resetting to start.\n");
        	pos = 0;
        }
      }
      Log(debug,"Stopped collecting data.");
      // Log(debug,"Shutting down the DMA engine.");
      // xdma_exit();
      // Log(debug,"DMA engine done.");
#endif /*ARM*/
    }
}


void PTBReader::DumpPacket(uint32_t* buffer, uint32_t tot_size) {

  // JCF, Jul-20-2015

  // Uncomment the following "for" loop in order to see the entire
  // contents of the buffer

  //  for (decltype(tot_size) i = 0; i < tot_size/sizeof(uint32_t); ++i) {
  //    std::ostringstream bitdump;
  //    bitdump << "Byte " << i*4 << ": " << std::bitset<32>(buffer[i]);
  //    Log(debug,"%s",bitdump.str().c_str());
  //  }

  // First get the 32 bit header
  uint32_t header = buffer[0];
  std::ostringstream bitdump;
  bitdump << std::bitset<32>(header);
  // And now print it
  Log(debug,"HEADER : 0x%X [%s]",header,bitdump.str().c_str());

  // extract the different parts of the header
  uint32_t version = (header >> 28 & 0xF); // 4msb [28-31]
  uint32_t version_completement = (header >> 24 & 0xF); // grab only the 4 lsb [24-27]
  uint32_t seq_num = (header >> 16) & 0xFF; // bits [16-23]
  uint32_t size = header & 0xFFFF; // 16 lsb [0-16]
  bitdump.str(""); bitdump << std::bitset<4>(version);
  Log(debug,"Version 0x%X (%u) [%s]",version,version,bitdump.str().c_str());
  bitdump.str(""); bitdump << std::bitset<4>(version_completement);
  Log(debug,"Version Complement 0x%X (%u) [%s]",version_completement,version_completement,bitdump.str().c_str());
  bitdump.str(""); bitdump << std::bitset<8>(seq_num & 0xFF);
  Log(debug,"Seq number 0x%X (%u) [%s]",seq_num,seq_num,bitdump.str().c_str());
  bitdump.str(""); bitdump << std::bitset<16>(size & 0xFFFF);
  Log(debug,"Size 0x%X (%u) [%s]",size,size,bitdump.str().c_str());
  
  // First sanity check: the size*sizeof(uint32_t)+sizeof(uint32_t) should be = to tot_size
  
  if (tot_size != size) {
    Log(warning,"Packet sizes do not match! encoded : %u calculated : %u",size,tot_size);
  }
  // Keep looping until we reach the end of the packet
  uint32_t counter = 1;
  // Calculate the number of uint32_t are stored in the array
  uint32_t num_units = size/sizeof(uint32_t);
  while (counter < num_units) {
    // Fetch the header of the frame
    uint32_t fheader = buffer[counter];
    counter++;
    // Extract the 3 msb to determine the frame type
    uint32_t ftype = (fheader >> 29) & 0x7;
    uint32_t tstamp= (fheader >> 1) & 0xFFFFFFF; 
    if (ftype == 0x1) {
      
      bitdump.str(""); bitdump << std::bitset<3>(ftype)
			       << " " << std::bitset<28>(tstamp);
      Log(debug,"Counter word : Time %u [%s]",(fheader >> 1) & 0xFFFFFFF,bitdump.str().c_str() );
      bitdump.str(""); bitdump << std::bitset<1>(fheader & 0x1) 
			       << " " << std::bitset<32>(buffer[counter]) 
			       << " " << std::bitset<32>(buffer[counter+1])
			       << " " << std::bitset<32>(buffer[counter+2]);
      counter += 3;
      Log(debug,"Counter word : Body [%s]",bitdump.str().c_str());
    } else if (ftype == 0x7) {
      
      bitdump.str(""); bitdump << std::bitset<3>(ftype) 
			       << " " << std::bitset<28>(tstamp)
			       << " " << std::bitset<32>(buffer[counter])
			       << " " << std::bitset<32>(buffer[counter+1]);
      uint64_t tmp1 = buffer[counter];
      uint64_t tmp2 = buffer[counter+1];
      tmp1 = (tmp1 << 32) | tmp2;
      Log(debug,"TS word : %X (%u) %X %X %lu",tstamp,tstamp, buffer[counter], buffer[counter+1], tmp1);
      Log(debug,"TS word : [%s]",bitdump.str().c_str());
      
      counter +=2;
    } else if (ftype == 0x4) {
      uint32_t checksum = fheader & 0x1FFFFFFF;
      bitdump.str(""); bitdump << std::bitset<3>(ftype) << " " << std::bitset<29>(checksum); 
      Log(debug,"Checksum word : %X (%u) [%s]",checksum,checksum,bitdump.str().c_str());
    } else if (ftype == 0x2) {
      bitdump.str(""); bitdump << std::bitset<3>(ftype) << " " << std::bitset<28>(tstamp);
      Log(debug,"Trigger word : Time %X (%u) [%s]", tstamp, tstamp,bitdump.str().c_str() );
      
      bitdump.str(""); bitdump << std::bitset<5>((buffer[counter] >> 27) & 0x1F)
			       << " " << std::bitset<4>((buffer[counter] >> 23) & 0xF); 
      uint32_t trig_type = (buffer[counter] >> 27) & 0x1F;
      uint32_t trig_id = (buffer[counter] >> 23) & 0xF;
      Log(debug,"Trigger word : Body  TrigType : %X TrigId %X [%s]", trig_type,trig_id,bitdump.str().c_str());
      counter += 1;
    } else {
      
      bitdump.str(""); bitdump << std::bitset<3>(ftype) << " " << std::bitset<28>(tstamp);
      Log(warning,"Unkown word type: %X  %X (%u) [%s]",ftype,tstamp,tstamp,bitdump.str().c_str());
    }

  }
}


void PTBReader::ClientTransmiter() {
  Log(verbose, "Starting transmitter\n");
  Log(verbose, "previous : %lu roll %lu \n",previous_ts_,time_rollover_);
  std::ostringstream bitdump;
  // FIXME: Finish implementation
  // Fetch a frame from the queue
  seq_num_ = 1;
  uint32_t header = 0;
  // Allocate the maximum memory that a eth packet can have.
  //FIXME: Change this to zero memory copy. Need to check this for memory leaks.
  // Ideally one would want to not allocate the eth packet and simply use the
  // allocated RAM directly. Doable, but must be careful.
  // For the moment copy the contents and see if performance is enough.
  uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);

  // The whole methd runs on an infinite loop with a control variable
  // That is set from the main thread.
  while(keep_transmitting_) {

    //continue;
    // Logic was dramatically changed.
    // FW decides when the packet is ready.
    // When DMA brings the TS word it is sign that packet has to be sent

    // 2 conditions to be careful of:
    //
    // 1. If time time based rollover is reached.
    // 2. If the size based rollover is reached.


    // The size should *never* go beyond the 64 kb
    // Actually, the size should never go beyond the rollover size + the header.
    //uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);

    // zero the memory in. Use memset as it is should be pretty fast
    memset(eth_buffer,0,0xFFFF);


    // Start by generating a header


    // Primary loop is at 32bit word packets
    // ipck   : Counter of packets (32 bit units)
    // iframe : Frame counter.
    uint32_t ipck = 0,iframe = 0;
    bool carry_on = true;
    uint32_t frame[4];
    header = (fw_version_ << 28 ) | (((~fw_version_) & 0xF) << 24) | ((seq_num_  << 16) & 0xFF0000);
    eth_buffer[0] = header;
    Log(verbose, "Temp HEADER : %x (%x [%u] %x [%u])\n",eth_buffer[0],fw_version_,fw_version_,seq_num_,seq_num_);
    // ipck should not include the header
    // But it also works as an index counter, so the manipulation is done at the end
    ipck += 1;
    // while a packet_sending_condition is not reached
    // keep the loop going...
    while(carry_on) {


      // Grab a frame
      if (buffer_queue_.size() == 0) {
        continue;
      }
      Log(debug,"Collecting data");
      Log(verbose, "Transmitting data...\n");
      pthread_mutex_lock(&lock_);
      uint32_t *frametmp = buffer_queue_.front();
      frame[0] = frametmp[3];
      frame[1] = frametmp[2];
      frame[2] = frametmp[1];
      frame[3] = frametmp[0];
      //Log(debug,"Collect the data %08X",frame);
      buffer_queue_.pop();
      pthread_mutex_unlock(&lock_);
      Log(debug,"Frame collected %08X ( %08X %08X %08X %08X)",frame,frame[0],frame[1],frame[2],frame[3]);

      
      // If we still didn't get the first timestamp just drop the packages
      //Log(verbose, "--> Frame1 %x \n",frame[0]);

      /// -- Check if it is a TS frame
      if ((frame[0] >> 29 & 0x7) == 0x7) {
        // This is a timestamp word.
        // Check if this is the first TS after StartRun
        Log(verbose, "Timestamp frame...\n");
        uint64_t tmp_val1 = frame[1];
        uint64_t tmp_val2 = frame[2];

        current_ts_ = (tmp_val1 << 32) | tmp_val2;

        if (first_ts_) {
          // First TS after Run start
          previous_ts_ = (frame[1] << 31) | frame[2];
          first_ts_ = false;
          //delete [] frame;
          continue;
        } else if (current_ts_ >= (previous_ts_ + time_rollover_)) {
          Log(verbose, "Sending the packet\n");
          // Check if we are ready to send the packet (reached the time rollover).
          // Close the packet
          eth_buffer[ipck] = frame[0];
          eth_buffer[ipck+1] = frame[1];
          eth_buffer[ipck+2] = frame[2];
          ipck +=3;
          carry_on = false;
          fragmented_ = false;
          // break out of the cycle
          //delete [] frame;
          break;
          // FIXME: Add the situation in which we arrive to the time rollover.

        } else {
          // Not ready to send yet. Drop this frame and get another
          //delete [] frame;
          continue;
        }
      }

      /// --  Not a TS packet...just accumulate it
      // Start another index to move in words of 32 bits
      // These can loop all the
      //Log(verbose, "--> Frame2 %x \n",frame[0]);
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
        eth_buffer[ipck] = frame[1];
        // The rest of the buffer is crap
        ipck += 1;
      }

      //delete [] frame;
      // Frame completed. check if we can wait for another or keep collecting
      iframe += 1;

      // Size Rollover reached. Produce a fragmented block;
      // Keep collecting only until the next TS word
      if (iframe == packet_rollover_) {
        fragmented_ = true;
        break;
      }

    }
    //Log(verbose, "Broke out of the loop.\n");
    
    //break;


   

    //    // Exited the packet loop.
    //    // Check if the first packet is a timestamp word
    //    if ((eth_buffer[1] >> 29 & 0x7) == 0x7) {
    //      Log(warning,"Empty packet found. Dropping it without sending." );
    //      continue;
    //    }

    // Transmit the data.
    Log(verbose, "Packet completed. Calculating the checksum.\n");

    // Write the size (in bytes)
    // keep in mind that ipck is summed 1 for the checksum
    uint16_t packet_size = ((ipck+1)*sizeof(uint32_t));
    eth_buffer[0] |= packet_size ;
    Log(verbose,"Size was calculated to be %hu",packet_size);
    bitdump.str(""); bitdump << std::bitset<4>(eth_buffer[0] >> 28) 
			     << " " << std::bitset<4>(eth_buffer[0] >> 24 & 0xF)
			     << " " << std::bitset<8>(eth_buffer[0] >> 16 & 0xFF)
			     << " " << std::bitset<16>(eth_buffer[0] & 0xFFFF);
    Log(verbose,"Temp header : %X [%s]",eth_buffer[0], bitdump.str().c_str());
    // Calculate the checksum
    //
    // -- Followed the recipe in
    //    https://en.wikipedia.org/wiki/BSD_checksum
    uint16_t checksum = 0x0;
    uint8_t*ch_buff = reinterpret_cast<uint8_t*>(eth_buffer);
    for (uint32_t i = 0; i < ipck*4; ++i) {
      // Recast as uint8_t
      checksum = (checksum >> 1) + ((checksum & 0x1) << 15) ;
      checksum += ch_buff[i];
      checksum &= 0xFFFF;
      //Log(debug," Byte %u : %08X Chksum %08X (%hu)",i,ch_buff[i],checksum,checksum);
    }

    Log(verbose,"Checksum : %hu",checksum);
    // Calculate 16 bit checksum of the packet
    // Add the checksum to the last packet
    eth_buffer[ipck] = (0x4 << 29) | checksum;

    // JCF, Jul-18-2015

    // I'm switching "ipck+1" in place of "ipck+2", below - not sure
    // why there was an extra uint32_t (above and beyond the
    // microslice header) being sent

    Log(debug,"Sending packet with %u bytes",sizeof(uint32_t)*(ipck+1));
    //DumpPacket(eth_buffer,sizeof(uint32_t)*(ipck+1));


    /// -- Send the packet:
    try {
      socket_->send(eth_buffer,sizeof(uint32_t)*(ipck+1));
    }
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      // Retry
      socket_->send(eth_buffer,sizeof(uint32_t)*(ipck+1));
    }
    // Clear out the memory
    //free(eth_buffer);

    // if we didn't have a fragmented block, update the sequence number
    // and update the timestamp to the latest one
    if (!fragmented_) {
      seq_num_++;
      previous_ts_ = current_ts_;
      fragmented_ =false;
    }


    // if (seq_num_ >= 5) {
    //   Log(warning,"FIXME: Forcing to stop after the first packet being generated");
    //   keep_collecting_ = false;
    //   keep_transmitting_ = false;
    //   // sleep for a while waiting for the generators to stop
    //   std::this_thread::sleep_for (std::chrono::seconds(5));
    // }
  } // -- while(keep_transmitting_)
  // Exited the  run loop. Return.
  Log(info,"Exited transmission loop. Checking for queued packets." );
  // Deallocate the memory
  free(eth_buffer);
  Log(debug,"Shutting down the DMA engine.");
  xdma_exit();
  Log(debug,"DMA engine done.");


}
///////////////////////////////////////////////////////////////
///
/// Everything that is under this part is for simulation
///
///////////////////////////////////////////////////////////////



void PTBReader::GenerateFrame(uint32_t **buffer) {

  std::ostringstream bitdump;
  static uint64_t prev = 0;

  // printf("In GenerateFrame\n");
  // printf("== [%x] \n",buffer[0][0]);
  // printf("== [%x] \n",buffer[0][1]);
  uint32_t*lbuf = buffer[0];
  //printf("Getting time\n");
  uint64_t now = ClockGetTime();
  //Log(verbose, "Clock %lu",now );
  // Here we decide which type of packet I am going to generate
  // It should be dependent on the frequencies that I attach to each type
  // (not using calibrations for now)

  // When should the next event occur

  evtType nextEvt = evt_queue_.top();
  //Log(verbose, "Got an event.\n");

    Log(verbose,"Checking time %lu against %lu (%lu+%lu)",now,prev+time_rollover_,prev,time_rollover_);

    if (now >= (prev+time_rollover_)) {
      
      Log(verbose, "Creating a TS packet\n");
      lbuf[0] = (0x7 << 29) | ((now & 0xFFFFFFF) << 1);
      lbuf[1] = (now >> 32 & 0xFFFFFFF);
      lbuf[2] = (now & 0xFFFFFFF);
      //bitdump.str(""); bitdump << std::bitset<32>(lbuf[0]) << " " <<  std::bitset<32>(lbuf[1]) << " " <<  std::bitset<32>(lbuf[2]);
      //Log(debug,"FRAME : %X %X %X [%s]",lbuf[0],lbuf[1],lbuf[2],bitdump.str().c_str());
      prev = now + time_rollover_;
      return;
    }
    
    // If we haven't reached the time for the next event just send a TS packet.
    if (now < nextEvt.next) {
      // If we are past the time rollover issue a TS packet.
      // Otherwise just do nothing
      
      return;
      
    }

  //Log(verbose, "Creating something else");

  // pop it from the queue now that we know that it is going to be returned
  evt_queue_.pop();

  // Write the header of the frame
  lbuf[0] = ((nextEvt.type & 0x7) << 29) | ((now & 0xFFFFFFF) << 1) | 0x0;
  if (nextEvt.type == 0x1) {
    // This is a counter word
    // Generate 3 random numbers and assign them
    uint32_t val = rand();
    lbuf[0] |= ((val >> 31) & 0x1);
    lbuf[1] = val;
    lbuf[2] = rand();
    lbuf[3] = rand();
    
    //bitdump.str(""); bitdump << std::bitset<32>(lbuf[0]) << " " <<  std::bitset<32>(lbuf[1]) << " " <<  std::bitset<32>(lbuf[2]) << " " <<  std::bitset<32>(lbuf[3]);
    //Log(debug,"FRAME : %X %X %X %X [%s]",lbuf[0],lbuf[1],lbuf[2],lbuf[3],bitdump.str().c_str());

  } else {
    // The 5 msb are the trigger word mask (10000 for muon counters)
    lbuf[1] = (0x10 << 27) | ((nextEvt.trigger & 0xF) << 23) | 0x0;

    //bitdump.str(""); bitdump << std::bitset<32>(lbuf[0]) << " " <<  std::bitset<32>(lbuf[1]);
    //Log(debug,"FRAME : %X %X [%s]",lbuf[0],lbuf[1],bitdump.str().c_str());
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
      Log(error,"Failed to figure out the evt type to requeue." );
    }
  }
  evt_queue_.push(nextEvt);
  //Log(verbose,"Pushed event to bottom of the stack again.");

}


void PTBReader::InitEmuSampler() {
  // Attach the evt frequencies -- in Hz
  Log(verbose, "Starting the Sampler\n");

  freq_counter = 100000;
  freq_trigA = 10000;
  freq_trigB = 2000;
  freq_trigC = 3000;
  freq_trigD = 1000;

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

  Log(verbose, "Finished the Sampler\n");

}
