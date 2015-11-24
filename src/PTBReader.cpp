/*
 * PTBReader.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#include "PTBReader.h"
#include "Logger.h"
#include "PracticalSocket.h"
#include "PTBexception.h"
#include "util.h"

#include <sys/time.h>
#include <stdint.h>
#include <bitset>
#include <sstream>
#include <thread>
#include <chrono>

#include <cstring>

#ifdef ARM
#include "libxdma.h"
#endif /*ARM*/



std::string display_bits(void* memstart, size_t nbytes) {

  std::stringstream bitstr;
  bitstr << "The " << nbytes << "-byte chunk of memory beginning at " << static_cast<void*>(memstart) << " is : [";

  // -- NFB : Nov-20-2015
  // reversed the order of the bits when printing, so that the msb remain on the left
  for(unsigned int i = 0; i < nbytes; i++) {
    bitstr << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+i)) << " ";
  }
  bitstr << "]";
  return bitstr.str();

//  std::cout << "Field [" << bitstr.str() << "]" << std::endl;
//  mf::LogInfo(sourcename.c_str()) << bitstr.str();
}

std::string display_bits_reversed(void* memstart, size_t nbytes) {

  std::stringstream bitstr;
  bitstr << "The " << nbytes << "-byte chunk of memory beginning at " << static_cast<void*>(memstart) << " is : [";

  // -- NFB : Nov-20-2015
  // reversed the order of the bits when printing, so that the msb remain on the left
  for(unsigned int i = 0; i < nbytes; i++) {
    bitstr << std::bitset<8>(*((reinterpret_cast<uint8_t*>(memstart))+(nbytes-i-1))) << " ";
  }
  bitstr << "]";
  return bitstr.str();

//  std::cout << "Field [" << bitstr.str() << "]" << std::endl;
//  mf::LogInfo(sourcename.c_str()) << bitstr.str();
}


// Completely auxiliary clock function
uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}

PTBReader::PTBReader(bool emu) : tcp_port_(0), tcp_host_(""),
    packet_rollover_(0),socket_(NULL),
    client_thread_collector_(0),client_thread_transmitor_(0),ready_(false), emu_mode_(emu),
				 fragmented_(false),keep_transmitting_(true),/*ready_to_send_(false),*/first_ts_(true),keep_collecting_(true),previous_ts_(0),timeout_cnt_(0)

{
  Log(verbose,"Creating an instance of the reader with emu %s\n",emu_mode_?"true":"false");
  (void) first_ts_;
  (void) current_ts_;

  if (emu_mode_) {
	  Log(warning,"Running in emulator mode.");
    Log(verbose,"Filling emulator queue");
    //-- InitEmuSampler();
  } else {
    Log(verbose,"Not filling emulator queue");
    // Nothing should need to be done here.
  }

  // Init the mutex for the queue
  if (pthread_mutex_init(&lock_, NULL) != 0)
  {
    Log(error,"\n Failed to create the mutex for the data queue\n" );
    throw std::string("Failed to create the mutex for the data queue.");
  }

}

PTBReader::~PTBReader() {
  Log(debug,"Destroying the reader." );
  pthread_mutex_destroy(&lock_);

  ready_ = false;

}

void PTBReader::ClearThreads() {
	// FIXME: Migrate this to C++11 constructs.
	Log(debug,"Killing the daughter threads.\n");
	// First stop the loops
	keep_collecting_ = false;
	std::this_thread::sleep_for (std::chrono::seconds(1));
	Log(info,"Killing collector thread.");
	// Kill the collector thread first
	pthread_cancel(client_thread_collector_);
	// Kill the transmittor thread.

	keep_transmitting_ = false;
	std::this_thread::sleep_for (std::chrono::seconds(2));
	// -- Apparently this is a bit of a problem since the transmitting thread never leaves cleanly
	Log(info,"Killing transmiter thread.");
	pthread_cancel(client_thread_transmitor_);

}

void PTBReader::StopDataTaking() {
  // This should be a full stop. However there should aso be a PauseDataTaking, that does not destroy the threads
  // and simply waits for a StartRun to come again.

  ClearThreads();
  //ready_ = false;
  // Prepare everything so that the PTB gets ready again

  // Stop the DMA.
#ifdef ARM
  //FIXME: There is a risk that the thread is killed before
  // the execution reaches this point.
  Log(warning,"Shutting down the DMA engine.");
  xdma_exit();
  Log(debug,"DMA engine done.");
#endif

}

void PTBReader::StartDataTaking() {
	//FIXME: Migrate this to C++11 constructs
  if (ready_) {
# ifdef ARM
	  int status = xdma_init();
    if (status < 0) {
      Log(error,"Failed to initialize the DMA engine for data collection.");
      throw std::string("Failed to initialize the DMA engine for data collection.");
      return;
    }
#endif
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
    throw PTBexception("Connection not available.");
  }
}

void PTBReader::ResetBuffers() {
	// FIXME: Migrate this to C++11 constructs (std::thread::mutex)
  Log(warning,"Resetting the software buffers.");
  uint32_t counter = 0;
  pthread_mutex_lock(&lock_);

  while (!buffer_queue_.empty()) {
    //uint32_t*frame = buffer_queue_.front();
    buffer_queue_.pop();
    counter++;
    //delete [] frame;
  }
  pthread_mutex_unlock(&lock_);
  Log(info,"Popped %u entries from the buffer queue.",counter);
}

void PTBReader::InitConnection(bool force) {

	//-- If a connection exists, assume it is correct and continue to use it
	if (socket_ != NULL) {
		if (force) {
			Log(warning,"Destroying existing socket!");
			delete socket_;
		} else {
			Log(info,"Reusing existing connection.");
			return;
		}
	}

  // Check if the server data is set up, and compute
  if (tcp_host_ != "" && tcp_port_ != 0) {

    try{
      Log(debug, "Opening socket connection : %s : %hu",tcp_host_.c_str(),tcp_port_ );
      socket_ = new TCPSocket(tcp_host_.c_str(),tcp_port_);

      if (socket_ == NULL) {
        Log(error,"Unable to establish the client socket. Failing." );
        ready_ = false;
        throw;
      }
      // Otherwise just tell we're ready and start waiting for data.
      ready_ = true;

    }
    // -- Catch and rethrow the exceptions sho that they can be dealt with at higher level.
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      ready_ = false;
      socket_ = NULL;
      throw;
    }
    catch(std::exception &e) {
        ready_ = false;
        socket_ = NULL;
      Log(error,"STD exception caught : %s",e.what() );
      throw;
    }
    catch(...) {
        ready_ = false;
        socket_ = NULL;
      Log(error,"Unknown exception caught." );
      throw;
    }
  } else {
    Log(error,"Calling to start connection without defining connection parameters." );
    ready_ = false;
    socket_ = NULL;
    throw;
  }

}

void PTBReader::ClientCollector() {
  Log(info, "Starting data collector\n" );
    if (emu_mode_) {
      while(keep_collecting_) {
      // Reserve the memory for one frame (128 bits long)
      uint8_t* frame = (uint8_t*)calloc(16,sizeof(uint8_t));
      Log(verbose, "Generating the frame\n");
      // -- GenerateFrame(&frame);
      // If the header is something else do nothing.
      if (((frame[0] >> 29) & 0x7) == 0x0) continue;
      Log(verbose, "Frame generated\n");
      pthread_mutex_lock(&lock_);
      buffer_queue_.push(frame);
      pthread_mutex_unlock(&lock_);
      (void)timeout_cnt_;
      (void)timeout_cnt_threshold_;
      }
    }
#ifdef ARM
    else {
      // Registers should be setup already.
      // First setup the DMA:
        ///FIXME: Maybe this code could be moved elsewhere to speed up initialization
      int status = 0;
      Log(debug,"Allocating the DMA buffer.");

      uint8_t *frame = NULL;
      //      const uint32_t nbytes_to_collect = 16; //128 bit frame
      // -- Can easily increase this to avoid overlaps?
      // FIXME: Increase the buffer size to have contingency memory.
      const uint32_t buffer_size = 1024*1024;
      uint32_t pos = 0;
      timeout_cnt_ = 0;
      // Allocate the memory necessary for a packet.
      //frame = reinterpret_cast<uint32_t*>(xdma_alloc(4,sizeof(uint32_t)));
      // This should build a 1000 value circular buffer
      // The bus is 16 bytes, and that should be the size of each frame
      frame = reinterpret_cast<uint8_t*>(xdma_alloc(buffer_size,16));
      // There should be a more efficient way of zeroing the data
      // Do we really need to zero the data?
      for (size_t i = 0; i < buffer_size*16; ++i) {
    	  frame[i] = 0;
        }
      while (keep_collecting_) {
//        Log(debug,"Calling for a transaction on position %d %08X",pos,&(frame[pos]));
        // the DMA passes data in little endian, i.e., the msb are in the highest index of the array
        // the bits within a byte are correct, though
        status = xdma_perform_transaction(0,XDMA_WAIT_DST,NULL,0,reinterpret_cast<uint32_t*>(&frame[pos]),16);
        Log(verbose,"Received contents [%s]",display_bits(&frame[pos],16).c_str());
        // Try to make a fancy printf
        for (uint32_t i = 0; i < 16; ++i) {
          printf("%02X",frame[pos+i] & 0xFF);
          if (i!=0 && ((i+1)%4 == 0)) printf(" ");
          }
        printf("\n");

        if (status == -1) {
          Log(warning,"Reached a timeout in the DMA transfer.");
          timeout_cnt_++;
          if (timeout_cnt_ > timeout_cnt_threshold_) {
            Log(error,"Received too many timeouts. Failing the run.");
            throw std::string("Reached timeout counter limit.");
          }
        } else {
          timeout_cnt_ = 0;
          //Log(debug,"Received %08X %08X %08X %08X",frame[pos],frame[pos+1],frame[pos+2],frame[pos+3]);
        }
        // DMA transaction was successful.
        // -- add the data to the queue;
//        Log(debug,"Storing the buffer");
        pthread_mutex_lock(&lock_);
        buffer_queue_.push(&frame[pos]);
        pthread_mutex_unlock(&lock_);
//        Log(debug,"Done storing the buffer");
        pos += 16;
        if (pos+16 >= buffer_size) {
//        	Log(info,"Reached buffer size.Resetting to start.\n");
        	pos = 0;
        }
      }
      Log(warning,"Stopped collecting data.");
      /// We can't exit the engine here otherwise the memory mapped registers disappear and the queue vanishes causing a crash
      // Log(debug,"Shutting down the DMA engine.");
      // xdma_exit();
      // Log(debug,"DMA engine done.");
    }
#endif /*ARM*/
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
      bitdump.str(""); //bitdump << std::bitset<1>(fheader & 0x1)
      bitdump << " " << std::bitset<32>(buffer[counter])
			       << " " << std::bitset<32>(buffer[counter+1])
			       << " " << std::bitset<32>(buffer[counter+2])
             << " " << std::bitset<32>(buffer[counter+3]);
      counter += 4;
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

  seq_num_ = 1;
  uint32_t header = 0;
  // Allocate the maximum memory that an eth packet can have.
  //FIXME: Change this to zero memory copy.
  // -- No memory leak found here.
  // Ideally one would want to not allocate the eth packet and simply use the
  // allocated RAM directly. Doable, but must be careful because of the circular buffer.
  // For the moment copy the contents and see if performance is good enough.
  // I suspect there is a problem of endianess here
  //uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);
  uint8_t *eth_buffer = (uint8_t*)calloc(0xFFFF,1);

  // The whole method runs on an infinite loop with a control variable
  // That is set from the main thread.
  while(keep_transmitting_) {

    // FW decides when the packet is ready to be sent.
    // When DMA brings the TS word it is sign that packet has to be sent

    // 2 conditions to be careful of:
    //
    // 1. If time time based rollover is reached.
    // 2. If the size based rollover is reached.


    // The size should *never* go beyond the 64 kb
    // Actually, the size should never go beyond the rollover size + the header.
    //uint32_t *eth_buffer = (uint32_t*)calloc(0xFFFF,1);

    // zero the memory in. Use memset as it is should be pretty fast
    // FIXME: This eventually will not be needed
    memset(eth_buffer,0,0xFFFF);


    /// -- Start by generating a header


    // Primary loop is at 32bit word packets
    // ipck   : Counter of packets (32 bit units)
    // iframe : Frame counter.
    uint32_t ipck = 0,iframe = 0;
    bool carry_on = true;
//    uint32_t frame[4];
//    uint8_t frame[16];
    uint8_t frame[16];
    uint8_t *frame_raw = NULL;
    header = (fw_version_ << 28 ) | (((~fw_version_) & 0xF) << 24) | ((seq_num_  << 16) & 0xFF0000);
    std::memcpy(&eth_buffer[0],&header,sizeof(header));



//    eth_buffer[0] = header;
    // Log(verbose, "Temp HEADER : %x (%x [%u] %x [%u])\n",eth_buffer[0],fw_version_,fw_version_,seq_num_,seq_num_);
    // ipck should not include the header
    // But it also works as an index counter, so the manipulation is done at the end

    ipck += sizeof(header);
    // while a packet_sending_condition is not reached
    // keep the loop going...
    // Needs some extra protection for when the StopRun is called, otherwise the loop does not
    //really stop since it will waiting forever for a timestamp word that will never arrive.
    while(carry_on and keep_transmitting_) {


      // Grab a frame
      if (buffer_queue_.size() == 0) {
        continue;
      }
      Log(debug,"Collecting data");
      pthread_mutex_lock(&lock_);
      frame_raw = buffer_queue_.front();
      for (uint32_t i =0; i < 16; ++i) {
    	  frame[i] = frame_raw[15-i];
      }
//      frame = buffer_queue_.front();

//      // This could be avoided with an adjustment of the indexes in the code below
//      frame[0] = frametmp[3];
//      frame[1] = frametmp[2];
//      frame[2] = frametmp[1];
//      frame[3] = frametmp[0];
      //Log(debug,"Collect the data %08X",frame);
      buffer_queue_.pop();
      pthread_mutex_unlock(&lock_);
      //Log(debug,"Frame collected %08X ( %08X %08X %08X %08X)",frame,frame[3],frame[2],frame[1],frame[0]);
      Log(debug,"Frame collected :");
      // Keep in mind that the msb are in the last byte.
      Log(debug,"%s",display_bits(&frame[0], 16).c_str());
      // Reverse the whole word here. Simpler to deal with


      Payload_Header* payload_header = reinterpret_cast<Payload_Header*>(frame);

      // Very first check to discard "ghost frames"
      // -- Ghost frames are caused by the NOvA timing not being fully initialized by
      // the time the word was generated. These usually occur because there is a delay
      // between the sync pulse and the timestamps starting to be populated in the NOvA firmware.
      // Unfortunately it doesn't seem that anything can be done about it

//      if ((reinterpret_cast<uint32_t*>(&(frame[0]))[0] & 0xFFFFFFF) == 0x0) {
//        continue;
//      }
      if ((payload_header->short_nova_timestamp & 0xFFFFFFF) == 0x0)
        continue;

      /// -- Check if it is a TS frame
//      if ((frame[0] >> 5 & 0x7) == 0x7) {
      if(payload_header->data_packet_type == DataTypeTimestamp) {
        // This is a timestamp word.
        // Check if this is the first TS after StartRun
        Log(verbose, "Timestamp frame...\n");
//        uint64_t tmp_val1 = frame[2];
//        uint64_t tmp_val2 = frame[1];
//
//        current_ts_ = (tmp_val1 << 32) | tmp_val2;
//
//        if (first_ts_) {
//          // First TS after Run start
////          previous_ts_ = (frame[2] << 31) | frame[1];
//        	previous_ts_ = current_ts_;
//          first_ts_ = false;
//          //delete [] frame;
//          continue;
//        } else {

          Log(verbose, "Sending the packet\n");
          // Check if we are ready to send the packet (reached the time rollover).
          // Close the packet
//          eth_buffer[ipck] = frame[3];
//          eth_buffer[ipck+1] = frame[2];
//          eth_buffer[ipck+2] = frame[1];

          std::memcpy(&eth_buffer[ipck],&frame[0],Payload_Header::size_words +payload_size_timestamp);
          ipck += Payload_Header::size_words +payload_size_timestamp;
          carry_on = false;
          fragmented_ = false;
          // break out of the cycle
          //delete [] frame;
          break;
//        }
      }

      /// --  Not a TS packet...just accumulate it

      // -- Grab the header (valid for all situations
      Log(verbose, "Grabbing the header\n");

      std::memcpy(&eth_buffer[ipck],&frame[0],Payload_Header::size_words);
      ipck += Payload_Header::size_words;
//      eth_buffer[ipck] = frame[3];
//      ipck += 1;



      switch(payload_header->data_packet_type) {
        case DataTypeCounter:
          Log(verbose, "Counter word\n");
          // This one requires some special handling since it needs to shift the bits from the header
          for (size_t k=0; k< payload_size_counter-1; ++k) {
            eth_buffer[ipck+k] = ((frame[Payload_Header::size_words-1+k] & 0x1) << 31) | (frame[Payload_Header::size_words+k] & 0xFE);
          }
          eth_buffer[ipck+payload_size_counter-1] = (frame[payload_size_counter-1] & 0x1) << 7;
          ipck += payload_size_counter;
          break;
        case DataTypeTrigger:
          Log(verbose, "Trigger word\n");
          std::memcpy(&eth_buffer[ipck],&frame[Payload_Header::size_words],payload_size_trigger);
          ipck += payload_size_trigger;
          break;
        case DataTypeSelftest:
          Log(verbose, "Selftest word\n");

          std::memcpy(&eth_buffer[ipck],&frame[Payload_Header::size_words],payload_size_selftest);
          ipck += payload_size_selftest;
          break;
        default:
          char msg[50];
          sprintf(msg,"Unknown data type [%X]",frame[0] & 0xFF);
          throw PTBexception(msg);
      }
      // trim the data depending on the type
//      if ((frame[3] >> 29 & 0x7) == 0x1) { // counter word.
//        // NOTE: The word received from the PL has to be
//        // unrolled to avoid problems in the board reader.
//        for (size_t k = 0; k < 3; ++k) {
//        	// k: 0 , k+1 : 1, 4-(k+1): 3
//        	// k: 1 , k+1 : 2, 4-(k+1): 2
//        	// k: 2 , k+1 : 3, 4-(k+1): 1
//          // k: 3 , k+1 : 4, 4-(k+1): 0
//          // -- The pattern is to pick up the lsb from previous word and the rest from the follow-up
//          // except for the last one
////          eth_buffer[ipck+k] = frame[3-(k+1)];
//
//          eth_buffer[ipck+k] = ((frame[4-(k+1)] & 0x1) << 31) | ((frame[4-(k+2)] & 0xFFFFFFFE) >> 1);
//        }
//        eth_buffer[ipck+3] = ((frame[3] & 0x1) << 31);
////        Log(debug,"Counter word (%08X %08X %08X %08X %08X)",eth_buffer[ipck-1],
////            eth_buffer[ipck], eth_buffer[ipck+1],eth_buffer[ipck+2],eth_buffer[ipck+3]);
//
//        // Refresh ipck to the latest position
//        ipck += 4;
//      } else if ((frame[3] >> 29 & 0x7) == 0x2) {
//        // trigger word: Only the first 32 bits of the payload
//        // are actually needed
//        eth_buffer[ipck] = frame[2];
//        // The rest of the buffer is crap
//        ipck += 1;
//      }
      Log(verbose, "Frame processed\n");

      //delete [] frame;
      // Frame completed. check if we can wait for another or keep collecting
      iframe += 1;

      // Size Rollover reached. Produce a fragmented block;
      // Keep collecting only until the next TS word
      // The check should be a bit more secure.
      if ((iframe+4) >= packet_rollover_) {
        fragmented_ = true;
        break;
      }

    }

    // Transmit the data.
    Log(verbose, "Packet completed. Calculating the checksum.\n");

    // Write the size (in bytes)
    // keep in mind that ipck is summed 1 for the checksum
//    uint16_t packet_size = ((ipck+1)*sizeof(uint32_t));
    uint32_t packet_size = ipck+sizeof(uint32_t);
    Log(verbose,"Size was calculated to be %u",packet_size);

    eth_buffer[0] = SetBitRange(eth_buffer[0],packet_size,0,16);
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
//    uint8_t*ch_buff = reinterpret_cast<uint8_t*>(eth_buffer);
    uint8_t*ch_buff = eth_buffer;
    for (uint32_t i = 0; i < ipck; ++i) {

      checksum = (checksum >> 1) + ((checksum & 0x1) << 15) ;
      checksum += ch_buff[i];
      checksum &= 0xFFFF;
      //Log(debug," Byte %u : %08X Chksum %08X (%hu)",i,ch_buff[i],checksum,checksum);
    }


    Log(verbose,"Checksum : %hu",checksum);
    // Calculate 16 bit checksum of the packet
    // Add the checksum to the last packet
    //eth_buffer[ipck] = (0x4 << 29) | checksum;
    uint32_t checksum_word = (0x4 << 29) | checksum;
    std::memcpy(&eth_buffer[ipck],&checksum_word,sizeof(uint32_t));
    // JCF, Jul-18-2015
    ipck += sizeof(checksum_word);
    // I'm switching "ipck+1" in place of "ipck+2", below - not sure
    // why there was an extra uint32_t (above and beyond the
    // microslice header) being sent

//    Log(debug,"Sending packet with %u bytes (including header)",sizeof(uint32_t)*(ipck+1));
    Log(debug,"Sending packet with %u bytes (including header)",ipck);
    //DumpPacket(eth_buffer,sizeof(uint32_t)*(ipck+1));

    /// -- Send the packet:
    try {
      socket_->send(eth_buffer,ipck);
    }
    catch(SocketException &e) {
      Log(error,"Socket exception caught : %s",e.what() );
      // Retry
      socket_->send(eth_buffer,ipck);
    }
    // Clear out the memory
    //free(eth_buffer);

    // if we didn't have a fragmented block, update the sequence number
    // and update the timestamp to the latest one
    if (!fragmented_) {
      seq_num_++;
      //previous_ts_ = current_ts_;
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
  Log(info,"Exited transmission loop.");
  // Deallocate the memory
  free(eth_buffer);

}
