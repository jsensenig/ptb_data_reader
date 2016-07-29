#include <iostream>
#include <cstdlib>

#include <time.h>
#include <sys/timeb.h>
#include <cstdlib>

#include <sys/time.h>


uint64_t ClockGetTime() {

  struct timeval tv;
  gettimeofday(&tv,NULL);
  return (uint64_t)tv.tv_sec*1000000LL + (uint64_t)tv.tv_usec;

}

// uint64_t ClockGetTimeLinux()
// {
//   timespec ts;
//     // for 

//     // for linux
//     //clock_gettime(CLOCK_REALTIME, &ts);
//     return (uint64_t)ts.tv_sec * 1000000LL + (uint64_t)ts.tv_nsec / 1000LL;
// }



void build_header(void**buffer,uint32_t &seq_num, uint8_t &version) {
  
  *((volatile uint32_t*)buffer[0]) = (seq_num & 0xFF) << 24 | (version << 16);

}

int main() {
  
  // Max number of packets accumulated until size is obtained. 
  uint32_t rollover = 80;
  // Size of header in 32 bit increments
  uint32_t header_size = 1;
  void* pbuffer = calloc(rollover+5,sizeof(uint32_t));

  // Block of 32-bits after which the data should be inserted
  uint32_t offset_data = 1; 
  uint32_t seq_num = 0;
  uint8_t version = 0x4;
  
  // packet size is 128 bits long:
  // 32 bit header
  // max 96 bit body (counter word needs 97...use the lsb in header)
  uint32_t dma_pkt_size = 128; // 16 bytes
  // Keep a copy of the timestamp of the first packet to be assigned
  uint16_t tsroll0 = 0x0;
  // keep a copy of the number of size that we have in the packet
  // (excluding the 32 bit CRC word)
  uint16_t size = 0x0;
  uint16_t max_size = 0xFFFF - 4;
  uint32_t tsroll = 0, pl1=0, pl2 =0, pl3=0;
  uint8_t packet_type = 0x1;
  for (uint32_t  i=0; i < 100; ++i) {
    size = 0x0;
    
    // First construct the header
    build_header(&pbuffer,i,version);
    // The header is 32 bits (4 bytes)
    size += 4;
    std::cout << " Buffer as of now: " << ((volatile uint64_t*) (pbuffer))[0] 
	      << " H " << std::hex << ((volatile uint64_t*) (pbuffer))[0]  << std::endl;
    
    // There are 3 conditions to complete a tramission packet:
    // 
    // 1. The rollover is reached (full packet)
    // --    j == rollover
    // 2. The 64kb size is reached (fragmented block)
    // -- j ==  0xFFFFFFFF/(dma_pkt_size)
    // 3. The rollover time is reached (full packet)
    // tn == (t0-1)
    uint32_t ret_status = 0;
    uint64_t clock = 0;
    for (uint32_t j=0; j < rollover; ++j) {
      // Get the lower 28 bits of the TS
      clock = ClockGetTime(); 
      std::cout << "TS " << std::hex << clock << std::endl;
      tsroll = clock & 0xFFFFFFF;
      std::cout << "TS rollover "  << std::hex << tsroll << std::endl;
      
      if (tsroll0 == 0x0 ) tsroll0 = tsroll;
      // Passed the TS roll
      // Step out of the loop and send packet
      if (tsroll <= tsroll0) {
	// Rollover is passed
	std::cout << "TS rollover reached" << std::endl;
	break;
      }

      // Calculate the position this is supposed to go into
      // header is pos 0
      // each packet takes 4 (1 header + 3 payload)
      // Positions: 1,5,9
      // packet type is for the first 3 bits
      // TS rollover is the next 24
      // unused bit is the next 1
      // Need to generate 3 random words of 32 bits
      pl1 = rand();
      pl2 = rand();
      pl3 = rand();
      
      
      ((volatile uint32_t*)pbuffer)[(j*4)+1] = ((packet_type  & 0x7) << 29) | (tsroll << 1);
      std::cout << "Header : " << std::hex << ((volatile uint32_t*)pbuffer)[(j*4)+1] << std::dec << std::endl;
      // Deconstruct header for cross check
      std::cout << "Type : " << std::hex << ((((volatile uint32_t*)pbuffer)[(j*4)+1] >> 29 ) & 0x7) << " " << ((((volatile uint32_t*)pbuffer)[(j*4)+1] >> 1 ) & 0xFFFFFFF) << std::dec << std::endl;

      ((volatile uint32_t*)pbuffer)[(j*4)+2] = pl1;
      ((volatile uint32_t*)pbuffer)[(j*4)+3] = pl2;
      ((volatile uint32_t*)pbuffer)[(j*4)+4] = pl3;
      // Put the last bit in the lsb of the first word
      ((volatile uint32_t*)pbuffer)[(j*4)+1] |= ((pl1%2) & 0x1);
    
      std::cout << "Packet " << std::dec << j << " : " << std::hex << ((volatile uint32_t*)pbuffer)[(j*4)+1] << " " << std::hex << ((volatile uint32_t*)pbuffer)[(j*4)+2] << " " << std::hex << ((volatile uint32_t*)pbuffer)[(j*4)+3] << " " << std::hex << ((volatile uint32_t*)pbuffer)[(j*4)+4] << " " << std::endl;
      
      // Make checks whether we should accept one more or call it a day
      if (j == (rollover-1)) {
	std::cout << "Reached the end of the rollover." << std::endl;
      }
      // Reached the maximum size of 64 kbit == 8kb
      if (j == 0xFFFF/(dma_pkt_size)) {
	std:: cout << "Reached max_size" << std::endl;
      }
      // We have to play it safe, since we don't know if the next timestamp will be already over the rollover
      // Reached the ts rollover by
      
      
    }
    // We have a packet ready to send. 
    // Check if it is fragmented
    // Assign the size 
    // Print it
    //print_packet(buffer);
    
  }


  return 0;

}
