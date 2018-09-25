/*
 * test_sizes.cc
 *
 *  Created on: May 7, 2018
 *      Author: nbarros
 */


#include <iostream>
#include <bitset>
#include <cinttypes>

typedef struct low_level_trigger_t {
    uint64_t mask : 61;
    uint64_t padding: 3;
    uint32_t padding_high;

} low_level_trigger_t;

typedef struct header_t {
   typedef uint64_t pad_size_t;
   typedef uint8_t  word_type_t;

   pad_size_t padding_low;
   pad_size_t padding_high : 61;
   word_type_t word_type : 3;

 } header_t;

     /// -- Several different structures that can be used to reinterpret the payload depending on
      /// the word type. All these structures map into the full 16 bytes of the CTB words
      ///

 // typedef struct feedback_t {
 //     typedef uint64_t ts_size_t;
 //     typedef uint16_t code_size_t;
 //     typedef uint16_t source_size_t;
 //     typedef uint8_t  word_type_t;
 //     typedef uint32_t pad_size_t;

 //     ts_size_t     timestamp;
 //     code_size_t   code   : 16;
 //     source_size_t source : 16;
 //     pad_size_t    padding: 29;
 //     word_type_t   word_type : 3;


 //     static size_t const size_bytes = 2*sizeof(uint64_t);
 //     static size_t const size_u32 = size_bytes/sizeof(uint32_t);

 //     static size_t const n_bits_timestamp  = 64;
 //     static size_t const n_bits_payload = 32;
 //     static size_t const n_bits_type     = 3;

 // } feedback_t;

 typedef struct ch_status_t {
      typedef uint64_t ts_size_t;
      typedef uint32_t pds_size_t;
      typedef uint64_t crt_size_t;
      typedef uint16_t bi_size_t;
      typedef uint8_t  wtype_size_t;

      ts_size_t     timestamp  : 60;
      bi_size_t     beam_lo    : 4;
      bi_size_t     beam_hi    : 5;
      crt_size_t    crt        : 32;
      pds_size_t    pds        : 24;
      wtype_size_t  word_type  : 3;


      static size_t const size_bytes = 2*sizeof(uint64_t);
      static size_t const size_u32 = size_bytes/sizeof(uint32_t);

      static size_t const n_bits_timestamp  = 60;
      static size_t const n_bits_payload = 32;
      static size_t const n_bits_type     = 3;


      // aux_functions
      uint16_t get_beam() {return (beam_hi << 4 | beam_lo);}
      uint32_t get_crt() {return (crt & 0xFFFFFFFF);}
      uint32_t get_pds() {return (pds & 0xFFFFFFF);}

      bool get_state_crt(const uint16_t channel) {
        return ((crt & (0x1 << channel)) != 0x0);
      }
      bool get_state_pds(const uint16_t channel) {
        return ((pds & (0x1 << channel)) != 0x0);
      }
      bool get_state_beam(const uint16_t channel) {
        return (((beam_hi << 4 | beam_lo) & (0x1 << channel)) != 0x0);
      }

  } ch_status_t;

 typedef struct timestamp_t {
     typedef uint64_t ts_size_t;
     typedef uint64_t pad_size_t;
     typedef uint8_t wtype_size_t;

     ts_size_t    timestamp;
     pad_size_t   padding   : 61;
     wtype_size_t word_type : 3;

     static size_t const size_bytes = 2*sizeof(uint64_t);
     static size_t const size_u32 = size_bytes/sizeof(uint32_t);

     static size_t const n_bits_timestamp  = 64;
     static size_t const n_bits_unused = 61;
     static size_t const n_bits_type     = 3;

 } timestamp_t;


  typedef struct feedback_t {
      typedef uint64_t  ts_size_t;
      typedef uint8_t  code_size_t;
      typedef uint8_t  source_size_t;
      typedef uint8_t   wtype_size_t;
      typedef uint32_t  payload_size_t;

      ts_size_t       timestamp;
      code_size_t     code      : 8;
      source_size_t   source    : 8;
      payload_size_t  payload1   : 16;
      payload_size_t  payload2   : 29;
      wtype_size_t    word_type : 3;

      void set_payload(uint64_t pl) {payload1 = (pl & 0xFFFF); payload2 = ((pl >> 16) & 0x1FFFFFFF);}
      uint64_t get_payload() {return ((((uint64_t)payload2) << 16) | (uint64_t)payload1);}
      static size_t const size_bytes = 2*sizeof(uint64_t);
      static size_t const size_u32 = size_bytes/sizeof(uint32_t);

      static size_t const n_bits_timestamp  = 64;
      static size_t const n_bits_payload = 32;
      static size_t const n_bits_type     = 3;

  } feedback_t;

 using std::cout;
using std::endl;
using std::hex;
using std::dec;
int main() {

  ch_status_t st;
  st.pds = 0x0;
  st.beam_lo = 0xF;
  st.beam_hi = 0x1F;
  st.crt = 0x0;
  st.word_type = 0x0;
  st.timestamp = 0x0;

  cout << "Size :  " << sizeof(ch_status_t) << endl;


  feedback_t fb;
  cout << "Feedback. size : " << sizeof(fb) << endl;

  fb.set_payload(0xAAAAAABBBB);
  fb.code = 0x3;
  fb.source = 0x1;
  fb.timestamp = 1234567890;
  cout << "Payload parts : 1 : " << std::hex << fb.payload2 << std::dec << " 2 : " << hex << fb.payload1 << dec << endl;
  cout << "Payload : " << std::hex << fb.get_payload() << std::dec << endl;

  // Now lets play with a printf
  feedback_t *fbk = &fb;
  printf("Feedback word caught : \nTS : [%" PRIu64 "]\nSource : [%X]\nCode : [%X]\nPayload : [%" PRIX64 "]\n",
               fbk->timestamp,fbk->source,fbk->code,fbk->get_payload());
  


//  uint64_t * data = reinterpret_cast<uint64_t*>(&st);
////  uint64_t ""
//  cout << "BI " << std::bitset<9>(st.beam) << " PDS " << std::bitset<24>(st.pds) << " CRT " << std::bitset<32>(st.crt) << endl;
//  cout << "TS " << std::bitset<9>(st.timestamp) << " WTYPE " << std::bitset<3>(st.word_type) << endl;
//  cout << "HI " << std::bitset<64>(data[1]) << endl;
//  cout << "LO " << std::bitset<64>(data[0]) << endl;
//
//  uint8_t * d8 = reinterpret_cast<uint8_t*>(&st);
//  for (size_t i = 0; i < 16; ++i) {
//    cout << std::bitset<8>(d8[i]) << " ";
//  }
//  cout << endl;

  return 0;
}
