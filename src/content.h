/*
 * content.h
 *
 *  Created on: Oct 8, 2017
 *      Author: nbarros
 */

#ifndef SRC_CONTENT_H_
#define SRC_CONTENT_H_

#include <cstdint>

namespace ptb {

  namespace content {

    // NFB: Note that order denotes bit order in the word:
    //      Upper fields are in the lsb and lower field is in the msb

    /// Internal buffer definition
    ///

    static uint8_t format_version = 0x2;

    typedef struct buffer_t {
        int       handle;
        size_t    len;
    } buffer_t;

    ///
    /// -- TCP header
    ///
    typedef struct tcp_header_t {
        typedef uint16_t pkt_size_t;
        typedef uint8_t   seq_size_t;
        typedef uint8_t   ver_size_t;
        uint16_t packet_size 	: 16; // Size of the data content in bytes
        uint8_t  sequence_id 	: 8; // packet order...rotates every 256
        uint8_t  format_version : 8;

        static size_t const size_bytes = sizeof(uint32_t);
        static size_t const n_bits_size 		= 16;
        static size_t const n_bits_sequence_id 	= 8;
        static size_t const n_bits_version 	    = 8;

    } tcp_header_t;

    // NFB: Careful with unions. Setting one member and then accessing another
    // is undefined behavior in C++.
    // However, I have tested that they work on gcc on the PTB
    typedef union tcp_header {
        tcp_header_t word;
        uint32_t 	 value;
    } tcp_header;

    namespace word {


      enum word_type {t_fback=0x0,t_lt=0x1,t_gt=0x2, t_ch=0x3,t_chksum=0x4,t_ts=0x7};



      ///
      /// -- payload
      ///

      // -- The body of a word is composed of 12 bytes
      typedef uint16_t trigger_code_t;


      ///
      /// N. Barros - May 7, 2018 : Changed the structure of the parsing
      ///
      /// Now all structures cast into 16 bytes, but declare different
      /// bit fields to interpret each frame in the context of its respective type
      ///
      /// Also, to move into a format where the full timestamp is stored, the full timestamp
      /// is now placed in the 64 lsb, instead of the msb. Does complies with the memory alignment
      /// and simplifies the parsing of the structures
      ///

      typedef struct word_t {
        typedef uint64_t ts_size_t;
        typedef uint64_t pad_size_t;
        typedef uint8_t  word_type_t;

        ts_size_t timestamp;
        pad_size_t payload : 61;
        word_type_t word_type : 3;

        static size_t const size_bytes = 4*sizeof(uint32_t);
        static size_t const size_u32 = 4*sizeof(uint32_t)/sizeof(uint32_t);

        static size_t const n_bits_timestamp  = 64;
        static size_t const n_bits_payload = 61;
        static size_t const n_bits_type     = 3;

      } word_t;


      typedef union word{
          word_t frame;
          uint8_t *get_bytes() {return reinterpret_cast<uint8_t*>(&frame);}
      } word;


      /// -- Several different structures that can be used to reinterpret the payload depending on
      /// the word type. All these structures map into the full 16 bytes of the CTB words
      ///
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

           static size_t const n_bits_timestamp = 64;
           static size_t const n_bits_unused    = 61;
           static size_t const n_bits_type      = 3;

       } timestamp_t;


       typedef struct trigger_t {
           typedef uint64_t ts_size_t;
           typedef uint64_t mask_size_t;
           typedef uint8_t  wtype_size_t;

           ts_size_t timestamp;

           mask_size_t  trigger_word       : 61;
           wtype_size_t word_type : 3;

           static size_t const size_bytes = 2*sizeof(uint64_t);
           static size_t const size_u32 = size_bytes/sizeof(uint32_t);

           static size_t const n_bits_timestamp = 64;
           static size_t const n_bits_tmask     = 61;
           static size_t const n_bits_type      = 3;

       } trigger_t;

    } // -- namespace word
  } // -- namespace content
} // -- namespace ptb




#endif /* SRC_CONTENT_H_ */
