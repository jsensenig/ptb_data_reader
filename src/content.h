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
          typedef uint16_t  code_size_t;
          typedef uint16_t  source_size_t;
          typedef uint8_t   wtype_size_t;
          typedef uint32_t  pad_size_t;

          ts_size_t     timestamp;
          code_size_t   code      : 16;
          source_size_t source    : 16;
          pad_size_t    padding   : 29;
          wtype_size_t  word_type : 3;


          static size_t const size_bytes = 2*sizeof(uint64_t);
          static size_t const size_u32 = size_bytes/sizeof(uint32_t);

          static size_t const n_bits_timestamp  = 64;
          static size_t const n_bits_payload = 32;
          static size_t const n_bits_type     = 3;

      } feedback_t;


      typedef struct ch_status_t {
           typedef uint64_t ts_size_t;
           typedef uint32_t pds_size_t;
           typedef uint32_t crt_size_t;
           typedef uint16_t bi_size_t;
           typedef uint8_t wtype_size_t;

           ts_size_t     timestamp  : 60;
           bi_size_t     beam       : 9;
           pds_size_t    pds        : 16;
           crt_size_t    crt        : 29;
           wtype_size_t  word_type  : 3;


           static size_t const size_bytes = 2*sizeof(uint64_t);
           static size_t const size_u32 = size_bytes/sizeof(uint32_t);

           static size_t const n_bits_timestamp  = 60;
           static size_t const n_bits_payload = 32;
           static size_t const n_bits_type     = 3;


           // aux_functions
           bool get_state_crt(const uint16_t channel) {
             return ((crt & (0x1 << channel)) != 0x0);
           }
           bool get_state_pds(const uint16_t channel) {
             return ((pds & (0x1 << channel)) != 0x0);
           }
           bool get_state_beam(const uint16_t channel) {
             return ((beam & (0x1 << channel)) != 0x0);
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

           mask_size_t  trigger_mask       : 61;
           wtype_size_t word_type : 3;

           static size_t const size_bytes = 2*sizeof(uint64_t);
           static size_t const size_u32 = size_bytes/sizeof(uint32_t);

           static size_t const n_bits_timestamp = 64;
           static size_t const n_bits_tmask     = 61;
           static size_t const n_bits_type      = 3;

       } trigger_t;



//
//
//
//
//
//
//
//      typedef struct body_t {
//          uint8_t data[12];
//
//          static size_t const size_bytes = 12*sizeof(uint8_t);
//          static size_t const size_u32 = 12*sizeof(uint8_t)/sizeof(uint32_t);
//      } body_t;
//
//      typedef union body {
//          body_t    pstruct;
//          uint32_t  content32[3];
//          uint8_t   content8[12];
//      } body;
//
//      typedef struct header_t {
//          uint8_t  padding     : 2;
//          uint32_t ts_rollover : 27;
//          uint8_t  word_type   : 3;
//
//          static size_t const size_bytes = sizeof(uint32_t);
//          static size_t const size_u32 = sizeof(uint32_t)/sizeof(uint32_t);
//
//          static size_t const n_bits_padding  = 2;
//          static size_t const n_bits_rollover = 27;
//          static size_t const n_bits_type     = 3;
//
//          // Calculates the full timestamp from the rollover
//          // using the next full TS as reference
//          uint64_t get_ts_from_next_full(const uint64_t &next_full_ts ) {
//            static uint32_t rollover_mask =((0x1 << n_bits_rollover)-1);
//            uint32_t full_ts_rollover = next_full_ts & rollover_mask;
//            if (full_ts_rollover == ts_rollover) {
//              // the rollover is the same as the previous TS
//              // just return the reference
//              return next_full_ts;
//            } else if (full_ts_rollover > ts_rollover) {
//              // there was no bit rollover
//              return next_full_ts - (full_ts_rollover - ts_rollover);
//            } else {
//              // the rollover rolled over 0
//              return next_full_ts - (full_ts_rollover + (rollover_mask - ts_rollover));
//            }
//          }
//
//          // Same as above, but using the full TS immediately before
//          uint64_t get_ts_from_prev_full(const uint64_t &prev_full_ts ) {
//            static uint32_t rollover_mask =((0x1 << n_bits_rollover)-1);
//            uint32_t full_ts_rollover = prev_full_ts & rollover_mask;
//            if (full_ts_rollover == ts_rollover) {
//              // the rollover is the same as the previous TS
//              // just return the reference
//              return prev_full_ts;
//            } else if (full_ts_rollover > ts_rollover) {
//              // there was no bit rollover
//              return prev_full_ts + (rollover_mask - full_ts_rollover) + ts_rollover;
//            } else {
//              // the rollover rolled over 0
//              return prev_full_ts + (ts_rollover - full_ts_rollover);
//            }
//          }
//
//      } header_t;
//
//      typedef union header {
//          header_t word;
//          uint32_t value;
//      } header;
//
//      typedef struct word_t {
//          body_t   wbody;
//          header_t wheader;
//
//          static size_t const word_size_bytes   = 4*sizeof(uint32_t); //128 bit payload.
//          static size_t const word_size_u32     = 4; //128 bit payload.
//
//      } word_t;
//
//      typedef union word{
//          word_t frame;
//          uint8_t *get_bytes() {return reinterpret_cast<uint8_t*>(&frame);}
//      } word;
//
//
//
//      // -- This holds the strucure of the different payloads that are possible
//      // All these structures are meant to be cast into a 12 byte word
//
//      // FIXME: -- May be worth refactor this into 16 bytes and just set the header as padding
//      namespace payload {
//        // Warning pre_computed words
//
//
//        // The warning only uses the top 32 bits
//        typedef struct feedback_t {
//            uint64_t padding;
//            uint32_t code   : 16;
//            uint32_t source : 16;
//        } feedback_t;
//
//        //        typedef union warning {
//        //            warning_t word;
//        //            uint32_t  value;
//        //        } warning;
//
//
//
//
//
//        // there are 67 data bits
//        // everything else up to the size of a full word payload (96 bits)
//        // is padding
//        // FIXME: Decide if the padding should be added to the lsb or msb
//        typedef struct ch_status_t {
//            typedef uint32_t channel_set_t;
//            channel_set_t padding : 23;
//            channel_set_t beam    : 9;
//            channel_set_t crt     : 32;
//            channel_set_t pad1 : 8;
//            channel_set_t pds     : 24;
//
//            static size_t const size_bytes      = 2*sizeof(uint64_t);
//            static size_t const size_u32        = 2*sizeof(uint64_t)/sizeof(uint32_t);
//
//            static size_t const n_bits_crt      = 32;
//            static size_t const n_bits_pds      = 24;
//            static size_t const n_bits_beam     = 9;
//            static size_t const n_bits_padding  = 29;
//
//            // aux_functions
//            bool get_state_crt(const uint16_t bit_pos) {
//              return ((crt & (0x1 << bit_pos)) != 0x0);
//            }
//            bool get_state_pds(const uint16_t bit_pos) {
//              return ((pds & (0x1 << bit_pos)) != 0x0);
//            }
//            bool get_state_beam(const uint16_t bit_pos) {
//              return ((beam & (0x1 << bit_pos)) != 0x0);
//            }
//        } ch_status_t;
//
//        //  typedef union ch_status {
//        //      ch_status_t word;
//        //
//        //  } ch_status;
//        //#endif
//        // How big should a trigger word be?
//        // Let's go with 32 bits
//        // 16 for trigger ID
//        // 16 for subsystem
//
//        typedef struct timestamp_t {
//            uint32_t padding;
//            uint32_t time_low;
//            uint32_t time_up;
//
//            uint64_t timestamp() {return static_cast<uint64_t>(time_up) << 32 | time_low;}
//        } timestamp_t;
//
//        //        typedef union timestamp {
//        //            timestamp_t word;
//        //            uint64_t    value;
//        //        } timestamp;
//
//
//        // --
//        typedef struct low_level_trigger_t {
//            uint64_t mask : 61;
//            uint64_t padding: 3;
//            uint32_t padding_high;
//        } low_level_trigger_t;
//
//
//        typedef struct low_trigger_t {
//            uint64_t padding_low;
//            trigger_code_t trigger_id : 16;
//            uint32_t subsystem  : 8;
//            uint32_t padding    : 8;
//        } low_trigger_t;
//
//        //        typedef union low_trigger {
//        //            low_trigger_t word;
//        //            uint32_t      value;
//        //        } low_trigger;
//
//        // How big should a trigger word be?
//        typedef struct global_trigger_t {
//            //uint32_t word : 32;
//            uint64_t padding_low;
//            trigger_code_t trigger_id : 16;
//            uint32_t padding    : 16;
//        } global_trigger_t;
//
//        //        typedef union global_trigger {
//        //            global_trigger_t word;
//        //            uint32_t         value;
//        //        } global_trigger;
//
//      } // -- namespace payload

    } // -- namespace word
  } // -- namespace content
} // -- namespace ptb




#endif /* SRC_CONTENT_H_ */
