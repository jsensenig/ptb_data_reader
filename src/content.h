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

///
/// -- TCP header
///
typedef struct tcp_header_t {
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
///
/// -- payload
///

typedef struct payload_t {
	uint32_t payload1;
	uint32_t payload2;
	uint32_t payload3;
} payload_t;

typedef union payload {
	payload_t pstruct;
	uint32_t  content32[3];
	uint8_t   content8[12];
} payload;

typedef struct word_t {
	uint32_t header;
	payload  body;

} word_t;

typedef union word{
	word_t frame;
	uint8_t *get_word() {return reinterpret_cast<uint8_t*>(&frame);}
} word;


typedef struct header_t {
	uint8_t  padding     : 2;
	uint32_t ts_rollover : 27;
	uint8_t  word_type   : 3;

  static size_t const size_bytes = sizeof(uint32_t);
  static size_t const size_u32 = sizeof(uint32_t)/sizeof(uint32_t);

  static size_t const n_bits_padding  = 2;
  static size_t const n_bits_rollover = 27;
  static size_t const n_bits_type     = 3;

  // Calculates the full timestamp from the rollover
  // using the next full TS as reference
  uint64_t get_ts_from_next_full(const uint64_t &next_full_ts ) {
    static uint32_t rollover_mask =((0x1 << n_bits_rollover)-1);
    uint32_t full_ts_rollover = next_full_ts & rollover_mask;
    if (full_ts_rollover == ts_rollover) {
      // the rollover is the same as the previous TS
      // just return the reference
      return next_full_ts;
    } else if (full_ts_rollover > ts_rollover) {
      // there was no bit rollover
      return next_full_ts - (full_ts_rollover - ts_rollover);
    } else {
      // the rollover rolled over 0
      return next_full_ts - (full_ts_rollover + (rollover_mask - ts_rollover));
    }
  }

  // Same as above, but using the full TS immediately before
  uint64_t get_ts_from_next_full(const uint64_t &prev_full_ts ) {
    static uint32_t rollover_mask =((0x1 << n_bits_rollover)-1);
    uint32_t full_ts_rollover = prev_full_ts & rollover_mask;
    if (full_ts_rollover == ts_rollover) {
      // the rollover is the same as the previous TS
      // just return the reference
      return prev_full_ts;
    } else if (full_ts_rollover > ts_rollover) {
      // there was no bit rollover
      return prev_full_ts + (rollover_mask - full_ts_rollover) + ts_rollover;
    } else {
      // the rollover rolled over 0
      return prev_full_ts + (ts_rollover - full_ts_rollover);
    }
  }
} header_t;

typedef union header {
    header_t word;
    uint32_t value;
} header;

namespace payload {
  typedef struct warning_t {
      uint32_t padding  : 16;
      uint16_t code     : 16;
  } warning_t;

  typedef union warning {
      warning_t word;
      uint32_t  value;
  } warning;

  // there are 67 data bits
  // everything else up to the size of a full word payload (96 bits)
  // is padding
  // FIXME: Decide if the padding should be added to the lsb or msb
  typedef struct ch_status_t {
      typedef uint64_t channel_set_t;
      channel_set_t crt     : 32;
      channel_set_t pds     : 24;
      channel_set_t beam    : 11;
      channel_set_t padding : 29;

      static size_t const size_bytes = 2*sizeof(uint64_t);
      static size_t const size_u32 = 2*sizeof(uint64_t)/sizeof(uint32_t);

      static size_t const n_bits_crt      = 32;
      static size_t const n_bits_pds      = 24;
      static size_t const n_bits_beam     = 11;
      static size_t const n_bits_padding  = 29;

      // aux_functions
      bool get_state_crt(const uint16_t bit_pos) {
        return ((crt & (0x1 << bit_pos)) != 0x0);
      }
      bool get_state_pds(const uint16_t bit_pos) {
        return ((pds & (0x1 << bit_pos)) != 0x0);
      }
      bool get_state_beam(const uint16_t bit_pos) {
        return ((beam & (0x1 << bit_pos)) != 0x0);
      }
  } ch_status_t;

//  typedef union ch_status {
//      ch_status_t word;
//
//  } ch_status;

    // How big should a trigger word be?
    typedef struct low_level_trigger_t {
        uint32_t padding1
    } low_level_trigger_t;

    // How big should a trigger word be?
    typedef struct global_trigger_t {
        uint32_t padding1
    } global_trigger_t;


} // -- namespace payload
} // -- namespace word
} // -- namespace content
} // -- namespace ptb




#endif /* SRC_CONTENT_H_ */
