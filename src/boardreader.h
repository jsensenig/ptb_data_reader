/*
 * PTBReadefr.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

#include "config.h"

#include <string>
#include <iostream>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <thread>
#if defined(BOOST)
#include <boost/lockfree/spsc_queue.hpp>
#elif defined(LOCKFREE)
#include "atomicops.h"
#include "readerwriterqueue.h"
#else
#include <queue>
#endif

#if defined(SIMULATION)
#else
#include "util.h"
 #if defined(ARM_XDMA)
 #include "xdma.h"
 #elif defined(ARM_SG_DMA) // needed to use the LocalRegister structure
//  struct pzdud_t;
#include "pothos_zynq_dma_driver.h"

#endif
#endif

#include "content.h"

class TCPSocket;

namespace ptb {

/**
 * This class is responsible for reading the DMA and transferring the data to
 *  the server.
 * \class PTBReader
 *
 * \details This class is kept simple on purpose. 
 * It does not talk with the [ConfigServer]
 * by design and all it does is really collect the data from the memory 
 * assembles the TCP packets and sends them to the board reader.
 *
 */
class board_reader {
 public:
  
    // NFB : The data structure specification had been moved outside into content.h

  /// Bunch of structures that help manipulating the data.
  /// These *must* be kept in sync with lbne-raw-data

#ifdef OLD_WORDS
    /// Header of ethernet packet
    struct Header {
    
    typedef uint32_t data_t;
    typedef uint16_t data_size_t;

    typedef uint8_t  format_version_t;
    typedef uint8_t  sequence_id_t;
    typedef uint16_t block_size_t;
    
    block_size_t     block_size     : 16;
    sequence_id_t    sequence_id    : 8;
    format_version_t format_version : 8;
    
    static size_t const size_words = sizeof(data_t);
    
    static data_size_t const num_bits_size        = 16;
    static data_size_t const num_bits_sequence_id = 8;
    static data_size_t const num_bits_format      = 8;
  };

  /// Internal warning word
    /// Internal warning word
    struct Warning_Word {
      typedef uint32_t data_t;
      typedef uint16_t data_size_t;

      typedef uint8_t  warning_type_t;
      typedef uint8_t  data_packet_type_t;

      uint32_t padding                        : 24;
      warning_type_t warning_type             : 5;
      data_packet_type_t data_packet_type     : 3;

      static size_t const size_words = sizeof(data_t);
      static data_size_t const num_bits_padding     = 24;
      static data_size_t const num_bits_warning     = 5;
      static data_size_t const num_bits_packet_type = 3;
    };

    /// Frame header : Common header to everything except warning words
    struct Payload_Header {
      typedef uint32_t data_t;
      typedef uint16_t data_size_t;

      typedef uint8_t  data_packet_type_t;
      typedef uint32_t short_nova_timestamp_t;

      // The order of the data packet type and the timestamp have been
      // swapped to reflect that it's the MOST significant three bits in
      // the payload header which contain the type. I've also added a
      // 2-bit pad to reflect that the 2 least significant bits are unused.

      uint8_t padding                             : 2;
      short_nova_timestamp_t short_nova_timestamp : 27;
      data_packet_type_t     data_packet_type     : 3;

      static size_t const size_bytes = sizeof(data_t);
      static size_t const size_u32 = sizeof(data_t)/sizeof(uint32_t);

      static data_size_t const num_bits_padding       = 2;
      static data_size_t const num_bits_short_tstamp  = 27;
      static data_size_t const num_bits_packet_type   = 3;

      // This function converts the TS rollover into a full TS
      // it takes the payload of the timestamp word that came after this word
      // (the microslice border)
      // and calculates the offset
      // to use the full timestamp received before use the other method (pre)
      uint64_t get_full_timestamp_pre(uint64_t ts_ref) {
        if ((ts_ref & 0x7FFFFFF) == short_nova_timestamp) {
          // they are equal. This word is right on the border of the microslice
          return ts_ref;
        } else if ((ts_ref & 0x7FFFFFF) > short_nova_timestamp) {
          // there was no bit rollover in between
          return ts_ref - ((ts_ref & 0x7FFFFFF) - short_nova_timestamp);
        } else {
          // it rolled over.
          // Be sure of the values being set
          return ts_ref - ((ts_ref & 0x7FFFFFF) + (0x7FFFFFF - short_nova_timestamp));
        }
      }
      // Does the same as the previous but with the timestamp that came before
      // They should be equivalent but need to check to confirm
      uint64_t get_full_timestamp_post(uint64_t ts_ref) {
        if ((ts_ref & 0x7FFFFFF) == short_nova_timestamp) {
          // they are equal. This word is right on the border of the microslice
          return ts_ref;
        } else if ((ts_ref & 0x7FFFFFF) > short_nova_timestamp) {
          // it rolled over. Has to sum the short and the difference
          // that takes for the reference to roll over
          return ts_ref + (0x7FFFFFF - ((ts_ref & 0x7FFFFFF))) + short_nova_timestamp;
        } else {
          // it didn't roll over. Just add the difference of the rollovers
          return ts_ref + (short_nova_timestamp - (ts_ref & 0x7FFFFFF));
        }
      }


    };

    
    /// Counter payload description
    struct Payload_Counter {
      typedef uint64_t counter_set_t;
      typedef uint16_t data_size_t;
      // -- Must be careful to follow the right order
      // from lsb to msb it is
      // -- TSU mappings
      counter_set_t tsu_wu     : 10;
      counter_set_t tsu_el     : 10;
      counter_set_t tsu_extra  :  4;
      counter_set_t tsu_nu     :  6;
      counter_set_t tsu_sl     :  6;
      counter_set_t tsu_nl     :  6;
      counter_set_t tsu_su     :  6;
      // -- BSU mappings
      counter_set_t bsu_rm     : 16;//end of first counter_set_t==uint64_t
      counter_set_t bsu_cu     : 10;
      //FIXME: The panels should be remapped so that bsu_cl would be all together
      counter_set_t bsu_cl1    : 6;
      counter_set_t bsu_extra  : 1;
      counter_set_t bsu_cl2    : 7;
      counter_set_t bsu_rl     : 10;
      // Just ignore the rest of the word
      counter_set_t padding    : 30;

      static data_size_t const num_bits_tsu_wu    = 10;
      static data_size_t const num_bits_tsu_el    = 10;
      static data_size_t const num_bits_tsu_extra =  4;
      static data_size_t const num_bits_tsu_nu    =  6;
      static data_size_t const num_bits_tsu_sl    =  6;
      static data_size_t const num_bits_tsu_nl    =  6;
      static data_size_t const num_bits_tsu_su    =  6;
      static data_size_t const num_bits_bsu_rm    = 16;
      static data_size_t const num_bits_bsu_cu    = 10;
      static data_size_t const num_bits_bsu_cl1   = 6;
      static data_size_t const num_bits_bsu_extra = 1; // -- unused channel (32)
      static data_size_t const num_bits_bsu_cl2   = 7;
      static data_size_t const num_bits_bsu_rl    = 10;
      static data_size_t const num_bits_padding   = 30;
      static data_size_t const num_bits_bsu_cl    = 13;

      // position boundaries for the different counter types
      // it also works as a sort of enumerator for the different counter types
      // so that they can be uniquely identified
      // the value is the first element outside the type
      static data_size_t const counter_type_tsu_wu    = 10;
      static data_size_t const counter_type_tsu_el    = 20;
      static data_size_t const counter_type_tsu_extra = 24;
      static data_size_t const counter_type_tsu_nu    = 30;
      static data_size_t const counter_type_tsu_sl    = 36;
      static data_size_t const counter_type_tsu_nl    = 42;
      static data_size_t const counter_type_tsu_su    = 48;
      static data_size_t const counter_type_bsu_rm    = 64;
      static data_size_t const counter_type_bsu_cu    = 74;
      static data_size_t const counter_type_bsu_cl1   = 80;
      static data_size_t const counter_type_bsu_extra = 81; // -- unused channel (32)
      static data_size_t const counter_type_bsu_cl2   = 88;
      static data_size_t const counter_type_bsu_rl    = 98;
      static data_size_t const counter_type_padding   = 128;
      // special types that are only used in the enum
      static data_size_t const counter_type_unknown   = 129;
      static data_size_t const counter_type_bsu_cl    = 130;

      static size_t const size_bytes = 2*sizeof(uint64_t);
      static size_t const size_u32 = size_bytes/sizeof(uint32_t);

      // The size that arrives from the PTB
      static size_t const size_words_ptb_u32 = 3;
      static size_t const size_words_ptb_bytes = size_words_ptb_u32*sizeof(uint32_t);
      // The offset to grab the data
      static size_t const ptb_offset_u32 = 0;
      static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);

      // The payload position offset from the top of the frame (header + discard)
      static size_t const payload_offset_u32 = 1+ptb_offset_u32;
      static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);

      counter_set_t get_bsu_cl() {return ((bsu_cl2 << 10) | (bsu_cl1));};

      bool get_tsu_wu(uint32_t idx) {
        return ((tsu_wu & (1 << idx)) != 0x0);
      }
      bool get_tsu_el(uint32_t idx) {
        return ((tsu_el & (1 << idx)) != 0x0);
      }
      bool get_tsu_extra(uint32_t idx) {
        return ((tsu_extra & (1 << idx)) != 0x0);
      }
      bool get_tsu_nu(uint32_t idx) {
        return ((tsu_nu & (1 << idx)) != 0x0);
      }
      bool get_tsu_sl(uint32_t idx) {
        return ((tsu_sl & (1 << idx)) != 0x0);
      }
      bool get_tsu_nl(uint32_t idx) {
        return ((tsu_nl & (1 << idx)) != 0x0);
      }
      bool get_tsu_su(uint32_t idx) {
        return ((tsu_su & (1 << idx)) != 0x0);
      }
      // BSU getters
      bool get_bsu_rm(uint32_t idx) {
        return ((bsu_rm & (1 << idx)) != 0x0);
      }
      bool get_bsu_cu(uint32_t idx) {
        return ((bsu_cu & (1 << idx)) != 0x0);
      }
      // The CL is a bit special due to the crazy mapping
      bool get_bsu_cl(uint32_t idx) {
        if (idx <  num_bits_bsu_cl1) {
          return ((bsu_cl1 & (1 << idx)) != 0x0);
        } else {
          return ((bsu_cl2 & (1 << (idx-num_bits_bsu_cl1))) != 0x0);
        }
      }
      bool get_bsu_rl(uint32_t idx) {
        return ((bsu_rl & (1 << idx)) != 0x0);
      }

      bool get_counter_status(uint32_t idx) const {
        // This was tested to return the right bits in the right order in a standalone test
        // However, it is up to the user to know what type this bit corresponds to
        return (((reinterpret_cast<const uint8_t*>(this)[idx/8]) & (1 << (idx%8))) != 0x0);
      }

      static data_size_t get_counter_type(uint32_t bit_idx) {
        // NFB: By construction bit_idx cannot be negative
        //    if (bit_idx < 0) return counter_type_unknown; else
        if (bit_idx < counter_type_tsu_wu) return counter_type_tsu_wu;
        else if (bit_idx < counter_type_tsu_el) return counter_type_tsu_el;
        else if (bit_idx < counter_type_tsu_extra) return counter_type_tsu_extra;
        else if (bit_idx < counter_type_tsu_nu) return counter_type_tsu_nu;
        else if (bit_idx < counter_type_tsu_sl) return counter_type_tsu_sl;
        else if (bit_idx < counter_type_tsu_nl) return counter_type_tsu_nl;
        else if (bit_idx < counter_type_tsu_su) return counter_type_tsu_su;
        else if (bit_idx < counter_type_bsu_rm) return counter_type_bsu_rm;
        else if (bit_idx < counter_type_bsu_cu) return counter_type_bsu_cu;
        else if (bit_idx == (counter_type_bsu_extra-1)) return counter_type_bsu_extra;
        else if (bit_idx < counter_type_bsu_cl2) return counter_type_bsu_cl;
        else if (bit_idx < counter_type_bsu_rl) return counter_type_bsu_rl;
        else if (bit_idx < counter_type_padding) return counter_type_padding;
        else return counter_type_unknown;
      }
      // returns the index of the counter within its own group
      // for instance get_counter_type_pos(10) should return 0 as this is
      // the first counter in the tsu_el group
    static uint32_t get_counter_type_pos(uint32_t bit_idx) {
      if (bit_idx < counter_type_tsu_wu) return bit_idx;
      else if (bit_idx < counter_type_tsu_el) return (bit_idx - counter_type_tsu_wu);
      else if (bit_idx < counter_type_tsu_extra) return (bit_idx - counter_type_tsu_el);
      else if (bit_idx < counter_type_tsu_nu) return (bit_idx - counter_type_tsu_extra);
      else if (bit_idx < counter_type_tsu_sl) return (bit_idx - counter_type_tsu_nu);
      else if (bit_idx < counter_type_tsu_nl) return (bit_idx - counter_type_tsu_sl);
      else if (bit_idx < counter_type_tsu_su) return (bit_idx - counter_type_tsu_nl);
      else if (bit_idx < counter_type_bsu_rm) return (bit_idx - counter_type_tsu_su);
      else if (bit_idx < counter_type_bsu_cu) return (bit_idx - counter_type_bsu_rm);
      else if (bit_idx == (counter_type_bsu_extra-1)) return 0;
      else if (bit_idx < counter_type_bsu_cl1) return (bit_idx - counter_type_bsu_cu);
      // -- offset the extra position that is unused in between
      else if (bit_idx < counter_type_bsu_cl2) return (bit_idx - counter_type_bsu_cu -1);
      else if (bit_idx < counter_type_bsu_rl) return (bit_idx - counter_type_bsu_cl2);
      else if (bit_idx < counter_type_padding) return (bit_idx - counter_type_bsu_rl);
      else return counter_type_unknown;

    }

    };


    /// Trigger description
    struct Payload_Trigger {
      typedef uint32_t trigger_type_t;
      typedef uint16_t data_size_t;

      // The 16 lsb are padding. No information is passed there
      trigger_type_t padding_low : 8;

      // This is to be remapped so that calib words can be OR-ed with
      // calibration words
      // [8 : t_type][4 : muon_type][4 : calib type]
      trigger_type_t trigger_id_ext  : 4;
      trigger_type_t trigger_id_calib: 4; // which of the calibration channels
      trigger_type_t trigger_id_muon : 8; // which of the muon triggers
      trigger_type_t trigger_type    : 5; // the 5 msb are the trigger type
      trigger_type_t padding_high    : 3; // this makes the information byte aligned

      static data_size_t const num_bits_padding_low     = 8;
      static data_size_t const num_bits_trigger_id_ext  = 4;
      static data_size_t const num_bits_trigger_id_calib= 4;
      static data_size_t const num_bits_trigger_id_muon = 8;
      static data_size_t const num_bits_trigger_type    = 5;
      static data_size_t const num_bits_padding_high    = 3;

      // The logic below this point is a bit different since now multiple
      // triggers can be ID-ed in the same word
      // Better to ask for specific types


      // ID the trigger types for figure reference
      // The logic is flawed. Need to look at this otherwise
      /** Old map
      static trigger_type_t const calibration = 0x00; //00000
      static trigger_type_t const muon        = 0x10; //10000
      static trigger_type_t const ssp         = 0x08; //01000
      // -- This should probably be split into RCE and then RCE types
      static trigger_type_t const rce_1       = 0x01; //00001
      static trigger_type_t const rce_2       = 0x02; //00010
      static trigger_type_t const rce_12      = 0x03; //00011
      static trigger_type_t const rce_3       = 0x04; //00100
      static trigger_type_t const rce_13      = 0x05; //00101
      static trigger_type_t const rce_23      = 0x06; //00110
      static trigger_type_t const rce_123     = 0x07; //00111
       **/
      static trigger_type_t const muon        = 0x10; //10000
      static trigger_type_t const external    = 0x08; //01000
      static trigger_type_t const calibration = 0x04; //00100

      // -- This should probably be split into RCE and then RCE types
      // External trigger types
      static trigger_type_t const I01_08      = 0x8; //1000
      static trigger_type_t const ssp         = 0x8; //1000
      static trigger_type_t const I09         = 0x4; //0100
      static trigger_type_t const I10         = 0x2; //0010
      static trigger_type_t const I12         = 0x1; //0001
      // NOTE: There is no I11. It is a adead channel.

      // C1 : 1000 : 0x8
      // C2 : 0100 : 0x4
      // C3 : 0010 : 0x2
      // C4 : 0001 : 0x1
      static trigger_type_t const C1 = 0x8;
      static trigger_type_t const C2 = 0x4;
      static trigger_type_t const C3 = 0x2;
      static trigger_type_t const C4 = 0x1;

      // TA : BSU RM/CL : 10000000
      // TB : TSU SL/NU : 01000000
      // TC : TSU SU/NL : 00100000
      // TD : TSU EL/WU : 00010000
      // TE : ????????? : 00001000 <-- Not implemented in FW
      // TF : ????????? : 00000100 <-- Not implemented in FW
      // TG : ????????? : 00000010 <-- Not implemented in FW
      // TH : ????????? : 00000001 <-- Not implemented in FW
      static trigger_type_t const TA = 0x80;
      static trigger_type_t const TB = 0x40;
      static trigger_type_t const TC = 0x20;
      static trigger_type_t const TD = 0x10;
      static trigger_type_t const TE = 0x08;
      static trigger_type_t const TF = 0x04;
      static trigger_type_t const TG = 0x02;
      static trigger_type_t const TH = 0x01;

      static trigger_type_t const bsu_rm_cl = 0x80;
      static trigger_type_t const tsu_sl_nu = 0x40;
      static trigger_type_t const tsu_su_nl = 0x20;
      static trigger_type_t const tsu_el_wu = 0x10;

      static size_t const size_bytes = sizeof(uint32_t);
      static size_t const size_u32 = size_bytes/sizeof(uint32_t);

      // The number of u32 that have to offset from the PTB packet to grab the data that matters
      // In this case the two lsInts are empty and can be offset
      static size_t const ptb_offset_u32 = 2;
      static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);

      // The payload position offset from the top of the frame (header + discard)
      static size_t const payload_offset_u32 = 1+ptb_offset_u32;
      static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);



      ///Bunch of auxiliary functions to help parse the word
      ///
      bool has_muon_trigger() {
        return ((trigger_type & muon) != 0x0);
      }
      bool has_external_trigger() {
        return ((trigger_type & external) != 0x0);
      }
      bool has_calibration() {
        return ((trigger_type & calibration) != 0x0);
      }

      // -- External trigger types

      bool has_external_trigger(trigger_type_t t) {
        return ((trigger_id_ext & t) != 0x0);
      }

      bool has_ssp_trigger() {
        return ((trigger_id_ext & ssp) != 0x0);
      }
      bool has_I09() {
        return ((trigger_id_ext & I09) != 0x0);
      }
      bool has_I10() {
        return ((trigger_id_ext & I10) != 0x0);
      }
      bool has_I12() {
        return ((trigger_id_ext & I12) != 0x0);
      }

      // calibration trigger types
      /// Test for the different calibration types
      ///

      bool has_calibration(trigger_type_t t) {
        return ((trigger_id_calib & t) != 0x0);
      }

      bool has_C1() {
        return ((trigger_id_calib & C1) != 0);
      }
      bool has_C2() {
        return ((trigger_id_calib & C2) != 0);
      }
      bool has_C3() {
        return ((trigger_id_calib & C3) != 0);
      }
      bool has_C4() {
        return ((trigger_id_calib & C4) != 0);
      }

      /// Test the different muon trigger types
      ///
      bool has_muon_trigger(trigger_type_t t) {
        return ((trigger_id_muon & t) != 0x0);
      }
      ///
      bool has_muon_TA() {
        return ((trigger_id_muon & TA) != 0);
      }
      bool has_muon_TB() {
        return ((trigger_id_muon & TB) != 0);
      }
      bool has_muon_TC() {
        return ((trigger_id_muon & TC) != 0);
      }
      bool has_muon_TD() {
        return ((trigger_id_muon & TD) != 0);
      }
      bool has_muon_TE() {
        return ((trigger_id_muon & TE) != 0);
      }
      bool has_muon_TF() {
        return ((trigger_id_muon & TF) != 0);
      }
      bool has_muon_TG() {
        return ((trigger_id_muon & TG) != 0);
      }
      bool has_muon_TH() {
        return ((trigger_id_muon & TH) != 0);
      }

      /// Test the different muon trigger types
      ///
      bool has_muon_bsu_rm_cl() {
        return ((trigger_id_muon & bsu_rm_cl) != 0);
      }
      bool has_muon_tsu_sl_nu() {
        return ((trigger_id_muon & tsu_sl_nu) != 0);
      }
      bool has_muon_tsu_su_nl() {
        return ((trigger_id_muon & tsu_su_nl) != 0);
      }
      bool has_muon_tsu_el_wu() {
        return ((trigger_id_muon & tsu_el_wu) != 0);
      }


      // Add a function that can be used to parse the trigger payload
      //FIXME: This should be considered obsolete and removed
      // in a near future

      static std::string getTriggerTypeName(trigger_type_t trigger_type) {
        switch (trigger_type) {
          case calibration:
            return "calibration";
            break;
          case external:
            return "external";
            break;
          case muon :
            return "muon";
            break;
          default:
            return "unknown";
            break;
        }
        return "";
      }
    };

    struct Payload_Timestamp {
      typedef uint64_t timestamp_t;
      typedef uint16_t data_size_t;
      timestamp_t nova_timestamp : 64;

      static data_size_t const num_bits_timestamp = 64;
      static size_t const size_bytes = sizeof(uint64_t);
      static size_t const size_u32 = size_bytes/sizeof(uint32_t);
      // drop 1 int
      static size_t const ptb_offset_u32 = 1;
      static size_t const ptb_offset_bytes = ptb_offset_u32*sizeof(uint32_t);
      // The payload position offset from the top of the frame (header + discard)

      static size_t const payload_offset_u32 = 1+ptb_offset_u32;
      static size_t const payload_offset_bytes = payload_offset_u32*sizeof(uint32_t);

    };

  typedef Header::block_size_t microslice_size_t;
  
  //the size of the payloads (neglecting the Payload_Header)
  // FIXME: Since everythig is being worked in bytes the sizes can be trimmed
  static microslice_size_t const payload_size_counter   = 4*sizeof(uint32_t); //128 bit payload.
  static microslice_size_t const payload_size_trigger   = 1 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_timestamp = 2 * sizeof(uint32_t); //64-bit payload
  static microslice_size_t const payload_size_selftest  = 1 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_checksum  = 0 * sizeof(uint32_t); //32-bit payload
  static microslice_size_t const payload_size_warning   = 0 * sizeof(uint32_t); //32-bit payload
  
  //The types of data words
  static const Payload_Header::data_packet_type_t DataTypeWarning   = 0x0; //0b000
  static const Payload_Header::data_packet_type_t DataTypeCounter   = 0x1; //0b001
  static const Payload_Header::data_packet_type_t DataTypeTrigger   = 0x2; //0b010
  static const Payload_Header::data_packet_type_t DataTypeChecksum  = 0x4; //0b100
  static const Payload_Header::data_packet_type_t DataTypeTimestamp = 0x7; //0b111


#endif

  /** 
   * Implementation of the PTBReader class
   **/

  board_reader();
  
  virtual ~board_reader();
  
  bool get_ready() const {
    return ready_;
  }
  
  void set_ready(bool ready) {
    ready_ = ready;
  }

  const std::string& get_tcp_host() const {
    return tcp_host_;
  }

  void set_tcp_host(const std::string& tcpHost) {
    tcp_host_ = tcpHost;
  }

  unsigned short get_tcp_port() const {
    return tcp_port_;
  }

  void set_tcp_port(unsigned short tcpPort) {
    tcp_port_ = tcpPort;
  }

  // FIXME: Implement this along with the fragmented blocks
#ifdef ENABLE_FRAG_BLOCKS
  uint32_t get_packet_rollover() const {
    return packet_rollover_;
  }

  void set_packet_rollover(uint32_t packetRollover) {
    packet_rollover_ = packetRollover;
  }
#endif
  /**
   * Stop the client threads without touching the mutex.
   * Important in the case that one wants to delete the object 
   * without having started a run.
   */
  void clear_threads();

  /** 
   * Function called by the manager to cleanly stop 
   * the data transmission and connection
   */
  void stop_data_taking();

  /** Starts the collection and transmission threads
   * if the class was instanciated in "dry run" mode, 
   * it does nothing. 
   **/
  void start_data_taking();

  /**
   * Resets the buffers. The run should already be stopped.
   */
  void reset_buffers();
  
  /** Start the connection to the board reader **/
  void init_data_connection(bool force = false);
  void close_data_connection();

  bool test_socket();

  bool get_data_connection_valid() const {
    // This might not be necessarily the best option
    return (data_socket_ != NULL);
  }
  
  // -- Statistics methods:
  uint32_t get_n_sent_frags() {return num_eth_fragments_;};
  uint32_t get_n_status() {return num_word_counter_;};
  uint32_t get_n_triggers() {return num_word_trigger_;};
  uint32_t get_n_warns() {return num_word_warning_;};
  uint32_t get_n_timestamps() {return num_word_tstamp_;};
  uint32_t get_sent_bytes() {return bytes_sent_;};


  void set_dry_run(bool status) {dry_run_ = status;};

  bool get_dry_run() {return dry_run_;};

  bool get_error_state() {return error_state_;};

  std::string get_error_msgs() {return error_messages_;};

protected:
  /** Data collector function into the queue. Runs on it's own thread**/
  void data_collector();

  /** Data transmitter **/
  void data_transmitter();

  void dump_packet(uint32_t* buffer, uint32_t tot_size);
#ifdef ARM_SG_DMA
  void clean_and_shutdown_dma();
#endif /*ARM_SG_DMA*/
private:
//  static void * ClientCollectorFunc(void * this) {((board_reader *)This)->data_collector(); return NULL;}
//  static void * ClientTransmitterFunc(void * this) {((board_reader *)This)->data_transmitter(); return NULL;}

  // -- Structures for data socket connection

  unsigned short  tcp_port_;
  std::string     tcp_host_;

  TCPSocket *     data_socket_;

  std::thread     *client_thread_collector_;
  std::thread     *client_thread_transmitter_;

#ifdef ENABLE_FRAG_BLOCKS
  uint32_t packet_rollover_;
#endif
  bool ready_;



#if defined(SIMULATION)
  uint32_t * memory_pool_;
  void * mapped_data_base_addr_;

#elif defined(ARM_XDMA)
// declare a bunch of variables that are common to the program
  struct xdma_dev xdma_device;
  struct xdma_chan_cfg xdma_dst_cfg;
  struct xdma_buf_info xdma_buf;
  struct xdma_transfer xdma_trans;
  uint32_t *dma_buffer_;

#elif defined(ARM_MMAP)

  ///! Number of frames possible to buffer in the RAM memory
  /// Increased from 2M to 8M => 32MB
  static const uint32_t buffer_size_frames = 8*1024*1024;



  // Keeps frames stored
  ptb::util::mem_reg control_register_;
  ptb::util::mem_reg data_register_;
  uint32_t * memory_pool_;
  void * mapped_data_base_addr_;
#elif defined(ARM_SG_DMA)
  pzdud_t *s2mm;
  static const size_t num_buffs_ = 1024;
  static const size_t buff_size_ = 4096; // bytes
  uint32_t buff_addr_[num_buffs_];
#else
#error DMA mode not specified
#endif

#if defined(BOOST)
//  boost::lockfree::spsc_queue<ptb::content::buffer_t> buffer_queue_{num_buffs_};
  boost::lockfree::spsc_queue<ptb::content::buffer_t, boost::lockfree::capacity<num_buffs_> >buffer_queue_;
#elif defined(LOCKFREE)
  moodycamel::ReaderWriterQueue<uint32_t*> buffer_queue_;
#else
  std::queue<uint32_t*> buffer_queue_;
#endif

//  // Warning pre_computed words
//  static const uint32_t WARN_TIMEOUT = 0x04000000;
//  static const uint32_t WARN_UNKNOWN_DATA = 0x02000000;


  // A few auxiliary constants
  static const uint32_t eth_buffer_size_u32 = 0xFFFF; // Max possible ethernet packet
  static const uint32_t frame_size_bits   = 0x80;   // the buffer is 128 bits
  static const uint32_t frame_size_bytes  = 0x10;   // 16 bytes
  static const uint32_t frame_size_u32    = 0x4;    // 4xuint32_t
  // This is the buffer size in number of frames
  // Could easily do something else, or even set this as a configuration
  // parameter
  // FIXME: Change to a fhicl param


  // A few more constants that are important
  static const uint32_t fw_version_ = FIRMWARE_REVISION;

  // Frame sequence number
  uint32_t seq_num_;
#ifdef ENABLE_FRAG_BLOCKS
  bool fragmented_;
#endif
  bool keep_transmitting_;
  bool keep_collecting_;
  uint64_t time_rollover_;

  // -- Debugging and control variables
  // timeouts don't make sense with MMAP
#if defined(ARM_XDMA)
uint32_t timeout_cnt_;
const uint32_t timeout_cnt_threshold_ = 10000;
#endif

bool dry_run_; // Run the reader without collecting data
bool error_state_;
std::string error_messages_;

  // -- Internal run statistics
  uint32_t num_eth_fragments_;
  uint32_t num_word_counter_;
  uint32_t num_word_trigger_;
  uint32_t num_word_warning_;
  uint32_t num_word_tstamp_;
  uint32_t bytes_sent_;
  std::map<int,int> counter_stats_;
  std::map<ptb::content::word::trigger_code_t,int> trigger_stats_;

};

}

#endif /* PTBREADER_H_ */
