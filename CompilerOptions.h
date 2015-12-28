/*
 * CompilerOptions.h
 *
 *  Created on: Dec 28, 2015
 *      Author: nbarros
 */

#ifndef COMPILEROPTIONS_H_
#define COMPILEROPTIONS_H_

///
/// This file collects all the defines that are used in the code
///
///

/// -- Some extra debug operations. Add significant overhead and verbosity to code
//#define DEBUG 1
#undef DEBUG

/// -- Add data collection statistics.
/// -- The times are reported at the end of the run
#define MEASURE_PERFORMANCE 1
//#undef MEASURE_PERFORMANCE

/// -- Define whether a standard mutex base code should be used, or
/// -- the contributed moodycamel lock-free implementation
#define LOCKFREE 1

/// -- Data transfer method for PL-PS.
/// -- Three options exist:
/// ARM_XDMA : The original implementation using the xdma polling kernel driver.
///            this driver does a simple polling DMA and works fine, except for the
///            fact that it is unbearably slow. Kept getting latencies of ~3 ms
///            on DMA data transfers
/// ARM_POTHOS : Uses the better POTHOS kernel driver with scatter-gather. Should be much
///              faster but ran into problems synthesizing the firmware with SG enabled.
///              To be further investigated in the future. The code has never been truly tested.
/// ARM_MMAP  : Memory mapped register. The process is simple and seems to work fine.
///             Data is collected one frame at a time and essentially implements my own
///             DMA by mapping 4 registers aligned in memory so that the contents can be collected
///             in a single transaction
#undef ARM_XDMA
#undef ARM_POTHOS
#define ARM_MMAP 1

/// ENABLE_FRAG_BLOCKS
/// Define whether fragmented blocks are enabled or not. For the moment they are not
/// but eventually it might be better to enable them.
/// Enabling fragmented blocks might require some modifications on the board reader
/// to deal with the timestamp calculations.
//#define ENABLE_FRAG_BLOCKS 1
#undef ENABLE_FRAG_BLOCKS
///
///
/// OTHER OPTIONS. These are not documented and will require further investigation
/// THey affect contributed code
///
///
//#define PUGIXML_NO_EXCEPTIONS 1
//#define AE_USE_STD_ATOMIC_FOR_WEAK_ATOMIC 1
//#define PUGIXML_HAS_LONG_LONG

#endif /* COMPILEROPTIONS_H_ */
