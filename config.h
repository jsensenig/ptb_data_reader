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

#define FIRMWARE_REVISION 0x5

//#define NO_PDS_DAC  1
#undef NO_PDS_DAC

#define PDUNE_COMPILATION 1
#undef SBND_COMPILATION

//#define STANDALONE_TIMING 1
#undef STANDALONE_TIMING

//#define DEBUG 1
#undef DEBUG


/// -- Define whether a standard mutex base code should be used, or
/// -- the contributed moodycamel lock-free implementation
//#define LOCKFREE 1
//#undef LOCKFREE
// -- use boost features
#define BOOST 1
//#define MUTEX
//#define LOG_MUTEX
/// -- Data transfer method for PL-PS.
/// -- Three options exist:
/// ARM_XDMA : The original implementation using the xdma polling kernel driver.
///            this driver does a simple polling DMA and works fine, except for the
///            fact that it is unbearably slow. Kept getting latencies of ~3 ms
///            on DMA data transfers
///				To use, one has to add the 'cma=NM' parameter to the bootargs
/// ARM_MMAP  : Memory mapped register. The process is simple and seems to work fine.
///             Data is collected one frame at a time and essentially implements my own
///             DMA by mapping 4 registers aligned in memory so that the contents can be collected
///             in a single transaction
/// ARM_SG_DMA : The Pothos driver-based SG DMA implementation. Latency depends on the size of the transfers
//              but typically are located in the ~2-3 us. It does not require any special boot paramters
///				but uses a substantially larger amount of resources in the PL.
#undef ARM_XDMA
#undef ARM_MMAP
#undef ARM_SG_DMA
#define ARM_SG_DMA 1
#undef SIMULATION

/// ENABLE_FRAG_BLOCKS
/// Define whether fragmented blocks are enabled or not. For the moment they are not
/// but eventually it might be better to enable them.
/// Enabling fragmented blocks might require some modifications on the board reader
/// to deal with the timestamp calculations.
//#define ENABLE_FRAG_BLOCKS 1
#undef ENABLE_FRAG_BLOCKS

#endif /* COMPILEROPTIONS_H_ */
