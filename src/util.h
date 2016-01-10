/*
 * util.h
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

#ifndef UTIL_H_
#define UTIL_H_
#include <cstdlib>
#include <sstream>
#include "Logger.h"

uint32_t CreateMask(uint32_t begin, uint32_t end);
uint32_t SetBitRange(uint32_t content, uint32_t value, uint32_t pos, uint32_t len);
const std::string currentDateTime();

typedef struct LocalRegister {
  void      *address;
  // Should use this method to access the value held in address
  // since it shortens the command
  volatile uint32_t& value () {return *(static_cast<volatile uint32_t*>(address));}
} LocalRegister;

// JCF, Jul-16-2015

// reinterpret_cast_checked will double check that the pointer we
// cast to actually points to the same location in memory as the
// pointer we cast from. This almost certainly is the case, but it's
// not an ironclad guarantee in the C++ standard, unfortunately.

template <typename S, typename T>
  S reinterpret_cast_checked(T inptr) {

  S outptr = reinterpret_cast<S>( inptr );

  const void* inptr_void = static_cast<const void*>( inptr );
  const void* outptr_void = static_cast<const void*>( outptr );

  if ( inptr_void != outptr_void ) {
    std::ostringstream msg;
    msg << "reinterpret_cast_checked::ERROR: casted " << inptr_void << " to " << outptr_void << " => results of the cast can't be trusted";
//    throw PTBexception(msg.str());
    Log(error,"%s",msg.str().c_str());
  }

  return outptr;
}

// PTBregisters declarations
void *MapPhysMemory(uint32_t base_addr, uint32_t high_addr);
void UnmapPhysMemory(void * address, size_t size, bool close_file = false);
uint32_t Xil_In32(uint32_t Addr);
void Xil_Out32(uint32_t OutAddress, uint32_t Value);

#define WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (uint32_t)(Data))

#define ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))

#endif /* UTIL_H_ */
