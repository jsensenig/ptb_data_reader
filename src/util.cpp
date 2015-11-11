/*
 * util.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

extern "C" {
#include <stdint.h>
};

#include <ctime>
#include <string>

#include "util.h"
#include "Logger.h"

uint32_t CreateMask(uint32_t begin, uint32_t end) {

  if (begin > end) {
    Log(error,"Trying to create an invalid mask [%u,%u]", begin ,end );
    return 0;
  }
  uint32_t mask = 0;
  for (uint32_t i = begin; i <= end; ++i) {
    mask |= 1 << i;
  }
  return mask;
}


uint32_t SetBitRange(uint32_t content, uint32_t value, uint32_t pos, uint32_t len) {
	uint32_t mask = (((uint32_t)1 << len)-1) << pos;
	uint32_t result = (content & ~mask) | (value & mask);
	return result;
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

