/*
 * util.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

extern "C" {
#include <stdint.h>
};
#include "util.h"
#include "Logger.h"

uint32_t CreateMask(uint32_t begin, uint32_t end) {

  if (begin > end) {
    Log(error) << "Trying to create an invalid mask [" << begin << "," << end << "]."<< endlog;
    return 0;
  }
  uint32_t mask = 0;
  for (uint32_t i = begin; i <= end; ++i) {
    mask |= 1 << i;
  }
  return mask;
}



