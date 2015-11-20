/*
 * util.h
 *
 *  Created on: Jun 25, 2015
 *      Author: nbarros
 */

#ifndef UTIL_H_
#define UTIL_H_
#include <cstdlib>

uint32_t CreateMask(uint32_t begin, uint32_t end);
uint32_t SetBitRange(uint32_t content, uint32_t value, uint32_t pos, uint32_t len);
const std::string currentDateTime();

#endif /* UTIL_H_ */
