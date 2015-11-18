/*
 * PTBexception.h
 *
 *  Created on: Nov 13, 2015
 *      Author: nbarros
 */

#ifndef PTBEXCEPTION_H_
#define PTBEXCEPTION_H_

#include <exception>
#include <string>

class PTBexception: public std::exception {
  public:
    PTBexception(const char *msg)  {msg_ = msg;};;
    PTBexception(std::string msg) {msg_ = msg;};

    PTBexception();
    virtual ~PTBexception();
    const char * what() {return msg_.c_str();};

  protected:
    std::string msg_;
};

#endif /* PTBEXCEPTION_H_ */
