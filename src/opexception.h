/*
 * opexception.h
 *
 *  Created on: Nov 13, 2015
 *      Author: nbarros
 */

#ifndef OP_EXCEPTION_H_
#define OP_EXCEPTION_H_

#include <exception>
#include <string>

namespace ptb {
class op_exception: public std::exception {
  public:
    op_exception(const char *msg)  {msg_ = msg;};
    op_exception(std::string msg) {msg_ = msg;};

    op_exception();
    virtual ~op_exception();
    virtual const char * what() const noexcept {return msg_.c_str();};

  protected:
    std::string msg_;
};
}
#endif /* OP_EXCEPTION_H_ */
