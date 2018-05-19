/*
 * ctbclient.cpp
 *
 *  Created on: May 7, 2018
 *      Author: nbarros
 */

#include "ctbclient.hh"
#include "json.hpp"
#include <iostream>
#include <sstream>
#include <boost/algorithm/string.hpp>

using std::cout;
using std::endl;
using json = nlohmann::json;

ctb_client::ctb_client(const std::string& host_name, const std::string& port_or_service, const unsigned int timeout_usecs) :
    socket_(io_service_),
    deadline_(io_service_),
    timeout_usecs_(timeout_usecs),
    exception_(false)
{

  printf("ctb_client::ctb_client: constructor\n");
  // Initialise deadline timer to positive infinity so that no action will be taken until a
  // deadline is set
  deadline_.expires_at(boost::posix_time::pos_infin);

  // Start the persistent deadline actor that checks for timeouts
  this->check_deadline();

  // Attempt to establish the connection to the PENN, using a timeout if necessary
  try
  {

    // Resolve possible endpoints from specified host and port/service
    tcp::resolver resolver(io_service_);
    tcp::resolver::iterator endpoint_iter = resolver.resolve(tcp::resolver::query(host_name, port_or_service));
    tcp::resolver::iterator end;

    while ((endpoint_iter != end) && (socket_.is_open() == false))
    {
      tcp::endpoint endpoint = *endpoint_iter++;
      printf("ctb_client::ctb_client: Connecting to PTB at %s:%hu\n",endpoint.address().to_string().c_str(),endpoint.port());

      // If a client timeout is specified, set the deadline timer appropriately
      this->set_deadline();

      // Set error code to would_block as async operations guarantee not to set this value
      boost::system::error_code error = boost::asio::error::would_block;

      // Start the asynchronous connection attempt, which will call the appropriate handler
      // on completion
      socket_.async_connect(endpoint, boost::bind(&ctb_client::async_connect_handler, _1, &error));

      // Run the IO service and block until the connection handler completes
      do
      {
        io_service_.run_one();
      }
      while (error == boost::asio::error::would_block);

      // If the deadline timer has been called and cancelled the socket operation before the
      // connection has established, a timeout occurred
      if (error == boost::asio::error::operation_aborted)
      {
        socket_.close();
        printf("ctb_client::ctb_client: Timeout establishing client connection to PTB at %s:%hu\n",
            endpoint.address().to_string().c_str(),endpoint.port());
        // TODO replace with exception
      }
      // If another error occurred during connect - throw an exception
      else if (error)
      {
        socket_.close();

        // JCF, Jul-29-2015

        // Swallow exception thrown; does not necessarily prevent datataking
        try {
          printf("ctb_client::ctb_client: Error establishing connection to PTB at %s:%hu : %s\n",
              endpoint.address().to_string().c_str(),endpoint.port(),
              error.message().c_str());
        } catch (...) {} // Swallow
      }
    }
    if (socket_.is_open() == false)
    {
      printf("ctb_client::ctb_client: Failed to open connection to the PTB\n");
      set_exception(true);

      throw(std::runtime_error("ctb_client::ctb_client: Failed to open connection to the PTB"));
    } else {

      // Flush the socket of any stale aysnc update data from PENN
      size_t bytesFlushed = 0;
      std::string data;
      do {
        data = this->receive();
        bytesFlushed += data.length();
      } while (data.length() > 0);
      printf("ctb_client::ctb_client: Flushed %zu bytes of stale data from client socket\n",bytesFlushed );

    }
  }
  catch (boost::system::system_error& e)
  {
    printf("ctb_client::ctb_client: Exception caught opening connection to PENN: %s \n",e.what());
  }

}

ctb_client::~ctb_client() {
  try {
    socket_.close();
  }
  catch (boost::system::system_error& e)
  {
    printf("ctb_client::ctb_client: Exception caught closing PennClient connection: %s \n",e.what());
  }
}

void ctb_client::send_command(std::string const & command)
{
  printf("ctb_client::ctb_client: Sending command: %s\n",command.c_str());

  json cmd;
  cmd["command"] = command;

  // Send it
  try {
    this->send_json(cmd);
  }
  catch (boost::system::system_error& e)
  {
    printf("ctb_client::send_command: Exception caught sending a command: %s \n",e.what());
  }

}

// ------------ Private methods ---------------

void ctb_client::set_deadline(void)
{
  // Set the deadline for the asynchronous write operation if the timeout is set, otherwise
  // revert back to +ve infinity to stall the deadline timer actor
  if (timeout_usecs_ > 0)
  {
    deadline_.expires_from_now(boost::posix_time::microseconds(timeout_usecs_));
  }
  else
  {
    deadline_.expires_from_now(boost::posix_time::pos_infin);
  }
}

void ctb_client::check_deadline(void)
{

  // Handle deadline if expired
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    // Cancel pending socket operation
    socket_.cancel();

    // No longer an active deadline to set the expiry to positive inifinity
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Put the deadline actor back to sleep
  deadline_.async_wait(boost::bind(&ctb_client::check_deadline, this));
}

void ctb_client::async_connect_handler(const boost::system::error_code& ec, boost::system::error_code* output_ec)
{
  *output_ec = ec;
}

void ctb_client::async_completion_handler(
    const boost::system::error_code &error_code, std::size_t length,
    boost::system::error_code* output_error_code, std::size_t* output_length)
{
  *output_error_code = error_code;
  *output_length = length;
}

std::size_t ctb_client::send(std::string const & send_str)
{

  if (!socket_.is_open()) {
    printf("ctb_client::send: Socket is not open. Won't be able to send any commend.\n");
  }

  // Set the deadline to enable timeout on send if appropriate
  this->set_deadline();

  // Set up variables to receive the result of the async operation. The error code is set
  // to would_block as ASIO guarantees that its async calls never fail with this value - any
  // other value therefore indicates completion or failure
  std::size_t send_len = 0;
  boost::system::error_code error = boost::asio::error::would_block;

  // Start the async operation. The asyncCompletionHandler function is bound as a callback
  // to update the error and length variables
  boost::asio::async_write(socket_, boost::asio::buffer(send_str),
      boost::bind(&ctb_client::async_completion_handler, _1, _2, &error, &send_len));

  // Block until the async operation has completed
  do
  {
    io_service_.run_one();
  }
  while (error == boost::asio::error::would_block);

  // Handle any error conditions arising during the async operation
  if (error == boost::asio::error::eof)
  {
    // Connection closed by peer
    printf("ctb_client::send: Connection closed by PENN\n");
  }
  else if (error == boost::asio::error::operation_aborted)
  {
    // Timeout signalled by deadline actor
    printf("ctb_client::send: Timeout sending message to PENN\n");
  }
  else if (error)
  {
    printf("ctb_client::send: Error sending message to PENN: %s\n",error.message().c_str());
  }
  else if (send_len != send_str.size())
  {
    printf("ctb_client::send: Size mismatch when sending transaction: wrote %zu expected %zu\n",send_len ,send_str.size());
  }

  return send_len;
}

std::pair<boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type>, bool>
match_json( boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> begin,
                        boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> end)
{
  //static boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> ptr = 0;
  static std::size_t n_bkt = 0;
  boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> ptr = begin;
  for (ptr = begin; ptr != end; ++ptr)
  {
    if (*ptr == '{') {
      n_bkt++;
      continue;
    }
    if (*ptr == '}') {
      n_bkt--;
      if (n_bkt == 0) {
        return std::make_pair(ptr,true);
      }
    }
  }
  return std::make_pair(ptr, false);
}

std::string ctb_client::receive(void)
{
  this->set_deadline();
  std::size_t receive_length = 0;
  boost::system::error_code error = boost::asio::error::would_block;

  boost::asio::streambuf response;

  // -- What is the delimiter?
  //TODO: Check what are the parameters of this async_read_until
//  boost::asio::async_read_until(socket_, response, "\f",
//      boost::bind(&ctb_client::async_completion_handler, _1, _2, &error, &receive_length));
    boost::asio::async_read_until(socket_, response, match_json,
        boost::bind(&ctb_client::async_completion_handler, _1, _2, &error, &receive_length));

  do
  {
    io_service_.run_one();
  }
  while (error == boost::asio::error::would_block);

  boost::asio::streambuf::const_buffers_type bufs = response.data();
  return std::string(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + response.size());

}


/// -- The real workhorse of the class
/// It sends the commands and parses the output



void ctb_client::send_json(json & json_frag)
{


  // Send the JSON request to the PTB
  printf("ctb_client::send_json: Sending JSON fragment [%s]\n",json_frag.dump(2).c_str());
  std::size_t len = this->send(json_frag.dump());
  if (len != json_frag.dump().size()) {
    printf("ctb_client::send_json: Mismatch between sent size (%zu bytes) and expected size (%zu bytes)\n",
        len,json_frag.dump().size());
  }
  // Get the response
  //json doc;

  std::string response = "";

  int max_response_timeout_us = 10000000;
  int max_retries = max_response_timeout_us / timeout_usecs_;
  int retries = 0;
  bool first_brckt = false;
  size_t pos_start = 0, pos_end = 0 ;
  size_t brckt_count = 0;
  // The issue here is knowing when a response is complete.
  // We have to do something like in the ctb to count brackets

  json answer_doc;
  // -- response should in principle only call once with the necessary content
  size_t pos_aux = 0;
  while (retries++ < max_retries) {
    pos_aux = response.size();

    response += this->receive();
//    printf("ctb_client::send_json: Received a response with %zu chars accumulated [%s] (try %d)\n",
//        response.size(),
//        response.c_str(),
//        retries);
    // there was a successful transaction
    if (pos_aux != response.size())
    {
      break;
    }
  }

//  while ( retries++ < max_retries) {
//    size_t pos_aux = response.size();
//
//    response += this->receive();
//
//    printf("ctb_client::send_json: Received a response with %zu chars accumulated [%s] (try %d)\n",
//              response.size(),
//              response.c_str(),
//              retries);
    // -- In principle this should not really be necessary. Keep it for now
    for (std::size_t pos = pos_aux; pos < response.size(); pos++) {
      if (response.at(pos) == '{') {
        if (first_brckt) {
          first_brckt = false;
          pos_start = pos;
        }
        brckt_count++;
      }
      if (response.at(pos) == '}') {
        brckt_count--;
        if (brckt_count == 0) {
          pos_end = pos+1;
          // We have a full message
          // Print it
          try {
//            printf("ctb_client::send_json: Raw response [%s]\n",response.substr(pos_start,pos_end-pos_start).c_str());
            answer_doc = json::parse(response.substr(pos_start,pos_end-pos_start));
            if (pos_end == response.length()) {
              response.clear();
            } else {
              response = response.substr(pos_end);
            }
            pos = 0;
            pos_start = 0;
            first_brckt = true;
          }
        catch(json::exception &e) {
          printf("ctb_client::send_json: JSON Exception : %s\n",e.what());
        }
        catch(std::exception &e) {
          printf("ctb_client::send_json: STL exception : %s\n",e.what());
        }
        catch (...) {
          printf("ctb_client::send_json: Unidentified exception caught\n");
        }
      }
    }
  }

    if(answer_doc.size() == 0) {
      printf("Answer seems to be empty. This should never happen\n");
      return;
    }
//      printf("%s\n",answer_doc.dump(2).c_str());
//      break;
//    }
//  }//while(retries)

  // -- Answer documents are relatively simple.
  // They can only contain a feedback object

  try {
    if (answer_doc.find("feedback") == answer_doc.end()) {
      printf("ctb_client::send_json: Unknown document structure: [%s]\n",answer_doc.dump().c_str());
    }
    else {

      for (json::iterator it = answer_doc.at("feedback").begin(); it != answer_doc.at("feedback").end(); ++it) {
        std::cout << it->dump(2) << std::endl;
        //std::cout << it.key() << " : " << it.value() << std::endl;
      }

//      // Parse the objects one by one
//      std::vector<json> content = answer_doc.at("feedback");
//      // -- grab the vector of feedback documents
//      // -- They will come in order of declaration
//      for (std::vector<json>::iterator it = content.begin(); it != content.end(); ++it)
//      {
//        if (it->at("type") == std::string("info"))
//        {
//          printf("ctb_client::send_json: INFO : %s\n",it->at("message").get<std::string>().c_str());
//          if (it->find("where")!= it->end()) {
//            printf("--> Extra info : %s\n",it->at("where").get<std::string>().c_str());
//          }
//        }
//        if (it->at("type") == std::string("error"))
//        {
//          printf("ctb_client::send_json: ERROR : %s\n",it->at("message").get<std::string>().c_str());
//          if (it->find("where")!= it->end()) {
//            printf("--> Extra info : %s\n",it->at("where").get<std::string>().c_str());
//          }
//        }
//
//        if (it->at("type") == std::string("statistics"))
//        {
//          for (json::iterator iti = it->begin(); iti != it->end(); ++iti) {
//            std::cout << *iti << '\n';
//          }
//        }
//      }
      answer_doc.clear();
    }
  }
  catch(json::exception &e) {
    printf("ctb_client::send_json: ERROR :JSON exception %s \n",e.what());
  }
  catch(std::exception &e) {
    printf("ctb_client::send_json: ERROR :STL exception %s \n",e.what());

  }
  catch(...) {
    printf("ctb_client::send_json: ERROR :Unknown exception \n");
  }
}

