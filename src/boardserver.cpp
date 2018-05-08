/*
 * ConfigServer.cc
 *
 *  Created on: Jun 2, 2015
 *      Author: nbarros
 *
 *  Revision History: 10-Jun-2015  - N. Barros:
 *                                    * Added threaded server configuration
 *                                    * Added forced shutdown
 */


// -- Local classes
#include "opexception.h"
#include "Logger.h"

// -- Contrib classes
// For Socket, ServerSocket, and SocketException
//TODO: Replace practical socket with boost socket manipulation
#include "PracticalSocket.h"

// -- STL classes
#include <string>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <csignal>
#include <queue>

#include "boardmanager.h"
#include "boardreader.h"
#include "boardserver.h"

#include "json.hpp"
using json = nlohmann::json;

using std::string;
namespace ptb {
//-- Assignment of the static variables

  std::string board_server::tcp_buffer_ = "";
  unsigned int board_server::num_instances_ = 0;
  unsigned int board_server::port_ = 8991;
  board_manager *board_server::board_manager_ = NULL;
  std::queue<std::string> board_server::queue_;
  bool board_server::shutdown_requested_= false;
  TCPSocket* board_server::client_socket_ = nullptr;
  //std::vector<std::string> board_server::msg_answers_;
  json board_server::msg_answers_;
//std::string board_server::msg_answer_ = "";
  // -- this is the amount of time the server waits before relaunching the listening socket
  const long long board_server::relaunch_wait_ = 3;


  board_server::board_server() {
  // Use the practical socket to establish a connection
    Log(verbose,"Building socket server");
    Log(verbose,"Attaching default board manager");
  // -- Construct a board manager along with it
    board_manager_ = new board_manager();

  }

  void board_server::clean_and_relaunch() {
	// if there is a data manager, tell it to stop taking data
	// If there is no data taking going on, no problems, it is ignored
    if (client_socket_ != nullptr) delete client_socket_;
    client_socket_ = nullptr;

    if (board_manager_ != nullptr) {
      Log(info,"Passing down a shutdown signal to the PTB.");
      board_manager_->exec_command("StopRun",msg_answers_);
      // -- Force a hard reset in the PTB to account for the case the
      // DAQ crashed.
      board_manager_->exec_command("HardReset",msg_answers_);
    }
    Log(warning,"Relaunching the socket for connection acceptance in %ld s.",relaunch_wait_);
    std::this_thread::sleep_for(std::chrono::seconds(relaunch_wait_));
  }

  void board_server::run() {
    Log(verbose,"Starting to listen for a new socket...");
    TCPServerSocket * servSock = nullptr;
  // The purpose of this first loop is to bypass a common problem if ghost binding
  // If for some reason one loses the connection and it takes some time
  // for the port bind to be released...this loop just says to keep trying
  // to bind every 5s
    do {
     try{
       Log(info,"Creating server listening socket...");
	      // This simply binds the port. Should not need to destroy and relaunch
	      //TCPServerSocket servSock(port_);   // Socket descriptor for server
       servSock = new TCPServerSocket(port_);
       Log(info,"Entering wait mode for client connection at port %u",port_);
     }
     catch(SocketException &e) {
       Log(error,"Socket exception caught : %s",e.what());
       if (servSock != nullptr) delete servSock;
       servSock = nullptr;
       Log(warning,"Relaunching the socket for connection acceptance in %ld s.",relaunch_wait_);
       std::this_thread::sleep_for(std::chrono::seconds(relaunch_wait_));
     }
     catch(...) {
      Log(error,"Unknown exception caught");
      if (servSock != nullptr) delete servSock;
      servSock = nullptr;
      Log(warning,"Relaunching the socket for connection acceptance in %lds.",relaunch_wait_);
      std::this_thread::sleep_for(std::chrono::seconds(relaunch_wait_));
    }
  } while (servSock == nullptr);

  // If we reach this stage, we now have a socket (client connection)
  Log(verbose,"Socket bound at port %u.",port_);
  while (true) {
   client_socket_ = nullptr;
   if (shutdown_requested_) {
    Log(info,"Received a shutdown request...");
    delete servSock;
    servSock = nullptr;
    return;
  }

  try{
    Log(debug,"Starting to listen for the connection...");
    client_socket_ = servSock->accept();
    Log(info,"Received client connection request.");
    handle_tcp_client();
	      // -- At this point we know that the client closed the connection
    Log(info,"Client connection closed. Clearing socket...");
    clean_and_relaunch();
  }
	  // -- Catch all possible exceptions here
	  // All will result in the connection being closed and the socket being reopened
  catch(SocketException &e) {
   Log(error,"Socket exception caught : %s",e.what());
   clean_and_relaunch();
 }
 catch(...) {
   Log(error,"An unspecified exception was caught. Cleaning up and relaunching.");
   clean_and_relaunch();
 }
 Log(warning,"Relaunching the socket for connection acceptance in %ld s.",relaunch_wait_);
 std::this_thread::sleep_for(std::chrono::seconds(relaunch_wait_));
}
Log(warning,"Reaching the end of execution, but not sure why...should *never* happen!");
}

// TCP client handling function
void board_server::handle_tcp_client(/*TCPSocket *sock */) {
if (client_socket_ == nullptr) {
  Log(error,"Refusing to handle a NULL socket.");
  return;
}
  // Test the quality of the socket
Log(debug,"Handling client...");
try {

  Log(debug,"Address : %s",client_socket_->getForeignAddress().c_str());;
} catch (SocketException &e) {
  Log(error,"Unable to get foreign address");
}
try {
  uint32_t localPort = client_socket_->getForeignPort();
  Log(debug,"Port : %u",localPort);
} catch (SocketException &e) {
  Log(error,"Unable to get foreign port");
}
Log(info,"Working  with thread 0x%x",std::this_thread::get_id());

  // Send received string and receive again until the end of transmission

//FIXME: Review this once the ethernet software is migrated to boost
  // Receive 1kB at a time.
  // Loop into the permanent buffer so that we know have all the configuration
  const size_t RCVBUFSIZE = 20480; //20 kB. Really hope will not overdo this.
  static char instBuffer[RCVBUFSIZE];

  int recvMsgSize;

  // All the transmissions are done in two parts. At the top comes a token
  // with the size of the incoming transmission
  // Then the packet itself follows.
  // Keep accepting until the </system> token comes in.
  // THis signals the end of the message

  std::string localBuffer;
  localBuffer.resize(RCVBUFSIZE);
  localBuffer = "";

  int brckt_count  = 0;
  bool first_brckt = true;
  size_t pos_start, pos_end;
  size_t pos_aux;
  // What if the string sent is bigger than the one being read?
  // zero means transmission is finished.
  while ((recvMsgSize = client_socket_->recv(instBuffer, RCVBUFSIZE-2)) > 0) {

    // Truncate the instantaneous buffer on the number of bytes
    instBuffer[recvMsgSize] = '\0';
    // end of transmission
    Log(debug,"Received %u bytes",recvMsgSize);
    Log(verbose," Inst buffer [%s]",instBuffer);
    // Append into the permanent buffer
    //cfg_buffer_ = tmpBuffer;

    // Keep accumulating the transmission until one of the closing
    // tokens are passed (</config></command>)

    Log(verbose,"Duplicating the input");
    pos_aux = localBuffer.size();
    localBuffer += instBuffer;
    Log(verbose,"Duplicated [%s] \n Length [%u]",localBuffer.c_str(),localBuffer.length());

    /** Logic for parsing a full JSON packet
     * We have more than one issue to deal with:
     *
     * 1. We want to have a net result of parenthesis of 0 (as many { as }
     * 2. We may receive piled up messages, so we have to scan all chars
     */


    for (std::size_t pos = pos_aux; pos < localBuffer.size(); pos++) {
      if (localBuffer.at(pos) == '{') {
        if (first_brckt) {
          first_brckt = false;
          pos_start = pos;
        }
        brckt_count++;
      }
      if (localBuffer.at(pos) == '}') {
        brckt_count--;
        if (brckt_count == 0) {
          pos_end = pos+1;
          // We have a full message
          tcp_buffer_ =localBuffer.substr(pos_start,pos_end-pos_start);
          Log(verbose,"new tcp buffer %d-%d [%s]",pos_start,pos_end,tcp_buffer_.c_str());
          if (pos_end == localBuffer.length()) {
           localBuffer.clear();
         } else { 
           localBuffer = localBuffer.substr(pos_end);
         }
         Log(verbose,"Cropped local buffer [%s]",localBuffer.c_str());
         pos = 0;
         pos_start = 0;
         first_brckt = true;

         try {
          process_request(tcp_buffer_.c_str(),msg_answers_);
        }
        catch(json::exception &e) {
          std::string msg = "JSON exception : ";
          msg += e.what();
          Log(error,"%s",msg.c_str());
          json obj;
          obj["type"]="error";
          obj["message"]=msg;
          msg_answers_.push_back(obj);
        }
        catch(std::exception &e) {
          std::string msg = "STD exception : ";
          msg += e.what();
          Log(error,"%s",msg.c_str());
          json obj;
          obj["type"]="error";
          obj["message"]=msg;
          msg_answers_.push_back(obj);
        }
        catch(ptb::op_exception &e) {
          std::string msg = "PTB exception : ";
          msg += e.what();
          Log(error,"%s",msg.c_str());
          json obj;
          obj["type"]="error";
          obj["message"]=msg;
          msg_answers_.push_back(obj);
        }
        catch (...) {
          std::string msg = "Unidentified exception caught";
          Log(error,"%s",msg.c_str());
          json obj;
          obj["type"]="error";
          obj["message"]=msg;
          msg_answers_.push_back(obj);
        }

          //Log(verbose,"Returning answer : %s",msg_answers_.c_str());
          // -- create a json object with all the messages
        json answer;
        answer["feedback"] = msg_answers_;

        Log(debug,"Answers : %s",answer.at("feedback").dump(2).c_str());

        client_socket_->send(answer.dump().c_str(),answer.dump().size());
        Log(debug,"Answer sent");
        msg_answers_.clear();

      }
    }
  }

  } // recv_size > =

  //
  Log(warning,"Connection to the client was lost.");
  Log(warning,"This socket will be closed.");
  // The socket is destroyed in clean_and_relaunch()
}


board_server::~board_server() {
  num_instances_--;
  if (num_instances_ <= 0) {
    Log(info,"Reached last instance. Destroying myself...or not?");
  }
}

void board_server::get_num_instances() const {
  Log(info,"Have currently %u in use!",num_instances_ );
}

/**
 * There is a bit of a conundrum here: on one hand we want to be able to parse the configuration
 * and on another we want to be able to pass commands to start and end the run.
 * Implement a register handle, that can be used to trigger the manager
 */
void board_server::process_request(const std::string &buffer,std::vector<std::string> &answer) {
  // Before doing any processing check that a DataManager has already
  // been registered. If not simply queue the command into a list and wait
  // Careful if the client connection in the meantime is lost, the memory will disappear.
  // Have to copy the string into the queue

  if (board_manager_ == NULL) {
    //answer.push_back("WARNING: Request received without valid board manager. Queueing the msg.");
    Log(warning,"Attempting to process a transmission without a valid data Manager. Queueing.");
    Log(verbose,"[%s]",buffer.c_str());
    json obj;
    obj["type"] = "warning";
    obj["message"] = "Attempting to process a transmission without a valid data Manager. Queueing";
    msg_answers_.push_back(obj);

    queue_.push(buffer);
    //    queue_.push_back(buffer);
    Log(debug,"Queue now composed of %u requests.",queue_.size());
    return;
  }
  Log(verbose,"Processing a transmission");
  Log(verbose,"[%s]",buffer.c_str());

  try {
  // -- We just need to figure out if this is a configuration or command message
    json doc = json::parse(buffer);
    std::string jd = doc.dump(4);
    Log(verbose,"Buffer [%s]",jd.c_str());
    if (doc.count("ctb") >= 1) {
    // we have a configuration document
      Log(debug,"Processing config");
      board_manager_->process_config(doc,msg_answers_);
    } else if (doc.count("command") >= 1) {
      Log(debug,"Processing command : %s",doc.at("command").get<std::string>().c_str());
      board_manager_->exec_command(doc.at("command").get<std::string>(),msg_answers_);
    } else {
      Log(error,"Unknown document type.");
      msg_answers_.push_back("ERROR: Unknown document type");
    }
  } catch(std::exception &e) {
    Log(error,"Caught an exception: %s",e.what());
  }

}

void board_server::register_board_manager(board_manager* newmgr) {
  Log(warning,"Replacing existing instance of board manager.");
  if (board_manager_!= nullptr) {
    // -- Should we copy the local information? Assuming not.
    delete board_manager_;
  }
  board_manager_ = newmgr;
}

void board_server::shutdown(bool force) {
  shutdown_requested_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  Log(warning,"Shutdown requested.");
  // Check if force is passed
  if (force) {
   Log(warning,"Forcing my way out...");
   clean_and_relaunch();
   delete board_manager_;

 } else if (queue_.size() != 0){
    // Soft shutdown requested. Do nothing and wait for the client to disconnect
  Log(warning,"Soft shutdown requested with non-empty queue. Shutdown will occur when queue empties.");

    // Process the queue
  while (queue_.size() > 0) {
    Log(verbose,"Processing an entry");
    process_request(queue_.front(), msg_answers_);
//      Log(verbose,"Transmission processed. Answer: %s",msg_answers_.c_str());
    queue_.pop();
  }
} else {
  Log(info,"Performing a soft shutdown");


    // Try to do a soft exit
    // If there is a manager tell it to stop
  if (board_manager_) {
   clean_and_relaunch();
   board_manager_->free_registers();
   delete board_manager_;
   Log(verbose,"Manager destroyed");
 }
}
}
}

