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


using std::string;
namespace ptb {
//-- Assignment of the static variables
std::string board_server::tcp_buffer_ = "";
unsigned int board_server::num_instances_ = 0;
unsigned int board_server::port_ = 8991;
board_manager *board_server::board_manager_ = NULL;
static std::queue<std::string> queue_;
bool board_server::shutdown_requested_= false;
TCPSocket* board_server::client_socket_ = nullptr;
std::string board_server::msg_answer_ = "";

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
		Log(warning,"Passing down a shutdown signal to the PTB.");
	  board_manager_->exec_command("StopRun",msg_answer_);
      // -- Force a hard reset in the PTB to account for the case the
      // DAQ crashed.
	  board_manager_->exec_command("HardReset",msg_answer_);
	}
	Log(warning,"Relaunching the socket for connection acceptance in 5s.");
	std::this_thread::sleep_for(std::chrono::seconds(5));
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
	      Log(warning,"Relaunching the socket for connection acceptance in 5s.");
	      std::this_thread::sleep_for(std::chrono::seconds(5));
	    }
	  catch(...) {
		  Log(error,"Unknown exception caught");
	      if (servSock != nullptr) delete servSock;
	      servSock = nullptr;
	      Log(warning,"Relaunching the socket for connection acceptance in 5s.");
	      std::this_thread::sleep_for(std::chrono::seconds(5));
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
	    Log(warning,"Relaunching the socket for connection acceptance in 5s.");
	    std::this_thread::sleep_for(std::chrono::seconds(5));
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
    localBuffer += instBuffer;
    Log(verbose,"Duplicated [%s] \n Length [%u]",localBuffer.c_str(),localBuffer.length());
    std::size_t pos = 0;
    // Check if it is a compelte configuration set
    // Loop until all configurations and commands are processed
    while (localBuffer.find("</config>")!= std::string::npos  || localBuffer.find("</command>")!= std::string::npos) {
      Log(verbose,"There is something to be processed");
      if ((pos = localBuffer.find("</config>")) != std::string::npos && (localBuffer.find("<config>")!= std::string::npos)) {
        // Found the end of a config block.
        // Pass that buffer to process and erase it from the string
        // 9 = strlen("</config>")
        Log(verbose,"Found a config block.");
        try{
          tcp_buffer_ = localBuffer.substr(0,pos+strlen("</config>"));
          // Remove the entry from localBuffer
          Log(verbose,"Shifting buffer");
          localBuffer = localBuffer.substr(pos+strlen("</config>"));
          Log(verbose,"new buffer [%s]",localBuffer.c_str());
        }
        catch(std::length_error &e) {
          Log(error,"Length error : %s ",e.what());
        }
        catch(std::out_of_range &e) {
          Log(error,"Out of range : %s",e.what());
        }
        catch(std::bad_alloc &e) {
          Log(error,"Bad alloc %s",e.what());
        }
        catch(std::invalid_argument &e) {
          Log(error,"Invalid argument error : %s",e.what());
        }
        catch(std::domain_error &e) {
          Log(error,"Domain error : %s",e.what());
        }
        catch(std::overflow_error &e) {
          Log(error,"Overflow error : %s",e.what());
        }
        catch(std::range_error &e) {
          Log(error,"Range error : %s",e.what());
        }
        // Generic
        catch(std::exception &e) {
          Log(error,"Caught std exception : %s",e.what());
        }

        // Process the transmission.
        // These exceptions should never be caught. They should be caught underneath
        try {
          Log(verbose,"Processing buffer [%s]",tcp_buffer_.c_str());
          // Don't understand what this sleep is doing here
          //std::this_thread::sleep_for (std::chrono::seconds(2));
          process_request(tcp_buffer_.c_str(),msg_answer_);
          sprintf(instBuffer,"<feedback>%s</feedback>",msg_answer_.c_str());
        }
        catch (std::string &e) {
          Log(error,"Config exception caught : %s",e.c_str());
          // Return a failure signal.
          std::ostringstream msg;
          msg << "<error>" << e.c_str() << "</error>";
          msg_answer_ += msg.str();
        }
        catch (std::exception &e) {
          Log(error,"STD exception caught : %s",e.what());
          // Return a failure signal.
//          sprintf(instBuffer,"<feedback><error>%s</error></feedback>",e.what());
          std::ostringstream msg;
          msg << "<error>" << e.what() << "</error>";
          msg_answer_ += msg.str();
        }
        catch (...) {
        	Log(error,"Unknown exception caught");
            msg_answer_ += "<error>Unknown failure</error>";
        }
        Log(verbose,"Returning answer : %s",msg_answer_.c_str());
        client_socket_->send(msg_answer_.c_str(),msg_answer_.size());
      } else if ((pos = localBuffer.find("</command>")) != std::string::npos) {
        Log(verbose,"Found a command block.");
        Log(verbose,"|%s|",localBuffer.c_str());
        // Found the end of a command block.
        // Pass that buffer to process and erase it from the string
        // 9 = strlen("</config>")

        tcp_buffer_ = localBuffer.substr(0,pos+strlen("</command>"));
        // Remove the entry from localBuffer
        try{
          Log(verbose,"Shifting buffer");
          localBuffer = localBuffer.substr(pos+strlen("</command>"));
          Log(verbose,"new buffer [%s]",localBuffer.c_str());

          Log(verbose,"Processing command buffer [%s]",tcp_buffer_.c_str() );
          process_request(tcp_buffer_,msg_answer_);
        }
        catch (std::string &e) {
          Log(error,"Config exception caught : %s",e.c_str());
          // Return a failure signal.
          std::ostringstream msg;
          msg << "<error>" << e.c_str() << "</error>";
          msg_answer_ += msg.str();
        }
        catch (ptb::op_exception &e) {
          Log(error,"Operational exception caught : %s",e.what());
          std::ostringstream msg;
          msg << "<error>" << e.what() << "</error>";
          msg_answer_ += msg.str();
        }
        catch (std::runtime_error &e) {
          Log(error,"STD runtime_error exception caught : %s", e.what());
          std::ostringstream msg;
          msg << "<error>" << e.what() << "</error>";
          msg_answer_ += msg.str();
        }
        catch (std::exception &e) {
          Log(error,"STD exception caught : %s", e.what());
          // Return a failure signal.
          std::ostringstream msg;
          msg << "<error>" << e.what() << "</error>";
          msg_answer_ += msg.str();

        }
        catch (...) {
        	Log(error,"Unknown error caught");
            msg_answer_ += "<error>Unidentified error</error>";
        }
        Log(info,"Sending answer : ");
        Log(info,"%s",msg_answer_.c_str());
        client_socket_->send(msg_answer_.c_str(),msg_answer_.size());
        Log(debug,"Answer sent");
      } else {
    	  // -- This should be an error
    	  Log(error,"Mangled request");
    	  Log(error,"Found the end of a block but not its beginning...");
    	  size_t pos_cmd_end = localBuffer.find("</command>");
    	  size_t pos_cmd_start = localBuffer.find("<command>");
    	  size_t pos_conf_end = localBuffer.find("</config>");
    	  size_t pos_conf_start = localBuffer.find("<config>");
    	  if (pos_cmd_end != std::string::npos) {
    		  Log(warning,"Positions of command block : start %u end %u",pos_cmd_start,pos_cmd_end);
    	  }
    	  if (pos_conf_end != std::string::npos) {
    		  Log(warning,"Positions of config block : start %u end %u",pos_conf_start,pos_conf_end);
    	  }
    	  msg_answer_ += "<error>Mangled socket request. Check board logs.</error>";
    	  client_socket_->send(msg_answer_.c_str(),msg_answer_.size());
      }
    } // -- while
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
void board_server::process_request(const std::string &buffer,std::string &answer) {
  // Before doing any processing check that a DataManager has already
  // been registered. If not simply queue the command into a list and wait
  // Careful if the client connection in the meantime is lost, the memory will disappear.
  // Have to copy the string into the queue

  if (board_manager_ == NULL) {
    answer += "<warning>Request received without valid board manager</warning>";
    Log(warning,"Attempting to process a transmission without a valid data Manager. Queueing.");
    Log(verbose,"[%s]",buffer.c_str());

    queue_.push(buffer);
    //    queue_.push_back(buffer);
    Log(debug,"Queue now composed of %u requests.",queue_.size());
    return;
  }
  Log(verbose,"Processing a transmission");
  Log(verbose,"[%s]",buffer.c_str());

  // Instanciate the XML plugin
  pugi::xml_document doc;

  //[code_load_memory_buffer
  // You can use load_buffer to load document from immutable memory block:
  pugi::xml_parse_result result = doc.load(buffer.c_str());
  //]

  Log(verbose,"Load result : %s",result.description());
  // Search if there is a command in here
  pugi::xml_node command = doc.child("command");
  pugi::xml_node config = doc.child("config");
  // First look for 2 ridiculous situations:
  // 1.) There is neither config nor command nodes
  // 2.) There are both config and command nodes
  if (command == NULL &&  config == NULL) {
    Log(error,"Neither config nor command blocks were found. Ignoring the data.");
    throw std::runtime_error("No valid command blocks found.");
    //answer += "<error>Unknown command block</error>";
    //    std::ostringstream docstr;
    //    doc.print(docstr);
    //    throw PTBexception(docstr.str().c_str());

  } else if (command != NULL and config != NULL){
    Log(error,"Found both a config and a command node. This is not supported yet.");
    throw std::runtime_error("Found both config and command blocks.");

  } else if (config != NULL) {
    Log(verbose,"Processing config %s : %s ",config.name(),config.child_value());
    Log(verbose,"Name: [%s]",config.name());
    Log(verbose,"Value: [%s]",config.child_value());
#ifdef DEBUG
    // Store the buffer in a local variable.
    if (Logger::GetSeverity() <= Logger::verbose) {
      config.print(std::cout,"",pugi::format_raw);
    }
#endif
    // NOTE: No exception should be caught at this point. All were caught at a lower level.
    // Let's treat the writing of the answers directly in the low level code.
    // easier to evaluate on a run by run basis.
    board_manager_->process_config(config,answer);

    Log(verbose,"Config processed with answer [%s]",answer.c_str());
  } else if (command != NULL){
    Log(verbose,"Processing command [%s] : [%s]",command.name(),command.child_value());

    board_manager_->exec_command(command.child_value(),answer);
    Log(debug,"Returning from ProcessCommand with answer [%s].",answer.c_str());

  } else {
    Log(warning,"Reached an impossible situation. Pretending nothing happened and carrying on.");
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
    // Kill the server.
    // Detach to deallocate resources
    //    pthread_detach(thread_id_);
    //    // Send a cancel call.
    //    pthread_cancel(thread_id_);
    // Process the queue
    while (queue_.size() > 0) {
      Log(verbose,"Processing an entry");
      process_request(queue_.front(), msg_answer_);
      Log(verbose,"Transmission processed. Answer: %s",msg_answer_.c_str());
      queue_.pop();
    }
  } else {
    Log(info,"Performing a soft shutdown");
    // First kill the thread that is waiting for a connection
    //pthread_detach(thread_id_);
    // Send a cancel call.
    //    if (thread_id_) {
    //      pthread_cancel(thread_id_);
    //    }
      //pthread_kill(thread_id_,SIGINT);

    //thread_id_ = 0;

    // Try to do a soft exit
    // If there is a manager tell it to stop
    if (board_manager_) {
    	clean_and_relaunch();
      board_manager_->free_registers();
      delete board_manager_;
//    	// Get the reader
//      PTBReader *reader = const_cast<PTBReader*>(board_manager_->getReader());
//      reader->ClearThreads();
//      delete reader;
//      board_manager_->free_registers();
//      board_manager_->ClearCommands();
//      Log(verbose,"Destroying the manager");
      //delete data_manager_;
      Log(verbose,"Manager destroyed");
    }
  }
}
}

