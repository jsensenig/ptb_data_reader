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
#include "ConfigServer.h"
#include "PTBManager.h"
#include "Logger.h"

// -- Contrib classes
// For Socket, ServerSocket, and SocketException
#include "PracticalSocket.h"

// -- STL classes
#include <string>
#include <iostream>
#include <stdexcept>
#include <cstring>

using std::string;

//-- Assignment of the static variables
std::string ConfigServer::tcp_buffer_ = "";
//std::string ConfigServer::cfg_buffer_ = "";
unsigned int ConfigServer::num_instances_ = 0;
unsigned int ConfigServer::port_ = 8991;
//std::map<std::string, int> ConfigServer::commands_ = init_map();
PTBManager *ConfigServer::data_manager_ = NULL;
std::list<const char*> ConfigServer::queue_ = std::list<const char*>();
pthread_t ConfigServer::thread_id_ = 0;
bool ConfigServer::shutdown_= false;

ConfigServer::ConfigServer() {
  // Use the practical socket to establish a connection
  Log(verbose) << "Building object" << endlog;

  // Start a new thread to handle the TCP server socket
  // for the control channel
  if (pthread_create(&thread_id_, NULL, &(ConfigServer::listen),(void*)NULL) != 0) {
    Log(fatal) << "Unable to create master thread." << endlog;
  }
  Log(info) << "Master server thread created. [" << thread_id_ << "]" << endlog;
  Log(verbose) << thread_id_->__sig << endlog;

}

void* ConfigServer::listen(void *arg) {
  try {
    Log(info) << "Creating server listening socket." << endlog;
    TCPServerSocket servSock(port_);   // Socket descriptor for server
    Log(info) << "Entering wait mode for client connection." << endlog;
    for (;;) {
      // Check if there is a request to terminate the connection.
      if (shutdown_) {
        Log(info) << "Received a request to shut down. Complying..." << endlog;
        return NULL;
      }

      Log(verbose) << "Starting listener for connection." << endlog;
      // Create a separate socket for the client
      // Wait indefinitely until the client kicks in.
      TCPSocket *clntSock = servSock.accept();
      Log(verbose) << "Received client connection request." << endlog;
      ThreadMain(clntSock);
    }
  }
  catch(SocketException &e) {
    Log(error) << "Socket exception caught." << endlog;
    Log(error) << e.what() << endlog;
    ::abort();

  }
  catch(std::exception &e) {
    Log(error) << "Caught standard exception : " << e.what() << endlog;
  }
  catch(...) {
    Log(error) << "Failed to establish configuration socket with unknown failure." << endlog;
  }
  Log(warning) << "Reaching the end, but not sure why..." << endlog;
  return NULL;
}


// TCP client handling function
void ConfigServer::HandleTCPClient(TCPSocket *sock) {
  Log(debug) << "Handling client "  << endlog;
  try {
    Log(debug) << "Address : " << sock->getForeignAddress() << endlog;
  } catch (SocketException &e) {
    Log(error) << "Unable to get foreign address" << endlog;
  }
  try {
    Log(debug) << "Port : " << sock->getForeignPort() << endlog;;
  } catch (SocketException &e) {
    Log(error) << "Unable to get foreign port" << endlog;
  }
  Log(info) << "Working  with thread " << pthread_self() << endlog;

  // Send received string and receive again until the end of transmission

  // Receive 1kB at a time.
  // Loop into the permanent buffer so that we know have all the configuration
  const int RCVBUFSIZE = 20480; //20 kB. Really hope will not overdo this.

  static char instBuffer[RCVBUFSIZE];
  int recvMsgSize;

  // All the transmissions are done in two parts. At the top comes a token
  // with the size of the incoming transmission
  // Then the packet itself follows.
  // Keep accepting until the </system> token comes in.
  // THis signals the end of the message

  std::string localBuffer = "";
  // What if the string sent is bigger than the one being read?
  while ((recvMsgSize = sock->recv(instBuffer, RCVBUFSIZE)) > 0) { // Zero means

    // Truncate the instantaneous buffer on the number of bytes
    instBuffer[recvMsgSize+1] = '\0';
    // end of transmission
    Log(debug) << "Received " << recvMsgSize << " bytes." << endlog;
    Log(verbose) << "[" << instBuffer << "]" << endlog;
    // Append into the permanent buffer
    //cfg_buffer_ = tmpBuffer;

    // Keep accumulating the transmission until one of the closing
    // tokens are passed (</config></command>)

    localBuffer += instBuffer;
    std::size_t pos = 0;
    if ((pos = localBuffer.find("</config>")) != std::string::npos) {
      // Found the end of a config block.
      // Pass that buffer to process and erase it from the string
      // 9 = strlen("</config>")
      Log(verbose) << "Found a config block." << endlog;
      try{
        tcp_buffer_ = localBuffer.substr(0,pos+strlen("</config>"));
        // Remove the entry from localBuffer
        localBuffer = localBuffer.substr(pos+strlen("</config>")+1);
      }
      catch(std::length_error &e) {
        Log(error) << "Length error : " << e.what() << endlog;
      }
      catch(std::out_of_range &e) {
        Log(error) << "Out of range : " << e.what() << endlog;
      }
      catch(std::bad_alloc &e) {
        Log(error) << "Bad alloc " << e.what() << endlog;
      }
      catch(std::invalid_argument &e) {
        Log(error) << "Invalid argument error :" << e.what() << endlog;
      }
      catch(std::domain_error &e) {
        Log(error) << "Domain error :" << e.what() << endlog;
      }
      catch(std::overflow_error &e) {
        Log(error) << "Overflow error :" << e.what() << endlog;
      }
      catch(std::range_error &e) {
        Log(error) << "Range error :" << e.what() << endlog;
      }

      // Generic
      catch(std::exception &e) {
        Log(error) << "Caught std exception : " << e.what() << endlog;
      }
      ProcessTransmission(tcp_buffer_.c_str());
    } else if ((pos = localBuffer.find("</command>")) != std::string::npos) {
      Log(verbose) << "Found a command block." << endlog;
      // Found the end of a config block.
      // Pass that buffer to process and erase it from the string
      // 9 = strlen("</config>")
      tcp_buffer_ = localBuffer.substr(0,pos+strlen("</command>"));
      // Remove the entry from localBuffer
      localBuffer = localBuffer.substr(pos+strlen("</command>")+1);
      ProcessTransmission(tcp_buffer_.c_str());
    }

    // Process the string
    // Now the newly received string should be processed.
    // This method decides only if the data is configuration or a command
    //ProcessTransmission(tmpBuffer);

    // Echo message back to client
    //sock->send(echoBuffer, recvMsgSize);
  }

  //
  Log(warning) << "Connection to the client was lost." << endlog;
  Log(warning) << "Socket will be closed." << endlog;
  // Destructor closes socket
}


void* ConfigServer::ThreadMain(void* clntSocket) {
  // Guarantees that thread resources are deallocated upon return
  //pthread_detach(pthread_self());

  // Extract socket file descriptor from argument
  HandleTCPClient((TCPSocket *) clntSocket);

  delete (TCPSocket *) clntSocket;
  return NULL;
}

ConfigServer::~ConfigServer() {
  num_instances_--;
  if (num_instances_ == 0) {
    Log(info) << "Reached last instance. Destroying myself." << endlog;
  }
}

void ConfigServer::CheckInstances() const {
  std::cout << "ConfigServer::CheckInstances : Have currently " << num_instances_ << " in use!" << std::endl;
}

/**
 * There is a bit of a conundrum here: on one hand we want to be able to parse the configuration
 * and on another we want to be able to pass commands to start and end the run.
 * Implement a register handle, that can be used to trigger the manager
 */
void ConfigServer::ProcessTransmission(const char* buffer) {
  // Before doing any processing check that a DataManager has already
  // been registered. If not simply queue the command into a list and wait
  // Careful if the client connection in the meantime is lost, the memory will disappear.
  // Have to copy the string into the queue
  if (data_manager_ == NULL) {
    Log(warning) << "Attempting to process a transmission without a valid data Manager. Queueing." << endlog;
    Log(verbose) << "[" << buffer << "]" << endlog;

    queue_.push_back(strdup(buffer));
    Log(verbose) << "Queue now composed of " << queue_.size() << " elements." << endlog;
    return;
  }
  Log(verbose) << "Processing a transmission" << endlog;
  Log(verbose) << "[" << buffer << "]" << endlog;

  // Instanciate the XML plugin
  pugi::xml_document doc;

  //[code_load_memory_buffer
  // You can use load_buffer to load document from immutable memory block:
  pugi::xml_parse_result result = doc.load(buffer);
  //]

  Log(verbose) << "Load result : " << result.description() << endlog;
  // Search if there is a command in here
  pugi::xml_node command = doc.child("command");
  pugi::xml_node config = doc.child("config");
  // First look for 2 ridiculous situations:
  // 1.) There is neither config nor command nodes
  // 2.) There are both config and command nodes
  if (command == NULL &&  config == NULL) {
    Log(error) << "Neither config nor command blocks were found. Ignoring the data." << endlog;
  } else if (command != NULL and config != NULL){
    Log(error) << "Found both a config and a command node. This is not supported yet." << endlog;
  } else if (config != NULL) {
    Log(verbose) << "Processing config " << config.name() << " : " << config.child_value() << endlog;
    ProcessConfig(config);
  } else if (command != NULL){
    Log(verbose) << "Processing command [" << command.name() << "] : [" << command.child_value() << "]" << endlog;
    ProcessCommand(command);
  } else {
    Log(warning) << "Reached an impossible situation. Pretending nothing happened and carrying on." << endlog;
  }

}


void ConfigServer::ProcessConfig(pugi::xml_node &config) {
  Log(verbose) << "Processing input buffer" << endlog;
  Log(verbose) << "Name: [" << config.name() << "]"<< endlog;
  Log(verbose) << "Value: [" << config.child_value() << "]"<< endlog;

  // Store the buffer in the local variable.
  if (Logger::GetSeverity() <= Logger::debug) {
    config.print(std::cout,"",pugi::format_raw);
  }
  data_manager_->ProcessConfig(config);
  //FIXME: Add call to process the configuration in the manager.
}

void ConfigServer::ProcessCommand(pugi::xml_node &command) {

  Log(verbose) << "Processing input buffer" << endlog;
  Log(verbose) << "Name: [" << command.name() << "]"<< endlog;
  Log(verbose) << "Value: [" << command.child_value() << "]"<< endlog;
  try{
    const char* cmd = command.child_value();
    Log(debug) << "Checking command [" << cmd << "]." << endlog;

    // Pass the command to the manager and let it handle it properly
    data_manager_->ExecuteCommand(cmd);

  }
  catch (const std::exception &e) {
    Log(error) << "STL exception caught : " << e.what() << endlog;
  }
  catch(...) {
    Log(error) << "Unknown exception caught." << endlog;
  }
  // Get the command

}

void ConfigServer::RegisterDataManager(PTBManager *manager)  {
  data_manager_ = manager;
  if (queue_.size() > 0) {
    Log(debug) << "Processing outstanding transmission queue." << endlog;
    while (queue_.size() > 0) {
      Log(verbose) << "Processing an entry" << endlog;
      ProcessTransmission(queue_.front());
      queue_.pop_front();
    }
  }
}


void ConfigServer::Shutdown(bool force) {
  shutdown_ = true;
  Log(warning) << "Shutdown requested." << endlog;
  // Check if force is passed
  if (force || queue_.size() == 0) {
    if (force)
      Log(warning) << "Forcing it into server thread." << endlog;
    // Detach to deallocate resources
    pthread_detach(thread_id_);
    // Send a cancel call.
    pthread_cancel(thread_id_);
  } else if (queue_.size() != 0){
    // Soft shutdown requested. Do nothing and wait for the client to disconnect
    Log(warning) << "Soft shutdown requested with non-empty queue. Shutdown will occur when queue empties." << endlog;
    // Kill the server.
    // Detach to deallocate resources
    pthread_detach(thread_id_);
    // Send a cancel call.
    pthread_cancel(thread_id_);
    // Process the queue
    while (queue_.size() > 0) {
      Log(verbose) << "Processing an entry" << endlog;
      ProcessTransmission(queue_.front());
      queue_.pop_front();
    }
  }
}

