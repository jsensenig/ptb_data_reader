/*
 * ConfigServer.cc
 *
 *  Created on: Jun 2, 2015
 *      Author: nbarros
 */


#include "ConfigServer.h"
#include "PTBManager.h"
// For Socket, ServerSocket, and SocketException
#include "PracticalSocket.h"
#include "Logger.h"
#include <string>

using std::string;


// -- STL headers
#include <iostream>
#include <stdexcept>
extern "C" {
#include <pthread.h>         // For POSIX threads
};

//-- Assignment of the static variables
std::string ConfigServer::buffer_ = "";
unsigned int ConfigServer::num_instances_ = 0;
unsigned int ConfigServer::port_ = 8991;
std::map<std::string, int> ConfigServer::commands_ = init_map();
PTBManager *ConfigServer::data_manager_ = NULL;
std::list<const char*> ConfigServer::queue_ = std::list<const char*>();
pthread_t ConfigServer::thread_id_ = 0;
bool ConfigServer::shutdown_= false;

ConfigServer::ConfigServer() {
  // Use the practical socket to establish a connection
  Log(verbose) << "Building object" << endlog;

  // Start a new thread here for listening

  //pthread_t threadID;  // Thread ID from pthread_create()
  // pthread_attr_t threadAttr;
  if (pthread_create(&thread_id_, NULL, &(ConfigServer::listen),(void*)NULL) != 0) {
    Log(fatal) << "Unable to create master thread." << endlog;
  }
  Log(info) << "Master server thread created. [" << thread_id_ << "]" << endlog;
  Log(info) << thread_id_->__sig << endlog;

}

void* ConfigServer::listen(void *arg) {
  // Guarantees that resources are deallocated upon return
  //pthread_detach(pthread_self());

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
      // Wait indefinitely until the client sets in.
      TCPSocket *clntSock = servSock.accept();
      Log(debug) << "Received client connection request." << endlog;
      ThreadMain(clntSock);
      // We have a model of 1 server- 1 client, so no need to spawn another thread.
      // Create a client thread
      // This is wrong... the whole server should be living on a separate thread
      //      pthread_t threadID;  // Thread ID from pthread_create()
      //      if (pthread_create(&threadID, NULL, &(ConfigServer::ThreadMain),(void *) clntSock) != 0) {
      //        Log(fatal) << "Unable to create client thread" << endlog;
      //      }
      //      Log(info) << "Client configuration thread created." << endlog;
    }
  }
  catch(SocketException &e) {
    Log(error) << "Socket exception caught." << endlog;
    Log(fatal) << e.what() << endlog;

  }
  catch(...) {
    Log(fatal) << "Failed to establish configuration socket with unknown failure." << endlog;
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
  static char tmpBuffer[RCVBUFSIZE];
  int recvMsgSize;
  // What doesthis loop mean?
  // What if the string sent is bigger than the one being read?
  while ((recvMsgSize = sock->recv(tmpBuffer, RCVBUFSIZE)) > 0) { // Zero means
    // end of transmission
    Log(debug) << "Received " << recvMsgSize << " bytes." << endlog;
    Log(verbose) << "[" << tmpBuffer << "]" << endlog;
    // Append into the permanent buffer
    buffer_ = tmpBuffer;

    // Process the string
    // Now the newly received string should be processed.
    // This method decides only if the data is configuration or a command
    ProcessTransmission(buffer_.c_str());

    // Echo message back to client
    //sock->send(echoBuffer, recvMsgSize);
  }

  //
  Log(warning) << "Connection was requested to be closed." << endlog;
  Log(warning) << "Socket will be closed" << endlog;
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
  if (data_manager_ == NULL) {
    Log(warning) << "Attempting to process a transmission without a valid data Manager. Queueing." << endlog;
    queue_.push_back(buffer);
    Log(verbose) << "Queue now composed of " << queue_.size() << " elements." << endlog;
    return;
  }


  // Instanciate the XML plugin
  pugi::xml_document doc;

  //[code_load_memory_buffer
  // You can use load_buffer to load document from immutable memory block:
  pugi::xml_parse_result result = doc.load(buffer);
  //]

  Log(debug) << "Load result : " << result.description() << endlog;
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

}

void ConfigServer::ProcessCommand(pugi::xml_node &command) {

  Log(verbose) << "Processing input buffer" << endlog;
  Log(verbose) << "Name: [" << command.name() << "]"<< endlog;
  Log(verbose) << "Value: [" << command.child_value() << "]"<< endlog;
  try{
    const char* cmd = command.child_value();
    Log(debug) << "Checking command [" << cmd << "] as part of :" << endlog;

    std::map<std::string, int>::iterator it;
    for (it = commands_.begin(); it != commands_.end(); ++it) {
      Log(verbose) << "Key : [" << it->first << "] : Value : [" << it->second << "]" << endlog;
      if (cmd == it->first) {
        Log(debug) << "It is the same" << endlog;
      }
    }

    Log(verbose) << "Going into the switch to try " <<  commands_.at(cmd) << endlog;
    Log(verbose) << "DataManager is "<< data_manager_ << endlog;
    switch (commands_.at(cmd)) {
    case RUNSTART:
      // Only starts a run if the manager is in IDLE state
      // Otherwise issue a warning
      if (data_manager_->getStatus() != PTBManager::IDLE) {
        Log(warning) << "A run is already running. Ignoring command" << endlog;
      } else {
        // Start the run
        Log(verbose) << "The Run should START now" << endlog;
        break;
      }
      // -- Send the signal to start the run. Should be a register, I guess.
      break;
    case RUNSTOP:
      if (data_manager_->getStatus() != PTBManager::RUNNING) {
        Log(warning) << "Called for RUNSTOP but there is no run ongoing. Ignoring." << endlog;
        // -- Same thing. This for sure
        break;
      } else {
        Log(verbose) << "The Run should STOP now" << endlog;

      }
    }
  }
  catch (const std::out_of_range &oor) {
    Log(error) << "Out of range error: " << oor.what() << endlog;
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


void ConfigServer::Shutdown(bool shutdown) {
  shutdown_ = shutdown;
  Log(warning) << "Shutdown asked. Forcing it into server thread." << endlog;
  if (queue_.size() == 0) {
    // Detach to deallocate resources
    pthread_detach(thread_id_);
    // Send a cancel call.
    pthread_cancel(thread_id_);
  }
}

