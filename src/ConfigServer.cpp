/*
 * ConfigServer.cc
 *
 *  Created on: Jun 2, 2015
 *      Author: nbarros
 */


#include "ConfigServer.h"
// For Socket, ServerSocket, and SocketException
#include "PracticalSocket.h"
#include "Logger.h"
#include "pugixml.hpp"


// -- STL headers
#include <iostream>

extern "C" {
#include <pthread.h>         // For POSIX threads
};

std::string ConfigServer::fBuffer = "";
unsigned int ConfigServer::fNInstances = 0;
unsigned int ConfigServer::fPort = 8991;

ConfigServer::ConfigServer() {
	// Use the practical socket to establish a connection
  try {
    TCPServerSocket servSock(fPort);   // Socket descriptor for server
    Log(info) << "Entering wait mode for client connection." << endlog;
    for (;;) {
      // Create a separate socket for the client
      // Wait indefinitely until the client sets in.
      TCPSocket *clntSock = servSock.accept();
      Log(debug) << "Received client connection request." << endlog;
      // Create a client thread
      pthread_t threadID;  // Thread ID from pthread_create()
      if (pthread_create(&threadID, NULL, &(ConfigServer::ThreadMain),
          (void *) clntSock) != 0) {
        Log(fatal) << "Unable to create client thread" << endl;
      }
    }
  }
  catch(SocketException &e) {
    Log(error) << "Socket exception caught." << endlog;
    Log(fatal) << e.what() << endlog;
  }
  catch(...) {
    Log(fatal) << "Failed to extablish configuration socket with unknown failure." << endlog;
  }
}
// TCP client handling function
void ConfigServer::HandleTCPClient(TCPSocket *sock) {
  Log(debug) << "Handling client " << endlog;
  try {
    cout << sock->getForeignAddress() << ":";
  } catch (SocketException &e) {
    Log(error) << "Unable to get foreign address" << endlog;
  }
  try {
    cout << sock->getForeignPort();
  } catch (SocketException &e) {
    Log(error) << "Unable to get foreign port" << endlog;
  }
  Log(info) << "Working  with thread " << pthread_self() << endlog;

  // Send received string and receive again until the end of transmission

  // Receive 1kB at a time.
  // Loop into the permanent buffer so that we know have all the configuration
  const int RCVBUFSIZE = 1024;
  static char tmpBuffer[RCVBUFSIZE];
  int recvMsgSize;
  // What doesthis loop mean?
  // What if the string sent is bigger than the one being read?
  while ((recvMsgSize = sock->recv(tmpBuffer, RCVBUFSIZE)) > 0) { // Zero means
                                                         // end of transmission
      Log(debug) << "Received " << recvMsgSize << " bytes." << endlog;
      Log(verbose) << "[" << tmpBuffer << "]" << endlog;
      // Append into the permanent buffer
      fBuffer+= tmpBuffer;
      // Echo message back to client
      //sock->send(echoBuffer, recvMsgSize);
  }

  //
  Log(verbose) << "Full configuration received." << endlog;
  Log(verbose) << "[" << tmpBuffer << "]" << endlog;

  // Now the newly received string should be processed.
  // This method decides only if the data is configuration or a command
  ProcessTransmission(fBuffer.c_str());

  // Destructor closes socket
}


void* ConfigServer::ThreadMain(void* clntSocket) {
  // Guarantees that thread resources are deallocated upon return
  pthread_detach(pthread_self());

  // Extract socket file descriptor from argument
  HandleTCPClient((TCPSocket *) clntSocket);

  delete (TCPSocket *) clntSocket;
  return NULL;
}

ConfigServer::~ConfigServer() {
	fNInstances--;
	if (fNInstances == 0) {
		Log(info) << "Reached last instance. Destroying myself." << endlog;
		delete this;
	}
}

void ConfigServer::CheckInstances() const {
	std::cout << "ConfigServer::CheckInstances : Have currently " << fNInstances << " in use!" << std::endl;
}

/**
 * There is a bit of a  here: on one hand we want to be able to parse the configuration
 * and on another we want to be able to pass commands to start and end the run.
 * Probably the best option is to implement a dispatcher.
 */
void ConfigServer::ProcessTransmission(const char* buffer) {
  // Instanciate the XML plugin
  pugi::xml_document doc;

  //[code_load_memory_buffer
  // You can use load_buffer to load document from immutable memory block:
  pugi::xml_parse_result result = doc.load_buffer(buffer, sizeof(buffer));
  //]

  Log(debug) << "Load result : " << result.description() << endlog;
  // Search if there is a command in here
  pugi::xml_node command = doc.child("command");
  if (command == NULL) {
    Log(info) << "There's no command node. Assuming this to be a configuration file." << endlog;
    // Cross check that there is no configuration string here eithers.
    if (doc.child("config") == NULL) {
      Log(error) << "Malformed communication. Neither config nor command." << endlog;
    } else {
      ParseConfig(buffer);
    }

  } else {
    // command exists.
    // Check if a configuration also exists

  }


}
