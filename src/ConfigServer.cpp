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
#include "PTBReader.h"
#include "Logger.h"

// -- Contrib classes
// For Socket, ServerSocket, and SocketException
#include "PracticalSocket.h"

// -- STL classes
#include <string>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <thread>

extern "C" {
#include <stdio.h>
#include <signal.h>
};
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
  Log(verbose,"Building object");

  // Start a new thread to handle the TCP server socket
  // for the control channel
  if (pthread_create(&thread_id_, NULL, &(ConfigServer::listen),(void*)NULL) != 0) {
    Log(error,"Unable to create master thread.");
  }
  Log(info,"Master server thread created. [%p]",thread_id_);

}

void* ConfigServer::listen(void *arg) {
  //printf("In listen\n");
  std::thread::id thread_id = std::this_thread::get_id();
  std::ostringstream stream;
  stream << std::hex << thread_id << " " << std::dec << thread_id;
  Log(info,"#### PTB Listen thread: %s",stream.str().c_str());

  try {
    Log(info,"Creating server listening socket.");
    TCPServerSocket servSock(port_);   // Socket descriptor for server
    Log(info,"Entering wait mode for client connection at port %u",port_);
    for (;;) {
      // Check if there is a request to terminate the connection.
      if (shutdown_) {
        Log(info,"Received a request to shut down. Complying...");
        return NULL;
      }

      Log(verbose,"Starting listener for connection.");
      // Create a separate socket for the client
      // Wait indefinitely until the client kicks in.
      TCPSocket *clntSock = servSock.accept();
      Log(verbose,"Received client connection request.");
      ThreadMain(clntSock);
    }
  }
  catch(SocketException &e) {
    Log(error,"Socket exception caught : %s",e.what());
    std::abort();

  }
  catch(std::exception &e) {
    Log(error,"Caught standard exception : %s",e.what());
  }
  /**
  catch(...) {
    Log(error,"Failed to establish configuration socket with unknown failure.");
  }
  */
  Log(warning,"Reaching the end, but not sure why...");
  return NULL;
}


// TCP client handling function
void ConfigServer::HandleTCPClient(TCPSocket *sock) {
  Log(debug,"Handling client ");
  try {

    Log(debug,"Address : %s",sock->getForeignAddress().c_str());;
  } catch (SocketException &e) {
    Log(error,"Unable to get foreign address");
  }
  try {
    Log(debug,"Port : %hu",sock->getForeignPort());
  } catch (SocketException &e) {
    Log(error,"Unable to get foreign port");
  }
  Log(info,"Working  with thread %x",pthread_self());

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

  std::string localBuffer;
  localBuffer.resize(RCVBUFSIZE);
  localBuffer = "";
  // What if the string sent is bigger than the one being read?
  while ((recvMsgSize = sock->recv(instBuffer, RCVBUFSIZE-2)) > 0) { // Zero means

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
    printf("!!!! Initializing pos!\n");
    std::size_t pos = 0;
    // CHeck if it is a compelte configuration set
    // Loop until all configurations and commands are processed
    printf("!!!! Looking for find!\n");
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
        try {
          Log(verbose,"Processing buffer [%s]",tcp_buffer_.c_str());
          printf("About to process\n");
	  std::this_thread::sleep_for (std::chrono::seconds(2));
	  ProcessTransmission(tcp_buffer_.c_str());
          sprintf(instBuffer,"<success>true</success>");
        }
        catch (std::string &e) {
          Log(error,"Config exception caught : %s",e.c_str());
          // Return a failure signal.
          sprintf(instBuffer,"<error>%s</error>",e.c_str());
        }
        catch (std::exception &e) {
          Log(error,"STD exception caught : %s",e.what());
          // Return a failure signal.
          sprintf(instBuffer,"<error>%s</error>",e.what());
        }
        catch (...) {
          sprintf(instBuffer,"<error>Unknown</error>");
        }
        sock->send(instBuffer,strlen(instBuffer));

      } else if ((pos = localBuffer.find("</command>")) != std::string::npos) {
        Log(verbose,"Found a command block.");
        Log(verbose,"|%s|",localBuffer.c_str());
        // Found the end of a command block.
        // Pass that buffer to process and erase it from the string
        // 9 = strlen("</config>")
        Log(debug,"POS %u LEN %u",pos,strlen("</command>"));

        tcp_buffer_ = localBuffer.substr(0,pos+strlen("</command>"));
        // Remove the entry from localBuffer
        try{
          Log(verbose,"Shifting buffer");
          localBuffer = localBuffer.substr(pos+strlen("</command>"));
          Log(verbose,"new buffer [%s]",localBuffer.c_str());

          Log(verbose,"Processing commadn buffer [%s]",tcp_buffer_.c_str() );
          ProcessTransmission(tcp_buffer_.c_str());
          sprintf(instBuffer,"<success>true</success>");
        }
        catch (std::string &e) {
          Log(error,"Config exception caught : %s",e.c_str());
          // Return a failure signal.
          sprintf(instBuffer,"<error>%s</error>",e.c_str());
        }
        catch (std::exception &e) {
          Log(error,"STD exception caught :", e.what());
          // Return a failure signal.
          sprintf(instBuffer,"<error>%s</error>",e.what());
        }
        catch (...) {
          sprintf(instBuffer,"<error>Unknown error</error>");
        }
        Log(debug,"Sending answer");
        Log(debug,"%s",instBuffer);
        sock->send(instBuffer,strlen(instBuffer));
        Log(debug,"Answer sent");
      } else {
        Log(verbose,"Found nothing");
      }
    }

    // Process the string
    // Now the newly received string should be processed.
    // This method decides only if the data is configuration or a command
    //ProcessTransmission(tmpBuffer);

    // Echo message back to client
    //sock->send(echoBuffer, recvMsgSize);
  }

  //
  Log(warning,"Connection to the client was lost.");
  Log(warning,"Socket will be closed.");
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
  if (num_instances_ <= 0) {
    Log(info,"Reached last instance. Destroying myself.");
  }
}

void ConfigServer::CheckInstances() const {
  Log(verbose,"Have currently %u in use!",num_instances_ );
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
  printf("In ProcessTransmission\n");
  if (data_manager_ == NULL) {
    Log(warning,"Attempting to process a transmission without a valid data Manager. Queueing.");
    Log(verbose,"[%s]",buffer);

    queue_.push_back(strdup(buffer));
    Log(verbose,"Queue now composed of %u elements.",queue_.size());
    return;
  }
  printf("Going to start processing\n");
  printf("The string [%s]\n",buffer);
  Log(verbose,"Processing a transmission");
  Log(verbose,"[%s]",buffer);

  // Instanciate the XML plugin
  pugi::xml_document doc;

  //[code_load_memory_buffer
  // You can use load_buffer to load document from immutable memory block:
  pugi::xml_parse_result result = doc.load(buffer);
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
  } else if (command != NULL and config != NULL){
    Log(error,"Found both a config and a command node. This is not supported yet.");
  } else if (config != NULL) {
    Log(verbose,"Processing config %s : %s ",config.name(),config.child_value());
    ProcessConfig(config);
  } else if (command != NULL){
    Log(verbose,"Processing command [%s] : [%s]",command.name(),command.child_value());
    ProcessCommand(command);
  } else {
    Log(warning,"Reached an impossible situation. Pretending nothing happened and carrying on.");
  }

}


void ConfigServer::ProcessConfig(pugi::xml_node &config) {
  Log(verbose,"Processing input buffer");
  Log(verbose,"Name: [%s]",config.name());
  Log(verbose,"Value: [%s]",config.child_value());

  // Store the buffer in the local variable.
  if (Logger::GetSeverity() <= Logger::debug) {
    config.print(std::cout,"",pugi::format_raw);
  }
  try {
    data_manager_->ProcessConfig(config);
  }
  catch(std::string &e) {
    Log(error,"Configuration exception caught: %s",e.c_str());

  }

  //FIXME: Add call to process the configuration in the manager.
}

void ConfigServer::ProcessCommand(pugi::xml_node &command) {

  Log(verbose,"Processing input buffer");
  Log(verbose,"Name: [%s]",command.name());
  Log(verbose,"Value: [%s]",command.value());

  Log(verbose,"Value: [%s]",command.child_value());
  try{
    const char* cmd = command.child_value();
    Log(debug,"Checking command [%s]",cmd);

    // Pass the command to the manager and let it handle it properly
    data_manager_->ExecuteCommand(cmd);
  }
  catch (const std::exception &e) {
    Log(error,"STL exception caught : %s ",e.what());
    throw;
  }
  catch (std::string &str) {
    Log(error,"Caught STR exception %s ",str.c_str());
    throw;
  }
  catch(...) {
    Log(error,"Unknown exception caught.");
    throw;
  }
  // Get the command answer

}

void ConfigServer::RegisterDataManager(PTBManager *manager)  {
  data_manager_ = manager;
  if (queue_.size() > 0) {
    Log(debug,"Processing outstanding transmission queue.");
    while (queue_.size() > 0) {
      Log(verbose,"Processing an entry");
      ProcessTransmission(queue_.front());
      queue_.pop_front();
    }
  }
}


void ConfigServer::Shutdown(bool force) {
  shutdown_ = true;
  Log(warning,"Shutdown requested.");
  // Check if force is passed
  if (force) {
	Log(warning,"Forcing it into server thread.");
    // Detach to deallocate resources
    pthread_detach(thread_id_);
    // Send a cancel call.
    pthread_cancel(thread_id_);
  } else if (queue_.size() != 0){
    // Soft shutdown requested. Do nothing and wait for the client to disconnect
    Log(warning,"Soft shutdown requested with non-empty queue. Shutdown will occur when queue empties.");
    // Kill the server.
    // Detach to deallocate resources
    pthread_detach(thread_id_);
    // Send a cancel call.
    pthread_cancel(thread_id_);
    // Process the queue
    while (queue_.size() > 0) {
      Log(verbose,"Processing an entry");
      ProcessTransmission(queue_.front());
      queue_.pop_front();
    }
  } else {
	  Log(debug,"Performing a soft shutdown");
	  // First kill the thread that is waiting for a connection
	    //pthread_detach(thread_id_);
	    // Send a cancel call.
	    pthread_cancel(thread_id_);
	  	//pthread_kill(thread_id_,SIGINT);

	    thread_id_ = 0;

	  // Try to do a soft exit
	  // If there is a manager tell it to stop
	  if (data_manager_) {
		  // Get the reader
		  PTBReader *reader = const_cast<PTBReader*>(data_manager_->getReader());
		  reader->ClearThreads();
		  delete reader;
		  data_manager_->FreeRegisters();
		  data_manager_->ClearCommands();
		  Log(verbose,"Destroying the manager");
		  //delete data_manager_;
		  Log(verbose,"Manager destroyed");
	  }
  }
}

