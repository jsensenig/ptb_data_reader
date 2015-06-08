/**
 * \class ConfigServer
 * \brief Implements a configuration manager using XML to parse the options.
 * ConfigServer.hh
 *
 *  Created on: Jun 2, 2015
 *      Author: Nuno Barros
 *  Description
 */

#ifndef CONFIGSERVER_H_
#define CONFIGSERVER_H_

#include <string>

class TCPSocket;
class CommandDispatcher;

class ConfigServer {
public:
	inline static ConfigServer* get();
	virtual ~ConfigServer();

	void CheckInstances() const;

	// Main client thread.
	static void HandleTCPClient(TCPSocket *sock);
	static void *ThreadMain(void *clntSock);

	static void ParseConfig(const char* buffer);
	static void ProcessTransmission(const char* buffer);

	/**
	 * Set the command dispatcher for this session.
	 * There should only be one in the whole session.
	 *
	 * @param dispatcher pointer to the CommandDispatcher.
	 */
	void setDispatcher(CommandDispatcher *dispatcher) {fDispatcher = dispatcher;};

private:
	// Disable constructors and assignment
	ConfigServer();
	ConfigServer(const ConfigServer &other);
	void operator=(const ConfigServer &other);

	// parameters
	static unsigned int fNInstances;
	static unsigned int fPort;
	static std::string fBuffer;
	CommandDispatcher* fDispatcher;

};

inline ConfigServer* ConfigServer::get() {
	static ConfigServer conf;
	fNInstances++;
	return &conf;
}

#endif /* CONFIGSERVER_H_ */
