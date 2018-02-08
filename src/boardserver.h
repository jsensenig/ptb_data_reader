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

#include <queue>

class TCPSocket;

namespace ptb {
class board_manager;


class board_server {
public:

	inline static board_server* get();
	virtual ~board_server();

	void get_num_instances() const;

	static void run();

	// Main client thread.
	static void handle_tcp_client(/*TCPSocket *sock*/);


	//static void *ThreadMain();
	//	static void *listen(void *arg);
	// Why would we want to register a board manager outside this?
	// If anything one could grab the manager to manipulate,
	// but I see no reason to create it outside and then register
	void register_board_manager(board_manager*manager);
	board_manager* get_board_manager() {return board_manager_;};

	//  pthread_t getThreadId() const {
	//    return thread_id_;
	//  }

	void shutdown(bool force = false);
protected:
	static void process_request(const std::string &buffer,std::vector<std::string> & answer);
	//static void process_config(pugi::xml_node &config,char*& answer);
	//  static void ProcessCommand(pugi::xml_node &command,char*& answer);
	static void clean_and_relaunch();

	static board_manager *board_manager_;
private:
	// Disable constructors and assignment
	board_server();
	board_server(const board_server &other);
	void operator=(const board_server &other);



	// Self thread of the server.
	static TCPSocket *client_socket_;
	//static std::string msg_answer_;
	std::vector<std::string> msg_answers_;
	//static pthread_t thread_id_;
	//	static std::thread::id thread_id_;
	// parameters
	static unsigned int num_instances_;
	static unsigned int port_;
	static bool shutdown_requested_; ///< Handler for destroying object.
	static std::string tcp_buffer_;

	//static std::list<const char*> queue_;
	static std::queue<std::string> queue_;


};

inline board_server* board_server::get() {
	static board_server conf;
	num_instances_++;
	return &conf;
}
}
#endif /* CONFIGSERVER_H_ */
