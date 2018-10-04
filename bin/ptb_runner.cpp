/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"

#include "../src/boardmanager.h"
#include "../src/boardreader.h"
#include "../src/boardserver.h"
#include "PracticalSocket.h"

#include <bitset>
#include <csignal>
#include <thread>
#include <chrono>

using namespace std;
using ptb::board_server;

bool g_relaunch;
bool g_shutdown;
//FIXME: Add a manager for all the threads that are running

void handler(int signal) {
	if (signal == SIGINT) {
		Log(warning,"Received a SIGINT request. Attempting a clean stop.");
		g_relaunch = false;
		board_server*cfg = board_server::get();
		cfg->shutdown(false);
		cfg->get_num_instances();
		Log(info,"Cleaning the config server itself");
		//pthread_join(cfg->getThreadId(),NULL);
		cfg = NULL;
		g_shutdown = true;
		//delete cfg;
		exit(0);
	}
}


void run() {

	board_server*cfg = board_server::get();

	//Log(debug) << "Going to sleep" <<endlog;
	//std::this_thread::sleep_for (std::chrono::seconds(10));
	Log(info,"Starting manager");

	// -- Grab the manager
	ptb::board_manager *mgr = cfg->get_board_manager();
	Log(debug,"Dumping initial config registers...");
	mgr->dump_config_registers();

	const ptb::board_reader *brd = mgr->getReader();
	Log(debug,"Board reader instance: %p",brd);

	cfg->run();
//	if (cfg->get)
//
//	if (cfg->getThreadId()) {
//		Log(info,"Waiting for thread %d",cfg->getThreadId());
//		pthread_join(cfg->getThreadId(),NULL);
//	}

}

int main() {
  // this does nothing but gets rid of a compilation warning
  (void)ptb::content::format_version;

	// Register a signal handler
	std::signal(SIGPIPE, SIG_IGN);
	std::signal(SIGINT,handler);
	g_relaunch = true;
	g_shutdown = false;
	//Logger::SetSeverity(Logger::verbose);
	Logger::SetSeverity(Logger::debug);
	//Logger::SetSeverity(Logger::info);
	// Logger::SetSeverity(Logger::warning);
	// Logger::SetSeverity(Logger::error);

	while(g_relaunch && !g_shutdown) {
		if (g_shutdown) {
			Log(info,"Shutting down the server.");
			break;
		}
		try{
			run();
		}

		catch(std::exception &e) {
			Log(error,"STD exception caught: %s",e.what());
			g_relaunch = true;
		}
		catch(...) {
			// Nope. This is a terrible idea. What you want to do is wait for a few seconds and relaunch
			//return -1;
			Log(warning,"An exception was caught.");
			Log(warning,"An exception was caught. Waiting for 10 s and relaunching.");
			std::this_thread::sleep_for (std::chrono::seconds(10));
			g_relaunch = true;
		}
	}

	return 0;
}

