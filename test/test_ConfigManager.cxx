/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include "Logger.h"
#include "ConfigServer.h"

int main() {
  Logger::SetSeverity(Logger::verbose);
  Log(info) << "Just a test" << endlog;
  ConfigServer*cfg = ConfigServer::get();
  return 0;
}

