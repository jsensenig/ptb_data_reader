/*
 * CommandDispatcher.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef COMMANDDISPATCHER_H_
#define COMMANDDISPATCHER_H_
class PTBManager;

class CommandDispatcher {
public:
  inline static CommandDispatcher* get();
  virtual ~CommandDispatcher();

  void RegisterDataManager(const PTBManager *dataManager) {
    fPTBManager = dataManager;
  }

  void ProcessCommand(const char* buffer);

private:
  /// Disallow default and copy constructors
  /// We want to have a single insteance in the session.
  CommandDispatcher() : fPTBManager(0) {};
  CommandDispatcher(const CommandDispatcher &other);
  void operator=(const CommandDispatcher &other);

  // Data members
  PTBManager* fPTBManager;

};

inline CommandDispatcher* CommandDispatcher::get() {
  static CommandDispatcher disp;
  return &disp;
}
#endif /* COMMANDDISPATCHER_H_ */
