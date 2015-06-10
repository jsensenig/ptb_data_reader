/*
 * PTBReader.h
 *
 *  Created on: Jun 8, 2015
 *      Author: nbarros
 */

#ifndef PTBREADER_H_
#define PTBREADER_H_

class ConfigServer;

class PTBReader {
public:
  PTBReader();
  virtual ~PTBReader();


private:

  ConfigServer *cfg_srv_;

};

#endif /* PTBREADER_H_ */
