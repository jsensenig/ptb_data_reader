/*
 * board_reader_interface.h
 *
 *  Created on: Aug 5, 2020
 *      Author: jon
 *
 *      Interface to the boarder reader software.
 *      This creates a simple interface so the
 *      data can be accessed in a
 *      convenient way without needing detailed
 *      knowledge of the board reader.
 */

#ifndef SRC_BOARD_READER_INTERFACE_H_
#define SRC_BOARD_READER_INTERFACE_H_

#include <cstdint>
#include "json.hpp"

namespace ptb {
  class board_reader;  // forward-declare board_reader dependency.
}

using json = nlohmann::json;

class DataReaderI {
  public:

	 DataReaderI();
	 virtual ~DataReaderI();

	 void init_data_connectionI();
	 void clear_threadsI();
	 void stop_data_takingI();
	 void start_data_takingI();
	 void reset_buffersI();
	 void reset_countersI();
	 void get_feedbackI(bool &error, json&msgs,const bool reset);

	 bool     get_readyI();
	 void     set_readyI(bool rdy);
	 uint32_t get_n_sent_fragsI();
	 uint32_t get_n_statusI();
	 uint32_t get_n_gtriggersI();
	 uint32_t get_n_ltriggersI();
	 uint32_t get_n_feedbackI();
	 uint32_t get_n_timestampsI();
	 uint32_t get_sent_bytesI();

  protected :

  private :

	ptb::board_reader *reader_;

};

#endif /* SRC_BOARD_READER_INTERFACE_H_ */


