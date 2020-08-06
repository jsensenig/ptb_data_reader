/*
 * board_reader_interface.cpp
 *
 *  Created on: Aug 5, 2020
 *      Author: jon
 *
 *  See header for description
 */

#include "boardreader.h"
#include "board_reader_interface.h"


DataReaderI::DataReaderI() :
		reader_(0)
{
	reader_ = new ptb::board_reader();
}

DataReaderI::~DataReaderI() {
	delete reader_;
	reader_ = nullptr;
}

void DataReaderI::init_data_connectionI() {
	reader_->init_data_connection();
}
void DataReaderI::clear_threadsI() {
    reader_->clear_threads();
}
void DataReaderI::stop_data_takingI() {
    reader_->start_data_taking();
}
void DataReaderI::start_data_takingI() {
    reader_->start_data_taking();
}
void DataReaderI::reset_buffersI() {
    reader_->reset_buffers();
}
void DataReaderI::reset_countersI() {
    reader_->reset_counters();
}
void DataReaderI::get_feedbackI(bool &error, json&msgs,const bool reset = true) {
    reader_->get_feedback(error, msgs, reset);
}
bool DataReaderI::get_readyI() {
    return reader_->get_ready();
}
void DataReaderI::set_readyI(bool rdy) {
    reader_->set_ready(rdy);
}
uint32_t DataReaderI::get_n_sent_fragsI() {
    return reader_->get_n_sent_frags();
}
uint32_t DataReaderI::get_n_statusI() {
    return reader_->get_n_status();
}
uint32_t DataReaderI::get_n_gtriggersI() {
    return reader_->get_n_gtriggers();
}
uint32_t DataReaderI::get_n_ltriggersI() {
    return reader_->get_n_ltriggers();
}
uint32_t DataReaderI::get_n_feedbackI() {
    return reader_->get_n_feedback();
}
uint32_t DataReaderI::get_n_timestampsI() {
    return reader_->get_n_timestamps();
}
uint32_t DataReaderI::get_sent_bytesI() {
    return reader_->get_sent_bytes();
}
