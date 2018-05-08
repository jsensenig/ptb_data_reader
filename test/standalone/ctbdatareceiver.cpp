/*
 * ctbdatareceiver.cpp
 *
 *  Created on: May 4, 2018
 *      Author: nbarros
 */

#include "ctbdatareceiver.hh"

#include <iostream>
#include "content.h"

using std::cout;
using std::endl;
//ctb_data_receiver::ctb_data_receiver() {
//  // TODO Auto-generated constructor stub
//}

ctb_data_receiver::ctb_data_receiver(int debug_level, uint32_t tick_period_usecs,
    uint16_t receive_port) :
        debug_level_(debug_level),
        acceptor_(io_service_, tcp::endpoint(tcp::v4(), (short)receive_port)),
        accept_socket_(io_service_),
        data_socket_(io_service_),
        deadline_(io_service_),
        deadline_io_object_(None),
        tick_period_usecs_(tick_period_usecs),
        receive_port_(receive_port),
        run_receiver_(true),
        suspend_readout_(false),
        readout_suspended_(false),
        exception_(false),  // GBcopy
        recv_socket_(0),
        receiver_thread_(nullptr),
        state_nbytes_recvd_(0),
        next_receive_size_(0),
        total_bytes_recvd_(0),
        total_packets_recvd_(0),
        next_receive_state_(ReceiveTCPHeader),
        sleep_on_stop_(5),
        current_write_ptr_(nullptr),
        sequence_id_initialised_(false),
        last_sequence_id_(0),
        n_counter_words_(0),
        n_llt_words_(0),
        n_hlt_words_(0),
        n_warn_words_(0),
        n_ts_words_(0),
        n_words_(0)
{
  printf("ctb_data_receiver: In constructor\n");

  this->set_deadline(None,0);
  this->check_deadline();

  // -- start asynchronous accept handler
  this->do_accept();

  // Launch thread to run receiver IO service
  receiver_thread_ = std::unique_ptr<std::thread>(new std::thread(&ctb_data_receiver::run_service, this));

}

ctb_data_receiver::~ctb_data_receiver() {
  printf("ctb_data_receiver:: In destructor\n");
  // Flag receiver as no longer running
  run_receiver_.store(false);

  // Cancel any currently pending IO object deadlines and terminate timer
  deadline_.expires_at(boost::asio::deadline_timer::traits_type::now());

  // Wait for thread running receiver IO service to terminate
  receiver_thread_->join();

  printf("destructor: receiver thread joined OK\n");
}


void ctb_data_receiver::start(void)
{
  printf("ctb_data_receiver::start: Start called\n");
  start_time_ = std::chrono::high_resolution_clock::now();

  // If the data receive socket is open, flush any stale data off it
  if (deadline_io_object_ == DataSocket)
  {

    std::size_t flush_length = 0;

    // -- Flush stale data on the socket

    while (data_socket_.available() > 0)
    {
      boost::array<char, 65536> buf;
      boost::system::error_code ec;

      size_t len = data_socket_.read_some(boost::asio::buffer(buf), ec);
      flush_length += len;
      cout << "ctb_data_receiver::start: Flushed: " << len << " total: " << flush_length
          << " available: " << data_socket_.available() << endl;

      if (ec == boost::asio::error::eof)
      {
        cout << "ctb_data_receiver::start: client closed data socket connection during flush operation" << endl;
        break;
      }
      else if (ec)
      {
        cout << "ctb_data_receiver::start: got unexpected socket error flushing data socket: " << ec << endl;
        break;
      }
    }
    if (flush_length > 0)
    {
      // JCF, Feb-4-2016

      // I've observed that when the RceDataReceiver has
      // to flush bytes of stale data it pretty much
      // guarantees an error later in the run; given that
      // this code is largely based on the
      // RceDataReceiver, then, I've promoted this message
      // from "Info" level to "Warning" level

      cout << "ctb_data_receiver::start: flushed " << flush_length << " bytes stale data off open socket" << endl;
    }
    else
    {
      cout << "lbne::PennDataReceiver::start: no stale data to flush off socket" << endl;
    }
  }

  // Clean up current buffer state of any partially-completed readouts in previous runs
  //  if (current_raw_buffer_ != nullptr)
  //  {
  //    DAQLogger::LogInfo("PennDataReceiver") << "::start: dropping unused or partially filled buffer containing "
  //        << microslices_recvd_ << " microslices";
  //    current_raw_buffer_.reset();
  //    millislice_state_ = MillisliceEmpty;
  //  }

  // Initialise receive state to read a microslice header first
  next_receive_state_ = ReceiveTCPHeader;
  next_receive_size_  = sizeof(ptb::content::tcp_header_t);
  current_write_ptr_ = &(raw_buffer_[0]);
  // Initalise to make sure we wait for the full Header
  state_nbytes_recvd_ = 0;
  total_bytes_recvd_ = 0;
  total_packets_recvd_ = 0;

  // Clear suspend readout handshake flags
  suspend_readout_.store(false);
  readout_suspended_.store(false);
  exception_.store(false);

}


void ctb_data_receiver::stop(void)
{
  cout << "ctb_data_receiver::stop called" << endl;

  // Suspend readout and wait for receiver thread to respond accordingly
  suspend_readout_.store(true);

  //  const uint32_t stop_timeout_usecs = 5000000;
  // uint32_t max_timeout_count = stop_timeout_usecs / tick_period_usecs_;
  uint32_t max_timeout_count = sleep_on_stop_ / tick_period_usecs_;
  uint32_t timeout_count = 0;
  while (!readout_suspended_.load())
  {
    usleep(tick_period_usecs_);
    timeout_count++;
    if (timeout_count > max_timeout_count)
    {
      // GBcopy: In the RCEs, JCF (Oct-24, 2015) recommends we either
      // downgrade this to a warning or swallow the exception this automatically throws
      try {    // GBcopy
        cout<< "ERROR - timeout waiting for ctb_data_receiver thread to suspend readout" << endl;
      } catch (...) {  //GBcopy
      }  // GBcopy
      break;
    }
  }

  auto elapsed_msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time_).count();
  double elapsed_secs = ((double)elapsed_msecs) / 1000;
  double rate = ((double)total_packets_recvd_) / elapsed_secs;


  cout  << "ctb_data_receiver::stop : received " << total_packets_recvd_ << " packets in "
      << elapsed_secs << " seconds, rate "
      << rate << " Hz" << endl;

}

void ctb_data_receiver::run_service(void)
{
  cout << "ctb_data_receiver::run_service starting" << endl;

  io_service_.run();

  // GBcopy:   Make this a warning if we are not officially stopping the run
  if (suspend_readout_.load())  // Normal situation, it should stop after a bit when we stop the run
  {
    cout << "ctb_data_receiver::run_service stopping" << endl;
  } else {
    cout << "ctb_data_receiver::run_service: boost::asio::io_service has stopped running mysteriously inside the run.  Continuing the run from here is probably not useful." << endl;
  }
}

// NFB Dec-02-2015
//
//do_accept simply keeps track of accepting the ethernet connection from the PTB
// Then redirects the handling of the received data to do_read
void ctb_data_receiver::do_accept(void)
{
  static bool first = true;
  if (first)
  {
    printf("ctb_data_receiver::do_accept starting\n");
  }
  // JCF, Jul-29-2015

  // The "Timeout on async_accept" message appears many, many times --
  // O(1000) -- before the connection is made; to reduce clutter, I'm
  // only going to have it display every nth_timeout times

  const size_t print_nth_timeout = 200;
  static size_t nth_timeout = 0;

  // Suspend readout and cleanup any incomplete millislices if stop has been called
  if (suspend_readout_.load())
  {

    printf("ctb_data_receiver::do_accept: Suspending readout at do_accept entry\n");
    this->suspend_readout(false);
  }

  // Exit if shutting receiver down
  if (!run_receiver_.load()) {
    cout << "ctb_data_receiver::do_accept: Stopping do_accept() at entry" << endl;
    return;
  }

  printf("ctb_data_receiver::do_accept: Acceptor deadline set to %u us\n",tick_period_usecs_);
  this->set_deadline(Acceptor, tick_period_usecs_);

  acceptor_.async_accept(accept_socket_,
      [this](boost::system::error_code ec)
      {
    if (!ec)
    {
      cout << "ctb_data_receiver::do_accept: Accepted new data connection from source " << accept_socket_.remote_endpoint() << endl;
      data_socket_ = std::move(accept_socket_);
      this->do_read();
    }
    else
    {
      if (ec == boost::asio::error::operation_aborted)
      {
        if (nth_timeout % print_nth_timeout == 0) {
          cout << "ctb_data_receiver::do_accept: Timeout on async_accept" << endl;
        }

        nth_timeout++;
      }
      else
      {
        try {
          cout << "ctb_data_receiver::do_accept: Got error on asynchronous accept: " << ec << " " << ec.message() << endl;
        } catch (...) {
          set_exception(true);
        }
      }
      this->do_accept();
    }
      }
  );
}

// NFB Dec-02-2015

// do_read handles the data received from the PTB, casts them into RawBuffer,
// converts them into MicroSlices and
// and pushes them into millislices
void ctb_data_receiver::do_read(void)
{

  // Suspend readout and cleanup any incomplete millislices if stop has been called
  if (suspend_readout_.load())
  {
    cout << "ctb_data_receiver::do_read: Suspending readout at do_read entry" << endl;
    this->suspend_readout(true);
  }

  // Terminate receiver read loop if required
  if (!run_receiver_.load())
  {
    cout << "ctb_data_receiver::do_read: Stopping do_read at entry" << endl;
    return;
  }

  // Set timeout on read from data socket
  this->set_deadline(DataSocket, tick_period_usecs_);

  /// Overlaps and remainings are now dealt with.
  /// -- Process the new data

  // NFB Dec-06-2015
  // There are only two types of data loaded from the socket: headers and payloads
  if (debug_level_ > 2) {
    cout << "\nreceive state "     << (unsigned int)next_receive_state_
        << "\ncurrent write ptr "           << current_write_ptr_
        << "\nnext recv size " << next_receive_size_;
  }
  // The data should not go
  // Start the asynchronous receive operation into the (existing) current raw buffer.

  // NFB -- Jan, 14 2016
  // According to boost documentation:
  /**
   * The receive operation may not receive all of the requested number of bytes.
   * Consider using the async_read function if you need to ensure that the requested
   * amount of data is received before the asynchronous operation completes.
   */
  //data_socket_.async_receive(
  boost::asio::async_read(data_socket_,
      boost::asio::buffer(current_write_ptr_, next_receive_size_),
      [this](boost::system::error_code ec, std::size_t length)
      {
    if (!ec)
    {
      cout << "ctb_data_receiver::do_read: Received " << length << " bytes on socket" << endl;

      this->handle_received_data(length);

      this->do_read();
    }
    else
    {
      if (ec == boost::asio::error::eof)
      {
        cout  << "ctb_data_receiver::do_read: Client socket closed the connection" << endl;
        this->do_accept();
      }
      else if (ec == boost::asio::error::operation_aborted)
      {
        // NFB : Shoudln't this be a warning?  GDB: I think this is OK, this is the
        // usual thing to do if no data arrives in a certain time, which should
        // happen when we are waiting for the run to start, and if we are
        // checking often enough during the run.
        cout << "ctb_data_receiver::do_read: Timeout on read from data socket" << endl;
        this->do_read();
      }
      else
      {
        try {
          cout << "ctb_data_receiver::do_read: Got error on aysnchronous read from the socket: " << ec << " " << ec.message() << endl;
        } catch (...) {
          set_exception(true);
        }
      }

    }
      }
  );
}

//// -- handle received data
///  This function should be implemented in different ways, depending on how it is meant to be functioning
///

void ctb_data_receiver::handle_received_data(std::size_t length)
{

  cout << "ctb_data_receiver::handle_received_data: Handling "
    << " data with size " << (unsigned int)length << " at status " << next_receive_state_ << endl;
    //nextReceiveStateToString(next_receive_state_);

#ifdef __PTB_BOARD_READER_DEVEL_MODE__
  display_bits(current_write_ptr_, length, "PennDataReceiver");
#endif

  // The way this is working is by stitching a microslice and then roll back the
  // pointers to subtract stuff that should not be going there.
  // The problem with this approach is that it might fail if
  // there are packets that should not be sent inside the microslice.

  //update size of uslice component, uslice & mslice
  state_nbytes_recvd_    += length;
  total_bytes_recvd_ += length;


  /// no need for this. We will store straight away...or just print.
  //now we can update the current_write_ptr_
  // Set it to the end of the received data
  //current_write_ptr_ = static_cast<void*>(reinterpret_cast_checked<uint8_t*>(current_write_ptr_) + length );


  uint16_t body_size = 0;
  ptb::content::tcp_header_t * tcp_header = nullptr;


  /// -- first make a couple of checks on the size of the package
  ///

  if (next_receive_size_ != state_nbytes_recvd_)
  {
    cout << "ctb_data_receiver::handle_received_data: ERROR: Received " << state_nbytes_recvd_
        << " but was expecting " << next_receive_size_ << endl;
    set_exception(true);
    return;
  }

  // JCF, Jul-28-2015

  // I've replaced Tom Dealtry's use of the
  // boost::crc_32_type calculation for the checksum with
  // Nuno's BSD method
  // (https://en.wikipedia.org/wiki/BSD_checksum),
  // implemented in the ptb_runner program

  // Note this code relies on the only two possible receive
  // states being "ReceiveMicrosliceHeader" and
  // "ReceiveMicroslicePayload"

#ifdef DO_CHECKSUM
  static uint16_t software_checksum = 0; // "static" means this
#endif /*DO_CHECKSUM*/
  // is the same variable
  // across calls to this
  // function

//  size_t bytes_to_check = 0;
//
//  // Reset the checksum to zero when we're expecting a new
//  // microslice, and if we're looking at the microslice payload,
//  // don't factor the contents of the checksum word into the
//  // checksum itself
//  if (next_receive_state_ == ReceiveMicrosliceHeader) {
//#ifdef DO_CHECKSUM
//    software_checksum = 0;
//#endif /*DO_CHECKSUM*/
//    bytes_to_check = length;
//  } else {
//    // If the next state is the payload we want to discount the header from the size to be checked
//    bytes_to_check = length - sizeof(lbne::PennMicroSlice::Header);
//  }
//
//  RECV_DEBUG(2) << "Calculating checksum with status " << nextReceiveStateToString(next_receive_state_)
//                << " on length  " << length
//                << " bytes_to_check " << bytes_to_check;
//
//
//#ifdef DO_CHECKSUM
//  uint8_t* byte_ptr = reinterpret_cast_checked<uint8_t*>(receiver_state_start_ptr_);
//  //  RECV_DEBUG(4) << "JCF: current value of checksum is " << software_checksum <<
//  //    ", will look at " << bytes_to_check << " bytes starting at " <<
//  //    static_cast<void*>(byte_ptr) << " in the checksum calculation";
//  for (size_t i_byte = 0; i_byte < bytes_to_check; ++i_byte) {
//    software_checksum = (software_checksum >> 1) + ((software_checksum & 0x1) << 15) ;
//    software_checksum += *(byte_ptr + i_byte);
//    software_checksum &= 0xFFFF;
//  }
//#endif /*DO_CHECKSUM*/
//  // -- Finished checking checksum

  switch (next_receive_state_)
  {
  case ReceiveTCPHeader:
    {
      // Just testing that it does not go belly up
      // validate_microslice_header();

      tcp_header = reinterpret_cast<ptb::content::tcp_header_t *>(current_write_ptr_);
      ptb::content::tcp_header_t::ver_size_t data_version = tcp_header->format_version;
      ptb::content::tcp_header_t::seq_size_t seq_id = tcp_header->sequence_id;
      ptb::content::tcp_header_t::pkt_size_t next_size = tcp_header->packet_size;

      if (debug_level_ > 2)
      {
        cout << "ctb_data_receiver::handle_received_data: Received TCP header VER [0x"
            << std::hex << (unsigned int) data_version
            << std::dec << "] SIZE [" << (unsigned int) next_size
            << "] SEQ [" << (unsigned int) next_size << "]" << endl;
      }

      // Check that the version matches the firmware
      if (data_version != ptb::content::format_version)
      {
        cout << "ctb_data_receiver::handle_received_data: ERROR: Data has version 0x"
            << std::hex << (unsigned int) data_version << std::dec << " but expected 0x"
            << std::hex << (unsigned int) ptb::content::format_version << std::dec << endl;
        set_exception(true);
      }


      // Validate the version in the packet
      uint8_t version            =  data_version & 0x0F;
      uint8_t version_complement = (data_version & 0xF0) >> 4;
      if( ! ((version ^ version_complement) << 4) ) {
        try {
          cout << "ctb_data_receiver::handle_received_data: ERROR: Microslice version and version complement do not agree 0x"
          << std::hex << (unsigned int)version << ", 0x"
          << (unsigned int)version_complement << std::dec << endl;
        } catch (...) {
          set_exception(true);
        }
      }


      // Validate the sequence ID - should be incrementing
      // monotonically (or identical to previous if it was
      // fragmented)

      if (sequence_id_initialised_ && (seq_id != uint8_t(last_sequence_id_+1)))
      {
        try {
          cout << "ctb_data_receiver::handle_received_data: WARNING: mismatch in TCP packet sequence IDs! Got "
              << (unsigned int)seq_id << " expected " << (unsigned int)(uint8_t(last_sequence_id_+1)) << endl;
        } catch (...) {
          set_exception(true);
        }
      }

      if (!sequence_id_initialised_)
      {
        sequence_id_initialised_ = true;
      }
      last_sequence_id_ = seq_id;

      // update states ready for next call.
      // Since this was a header we don't really need anything else
      next_receive_state_ = ReceiveTCPPayload;
      next_receive_size_ = next_size;

      // The code below overwrites the TCP header since it is of no use to store the data
      // and roll back the current_write_ptr_, as to overwrite the Header in the next recv
      current_write_ptr_ = static_cast<void*>(reinterpret_cast<uint8_t*>(current_write_ptr_) - sizeof(ptb::content::tcp_header_t));

      break;
    }

  case ReceiveTCPPayload:
    {
      //got a full microslice (complete size checks already done)
      cout << "ctb_data_receiver::handle_received_data: Complete payload received making a total TCP " << total_packets_recvd_ << " length " << state_nbytes_recvd_;
      total_packets_recvd_++;


      //      FIXME: Implement a crosscheck on the structure and the .
      //      try{
      //        validate_microslice_payload();
      //      }catch(...) {
      //        // payload didn't validate for some reason. Send an error and print the whole thing
      //        DAQLogger::LogInfo("PennDataReceiver") << "Error was caught validating a run. Dumping the culprit microslice";
      //        display_bits(receiver_state_start_ptr_,length,"PennDataReceiver");
      //        set_exception(true);    // GB+JM-A
      //      }
      //
      //      if(!run_start_time_) {
      //        DAQLogger::LogInfo("PennDataReceiver") << "This is the first MicroSlice. Estimating run start time from the first payload.";
      //
      //        // NFB Dec-06-2015
      //        // This is tricky. The easiest way would be to drop the data until a timestamp was found.
      //
      //        // Actually, the best way is to do a multiphase approach:
      //        //1. grab the first timestamp from the first payload_header
      //        //2. Walk the payloads until a full timestamp is found.
      //        //3. Calculate the difference between the rollovers and subtract from the full TS
      //        //4. Set the start run time to that value
      //
      //        uint8_t *current_data_ptr = static_cast<uint8_t*>(receiver_state_start_ptr_);
      //
      //        // I know that the first microslice sent by the PTB
      //        // is a timestamp. Just grab it
      //
      //        // 1. -- Grab the first timestamp -- confirm it is indeed a timestamp
      //        lbne::PennMicroSlice::Payload_Header *payload_header = static_cast<lbne::PennMicroSlice::Payload_Header *>(receiver_state_start_ptr_);
      //
      //        if (payload_header->data_packet_type != lbne::PennMicroSlice::DataTypeTimestamp) {
      //          DAQLogger::LogWarning("PennDataReceiver") << "Expected the first word to be a timestamp.  ";
      //        }
      //        current_data_ptr+= sizeof(lbne::PennMicroSlice::Payload_Header);
      //        lbne::PennMicroSlice::Payload_Timestamp *ts_word = reinterpret_cast_checked<lbne::PennMicroSlice::Payload_Timestamp*>(current_data_ptr);
      //
      //        run_start_time_ = ts_word->nova_timestamp;
      //        boundary_time_  = (run_start_time_ + millislice_size_ - 1);
      //        overlap_time_   = (boundary_time_  - millislice_overlap_size_);
      //        DAQLogger::LogInfo("PennDataReceiver") << "start run time estimated to be " << run_start_time_
      //              << " boundary_time " << boundary_time_
      //              << " overlap time " << overlap_time_;
      //      } // if !run_start_time_
      //
      //      form a microslice
      //      This microslice will only have the payload (including checksum)
      //      lbne::PennMicroSlice uslice(((uint8_t*)receiver_state_start_ptr_));



      ///
      /// The real work is done here
      ///

      ///
      /// We want to actually walk the whole packet and store everything to a ROOT file
      ///

      bool success = process_payload();

      if (!success)
      {
        cout << "ctb_data_receiver::handle_received_data: Failed to process received packet." << endl;
        set_exception(true);
        break;
      }

      // FIXME: Print local statistics...at some other point
//
//      cout << "ctb_data_receiver::handle_received_data: Payload contains " << n_words_local_
//          << " total words ("    << n_counter_words_local_
//          << " counter + "       << n_trigger_words_local_
//          << " trigger + "       << n_timestamp_words_local_
//          << " timestamp + "     << n_warning_words_local_
//          << " warning)" << endl;



      next_receive_state_ = ReceiveTCPHeader;
      next_receive_size_ = sizeof(ptb::content::tcp_header_t);

      break;
  }//case

    default:
    {
      // Should never happen - bug or data corruption
      try {
  cout << "ctb_data_receiver::handle_received_data: FATAL ERROR after async_recv - unrecognised next receive state: " << next_receive_state_;
      } catch (...) {
  set_exception(true);
      }
      return;
      break;
    }
  }//switch(next_receive_state_)

  //reset counters & ptrs for the next call
  state_nbytes_recvd_ = 0;


}

//void ctb_data_receiver::suspend_readout(bool await_restart)
//{
//  readout_suspended_.store(true);
//
//  if (await_restart)
//  {
//    cout << "::suspend_readout: awaiting restart or shutdown" << endl;
//    while (suspend_readout_.load() && run_receiver_.load())
//    {
//      usleep(tick_period_usecs_);
//    }
//    cout << "::suspend_readout: restart or shutdown detected, exiting wait loop" << endl;
//  }
//
//}



void ctb_data_receiver::suspend_readout(bool await_restart)
{
  readout_suspended_.store(true);

  if (await_restart)
  {
    printf("ctb_data_receiver::suspend_readout: awaiting restart or shutdown\n");
    while (suspend_readout_.load() && run_receiver_.load())
    {
      usleep(tick_period_usecs_);
    }
    printf("ctb_data_receiver::suspend_readout: restart or shutdown detected, exiting wait loop\n");
  }

}

void ctb_data_receiver::set_deadline(DeadlineIoObject io_object, unsigned int timeout_usecs)
{

  // Set the current IO object that the deadline is being used for
  deadline_io_object_ = io_object;

  // Set the deadline for the asynchronous write operation if the timeout is set, otherwise
  // revert back to +ve infinity to stall the deadline timer actor
  if (timeout_usecs > 0)
  {
    deadline_.expires_from_now(boost::posix_time::microseconds(timeout_usecs));
  }
  else
  {
    deadline_.expires_from_now(boost::posix_time::pos_infin);
  }
}

void ctb_data_receiver::check_deadline(void)
{

  // Handle deadline if expired
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {

    // Cancel pending operation on current specified object
    switch (deadline_io_object_)
    {
      case Acceptor:
        acceptor_.cancel();
        break;
      case DataSocket:
        data_socket_.cancel();
        break;
      case None:
        // Do nothing
        break;
      default:
        // Shouldn't happen
        break;
    }

    // No longer an active deadline to set the expiry to positive inifinity
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Put the deadline actor back to sleep if receiver is still running
  if (run_receiver_.load())
  {
    deadline_.async_wait(boost::bind(&ctb_data_receiver::check_deadline, this));
  }
  else
  {
    printf("ctb_data_receiver::check_deadline: Deadline actor terminating\n");
  }
}






bool ctb_data_receiver::process_payload(void)
{
  // -- Start by just showing the contents and incrementing the counters
  ptb::content::word::word_t      *word = NULL;
  ptb::content::word::timestamp_t *ts = NULL;
  ptb::content::word::ch_status_t *chs = NULL;
  ptb::content::word::trigger_t   *trigger = NULL;
  ptb::content::word::feedback_t  *warn = NULL;


  size_t pos = 0;
  uint8_t* tcp_data = (uint8_t*)current_write_ptr_;

  while(pos < state_nbytes_recvd_)
  {
    // -- grab the full frame
    word = reinterpret_cast<ptb::content::word::word_t*>(&tcp_data[pos]);
    pos += word->size_bytes;

    switch (word->word_type) {
      case ptb::content::word::t_fback:
      {
        warn = reinterpret_cast<ptb::content::word::feedback_t *>(word);
        cout << "!! --> Received a warning! " << std::bitset<3>(warn->word_type)
            << "ts " << warn->timestamp
            << " SOURCE " << warn->source
            << " CODE " << warn->code << endl;
        n_warn_words_++;
        n_words_++;
        break;
      }
      case ptb::content::word::t_gt:
      {
        // -- Receive a global trigger Print TS and mask
        trigger = reinterpret_cast<ptb::content::word::trigger_t *>(word);
        cout << "HLT : ts " << trigger->timestamp << " "
            << " mask " << std::bitset<61>(trigger->trigger_mask) << endl;
        n_hlt_words_++;
        n_words_++;
        break;
      }
      case ptb::content::word::t_lt:
      {
        // -- Receive a low level trigger Print TS and mask
        trigger = reinterpret_cast<ptb::content::word::trigger_t *>(word);
        cout << "LLT : ts " << trigger->timestamp << " "
            << " mask " << std::bitset<61>(trigger->trigger_mask) << endl;
        n_llt_words_++;
        n_words_++;
        break;
      }
      case ptb::content::word::t_ts:
      {
        ts = reinterpret_cast<ptb::content::word::timestamp_t *>(word);
        cout << "TS : ts " << ts->timestamp << endl;
        n_ts_words_++;
        n_words_++;
        break;
      }
      case ptb::content::word::t_ch:
      {
        chs = reinterpret_cast<ptb::content::word::ch_status_t *>(word);
        cout << "CHS : ts " << chs->timestamp
            << " PDS 0x" << std::hex << chs->pds  << std::dec
            << " CRT 0x" << std::hex << chs->crt  << std::dec
            << " BI  0x" << std::hex << chs->beam << std::dec << endl;
        n_counter_words_++;
        n_words_++;
        break;
      }
      default:
      {
        cout << "ERROR : Unknown word type : " << std::bitset<3>(word->word_type) << endl;
        break;
      }
    }

  }

  return true;
}







