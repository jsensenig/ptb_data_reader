/*
 * ctbdatareceiver.cpp
 *
 *  Created on: May 4, 2018
 *      Author: nbarros
 */

#include "ctbdatareceiver.hh"

#include <iostream>
#include <cinttypes>
#include "content.h"


#include <TFile.h>
#include <TTree.h>


ctb_data_receiver::ctb_data_receiver(uint16_t receive_port,int debug_level) :
                        debug_level_(debug_level),
                        acceptor_(io_service_, tcp::endpoint(tcp::v4(), (short)receive_port)),
                        accept_socket_(io_service_),
                        data_socket_(io_service_),
                        deadline_(io_service_),
                        deadline_io_object_(None),
                        tick_period_usecs_(5),
                        socket_timeout_us_(500000),
                        receive_port_(receive_port),
                        run_receiver_(true),
                        suspend_readout_(false),
                        readout_suspended_(false),
                        exception_(false),  // GBcopy
                        //recv_socket_(0),
                        receiver_thread_(nullptr),
                        archiver_thread_(nullptr),
                        archiver_ready_(false),
                        state_nbytes_recvd_(0),
                        total_bytes_recvd_(0),
                        total_payload_bytes_recvd_(0),
                        total_packets_recvd_(0),
                        next_receive_state_(ReceiveTCPHeader),
                        next_receive_size_(0),
                        sleep_on_stop_(10),
                        current_write_ptr_(nullptr),
                        current_read_ptr_(nullptr),
                        sequence_id_initialised_(false),
                        last_sequence_id_(0),
                        n_counter_words_(0),
                        n_llt_words_(0),
                        n_hlt_words_(0),
                        n_warn_words_(0),
                        n_ts_words_(0),
                        n_words_(0),
                        enable_root_output_(false),
                        root_file_name_("ctb_output.root")

{
  printf("ctb_data_receiver: In constructor. Going to receive on port %hu\n",receive_port);

  this->set_deadline(None,0);
  this->check_deadline();

  // -- start asynchronous accept handler
  this->do_accept();

  // Launch thread to run receiver IO service
  // -- this activates the asynchronous service...not the socket
  // the socket would in principle be automatically destroyed when start is called
  receiver_thread_ = std::unique_ptr<std::thread>(new std::thread(&ctb_data_receiver::run_service, this));

  current_read_ptr_ = &(raw_buffer_[0]);
  current_write_ptr_ = &(raw_buffer_[0]);
  tick_period_usecs_ = 10; // -- Wait time for fast waits
  socket_timeout_us_ = 500000; // -- hald a second timeout
  sleep_on_stop_     = 100;

  /// --  Silence warnings on unused variables
  ///     size_t n_counter_words_;
  (void)n_counter_words_;
  (void) n_llt_words_;
  (void) n_hlt_words_;
  (void) n_warn_words_;
  (void) n_ts_words_;
  (void) n_words_;
  (void) receive_port_;
  ///

  printf("ctb_data_receiver: In constructor. Limits of raw buffer: [%p , %p]\n",current_read_ptr_,&(raw_buffer_[buffer_n_bytes-1]));


}

ctb_data_receiver::~ctb_data_receiver() {
  printf("ctb_data_receiver:: In destructor\n");
  // Flag receiver as no longer running
  run_receiver_.store(false);

  // Cancel any currently pending IO object deadlines and terminate timer
  deadline_.expires_at(boost::asio::deadline_timer::traits_type::now());

  // Wait for thread running receiver IO service to terminate
  receiver_thread_->join();

  printf("ctb_data_receiver::destructor: receiver thread joined OK\n");
}


void ctb_data_receiver::start(void)
{
  printf("ctb_data_receiver::start: Start called\n");

  printf("ctb_data_receiver::start: Starting archiver_thread_\n");

  archiver_thread_ = std::unique_ptr<std::thread>(new std::thread(&ctb_data_receiver::process_received_data, this));

  printf("ctb_data_receiver::start: Waiting archiver to be ready to store data\n");
  while(!archiver_ready_.load())
  {
    usleep(tick_period_usecs_);
  }
  printf("ctb_data_receiver::start: Archiver ready\n");

  start_time_ = std::chrono::high_resolution_clock::now();

  // If the data receive socket is open, flush any stale data off it
  if (deadline_io_object_ == DataSocket)
  {
    printf("ctb_data_receiver::start: Reusing existing data socket. Flushing stale data.\n");
    std::size_t flush_length = 0;

    // -- Flush stale data on the socket

    while (data_socket_.available() > 0)
    {
      boost::array<char, 65536> buf;
      boost::system::error_code ec;

      size_t len = data_socket_.read_some(boost::asio::buffer(buf), ec);
      flush_length += len;
      printf("ctb_data_receiver::start: Flushed: %zu total: %zu available: %zu\n", len ,
          flush_length,
          data_socket_.available());

      if (ec == boost::asio::error::eof)
      {
        printf("ctb_data_receiver::start: client closed data socket connection during flush operation\n");
        break;
      }
      else if (ec)
      {
        printf("ctb_data_receiver::start: got unexpected socket error flushing data socket: %s\n",ec.message().c_str());
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

      printf("ctb_data_receiver::start: flushed %zu  bytes stale data off open socket\n",flush_length );
    }
    else
    {
      printf("ctb_data_receiver::start: no stale data to flush off socket\n");
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
  printf("ctb_data_receiver::stop called\n");

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
        printf("ERROR - timeout waiting for ctb_data_receiver thread to suspend readout\n");
      } catch (...) {  //GBcopy
      }  // GBcopy
      break;
    }
  }

  auto elapsed_msecs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time_).count();
  double elapsed_secs = ((double)elapsed_msecs) / 1000.0;
  double rate = ((double)total_packets_recvd_) / elapsed_secs;


  printf("ctb_data_receiver::stop : received %zu  packets in %lf seconds, rate %lf Hz\n",total_packets_recvd_,elapsed_secs , rate);

  // -- Join the archiver thread
  printf("ctb_data_receiver::stop: Joining archiver thread\n");
  archiver_thread_->join();
  printf("ctb_data_receiver::stop: Archiver thread joined\n");

}

void ctb_data_receiver::run_service(void)
{
  printf("ctb_data_receiver::run_service starting\n");

  io_service_.run();
  printf("ctb_data_receiver::run_service started\n");


  // GBcopy:   Make this a warning if we are not officially stopping the run
  if (suspend_readout_.load())  // Normal situation, it should stop after a bit when we stop the run
  {
    printf("ctb_data_receiver::run_service stopping\n");
  } else {
    printf("ctb_data_receiver::run_service: boost::asio::io_service has stopped running mysteriously inside the run.  Continuing the run from here is probably not useful.\n");
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

//  const size_t print_nth_timeout = 200;
//  static size_t nth_timeout = 0;

  if (exception())
  {
    printf("ctb_data_receiver::do_accept: Found exception in instance. Stopping data taking.\n");
    // -- stop should never be called from inside the instance
    suspend_readout_.store(true);
    //this->stop();
  }

  // Suspend readout and cleanup any incomplete millislices if stop has been called
  if (suspend_readout_.load())
  {

    if (first) {
      printf("ctb_data_receiver::do_accept: Suspending readout at do_accept entry\n");
    }
    this->suspend_readout(true);
    // -- Since this is a recursive function, call suspend_readout in waiting mode
    // to make sure that if a new start is called it can pick up from where it left off
  }

  // Exit if shutting receiver down
  if (!run_receiver_.load()) {
    // -- this is a trickier one, since it is over the mother service
    // if the service is shutting down, the function should do the same
    printf("ctb_data_receiver::do_accept: Stopping do_accept() at entry\n");
    return;
  }

  if (first) {
    printf("ctb_data_receiver::do_accept: Acceptor deadline set to %u us\n",socket_timeout_us_);
    first = false;
  }
  // -- Socket timeout to receive data
  this->set_deadline(Acceptor, socket_timeout_us_);

  acceptor_.async_accept(accept_socket_,
      [this](boost::system::error_code ec)
      {
    if (!ec)
    {
      // -- No error received...proceed to read the socket
      printf("ctb_data_receiver::do_accept: Accepted new data connection from source [%s:%hu]\n",
          accept_socket_.remote_endpoint().address().to_string().c_str(),
          accept_socket_.remote_endpoint().port());
      data_socket_ = std::move(accept_socket_);
      this->do_read();
    }
    else
    {
      if (ec == boost::asio::error::operation_aborted)
      {
        ;
        // -- the timeout was reached. Retry again.
        // No need to print confusing message
//        if (nth_timeout % print_nth_timeout == 0) {
//          static bool first_1 = true;
//          if (first_1) {
//            printf("ctb_data_receiver::do_accept: Timeout on async_accept\n");
//            first_1 = false;
//          }
//        }
//
//        nth_timeout++;
      }
      else
      {
        try {
          static bool first_2 = true;
          if (first_2) {
            printf("ctb_data_receiver::do_accept: Got error on asynchronous accept: %s\n",ec.message().c_str());
            first_2 = false;
          }
        } catch (...) {
          set_exception(true);
        }
      }
      // -- unsure if this should not be happening only in a known error situation
      // like the abort.
      this->do_accept();
    }
      }
  );
}




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



// NFB Dec-02-2015

// do_read handles the data received from the PTB, casts them into RawBuffer,
// converts them into MicroSlices and
// and pushes them into millislices
void ctb_data_receiver::do_read(void)
{

  if (exception())
  {
    static bool first = true;
    if (first) {
      printf("ctb_data_receiver::do_read: Exception state detected. Refusing to read\n");
      first = false;
    }
    // -- shouldn't even try to read the data. Just get out
    //this->suspend_readout(false);
    return;
  }

  // Suspend readout and cleanup any incomplete millislices if stop has been called
  if (suspend_readout_.load())
  {
    printf("ctb_data_receiver::do_read: Suspending readout at do_read entry\n");
    // keep going just for the sake of cleaning up the buffers
    //this->suspend_readout(true);
  }

  // Terminate receiver read loop if required
  if (!run_receiver_.load())
  {
    printf("ctb_data_receiver::do_read: Stopping do_read at entry\n");
    return;
  }

  // Set timeout on read from data socket
  this->set_deadline(DataSocket, socket_timeout_us_);

  /// Overlaps and remainings are now dealt with.
  /// -- Process the new data

  // NFB Dec-06-2015
  // There are only two types of data loaded from the socket: headers and payloads
  if (debug_level_ > 2) {
    printf("\nreceive state %u \ncurrent write ptr %p \nnext recv size %zu \n",
        (unsigned int)next_receive_state_,
        current_write_ptr_,
        next_receive_size_);
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
      printf("ctb_data_receiver::do_read: Received %zu bytes on socket\n",length);

      this->handle_received_data(length);

      this->do_read();
    }
    else
    {
      if (ec == boost::asio::error::eof)
      {
        printf("ctb_data_receiver::do_read: Client socket closed the connection\n");
        this->do_accept();
      }
      else if (ec == boost::asio::error::operation_aborted)
      {
        // NFB : Shoudln't this be a warning?  GDB: I think this is OK, this is the
        // usual thing to do if no data arrives in a certain time, which should
        // happen when we are waiting for the run to start, and if we are
        // checking often enough during the run.
        //printf("ctb_data_receiver::do_read: Timeout on read from data socket\n");
        this->do_read();
      }
      else
      {
        try {
          printf("ctb_data_receiver::do_read: Got error on aysnchronous read from the socket: %s\n",ec.message().c_str());
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

  printf("ctb_data_receiver::handle_received_data: Handling data with size %zu at state %u on addr %p\n",length,
      next_receive_state_ ,current_write_ptr_);

#ifdef __PTB_BOARD_READER_DEVEL_MODE__
  display_bits(current_write_ptr_, length, "PennDataReceiver");
#endif

  // The way this is working is by stitching a microslice and then roll back the
  // pointers to subtract stuff that should not be going there.
  // The problem with this approach is that it might fail if
  // there are packets that should not be sent inside the microslice.

  // update size of uslice component, uslice & mslice
  state_nbytes_recvd_    += length;
  total_bytes_recvd_     += length;


  /// no need for this. We will store straight away...or just print.
  // now we can update the current_write_ptr_
  // Set it to the end of the received data
  // current_write_ptr_ = static_cast<void*>(reinterpret_cast_checked<uint8_t*>(current_write_ptr_) + length );


  //uint16_t body_size = 0;
  ptb::content::tcp_header_t * tcp_header = nullptr;


  /// -- first make a couple of checks on the size of the package
  ///

  if (next_receive_size_ != state_nbytes_recvd_)
  {
    printf("ctb_data_receiver::handle_received_data: ERROR: Received %zu bytes but was expecting %zu\n",state_nbytes_recvd_,next_receive_size_);
    set_exception(true);
    return;
  }

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
        printf("ctb_data_receiver::handle_received_data: Received TCP header VER [0x%X] SIZE [%u] SEQ [%u]\n",
            (unsigned int) data_version,
            (unsigned int) next_size,
            (unsigned int) next_size );
      }

      // Check that the version matches the firmware
      if (((data_version >> 4) & 0xF) != ptb::content::format_version)
      {
        printf("ctb_data_receiver::handle_received_data: ERROR: Data has version [0x%X] but expected [0x%X]\n",
            (unsigned int) data_version,
            (unsigned int) ptb::content::format_version);
        set_exception(true);
        break;
      }

      // Validate the version in the packet
      uint8_t version            =  data_version & 0x0F;
      uint8_t version_complement = (data_version & 0xF0) >> 4;
      if( ! ((version ^ version_complement) << 4) ) {
        try {
          printf("ctb_data_receiver::handle_received_data: ERROR: packet version and version complement do not agree [0x%X] [0x%X]\n",
              (unsigned int)version,(unsigned int)version_complement);
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
          printf("ctb_data_receiver::handle_received_data: WARNING: mismatch in TCP packet sequence IDs! Got %u expected %u\n",
              (unsigned int)seq_id ,(unsigned int)(uint8_t(last_sequence_id_+1)));
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
      printf("ctb_data_receiver::handle_received_data: Next receive size : %zu bytes\n",next_receive_size_);
      // -- This is really the only relevant thing in this whole block
      next_receive_size_ = next_size;

      // The code below overwrites the TCP header since it is of no use to store the data
      // and roll back the current_write_ptr_, as to overwrite the Header in the next recv
      // We didn't advance the pointer, so we have no interest in backing it up
      //current_write_ptr_ = static_cast<void*>(reinterpret_cast<uint8_t*>(current_write_ptr_) - sizeof(ptb::content::tcp_header_t));

      break;
    }

    case ReceiveTCPPayload:
    {
      total_payload_bytes_recvd_ += length;
      //got a full microslice (complete size checks already done)
      total_packets_recvd_++;
      printf("ctb_data_receiver::handle_received_data: Complete payload received making a total %zu TCP packets with length %zu bytes (total %zu)\n",
          total_packets_recvd_,
          state_nbytes_recvd_,
          total_payload_bytes_recvd_);




      ///
      /// The real work is done here
      ///

      ///
      /// We want to actually walk the whole packet and store everything to a ROOT file
      ///

      // -- Actually, just shove the pointer into a buffer and queue it


      // --
      buffer_t buff;
      buff.addr = current_write_ptr_;
      buff.len = next_receive_size_;

      // queue the buffer
      bool res = buffer_queue_.push(buff);
      if (!res) {
        printf("ctb_data_receiver::handle_received_data: FATAL ERROR : All buffers are taken. ");
        set_exception(true);
        return;
      }

      next_receive_state_ = ReceiveTCPHeader;
      next_receive_size_ = sizeof(ptb::content::tcp_header_t);

      break;
    }//case

    default:
    {
      // Should never happen - bug or data corruption
      try {
        printf("ctb_data_receiver::handle_received_data: FATAL ERROR after async_recv - unrecognised next receive state: %u \n",next_receive_state_);
      } catch (...) {
        set_exception(true);
      }
      return;
      break;
    }
  }//switch(next_receive_state_)


  // -- advance the write pointer forward
  // but check that the next size does not go beyond the end of the buffer
  // Here one has to be careful with having the archiver instance running slower than the
  // data collector. But that is a problem beyond the scope of this example.
  // In case that happens, blame ROOT.
  // NOTE: In this case, I turned off the compression on the ROOT file precisely to avoid
  // the archival to be too slow

  if ((static_cast<uint8_t*>(&(raw_buffer_[buffer_n_bytes-1])) - static_cast<uint8_t*>(current_write_ptr_)) < next_receive_size_) {
    printf("ctb_data_receiver::handle_received_data: Reached the end of the circular buffer. Starting from the beginning.\n");
    // -- additional check that the current read buffer is not going to be overlapped by the write buffer
    //
    current_write_ptr_ = &(raw_buffer_[0]);
    printf("ctb_data_receiver::handle_received_data: empty %u %p-%p=%ld (next size %zu)\n",(!buffer_queue_.empty())?1:0,
        current_read_ptr_,
        current_write_ptr_,
        static_cast<long>((static_cast<uint8_t*>(current_read_ptr_) - static_cast<uint8_t*>(current_write_ptr_))),
        next_receive_size_);
    if (!buffer_queue_.empty() && (current_read_ptr_ >= current_write_ptr_) && ((static_cast<uint8_t*>(current_read_ptr_) - static_cast<uint8_t*>(current_write_ptr_)) < next_receive_size_))
    {
      printf("ERROR: No room left on buffer. You might consider increasing it\n");
      set_exception(true);
    } else {
      current_write_ptr_ = &(raw_buffer_[0]);
    }
  } else {
    if (next_receive_state_ == ReceiveTCPHeader) {
      current_write_ptr_ = static_cast<void*>(static_cast<uint8_t*>(current_write_ptr_)+state_nbytes_recvd_);
    }
  }

  //reset counters & ptrs for the next call
  state_nbytes_recvd_ = 0;


}

void ctb_data_receiver::process_received_data(void)
{
  // -- Flag that it is not ready to roll
  archiver_ready_.store(false);

  bool enable_root_output_ = true;
  if (enable_root_output_) {
    store_root();
  }
//  else {
//    // -- Just do some parting?
//  }

  return;
}

void ctb_data_receiver::store_root(void)
{

  TFile fout(root_file_name_.c_str(),"RECREATE");
  fout.SetCompressionLevel(0); // -- no compression
  fout.cd();
  TTree tout("data","CTB output data");

  unsigned int word_type;
  uint64_t timestamp;
  uint64_t payload;
  // -- In case the word is a channel_status word, 64 bits of the payload
  // won't be enough. In that case, the other two variables will come to it's
  // aid
  uint32_t beam_status;
  uint32_t crt_status;

  tout.Branch("word_type",&word_type);
  tout.Branch("timestamp",&timestamp,"timestamp/l");
  tout.Branch("payload",&payload,"payload/l");
  tout.Branch("beam_status",&beam_status);
  tout.Branch("crt_status",&crt_status);

  buffer_t buff;

  // -- Declare all possible types of words
  // -- If the word came, write it to disk
  ptb::content::word::ch_status_t *chs = NULL;
  //ptb::content::word::feedback_t  *fbk = NULL;
  ptb::content::word::timestamp_t *ts = NULL;
  ptb::content::word::trigger_t   *tg = NULL;
  ptb::content::word::word_t      *word = NULL;

  printf("ctb_data_receiver::store_root: Entering archiving loop\n");
  bool ret = false;
  archiver_ready_.store(true);
  // while at least one of these conditions is satisfied, just keep recording the data
  // Keep in mind that we need a double loop. One to fetch the buffer addresses
  // and another to loop over the contents of the buffer (size)
  while ((!buffer_queue_.empty()) || (!suspend_readout_.load()))
  {

    // -- if the queue is empty sleep for a tick and retry
    if (buffer_queue_.empty())
    {
      usleep(tick_period_usecs_);
      continue;
    }

    ret = buffer_queue_.pop(buff);
    if (ret == false) {
      printf("ctb_data_receiver::store_root: Failed to grab buffer from queue\n");
      set_exception(true);
      break;
    }

    current_read_ptr_ = buff.addr;
    //-- Loop over the contents of the buffer
    // -- we know that each word is 16 bytes long
    for ( size_t pos = 0; pos < buff.len; pos += ptb::content::word::word_t::size_bytes)
    {

      word = reinterpret_cast<ptb::content::word::word_t*>(current_read_ptr_);
      switch(word->word_type)
      {
        case ptb::content::word::t_fback:
        {
          printf("::store_root: WARNING:: Feedback word. TS: %" PRIX64 " Payload : [%" PRIX64 "]\n",word->timestamp,word->payload);
          break;
        }
        case ptb::content::word::t_gt:
        case ptb::content::word::t_lt:
        {
          tg = reinterpret_cast<ptb::content::word::trigger_t*>(current_read_ptr_);
          word_type = tg->word_type;
          timestamp = tg->timestamp;
          payload = tg->trigger_word;
          beam_status = 0;
          crt_status = 0;
          tout.Fill();
          break;
        }
        case ptb::content::word::t_ts:
        {
          // -- Timestamp words are keepalives
          // There is really no point in storing them
          // For educational purposes, I also mark here how to interpret them
          ts = reinterpret_cast<ptb::content::word::timestamp_t*>(current_read_ptr_);
          word_type = ts->word_type;
          timestamp = ts->timestamp;
          payload = 0;
          beam_status = 0;
          crt_status = 0;
          // don't store
          // tout.Fill();
          break;
        }
        case ptb::content::word::t_ch:
        {
          chs = reinterpret_cast<ptb::content::word::ch_status_t*>(current_read_ptr_);
          word_type = chs->word_type;
          timestamp = chs->timestamp;
          payload = chs->get_pds();
          crt_status = chs->get_crt();
          beam_status = chs->get_beam();
          tout.Fill();
          break;
        }
        default:
        {
          printf("::store_root: WARNING:: Unknown word type\n");
          printf("--> TP [%X] TS [%" PRIu64 "] PL [%" PRIx64 "]\n",word->word_type,word->timestamp,word->payload);
          printf("--> TP [%s] TS [%s] PL [%s]\n",std::bitset<3>(word->word_type).to_string().c_str(),
              std::bitset<64>(word->timestamp).to_string().c_str(),
              std::bitset<61>(word->payload).to_string().c_str());
          break;
        }
      }

      // -- advance the pointer
      current_read_ptr_ = static_cast<void*>(static_cast<uint8_t*>(current_read_ptr_) + ptb::content::word::word_t::size_bytes);
    } // -- for words in buffer

    // -- Fetch another word

  } // -- while run

  printf("::store_root: Stopping the archiving.Closing file.\n");
  tout.Write();
  fout.Close();
  archiver_ready_.store(false);
}





