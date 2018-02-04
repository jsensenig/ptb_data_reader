/*
 * test.cxx
 *
 *  Created on: Jun 7, 2015
 *      Author: nbarros
 */

#include <bitset>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <string>
#include <fstream>
#include <iostream>
#include <csignal>
#include <cstdint>
#include <sstream>

// -- Data type definitions
#include "content.h"

// -- Boost library for TCP socket use
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

// -- namespace declarations for common STL objects
using std::cout;
using std::cerr;
using std::endl;
using std::cin;

using boost::asio::ip::tcp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;


// -- configuration that we are running with for now
// EDIT here where the IP where you are running, which is where the CTB will attempt to connect to send the data
static const std::string g_config = "<config><DataBuffer><DaqHost>128.91.191.222</DaqHost><DaqPort>8992</DaqPort></DataBuffer><RolloverClocks>25000000</RolloverClocks></config>";

class ctb_robot {
  public:
    ctb_robot(const std::string &host = "localhost",const uint16_t &port = 8991, const uint16_t&listen_port = 8992)
  : stop_req_(false), is_running_(false),is_conf_(false),quit_req_(false),listen_port_(listen_port),
    srv_thread_(nullptr),
    acceptor_(io_service_,tcp::v4(),(unsigned short)listen_port),
    accept_socket_(io_service_),
    data_socket_(io_service_),
    deadline_(io_service_),
    deadline_io_object_(None),
    run_receiver_(true),
    suspend_readout_(false),
    readout_suspended_(false),
    recv_socket_(0),
    state_nbytes_recvd_(0),
    current_receive_state_(TcpHeader),
    next_receive_size_(0),
    pld(nullptr), tcp_header(nullptr),word(nullptr),hdr(nullptr)


  {
      std::cout.setf(std::ios::unitbuf);

      //      tcp::resolver resolver(*io_service_);
      //
      //      tcp::resolver::query query(tcp::v4(), host,std::to_string(port));
      //      tcp::resolver::iterator iterator = resolver.resolve(query);
      //      client_socket_ = boost::make_shared<boost::asio::ip::tcp::socket>(*io_service_);
      //      boost::asio::connect(*client_socket_, iterator);
      //      ec_ = boost::make_shared<boost::system::error_code>();
      // Initialize and start the persistent deadline actor that handles socket operation
      // timeouts. For now set the timeout to zero (no timeout)
      this->set_deadline(None, 0);
      this->check_deadline();

      // Start asynchronous accept handler
      this->do_accept();

      receiver_thread_ = std::unique_ptr<std::thread>(new std::thread(&ctb_robot::run_service, this));


  }
    virtual ~ctb_robot() {
      // Flag receiver as no longer running
      run_receiver_.store(false);
      // Cancel any currently pending IO object deadlines and terminate timer
      deadline_.expires_at(boost::asio::deadline_timer::traits_type::now());

      // Wait for thread running receiver IO service to terminate
      receiver_thread_->join();


    }

    void suspend_readout(bool await_restart)
    {
      readout_suspended_.store(true);

      if (await_restart)
      {
        cout << "::suspend_readout: awaiting restart or shutdown" << endl;
        while (suspend_readout_.load() && run_receiver_.load())
        {
          usleep(tick_period_usecs_);
        }
        cout << "::suspend_readout: restart or shutdown detected, exiting wait loop" << endl;
      }

    }

    void start() {
      //start_time_ = std::chrono::high_resolution_clock::now();

      // If the data receive socket is open, flush any stale data off it
      if (deadline_io_object_ == DataSocket)
      {

        std::size_t flush_length = 0;

        while (data_socket_.available() > 0)
        {
          boost::array<char, 65536> buf;
          boost::system::error_code ec;

          size_t len = data_socket_.read_some(boost::asio::buffer(buf), ec);
          flush_length += len;
          if (ec == boost::asio::error::eof)
          {
            cout<< ":start: client closed data socket connection during flush operation" << endl;
            break;
          }
          else if (ec)
          {
            cout << "::start: got unexpected socket error flushing data socket: " << ec << endl;
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

          cout  << "lbne::PennDataReceiver::start: flushed " << flush_length << " bytes stale data off open socket" << endl;
        }
        else
        {
          cout << "lbne::PennDataReceiver::start: no stale data to flush off socket" << endl;
        }
      }

      state_nbytes_recvd_ = 0;

      // Clear suspend readout handshake flags
      suspend_readout_.store(false);
      readout_suspended_.store(false);

    }

    void stop() {
      // Suspend readout and wait for receiver thread to respond accordingly
      suspend_readout_.store(true);

      uint32_t timeout_count = 0;
      while (!readout_suspended_.load())
      {
        usleep(100);
        timeout_count++;
      }
    }


    void run_service(void) {
      io_service_.run();
      if (suspend_readout_.load())  // Normal situation, it should stop after a bit when we stop the run
      {
        cout << "::run_service stopping" << endl;
      } else {
        cout << "boost::asio::io_service has stopped running mysteriously inside the run.  Continuing the run from here is probably not useful." << endl;
      }

    }

    //do_accept simply keeps track of accepting the ethernet connection from the PTB
    // Then redirects the handling of the received data to do_read
    void do_accept() {

      const size_t print_nth_timeout = 200;
      static size_t nth_timeout = 0;

      // Suspend readout and cleanup any incomplete millislices if stop has been called
      if (suspend_readout_.load())
      {
        cout << "Suspending readout at do_accept entry" << endl;
        this->suspend_readout(false);
      }

      // Exit if shutting receiver down
      if (!run_receiver_.load()) {
        cout << "Stopping do_accept() at entry" << endl;
        return;
      }

      this->set_deadline(Acceptor, 100);

      acceptor_.async_accept(accept_socket_,
          [this](boost::system::error_code ec)
          {
        if (!ec)
        {
          cout << "Accepted new data connection from source " << accept_socket_.remote_endpoint() << endl;
          data_socket_ = std::move(accept_socket_);
          this->do_read();
        }
        else
        {
          if (ec == boost::asio::error::operation_aborted)
          {
            if (nth_timeout % print_nth_timeout == 0) {
              cout << "Timeout on async_accept" << endl;
            }

            nth_timeout++;
          }
          else
          {
            try {
              cout << "Got error on asynchronous accept: " << ec << " " << ec.message() << endl;
            } catch (...) {
              cout << "Something went wrong " << endl;
            }
          }
          this->do_accept();
        }
          }
      );

    }


    // do_read handles the data received from the PTB, casts them into RawBuffer,
    // converts them into MicroSlices and
    // and pushes them into millislices
    void do_read(void)
    {

      // Suspend readout and cleanup any incomplete millislices if stop has been called
      if (suspend_readout_.load())
      {
        cout << "Suspending readout at do_read entry" << endl;
        this->suspend_readout(true);
      }

      // Terminate receiver read loop if required
      if (!run_receiver_.load())
      {
        cout << "Stopping do_read at entry" << endl;
        return;
      }

      // Set timeout on read from data socket
      this->set_deadline(DataSocket, 100);


      // The data should not go
      // Start the asynchronous receive operation into the (existing) current raw buffer.
      uint16_t bytes_collected= 0;

      //      do {
      // NFB -- Jan, 14 2016
      // According to boost documentation:
      /**
       * The receive operation may not receive all of the requested number of bytes.
       * Consider using the async_read function if you need to ensure that the requested
       * amount of data is received before the asynchronous operation completes.
       */
      //data_socket_.async_receive(
      next_receive_size_ = sizeof(header->size_bytes);
      boost::asio::async_read(data_socket_,
          boost::asio::buffer(tcp_data_, next_receive_size_),
          [this](boost::system::error_code ec, std::size_t length)
          {
        if (!ec)
        {
          cout << "Received " << length << " bytes on socket" << endl;

          // -- handle received data here
          this->handle_received_data(length);

          this->do_read();
        }
        else
        {
          if (ec == boost::asio::error::eof)
          {
            cout << "Client socket closed the connection" << endl;
            this->do_accept();
          }
          else if (ec == boost::asio::error::operation_aborted)
          {
            // NFB : Shoudln't this be a warning?  GDB: I think this is OK, this is the
            // usual thing to do if no data arrives in a certain time, which should
            // happen when we are waiting for the run to start, and if we are
            // checking often enough during the run.
            cout << "Timeout on read from data socket" << endl;
            this->do_read();
          }
          else
          {
            try {
              cout << "Got error on aysnchronous read from the socket: " << ec << " " << ec.message() << endl;
            } catch (...) {
              ;
            }
          }

        }
          }
      );
    }


    void handle_received_data(std::size_t length) {
      // -- check state to know how to handle
      if (current_receive_state_ == TcpHeader) {
        tcp_header = reinterpret_cast<ptb::content::tcp_header_t *>(tcp_data_);
        count_++;
        tcp_body_size_ = tcp_header->packet_size;
        next_receive_size_ = tcp_body_size_;
      } else {
        if (length != tcp_body_size_) {
          cout << "Was expecting " << tcp_body_size_ << " but received " << length << endl;
        }
        uint32_t pos = 0;
        while (pos < tcp_body_size_) {
          // 1. grab the frame:
          word = reinterpret_cast<ptb::content::word::word_t*>(&tcp_data_[pos]);
          // 2. Grab the header (we know what's its size)
          hdr = &(word->wheader);
          pos += hdr->size_bytes;

          // -- For now we are assuming that all payloads are of the same size
          cout << "Word: type " << static_cast<uint32_t>(hdr->word_type)
               << " ts roll " << hdr->ts_rollover << endl;
          switch(hdr->word_type) {
            case ptb::content::word::t_fback:
              cout << "Received a warning!!! This is rare! Do something smart to fix the problem" << endl;
              // advance the pointer by the size of this payload
              pos += ptb::content::word::body_t::size_bytes;
              break;
            case ptb::content::word::t_gt:
              cout << "Received a global trigger. Do something here on how to parse it" << endl;
              // advance the pointer by the size of this payload
              pos += ptb::content::word::body_t::size_bytes;
              break;
            case ptb::content::word::t_lt:
              cout << "Received a low level trigger. Do something here on how to parse it" << endl;
              // advance the pointer by the size of this payload
              pos += ptb::content::word::body_t::size_bytes;
              break;
            case ptb::content::word::t_ts:
              ptb::content::word::payload::timestamp_t *ts = reinterpret_cast<ptb::content::word::payload::timestamp_t *>(&(word->wbody));
              cout << "Received timestamp           " << ts->timestamp() << endl;
              // -- Alternative way is going through the body_t structure
              pld = &(word->wbody);
              ts = reinterpret_cast<ptb::content::word::payload::timestamp_t *>(pld);
              cout << "Received timestamp (xcheck)  " << ts->timestamp() << endl;
              pos += ptb::content::word::body_t::size_bytes;
              break;
          }
        }

      }

    }

    void execute_command(const std::string &cmd) {
      boost::asio::write(*client_socket_, boost::asio::buffer(cmd, cmd.size()));
      //Request an answer
      size_t reply_length = boost::asio::read(*client_socket_,
          boost::asio::buffer(answer_),boost::asio::transfer_at_least(50),*ec_);

      if (*ec_) {
        cout << "Something failed." << endl;
        cout << ec_->message() << endl;

      }
      cout << "Received answer [" << answer_ << "]" << endl;
      answer_ = "";

    }

    void send_reset() {
      cout << "Sending a reset" << endl;
      std::string cmd = "<command>HardReset</command>";
      execute_command(cmd);
      is_running_ = false;
      is_conf_ = false;
      stop_req_ = false;
      quit_req_ = false;
    }

    void send_start() {
      if (!is_conf_) {
        cout << "ERROR: Can't start a run without configuring first" << endl;
      }
      cout << "Sending a start run" << endl;
      std::string cmd = "<command>StartRun</command>";
      execute_command(cmd);
      if (*ec_) {
        cout << "Failed command execution" << endl;
        cout << ec_->message() << endl;

      } else {
        is_running_ = true;
        stop_req_ = false;
        quit_req_ = false;
      }
    }

    void send_stop() {
      stop_req_ = true;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      cout << "Sending a stop run" << endl;
      std::string cmd = "<command>StopRun</command> ";
      execute_command(cmd);
      if (*ec_) {
        cout << "Failed command execution" << endl;
        cout << ec_->message() << endl;

      } else {
        is_running_ = false;
        quit_req_ = false;
      }
    }

    void send_config() {
      cout << "Sending config" << endl;
      if (!is_conf_) {
        cout << "Resetting before configuring" << endl;
        send_reset();
      }
      execute_command(g_config);
      if (*ec_) {
        cout << "Failed command execution" << endl;
        cout << ec_->message() << endl;

      } else {
        is_conf_ = true;
        stop_req_ = false;
        quit_req_ = false;
      }
    }

    void process_quit()
    {
      if (is_running_) {
        // if we are taking data, stop it
        send_stop();

      } else {
        // If we are not, stop execution
        acceptor_->close();

        quit_req_ = true;
      }
    }


    void run() {
      enum commands {init=1,start=2,stop=3,quit=4};
      // Set the stop condition to false to wait for the data
      int command = -1;
      // Now make the receiving thread
      std::cout.setf(std::ios::unitbuf);

      cout << "### Launching reader thread" << endl;
      srv_thread_ = new boost::thread(boost::bind(&ctb_robot::listen_server, this));

      // Now loop for commands
      while(!quit_req_) {
        cout << "### Select command : " << endl;
        cout << " 1 : Init" << endl;
        cout << " 2 : Start Run" << endl;
        cout << " 3 : Stop Run" << endl;
        cout << " 4 : Quit" << endl;

        cin >> command;
        switch(command) {
          case init:
            send_config();
            break;
          case start:
            send_start();
            break;
          case stop:
            send_stop();
            break;
          case quit:
            process_quit();
            break;
          default:
            cout << "Wrong option (" << command << ")." << endl;
            break;
        }
      }
      cout << "### Stop requested." << endl;
      // Wait for the reading function to return

    }


    void listen_server() {

      // Important to avoid mangling of the output due to race conditions to the
      // IO buffers. The trick is to use unbuffered (ie. direct) output
      std::cout.setf(std::ios::unitbuf);
      cout << "Establishing listen server!!!" << endl;

      acceptor_ = boost::make_shared<boost::asio::ip::tcp::acceptor>(*io_service_, tcp::endpoint(tcp::v4(), listen_port_));
      socket_ptr sock(new tcp::socket(*io_service_));
      while (!quit_req_) {
        cout << "listen_server starting acceptor" << endl;
        acceptor_->async_accept(*sock, boost::bind(&ctb_robot::receive_data, this, sock));
        io_service_->run();
        cout << "listen_server finished iteration" << endl;

      }
      cout << "listen_server terminating" << endl;
    }

    void receive_data(socket_ptr sock) {

      // Important to avoid mangling of the output due to race conditions to the
      // IO buffers. The trick is to use unbuffered (ie. direct) output
      cout << "Working on a receiving thread!!!" << endl;

      // Create a server that is meant to receive the data from the
      // PTB and keep dumping the contents into somewhere...like a binary file
      try{
        // Create a tcp server and listen to the connection
        // Some aux variables
        uint16_t bytes_collected= 0;
        // Content structures from the C/PTB
        ptb::content::tcp_header_t *header;
        ptb::content::word::header_t *hdr;
        ptb::content::word::body_t *pld;
        ptb::content::word::word_t*word;

        const uint32_t max_length = 4096;
        uint8_t tcp_data[4096];
        size_t count = 0;
        bool timeout_reached = false;
        size_t tcp_body_size = 0;

        while (!stop_req_) {

          if (stop_req_) {
            break;
          }
          // grab a header.
          // We always know exactly how many bytes we want to receive, so we can use the blocking methods
          // from boost
          bytes_collected = boost::asio::read(*sock,boost::asio::buffer(tcp_data, header->size_bytes),boost::asio::transfer_exactly(header->size_bytes),*ec_);
          if (*ec_) {
            if (*ec_ != boost::asio::error::eof) {
              cout << "Got an error reading..." << endl;
              cout << ec_->message() << endl;
            }
            break;
          }

          header = reinterpret_cast<ptb::content::tcp_header_t *>(tcp_data);
          count++;
          if (!(count%10)) cout << "Counting " << count << " packets received..." << endl;
          // check the number of bytes in this packet
          //cout << "Expecting to receive " << header->word.packet_size << " bytes " << endl;
          tcp_body_size = header->packet_size;
          // -- Read the TCP body.
          if (stop_req_) {
            break;
          }
          bytes_collected = boost::asio::read(*sock,boost::asio::buffer(tcp_data, tcp_body_size),boost::asio::transfer_exactly(tcp_body_size),*ec_);
          if (*ec_) {
            if (*ec_ != boost::asio::error::eof) {
              cout << "Got an error reading..." << endl;
              cout << ec_->message() << endl;
            }
            break;
          }
          // cout << "Collected expected bytes. " << endl;
          // for(size_t i = 0; i < bytes_collected; i++) {
          //   printf("%02X ",tcp_data[i]);
          // }
          // printf("\n");
          // -- We now should have the whole sent packet
          uint32_t pos = 0;
          while (pos < tcp_body_size) {
            // 1. grab the frame:
            word = reinterpret_cast<ptb::content::word::word_t*>(&tcp_data[pos]);
            // 2. Grab the header (we know what's its size)
            hdr = &(word->wheader);
            pos += hdr->size_bytes;

            // -- For now we are assuming that all payloads are of the same size
            cout << "Word: type " << static_cast<uint32_t>(hdr->word_type)
	               << " ts roll " << hdr->ts_rollover << endl;
            switch(hdr->word_type) {
              case ptb::content::word::t_fback:
                cout << "Received a warning!!! This is rare! Do something smart to fix the problem" << endl;
                // advance the pointer by the size of this payload
                pos += ptb::content::word::body_t::size_bytes;
                break;
              case ptb::content::word::t_gt:
                cout << "Received a global trigger. Do something here on how to parse it" << endl;
                // advance the pointer by the size of this payload
                pos += ptb::content::word::body_t::size_bytes;
                break;
              case ptb::content::word::t_lt:
                cout << "Received a low level trigger. Do something here on how to parse it" << endl;
                // advance the pointer by the size of this payload
                pos += ptb::content::word::body_t::size_bytes;
                break;
              case ptb::content::word::t_ts:
                ptb::content::word::payload::timestamp_t *ts = reinterpret_cast<ptb::content::word::payload::timestamp_t *>(&(word->wbody));
                cout << "Received timestamp           " << ts->timestamp() << endl;
                // -- Alternative way is going through the body_t structure
                pld = &(word->wbody);
                ts = reinterpret_cast<ptb::content::word::payload::timestamp_t *>(pld);
                cout << "Received timestamp (xcheck)  " << ts->timestamp() << endl;
                pos += ptb::content::word::body_t::size_bytes;
                break;
            }
          }
        }
        // -- Don't hang in here. Disconnect the thing
      }
      catch(std::exception &e) {
        cout << "STD exception caught in inner thread: " << e.what() << endl;
      }
      catch(...) {
        cerr << "Something went wrong in inner thread. " << endl;
      }

      cout << "receive_data terminating" << endl;
    }


  private:

    enum DeadlineIoObject { None, Acceptor, DataSocket };

    enum ReceiveState {TcpHeader,TcpBody};


    void set_deadline(DeadlineIoObject io_object, unsigned int timeout_usecs)
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

    void check_deadline(void) {
      if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {

      }
      // Put the deadline actor back to sleep if receiver is still running
      if (run_receiver_.load())
      {
        deadline_.async_wait(boost::bind(&ptb_robot::check_deadline, this));
      }

    }

    /// -- Ethernet connection management variables
    boost::asio::io_service io_service_;
    tcp::acceptor           acceptor_;
    tcp::socket         accept_socket_;
    tcp::socket   data_socket_;

    // -- my own variable
    boost::system::error_code ec_;

    // -- timer to find lost connections
    boost::asio::deadline_timer deadline_;
    DeadlineIoObject deadline_io_object_;


    // -- a couple of atomics
    std::atomic<bool> run_receiver_;
    std::atomic<bool> suspend_readout_;
    std::atomic<bool> readout_suspended_;

    int recv_socket_;

    std::unique_ptr<std::thread> receiver_thread_;
    std::size_t      state_nbytes_recvd_;

    // Content structures from the C/PTB
    ptb::content::tcp_header_t *tcp_header;
    ptb::content::word::header_t *hdr;
    ptb::content::word::body_t *pld;
    ptb::content::word::word_t*word;

    const uint32_t max_length_ = 4096;
    uint8_t tcp_data_[4096];
    size_t count_ = 0;
    size_t tcp_body_size_ = 0;
//    size_t next_receive_size = sizeof(header->size_bytes);
    size_t next_receive_size_;// = sizeof(ctb::content::tcp_header_t::size_bytes);

    ReceiveState current_receive_state_;
    //    boost::shared_ptr<boost::asio::io_service> io_service_;
    //    boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    //    boost::shared_ptr<tcp::socket> client_socket_;
    //    boost::shared_ptr<boost::system::error_code> ec_;

    // Server thread
    boost::thread* srv_thread_;

    uint16_t listen_port_;

    // control variables
    bool stop_req_;
    bool quit_req_;
    bool is_running_;
    bool is_conf_;
    std::thread::id receiver_id_;
    // Aux vars
    std::string answer_;
};

int main() {
  std::cout.setf(std::ios::unitbuf);

  ctb_robot robot("128.91.41.238",8991);
  robot.run();

  return 0;
}

