/*
 * ctbdatareceiver.hh
 *
 *  Created on: May 4, 2018
 *      Author: nbarros
 */

#ifndef TEST_STANDALONE_CTBDATARECEIVER_HH_
#define TEST_STANDALONE_CTBDATARECEIVER_HH_


#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>

using boost::asio::ip::tcp;

class ctb_data_receiver {
  public:
    //ctb_data_receiver();
    ctb_data_receiver(int debug_level, uint32_t tick_period_usecs, uint16_t receive_port);

    virtual ~ctb_data_receiver();

    void start(void);
    void stop(void);

    bool exception() const { return exception_.load(); }   // GBcopy

    void set_stop_delay(uint32_t delay_us) {sleep_on_stop_ = delay_us;};
    uint32_t get_stop_delay() {return sleep_on_stop_;};

  private:
    enum DeadlineIoObject { None, Acceptor, DataSocket };

    void run_service(void);
    void do_accept(void);
    void do_read(void);
    void handle_received_data(std::size_t length);
    void suspend_readout(bool await_restart);

    void set_deadline(DeadlineIoObject io_object, unsigned int timeout_us);
    void check_deadline(void);
    void set_exception( bool exception ) { exception_.store( exception ); }  // GBcopy

    bool process_payload(void);

    int debug_level_;

    /// -- Ethernet connection management variables
    boost::asio::io_service io_service_;
    tcp::acceptor           acceptor_;
    tcp::socket             accept_socket_;
    tcp::socket             data_socket_;

    boost::asio::deadline_timer deadline_;
    DeadlineIoObject            deadline_io_object_;
    uint32_t                    tick_period_usecs_;

    // Port that should be listened for connection from the PTB (conf param)
    uint16_t receive_port_;

    std::atomic<bool> run_receiver_;
    std::atomic<bool> suspend_readout_;
    std::atomic<bool> readout_suspended_;
    std::atomic<bool> exception_;    // GBcopy

    int recv_socket_;

    std::unique_ptr<std::thread> receiver_thread_;

    std::size_t      state_nbytes_recvd_;

    std::size_t      total_bytes_recvd_;
    std::size_t      total_packets_recvd_;


    enum NextReceiveState { ReceiveTCPHeader, ReceiveTCPPayload };
    NextReceiveState next_receive_state_;
    size_t           next_receive_size_;

//    size_t           millislice_size_recvd_;
//    uint32_t         microslices_recvd_;
//    uint32_t         payloads_recvd_;
//    uint32_t         payloads_recvd_counter_;
//    uint32_t         payloads_recvd_trigger_;
//    uint32_t         payloads_recvd_timestamp_;
//    uint32_t         payloads_recvd_warning_;
//    uint32_t         payloads_recvd_checksum_;
//    size_t           microslice_size_recvd_;
//    uint32_t         millislices_recvd_;


    std::chrono::high_resolution_clock::time_point start_time_;

    uint32_t sleep_on_stop_; // time (us) to sleep before stopping


    void* current_write_ptr_;
    // Keep a buffer of 4MB
    uint8_t raw_buffer_[4194304];
    bool sequence_id_initialised_;
    uint8_t last_sequence_id_;

    size_t n_counter_words_;
    size_t n_llt_words_;
    size_t n_hlt_words_;
    size_t n_warn_words_;
    size_t n_ts_words_;
    size_t n_words_;

};

#endif /* TEST_STANDALONE_CTBDATARECEIVER_HH_ */
