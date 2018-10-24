//-- Boost network libraries : to define the TCP request server
#include <ctime>
#include <iostream>
#include <string>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <csignal>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap
#include <unistd.h> //close
#include <signal.h> 

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <ctime>

// -- json
#include <json.hpp>

// GPIO 0
#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8
// Config memory mapped regs
#define CONFIG_BASEADDR 0x43C00000
#define CONFIG_HIGHADDR 0x43C0FFFF
#define CONFIG_CH0_OFFSET 0x0
#define CONFIG_CH1_OFFSET 0x4

using std::cout;
using std::endl;
using boost::asio::ip::tcp;
using json = nlohmann::json;

const int max_length = 1024;

// -- External stuff that should be built at link time

extern int g_mem_fd;

// -- Map a physical address into a virtual one
extern void *map_phys_mem(uint32_t base_addr, uint32_t high_addr);
extern void release_mem();
extern void *unmap_phys_mem(void * address, size_t size);
extern uint32_t read_reg32(uint32_t addr);
extern uint32_t write_reg32(uint32_t addr,uint32_t val);




// -- Request processing logic

bool keep_running_;
void *mmapaddr_timing;
void *mmapaddr_config;

typedef boost::shared_ptr<tcp::socket> socket_ptr;


void session(socket_ptr sock);
void server(boost::asio::io_service& io_service, unsigned short port);
void process_request(const json &req, json &answer);
void read_register(json &answer,const uint32_t reg);
void read_timing_status(json &answer);
void reset_timing_status(json &answer, bool force);







// -- represent a single blocking session
void session(socket_ptr sock)
{
  try
  {
    unsigned int n_bck = 0;
    char data[max_length];
    std::string localBuffer;
    localBuffer = "";
    int brckt_count  = 0;
    bool first_brckt = true;
    size_t pos_start, pos_end;
    size_t pos_aux;
    std::string buffer; 

    json msg, answer;

    boost::system::error_code error;
    size_t length = 0;
    
    while ((length = sock->read_some(boost::asio::buffer(data), error)) > 0) {
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      pos_aux = localBuffer.size();
      localBuffer += data;


      for (std::size_t pos = pos_aux; pos < localBuffer.size(); pos++) {
        if (localBuffer.at(pos) == '{') {
          if (first_brckt) {
            first_brckt = false;
            pos_start = pos;
          }
          brckt_count++;
        }
        if (localBuffer.at(pos) == '}') {
          brckt_count--;
          if (brckt_count == 0) {
            pos_end = pos+1;
            // We have a full message -- process it
            buffer = localBuffer.substr(pos_start,pos_end-pos_start);
            if (pos_end == localBuffer.length()) {
              localBuffer.clear();
            } else {
              localBuffer = localBuffer.substr(pos_end);
            }
            pos = 0;
            pos_start = 0;
            first_brckt = true;
            msg = json::parse(buffer);
            process_request(msg,answer);
          }
        }
      }
    }
    if (error == boost::asio::error::eof)
      //break; // Connection closed cleanly by peer.
      cout << "EOF received while taking data" << std::endl;
    else if (error)
      throw boost::system::system_error(error); // Some other error.

/**

    // FIXME: keep receiving until there is a full request
    while (n_bck != 0) {
      length = sock->read_some(boost::asio::buffer(data), error);
      localBuffer += data;
      pos_aux
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.
    }
    // FIXME: Implement the processing of the request here
    json req;// = json::parse();
    json answer;
    process_request(req,answer);
    std::string asw = answer.dump();
    // -- Write the answer
    boost::asio::write(*sock, boost::asio::buffer(asw.c_str(), asw.length()));
**/
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception in thread: " << e.what() << "\n";
  }
}

// -- represents a simple TCP server
void server(boost::asio::io_service& io_service, unsigned short port)
{
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  {
    socket_ptr sock(new tcp::socket(io_service));
    a.accept(*sock);
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::cout << "Received a new connection from " << sock->remote_endpoint().address().to_string()
    << " at port " << sock->remote_endpoint().port() 
    << " on " << std::ctime(&now_time) << std::endl;
    boost::thread t(boost::bind(session, sock));
  }
}

void process_request(const json &req, json &answer)
{
  if (req.at("command") == std::string("check_registers")) {
    ;; // do nothing for now
  } else if (req.at("command") == std::string("force_reset_timing")) {
    reset_timing_status(answer,true);
  } else if (req.at("command") == std::string("reset_timing")) {
    reset_timing_status(answer,false);
  } else if (req.at("command") == std::string("check_timing")) {
    read_timing_status(answer);
  } else {
    answer["status"] = "ERROR";
    answer["message"] = "Unknown command [" + req.at("command").get<std::string>() + "]";
  }

}

void read_register(json &answer,const uint32_t reg) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::cout << "Checking status of register " << reg << " on " << std::ctime(&now_time) << std::endl;
  
  void* mapped_addr = map_phys_mem(CONFIG_BASEADDR,CONFIG_HIGHADDR);
  if (mapped_addr == NULL) {
    answer["status"] = "ERROR";
    answer["message"] = "Failed to map memory to register. Contact CTB expert.";
    release_mem();
  }
  printf("Received pointer to Config Regs at %p\n", mapped_addr);  

  uint32_t config = read_reg32((uint32_t)((uint32_t)mapped_addr + (CONFIG_CH1_OFFSET * reg)));
  answer["status"] = "OK";
  answer["message"] = config;
  
  std::cout << "Status of register " << reg << " on " << std::ctime(&now_time) 
  << " : " << config << std::endl;

  release_mem();
}


void read_timing_status(json &answer) {
  read_register(answer,91);
}


void reset_timing_status(json &answer, bool force) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::cout << "Resetting timing state on " << std::ctime(&now_time) << std::endl;

  // -- first check if the board is (or thinks it is ) running
  json dummy;
  uint32_t val;
  read_register(dummy,0);
  if (dummy.at("status") == "OK") {
    if ((val & (0x1 << 31)) && !force) {
        // -- a run is ongoing. Fail
        answer["status"] = "ERROR";
        answer["message"] = "Trying to reset timing while CTB is taking data. If you really want to do this use the 'force' command";
    } else {
      if (force) {
        answer["extra"] = "CTB was taking data but force requested. Resetting anyway.";
      }     
      // -- no run ongoing. Reset the endpoint

      void * mmap_addr = map_phys_mem(GPIO_BASEADDR,GPIO_HIGHADDR);
      uint32_t reg_val = 0x20;
      write_reg32((uint32_t)mmap_addr + GPIO_CH0_OFFSET, reg_val);
      // -- sleep for a millisecond
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      reg_val = 0x0;
      write_reg32((uint32_t)mmap_addr + GPIO_CH0_OFFSET, reg_val);
      // -- Now check the endpoint status
      read_timing_status(answer);
      auto now = std::chrono::system_clock::now();
      std::time_t now_time = std::chrono::system_clock::to_time_t(now);
      if (answer.at("status") == "OK") {
        std::cout << "Timing status [" << std::hex << answer.at("message") << std::dec << "] after reset  on " << std::ctime(&now_time) << std::endl;
      } else {
        std::cout << "Failed to read timing status [" << std::hex << answer.at("message") << std::dec << "] after reset  on " << std::ctime(&now_time) << std::endl;        
      }
    }
  } else {
    // failed even to check register
    answer = dummy;
  }

}

int main() {
  keep_running_ = true;
  try
  {
    boost::asio::io_service io_service;
    server(io_service, 9000);

    // boost::asio::io_service io_service;
    // // listen on port 9000
    // tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 9000));
    // while (keep_running_)
    // {
    //   tcp::socket socket(io_service);
    //   acceptor.accept(socket);
    //   // at this point there is a client request
    //   process_request(socket);
    // }
  } 
  catch (std::exception& e)
  {
    std::cerr << "Caught exception in timing control server : " <<  e.what() << std::endl;
  }
  return 0;
}



