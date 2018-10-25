//-- Boost network libraries : to define the TCP request server
#include <ctime>
#include <iostream>
#include <string>
#include <cstdio>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <thread>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

// -- json
#include <json.hpp>

// -- Memory definitions for the CTB
// FIXME: These have to be updated as the design changes

// GPIO 0
#define GPIO_BASEADDR 0x41200000
#define GPIO_HIGHADDR 0x4120FFFF
#define GPIO_CH0_OFFSET 0x0
#define GPIO_CH1_OFFSET 0x8
// Config memory mapped regs
#define CONFIG_BASEADDR 0x43C00000
#define CONFIG_HIGHADDR 0x43C0FFFF
#define CONFIG_CH_OFFSET 0x4

using std::cout;
using std::endl;
using boost::asio::ip::tcp;
using json = nlohmann::json;

const int max_length = 1024;

// -- External stuff that should be built at link time

// -- Map a physical address into a virtual one
extern void *map_phys_mem(uint32_t base_addr, uint32_t high_addr);
extern void release_mem();
extern uint32_t read_reg32(uint32_t addr);
extern uint32_t write_reg32(uint32_t addr,uint32_t val);




// -- Request processing logic

bool keep_running_;
void *mmapaddr_timing;
void *mmapaddr_config;

typedef boost::shared_ptr<tcp::socket> socket_ptr;


void session(tcp::socket &sock);
void server(boost::asio::io_service& io_service, unsigned short port);
void process_request(const json &req, json &answer);
void read_register(json &answer,const uint32_t reg);
void read_timing_status(json &answer);
void reset_timing_status(json &answer, bool force);
void dump_registers(json &answer);






// -- represent a single blocking session
void session(tcp::socket &sock)
{
  try
  {
//    unsigned int n_bck = 0;
//    char data[max_length];
//    std::string localBuffer;
//    localBuffer = "";
//    int brckt_count  = 0;
//    bool first_brckt = true;
//    size_t pos_start, pos_end;
//    size_t pos_aux = 0;
//    std::string buffer;

    json msg, answer;
    bool close_socket = false;
    boost::system::error_code error;
//    size_t length = 0;
    
    boost::asio::streambuf tcpmsg;
    size_t nbytes = boost::asio::read_until(sock, tcpmsg, '}', error);
    if (error)
    {
      printf("Failed to receive with error %u : %s\n",error.value(),error.message().c_str());
    } else {
      printf("Received %u bytes\n",nbytes);
    }
    std::istream tcp_stream(&tcpmsg);
    std::string req;
    std::getline(tcp_stream,req,'}');
    if (req.size())
    {
      req += "}";
      msg = json::parse(req);
      process_request(msg,answer);
      printf("Process requested. Answer : %s\n",answer.dump().c_str());
    } else {
      printf("Failed to receive the message for the request. Please try again.\n");
      answer["status"] = "ERROR";
      answer["message"] = "Lost connection while processing request. Please retry.";
    }

    // -- answer the request
    std::string tcp_resp = answer.dump();
    boost::asio::write(sock,boost::asio::buffer(tcp_resp.c_str(),tcp_resp.size()),error);
    if (error)
    {
      printf("Failed with error %u : %s\n",error.value(),error.message().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    //sock->send(tcpmsg.c_str(),tcpmsg.size());
    //sock->send(boost::asio::buffer(tcpmsg.c_str(),tcpmsg.size()));
    //}
    //}
    //}
    //    }
    //    if (error == boost::asio::error::eof)
    //    {
    //      //break; // Connection closed cleanly by peer.
    //      cout << "EOF received while processing" << std::endl;
    //      sock->close();
    //    }
    //    else if (error) {
    //      printf("Failed with error %u : %s\n",error.value(),error.message().c_str());
    //      sock->close();
    //    }
  }
  catch(json::exception &e)
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Caught a JSON exception. Message : %s\n",std::ctime(&now_time),e.what());
    throw;
  }
  catch (std::exception& e)
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Caught STD exception in thread. Message : %s\n",std::ctime(&now_time),e.what());
    throw;
  }
  catch(...)
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Caught an unexpected exception\n",std::ctime(&now_time));
    throw;
  }
}

// -- represents a simple TCP server
void server(boost::asio::io_service& io_service, unsigned short port)
{
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  {
      printf("Creating a new listering socket.\n");
      boost::system::error_code error;
      tcp::socket sock(io_service);
      //socket_ptr sock(new tcp::socket(io_service));
      a.accept(sock);
      auto now = std::chrono::system_clock::now();
      std::time_t now_time = std::chrono::system_clock::to_time_t(now);
      printf("%s : Received a new connection from %s : %hu\n",std::ctime(&now_time),
    	sock.remote_endpoint().address().to_string().c_str(),
			sock.remote_endpoint().port());
    // I don't need another thread
      //boost::thread t(boost::bind(session, sock));
    try {
      session(sock);
      std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }
    catch(...)
    {
      printf("Caught a failure at top level");
    }
    sock.close(error);
  }
}

void process_request(const json &req, json &answer)
{
  std::string comm = req.at("command").get<std::string>();
	auto now = std::chrono::system_clock::now();
	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	printf("%s : Processing request with command : %s\n",std::ctime(&now_time), comm.c_str());

  if ( comm == std::string("check_registers")) {
    printf("%s : Checking registers\n",std::ctime(&now_time));
    dump_registers(answer);
  } else if (comm  == std::string("force_reset_timing")) {
    printf("%s : Resetting timing endpoint with force option\n",std::ctime(&now_time));
    reset_timing_status(answer,true);
  } else if (comm  == std::string("reset_timing")) {
    printf("%s : Resetting timing endpoint without force option\n",std::ctime(&now_time));
    reset_timing_status(answer,false);
  } else if (comm  == std::string("check_timing")) {
    printf("%s : Checking status of timing endpoint \n",std::ctime(&now_time));
    read_timing_status(answer);
  } else {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Unknown command : %s\n",std::ctime(&now_time), comm.c_str());
    answer["status"] = "ERROR";
    answer["message"] = "Unknown command [" + req.at("command").get<std::string>() + "]";
  }

}

void read_register(json &answer,const uint32_t reg) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  printf("%s : Checking status of register %u\n",std::ctime(&now_time),reg );
  
  void* mapped_addr = map_phys_mem(CONFIG_BASEADDR,CONFIG_HIGHADDR);
  if (mapped_addr == NULL) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Error mapping config register %u\n",std::ctime(&now_time),reg );
    answer["status"] = "ERROR";
    answer["message"] = "Failed to map memory to register. Contact CTB expert.";
    std::ostringstream msg;
    msg << "Register=" << reg << ", baseaddr="<< std::hex << CONFIG_BASEADDR<< std::dec << ", highaddr=" << std::hex << CONFIG_HIGHADDR << std::dec << ", ret="<<mapped_addr;
    answer["extra"] = msg.str();
    release_mem();
    //printf("Returning\n");
    return;
  }
  printf("%s : Received pointer to config reg %u at %p\n",std::ctime(&now_time),reg, mapped_addr);

  uint32_t config = read_reg32((uint32_t)((uint32_t)mapped_addr + (CONFIG_CH_OFFSET * reg)));
  answer["status"] = "OK";
  answer["message"] = config;
  
  printf("%s : Status of register %u : %u [%X]\n",std::ctime(&now_time),reg,config,config);

  release_mem();
}


void read_timing_status(json &answer) {
  read_register(answer,91);
}

void dump_registers(json &answer)
{
  std::map<std::size_t,uint32_t> regs;
  for (std::size_t i = 0; i < 0xFF; i++)
  {
    uint32_t tmp_val;
    read_register(answer,i);
    if (answer.at("status") == "OK")
    {
      regs[i] = answer.at("message").get<uint32_t>();
    } else {
      return;
    }
  }
  answer["message"] = json(regs);
}

void reset_timing_status(json &answer, bool force) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  printf("%s : Resetting timing state\n",std::ctime(&now_time));

  // -- first check if the board is (or thinks it is ) running
  json dummy;
  uint32_t val;
  read_register(dummy,0);
  if (dummy.at("status") == "OK") {
    if ((val & (0x1 << 31)) && !force) {
      // -- a run is ongoing. Fail
      auto now = std::chrono::system_clock::now();
      std::time_t now_time = std::chrono::system_clock::to_time_t(now);
      printf("%s : Refusing to reset timing while CTB is taking data. The Force is weak on this request.",std::ctime(&now_time));
      answer["status"] = "ERROR";
      answer["message"] = "Trying to reset timing while CTB is taking data. If you really want to do this use the 'force' command";
    } else {
      if (force) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        printf("%s : Forcing to reset timing while CTB is taking data. The Force is strong on this request.",std::ctime(&now_time));
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
        printf("%s : Timing status register after reset : [%X].\n",
               std::ctime(&now_time),
               answer.at("message").get<uint32_t>());
      } else {
        printf("%s : Failed to read timing status after reset. Return message : %s\n",std::ctime(&now_time),answer.at("message").get<std::string>().c_str());
      }
    }
  } else {
    // failed even to check register
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Failed to read CTB status. Message : %s\n",std::ctime(&now_time),dummy.at("message").get<std::string>().c_str());
    std::string msg = "Failed to check running status of the CTB. ";
    msg += dummy.at("message").get<std::string>();
    answer["message"] = msg;
    answer["status"] = dummy.at("status");
    answer["extra"] = dummy.at("extra");
  }

}

int main() {
  keep_running_ = true;
  try
  {
    boost::asio::io_service io_service;
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Initiating a reading server on port 8995\n",std::ctime(&now_time));

    server(io_service, 8995);

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
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Caught exception in timing control server : %s\n",std::ctime(&now_time),e.what());
  }
  return 0;
}



