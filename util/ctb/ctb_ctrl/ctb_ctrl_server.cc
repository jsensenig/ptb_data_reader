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
#include "json.hpp"

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

using boost::asio::ip::tcp;
//using json = nlohmann::json;
using nlohmann::json;

//const int max_length = 1024;

// -- External stuff that should be built at link time

// -- Map a physical address into a virtual one
extern int g_mem_fd;
extern void *map_phys_mem(uint32_t base_addr, uint32_t high_addr);
extern void unmap_phys_mem(void * address, uint32_t base_addr, uint32_t high_addr);
extern void release_mem();
extern uint32_t read_reg32(uint32_t addr);
extern uint32_t write_reg32(uint32_t addr,uint32_t val);


typedef boost::shared_ptr<tcp::socket> socket_ptr;

// -- Forward declarations
void session(tcp::socket &sock);
void server(boost::asio::io_service& io_service, unsigned short port);
void process_request(const json &req, json &answer);
void read_register(json &answer,const uint32_t reg);
void read_timing_status(json &answer);
void reset_timing_status(json &answer, bool force);
void dump_registers(json &answer);

// -- global variable for timestamps
char g_time[0xFF];

char* mtime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  //time_t t = time(NULL);
  struct tm * p = localtime(&tv.tv_sec);
  strftime(g_time, 128, "%c", p);
  sprintf(g_time,"%s.%ld",g_time,tv.tv_usec);
  return g_time;
}


// -- represent a single blocking session
void session(tcp::socket &sock)
{
  try
  {

    json msg, answer;
    boost::system::error_code error;

    boost::asio::streambuf tcpmsg;
    size_t nbytes = boost::asio::read_until(sock, tcpmsg, '}', error);
    if (error)
    {
      printf("%s : Failed to receive with error %u : %s\n",mtime(),error.value(),error.message().c_str());
    } else {
      printf("%s : Received %u bytes\n",mtime(),nbytes);
    }
    std::istream tcp_stream(&tcpmsg);
    std::string req;
    std::getline(tcp_stream,req,'}');
    if (req.size())
    {
      req += "}";
      msg = json::parse(req);
      process_request(msg,answer);
      printf("%s : Process requested. Answer : %s\n",mtime(),answer.dump().c_str());
    } else {
      printf("%s : Failed to receive the message for the request. Please try again.\n",mtime());
      answer["status"] = "ERROR";
      answer["message"] = "Lost connection while processing request. Please retry.";
    }

    // -- answer the request
    std::string tcp_resp = answer.dump();
    boost::asio::write(sock,boost::asio::buffer(tcp_resp.c_str(),tcp_resp.size()),error);
    if (error)
    {
      printf("%s : Failed with error %u : %s\n",mtime(),error.value(),error.message().c_str());
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(700));

  }
  catch(json::exception &e)
  {
    printf("%s : Caught a JSON exception. Message : %s\n",mtime(),e.what());
    throw;
  }
  catch (std::exception& e)
  {
    printf("%s : Caught STD exception in thread. Message : %s\n",mtime(),e.what());
    throw;
  }
  catch(...)
  {
    printf("%s : Caught an unexpected exception\n",mtime());
    throw;
  }
}

// -- represents a simple TCP server
void server(boost::asio::io_service& io_service, unsigned short port)
{
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  {
    printf("\n\n%s : Creating a new listening socket.\n",mtime());
    boost::system::error_code error;
    tcp::socket sock(io_service);
    //socket_ptr sock(new tcp::socket(io_service));
    a.accept(sock);
    printf("%s : Received a new connection from %s : %hu\n",mtime(),
           sock.remote_endpoint().address().to_string().c_str(),
           sock.remote_endpoint().port());
    // I don't need another thread
    try {
      session(sock);
      //std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }
    catch(...)
    {
      printf("Caught a failure at top level");
    }
    //sock.close(error);
  }
}

void process_request(const json &req, json &answer)
{
  std::string comm = req.at("command").get<std::string>();
  printf("%s : Processing request with command : %s\n",mtime(), comm.c_str());

  if ( comm == std::string("check_registers")) {
    printf("%s : Checking registers\n",mtime());
    dump_registers(answer);
  } else if (comm  == std::string("force_reset_timing")) {
    printf("%s : Resetting timing endpoint with force option\n",mtime());
    reset_timing_status(answer,true);
  } else if (comm  == std::string("reset_timing")) {
    printf("%s : Resetting timing endpoint without force option\n",mtime());
    reset_timing_status(answer,false);
  } else if (comm  == std::string("check_timing")) {
    printf("%s : Checking status of timing endpoint \n",mtime());
    read_timing_status(answer);
  } else {
    printf("%s : Unknown command : %s\n",mtime(), comm.c_str());
    answer["status"] = "ERROR";
    answer["message"] = "Unknown command [" + req.at("command").get<std::string>() + "]";
  }
}

void read_register(json &answer,const uint32_t reg) {
  printf("%s : Checking status of register %u\n",mtime(),reg );

  void* mapped_addr = NULL;
  mapped_addr = map_phys_mem(CONFIG_BASEADDR,CONFIG_HIGHADDR);
  if (mapped_addr == NULL) {
    printf("%s : Error mapping config register %u\n",mtime(),reg );
    answer["status"] = "ERROR";
    answer["message"] = "Failed to map memory to register. Contact CTB expert.";
    std::ostringstream msg;
    msg << "Register=" << reg << ", baseaddr="<< std::hex << CONFIG_BASEADDR<< std::dec << ", highaddr=" << std::hex << CONFIG_HIGHADDR << std::dec << ", ret="<<mapped_addr;
    answer["extra"] = msg.str();
  } else {
    printf("%s : Received pointer to config reg %u at %p\n",mtime(),reg, mapped_addr);

    uint32_t config = read_reg32((uint32_t)((uint32_t)mapped_addr + (CONFIG_CH_OFFSET * reg)));
    answer["status"] = "OK";
    answer["message"] = config;

    printf("%s : Status of register %u : %u [%X]\n",mtime(),reg,config,config);

  }
  if (mapped_addr)
  {
    unmap_phys_mem(mapped_addr,CONFIG_BASEADDR,CONFIG_HIGHADDR);
  }

}


void read_timing_status(json &answer) {
  read_register(answer,91);
}

void dump_registers(json &answer)
{
  std::vector<uint32_t> regs(0xFF,0x0);
  for (std::size_t i = 0; i < 0xFF; i++)
  {
    read_register(answer,i);
    if (answer.at("status") == "OK")
    {
      regs[i] = answer.at("message").get<uint32_t>();
    } else {
      // an error was caught
      // Return with that error
      return;
    }
  }
  answer["message"] = json(regs);
}

void reset_timing_status(json &answer, bool force) {
  printf("%s : Resetting timing state\n",mtime());

  // -- first check if the board is (or thinks it is ) running
  json dummy;
  read_register(dummy,0);
  uint32_t tmp_val = 0x0;
  if (dummy.at("status") == "OK") {
    tmp_val = dummy.at("message").get<uint32_t>();
    if ((tmp_val & (0x1 << 31)) && (!force)) {
      // -- a run is ongoing. Fail
      printf("%s : Refusing to reset timing while CTB is taking data. The Force is weak on this request.",mtime());
      answer["status"] = "ERROR";
      answer["message"] = "Trying to reset timing while CTB is taking data. If you really want to do this use the 'force' command";
      return;
      // -- no run ongoing. Reset the endpoint
    } else {
      if (force)
      {
        printf("%s : Forcing to reset timing while CTB is taking data. The Force is strong on this request.",mtime());
        answer["extra"] = "Forcing reset regardless of CTB taking data.";
      }
      // -- check the status of the timing.
      json tmpj;
      read_timing_status(tmpj);
      if (tmpj.at("status") == "OK")
      {
        uint32_t state = tmpj.at("message").get<uint32_t>();
        if ((state >> 28) == 0x8)
        {
          std::string nmsg = "";
          if (answer.find("extra") != answer.end())
          {
            nmsg = answer.at("extra").get<std::string>();
          }
          nmsg += " The CTB timing endpoint is already in a good state (0x8).";
        }
      }
      void * mmap_addr = NULL;
      mmap_addr = map_phys_mem(GPIO_BASEADDR,GPIO_HIGHADDR);
      uint32_t reg_val = 0x20;
      write_reg32((uint32_t)mmap_addr + GPIO_CH0_OFFSET, reg_val);
      // -- sleep for a millisecond
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      reg_val = 0x0;
      write_reg32((uint32_t)mmap_addr + GPIO_CH0_OFFSET, reg_val);
      // -- sleep for a bit to give the endpoint time to initialize
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // -- Now check the endpoint status
      read_timing_status(answer);
      if (answer.at("status") == "OK") {
        printf("%s : Timing status register after reset : [%X].\n",
               mtime(),
               answer.at("message").get<uint32_t>());
      } else {
        printf("%s : Failed to read timing status after reset. Return message : %s\n",mtime(),answer.at("message").get<std::string>().c_str());
      }
      if (mmap_addr)
      {
        unmap_phys_mem(mmap_addr,GPIO_BASEADDR,GPIO_HIGHADDR);
      }

    }
  } else {
    // failed even to check register
    printf("%s : Failed to read CTB status. Message : %s\n",mtime(),dummy.at("message").get<std::string>().c_str());
    std::string msg = "Failed to check running status of the CTB. ";
    msg += dummy.at("message").get<std::string>();
    answer["message"] = msg;
    answer["status"] = dummy.at("status");
    answer["extra"] = dummy.at("extra");
  }
}

int main() {
  g_mem_fd = 0;
  try
  {
    boost::asio::io_service io_service;
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    printf("%s : Initiating a reading server on port 8990\n",std::ctime(&now_time));

    server(io_service, 8990);
  } 
  catch (std::exception& e)
  {
    printf("%s : Caught exception in timing control server : %s\n",mtime(),e.what());
  }
  release_mem();

  return 0;
}



