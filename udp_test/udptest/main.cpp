#include <iostream>
#include <boost/asio.hpp>
#include <udp_server.h>
#include <udp_client.h>
#include <boost/thread/thread.hpp>

void Run() {
  UDPClient client;
  client.ToogleCarUpdate(true);
  std::cout << "entering run()" << std::endl;
  while (true) {
  }
  std::cout << "leaving run()" << std::endl;
}


int main()
{
   boost::thread t(&Run);
  UDPServer server;
  std::vector<float> send_buff = {1, 2, 3, 4};

  std::string tmp;
  while (getline(std::cin, tmp))
  {
      if (tmp.empty())
        break;
        server.SendState(send_buff);
  }
  return 0;
}
