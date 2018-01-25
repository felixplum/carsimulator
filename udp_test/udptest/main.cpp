#include <iostream>
#include <boost/asio.hpp>
#include <udp_server.h>
#include <udp_client.h>
#include <boost/thread/thread.hpp>

//void Run() {
//  UDPClient client;
//  client.ToogleCarUpdate(true);
//  std::cout << "entering run()" << std::endl;
//  while (true) {
//  }
//  std::cout << "leaving run()" << std::endl;
//}

void RunServer() {
  UDPServer server;
  std::vector<float> state(4, 0);
  float t = 0.;
  float dt = 0.1;
  while(1) {
    boost::this_thread::sleep_for(
            boost::chrono::milliseconds(static_cast<int>(dt*1e3)));
    state[0] = fmod((t*0.3), 3.); // 0..3 0..3 ...
    state[1] = fmod(0.666*state[0], 2.);
    state[2] = 30.*3.141/180.;
    state[3] = t;
    t += dt;
    server.SendState(state);
//    std::cout << state[0]  << std::endl;
  }
}


int main()
{
//   boost::thread t(&Run);
//  UDPServer server;
//  std::vector<float> send_buff = {1, 2, 3, 4};
  boost::thread t(&RunServer);
  std::string tmp;
  while (getline(std::cin, tmp))
  {
      if (tmp.empty())
        break;
        //server.SendState(send_buff);
  }
  return 0;
}
