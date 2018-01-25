#include "udp_server.h"


UDPServer::UDPServer() :
  port_number_(1122),
  socket_(io_, ba::ip::udp::v4()),
  endpoint_client_(ba::ip::address::from_string("127.0.0.1"), port_number_)
{
  socket_.set_option(boost::asio::socket_base::reuse_address(true));
  socket_.bind(ba::ip::udp::endpoint(ba::ip::udp::v4(), port_number_));
}

void UDPServer::SendState(const std::vector<float>& state) {
//  socket_.send_to(ba::buffer((char*)&state.front(), state.size()*4), endpoint_client_);
  socket_.send_to(ba::buffer(state), endpoint_client_);
}
