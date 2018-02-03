#include "udp_server.h"


UDPServer::UDPServer() :
  port_number_(9931),
  socket_(io_, ba::ip::udp::v4()),
  endpoint_client_(ba::ip::address::from_string("127.0.0.1"), port_number_)
{
  socket_.set_option(boost::asio::socket_base::reuse_address(true));
  socket_.bind(ba::ip::udp::endpoint(ba::ip::udp::v4(), port_number_));
}

//void UDPServer::SendHandler(const boost::system::error_code &error,
//                            size_t bytes_transferred) {

//}

void UDPServer::SendState(const std::vector<float>& state) {
//  socket_.async_send_to(ba::buffer(state), endpoint_client_,
//                        boost::bind(&UDPServer::SendHandler, this,
//                        ba::placeholders::error,
//                        ba::placeholders::bytes_transferred));
  socket_.send_to(ba::buffer(state), endpoint_client_);
}
