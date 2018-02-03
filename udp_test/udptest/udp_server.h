#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <iostream>
#include <exception>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace ba = boost::asio;

class UDPServer
{
public:
  UDPServer();
  void SendState(const std::vector<float> &state);
  void SendHandler(const boost::system::error_code &error,
                   size_t bytes_transferred);
private:
  int port_number_;
  ba::io_service io_;
  ba::ip::udp::socket socket_;
  ba::ip::udp::endpoint endpoint_client_;

};

#endif // UDP_SERVER_H
