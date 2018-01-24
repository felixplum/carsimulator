#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <iostream>
#include <exception>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace ba = boost::asio;

class UDPClient
{
public:
  UDPClient();
  void ToogleCarUpdate(bool is_active);
private:
  void Run();
  void HandleReceive(const boost::system::error_code &error,
                     size_t bytes_transferred);
  bool is_listening_;
  int port_number_;
  ba::io_service io_;
  ba::ip::udp::socket socket_;
  ba::ip::udp::endpoint endpoint_server_;
  size_t n_receive_size_;
  std::vector<float> receive_buffer_;
};

#endif // UDP_CLIENT_H
