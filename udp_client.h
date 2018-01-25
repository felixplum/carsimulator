#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <iostream>
#include <exception>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <car.h>

namespace ba = boost::asio;

class UDPClient
{
public:
  UDPClient();
  void ToogleCarUpdate(bool is_active, CarPtr car_ptr = NULL);
private:
  void Run();
  void HandleReceive(const boost::system::error_code &error,
                     size_t bytes_transferred);
  void HandleListeningStatus();
  int port_number_;
  ba::io_service io_;
  ba::ip::udp::socket socket_;
  ba::ip::udp::endpoint endpoint_server_;
  size_t n_receive_size_;
  std::vector<float> receive_buffer_;
  CarPtr car_udp_;
  boost::scoped_ptr<boost::thread> thread_;
};

#endif // UDP_CLIENT_H
