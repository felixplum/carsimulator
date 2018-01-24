#include "udp_client.h"

UDPClient::UDPClient(CarPtr car_ptr, const bool& listen_to_udp) :
  is_listening_(listen_to_udp),
  port_number_(1122),
  socket_(io_, ba::ip::udp::v4()),
  endpoint_server_(ba::ip::address::from_string("127.0.0.1"), port_number_),
  n_receive_size_(4), // 4 float per udp package
  receive_buffer_(n_receive_size_, 0) // write received floats here
{

  socket_.set_option(boost::asio::socket_base::reuse_address(true));
  socket_.bind(ba::ip::udp::endpoint(ba::ip::udp::v4(), port_number_));
  car_udp_ = car_ptr;
  // start listening:
  socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                             boost::bind(&UDPClient::HandleReceive, this,
                             ba::placeholders::error,
                             ba::placeholders::bytes_transferred));
  io_.run();
}

void UDPClient::HandleReceive(const boost::system::error_code &error,
                              size_t bytes_transferred) {
  std::cout << "handle receive" << std::endl;
  for (int i = 0; i < n_receive_size_; ++i) {
    std::cout << receive_buffer_[i] << " ";
  }
  std::cout << std::endl;
  if (is_listening_) {
    socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                               boost::bind(&UDPClient::HandleReceive, this,
                               ba::placeholders::error,
                               ba::placeholders::bytes_transferred));
    } else {
      std::cout << "Stoppped listening" << std::endl;
    }

}

/* Take pointer to car, update on incoming packet
   For this, spawn a new thread, in which run() is called and asynch. received */

//void UDPClient::ToogleCarUpdate(bool is_active) {
//  if (is_active && !is_listening_) {
//    is_listening_ = true;
//    socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
//                               boost::bind(&UDPClient::HandleReceive, this,
//                               ba::placeholders::error,
//                               ba::placeholders::bytes_transferred));
//    io_.run();
//  } else if (!is_active) {
//    is_listening_ = false;
//    io_.stop();
//    io_.reset();
//  }
//}
