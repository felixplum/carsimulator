#include "udp_client.h"

UDPClient::UDPClient() :
  is_listening_(false),
  port_number_(1122),
  socket_(io_, ba::ip::udp::v4()),
  endpoint_server_(ba::ip::address::from_string("127.0.0.1"), port_number_),
  n_receive_size_(4), // 4 float per udp package
  receive_buffer_(n_receive_size_, 0) // write received floats here
{

  socket_.set_option(boost::asio::socket_base::reuse_address(true));
  socket_.bind(ba::ip::udp::endpoint(ba::ip::udp::v4(), port_number_));
}

void UDPClient::HandleReceive(const boost::system::error_code &error,
                              size_t bytes_transferred) {
  std::cout << "handle receive" << std::endl;
  for (int i = 0; i < n_receive_size_; ++i) {
    std::cout << receive_buffer_[i] << " ";
  }
  std::cout << std::endl;
  socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                             boost::bind(&UDPClient::HandleReceive, this,
                             ba::placeholders::error,
                             ba::placeholders::bytes_transferred));

}

//void UDPClient::Run() {
//  std::cout << "entering run()" << std::endl;
//  while (is_listening_) {
////    ba::buffer raw_buff( (char*)&receive_buffer_.front(), n_receive_size_*4);
//    std::cout << "wait for package" << std::endl;
//    socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
//                               boost::bind(&UDPClient::HandleReceive, this,
//                               ba::placeholders::error,
//                               ba::placeholders::bytes_transferred));
//    std::cout << "received package" << std::endl;
//    io_.run();
//  }
//  std::cout << "leaving run()" << std::endl;
//}

/* Take pointer to car, update on incoming packet
   For this, spawn a new thread, in which run() is called and asynch. received */

void UDPClient::ToogleCarUpdate(bool is_active) {
  if (is_active && !is_listening_) {
    is_listening_ = true;
    socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                               boost::bind(&UDPClient::HandleReceive, this,
                               ba::placeholders::error,
                               ba::placeholders::bytes_transferred));
    io_.run();
//    boost::thread t(&UDPClient::Run, this);
  } else if (!is_active) {
    is_listening_ = false;
    io_.stop();
    io_.reset();
  }
}
