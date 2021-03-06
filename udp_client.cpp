#include "udp_client.h"

UDPClient::UDPClient():
  port_number_(9933), // 9933 in RCL, order: identifier, &timestep, &xPos, &yPos, &psi
  socket_(io_, ba::ip::udp::v4()),
  endpoint_server_(ba::ip::address::from_string("127.0.0.1"), port_number_),

  n_receive_size_(5), // number of floats per udp package

  receive_buffer_(n_receive_size_, 0), // write received floats here
  is_bind_(false)
{
  socket_.set_option(boost::asio::socket_base::reuse_address(true));
}

void UDPClient::HandleReceive(const boost::system::error_code &error,
                              size_t bytes_transferred) {
  std::cout << "handle receive" << std::endl;
  for (int i = 0; i < n_receive_size_; ++i) {
    std::cout << receive_buffer_[i] << " ";
  }
  std::cout << std::endl;
  // Assume transmitted vector to contain [x, y, phi, time]
  if (!car_udp_) {
      std::cout << "UDPClient: Car is NULL! UDPClient stopped" << std::endl;
      return;
  }
  std::vector<float> car_state(receive_buffer_.begin()+2,
                               receive_buffer_.begin()+5);
  car_udp_->GetCarState().UpdateStateVector(car_state, 0);
  car_udp_->GetCarState().SetCurrentTime(receive_buffer_[1]);

  // add new listening job to queue
  socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                             boost::bind(&UDPClient::HandleReceive, this,
                             ba::placeholders::error,
                             ba::placeholders::bytes_transferred));
}

/* Take pointer to car, update on incoming packet
   For this, spawn a new thread, in which run() is called and asynch. received */

void UDPClient::ToogleCarUpdate(bool is_active, CarPtr car_ptr) {
  if (is_active && !thread_) {
    if (!car_ptr) {
      std::cerr << "Please provide valid car object to UDPClient" << std::endl;
      return;
    }
    // bind only once
    if (!is_bind_) {

      socket_.bind(ba::ip::udp::endpoint(ba::ip::udp::v4(), port_number_));
      is_bind_ = true;
    }
    car_udp_ = car_ptr;
//    io_.reset();
    // add first listening job
    socket_.async_receive_from(ba::buffer(receive_buffer_), endpoint_server_,
                               boost::bind(&UDPClient::HandleReceive, this,
                               ba::placeholders::error,
                               ba::placeholders::bytes_transferred));
    // execute service in new thread
    thread_.reset(new std::thread(
        boost::bind(&boost::asio::io_service::run, &io_)));
    std::cout << "UDPClient: Started listening .. " << std::endl;
  } else if (!is_active && thread_) {
    io_.stop(); // stop service
    thread_->join(); // stop thread
    io_.reset(); // make ready for new run invocation
    thread_.reset(); // delete thread pointee
    std::cout << "UDPClient: Stopped listening .. " << std::endl;
  }
}
