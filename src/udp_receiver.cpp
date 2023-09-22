#include "ros2_udp/udp_receiver.hpp"

namespace rur {

template <uint16_t PacketSize>
UDPReceiver<PacketSize>::UDPReceiver(const uint16_t &local_listen_port)
    : socket_(io_service_,
              asio::ip::udp::endpoint(asio::ip::udp::v4(), local_listen_port)),
      local_listen_port_(local_listen_port) {}

template <uint16_t PacketSize>
UDPReceiver<PacketSize>::~UDPReceiver() {
  io_service_.stop();
  socket_.close();
  socket_.shutdown(asio::ip::udp::socket::shutdown_both);
}

template <uint16_t PacketSize>
void UDPReceiver<PacketSize>::StartLoop(
    const std::function<void(const ReceiveResult &)> &callback,
    std::unique_ptr<bool> &is_running) {
  ReceiveResult result;

  while (rclcpp::ok() && !io_service_.stopped() && socket_.is_open() &&
         is_running) {
    try {
      std::array<uint8_t, PacketSize> buffer{};
      asio::ip::udp::endpoint remote_endpoint;
      size_t len = socket_.receive_from(asio::buffer(buffer), remote_endpoint,
                                        MSG_OOB, error_code_);
      if (error_code_) {
        throw error_code_.message().c_str();
      }
      if (len > PacketSize) {
        throw std::runtime_error("UDPReceiver: Received packet too large");
      }
      result.buffer = buffer;
      result.remote_endpoint = remote_endpoint;
      result.receive_time = std::chrono::steady_clock::now();
      callback(result);
    } catch (const char *s) {
      throw std::runtime_error(s);
    } catch (...) {
      throw std::runtime_error("UDPReceiver: Unknown error");
    }
  }
}

template class UDPReceiver<128>;

}  // namespace rur
