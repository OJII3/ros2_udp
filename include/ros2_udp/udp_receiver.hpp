#pragma once

#include <asio.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace rur {

template <uint16_t PacketSize>
class UDPReceiver {
 public:
  UDPReceiver(const UDPReceiver&) = delete;
  UDPReceiver(UDPReceiver&&) = delete;
  UDPReceiver& operator=(const UDPReceiver&) = delete;
  UDPReceiver& operator=(UDPReceiver&&) = delete;
  UDPReceiver(const uint16_t& local_listen_port);
  ~UDPReceiver();

  struct ReceiveResult {
    std::array<uint8_t, PacketSize> buffer{};
    asio::ip::udp::endpoint remote_endpoint;
    std::chrono::steady_clock::time_point receive_time;
  };

  void StartLoop(const std::function<void(const ReceiveResult&)>& callback,
                 std::unique_ptr<bool>& is_running);

 private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  uint16_t local_listen_port_;
  asio::error_code error_code_;
};

}  // namespace rur
