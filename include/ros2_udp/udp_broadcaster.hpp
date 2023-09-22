#pragma once

#include <array>
#include <asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "udp_client.hpp"
#include "udp_packet.hpp"

namespace rur::network {

template <uint16_t PacketSize>
class UDPBroadcaster : public rclcpp::Node, private UDPClient<PacketSize> {
 public:
  UDPBroadcaster(const uint16_t &local_send_port);

  void SetMessage(const std::array<uint8_t, PacketSize> &packet);
  void Start(const uint16_t &remote_listen_port, const uint32_t &interval_ms,
             std::unique_ptr<bool> &is_running);

 private:
  std::vector<UDPPacket<PacketSize>> packets_{};
  uint16_t remote_listen_port_{};
  uint32_t interval_ms_{};
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscriber_;
};

}  // namespace rur::network
