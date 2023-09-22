#include "ros2_udp/udp_broadcaster.hpp"

#include <memory>

namespace rur::network {

template <uint16_t PacketSize>
UDPBroadcaster<PacketSize>::UDPBroadcaster(const uint16_t &local_send_port)
    : rclcpp::Node("udp_broadcaster"), UDPClient<PacketSize>(local_send_port) {
  this->get_parameter_or("remote_listen_port", remote_listen_port_);
  this->get_parameter_or("interval_ms", interval_ms_);
}

template <uint16_t PacketSize>
void UDPBroadcaster<PacketSize>::Start(const uint16_t &remote_listen_port,
                                       const uint32_t &interval_ms,
                                       std::unique_ptr<bool> &is_running) {
  while (rclcpp::ok() && this->IsOpen() && is_running) {
    try {
      for (const auto &packet : packets_) {
        this->Broadcast(packet, remote_listen_port);
      }
      packets_.clear();
    } catch (const char *s) {
      RCLCPP_ERROR(this->get_logger(), "%s", s);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "UDPBroadcaster: Unknown error");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }
}

template <uint16_t PacketSize>
void UDPBroadcaster<PacketSize>::SetMessage(
    const std::array<uint8_t, PacketSize> &raw_packet) {
  bool flag = false;
  for (auto &packet : packets_) {
    if (packet.id == raw_packet[HeaderSize]) {
      packet = packet.FromRaw(raw_packet);
      flag = true;
      break;
    }
  }
  if (!flag) {
    packets_.emplace_back(UDPPacket<PacketSize>::FromRaw(raw_packet));
  }
}

template class UDPBroadcaster<128>;

}  // namespace rur::network
