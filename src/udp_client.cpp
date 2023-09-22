#include "ros2_udp/udp_client.hpp"

namespace rur::network {

template <uint16_t PacketSize>
UDPClient<PacketSize>::UDPClient(const uint16_t &local_port)
    : socket_(io_service_,
              asio::ip::udp::endpoint(asio::ip::udp::v4(), local_port)) {
  socket_.set_option(asio::socket_base::broadcast(true));
}

template <uint16_t PacketSize>
UDPClient<PacketSize>::~UDPClient() {
  io_service_.stop();
  socket_.close();
  socket_.shutdown(asio::ip::udp::socket::shutdown_both);
}

template <uint16_t PacketSize>
bool UDPClient<PacketSize>::Broadcast(const UDPPacket<PacketSize> &packet,
                                      const uint16_t &port) {
  try {
    socket_.send_to(
        packet.ToRaw(packet),
        asio::ip::udp::endpoint(asio::ip::address_v4::broadcast(), port));
    return true;
  } catch (...) {
    return false;
  }
}

template <uint16_t PacketSize>
bool UDPClient<PacketSize>::Send(const UDPPacket<PacketSize> &packet,
                                 const std::string &id_address,
                                 const uint16_t &port) {
  try {
    socket_.async_send_to(packet,
                          asio::ip::udp::endpoint(
                              asio::ip::address::from_string(id_address), port),
                          [](const asio::error_code &error, size_t bytes_sent) {
                            if (error || bytes_sent != PacketSize) {
                              throw error.message().c_str();
                            }
                          });
    return true;
  } catch (...) {
    return false;
  }
}

template <uint16_t PacketSize>
bool UDPClient<PacketSize>::Receive(UDPPacket<PacketSize> &packet) {
  try {
    packet = {};
    std::array<uint8_t, PacketSize> buffer{};
    auto r = socket_.receive_from(buffer, remote_endpoint_);
    if (r != PacketSize) {
      throw std::runtime_error("UDPClient: Received packet size mismatch");
    }
    packet.FromRaw(buffer);
    return true;
  } catch (...) {
    return false;
  }
}

template class UDPClient<128>;

}  // namespace rur::network
