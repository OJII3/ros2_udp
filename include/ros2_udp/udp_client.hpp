#include <asio.hpp>
#include <string>

#include "udp_packet.hpp"

namespace rur::network {

template <uint16_t PacketSize>
class UDPClient {
 public:
  UDPClient(const UDPClient &) = delete;
  UDPClient(UDPClient &&) = delete;
  UDPClient &operator=(const UDPClient &) = delete;
  UDPClient &operator=(UDPClient &&) = delete;

 protected:
  UDPClient(const uint16_t &local_port);
  ~UDPClient();

  bool Send(const UDPPacket<PacketSize> &packet, const std::string &ip_address,
            const uint16_t &port);
  bool Broadcast(const UDPPacket<PacketSize> &packet, const uint16_t &port);
  bool Receive(UDPPacket<PacketSize> &packet);
  bool IsOpen() const { return socket_.is_open() && !io_service_.stopped(); }

 private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_endpoint_;
};

}  // namespace rur::network
