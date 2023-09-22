#include <array>
#include <cstdint>
#include <cstring>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <variant>

namespace rur::network {

constexpr uint16_t MaxPacketSize = 1024;
constexpr std::array<uint8_t, 2> Header = {'S', 'S'};
constexpr std::array<uint8_t, 2> Footer = {'E', 'E'};
constexpr uint16_t HeaderSize = Header.size();
constexpr uint16_t FooterSize = Footer.size();
constexpr uint16_t IDSize = 1;

template <uint16_t PacketSize>
struct UDPPacket {
  static_assert(PacketSize <= MaxPacketSize,
                "Packet size must be less than MaxPacketSize");

 private:
  static constexpr uint16_t BodySize =
      PacketSize - HeaderSize - FooterSize - IDSize;
  std::array<uint8_t, PacketSize> raw_{};

 public:
  uint8_t id = 0;
  std::array<uint8_t, BodySize> body{};

  void Clear() {
    id = 0;
    body.fill(0);
  }

  static UDPPacket<PacketSize> FromRaw(
      const std::array<uint8_t, PacketSize>& raw) {
    UDPPacket<PacketSize> packet{};
    memcpy(packet.raw_.data(), raw.data(), PacketSize);
    packet.id = packet.raw_[HeaderSize];
    std::memcpy(packet.body.data(), packet.raw_.data() + HeaderSize + IDSize,
                BodySize);
    return packet;
  }

  static UDPPacket<PacketSize> FromRaw(
      const std_msgs::msg::ByteMultiArray& raw) {
    if (raw.data.size() != PacketSize) {
      throw std::runtime_error("UDPPacket: Received packet size mismatch");
    }
    UDPPacket<PacketSize> packet{};
    packet.id = raw.data[HeaderSize];
    memcpy(packet.body.data(), &raw.data[HeaderSize + IDSize], BodySize);
    return packet;
  }

  static std::array<uint8_t, PacketSize> ToRaw(
      const UDPPacket<PacketSize>& packet) {
    std::array<uint8_t, PacketSize> raw{};
    memcpy(raw.data(), Header.data(), HeaderSize);
    raw[HeaderSize] = packet.id;
    memcpy(&raw[HeaderSize + IDSize], packet.body.data(), BodySize);
    memcpy(&raw[PacketSize - FooterSize], Footer.data(), FooterSize);
    return raw;
  }
};

template struct UDPPacket<128>;

}  // namespace rur::network
