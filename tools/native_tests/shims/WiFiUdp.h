#pragma once

#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include "Arduino.h"
#include "WiFi.h"

namespace wifi_udp_fake {

inline std::string telemetry_output;
inline std::string incoming_packet;
inline IPAddress remote_ip;
inline uint16_t remote_port = 0;
// 보낸 데이터그램 전부를 순서대로(차분 하네스). 기본 null.
inline void (*on_send)(const std::string &) = nullptr;

inline void reset() {
  telemetry_output.clear();
  incoming_packet.clear();
  remote_ip = IPAddress();
  remote_port = 0;
}

}  // namespace wifi_udp_fake

class WiFiUDP {
public:
  uint8_t begin(uint16_t) { return 1; }

  int beginPacket(const IPAddress &, uint16_t) {
    outgoing_packet_.clear();
    return 1;
  }

  int printf(const char *format, ...) {
    const std::size_t before = outgoing_packet_.size();
    va_list arguments;
    va_start(arguments, format);
    arduino_fake::appendFormatted(outgoing_packet_, format, arguments);
    va_end(arguments);
    return static_cast<int>(outgoing_packet_.size() - before);
  }

  std::size_t write(const uint8_t *buffer, std::size_t length) {
    outgoing_packet_.append(
        reinterpret_cast<const char *>(buffer),
        length);
    return length;
  }

  int endPacket() {
    wifi_udp_fake::telemetry_output = outgoing_packet_;
    if (wifi_udp_fake::on_send) wifi_udp_fake::on_send(outgoing_packet_);
    return 1;
  }

  int parsePacket() {
    return static_cast<int>(wifi_udp_fake::incoming_packet.size());
  }

  int read(char *buffer, std::size_t length) {
    const std::size_t copied =
        std::min(length, wifi_udp_fake::incoming_packet.size());
    std::memcpy(buffer, wifi_udp_fake::incoming_packet.data(), copied);
    wifi_udp_fake::incoming_packet.erase(0, copied);
    return static_cast<int>(copied);
  }

  IPAddress remoteIP() const { return wifi_udp_fake::remote_ip; }
  uint16_t remotePort() const { return wifi_udp_fake::remote_port; }

private:
  std::string outgoing_packet_;
};
