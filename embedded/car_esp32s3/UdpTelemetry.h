#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DTaskProtocol.h>
#include <esp_system.h>

#include "Config.h"

static_assert(sizeof(car_config::AUTH_KEY) == 32, "车辆 HMAC 密钥必须为 32 字节");

class UdpTelemetry {
 public:
  void begin(uint32_t now_ms) {
    boot_id_ = esp_random();
    if (boot_id_ == 0) boot_id_ = 1;
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    connect(now_ms);
  }

  void update(uint32_t now_ms) {
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (!connected) {
      if (udp_started_) { udp_.stop(); udp_started_ = false; }
      if (now_ms - last_connect_attempt_ms_ >= car_config::WIFI_RETRY_PERIOD_MS) connect(now_ms);
      return;
    }
    if (!udp_started_) {
      udp_started_ = udp_.begin(car_config::LOCAL_UDP_PORT) == 1;
      if (udp_started_) Serial.printf("[网络] UDP 已监听端口 %u，地址 %s\n", car_config::LOCAL_UDP_PORT, WiFi.localIP().toString().c_str());
    }
    receivePeerPacket(now_ms);
  }

  bool sendTelemetry(const d_task::CarTelemetry &telemetry, uint32_t now_ms) {
    uint8_t payload[d_task::kMaxPayloadSize];
    const size_t payload_length = d_task::encodeCarTelemetry(telemetry, payload, sizeof(payload));
    return send(d_task::MessageType::CAR_TELEMETRY, payload, payload_length, now_ms);
  }

  bool sendHeartbeat(uint32_t now_ms) { return send(d_task::MessageType::HEARTBEAT, nullptr, 0, now_ms); }
  bool connected() const { return WiFi.status() == WL_CONNECTED && udp_started_; }
  uint32_t bootId() const { return boot_id_; }
  uint32_t rejectedPackets() const { return rejected_packets_; }

 private:
  void connect(uint32_t now_ms) {
    last_connect_attempt_ms_ = now_ms;
    Serial.printf("[网络] 正在连接封闭热点 %s\n", car_config::WIFI_SSID);
    WiFi.begin(car_config::WIFI_SSID, car_config::WIFI_PASSWORD);
  }

  bool send(d_task::MessageType type, const uint8_t *payload, size_t payload_length, uint32_t now_ms) {
    if (!connected()) return false;
    d_task::PacketHeader header{type, static_cast<uint16_t>(payload_length), car_config::SENDER_ID,
                                boot_id_, next_sequence_++, now_ms};
    uint8_t packet[d_task::kMaxPacketSize]; size_t packet_length = 0;
    if (!d_task::encodePacket(header, payload, car_config::AUTH_KEY, sizeof(car_config::AUTH_KEY),
                              packet, sizeof(packet), packet_length)) return false;
    const bool ros_ok = sendTo(car_config::ROS_IP, car_config::ROS_UDP_PORT, packet, packet_length);
    const bool hmi_ok = sendTo(car_config::HMI_IP, car_config::HMI_UDP_PORT, packet, packet_length);
    return ros_ok && hmi_ok;
  }

  bool sendTo(const IPAddress &address, uint16_t port, const uint8_t *packet, size_t packet_length) {
    if (!udp_.beginPacket(address, port)) return false;
    const size_t written = udp_.write(packet, packet_length);
    return written == packet_length && udp_.endPacket() == 1;
  }

  void receivePeerPacket(uint32_t now_ms) {
    const int packet_size = udp_.parsePacket();
    if (packet_size <= 0) return;
    if (udp_.remoteIP() != car_config::ROS_IP || udp_.remotePort() != car_config::ROS_UDP_PORT ||
        packet_size > static_cast<int>(d_task::kMaxPacketSize)) {
      drainPacket(); ++rejected_packets_; return;
    }
    uint8_t packet[d_task::kMaxPacketSize];
    const int length = udp_.read(packet, sizeof(packet));
    d_task::PacketHeader header{}; const uint8_t *payload = nullptr;
    if (length != packet_size || d_task::decodePacket(packet, length, car_config::AUTH_KEY,
        sizeof(car_config::AUTH_KEY), header, payload) != d_task::DecodeResult::OK ||
        header.sender_id != car_config::ROS_SENDER_ID) { ++rejected_packets_; return; }
    if (!peer_session_valid_ || header.boot_id != peer_boot_id_) {
      peer_boot_id_ = header.boot_id;
      peer_sequence_.beginSession(header.sender_id, header.boot_id);
      peer_session_valid_ = true;
      Serial.printf("[网络] 已绑定 ROS 新会话 boot=%08lX\n", static_cast<unsigned long>(peer_boot_id_));
    }
    if (!peer_sequence_.accept(header.sender_id, header.boot_id, header.sequence)) { ++rejected_packets_; return; }
    last_peer_packet_ms_ = now_ms;
  }

  void drainPacket() { while (udp_.available() > 0) udp_.read(); }

  WiFiUDP udp_;
  d_task::SequenceTracker peer_sequence_;
  uint32_t boot_id_ = 0;
  uint32_t next_sequence_ = 0;
  uint32_t peer_boot_id_ = 0;
  uint32_t last_connect_attempt_ms_ = 0;
  uint32_t last_peer_packet_ms_ = 0;
  uint32_t rejected_packets_ = 0;
  bool udp_started_ = false;
  bool peer_session_valid_ = false;
};
