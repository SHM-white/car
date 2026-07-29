#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <DTaskProtocol.h>
#include <esp_system.h>

#include "Config.h"

static_assert(sizeof(hmi_config::AUTH_KEY) == 32, "HMI HMAC 密钥必须为 32 字节");

enum class IncomingKind : uint8_t { NONE = 0, HEARTBEAT, CAR_TELEMETRY, MISSION_STATUS };

struct IncomingMessage {
  IncomingKind kind = IncomingKind::NONE;
  uint32_t sender_boot_id = 0;
  d_task::CarTelemetry car{};
  d_task::MissionStatus mission{};
};

class UdpLink {
 public:
  void begin(uint32_t now_ms) {
    boot_id_ = esp_random(); if (boot_id_ == 0) boot_id_ = 1;
    WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true); connect(now_ms);
  }

  void update(uint32_t now_ms) {
    if (WiFi.status() != WL_CONNECTED) {
      if (udp_started_) { udp_.stop(); udp_started_ = false; }
      if (now_ms - last_connect_attempt_ms_ >= hmi_config::WIFI_RETRY_PERIOD_MS) connect(now_ms);
      return;
    }
    if (!udp_started_) {
      udp_started_ = udp_.begin(hmi_config::LOCAL_UDP_PORT) == 1;
      if (udp_started_) Serial.printf("[网络] HMI UDP 已监听 %u，地址 %s\n", hmi_config::LOCAL_UDP_PORT, WiFi.localIP().toString().c_str());
    }
  }

  IncomingMessage poll(uint32_t now_ms) {
    IncomingMessage result;
    const int packet_size = udp_started_ ? udp_.parsePacket() : 0;
    if (packet_size <= 0) return result;
    const IPAddress source_ip = udp_.remoteIP(); const uint16_t source_port = udp_.remotePort();
    const bool from_car = source_ip == hmi_config::CAR_IP && source_port == hmi_config::CAR_UDP_PORT;
    const bool from_ros = source_ip == hmi_config::ROS_IP && source_port == hmi_config::ROS_UDP_PORT;
    if ((!from_car && !from_ros) || packet_size > static_cast<int>(d_task::kMaxPacketSize)) {
      drainPacket(); ++rejected_packets_; return result;
    }
    uint8_t packet[d_task::kMaxPacketSize]; const int length = udp_.read(packet, sizeof(packet));
    d_task::PacketHeader header{}; const uint8_t *payload = nullptr;
    if (length != packet_size || d_task::decodePacket(packet, length, hmi_config::AUTH_KEY,
        sizeof(hmi_config::AUTH_KEY), header, payload) != d_task::DecodeResult::OK) {
      ++rejected_packets_; return result;
    }

    PeerSession &peer = from_car ? car_peer_ : ros_peer_;
    const uint32_t expected_sender = from_car ? hmi_config::CAR_SENDER_ID : hmi_config::ROS_SENDER_ID;
    if (header.sender_id != expected_sender) { ++rejected_packets_; return result; }
    if (!peer.active || peer.boot_id != header.boot_id) {
      peer.boot_id = header.boot_id; peer.sequence.beginSession(header.sender_id, header.boot_id); peer.active = true;
      Serial.printf("[网络] %s 新会话 boot=%08lX\n", from_car ? "车辆" : "ROS", static_cast<unsigned long>(header.boot_id));
    }
    if (!peer.sequence.accept(header.sender_id, header.boot_id, header.sequence)) { ++rejected_packets_; return result; }
    result.sender_boot_id = header.boot_id;

    if (from_car && header.type == d_task::MessageType::CAR_TELEMETRY &&
        d_task::decodeCarTelemetry(payload, header.payload_length, result.car)) {
      result.kind = IncomingKind::CAR_TELEMETRY;
      peer.last_received_ms = now_ms; peer.has_received = true;
    }
    else if (from_ros && header.type == d_task::MessageType::MISSION_STATUS &&
             d_task::decodeMissionStatus(payload, header.payload_length, result.mission)) {
      result.kind = IncomingKind::MISSION_STATUS;
    }
    else if (header.type == d_task::MessageType::HEARTBEAT && header.payload_length == 0) result.kind = IncomingKind::HEARTBEAT;
    else ++rejected_packets_;
    return result;
  }

  bool sendSelection(const d_task::TaskSelection &selection, uint32_t now_ms) {
    uint8_t payload[d_task::kMaxPayloadSize]; const size_t payload_length = d_task::encodeTaskSelection(selection, payload, sizeof(payload));
    if (payload_length == 0) return false;
    return send(d_task::MessageType::TASK_SELECTION, payload, payload_length, now_ms);
  }

  bool sendHeartbeat(uint32_t now_ms) { return send(d_task::MessageType::HEARTBEAT, nullptr, 0, now_ms); }
  void noteMissionStatusAccepted(uint32_t now_ms) { ros_peer_.last_received_ms = now_ms; ros_peer_.has_received = true; }
  uint32_t bootId() const { return boot_id_; }

  bool connected() const { return WiFi.status() == WL_CONNECTED && udp_started_; }
  bool carFresh(uint32_t now_ms) const { return car_peer_.has_received && now_ms - car_peer_.last_received_ms <= d_task::kTelemetryStaleMs; }
  bool rosFresh(uint32_t now_ms) const { return ros_peer_.has_received && now_ms - ros_peer_.last_received_ms <= hmi_config::ROS_STATUS_STALE_MS; }
  uint32_t carAge(uint32_t now_ms) const { return car_peer_.has_received ? now_ms - car_peer_.last_received_ms : UINT32_MAX; }
  uint32_t rosAge(uint32_t now_ms) const { return ros_peer_.has_received ? now_ms - ros_peer_.last_received_ms : UINT32_MAX; }
  uint32_t rejectedPackets() const { return rejected_packets_; }

 private:
  bool send(d_task::MessageType type, const uint8_t *payload, size_t payload_length, uint32_t now_ms) {
    if (!connected()) return false;
    d_task::PacketHeader header{type, static_cast<uint16_t>(payload_length), hmi_config::SENDER_ID,
                                boot_id_, next_sequence_++, now_ms};
    uint8_t packet[d_task::kMaxPacketSize]; size_t packet_length = 0;
    if (!d_task::encodePacket(header, payload, hmi_config::AUTH_KEY, sizeof(hmi_config::AUTH_KEY),
                              packet, sizeof(packet), packet_length)) return false;
    if (!udp_.beginPacket(hmi_config::ROS_IP, hmi_config::ROS_UDP_PORT)) return false;
    const size_t written = udp_.write(packet, packet_length);
    return written == packet_length && udp_.endPacket() == 1;
  }
  struct PeerSession {
    d_task::SequenceTracker sequence;
    uint32_t boot_id = 0;
    uint32_t last_received_ms = 0;
    bool active = false;
    bool has_received = false;
  };

  void connect(uint32_t now_ms) {
    last_connect_attempt_ms_ = now_ms;
    Serial.printf("[网络] 正在连接封闭热点 %s\n", hmi_config::WIFI_SSID);
    WiFi.begin(hmi_config::WIFI_SSID, hmi_config::WIFI_PASSWORD);
  }
  void drainPacket() { while (udp_.available() > 0) udp_.read(); }

  WiFiUDP udp_;
  PeerSession car_peer_;
  PeerSession ros_peer_;
  uint32_t boot_id_ = 0;
  uint32_t next_sequence_ = 0;
  uint32_t last_connect_attempt_ms_ = 0;
  uint32_t rejected_packets_ = 0;
  bool udp_started_ = false;
};
