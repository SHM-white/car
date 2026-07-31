#pragma once

#include <Arduino.h>
#include <IPAddress.h>

namespace hmi_config {

constexpr char WIFI_SSID[] = "REPLACE_WITH_OFFLINE_AP";
constexpr char WIFI_PASSWORD[] = "REPLACE_WITH_WPA2_PASSWORD";
constexpr uint8_t AUTH_KEY[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
};

const IPAddress LOCAL_IP(192, 168, 20, 3);
const IPAddress GATEWAY(192, 168, 20, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress ROS_IP(192, 168, 20, 1);
const IPAddress CAR_IP(192, 168, 20, 2);
constexpr uint16_t ROS_UDP_PORT = 42000;
constexpr uint16_t CAR_UDP_PORT = 42001;
constexpr uint16_t LOCAL_UDP_PORT = 42002;
constexpr uint32_t SENDER_ID = 0x484D4931;      // "HMI1"
constexpr uint32_t ROS_SENDER_ID = 0x524F5331;  // "ROS1"
constexpr uint32_t CAR_SENDER_ID = 0x43415231;  // "CAR1"
constexpr uint32_t WIFI_RETRY_PERIOD_MS = 2000;
constexpr uint32_t ROS_STATUS_STALE_MS = 750;
constexpr uint32_t SELECTION_RETRY_MS = 250;
// 心跳周期必须明显小于 ROS_STATUS_STALE_MS(750)，否则 ROS 链路会被周期性误判为陈旧
constexpr uint32_t HEARTBEAT_PERIOD_MS = 250;

}  // namespace hmi_config

