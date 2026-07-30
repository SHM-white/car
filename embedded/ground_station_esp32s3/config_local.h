#pragma once

#include <Arduino.h>
#include <IPAddress.h>

namespace hmi_config {

constexpr char WIFI_SSID[] = "ED-UAV";
constexpr char WIFI_PASSWORD[] = "5RQqDVzbg5GxZpLz";
constexpr uint8_t AUTH_KEY[] = {
    0x76, 0x5D, 0x69, 0x2A, 0xEE, 0xA6, 0x47, 0xB9,
    0x7C, 0x52, 0xC3, 0xD8, 0x52, 0xF4, 0x1F, 0xF5,
    0xC7, 0x60, 0xEE, 0xD5, 0xD8, 0x4F, 0x15, 0x9F,
    0x3F, 0x7C, 0x53, 0xD2, 0xCA, 0x0A, 0x18, 0x32,
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

}  // namespace hmi_config
