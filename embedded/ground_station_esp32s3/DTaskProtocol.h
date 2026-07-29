#pragma once

#include <stddef.h>
#include <stdint.h>

namespace d_task {

constexpr uint16_t kMagic = 0x4454;
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 22;
constexpr size_t kMaxPayloadSize = 64;
constexpr size_t kCrcSize = 2;
constexpr size_t kAuthTagSize = 8;
constexpr size_t kMaxPacketSize = kHeaderSize + kMaxPayloadSize + kCrcSize + kAuthTagSize;
constexpr uint32_t kCarTelemetryPeriodMs = 50;
constexpr uint32_t kTelemetryStaleMs = 750;

enum class MessageType : uint8_t {
  HEARTBEAT = 1,
  CAR_TELEMETRY = 2,
  TASK_SELECTION = 3,
  MISSION_STATUS = 4,
  DIAGNOSTIC = 5,
};

enum class CarState : uint8_t { READY = 0, RUNNING = 1, COMPLETE = 2, SAFE_STOP = 3 };
enum class TurnClass : uint8_t { STRAIGHT = 0, SMALL = 1, LARGE = 2 };
enum class RouteEvent : uint8_t { NONE = 0, START = 1, B = 2, D = 3, A = 4, COMPLETE = 5 };
enum class MissionPhase : uint8_t { PRESTART = 0, SELECTION_ACKED = 1, ARMED_READY = 2, CAR_RUNNING = 3, COMPLETE = 4, FAULT = 5 };

enum QualityFlag : uint16_t {
  QUALITY_LINE_VALID = 1U << 0,
  QUALITY_ENCODER_VALID = 1U << 1,
  QUALITY_WIFI_CONNECTED = 1U << 2,
  QUALITY_SELECTION_COMMITTED = 1U << 3,
};

// 这些状态由 ROS 权威节点填写，HMI 只负责原样显示，不能自行推断健康状态。
enum MissionStatusFlag : uint16_t {
  MISSION_DRONE_LINK_OK = 1U << 0,
  MISSION_DRONE_ARMED = 1U << 1,
  MISSION_VISION_VALID = 1U << 2,
  MISSION_ROS_READY = 1U << 3,
};

enum FaultFlag : uint16_t {
  FAULT_NONE = 0,
  FAULT_WIFI_TIMEOUT = 1U << 0,
  FAULT_LINE_LOST = 1U << 1,
  FAULT_ENCODER_DISAGREE = 1U << 2,
  FAULT_PID_OVERRUN = 1U << 3,
  FAULT_BUTTON_STUCK = 1U << 4,
  FAULT_MOTOR = 1U << 5,
  FAULT_STALE_DATA = 1U << 6,
  FAULT_PROTOCOL = 1U << 7,
  FAULT_NO_COMMITTED_SELECTION = 1U << 8,
  FAULT_BROWNOUT = 1U << 9,
};

struct PacketHeader {
  MessageType type;
  uint16_t payload_length;
  uint32_t sender_id;
  uint32_t boot_id;
  uint32_t sequence;
  uint32_t monotonic_ms;
};

struct CarTelemetry {
  CarState state;
  TurnClass turn;
  RouteEvent event;
  uint16_t event_id;
  uint16_t quality_flags;
  int32_t displacement_mm;
  int16_t velocity_mm_s;
  int16_t line_error_milli;
  uint16_t fault_flags;
};

struct TaskSelection {
  uint32_t selection_id;
  uint32_t car_boot_id;
  uint8_t task;
};

struct MissionStatus {
  uint32_t selection_id;
  uint32_t car_boot_id;
  uint32_t hmi_boot_id;
  MissionPhase phase;
  uint8_t selected_task;
  uint16_t reason_flags;
  uint16_t status_flags;
};

enum class DecodeResult : uint8_t {
  OK = 0,
  TOO_SHORT,
  TOO_LONG,
  BAD_MAGIC,
  BAD_VERSION,
  BAD_TYPE,
  BAD_LENGTH,
  BAD_CRC,
  BAD_AUTH,
};

// 所有多字节整数都按小端序逐字节编码，禁止直接转换 C 结构体。
bool encodePacket(const PacketHeader &header, const uint8_t *payload,
                  const uint8_t *auth_key, size_t auth_key_length,
                  uint8_t *output, size_t output_capacity, size_t &output_length);
DecodeResult decodePacket(const uint8_t *packet, size_t packet_length,
                          const uint8_t *auth_key, size_t auth_key_length,
                          PacketHeader &header, const uint8_t *&payload);

size_t encodeCarTelemetry(const CarTelemetry &message, uint8_t *output, size_t capacity);
bool decodeCarTelemetry(const uint8_t *payload, size_t length, CarTelemetry &message);
size_t encodeTaskSelection(const TaskSelection &message, uint8_t *output, size_t capacity);
bool decodeTaskSelection(const uint8_t *payload, size_t length, TaskSelection &message);
size_t encodeMissionStatus(const MissionStatus &message, uint8_t *output, size_t capacity);
bool decodeMissionStatus(const uint8_t *payload, size_t length, MissionStatus &message);

uint16_t crc16Ccitt(const uint8_t *data, size_t length);
void hmacSha256(const uint8_t *key, size_t key_length, const uint8_t *data,
                size_t data_length, uint8_t output[32]);
bool constantTimeEqual(const uint8_t *left, const uint8_t *right, size_t length);
bool isKnownMessageType(uint8_t type);

class SequenceTracker {
 public:
  SequenceTracker();

  // 只有上层验证了来源 IP/端口及新启动纪元后，才可建立新会话。
  void beginSession(uint32_t sender_id, uint32_t boot_id);
  bool accept(uint32_t sender_id, uint32_t boot_id, uint32_t sequence);
  void reset();

 private:
  uint32_t sender_id_;
  uint32_t boot_id_;
  uint32_t last_sequence_;
  bool session_active_;
  bool has_sequence_;
};

}  // namespace d_task
