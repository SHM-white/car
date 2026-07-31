#include <Arduino.h>
#include <esp_system.h>

#include "LineFollower.h"
#include "UdpTelemetry.h"

LineFollower line_follower;
UdpTelemetry udp_telemetry;

uint32_t next_telemetry_ms = 0;
uint32_t next_heartbeat_ms = 0;
bool initialized = false;

namespace {

constexpr uint32_t kNavigationPrintPeriodMs = 200;
constexpr uint32_t kRosStatusPrintPeriodMs = 1000;
constexpr uint32_t kRosStaleMs = 2500;

const char *carStateName(d_task::CarState state) {
  switch (state) {
    case d_task::CarState::READY: return "READY";
    case d_task::CarState::RUNNING: return "RUNNING";
    case d_task::CarState::COMPLETE: return "COMPLETE";
    case d_task::CarState::SAFE_STOP: return "SAFE_STOP";
  }
  return "UNKNOWN";
}

const char *turnName(d_task::TurnClass turn) {
  switch (turn) {
    case d_task::TurnClass::STRAIGHT: return "STRAIGHT";
    case d_task::TurnClass::SMALL: return "SMALL";
    case d_task::TurnClass::LARGE: return "LARGE";
  }
  return "UNKNOWN";
}

[[maybe_unused]] void navigationPrintTask(uint32_t now_ms) {
  static uint32_t next_print_ms = 0;
  if (static_cast<int32_t>(now_ms - next_print_ms) < 0) return;
  next_print_ms = now_ms + kNavigationPrintPeriodMs;

  const LineFollower::NavigationData data = line_follower.navigationData();
  Serial.printf("[NAV] state=%s turn=%s error=%+.3f left=%+.2f right=%+.2f encoder=%ld/%ld distance=%.3fm speed=%.3fm/s\n",
                carStateName(data.state), turnName(data.turn), data.line_error,
                data.left_command, data.right_command,
                static_cast<long>(data.left_encoder_count),
                static_cast<long>(data.right_encoder_count),
                data.displacement_m, data.velocity_m_s);
}

[[maybe_unused]] void lineChannelsPrintTask(uint32_t now_ms) {
  static uint32_t next_print_ms = 0;
  if (static_cast<int32_t>(now_ms - next_print_ms) < 0) return;
  next_print_ms = now_ms + kNavigationPrintPeriodMs;

  uint8_t channels[car_config::LINE_SENSOR_COUNT] = {};
  line_follower.lineChannels(channels);
  Serial.printf("[LINE_RAW] CH1=%3u CH2=%3u CH3=%3u CH4=%3u CH5=%3u CH6=%3u CH7=%3u CH8=%3u\n",
                static_cast<unsigned>(channels[0]), static_cast<unsigned>(channels[1]),
                static_cast<unsigned>(channels[2]), static_cast<unsigned>(channels[3]),
                static_cast<unsigned>(channels[4]), static_cast<unsigned>(channels[5]),
                static_cast<unsigned>(channels[6]), static_cast<unsigned>(channels[7]));
}

[[maybe_unused]] void rosConnectionPrintTask(uint32_t now_ms) {
  static uint32_t next_print_ms = 0;
  if (static_cast<int32_t>(now_ms - next_print_ms) < 0) return;
  next_print_ms = now_ms + kRosStatusPrintPeriodMs;

  const uint32_t age_ms = udp_telemetry.rosAge(now_ms);
  if (age_ms == UINT32_MAX) {
    Serial.printf("[ROS] wifi_udp=%s status=NO_DATA rejected=%lu\n",
                  udp_telemetry.connected() ? "ONLINE" : "OFFLINE",
                  static_cast<unsigned long>(udp_telemetry.rejectedPackets()));
    return;
  }
  Serial.printf("[ROS] wifi_udp=%s status=%s age=%lums rejected=%lu\n",
                udp_telemetry.connected() ? "ONLINE" : "OFFLINE",
                udp_telemetry.rosFresh(now_ms, kRosStaleMs) ? "ONLINE" : "STALE",
                static_cast<unsigned long>(age_ms),
                static_cast<unsigned long>(udp_telemetry.rejectedPackets()));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[自检] ESP32-S3 黑线循迹启动");
  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println("[自检] 检测到欠压复位，请检查电机电源后重新上电");
    return;
  }
  const uint32_t now = millis();
  udp_telemetry.begin(now);
  line_follower.begin(now);
  next_telemetry_ms = now;
  next_heartbeat_ms = now;
  initialized = true;
}

void loop() {
  if (!initialized) {
    delay(100);
    return;
  }

  const uint32_t now = millis();
  udp_telemetry.update(now);
  line_follower.update(now, micros());

  // 小车刚进入运行状态时，立即发送 START 遥测到地面站和机载主机（不等下一个50ms周期）
  if (line_follower.consumeStartAnnouncement()) {
    const d_task::CarTelemetry start_telemetry = line_follower.telemetry(udp_telemetry.connected());
    if (udp_telemetry.sendTelemetry(start_telemetry, now)) {
      line_follower.noteTelemetryTransmitted();
      Serial.printf("[启动] START 遥测已立即发送 (event_id=%u)\n", start_telemetry.event_id);
    } else {
      Serial.println("[启动] START 遥测发送失败 (WiFi未连接?)");
    }
  }

  if (static_cast<int32_t>(now - next_telemetry_ms) >= 0) {
    next_telemetry_ms = now + d_task::kCarTelemetryPeriodMs;
    const d_task::CarTelemetry telemetry = line_follower.telemetry(udp_telemetry.connected());
    if (udp_telemetry.sendTelemetry(telemetry, now)) {
      line_follower.noteTelemetryTransmitted();
    }
  }

  if (static_cast<int32_t>(now - next_heartbeat_ms) >= 0) {
    next_heartbeat_ms = now + 1000;
    udp_telemetry.sendHeartbeat(now);
  }

  // Each print task is enabled or disabled with one line.
  //lineChannelsPrintTask(now);
  //navigationPrintTask(now);
  //rosConnectionPrintTask(now);

  delay(1);
}
