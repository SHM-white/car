#include <Arduino.h>
#include <esp_system.h>

#include "CarController.h"
#include "UdpTelemetry.h"

CarController car;
UdpTelemetry udp_telemetry;

uint32_t next_telemetry_ms = 0;
uint32_t next_heartbeat_ms = 0;
bool initialized = false;

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
  car.begin(now);
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
  car.update(now, micros(), udp_telemetry.connected());

  if (static_cast<int32_t>(now - next_telemetry_ms) >= 0) {
    next_telemetry_ms = now + d_task::kCarTelemetryPeriodMs;
    const d_task::CarTelemetry telemetry = car.telemetry(udp_telemetry.connected());
    if (udp_telemetry.sendTelemetry(telemetry, now)) {
      car.noteTelemetryTransmitted();
    }
  }

  if (static_cast<int32_t>(now - next_heartbeat_ms) >= 0) {
    next_heartbeat_ms = now + 1000;
    udp_telemetry.sendHeartbeat(now);
  }

  delay(1);
}
