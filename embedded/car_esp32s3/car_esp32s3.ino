#include <Arduino.h>
#include <esp_system.h>

#include "CarController.h"
#include "UdpTelemetry.h"

CarController car;
UdpTelemetry telemetry;
uint32_t next_telemetry_ms = 0;
uint32_t next_heartbeat_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  const uint32_t now = millis();
  Serial.println("\n[自检] ESP32-S3 循迹车启动");

  if (!car.begin(now)) {
    Serial.println("[自检] 循迹传感器或电机初始化失败，车辆已锁定");
  }
  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println("[自检] 检测到欠压复位，必须人工检查并重新上电");
    car.forceSafeStop(d_task::FAULT_BROWNOUT);
  }
  telemetry.begin(now);
  next_telemetry_ms = now;
  next_heartbeat_ms = now;
}

void loop() {
  const uint32_t now_ms = millis();
  const uint32_t now_us = micros();
  telemetry.update(now_ms);
  car.update(now_ms, now_us, telemetry.connected());

  if (static_cast<int32_t>(now_ms - next_telemetry_ms) >= 0) {
    next_telemetry_ms += d_task::kCarTelemetryPeriodMs;
    if (telemetry.sendTelemetry(car.telemetry(telemetry.connected()), now_ms)) {
      // 同一事件重复若干帧且 event_id 不变；接收端按 ID 幂等处理，抵抗 UDP 丢包。
      car.noteTelemetryTransmitted();
    }
  }
  if (static_cast<int32_t>(now_ms - next_heartbeat_ms) >= 0) {
    next_heartbeat_ms += 1000;
    telemetry.sendHeartbeat(now_ms);
  }
}
