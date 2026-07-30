#include <Arduino.h>
#include <esp_system.h>

#include "LineFollower.h"

LineFollower line_follower;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[自检] ESP32-S3 黑线循迹启动");
  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println("[自检] 检测到欠压复位，请检查电机电源后重新上电");
    return;
  }
  line_follower.begin(millis());
}

void loop() {
  line_follower.update(millis(), micros());
  delay(1);
}
