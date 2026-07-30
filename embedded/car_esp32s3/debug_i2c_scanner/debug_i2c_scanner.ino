/*
 * I2C 总线诊断扫描器 v2 — 排查感为八路灰度传感器通信问题
 *
 * 这是一个独立的 Arduino sketch，在 Arduino IDE 中单独打开烧录。
 * 不要与 car_esp32s3.ino 放在同一文件夹下。
 */

#include <Arduino.h>
#include <Wire.h>

// ---- 与 config_local.example.h 一致的引脚 ----
constexpr int SDA_PIN = 1;
constexpr int SCL_PIN = 2;
constexpr uint32_t I2C_FREQ = 100000;

// ---- 传感器命令 ----
constexpr uint8_t CMD_PING      = 0xAA;
constexpr uint8_t CMD_FIRMWARE  = 0xC1;

/// 方式 A：Repeated START（当前代码使用的方式）
/// 写命令字节后不释放总线，直接发 Repeated START 读取
static bool readWithRepeatedStart(uint8_t addr, uint8_t cmd, uint8_t *out, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  if (Wire.endTransmission(false) != 0) return false;
  size_t received = Wire.requestFrom(addr, len, true);
  if (received != len) {
    while (Wire.available() > 0) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

/// 方式 B：STOP 后重新 START
/// 写命令字节后 STOP，延时，再发 START 读取
static bool readWithStopRestart(uint8_t addr, uint8_t cmd, uint8_t *out, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  if (Wire.endTransmission(true) != 0) return false;  // STOP
  delayMicroseconds(50);
  size_t received = Wire.requestFrom(addr, len, true);
  if (received != len) {
    while (Wire.available() > 0) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

/// 方式 C：只写不读（用于测试连续模式命令 0xB0）
static bool writeOnly(uint8_t addr, uint8_t cmd, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== I2C 总线诊断扫描器 v2 =====");

  // 1. 初始化 I2C
  Serial.printf("初始化 I2C: SDA=GPIO%d, SCL=GPIO%d, 频率=%lu Hz\n", SDA_PIN, SCL_PIN, I2C_FREQ);
  if (!Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ)) {
    Serial.println("[错误] Wire.begin() 失败！");
    return;
  }
  Serial.println("[OK] Wire.begin() 成功\n");

  // 2. 快速扫描
  Serial.println("--- 扫描 I2C 总线 ---");
  int found = 0;
  uint8_t sensorAddr = 0;
  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission(true) == 0) {
      Serial.printf("  [找到] 0x%02X\n", addr);
      if (addr >= 0x4C && addr <= 0x4F) sensorAddr = addr;
      found++;
    }
  }
  Serial.printf("共找到 %d 个设备\n\n", found);

  if (sensorAddr == 0) {
    Serial.println("[结论] 0x4C~0x4F 范围内未发现传感器，请检查接线和跳帽。");
    return;
  }

  // 3. 对传感器逐一测试各种通信方式
  Serial.printf("=== 传感器 @ 0x%02X 详细诊断 ===\n\n", sensorAddr);

  // --- Ping (0xAA) ---
  Serial.println("[1] Ping 命令 (0xAA, 期望返回 0x66)");

  uint8_t resp = 0xFF;
  Serial.print("    方式A (Repeated START): ");
  if (readWithRepeatedStart(sensorAddr, CMD_PING, &resp, 1)) {
    Serial.printf("成功, 返回=0x%02X %s\n", resp, resp == 0x66 ? "✓" : "✗ 值不对");
  } else {
    Serial.println("失败 — 无响应");
  }

  resp = 0xFF;
  Serial.print("    方式B (STOP + 重新START): ");
  if (readWithStopRestart(sensorAddr, CMD_PING, &resp, 1)) {
    Serial.printf("成功, 返回=0x%02X %s\n", resp, resp == 0x66 ? "✓" : "✗ 值不对");
  } else {
    Serial.println("失败 — 无响应");
  }

  // --- 固件版本 (0xC1) ---
  Serial.println("\n[2] 固件版本 (0xC1)");

  resp = 0xFF;
  Serial.print("    方式A (Repeated START): ");
  if (readWithRepeatedStart(sensorAddr, CMD_FIRMWARE, &resp, 1)) {
    Serial.printf("成功, 固件=V%u.%u\n", resp >> 4, resp & 0x0F);
  } else {
    Serial.println("失败 — 无响应");
  }

  resp = 0xFF;
  Serial.print("    方式B (STOP + 重新START): ");
  if (readWithStopRestart(sensorAddr, CMD_FIRMWARE, &resp, 1)) {
    Serial.printf("成功, 固件=V%u.%u\n", resp >> 4, resp & 0x0F);
  } else {
    Serial.println("失败 — 无响应");
  }

  // --- 写命令测试 ---
  Serial.println("\n[3] 写命令测试");
  Serial.print("    通道使能 (0xCE=0xFF): ");
  Serial.println(writeOnly(sensorAddr, 0xCE, 0xFF) ? "ACK ✓" : "NACK ✗");

  Serial.print("    连续模拟量 (0xB0): ");
  Serial.println(writeOnly(sensorAddr, 0xB0, 0x00) ? "ACK ✓" : "NACK ✗");

  // --- 读取八通道数据 (0xB0) ---
  Serial.println("\n[4] 读取八通道模拟量 (0xB0)");
  uint8_t raw[8] = {};
  bool okA = readWithRepeatedStart(sensorAddr, 0xB0, raw, 8);
  bool okB = readWithStopRestart(sensorAddr, 0xB0, raw, 8);

  Serial.print("    方式A (Repeated START): ");
  if (okA) {
    Serial.printf("成功 ch=[%u,%u,%u,%u,%u,%u,%u,%u]\n",
                  raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7]);
  } else {
    Serial.println("失败");
  }

  Serial.print("    方式B (STOP + 重新START): ");
  if (okB) {
    Serial.printf("成功 ch=[%u,%u,%u,%u,%u,%u,%u,%u]\n",
                  raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7]);
  } else {
    Serial.println("失败");
  }

  // 5. 结论
  Serial.println("\n===== 诊断结论 =====");
  if (okA || okB) {
    Serial.println("传感器可正常通信！请根据上面的结果选择正确的方式。");
  } else {
    Serial.println("两种方式均无法读取数据。可能原因：");
    Serial.println("  1. 传感器固件版本不兼容（尝试用示波器/逻辑分析仪抓 I2C 波形）");
    Serial.println("  2. 电平转换器波形畸变（尝试降到 50kHz）");
    Serial.println("  3. 传感器需要特定初始化序列");
  }
}

void loop() {
  delay(8000);
  Serial.println("\n--- 重新诊断 ---");
  setup();
}
