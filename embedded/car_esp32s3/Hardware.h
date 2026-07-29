#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

static_assert(car_config::LINE_SENSOR_COUNT == 8, "感为灰度传感器必须配置为八路");
static_assert(car_config::LINE_SENSOR_I2C_ADDRESS_FIRST <= car_config::LINE_SENSOR_I2C_ADDRESS_LAST,
              "循迹传感器 I2C 地址范围无效");
static_assert(car_config::ENCODER_COUNTS_PER_REVOLUTION > 0, "编码器每圈计数必须为正数");

struct LineReading {
  float error;
  float strength;
  bool valid;
};

class LineSensors {
 public:
  bool begin() {
    if (!Wire.begin(car_config::LINE_SENSOR_SDA_PIN, car_config::LINE_SENSOR_SCL_PIN,
                    car_config::LINE_SENSOR_I2C_FREQUENCY_HZ)) {
      Serial.println("[循迹] I2C 控制器初始化失败");
      return false;
    }
    Wire.setTimeOut(car_config::LINE_SENSOR_I2C_TIMEOUT_MS);

    for (uint8_t attempt = 0; attempt < car_config::LINE_SENSOR_STARTUP_ATTEMPTS && address_ == 0; ++attempt) {
      for (uint8_t address = car_config::LINE_SENSOR_I2C_ADDRESS_FIRST;
           address <= car_config::LINE_SENSOR_I2C_ADDRESS_LAST; ++address) {
        if (probe(address) && ping(address)) { address_ = address; break; }
      }
      if (address_ == 0) delay(car_config::LINE_SENSOR_STARTUP_RETRY_MS);
    }
    if (address_ == 0) {
      Serial.println("[循迹] 未检测到感为八路灰度传感器（已扫描 0x4C~0x4F）");
      return false;
    }

    uint8_t firmware = 0;
    if (!readCommand(kCommandFirmware, &firmware, 1)) {
      Serial.printf("[循迹] 传感器 0x%02X 固件版本读取失败\n", address_);
      return false;
    }
    const uint8_t firmware_major = firmware >> 4;
    const uint8_t firmware_minor = firmware & 0x0F;
    normalization_enabled_ = car_config::LINE_SENSOR_ENABLE_NORMALIZATION &&
                             (firmware_major > 3 || (firmware_major == 3 && firmware_minor >= 6));
    if (normalization_enabled_ && !writeCommand(kCommandNormalization, 0xFF)) {
      Serial.println("[循迹] 八通道归一化配置失败");
      return false;
    }
    if (!writeCommand(kCommandChannelEnable, 0xFF) || !selectCommand(kCommandContinuousAnalog)) {
      Serial.println("[循迹] 八通道连续模拟量模式配置失败");
      return false;
    }

    ready_ = true;
    Serial.printf("[循迹] 感为八路 I2C 已就绪，地址=0x%02X，固件=V%u.%u，归一化=%s\n",
                  address_, firmware_major, firmware_minor, normalization_enabled_ ? "开启" : "关闭");
    return true;
  }

  LineReading read() {
    uint8_t raw[car_config::LINE_SENSOR_COUNT];
    // 每帧重发 0xB0，使传感器复位或上次读取中断后仍从第 1 路重新对齐。
    if (!ready_ || !readCommand(kCommandContinuousAnalog, raw, sizeof(raw))) {
      return {0.0F, 0.0F, false};
    }

    float weighted_sum = 0.0F;
    float strength_sum = 0.0F;
    constexpr size_t count = car_config::LINE_SENSOR_COUNT;
    for (size_t i = 0; i < count; ++i) {
      const size_t channel = car_config::LINE_SENSOR_CHANNELS_REVERSED ? count - 1 - i : i;
      float value = raw[channel] / 255.0F;
      // 手册规定模拟量白色趋近 255、黑色趋近 0。
      if (car_config::LINE_IS_DARK) value = 1.0F - value;
      const float position = count > 1 ? (2.0F * static_cast<float>(i) / (count - 1)) - 1.0F : 0.0F;
      weighted_sum += position * value;
      strength_sum += value;
    }
    const float average_strength = strength_sum / count;
    return {strength_sum > 0.001F ? weighted_sum / strength_sum : 0.0F,
            average_strength, average_strength >= car_config::MIN_LINE_STRENGTH};
  }

 private:
  static constexpr uint8_t kCommandContinuousAnalog = 0xB0;
  static constexpr uint8_t kCommandChannelEnable = 0xCE;
  static constexpr uint8_t kCommandNormalization = 0xCF;
  static constexpr uint8_t kCommandPing = 0xAA;
  static constexpr uint8_t kCommandFirmware = 0xC1;

  static bool probe(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission(true) == 0;
  }

  bool ping(uint8_t address) {
    uint8_t response = 0;
    return readCommandAt(address, kCommandPing, &response, 1) && response == 0x66;
  }

  bool readCommand(uint8_t command, uint8_t *output, size_t length) {
    return readCommandAt(address_, command, output, length);
  }

  static bool readCommandAt(uint8_t address, uint8_t command, uint8_t *output, size_t length) {
    Wire.beginTransmission(address);
    Wire.write(command);
    if (Wire.endTransmission(false) != 0) return false;
    const size_t received = Wire.requestFrom(address, length, true);
    if (received != length) {
      while (Wire.available() > 0) Wire.read();
      return false;
    }
    for (size_t i = 0; i < length; ++i) {
      if (Wire.available() <= 0) return false;
      output[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
  }

  bool writeCommand(uint8_t command, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(command);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
  }

  bool selectCommand(uint8_t command) {
    Wire.beginTransmission(address_);
    Wire.write(command);
    return Wire.endTransmission(true) == 0;
  }

  uint8_t address_ = 0;
  bool ready_ = false;
  bool normalization_enabled_ = false;
};

class MotorDriver {
 public:
  bool begin() {
    pinMode(car_config::LEFT_MOTOR_IN1_PIN, OUTPUT); pinMode(car_config::LEFT_MOTOR_IN2_PIN, OUTPUT);
    pinMode(car_config::RIGHT_MOTOR_IN1_PIN, OUTPUT); pinMode(car_config::RIGHT_MOTOR_IN2_PIN, OUTPUT);
    const bool left_ok = ledcAttach(car_config::LEFT_MOTOR_PWM_PIN, car_config::PWM_FREQUENCY_HZ, car_config::PWM_RESOLUTION_BITS);
    const bool right_ok = ledcAttach(car_config::RIGHT_MOTOR_PWM_PIN, car_config::PWM_FREQUENCY_HZ, car_config::PWM_RESOLUTION_BITS);
    brake();
    return left_ok && right_ok;
  }

  void drive(float left, float right) {
    setOne(car_config::LEFT_MOTOR_PWM_PIN, car_config::LEFT_MOTOR_IN1_PIN, car_config::LEFT_MOTOR_IN2_PIN, left);
    setOne(car_config::RIGHT_MOTOR_PWM_PIN, car_config::RIGHT_MOTOR_IN1_PIN, car_config::RIGHT_MOTOR_IN2_PIN, right);
  }

  void brake() {
    ledcWrite(car_config::LEFT_MOTOR_PWM_PIN, 0); ledcWrite(car_config::RIGHT_MOTOR_PWM_PIN, 0);
    const uint8_t level = car_config::MOTOR_BRAKE_HIGH ? HIGH : LOW;
    digitalWrite(car_config::LEFT_MOTOR_IN1_PIN, level); digitalWrite(car_config::LEFT_MOTOR_IN2_PIN, level);
    digitalWrite(car_config::RIGHT_MOTOR_IN1_PIN, level); digitalWrite(car_config::RIGHT_MOTOR_IN2_PIN, level);
  }

 private:
  static void setOne(uint8_t pwm_pin, uint8_t in1, uint8_t in2, float command) {
    command = constrain(command, -1.0F, 1.0F);
    const bool forward = command >= 0.0F;
    digitalWrite(in1, forward ? HIGH : LOW); digitalWrite(in2, forward ? LOW : HIGH);
    const uint32_t maximum = (1UL << car_config::PWM_RESOLUTION_BITS) - 1UL;
    ledcWrite(pwm_pin, static_cast<uint32_t>(fabsf(command) * maximum));
  }
};

class Encoders {
 public:
  void begin() {
    instance_ = this;
    pinMode(car_config::LEFT_ENCODER_A_PIN, INPUT_PULLUP); pinMode(car_config::LEFT_ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(car_config::RIGHT_ENCODER_A_PIN, INPUT_PULLUP); pinMode(car_config::RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(car_config::LEFT_ENCODER_A_PIN), leftInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(car_config::RIGHT_ENCODER_A_PIN), rightInterrupt, CHANGE);
  }

  void snapshot(int32_t &left, int32_t &right) const {
    noInterrupts(); left = left_count_; right = right_count_; interrupts();
    if (car_config::LEFT_ENCODER_INVERTED) left = -left;
    if (car_config::RIGHT_ENCODER_INVERTED) right = -right;
  }

 private:
#if defined(ARDUINO_ARCH_ESP32)
  // 实机 ISR 放在 Hardware.cpp，避免头文件内联函数的 IRAM 文字池落到代码之后。
  static void IRAM_ATTR leftInterrupt();
  static void IRAM_ATTR rightInterrupt();
  static Encoders *instance_;
#else
  // 主机桩没有独立 Arduino 编译单元，保留等价的内联实现供回归测试链接。
  static void IRAM_ATTR leftInterrupt() {
    if (instance_ == nullptr) return;
    instance_->left_count_ += digitalRead(car_config::LEFT_ENCODER_A_PIN) == digitalRead(car_config::LEFT_ENCODER_B_PIN) ? 1 : -1;
  }
  static void IRAM_ATTR rightInterrupt() {
    if (instance_ == nullptr) return;
    instance_->right_count_ += digitalRead(car_config::RIGHT_ENCODER_A_PIN) == digitalRead(car_config::RIGHT_ENCODER_B_PIN) ? 1 : -1;
  }

  inline static Encoders *instance_ = nullptr;
#endif
  volatile int32_t left_count_ = 0;
  volatile int32_t right_count_ = 0;
};

class StartButton {
 public:
  void begin(uint32_t now_ms) {
    pinMode(car_config::START_BUTTON_PIN, car_config::BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    raw_pressed_ = readRaw(); stable_pressed_ = raw_pressed_; raw_changed_ms_ = now_ms;
    pressed_since_ms_ = raw_pressed_ ? now_ms : 0;
  }

  void update(uint32_t now_ms) {
    const bool current = readRaw();
    if (current != raw_pressed_) { raw_pressed_ = current; raw_changed_ms_ = now_ms; }
    if (current != stable_pressed_ && now_ms - raw_changed_ms_ >= car_config::BUTTON_DEBOUNCE_MS) {
      stable_pressed_ = current;
      if (stable_pressed_) { pressed_edge_ = true; pressed_since_ms_ = now_ms; }
      else pressed_since_ms_ = 0;
    }
  }

  bool consumePress() { const bool result = pressed_edge_; pressed_edge_ = false; return result; }
  bool isStuck(uint32_t now_ms) const { return stable_pressed_ && now_ms - pressed_since_ms_ >= car_config::BUTTON_STUCK_MS; }

 private:
  bool readRaw() const { return digitalRead(car_config::START_BUTTON_PIN) == (car_config::BUTTON_ACTIVE_LOW ? LOW : HIGH); }
  bool raw_pressed_ = false;
  bool stable_pressed_ = false;
  bool pressed_edge_ = false;
  uint32_t raw_changed_ms_ = 0;
  uint32_t pressed_since_ms_ = 0;
};
