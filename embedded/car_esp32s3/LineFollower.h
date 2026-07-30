#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Hardware.h"

class LineFollower {
 public:
  bool begin(uint32_t now_ms) {
    const bool motors_ok = motors_.begin();
    const bool sensor_ok = sensors_.begin();
    if (!motors_ok || !sensor_ok) {
      motors_.brake();
      Serial.println("[循迹] 初始化失败，电机保持制动");
      return false;
    }

    ready_ = true;
    start_ms_ = now_ms + kStartupDelayMs;
    next_control_us_ = micros();
    next_print_ms_ = now_ms;
    Serial.println("[循迹] 黑线跟随已就绪，1.5 秒后开始");
    return true;
  }

  void update(uint32_t now_ms, uint32_t now_us) {
    if (!ready_) return;
    if (static_cast<int32_t>(now_ms - start_ms_) < 0) {
      motors_.brake();
      return;
    }
    if (static_cast<int32_t>(now_us - next_control_us_) < 0) return;
    next_control_us_ = now_us + car_config::CONTROL_PERIOD_US;

    LineSensorFrame frame{};
    if (sensors_.readFrame(frame) && frame.line.valid) {
      follow(frame.line, now_ms);
    } else {
      recoverOrStop(now_ms);
    }
  }

 private:
  void follow(const LineReading &line, uint32_t now_ms) {
    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    const bool reacquired = !line_was_valid_;
    if (reacquired) {
      previous_error_ = line.error;
      derivative_ = 0.0F;
      integral_ = 0.0F;
      Serial.println("[循迹] 已识别黑线");
    }

    integral_ = constrain(integral_ + line.error * dt,
                          -car_config::PID_INTEGRAL_LIMIT,
                          car_config::PID_INTEGRAL_LIMIT);
    const float raw_derivative = (line.error - previous_error_) / dt;
    derivative_ += kDerivativeFilterAlpha * (raw_derivative - derivative_);
    previous_error_ = line.error;

    float correction = car_config::PID_KP * line.error +
                       car_config::PID_KI * integral_ +
                       car_config::PID_KD * derivative_;
    correction = constrain(correction, -kMaximumCorrection, kMaximumCorrection);

    // 弯道时主动降速；误差较大时允许内侧轮反转，以通过急弯。
    const float curve = constrain(fabsf(line.error), 0.0F, 1.0F);
    const float base = car_config::BASE_MOTOR_COMMAND * (1.0F - kCurveSlowdown * curve);
    const float left = constrain(base + correction, -1.0F, 1.0F);
    const float right = constrain(base - correction, -1.0F, 1.0F);
    motors_.drive(left, right);

    last_error_ = line.error;
    last_line_ms_ = now_ms;
    line_was_valid_ = true;
    if (static_cast<int32_t>(now_ms - next_print_ms_) >= 0) {
      Serial.printf("[循迹] error=%+.3f strength=%.3f left=%+.2f right=%+.2f\n",
                    line.error, line.strength, left, right);
      next_print_ms_ = now_ms + kPrintPeriodMs;
    }
  }

  void recoverOrStop(uint32_t now_ms) {
    integral_ = 0.0F;
    derivative_ = 0.0F;
    line_was_valid_ = false;

    const bool recently_seen = last_line_ms_ != 0 && now_ms - last_line_ms_ <= kRecoveryTimeMs;
    if (recently_seen && fabsf(last_error_) >= kRecoveryErrorThreshold) {
      // 按最后一次看到黑线的方向原地找线，适合传感器短暂越过急弯。
      const float direction = last_error_ > 0.0F ? 1.0F : -1.0F;
      motors_.drive(direction * kRecoveryOuterSpeed,
                    -direction * kRecoveryInnerSpeed);
      return;
    }

    motors_.brake();
    if (static_cast<int32_t>(now_ms - next_print_ms_) >= 0) {
      Serial.println("[循迹] 未识别到黑线，电机已制动");
      next_print_ms_ = now_ms + kPrintPeriodMs;
    }
  }

  static constexpr uint32_t kStartupDelayMs = 1500;
  static constexpr uint32_t kRecoveryTimeMs = 350;
  static constexpr uint32_t kPrintPeriodMs = 200;
  static constexpr float kCurveSlowdown = 0.45F;
  static constexpr float kMaximumCorrection = 0.65F;
  static constexpr float kDerivativeFilterAlpha = 0.20F;
  static constexpr float kRecoveryErrorThreshold = 0.08F;
  static constexpr float kRecoveryOuterSpeed = 0.28F;
  static constexpr float kRecoveryInnerSpeed = 0.18F;

  LineSensors sensors_;
  MotorDriver motors_;
  bool ready_ = false;
  bool line_was_valid_ = false;
  float integral_ = 0.0F;
  float derivative_ = 0.0F;
  float previous_error_ = 0.0F;
  float last_error_ = 0.0F;
  uint32_t start_ms_ = 0;
  uint32_t last_line_ms_ = 0;
  uint32_t next_print_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
