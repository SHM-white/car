#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Hardware.h"

class LineFollower {
 public:
  bool begin(uint32_t now_ms) {
    const bool motors_ok = motors_.begin();
    const bool sensor_ok = sensors_.begin();
    encoders_.begin();
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
    const bool frame_ok = sensors_.readFrame(frame);
    if (frame_ok && frame.line.valid) {
      follow(frame, now_ms);
    } else {
      recoverOrStop(now_ms, frame_ok ? &frame : nullptr);
    }
  }

 private:
  void follow(const LineSensorFrame &frame, uint32_t now_ms) {
    const LineReading &line = frame.line;
    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    const bool reacquired = !line_was_valid_;
    if (reacquired) {
      previous_error_ = line.error;
      derivative_ = 0.0F;
      integral_ = 0.0F;
      Serial.println("[循迹] 已识别黑线");
    }

    float correction = 0.0F;
    if (line.strength >= kFullCoverageStrength) {
      // 黑色覆盖整条探头时没有横向位置信息，清除历史转向并等速通过。
      integral_ = 0.0F;
      derivative_ = 0.0F;
      previous_error_ = 0.0F;
    } else {
      integral_ = constrain(integral_ + line.error * dt,
                            -car_config::PID_INTEGRAL_LIMIT,
                            car_config::PID_INTEGRAL_LIMIT);
      const float raw_derivative = (line.error - previous_error_) / dt;
      derivative_ += kDerivativeFilterAlpha * (raw_derivative - derivative_);
      previous_error_ = line.error;

      correction = car_config::PID_KP * line.error +
                   car_config::PID_KI * integral_ +
                   car_config::PID_KD * derivative_;
      correction = constrain(correction, -car_config::MAX_STEERING_CORRECTION,
                             car_config::MAX_STEERING_CORRECTION);
    }

    const float base = updateSpeedControl(dt);
    // 常规循迹不允许内侧轮反转，避免黑线稍偏时原地急转。
    const float left = constrain(base + correction, 0.0F, 1.0F);
    const float right = constrain(base - correction, 0.0F, 1.0F);
    motors_.drive(left, right);

    last_error_ = line.error;
    last_line_ms_ = now_ms;
    line_was_valid_ = true;
    if (static_cast<int32_t>(now_ms - next_print_ms_) >= 0) {
      const bool encoder_seen = encoder_ticks_since_print_left_ != 0 ||
                                encoder_ticks_since_print_right_ != 0;
      constexpr float kCenterChannel = (car_config::LINE_SENSOR_COUNT + 1) * 0.5F;
      constexpr float kChannelsPerError = (car_config::LINE_SENSOR_COUNT - 1) * 0.5F;
      const float offset_channels = line.error * kChannelsPerError;
      Serial.printf("[循迹] error=%+.3f offset=%+.2fch center=%.2fch strength=%.3f speed=%.3fm/s target=%.3fm/s encoder=%s ticks=%ld/%ld left=%+.2f right=%+.2f\n",
                    line.error, offset_channels, kCenterChannel + offset_channels,
                    line.strength, measured_speed_m_s_, car_config::TARGET_SPEED_M_S,
                    encoder_seen ? "ok" : "no-count",
                    static_cast<long>(encoder_ticks_since_print_left_),
                    static_cast<long>(encoder_ticks_since_print_right_), left, right);
      encoder_ticks_since_print_left_ = 0;
      encoder_ticks_since_print_right_ = 0;
      printChannels(frame);
      next_print_ms_ = now_ms + kPrintPeriodMs;
    }
  }

  void recoverOrStop(uint32_t now_ms, const LineSensorFrame *frame) {
    integral_ = 0.0F;
    derivative_ = 0.0F;
    line_was_valid_ = false;
    resetSpeedControl();

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
      if (frame != nullptr) printChannels(*frame);
      next_print_ms_ = now_ms + kPrintPeriodMs;
    }
  }

  static void printChannels(const LineSensorFrame &frame) {
    Serial.printf("[传感器] ch1..ch8=%u,%u,%u,%u,%u,%u,%u,%u（黑低，白高）\n",
                  static_cast<unsigned>(frame.channels[0]),
                  static_cast<unsigned>(frame.channels[1]),
                  static_cast<unsigned>(frame.channels[2]),
                  static_cast<unsigned>(frame.channels[3]),
                  static_cast<unsigned>(frame.channels[4]),
                  static_cast<unsigned>(frame.channels[5]),
                  static_cast<unsigned>(frame.channels[6]),
                  static_cast<unsigned>(frame.channels[7]));
  }

  float updateSpeedControl(float dt) {
    int32_t left_count = 0;
    int32_t right_count = 0;
    encoders_.snapshot(left_count, right_count);
    if (!speed_sample_ready_) {
      last_left_count_ = left_count;
      last_right_count_ = right_count;
      speed_sample_ready_ = true;
      return car_config::SPEED_FEED_FORWARD_COMMAND;
    }

    const int32_t left_delta = left_count - last_left_count_;
    const int32_t right_delta = right_count - last_right_count_;
    last_left_count_ = left_count;
    last_right_count_ = right_count;
    encoder_ticks_since_print_left_ += left_delta;
    encoder_ticks_since_print_right_ += right_delta;
    if (left_delta == 0 && right_delta == 0) {
      // 编码器没有反馈时保持保守开环命令，禁止 PI 因“速度为零”持续加速。
      measured_speed_m_s_ += kSpeedFilterAlpha * (0.0F - measured_speed_m_s_);
      return car_config::SPEED_FEED_FORWARD_COMMAND;
    }
    const float meters_per_count = PI * car_config::WHEEL_DIAMETER_M /
                                   car_config::ENCODER_COUNTS_PER_REVOLUTION;
    const float measured = (left_delta + right_delta) * 0.5F * meters_per_count / dt;
    measured_speed_m_s_ += kSpeedFilterAlpha * (measured - measured_speed_m_s_);

    const float error = car_config::TARGET_SPEED_M_S - measured_speed_m_s_;
    speed_integral_ = constrain(speed_integral_ + error * dt,
                                -car_config::SPEED_INTEGRAL_LIMIT,
                                car_config::SPEED_INTEGRAL_LIMIT);
    return constrain(car_config::SPEED_FEED_FORWARD_COMMAND +
                         car_config::SPEED_KP * error +
                         car_config::SPEED_KI * speed_integral_,
                     0.0F, car_config::MAX_BASE_MOTOR_COMMAND);
  }

  void resetSpeedControl() {
    speed_sample_ready_ = false;
    speed_integral_ = 0.0F;
    measured_speed_m_s_ = 0.0F;
  }

  static constexpr uint32_t kStartupDelayMs = 1500;
  static constexpr uint32_t kRecoveryTimeMs = 350;
  static constexpr uint32_t kPrintPeriodMs = 200;
  static constexpr float kFullCoverageStrength = 0.95F;
  static constexpr float kDerivativeFilterAlpha = 0.20F;
  static constexpr float kSpeedFilterAlpha = 0.25F;
  static constexpr float kRecoveryErrorThreshold = 0.08F;
  static constexpr float kRecoveryOuterSpeed = 0.28F;
  static constexpr float kRecoveryInnerSpeed = 0.18F;

  LineSensors sensors_;
  MotorDriver motors_;
  Encoders encoders_;
  bool ready_ = false;
  bool line_was_valid_ = false;
  float integral_ = 0.0F;
  float derivative_ = 0.0F;
  float previous_error_ = 0.0F;
  float last_error_ = 0.0F;
  float speed_integral_ = 0.0F;
  float measured_speed_m_s_ = 0.0F;
  int32_t last_left_count_ = 0;
  int32_t last_right_count_ = 0;
  int32_t encoder_ticks_since_print_left_ = 0;
  int32_t encoder_ticks_since_print_right_ = 0;
  bool speed_sample_ready_ = false;
  uint32_t start_ms_ = 0;
  uint32_t last_line_ms_ = 0;
  uint32_t next_print_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
