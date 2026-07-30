#pragma once

#include <Arduino.h>
#include <DTaskProtocol.h>

#include "Config.h"
#include "Hardware.h"

class LineFollower {
 public:
  struct NavigationData {
    d_task::CarState state;
    d_task::TurnClass turn;
    bool line_valid;
    float line_error;
    float line_strength;
    float left_command;
    float right_command;
  };

  bool begin(uint32_t now_ms) {
    const bool motors_ok = motors_.begin();
    const bool sensor_ok = sensors_.begin();
    if (!motors_ok || !sensor_ok) {
      fault_flags_ = (!motors_ok ? d_task::FAULT_MOTOR : 0) |
                     (!sensor_ok ? d_task::FAULT_LINE_LOST : 0);
      motors_.brake();
      Serial.println("[循迹] 初始化失败，电机保持制动");
      return false;
    }

    ready_ = true;
    start_ms_ = now_ms + kStartupDelayMs;
    next_control_us_ = micros();
    Serial.println("[循迹] 黑线跟随已就绪，1.5 秒后开始");
    return true;
  }

  void update(uint32_t now_ms, uint32_t now_us) {
    if (!ready_) return;
    if (static_cast<int32_t>(now_ms - start_ms_) < 0) {
      motors_.brake();
      return;
    }
    if (!running_) {
      running_ = true;
      event_ = d_task::RouteEvent::START;
      ++event_id_;
      event_transmissions_remaining_ = car_config::EVENT_REPEAT_FRAMES;
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

  NavigationData navigationData() const {
    d_task::CarState state = d_task::CarState::READY;
    if (fault_flags_ != d_task::FAULT_NONE) state = d_task::CarState::SAFE_STOP;
    else if (running_) state = d_task::CarState::RUNNING;

    const float magnitude = fabsf(last_error_);
    d_task::TurnClass turn = d_task::TurnClass::STRAIGHT;
    if (magnitude >= car_config::LARGE_TURN_ENTER) turn = d_task::TurnClass::LARGE;
    else if (magnitude >= car_config::SMALL_TURN_ENTER) turn = d_task::TurnClass::SMALL;

    return {state, turn, line_was_valid_, last_error_, last_strength_,
            left_command_, right_command_};
  }

  d_task::CarTelemetry telemetry(bool wifi_connected) const {
    const NavigationData navigation = navigationData();
    uint16_t quality_flags = 0;
    if (navigation.line_valid) quality_flags |= d_task::QUALITY_LINE_VALID;
    if (wifi_connected) quality_flags |= d_task::QUALITY_WIFI_CONNECTED;
    return {navigation.state, navigation.turn, event_, event_id_, quality_flags,
            0, 0,
            static_cast<int16_t>(constrain(navigation.line_error * 1000.0F,
                                           -1000.0F, 1000.0F)),
            fault_flags_};
  }

  void noteTelemetryTransmitted() {
    if (event_ == d_task::RouteEvent::NONE || event_transmissions_remaining_ == 0) return;
    if (--event_transmissions_remaining_ == 0) event_ = d_task::RouteEvent::NONE;
  }

 private:
  void follow(const LineReading &line, uint32_t now_ms) {
    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    const bool reacquired = !line_was_valid_;
    if (reacquired) {
      previous_error_ = line.error;
      derivative_ = 0.0F;
      integral_ = 0.0F;
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
    last_strength_ = line.strength;
    left_command_ = left;
    right_command_ = right;
    last_line_ms_ = now_ms;
    line_was_valid_ = true;
  }

  void recoverOrStop(uint32_t now_ms) {
    integral_ = 0.0F;
    derivative_ = 0.0F;
    line_was_valid_ = false;
    last_strength_ = 0.0F;

    const bool recently_seen = last_line_ms_ != 0 && now_ms - last_line_ms_ <= kRecoveryTimeMs;
    if (recently_seen && fabsf(last_error_) >= kRecoveryErrorThreshold) {
      // 按最后一次看到黑线的方向原地找线，适合传感器短暂越过急弯。
      const float direction = last_error_ > 0.0F ? 1.0F : -1.0F;
      left_command_ = direction * kRecoveryOuterSpeed;
      right_command_ = -direction * kRecoveryInnerSpeed;
      motors_.drive(left_command_, right_command_);
      return;
    }

    left_command_ = 0.0F;
    right_command_ = 0.0F;
    motors_.brake();
  }

  static constexpr uint32_t kStartupDelayMs = 1500;
  static constexpr uint32_t kRecoveryTimeMs = 350;
  static constexpr float kCurveSlowdown = 0.45F;
  static constexpr float kMaximumCorrection = 0.65F;
  static constexpr float kDerivativeFilterAlpha = 0.20F;
  static constexpr float kRecoveryErrorThreshold = 0.08F;
  static constexpr float kRecoveryOuterSpeed = 0.28F;
  static constexpr float kRecoveryInnerSpeed = 0.18F;

  LineSensors sensors_;
  MotorDriver motors_;
  bool ready_ = false;
  bool running_ = false;
  bool line_was_valid_ = false;
  float integral_ = 0.0F;
  float derivative_ = 0.0F;
  float previous_error_ = 0.0F;
  float last_error_ = 0.0F;
  float last_strength_ = 0.0F;
  float left_command_ = 0.0F;
  float right_command_ = 0.0F;
  d_task::RouteEvent event_ = d_task::RouteEvent::NONE;
  uint16_t event_id_ = 0;
  uint8_t event_transmissions_remaining_ = 0;
  uint16_t fault_flags_ = d_task::FAULT_NONE;
  uint32_t start_ms_ = 0;
  uint32_t last_line_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
