#pragma once

#include <Arduino.h>
#include <DTaskProtocol.h>

#include "Config.h"
#include "FinishLineStop.h"
#include "Hardware.h"
#include "LineSteering.h"

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
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    float displacement_m;
    float velocity_m_s;
  };

  bool begin(uint32_t now_ms) {
    const bool motors_ok = motors_.begin();
    const bool sensor_ok = sensors_.begin();
    encoders_.begin();
    encoders_.snapshot(last_left_encoder_count_, last_right_encoder_count_);
    last_odometry_ms_ = now_ms;
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
    if (completed_) {
      motors_.brake();
      return;
    }
    if (static_cast<int32_t>(now_ms - start_ms_) < 0) {
      motors_.brake();
      return;
    }
    if (!running_) {
      running_ = true;
      encoders_.snapshot(last_left_encoder_count_, last_right_encoder_count_);
      last_odometry_ms_ = now_ms;
      displacement_m_ = 0.0F;
      velocity_m_s_ = 0.0F;
      completed_ = false;
      finish_line_stop_.reset();
      speed_integral_ = 0.0F;
      speed_command_ = car_config::SPEED_FEED_FORWARD_COMMAND;
      event_ = d_task::RouteEvent::START;
      ++event_id_;
      event_transmissions_remaining_ = car_config::EVENT_REPEAT_FRAMES;
      start_announced_ = false;  // 通知主循环立即发送 START 遥测
      Serial.printf("[启动] 小车进入运行状态 event_id=%u\n", event_id_);
    }
    updateOdometry(now_ms);
    if (static_cast<int32_t>(now_us - next_control_us_) < 0) return;
    next_control_us_ = now_us + car_config::CONTROL_PERIOD_US;

    LineSensorFrame frame{};
    const bool frame_read = sensors_.readFrame(frame);
    if (frame_read) {
      for (size_t i = 0; i < car_config::LINE_SENSOR_COUNT; ++i) {
        last_line_channels_[i] = frame.channels[i];
      }
    }
    if (frame_read && frame.line.valid) {
      const bool all_channels_black = LineSensors::allChannelsOnLine(frame.channels);
      if (finish_line_stop_.update(displacement_m_, all_channels_black, now_ms)) {
        complete();
        return;
      }
      follow(frame.line, now_ms);
    } else {
      finish_line_stop_.update(displacement_m_, false, now_ms);
      recoverOrStop(now_ms);
    }
  }

  NavigationData navigationData() const {
    d_task::CarState state = d_task::CarState::READY;
    if (fault_flags_ != d_task::FAULT_NONE) state = d_task::CarState::SAFE_STOP;
    else if (completed_) state = d_task::CarState::COMPLETE;
    else if (running_) state = d_task::CarState::RUNNING;

    const float magnitude = fabsf(last_error_);
    d_task::TurnClass turn = d_task::TurnClass::STRAIGHT;
    if (magnitude >= car_config::LARGE_TURN_ENTER) turn = d_task::TurnClass::LARGE;
    else if (magnitude >= car_config::SMALL_TURN_ENTER) turn = d_task::TurnClass::SMALL;

    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    encoders_.snapshot(left_encoder_count, right_encoder_count);
    return {state, turn, line_was_valid_, last_error_, last_strength_,
            left_command_, right_command_, left_encoder_count, right_encoder_count,
            displacement_m_, velocity_m_s_};
  }

  d_task::CarTelemetry telemetry(bool wifi_connected) const {
    const NavigationData navigation = navigationData();
    uint16_t quality_flags = 0;
    if (navigation.line_valid) quality_flags |= d_task::QUALITY_LINE_VALID;
    if (encoder_initialized_) quality_flags |= d_task::QUALITY_ENCODER_VALID;
    if (wifi_connected) quality_flags |= d_task::QUALITY_WIFI_CONNECTED;
    return {navigation.state, navigation.turn, event_, event_id_, quality_flags,
            static_cast<int32_t>(constrain(navigation.displacement_m * 1000.0F,
                                           -2147483.0F, 2147483.0F)),
            static_cast<int16_t>(constrain(navigation.velocity_m_s * 1000.0F,
                                           -32768.0F, 32767.0F)),
            static_cast<int16_t>(constrain(navigation.line_error * 1000.0F,
                                           -1000.0F, 1000.0F)),
            fault_flags_};
  }

  void noteTelemetryTransmitted() {
    if (event_ == d_task::RouteEvent::NONE || event_transmissions_remaining_ == 0) return;
    if (--event_transmissions_remaining_ == 0) event_ = d_task::RouteEvent::NONE;
  }

  bool consumeStartAnnouncement() {
    if (start_announced_) return false;
    start_announced_ = true;
    return true;
  }

  void lineChannels(uint8_t channels[car_config::LINE_SENSOR_COUNT]) const {
    for (size_t i = 0; i < car_config::LINE_SENSOR_COUNT; ++i) {
      channels[i] = last_line_channels_[i];
    }
  }

 private:
  void complete() {
    if (completed_) return;
    completed_ = true;
    running_ = false;
    velocity_m_s_ = 0.0F;
    left_command_ = 0.0F;
    right_command_ = 0.0F;
    motors_.brake();
    event_ = d_task::RouteEvent::COMPLETE;
    ++event_id_;
    event_transmissions_remaining_ = car_config::EVENT_REPEAT_FRAMES;
    Serial.printf("[finish] finish line passed by %.2f m; car stopped (event_id=%u)\n",
                  car_config::FINISH_LINE_RUNOUT_M, event_id_);
  }

  void updateOdometry(uint32_t now_ms) {
    if (now_ms - last_odometry_ms_ < car_config::ENCODER_PERIOD_MS) return;

    int32_t left_count = 0;
    int32_t right_count = 0;
    encoders_.snapshot(left_count, right_count);
    const int32_t left_delta = left_count - last_left_encoder_count_;
    const int32_t right_delta = right_count - last_right_encoder_count_;
    const float meters_per_count = PI * car_config::WHEEL_DIAMETER_M /
                                   car_config::ENCODER_COUNTS_PER_REVOLUTION;
    const float left_delta_m = left_delta * meters_per_count;
    const float right_delta_m = right_delta * meters_per_count;
    const float center_delta_m = (left_delta_m + right_delta_m) * 0.5F;
    const float seconds = (now_ms - last_odometry_ms_) / 1000.0F;

    displacement_m_ += center_delta_m;
    if (seconds > 0.0F) {
      const float measured_velocity_m_s = center_delta_m / seconds;
      velocity_m_s_ += car_config::VELOCITY_FILTER_ALPHA *
                       (measured_velocity_m_s - velocity_m_s_);
    }
    last_left_encoder_count_ = left_count;
    last_right_encoder_count_ = right_count;
    last_odometry_ms_ = now_ms;
    encoder_initialized_ = true;
    odometry_updated_ = true;
  }

  void updateSpeedControl() {
    constexpr float kOdometryPeriodS = car_config::ENCODER_PERIOD_MS / 1000.0F;
    const float speed_error = car_config::TARGET_SPEED_M_S - velocity_m_s_;
    speed_integral_ = constrain(speed_integral_ + speed_error * kOdometryPeriodS,
                                -car_config::SPEED_INTEGRAL_LIMIT,
                                car_config::SPEED_INTEGRAL_LIMIT);
    speed_command_ = constrain(
        car_config::SPEED_FEED_FORWARD_COMMAND +
            car_config::SPEED_KP * speed_error +
            car_config::SPEED_KI * speed_integral_,
        0.0F, car_config::MAX_BASE_MOTOR_COMMAND);
  }

  void follow(const LineReading &line, uint32_t now_ms) {
    if (odometry_updated_) {
      updateSpeedControl();
      odometry_updated_ = false;
    }
    const bool reacquired = !line_was_valid_;
    if (reacquired) steering_.reset(line.error);
    const LineSteering::Output steering = steering_.update(line.error);

    // 偏离中心时同时提高比例增益并降速，让转向量相对前进速度足够大。
    const float base = speed_command_ * steering.base_speed_ratio;
    float left = constrain(base + steering.correction, -1.0F, 1.0F);
    float right = constrain(base - steering.correction, -1.0F, 1.0F);
    MotorDriver::limitCommandDifference(left, right);
    motors_.drive(left, right);

    last_error_ = steering.error;
    last_strength_ = line.strength;
    left_command_ = left;
    right_command_ = right;
    last_line_ms_ = now_ms;
    line_was_valid_ = true;
  }

  void recoverOrStop(uint32_t now_ms) {
    steering_.clear();
    line_was_valid_ = false;
    last_strength_ = 0.0F;

    const bool recently_seen = last_line_ms_ != 0 && now_ms - last_line_ms_ <= kRecoveryTimeMs;
    if (recently_seen && fabsf(last_error_) >= kRecoveryErrorThreshold) {
      // 按最后一次看到黑线的方向找线，最终差速仍受电机入口硬限制。
      const float direction = last_error_ > 0.0F ? 1.0F : -1.0F;
      left_command_ = direction * kRecoveryOuterSpeed;
      right_command_ = -direction * kRecoveryInnerSpeed;
      MotorDriver::limitCommandDifference(left_command_, right_command_);
      motors_.drive(left_command_, right_command_);
      return;
    }

    left_command_ = 0.0F;
    right_command_ = 0.0F;
    motors_.brake();
  }

  static constexpr uint32_t kStartupDelayMs = 1500;
  static constexpr uint32_t kRecoveryTimeMs = 350;
  static constexpr float kRecoveryErrorThreshold = 0.08F;
  static constexpr float kRecoveryOuterSpeed = 0.28F;
  static constexpr float kRecoveryInnerSpeed = 0.18F;

  LineSensors sensors_;
  MotorDriver motors_;
  Encoders encoders_;
  FinishLineStop finish_line_stop_;
  LineSteering steering_;
  bool ready_ = false;
  bool running_ = false;
  bool completed_ = false;
  bool line_was_valid_ = false;
  float last_error_ = 0.0F;
  float last_strength_ = 0.0F;
  float left_command_ = 0.0F;
  float right_command_ = 0.0F;
  float displacement_m_ = 0.0F;
  float velocity_m_s_ = 0.0F;
  float speed_integral_ = 0.0F;
  float speed_command_ = car_config::SPEED_FEED_FORWARD_COMMAND;
  uint8_t last_line_channels_[car_config::LINE_SENSOR_COUNT] = {};
  int32_t last_left_encoder_count_ = 0;
  int32_t last_right_encoder_count_ = 0;
  uint32_t last_odometry_ms_ = 0;
  bool encoder_initialized_ = false;
  bool odometry_updated_ = false;
  bool start_announced_ = true;  // begin() 时尚未启动，设为 true 避免误触发
  d_task::RouteEvent event_ = d_task::RouteEvent::NONE;
  uint16_t event_id_ = 0;
  uint8_t event_transmissions_remaining_ = 0;
  uint16_t fault_flags_ = d_task::FAULT_NONE;
  uint32_t start_ms_ = 0;
  uint32_t last_line_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
