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
      speed_integral_ = 0.0F;
      speed_command_ = car_config::SPEED_FEED_FORWARD_COMMAND;
      event_ = d_task::RouteEvent::START;
      ++event_id_;
      event_transmissions_remaining_ = car_config::EVENT_REPEAT_FRAMES;
    }
    updateOdometry(now_ms);
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

 private:
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
    if (odometry_updated_) {
      updateSpeedControl();
      odometry_updated_ = false;
    }
    const float base = speed_command_ * (1.0F - kCurveSlowdown * curve);
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
  Encoders encoders_;
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
  float displacement_m_ = 0.0F;
  float velocity_m_s_ = 0.0F;
  float speed_integral_ = 0.0F;
  float speed_command_ = car_config::SPEED_FEED_FORWARD_COMMAND;
  int32_t last_left_encoder_count_ = 0;
  int32_t last_right_encoder_count_ = 0;
  uint32_t last_odometry_ms_ = 0;
  bool encoder_initialized_ = false;
  bool odometry_updated_ = false;
  d_task::RouteEvent event_ = d_task::RouteEvent::NONE;
  uint16_t event_id_ = 0;
  uint8_t event_transmissions_remaining_ = 0;
  uint16_t fault_flags_ = d_task::FAULT_NONE;
  uint32_t start_ms_ = 0;
  uint32_t last_line_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
