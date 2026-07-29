#pragma once

#include <Arduino.h>
#include <DTaskProtocol.h>

#include "Config.h"
#include "Hardware.h"

static_assert(car_config::EVENT_REPEAT_FRAMES > 0, "事件至少发送一帧");
static_assert(car_config::ROUTE_B_M < car_config::ROUTE_D_M &&
              car_config::ROUTE_D_M < car_config::ROUTE_A_M &&
              car_config::ROUTE_A_M < car_config::ROUTE_COMPLETE_M,
              "路线距离必须严格按 B、D、A、完成递增");
static_assert(car_config::VELOCITY_FILTER_ALPHA > 0.0F && car_config::VELOCITY_FILTER_ALPHA <= 1.0F,
              "速度滤波系数必须位于 (0, 1]");

class CarController {
 public:
  bool begin(uint32_t now_ms) {
    const bool line_sensor_ok = line_sensors_.begin();
    encoders_.begin(); button_.begin(now_ms);
    state_ = d_task::CarState::READY; last_encoder_ms_ = now_ms;
    const bool motors_ok = motors_.begin();
    if (!line_sensor_ok) fault_flags_ |= d_task::FAULT_LINE_LOST;
    if (!motors_ok) fault_flags_ |= d_task::FAULT_MOTOR;
    if (!line_sensor_ok || !motors_ok) {
      state_ = d_task::CarState::SAFE_STOP;
      motors_.brake();
    }
    return line_sensor_ok && motors_ok;
  }

  void update(uint32_t now_ms, uint32_t now_us, bool network_healthy) {
    button_.update(now_ms);
    if (button_.isStuck(now_ms)) { safeStop(d_task::FAULT_BUTTON_STUCK); return; }
    if (state_ == d_task::CarState::READY && button_.consumePress()) {
      state_ = d_task::CarState::RUNNING; queueEvent(d_task::RouteEvent::START);
      last_network_ok_ms_ = now_ms; next_control_us_ = now_us;
    }
    if (state_ != d_task::CarState::RUNNING) return;

    if (network_healthy) last_network_ok_ms_ = now_ms;
    if (now_ms - last_network_ok_ms_ > car_config::WIFI_LOSS_STOP_MS) safeStop(d_task::FAULT_WIFI_TIMEOUT);
    if (state_ != d_task::CarState::RUNNING) return;
    if (now_ms - last_encoder_ms_ >= car_config::ENCODER_PERIOD_MS) updateOdometry(now_ms);
    if (state_ != d_task::CarState::RUNNING) return;

    if (static_cast<int32_t>(now_us - next_control_us_) >= 0) {
      next_control_us_ += car_config::CONTROL_PERIOD_US;
      const uint32_t started_us = micros();
      runControl(now_ms);
      if (micros() - started_us > car_config::PID_OVERRUN_US) safeStop(d_task::FAULT_PID_OVERRUN);
      if (static_cast<int32_t>(now_us - next_control_us_) >= static_cast<int32_t>(car_config::CONTROL_PERIOD_US)) safeStop(d_task::FAULT_PID_OVERRUN);
    }
    updateRouteEvent();
  }

  d_task::CarTelemetry telemetry(bool wifi_connected) const {
    uint16_t quality = 0;
    if (line_valid_) quality |= d_task::QUALITY_LINE_VALID;
    if (encoder_valid_) quality |= d_task::QUALITY_ENCODER_VALID;
    if (wifi_connected) quality |= d_task::QUALITY_WIFI_CONNECTED;
    return {state_, turn_, event_, event_id_, quality,
            static_cast<int32_t>(displacement_m_ * 1000.0F),
            static_cast<int16_t>(constrain(velocity_m_s_ * 1000.0F, -32768.0F, 32767.0F)),
            static_cast<int16_t>(constrain(line_error_ * 1000.0F, -1000.0F, 1000.0F)), fault_flags_};
  }

  d_task::CarState state() const { return state_; }
  void noteTelemetryTransmitted() {
    if (event_ == d_task::RouteEvent::NONE || event_transmissions_remaining_ == 0) return;
    if (--event_transmissions_remaining_ == 0) event_ = d_task::RouteEvent::NONE;
  }
  void forceSafeStop(uint16_t fault) { safeStop(fault); }

 private:
  void runControl(uint32_t now_ms) {
    const LineReading line = line_sensors_.read(); line_error_ = line.error; line_valid_ = line.valid;
    if (!line.valid) {
      // 短暂丢线也先制动；只有在消抖窗口内重新识别到线才恢复驱动。
      motors_.brake();
      if (line_lost_since_ms_ == 0) line_lost_since_ms_ = now_ms;
      if (now_ms - line_lost_since_ms_ >= car_config::LINE_LOST_STOP_MS) safeStop(d_task::FAULT_LINE_LOST);
      return;
    }
    line_lost_since_ms_ = 0;
    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    integral_ = constrain(integral_ + line.error * dt, -car_config::PID_INTEGRAL_LIMIT, car_config::PID_INTEGRAL_LIMIT);
    const float derivative = (line.error - previous_error_) / dt;
    const float correction = car_config::PID_KP * line.error + car_config::PID_KI * integral_ + car_config::PID_KD * derivative;
    previous_error_ = line.error;
    classifyTurn(fabsf(correction));
    motors_.drive(car_config::BASE_MOTOR_COMMAND + correction, car_config::BASE_MOTOR_COMMAND - correction);
  }

  void classifyTurn(float magnitude) {
    if (turn_ == d_task::TurnClass::LARGE && magnitude >= car_config::LARGE_TURN_EXIT) return;
    if (turn_ != d_task::TurnClass::LARGE && magnitude >= car_config::LARGE_TURN_ENTER) { turn_ = d_task::TurnClass::LARGE; return; }
    if (turn_ == d_task::TurnClass::SMALL && magnitude >= car_config::SMALL_TURN_EXIT) return;
    turn_ = magnitude >= car_config::SMALL_TURN_ENTER ? d_task::TurnClass::SMALL : d_task::TurnClass::STRAIGHT;
  }

  void updateOdometry(uint32_t now_ms) {
    int32_t left_count, right_count; encoders_.snapshot(left_count, right_count);
    const float meters_per_count = PI * car_config::WHEEL_DIAMETER_M / car_config::ENCODER_COUNTS_PER_REVOLUTION;
    const float left_m = left_count * meters_per_count, right_m = right_count * meters_per_count;
    const float left_step = fabsf(left_m - last_left_m_);
    const float right_step = fabsf(right_m - last_right_m_);
    const float new_displacement = (left_m + right_m) * 0.5F;
    const float seconds = (now_ms - last_encoder_ms_) / 1000.0F;
    if (seconds > 0.0F) {
      // 单周期不可能的计数跳变按编码器噪声处理，防止虚假推进路线事件。
      if (left_step / seconds > car_config::MAX_WHEEL_SPEED_M_S ||
          right_step / seconds > car_config::MAX_WHEEL_SPEED_M_S) {
        encoder_valid_ = false; safeStop(d_task::FAULT_ENCODER_DISAGREE); return;
      }
      const float instant_velocity = (new_displacement - displacement_m_) / seconds;
      velocity_m_s_ += car_config::VELOCITY_FILTER_ALPHA * (instant_velocity - velocity_m_s_);
    }
    displacement_m_ = new_displacement; last_encoder_ms_ = now_ms;
    last_left_m_ = left_m; last_right_m_ = right_m;
    encoder_valid_ = true;
    const float larger_step = left_step > right_step ? left_step : right_step;
    const float smaller_step = left_step < right_step ? left_step : right_step;
    if (larger_step < car_config::ENCODER_NO_MOTION_STEP_M) {
      if (encoder_no_motion_since_ms_ == 0) encoder_no_motion_since_ms_ = now_ms;
      if (now_ms - encoder_no_motion_since_ms_ >= car_config::ENCODER_NO_MOTION_MS) {
        encoder_valid_ = false; safeStop(d_task::FAULT_ENCODER_DISAGREE); return;
      }
    } else {
      encoder_no_motion_since_ms_ = 0;
    }
    const bool disagree = turn_ == d_task::TurnClass::STRAIGHT && larger_step >= car_config::ENCODER_MIN_STEP_M &&
                          smaller_step < larger_step * car_config::ENCODER_MIN_WHEEL_RATIO;
    if (disagree) {
      if (encoder_disagree_since_ms_ == 0) encoder_disagree_since_ms_ = now_ms;
      if (now_ms - encoder_disagree_since_ms_ >= car_config::ENCODER_DISAGREE_MS) {
        encoder_valid_ = false; safeStop(d_task::FAULT_ENCODER_DISAGREE);
      }
    } else {
      encoder_disagree_since_ms_ = 0;
    }
  }

  void updateRouteEvent() {
    if (state_ != d_task::CarState::RUNNING || event_ != d_task::RouteEvent::NONE) return;
    const float distance = fabsf(displacement_m_);
    d_task::RouteEvent next = d_task::RouteEvent::NONE;
    if (route_stage_ == 0 && distance >= car_config::ROUTE_B_M) next = d_task::RouteEvent::B;
    else if (route_stage_ == 1 && distance >= car_config::ROUTE_D_M) next = d_task::RouteEvent::D;
    else if (route_stage_ == 2 && distance >= car_config::ROUTE_A_M) next = d_task::RouteEvent::A;
    else if (route_stage_ == 3 && distance >= car_config::ROUTE_COMPLETE_M) next = d_task::RouteEvent::COMPLETE;
    if (next == d_task::RouteEvent::NONE) return;
    queueEvent(next); ++route_stage_;
    if (next == d_task::RouteEvent::COMPLETE) { state_ = d_task::CarState::COMPLETE; motors_.brake(); }
  }

  void safeStop(uint16_t fault) {
    fault_flags_ |= fault;
    if (state_ != d_task::CarState::COMPLETE) { state_ = d_task::CarState::SAFE_STOP; motors_.brake(); }
  }

  void queueEvent(d_task::RouteEvent event) {
    event_ = event; ++event_id_;
    event_transmissions_remaining_ = car_config::EVENT_REPEAT_FRAMES;
  }

  LineSensors line_sensors_;
  MotorDriver motors_;
  Encoders encoders_;
  StartButton button_;
  d_task::CarState state_ = d_task::CarState::READY;
  d_task::TurnClass turn_ = d_task::TurnClass::STRAIGHT;
  d_task::RouteEvent event_ = d_task::RouteEvent::NONE;
  uint16_t event_id_ = 0;
  uint8_t event_transmissions_remaining_ = 0;
  uint16_t fault_flags_ = d_task::FAULT_NONE;
  uint8_t route_stage_ = 0;
  bool line_valid_ = false;
  bool encoder_valid_ = false;
  float line_error_ = 0.0F;
  float integral_ = 0.0F;
  float previous_error_ = 0.0F;
  float displacement_m_ = 0.0F;
  float velocity_m_s_ = 0.0F;
  float last_left_m_ = 0.0F;
  float last_right_m_ = 0.0F;
  uint32_t last_network_ok_ms_ = 0;
  uint32_t line_lost_since_ms_ = 0;
  uint32_t last_encoder_ms_ = 0;
  uint32_t encoder_disagree_since_ms_ = 0;
  uint32_t encoder_no_motion_since_ms_ = 0;
  uint32_t next_control_us_ = 0;
};
