#pragma once

#include <Arduino.h>
#include <math.h>

#include "Config.h"

static_assert(car_config::LINE_ERROR_FILTER_ALPHA > 0.0F &&
                  car_config::LINE_ERROR_FILTER_ALPHA <= 1.0F,
              "line error filter alpha must be in (0, 1]");
static_assert(car_config::PID_DERIVATIVE_FILTER_ALPHA > 0.0F &&
                  car_config::PID_DERIVATIVE_FILTER_ALPHA <= 1.0F,
              "derivative filter alpha must be in (0, 1]");
static_assert(car_config::PID_RECOVERY_FULL_ERROR >
                  car_config::PID_RECOVERY_START_ERROR,
              "recovery full error must exceed start error");
static_assert(car_config::RECENTER_MIN_BASE_SPEED_RATIO > 0.0F &&
                  car_config::RECENTER_MIN_BASE_SPEED_RATIO <= 1.0F,
              "recenter base speed ratio must be in (0, 1]");
static_assert(car_config::STEERING_APPLY_RATE_PER_S > 0.0F,
              "steering apply rate must be positive");
static_assert(car_config::STEERING_RELEASE_RATE_PER_S >=
                  car_config::STEERING_APPLY_RATE_PER_S,
              "steering release rate must not be slower than apply rate");
static_assert(car_config::MAX_MOTOR_COMMAND_DIFFERENCE > 0.0F &&
                  car_config::MAX_MOTOR_COMMAND_DIFFERENCE <= 2.0F,
              "motor command difference must be in (0, 2]");
class LineSteering {
 public:
  struct Output {
    float error;
    float correction;
    float base_speed_ratio;
  };

  void reset(float error) {
    filtered_error_ = error;
    previous_error_ = applyDeadband(error);
    integral_ = 0.0F;
    derivative_ = 0.0F;
    applied_correction_ = 0.0F;
    initialized_ = true;
  }

  void clear() {
    filtered_error_ = 0.0F;
    previous_error_ = 0.0F;
    integral_ = 0.0F;
    derivative_ = 0.0F;
    applied_correction_ = 0.0F;
    initialized_ = false;
  }

  Output update(float measured_error) {
    if (!initialized_) reset(measured_error);

    filtered_error_ += car_config::LINE_ERROR_FILTER_ALPHA *
                       (measured_error - filtered_error_);
    const float error = applyDeadband(filtered_error_);

    // A direction change means the old integral would keep steering the car
    // past the line. Release it immediately instead of waiting for it to unwind.
    if (error * previous_error_ < 0.0F) integral_ = 0.0F;
    if (error == 0.0F) {
      integral_ *= car_config::PID_INTEGRAL_CENTER_DECAY;
    } else {
      constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
      integral_ = constrain(integral_ + error * dt,
                            -car_config::PID_INTEGRAL_LIMIT,
                            car_config::PID_INTEGRAL_LIMIT);
    }

    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    const float raw_derivative = (error - previous_error_) / dt;
    derivative_ += car_config::PID_DERIVATIVE_FILTER_ALPHA *
                   (raw_derivative - derivative_);
    previous_error_ = error;

    const float recovery = recoveryAmount(fabsf(error));
    const float kp = car_config::PID_KP +
                     recovery * (car_config::PID_RECOVERY_KP - car_config::PID_KP);
    const float base_speed_ratio = 1.0F -
        recovery * (1.0F - car_config::RECENTER_MIN_BASE_SPEED_RATIO);
    float target_correction = kp * error +
                              car_config::PID_KI * integral_ +
                              car_config::PID_KD * derivative_;
    const float differential_limit = 0.5F * car_config::MAX_MOTOR_COMMAND_DIFFERENCE;
    const float correction_limit = fminf(car_config::MAX_STEERING_CORRECTION,
                                         differential_limit);
    target_correction = constrain(target_correction,
                                  -correction_limit,
                                  correction_limit);
    applied_correction_ = approachCorrection(target_correction);

    return {error, applied_correction_, base_speed_ratio};
  }

 private:
  static float applyDeadband(float error) {
    return fabsf(error) < car_config::LINE_ERROR_DEADBAND ? 0.0F : error;
  }

  static float recoveryAmount(float error_magnitude) {
    const float range = car_config::PID_RECOVERY_FULL_ERROR -
                        car_config::PID_RECOVERY_START_ERROR;
    if (range <= 0.0F) return 1.0F;
    return constrain((error_magnitude - car_config::PID_RECOVERY_START_ERROR) / range,
                     0.0F, 1.0F);
  }

  float approachCorrection(float target) const {
    constexpr float dt = car_config::CONTROL_PERIOD_US / 1000000.0F;
    const bool releasing = target * applied_correction_ < 0.0F ||
                           fabsf(target) < fabsf(applied_correction_);
    const float rate = releasing ? car_config::STEERING_RELEASE_RATE_PER_S
                                 : car_config::STEERING_APPLY_RATE_PER_S;
    const float maximum_step = rate * dt;
    return applied_correction_ +
           constrain(target - applied_correction_, -maximum_step, maximum_step);
  }

  bool initialized_ = false;
  float filtered_error_ = 0.0F;
  float previous_error_ = 0.0F;
  float integral_ = 0.0F;
  float derivative_ = 0.0F;
  float applied_correction_ = 0.0F;
};
