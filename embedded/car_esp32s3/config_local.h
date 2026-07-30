#pragma once

#include <Arduino.h>
#include <IPAddress.h>

namespace car_config {

constexpr char WIFI_SSID[] = "ED-UAV";
constexpr char WIFI_PASSWORD[] = "5RQqDVzbg5GxZpLz";
constexpr uint8_t AUTH_KEY[] = {
    0x76, 0x5D, 0x69, 0x2A, 0xEE, 0xA6, 0x47, 0xB9,
    0x7C, 0x52, 0xC3, 0xD8, 0x52, 0xF4, 0x1F, 0xF5,
    0xC7, 0x60, 0xEE, 0xD5, 0xD8, 0x4F, 0x15, 0x9F,
    0x3F, 0x7C, 0x53, 0xD2, 0xCA, 0x0A, 0x18, 0x32,
};

const IPAddress LOCAL_IP(192, 168, 20, 2);
const IPAddress GATEWAY(192, 168, 20, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress ROS_IP(192, 168, 20, 1);
const IPAddress HMI_IP(192, 168, 20, 3);
constexpr uint16_t LOCAL_UDP_PORT = 42001;
constexpr uint16_t ROS_UDP_PORT = 42000;
constexpr uint16_t HMI_UDP_PORT = 42002;
constexpr uint32_t SENDER_ID = 0x43415231;  // "CAR1"
constexpr uint32_t ROS_SENDER_ID = 0x524F5331;  // "ROS1"

constexpr int LINE_SENSOR_SDA_PIN = 1;
constexpr int LINE_SENSOR_SCL_PIN = 2;
constexpr uint32_t LINE_SENSOR_I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t LINE_SENSOR_I2C_TIMEOUT_MS = 3;
constexpr uint8_t LINE_SENSOR_I2C_ADDRESS_FIRST = 0x4C;
constexpr uint8_t LINE_SENSOR_I2C_ADDRESS_LAST = 0x4F;
constexpr uint8_t LINE_SENSOR_STARTUP_ATTEMPTS = 50;
constexpr uint32_t LINE_SENSOR_STARTUP_RETRY_MS = 10;
constexpr uint8_t LINE_SENSOR_COUNT = 8;
constexpr bool LINE_SENSOR_CHANNELS_REVERSED = false;
constexpr float LINE_SENSOR_CENTER_OFFSET_CHANNELS = 0.0F;
constexpr bool LINE_SENSOR_ENABLE_NORMALIZATION = true;
constexpr bool LINE_IS_DARK = true;

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 6;
constexpr uint8_t LEFT_MOTOR_IN1_PIN = 7;
constexpr uint8_t LEFT_MOTOR_IN2_PIN = 8;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9;
constexpr uint8_t RIGHT_MOTOR_IN1_PIN = 10;
constexpr uint8_t RIGHT_MOTOR_IN2_PIN = 11;
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr uint8_t LEFT_ENCODER_A_PIN = 12;
constexpr uint8_t LEFT_ENCODER_B_PIN = 13;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 14;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 15;
constexpr uint8_t START_BUTTON_PIN = 16;

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr bool MOTOR_BRAKE_HIGH = true;
constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr bool LEFT_ENCODER_INVERTED = false;
constexpr bool RIGHT_ENCODER_INVERTED = true;

constexpr float WHEEL_DIAMETER_M = 0.065F;
constexpr float WHEEL_TRACK_M = 0.0652F;
constexpr int32_t ENCODER_COUNTS_PER_REVOLUTION = 780;
constexpr float TARGET_SPEED_M_S = 0.078F;
constexpr float SPEED_FEED_FORWARD_COMMAND = 0.21F;
constexpr float BASE_MOTOR_COMMAND = SPEED_FEED_FORWARD_COMMAND;
constexpr float SPEED_KP = 1.20F;
constexpr float SPEED_KI = 0.80F;
constexpr float SPEED_INTEGRAL_LIMIT = 0.20F;
constexpr float MAX_BASE_MOTOR_COMMAND = 0.45F;
constexpr float PID_KP = 0.32F;
constexpr float PID_KI = 0.03F;
constexpr float PID_KD = 0.015F;
constexpr float PID_INTEGRAL_LIMIT = 0.35F;
constexpr float MAX_STEERING_CORRECTION = 0.20F;
constexpr float SMALL_TURN_ENTER = 0.18F;
constexpr float SMALL_TURN_EXIT = 0.12F;
constexpr float LARGE_TURN_ENTER = 0.48F;
constexpr float LARGE_TURN_EXIT = 0.38F;
constexpr float MIN_LINE_STRENGTH = 0.12F;
constexpr float ENCODER_MIN_STEP_M = 0.002F;
constexpr float ENCODER_MIN_WHEEL_RATIO = 0.20F;
constexpr float ENCODER_NO_MOTION_STEP_M = 0.0002F;
constexpr float MAX_WHEEL_SPEED_M_S = 3.0F;
constexpr float VELOCITY_FILTER_ALPHA = 0.25F;

constexpr float ROUTE_B_M = 1.50F;
constexpr float ROUTE_D_M = 4.00F;
constexpr float ROUTE_A_M = 6.50F;
constexpr float ROUTE_COMPLETE_M = 8.00F;

constexpr uint32_t CONTROL_PERIOD_US = 5000;
constexpr uint32_t ENCODER_PERIOD_MS = 20;
constexpr uint32_t WIFI_RETRY_PERIOD_MS = 2000;
constexpr uint32_t WIFI_LOSS_STOP_MS = 1000;
constexpr uint32_t LINE_LOST_STOP_MS = 250;
constexpr uint32_t ENCODER_DISAGREE_MS = 300;
constexpr uint32_t ENCODER_NO_MOTION_MS = 500;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;
constexpr uint32_t BUTTON_STUCK_MS = 3000;
constexpr uint32_t PID_OVERRUN_US = 4000;
constexpr uint8_t EVENT_REPEAT_FRAMES = 5;

}  // namespace car_config
