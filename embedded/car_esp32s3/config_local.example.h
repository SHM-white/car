#pragma once

#include <Arduino.h>
#include <IPAddress.h>

namespace car_config {

// 必须在烧录前替换；真实口令和密钥只放在未纳入版本控制的 config_local.h。
constexpr char WIFI_SSID[] = "REPLACE_WITH_OFFLINE_AP";
constexpr char WIFI_PASSWORD[] = "REPLACE_WITH_WPA2_PASSWORD";
constexpr uint8_t AUTH_KEY[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
};

const IPAddress ROS_IP(192, 168, 20, 1);
const IPAddress HMI_IP(192, 168, 20, 3);
constexpr uint16_t LOCAL_UDP_PORT = 42001;
constexpr uint16_t ROS_UDP_PORT = 42000;
constexpr uint16_t HMI_UDP_PORT = 42002;
constexpr uint32_t SENDER_ID = 0x43415231;  // "CAR1"
constexpr uint32_t ROS_SENDER_ID = 0x524F5331;  // "ROS1"

// 感为八路灰度传感器：模块用稳定 5V 供电，I2C 两侧必须通过双向电平转换。
// GPIO1/2 一侧上拉至 3.3V；传感器一侧上拉至 5V。
constexpr int LINE_SENSOR_SDA_PIN = 1;
constexpr int LINE_SENSOR_SCL_PIN = 2;
constexpr uint32_t LINE_SENSOR_I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t LINE_SENSOR_I2C_TIMEOUT_MS = 3;
// 出厂软件地址位为 0b10011；AD1/AD0 的四种跳帽组合对应 0x4C~0x4F。
constexpr uint8_t LINE_SENSOR_I2C_ADDRESS_FIRST = 0x4C;
constexpr uint8_t LINE_SENSOR_I2C_ADDRESS_LAST = 0x4F;
constexpr uint8_t LINE_SENSOR_STARTUP_ATTEMPTS = 50;
constexpr uint32_t LINE_SENSOR_STARTUP_RETRY_MS = 10;
constexpr uint8_t LINE_SENSOR_COUNT = 8;
// 实车车头朝前时：左侧为 ch1，右侧为 ch8；I2C 0xB0 顺序无需反转。
constexpr bool LINE_SENSOR_CHANNELS_REVERSED = false;
// 传感器中心相对车体中心的横向偏移，单位为通道间距；右偏为正，初始为 0。
constexpr float LINE_SENSOR_CENTER_OFFSET_CHANNELS = 0.0F;
constexpr bool LINE_SENSOR_ENABLE_NORMALIZATION = true;
constexpr bool LINE_IS_DARK = true;

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 6;
constexpr uint8_t LEFT_MOTOR_IN1_PIN = 7;
constexpr uint8_t LEFT_MOTOR_IN2_PIN = 8;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9;
constexpr uint8_t RIGHT_MOTOR_IN1_PIN = 10;
constexpr uint8_t RIGHT_MOTOR_IN2_PIN = 11;
// TB6612FNG: 左轮接 A 通道（AO1/AO2），右轮接 B 通道（BO1/BO2）。
// 架空测试中某侧正向反转时，仅将对应项改为 true。
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
// 编码器闭环目标平均线速度,单位 m/s；实际速度由速度 PI 自动修正。
constexpr float TARGET_SPEED_M_S = 0.078F;
// 实车初始估计值；速度 PI 会根据编码器读数自动修正。
constexpr float SPEED_FEED_FORWARD_COMMAND = 0.21F;
// 保留给未启用的 CarController 入口使用；自主循迹使用速度闭环。
constexpr float BASE_MOTOR_COMMAND = SPEED_FEED_FORWARD_COMMAND;
constexpr float SPEED_KP = 1.20F;
constexpr float SPEED_KI = 0.80F;
constexpr float SPEED_INTEGRAL_LIMIT = 0.20F;
constexpr float MAX_BASE_MOTOR_COMMAND = 0.45F;
// 保守起步参数：最大左右轮差速为 2 * MAX_STEERING_CORRECTION。
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

// 起点为 0，正方向沿规定赛道前进；这些距离必须用实际赛道重新标定。
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
