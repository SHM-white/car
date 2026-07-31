#pragma once

#include <Arduino.h>

#include "Config.h"
#include "HmiStateMachine.h"

enum class TouchAction : uint8_t { NONE = 0, TASK_1, TASK_2, TASK_TEST, CONFIRM, CANCEL };

// 串口适配器只用于主机回归测试；实机默认启用微雪 7 英寸触屏驱动。
class SerialDisplayTouchDriver {
 public:
  bool begin() {
    Serial.println("[界面] 串口台架：输入 1/2 选择任务，c 确认，x 取消");
    return true;
  }

  TouchAction poll() {
    if (!Serial.available()) return TouchAction::NONE;
    switch (Serial.read()) {
      case '1': return TouchAction::TASK_1;
      case '2': return TouchAction::TASK_2;
      case 't': case 'T': return TouchAction::TASK_TEST;
      case 'c': case 'C': return TouchAction::CONFIRM;
      case 'x': case 'X': return TouchAction::CANCEL;
      default: return TouchAction::NONE;
    }
  }

  void render(const HmiStateMachine &model, uint32_t car_age_ms, uint32_t ros_age_ms) {
    if (model.state() == last_state_ && model.visibleFaults() == last_faults_ &&
        model.confirmationVisible() == last_confirmation_) return;
    last_state_ = model.state();
    last_faults_ = model.visibleFaults();
    last_confirmation_ = model.confirmationVisible();
    Serial.printf("[界面] 状态=%s 已确认任务=%u 待确认任务=%u 车辆数据龄=%lums "
                  "ROS数据龄=%lums 故障=0x%04X\n",
                  stateName(model.state()), model.selectedTask(), model.pendingTask(),
                  static_cast<unsigned long>(car_age_ms), static_cast<unsigned long>(ros_age_ms),
                  model.visibleFaults());
  }

 private:
  static const char *stateName(HmiState state) {
    switch (state) {
      case HmiState::BOOT_WAITING: return "启动等待";
      case HmiState::PRESTART: return "任务选择";
      case HmiState::SELECT_PENDING: return "等待ROS确认";
      case HmiState::SELECTED: return "任务已确认";
      case HmiState::ARMED_READY: return "已获准等待车辆";
      case HmiState::CAR_RUNNING: return "车辆运行";
      case HmiState::COMPLETE: return "任务完成/只读";
      case HmiState::FAULT: return "故障";
    }
    return "未知";
  }

  HmiState last_state_ = static_cast<HmiState>(255);
  uint16_t last_faults_ = 0xFFFF;
  bool last_confirmation_ = false;
};

#if !defined(HMI_USE_SERIAL_DISPLAY) && defined(_WIN32) && !defined(ARDUINO_ARCH_ESP32)
// Windows 主机测试没有面板库，自动切换到串口桩；ESP32 实机构建不走此分支。
#define HMI_USE_SERIAL_DISPLAY 1
#endif

#ifndef HMI_USE_SERIAL_DISPLAY
#define HMI_USE_SERIAL_DISPLAY 0
#endif

#ifndef HMI_DISPLAY_DRIVER
#if HMI_USE_SERIAL_DISPLAY
#define HMI_DISPLAY_DRIVER SerialDisplayTouchDriver
#else
#include "WaveshareDisplayTouchDriver.h"
#define HMI_DISPLAY_DRIVER WaveshareDisplayTouchDriver
#endif
#endif
