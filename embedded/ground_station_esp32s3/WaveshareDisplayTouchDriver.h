#pragma once

#if !HMI_USE_SERIAL_DISPLAY

#include <atomic>
#include <stdint.h>

#include <esp_display_panel.hpp>
#include <lvgl.h>

#if defined(ARDUINO_ARCH_ESP32) && !defined(BOARD_HAS_PSRAM)
#error "地面站必须在 Arduino IDE 中启用 PSRAM；请选择 Waveshare ESP32-S3-Touch-LCD-7，并将 PSRAM 设为 Enabled"
#endif

class HmiStateMachine;

// 微雪 ESP32-S3-Touch-LCD-7 的显示与 GT911 触摸适配层。
class WaveshareDisplayTouchDriver {
 public:
  bool begin();
  TouchAction poll();
  void render(const HmiStateMachine &model, uint32_t car_age_ms, uint32_t ros_age_ms);

 private:
  static void task1Event(lv_event_t *event);
  static void task2Event(lv_event_t *event);
  static void testTaskEvent(lv_event_t *event);
  static void confirmEvent(lv_event_t *event);
  static void cancelEvent(lv_event_t *event);

  void queueAction(TouchAction action);
  void createUi();
  void setStatus(lv_obj_t *label, bool fresh, bool healthy,
                 const char *healthy_text, const char *unhealthy_text);

  esp_panel::board::Board *board_ = nullptr;
  std::atomic<uint8_t> pending_action_{static_cast<uint8_t>(TouchAction::NONE)};
  bool ready_ = false;

  lv_obj_t *state_label_ = nullptr;
  lv_obj_t *mission_label_ = nullptr;
  lv_obj_t *link_label_ = nullptr;
  lv_obj_t *selected_task_label_ = nullptr;
  lv_obj_t *pending_task_label_ = nullptr;
  lv_obj_t *car_link_status_label_ = nullptr;
  lv_obj_t *drone_status_label_ = nullptr;
  lv_obj_t *vision_status_label_ = nullptr;
  lv_obj_t *ros_status_label_ = nullptr;
  lv_obj_t *track_panel_ = nullptr;
  lv_obj_t *track_line_ = nullptr;
  lv_obj_t *track_marker_ = nullptr;
  lv_obj_t *track_wp_b_ = nullptr;
  lv_obj_t *track_wp_d_ = nullptr;
  lv_obj_t *track_wp_a_ = nullptr;
  lv_obj_t *track_label_b_ = nullptr;
  lv_obj_t *track_label_d_ = nullptr;
  lv_obj_t *track_label_a_ = nullptr;
  lv_obj_t *car_state_label_ = nullptr;
  lv_obj_t *turn_label_ = nullptr;
  lv_obj_t *displacement_label_ = nullptr;
  lv_obj_t *velocity_label_ = nullptr;
  lv_obj_t *line_error_label_ = nullptr;
  lv_obj_t *quality_label_ = nullptr;
  lv_obj_t *fault_bar_ = nullptr;
  lv_obj_t *fault_label_ = nullptr;
  lv_obj_t *fps_label_ = nullptr;
  lv_obj_t *task1_button_ = nullptr;
  lv_obj_t *task2_button_ = nullptr;
  lv_obj_t *test_task_button_ = nullptr;
  lv_obj_t *confirm_overlay_ = nullptr;
  lv_obj_t *confirm_prompt_ = nullptr;
  bool local_test_mode_ = false;
  uint32_t fps_sample_ms_ = 0;
  uint32_t fps_sample_vsync_ = 0;
};

#endif
