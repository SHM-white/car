#include "DisplayAdapter.h"

#if !HMI_USE_SERIAL_DISPLAY

#include <cmath>
#include <cstdlib>
#include <new>
#include <stdio.h>

#include "lvgl_v8_port.h"

using esp_panel::board::Board;
using esp_panel::drivers::BusRGB;

namespace {

lv_color_t color(uint32_t value) {
  return lv_color_make((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF);
}

constexpr uint32_t kBackground = 0xF3F5F7;
constexpr uint32_t kTopBar = 0x1D252C;
constexpr uint32_t kText = 0x182026;
constexpr uint32_t kMuted = 0x5F6B73;
constexpr uint32_t kBlue = 0x1769AA;
constexpr uint32_t kGreen = 0x15803D;
constexpr uint32_t kAmber = 0xA75B00;
constexpr uint32_t kRed = 0xB42318;
constexpr uint32_t kLine = 0xD5DBE0;
// 两个 800x480 RGB565 帧缓冲约占 1.54 MB，预留余量给 LVGL 和网络任务。
constexpr uint32_t kMinimumPsramBytes = 4U * 1024U * 1024U;
constexpr uint32_t kFpsSamplePeriodMs = 1000;
// 官方建议在出现 RGB 动态漂移时从 width * 10 增大到 width * 20。
constexpr uint32_t kRgbBounceBufferLines = 20;

lv_obj_t *plainPanel(lv_obj_t *parent, int x, int y, int width, int height, uint32_t background) {
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_remove_style_all(panel);
  lv_obj_set_pos(panel, x, y);
  lv_obj_set_size(panel, width, height);
  lv_obj_set_style_bg_color(panel, color(background), 0);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  return panel;
}

lv_obj_t *textLabel(lv_obj_t *parent, const char *text, int x, int y,
                    int width, const lv_font_t *font, uint32_t text_color) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_pos(label, x, y);
  lv_obj_set_width(label, width);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color(text_color), 0);
  lv_obj_set_style_text_letter_space(label, 0, 0);
  return label;
}

lv_obj_t *commandButton(lv_obj_t *parent, const char *text, int x, int y,
                        int width, int height, lv_event_cb_t callback, void *owner) {
  lv_obj_t *button = lv_btn_create(parent);
  lv_obj_set_pos(button, x, y);
  lv_obj_set_size(button, width, height);
  lv_obj_set_style_radius(button, 4, 0);
  lv_obj_set_style_bg_color(button, color(kBlue), 0);
  lv_obj_set_style_bg_color(button, color(0x0F4F82), LV_PART_MAIN | LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(button, color(kGreen), LV_PART_MAIN | LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(button, color(0xA8B0B6), LV_PART_MAIN | LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(button, 0, 0);
  lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, owner);
  lv_obj_t *label = lv_label_create(button);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_letter_space(label, 0, 0);
  lv_obj_center(label);
  return button;
}

const char *hmiStateName(HmiState state) {
  switch (state) {
    case HmiState::BOOT_WAITING: return "BOOT WAITING";
    case HmiState::PRESTART: return "PRESTART";
    case HmiState::SELECT_PENDING: return "WAITING FOR ROS";
    case HmiState::SELECTED: return "TASK SELECTED";
    case HmiState::ARMED_READY: return "ARMED / READY";
    case HmiState::CAR_RUNNING: return "CAR RUNNING";
    case HmiState::COMPLETE: return "MISSION COMPLETE";
    case HmiState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char *missionPhaseName(d_task::MissionPhase phase) {
  switch (phase) {
    case d_task::MissionPhase::PRESTART: return "PRESTART";
    case d_task::MissionPhase::SELECTION_ACKED: return "SELECTION ACKED";
    case d_task::MissionPhase::ARMED_READY: return "ARMED READY";
    case d_task::MissionPhase::CAR_RUNNING: return "CAR RUNNING";
    case d_task::MissionPhase::COMPLETE: return "COMPLETE";
    case d_task::MissionPhase::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char *carStateName(d_task::CarState state) {
  switch (state) {
    case d_task::CarState::READY: return "READY";
    case d_task::CarState::RUNNING: return "RUNNING";
    case d_task::CarState::COMPLETE: return "COMPLETE";
    case d_task::CarState::SAFE_STOP: return "SAFE STOP";
  }
  return "UNKNOWN";
}

const char *turnName(d_task::TurnClass turn) {
  switch (turn) {
    case d_task::TurnClass::STRAIGHT: return "STRAIGHT";
    case d_task::TurnClass::SMALL: return "SMALL";
    case d_task::TurnClass::LARGE: return "LARGE";
  }
  return "UNKNOWN";
}

void formatAge(uint32_t age_ms, uint32_t stale_ms, char *output, size_t capacity) {
  if (age_ms == UINT32_MAX) snprintf(output, capacity, "NO DATA");
  // 远超陈旧阈值后显示 LOST，不再让数字无限增长。
  else if (age_ms > stale_ms * 8) snprintf(output, capacity, "LOST");
  else if (age_ms > stale_ms) snprintf(output, capacity, "STALE");
  else snprintf(output, capacity, "%lums", static_cast<unsigned long>(age_ms));
}

void formatMilliValue(int32_t value, char *output, size_t capacity) {
  const int64_t magnitude = value < 0 ? -static_cast<int64_t>(value) : value;
  snprintf(output, capacity, "%s%lld.%03lld", value < 0 ? "-" : "",
           static_cast<long long>(magnitude / 1000),
           static_cast<long long>(magnitude % 1000));
}

}  // namespace

bool WaveshareDisplayTouchDriver::begin() {
  Serial.println("[显示自检] 初始化 Waveshare ESP32-S3-Touch-LCD-7");
  const uint32_t psram_size = ESP.getPsramSize();
  const uint32_t psram_free = ESP.getFreePsram();
  Serial.printf("[显示自检] PSRAM 总容量=%lu 字节，可用=%lu 字节\n",
                static_cast<unsigned long>(psram_size),
                static_cast<unsigned long>(psram_free));
  if (!psramFound() || psram_size < kMinimumPsramBytes) {
    Serial.println("[显示故障] 未检测到足够的 PSRAM；请在工具菜单中将 PSRAM 设为 Enabled");
    return false;
  }

  board_ = new (std::nothrow) Board();
  if (board_ == nullptr || !board_->init()) {
    Serial.println("[显示故障] 无法创建或初始化面板对象");
    delete board_;
    board_ = nullptr;
    return false;
  }

  auto *lcd = board_->getLCD();
  if (lcd == nullptr) {
    Serial.println("[显示故障] 板型配置没有生成 LCD 驱动");
    delete board_;
    board_ = nullptr;
    return false;
  }

#if LVGL_PORT_AVOID_TEARING_MODE
  // 双缓冲和 RGB 回弹缓冲沿用官方示例，降低 800x480 刷新时的撕裂与漂移。
  lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
  auto *lcd_bus = lcd->getBus();
  if (lcd_bus != nullptr && lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
    const uint32_t bounce_buffer_size = lcd->getFrameWidth() * kRgbBounceBufferLines;
    static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(bounce_buffer_size);
    Serial.printf("[显示自检] RGB 回弹缓冲=%lu 字节\n",
                  static_cast<unsigned long>(bounce_buffer_size));
  }
#endif
#endif

  if (!board_->begin()) {
    Serial.println("[显示故障] ST7262、GT911 或 CH422G 启动失败");
    delete board_;
    board_ = nullptr;
    return false;
  }
  if (!lvgl_port_init(board_->getLCD(), board_->getTouch())) {
    Serial.println("[显示故障] LVGL 移植层启动失败");
    delete board_;
    board_ = nullptr;
    return false;
  }
  if (!lvgl_port_lock(-1)) {
    Serial.println("[显示故障] 无法取得 LVGL 互斥锁");
    return false;
  }
  createUi();
  lvgl_port_unlock();
  ready_ = true;
  Serial.println("[显示自检] 800x480 屏幕和 GT911 触摸已就绪");
  return true;
}

TouchAction WaveshareDisplayTouchDriver::poll() {
  return static_cast<TouchAction>(pending_action_.exchange(
      static_cast<uint8_t>(TouchAction::NONE), std::memory_order_acq_rel));
}

void WaveshareDisplayTouchDriver::queueAction(TouchAction action) {
  uint8_t expected = static_cast<uint8_t>(TouchAction::NONE);
  pending_action_.compare_exchange_strong(expected, static_cast<uint8_t>(action),
                                          std::memory_order_release, std::memory_order_relaxed);
}

void WaveshareDisplayTouchDriver::task1Event(lv_event_t *event) {
  auto *driver = static_cast<WaveshareDisplayTouchDriver *>(lv_event_get_user_data(event));
  driver->local_test_mode_ = false;
  driver->queueAction(TouchAction::TASK_1);
}

void WaveshareDisplayTouchDriver::task2Event(lv_event_t *event) {
  auto *driver = static_cast<WaveshareDisplayTouchDriver *>(lv_event_get_user_data(event));
  driver->local_test_mode_ = false;
  driver->queueAction(TouchAction::TASK_2);
}

void WaveshareDisplayTouchDriver::testTaskEvent(lv_event_t *event) {
  auto *driver = static_cast<WaveshareDisplayTouchDriver *>(lv_event_get_user_data(event));
  driver->local_test_mode_ = !driver->local_test_mode_;
  if (driver->local_test_mode_) {
    driver->queueAction(TouchAction::TASK_TEST);
  } else {
    driver->queueAction(TouchAction::CANCEL);
  }
}

void WaveshareDisplayTouchDriver::confirmEvent(lv_event_t *event) {
  auto *driver = static_cast<WaveshareDisplayTouchDriver *>(lv_event_get_user_data(event));
  // TEST 任务已接入模型确认流程（chooseTask(3)），确认框由模型统一管理；
  // 退出本地 TEST 模式标志后，TEST 按钮的绿色由 render 中的 pendingTask()==3 维持。
  driver->local_test_mode_ = false;
  driver->queueAction(TouchAction::CONFIRM);
}

void WaveshareDisplayTouchDriver::cancelEvent(lv_event_t *event) {
  auto *driver = static_cast<WaveshareDisplayTouchDriver *>(lv_event_get_user_data(event));
  driver->local_test_mode_ = false;
  driver->queueAction(TouchAction::CANCEL);
}

void WaveshareDisplayTouchDriver::createUi() {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, color(kBackground), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *top = plainPanel(screen, 0, 0, 800, 56, kTopBar);
  textLabel(top, "AIR-GROUND STATION", 22, 13, 360, &lv_font_montserrat_26, 0xFFFFFF);
  link_label_ = textLabel(top, "CAR NO DATA   ROS NO DATA", 470, 19, 308,
                          &lv_font_montserrat_16, 0xD9E0E5);
  lv_obj_set_style_text_align(link_label_, LV_TEXT_ALIGN_RIGHT, 0);

  lv_obj_t *left = plainPanel(screen, 0, 56, 255, 374, 0xFFFFFF);
  textLabel(left, "TASK CONTROL", 24, 20, 210, &lv_font_montserrat_16, kMuted);
  task1_button_ = commandButton(left, "TASK 1", 24, 56, 207, 48, task1Event, this);
  task2_button_ = commandButton(left, "TASK 2", 24, 114, 207, 48, task2Event, this);
  test_task_button_ = commandButton(left, "TEST", 24, 172, 207, 48, testTaskEvent, this);
  textLabel(left, "ROS CONFIRMED", 24, 232, 207, &lv_font_montserrat_16, kMuted);
  selected_task_label_ = textLabel(left, "--", 24, 258, 207, &lv_font_montserrat_30, kText);
  pending_task_label_ = textLabel(left, "PENDING --", 24, 312, 207, &lv_font_montserrat_16, kAmber);

  plainPanel(screen, 254, 56, 1, 374, kLine);
  lv_obj_t *mission = plainPanel(screen, 255, 56, 545, 72, 0xE7EFF5);
  textLabel(mission, "LOCAL STATE", 24, 10, 150, &lv_font_montserrat_16, kMuted);
  state_label_ = textLabel(mission, "BOOT WAITING", 24, 34, 320, &lv_font_montserrat_26, kText);
  lv_obj_t *phase_title = textLabel(mission, "ROS PHASE", 340, 10, 180,
                                    &lv_font_montserrat_16, kMuted);
  lv_obj_set_style_text_align(phase_title, LV_TEXT_ALIGN_RIGHT, 0);
  mission_label_ = textLabel(mission, "PRESTART", 340, 39, 180,
                              &lv_font_montserrat_16, kBlue);
  lv_obj_set_style_text_align(mission_label_, LV_TEXT_ALIGN_RIGHT, 0);

  textLabel(screen, "SYSTEM STATUS", 279, 146, 210, &lv_font_montserrat_16, kMuted);
  textLabel(screen, "CAR LINK", 279, 183, 110, &lv_font_montserrat_16, kText);
  car_link_status_label_ = textLabel(screen, "NO DATA", 402, 183, 110, &lv_font_montserrat_16, kRed);
  textLabel(screen, "DRONE LINK", 279, 211, 110, &lv_font_montserrat_16, kText);
  drone_status_label_ = textLabel(screen, "NO DATA", 402, 211, 110, &lv_font_montserrat_16, kRed);
  textLabel(screen, "VISION", 279, 251, 110, &lv_font_montserrat_16, kText);
  vision_status_label_ = textLabel(screen, "NO DATA", 402, 251, 110, &lv_font_montserrat_16, kRed);
  textLabel(screen, "ROS READY", 279, 287, 110, &lv_font_montserrat_16, kText);
  ros_status_label_ = textLabel(screen, "NO DATA", 402, 287, 110, &lv_font_montserrat_16, kRed);

  track_panel_ = plainPanel(screen, 558, 134, 204, 140, 0xFFFFFF);
  lv_obj_set_style_radius(track_panel_, 16, 0);
  lv_obj_set_style_border_width(track_panel_, 2, 0);
  lv_obj_set_style_border_color(track_panel_, color(kLine), 0);
  textLabel(track_panel_, "TRACK VIEW", 14, 10, 176, &lv_font_montserrat_12, kMuted);

  // 赛道为竖向圆角矩形：AB、DC 均为 150 cm 直边，上下圆弧半径 75 cm。
  // 图中 A→B→C→D→A 为顺时针方向。
  {
    constexpr int cx = 102, radius = 23;
    constexpr int y_top = 30, y_upper = 53, y_lower = 99, y_bottom = 122;
    static lv_point_t track_pts[] = {
        {cx - radius, y_lower}, {cx - radius, y_upper},
        {81, 44}, {86, 36}, {94, 32}, {102, y_top},
        {110, 32}, {118, 36}, {123, 44},
        {cx + radius, y_upper}, {cx + radius, y_lower},
        {123, 108}, {118, 116}, {110, 120}, {102, y_bottom},
        {94, 120}, {86, 116}, {81, 108},
        {cx - radius, y_lower},
    };
    track_line_ = lv_line_create(track_panel_);
    lv_line_set_points(track_line_, track_pts, sizeof(track_pts) / sizeof(track_pts[0]));
    lv_obj_set_style_line_color(track_line_, color(kLine), 0);
    lv_obj_set_style_line_width(track_line_, 3, 0);
    lv_obj_set_style_line_rounded(track_line_, true, 0);
  }

  // 关键点位与题图一致：A 左下、B 左上、C 右上、D 右下。
  constexpr int kDotSize = 8;
  track_wp_b_ = plainPanel(track_panel_, 75, 49, kDotSize, kDotSize, kAmber);
  lv_obj_set_style_radius(track_wp_b_, LV_RADIUS_CIRCLE, 0);
  track_label_b_ = textLabel(track_panel_, "B", 62, 45, 16, &lv_font_montserrat_12, kAmber);

  track_wp_c_ = plainPanel(track_panel_, 121, 49, kDotSize, kDotSize, kAmber);
  lv_obj_set_style_radius(track_wp_c_, LV_RADIUS_CIRCLE, 0);
  track_label_c_ = textLabel(track_panel_, "C", 130, 45, 16, &lv_font_montserrat_12, kAmber);

  track_wp_d_ = plainPanel(track_panel_, 121, 95, kDotSize, kDotSize, kAmber);
  lv_obj_set_style_radius(track_wp_d_, LV_RADIUS_CIRCLE, 0);
  track_label_d_ = textLabel(track_panel_, "D", 130, 91, 16, &lv_font_montserrat_12, kAmber);

  track_wp_a_ = plainPanel(track_panel_, 75, 95, kDotSize, kDotSize, kAmber);
  lv_obj_set_style_radius(track_wp_a_, LV_RADIUS_CIRCLE, 0);
  track_label_a_ = textLabel(track_panel_, "A", 62, 91, 16, &lv_font_montserrat_12, kAmber);

  track_marker_ = plainPanel(track_panel_, 72, 92, 14, 14, kBlue);
  lv_obj_set_style_radius(track_marker_, LV_RADIUS_CIRCLE, 0);

  textLabel(screen, "CAR TELEMETRY", 535, 280, 240, &lv_font_montserrat_16, kMuted);
  car_state_label_ = textLabel(screen, "STATE  NO DATA", 535, 308, 240, &lv_font_montserrat_16, kText);
  turn_label_ = textLabel(screen, "TURN   NO DATA", 535, 336, 240, &lv_font_montserrat_16, kText);
  displacement_label_ = textLabel(screen, "DIST   --", 535, 360, 240, &lv_font_montserrat_16, kText);
  velocity_label_ = textLabel(screen, "SPEED  --", 535, 380, 240, &lv_font_montserrat_16, kText);
  line_error_label_ = textLabel(screen, "LINE   --", 535, 396, 240, &lv_font_montserrat_16, kText);
  quality_label_ = textLabel(screen, "QUALITY 0x0000", 535, 410, 240, &lv_font_montserrat_16, kText);

  fault_bar_ = plainPanel(screen, 0, 430, 800, 50, kGreen);
  fps_label_ = textLabel(fault_bar_, "FPS --", 12, 18, 68,
                         &lv_font_montserrat_12, 0xFFFFFF);
  fault_label_ = textLabel(fault_bar_, "SYSTEM NOMINAL", 92, 15, 686,
                           &lv_font_montserrat_16, 0xFFFFFF);
  fps_sample_ms_ = millis();
  fps_sample_vsync_ = lvgl_port_get_vsync_count();

  // 确认层阻止下层按钮接收触摸，避免误触直接提交任务。
  confirm_overlay_ = plainPanel(screen, 0, 0, 800, 480, 0x20262B);
  lv_obj_set_style_bg_opa(confirm_overlay_, LV_OPA_50, 0);
  lv_obj_t *dialog = plainPanel(confirm_overlay_, 200, 145, 400, 190, 0xFFFFFF);
  lv_obj_set_style_radius(dialog, 4, 0);
  confirm_prompt_ = textLabel(dialog, "CONFIRM TASK?", 24, 28, 352,
                              &lv_font_montserrat_26, kText);
  commandButton(dialog, LV_SYMBOL_OK "  CONFIRM", 24, 112, 166, 52, confirmEvent, this);
  commandButton(dialog, LV_SYMBOL_CLOSE "  CANCEL", 210, 112, 166, 52, cancelEvent, this);
  lv_obj_add_flag(confirm_overlay_, LV_OBJ_FLAG_HIDDEN);
}

void WaveshareDisplayTouchDriver::setStatus(lv_obj_t *label, bool fresh, bool healthy,
                                             const char *healthy_text, const char *unhealthy_text) {
  if (!fresh) {
    lv_label_set_text(label, "STALE");
    lv_obj_set_style_text_color(label, color(kRed), 0);
    return;
  }
  lv_label_set_text(label, healthy ? healthy_text : unhealthy_text);
  lv_obj_set_style_text_color(label, color(healthy ? kGreen : kAmber), 0);
}

uint32_t WaveshareDisplayTouchDriver::smoothAge(uint32_t smoothed, uint32_t sample) {
  if (sample == UINT32_MAX) return UINT32_MAX;   // NO DATA 透传
  if (smoothed == UINT32_MAX) return sample;      // 首次采样
  return (smoothed * 7 + sample) / 8;             // EMA(1/8)，时间常数约 800ms
}

void WaveshareDisplayTouchDriver::setButtonLocked(lv_obj_t *button, bool locked) {
  if (locked) {
    lv_obj_add_state(button, LV_STATE_DISABLED);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_CLICKABLE);
  } else {
    lv_obj_clear_state(button, LV_STATE_DISABLED);
    lv_obj_add_flag(button, LV_OBJ_FLAG_CLICKABLE);
  }
}

void WaveshareDisplayTouchDriver::render(const HmiStateMachine &model,
                                          uint32_t car_age_ms, uint32_t ros_age_ms) {
  if (!ready_ || !lvgl_port_lock(50)) return;

  char car_age[24];
  char ros_age[24];
  // 数据龄 EMA 平滑：瞬时 age 是锯齿波（CAR 20Hz→0~50ms，ROS 心跳 250ms→0~250ms），
  // 100ms 渲染一次直接显示会像随机数跳变；平滑后显示稳定读数。
  // 注意：链路新鲜度判定仍用下面的瞬时值（car_fresh/ros_fresh），平滑只影响显示。
  smoothed_car_age_ms_ = smoothAge(smoothed_car_age_ms_, car_age_ms);
  smoothed_ros_age_ms_ = smoothAge(smoothed_ros_age_ms_, ros_age_ms);
  formatAge(smoothed_car_age_ms_, d_task::kTelemetryStaleMs, car_age, sizeof(car_age));
  formatAge(smoothed_ros_age_ms_, hmi_config::ROS_STATUS_STALE_MS, ros_age, sizeof(ros_age));
  lv_label_set_text_fmt(link_label_, "CAR %s   ROS %s", car_age, ros_age);
  lv_label_set_text(state_label_, hmiStateName(model.state()));
  lv_label_set_text(mission_label_, missionPhaseName(model.missionPhase()));

  if (local_test_mode_) {
    if (model.selectedTask() == 0) lv_label_set_text(selected_task_label_, "LOCAL");
    else lv_label_set_text_fmt(selected_task_label_, "TASK %u / LOCAL", model.selectedTask());
    if (model.pendingTask() == 0) lv_label_set_text(pending_task_label_, "PENDING TEST");
    else lv_label_set_text_fmt(pending_task_label_, "PENDING TASK %u / TEST", model.pendingTask());
  } else {
    if (model.selectedTask() == 0) lv_label_set_text(selected_task_label_, "--");
    else lv_label_set_text_fmt(selected_task_label_, "TASK %u", model.selectedTask());
    if (model.pendingTask() == 0) lv_label_set_text(pending_task_label_, "PENDING --");
    else lv_label_set_text_fmt(pending_task_label_, "PENDING TASK %u", model.pendingTask());
  }

  if (model.pendingTask() == 1) lv_obj_add_state(task1_button_, LV_STATE_CHECKED);
  else lv_obj_clear_state(task1_button_, LV_STATE_CHECKED);
  if (model.pendingTask() == 2) lv_obj_add_state(task2_button_, LV_STATE_CHECKED);
  else lv_obj_clear_state(task2_button_, LV_STATE_CHECKED);
  // TEST(task 3)：点击 TEST 或模型已确认启动测试任务时都保持绿色。
  // confirmEvent 会清除 local_test_mode_，但 pendingTask()==3 在确认后仍成立，
  // 因此“启动测试后按钮变灰”的 bug 被消除。
  if (local_test_mode_ || model.pendingTask() == 3) lv_obj_add_state(test_task_button_, LV_STATE_CHECKED);
  else lv_obj_clear_state(test_task_button_, LV_STATE_CHECKED);

  // 任务已提交/执行中：锁定三个任务按钮（灰色 + 不可点击），防止误触改选。
  // 车辆 boot 变化（onCarTelemetry 检测到新会话）会自动重置 selection，解锁按钮。
  const bool task_locked = model.controlsLocked();
  setButtonLocked(task1_button_, task_locked);
  setButtonLocked(task2_button_, task_locked);
  setButtonLocked(test_task_button_, task_locked);

  if (model.confirmationVisible()) {
    lv_label_set_text_fmt(confirm_prompt_, "CONFIRM TASK %u?", model.pendingTask());
    lv_obj_clear_flag(confirm_overlay_, LV_OBJ_FLAG_HIDDEN);
  } else if (local_test_mode_) {
    // TEST 已点击但模型尚未收到 TASK_TEST 事件（一帧过渡），显示同样的确认框
    lv_label_set_text(confirm_prompt_, "CONFIRM TASK 3?");
    lv_obj_clear_flag(confirm_overlay_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(confirm_overlay_, LV_OBJ_FLAG_HIDDEN);
  }

  const bool car_fresh = car_age_ms != UINT32_MAX && car_age_ms <= d_task::kTelemetryStaleMs;
  const bool ros_fresh = ros_age_ms != UINT32_MAX && ros_age_ms <= hmi_config::ROS_STATUS_STALE_MS;
  const uint16_t flags = model.missionStatusFlags();
  setStatus(car_link_status_label_, true, car_fresh, "ONLINE", "OFFLINE");
  setStatus(drone_status_label_, ros_fresh, (flags & d_task::MISSION_DRONE_LINK_OK) != 0,
            (flags & d_task::MISSION_DRONE_ARMED) != 0 ? "ARMED" : "ONLINE", "OFFLINE");
  setStatus(vision_status_label_, ros_fresh, (flags & d_task::MISSION_VISION_VALID) != 0,
            "VALID", "INVALID");
  setStatus(ros_status_label_, ros_fresh, (flags & d_task::MISSION_ROS_READY) != 0,
            "READY", "NOT READY");

  // 物理赛道：AB=DC=1500 mm，两个半圆的半径均为 750 mm。
  // A 为起点，顺时针依次经过 B、C、D，最终回到 A。
  constexpr float kPi = 3.14159265f;
  constexpr int kStraightMm = 1500;
  constexpr int kRadiusMm = 750;
  constexpr int kArcMm = static_cast<int>(kPi * kRadiusMm + 0.5f);
  constexpr int kRouteTotalMm = 2 * kStraightMm + 2 * kArcMm;
  constexpr int kRouteBMm = 1500;
  constexpr int kRouteCMm = kRouteBMm + kArcMm;
  constexpr int kRouteDMm = kRouteCMm + kStraightMm;
  constexpr int kMarkerSize = 14;
  constexpr int kHalfMarker = kMarkerSize / 2;
  constexpr float kCx = 102.0f, kPixelRadius = 23.0f;
  constexpr float kUpperY = 53.0f, kLowerY = 99.0f;

  // 将赛道距离映射到两条直边和两个半圆弧。
  auto routeToPixel = [](int dist_mm, int &out_x, int &out_y) {
    const int clamped = dist_mm < 0 ? 0 : (dist_mm > kRouteTotalMm ? kRouteTotalMm : dist_mm);
    if (clamped <= kRouteBMm) {
      const float t = static_cast<float>(clamped) / kStraightMm;
      out_x = static_cast<int>(kCx - kPixelRadius);
      out_y = static_cast<int>(kLowerY + (kUpperY - kLowerY) * t);
    } else if (clamped <= kRouteCMm) {
      const float angle = kPi - kPi * static_cast<float>(clamped - kRouteBMm) / kArcMm;
      out_x = static_cast<int>(kCx + kPixelRadius * std::cos(angle));
      out_y = static_cast<int>(kUpperY - kPixelRadius * std::sin(angle));
    } else if (clamped <= kRouteDMm) {
      const float t = static_cast<float>(clamped - kRouteCMm) / kStraightMm;
      out_x = static_cast<int>(kCx + kPixelRadius);
      out_y = static_cast<int>(kUpperY + (kLowerY - kUpperY) * t);
    } else {
      const float angle = kPi * static_cast<float>(clamped - kRouteDMm) / kArcMm;
      out_x = static_cast<int>(kCx + kPixelRadius * std::cos(angle));
      out_y = static_cast<int>(kLowerY + kPixelRadius * std::sin(angle));
    }
  };

  const int disp_mm = static_cast<int>(std::abs(static_cast<long>(model.carDisplacementMm())));
  const int clamped_disp = disp_mm > kRouteTotalMm ? kRouteTotalMm : disp_mm;

  if (!car_fresh) {
    int sx, sy; routeToPixel(0, sx, sy);
    lv_obj_set_pos(track_marker_, sx - kHalfMarker, sy - kHalfMarker);
    lv_obj_set_style_bg_color(track_marker_, color(kMuted), 0);
  } else {
    int mx, my; routeToPixel(clamped_disp, mx, my);
    lv_obj_set_pos(track_marker_, mx - kHalfMarker, my - kHalfMarker);
    lv_obj_set_style_bg_color(track_marker_, color(kBlue), 0);
  }

  // 路点颜色：已通过为绿色，未到达为琥珀色。
  const uint32_t wp_passed = kGreen;
  const uint32_t wp_pending = kAmber;
  lv_obj_set_style_bg_color(track_wp_b_, color(disp_mm >= kRouteBMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_bg_color(track_wp_c_, color(disp_mm >= kRouteCMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_bg_color(track_wp_d_, color(disp_mm >= kRouteDMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_bg_color(track_wp_a_, color(disp_mm >= kRouteTotalMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_text_color(track_label_b_, color(disp_mm >= kRouteBMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_text_color(track_label_c_, color(disp_mm >= kRouteCMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_text_color(track_label_d_, color(disp_mm >= kRouteDMm ? wp_passed : wp_pending), 0);
  lv_obj_set_style_text_color(track_label_a_, color(disp_mm >= kRouteTotalMm ? wp_passed : wp_pending), 0);

  if (car_fresh) {
    char fixed_value[24];
    lv_label_set_text_fmt(car_state_label_, "STATE  %s", carStateName(model.carState()));
    lv_label_set_text_fmt(turn_label_, "TURN   %s", turnName(model.carTurn()));
    formatMilliValue(model.carDisplacementMm(), fixed_value, sizeof(fixed_value));
    lv_label_set_text_fmt(displacement_label_, "DIST   %s m", fixed_value);
    formatMilliValue(model.carVelocityMmS(), fixed_value, sizeof(fixed_value));
    lv_label_set_text_fmt(velocity_label_, "SPEED  %s m/s", fixed_value);
    formatMilliValue(model.carLineErrorMilli(), fixed_value, sizeof(fixed_value));
    lv_label_set_text_fmt(line_error_label_, "LINE   %s", fixed_value);
    lv_label_set_text_fmt(quality_label_, "QUALITY 0x%04X", model.carQualityFlags());
  } else {
    lv_label_set_text(car_state_label_, "STATE  STALE");
    lv_label_set_text(turn_label_, "TURN   --");
    lv_label_set_text(displacement_label_, "DIST   --");
    lv_label_set_text(velocity_label_, "SPEED  --");
    lv_label_set_text(line_error_label_, "LINE   --");
    lv_label_set_text(quality_label_, "QUALITY ----");
  }

  const uint16_t faults = model.visibleFaults();
  const uint32_t now_ms = millis();
  const uint32_t fps_elapsed_ms = now_ms - fps_sample_ms_;
  if (fps_elapsed_ms >= kFpsSamplePeriodMs) {
    const uint32_t current_vsync = lvgl_port_get_vsync_count();
    const uint32_t fps = (current_vsync - fps_sample_vsync_) * 1000U / fps_elapsed_ms;
    lv_label_set_text_fmt(fps_label_, "FPS %lu", static_cast<unsigned long>(fps));
    fps_sample_ms_ = now_ms;
    fps_sample_vsync_ = current_vsync;
  }
  if (faults == d_task::FAULT_NONE) {
    lv_obj_set_style_bg_color(fault_bar_, color(kGreen), 0);
    lv_label_set_text(fault_label_, "SYSTEM NOMINAL");
  } else {
    lv_obj_set_style_bg_color(fault_bar_, color(kRed), 0);
    // 拼接人类可读故障名称，便于快速定位。
    char buf[96];
    size_t n = 0;
    auto append = [&](const char *tag) {
      if (n > 0 && n < sizeof(buf) - 1) buf[n++] = '/';
      while (*tag && n < sizeof(buf) - 1) buf[n++] = *tag++;
    };
    if (faults & d_task::FAULT_LINE_LOST)               append("LINE");
    if (faults & d_task::FAULT_WIFI_TIMEOUT)             append("WIFI");
    if (faults & d_task::FAULT_STALE_DATA)               append("STALE");
    if (faults & d_task::FAULT_NO_COMMITTED_SELECTION)   append("NO_TASK");
    if (faults & d_task::FAULT_ENCODER_DISAGREE)         append("ENC");
    if (faults & d_task::FAULT_MOTOR)                    append("MOTOR");
    if (faults & d_task::FAULT_PID_OVERRUN)              append("PID");
    if (faults & d_task::FAULT_BUTTON_STUCK)             append("BTN");
    if (faults & d_task::FAULT_PROTOCOL)                 append("PROTO");
    if (faults & d_task::FAULT_BROWNOUT)                 append("BROWNOUT");
    if (n == 0) { snprintf(buf, sizeof(buf), "0x%04X", faults); n = strlen(buf); }
    else if (n < sizeof(buf) - 8) { n += snprintf(buf + n, sizeof(buf) - n, " 0x%04X", faults); }
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    buf[n] = '\0';
    lv_label_set_text(fault_label_, buf);
  }
  lvgl_port_unlock();
}

#endif
