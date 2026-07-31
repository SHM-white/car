#include <Arduino.h>
#include <esp_system.h>

#include "DisplayAdapter.h"
#include "HmiStateMachine.h"
#include "UdpLink.h"

HmiStateMachine hmi;
HMI_DISPLAY_DRIVER display;
UdpLink udp_link;
uint32_t next_selection_send_ms = 0;
uint32_t next_heartbeat_ms = 0;
uint32_t next_render_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[自检] ESP32-S3 地面站启动，任务控件保持可用");
  if (!display.begin()) {
    // 显示自检失败时触摸不会产生动作，因此任务选择保持锁定；串口仍可用于定位硬件问题。
    Serial.println("[自检故障] 地面站显示或触摸不可用，请根据前述日志检查硬件");
  }
  udp_link.begin(millis());
  hmi.setLocalBootId(udp_link.bootId());
}

void loop() {
  const uint32_t now = millis();
  udp_link.update(now);

  // 一次循环清空当前 UDP 队列，避免旧状态堆积后被误显示为新数据。
  for (uint8_t i = 0; i < 8; ++i) {
    const IncomingMessage incoming = udp_link.poll(now);
    if (incoming.kind == IncomingKind::NONE) break;
    if (incoming.kind == IncomingKind::CAR_TELEMETRY) hmi.onCarTelemetry(incoming.sender_boot_id, incoming.car);
    else if (incoming.kind == IncomingKind::MISSION_STATUS && hmi.onMissionStatus(incoming.mission)) {
      udp_link.noteMissionStatusAccepted(now);
    }
  }
  hmi.updateLinks(udp_link.connected(), udp_link.carFresh(now), udp_link.rosFresh(now));

  // 任务提交后锁定三个任务按钮（显示层置灰 + 此处双保险），防止误触改选。
  // 车辆 boot 变化会重置模型 selection，自动解锁。
  switch (display.poll()) {
    case TouchAction::TASK_1:
      if (!hmi.controlsLocked()) hmi.chooseTask(1);
      break;
    case TouchAction::TASK_2:
      if (!hmi.controlsLocked()) hmi.chooseTask(2);
      break;
    // TEST 任务（task 3）走与 TASK 1/2 相同的模型确认流程
    case TouchAction::TASK_TEST:
      if (!hmi.controlsLocked()) hmi.chooseTask(3);
      break;
    case TouchAction::CONFIRM:
      if (hmi.confirmChoice(esp_random())) next_selection_send_ms = now;
      break;
    case TouchAction::CANCEL: hmi.cancelChoice(); break;
    case TouchAction::NONE: break;
  }

  if (hmi.selectionNeedsSending() && static_cast<int32_t>(now - next_selection_send_ms) >= 0) {
    // 重发使用同一 selection_id，ROS 只能幂等确认该选择，不能把重发解释成新任务。
    udp_link.sendSelection(hmi.selection(), now);
    next_selection_send_ms = now + hmi_config::SELECTION_RETRY_MS;
  }
  if (static_cast<int32_t>(now - next_heartbeat_ms) >= 0) {
    // 心跳周期 250ms（HEARTBEAT_PERIOD_MS）：链路新鲜窗口 750ms，周期若接近窗口
    // 会周期性误报 ROS 链路陈旧（FAULT_STALE_DATA）
    next_heartbeat_ms = now + hmi_config::HEARTBEAT_PERIOD_MS;
    udp_link.sendHeartbeat(now);
  }
  if (static_cast<int32_t>(now - next_render_ms) >= 0) {
    next_render_ms = now + 100;
    display.render(hmi, udp_link.carAge(now), udp_link.rosAge(now));
  }
}
