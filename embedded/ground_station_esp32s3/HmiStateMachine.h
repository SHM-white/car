#pragma once

#include <stdint.h>
#include <DTaskProtocol.h>

enum class HmiState : uint8_t {
  BOOT_WAITING = 0,
  PRESTART,
  SELECT_PENDING,
  SELECTED,
  ARMED_READY,
  CAR_RUNNING,
  COMPLETE,
  FAULT,
};

class HmiStateMachine {
 public:
  void setLocalBootId(uint32_t boot_id) { local_boot_id_ = boot_id; }

  void onCarTelemetry(uint32_t car_boot_id, const d_task::CarTelemetry &telemetry) {
    if (car_boot_id_ != car_boot_id) {
      // 车辆重启后旧选择必然失效，必须重新取得 ROS 的当前纪元确认。
      car_boot_id_ = car_boot_id;
      selected_task_ = 0; pending_task_ = 0; selection_id_ = 0;
      selection_committed_ = false; selection_pending_send_ = false;
      ros_prestart_confirmed_ = false; state_ = HmiState::BOOT_WAITING;
      mission_status_flags_ = 0; car_event_ = d_task::RouteEvent::NONE; car_event_id_ = 0;
      mission_phase_ = d_task::MissionPhase::PRESTART;
    }
    car_state_ = telemetry.state;
    car_turn_ = telemetry.turn;
    car_event_ = telemetry.event;
    car_event_id_ = telemetry.event_id;
    car_displacement_mm_ = telemetry.displacement_mm;
    car_velocity_mm_s_ = telemetry.velocity_mm_s;
    car_line_error_milli_ = telemetry.line_error_milli;
    car_quality_flags_ = telemetry.quality_flags;
    car_faults_ = telemetry.fault_flags;
    if (telemetry.state == d_task::CarState::RUNNING) {
      if (!selection_committed_) enterFault(d_task::FAULT_NO_COMMITTED_SELECTION, true);
      else state_ = HmiState::CAR_RUNNING;
    } else if (telemetry.state == d_task::CarState::COMPLETE) {
      state_ = selection_committed_ ? HmiState::COMPLETE : HmiState::FAULT;
    } else if (telemetry.state == d_task::CarState::SAFE_STOP) {
      enterFault(telemetry.fault_flags, true);
    }
  }

  bool onMissionStatus(const d_task::MissionStatus &status) {
    // 回执必须同时绑定本次 HMI 启动和当前车辆启动，旧会话包只能被拒绝。
    if (status.hmi_boot_id != local_boot_id_ || status.car_boot_id != car_boot_id_) return false;
    ros_prestart_confirmed_ = status.phase == d_task::MissionPhase::PRESTART;
    ros_reason_flags_ = status.reason_flags;
    mission_status_flags_ = status.status_flags;
    mission_phase_ = status.phase;
    if (status.phase == d_task::MissionPhase::FAULT) { enterFault(status.reason_flags, true); return true; }
    if (selection_pending_send_ && status.phase == d_task::MissionPhase::SELECTION_ACKED &&
        status.selection_id == selection_id_ && status.selected_task == pending_task_) {
      selected_task_ = pending_task_; selection_committed_ = true; selection_pending_send_ = false;
      if (car_state_ == d_task::CarState::RUNNING) state_ = HmiState::CAR_RUNNING;
      else if (car_state_ == d_task::CarState::COMPLETE) state_ = HmiState::COMPLETE;
      else if (car_state_ == d_task::CarState::SAFE_STOP) state_ = HmiState::FAULT;
      else state_ = HmiState::SELECTED;
    }
    if (selection_committed_ && status.selection_id == selection_id_) {
      if (status.phase == d_task::MissionPhase::ARMED_READY) state_ = HmiState::ARMED_READY;
      else if (status.phase == d_task::MissionPhase::CAR_RUNNING) state_ = HmiState::CAR_RUNNING;
      else if (status.phase == d_task::MissionPhase::COMPLETE) state_ = HmiState::COMPLETE;
    }
    return true;
  }

  void updateLinks(bool wifi_connected, bool car_fresh, bool ros_fresh) {
    const bool all_fresh = wifi_connected && car_fresh && ros_fresh;
    if (!all_fresh) {
      const uint16_t reason = d_task::FAULT_STALE_DATA;
      if (state_ == HmiState::CAR_RUNNING || state_ == HmiState::ARMED_READY || state_ == HmiState::COMPLETE) enterFault(reason, true);
      else {
        state_ = HmiState::BOOT_WAITING; transient_faults_ = reason;
      }
      return;
    }
    transient_faults_ = 0;
    if (!fault_latched_ && state_ == HmiState::BOOT_WAITING && ros_prestart_confirmed_ &&
        car_state_ == d_task::CarState::READY && car_boot_id_ != 0) state_ = HmiState::PRESTART;
  }

  bool chooseTask(uint8_t task) {
    if (task != 1 && task != 2) return false;
    pending_task_ = task; confirmation_visible_ = true; return true;
  }

  bool cancelChoice() {
    if (!confirmation_visible_) return false;
    pending_task_ = 0; confirmation_visible_ = false; return true;
  }

  bool confirmChoice(uint32_t selection_id) {
    if (!confirmation_visible_ || pending_task_ == 0) return false;
    selection_id_ = selection_id == 0 ? 1 : selection_id;
    confirmation_visible_ = false; selection_pending_send_ = true; state_ = HmiState::SELECT_PENDING; return true;
  }

  bool selectionNeedsSending() const { return selection_pending_send_; }
  d_task::TaskSelection selection() const { return {selection_id_, car_boot_id_, pending_task_}; }
  HmiState state() const { return state_; }
  uint8_t selectedTask() const { return selected_task_; }
  uint8_t pendingTask() const { return pending_task_; }
  uint32_t carBootId() const { return car_boot_id_; }
  d_task::CarState carState() const { return car_state_; }
  d_task::TurnClass carTurn() const { return car_turn_; }
  d_task::RouteEvent carRouteEvent() const { return car_event_; }
  uint16_t carEventId() const { return car_event_id_; }
  int32_t carDisplacementMm() const { return car_displacement_mm_; }
  int16_t carVelocityMmS() const { return car_velocity_mm_s_; }
  int16_t carLineErrorMilli() const { return car_line_error_milli_; }
  uint16_t carQualityFlags() const { return car_quality_flags_; }
  d_task::MissionPhase missionPhase() const { return mission_phase_; }
  uint16_t missionStatusFlags() const { return mission_status_flags_; }
  uint16_t visibleFaults() const { return fault_flags_ | transient_faults_ | car_faults_ | ros_reason_flags_; }
  bool confirmationVisible() const { return confirmation_visible_; }

 private:
  void enterFault(uint16_t reason, bool latch) {
    fault_flags_ |= reason; fault_latched_ |= latch; state_ = HmiState::FAULT;
    selection_committed_ = false;
  }

  HmiState state_ = HmiState::BOOT_WAITING;
  d_task::CarState car_state_ = d_task::CarState::READY;
  d_task::TurnClass car_turn_ = d_task::TurnClass::STRAIGHT;
  d_task::RouteEvent car_event_ = d_task::RouteEvent::NONE;
  d_task::MissionPhase mission_phase_ = d_task::MissionPhase::PRESTART;
  uint32_t local_boot_id_ = 0;
  uint32_t car_boot_id_ = 0;
  uint32_t selection_id_ = 0;
  uint8_t pending_task_ = 0;
  uint8_t selected_task_ = 0;
  uint16_t car_event_id_ = 0;
  uint16_t fault_flags_ = 0;
  uint16_t transient_faults_ = 0;
  uint16_t car_faults_ = 0;
  uint16_t ros_reason_flags_ = 0;
  uint16_t mission_status_flags_ = 0;
  uint16_t car_quality_flags_ = 0;
  int32_t car_displacement_mm_ = 0;
  int16_t car_velocity_mm_s_ = 0;
  int16_t car_line_error_milli_ = 0;
  bool confirmation_visible_ = false;
  bool selection_committed_ = false;
  bool selection_pending_send_ = false;
  bool ros_prestart_confirmed_ = false;
  bool fault_latched_ = false;
};
