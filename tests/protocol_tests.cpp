#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "DTaskProtocol.h"
#include "HmiStateMachine.h"

namespace {

constexpr uint8_t kKey[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
};

void testCrcAndHmac() {
  const uint8_t crc_input[] = "123456789";
  assert(d_task::crc16Ccitt(crc_input, 9) == 0x29B1);

  const uint8_t key[] = "key";
  const uint8_t message[] = "The quick brown fox jumps over the lazy dog";
  const uint8_t expected[32] = {
      0xf7, 0xbc, 0x83, 0xf4, 0x30, 0x53, 0x84, 0x24,
      0xb1, 0x32, 0x98, 0xe6, 0xaa, 0x6f, 0xb1, 0x43,
      0xef, 0x4d, 0x59, 0xa1, 0x49, 0x46, 0x17, 0x59,
      0x97, 0x47, 0x9d, 0xbc, 0x2d, 0x1a, 0x3c, 0xd8,
  };
  uint8_t actual[32];
  d_task::hmacSha256(key, 3, message, 43, actual);
  assert(memcmp(actual, expected, sizeof(expected)) == 0);
}

void testGoldenPacketAndTampering() {
  d_task::CarTelemetry telemetry{d_task::CarState::RUNNING, d_task::TurnClass::SMALL,
      d_task::RouteEvent::B, 7, static_cast<uint16_t>(d_task::QUALITY_LINE_VALID | d_task::QUALITY_ENCODER_VALID),
      -1234, 321, -125, d_task::FAULT_NONE};
  uint8_t payload[64];
  const size_t payload_length = d_task::encodeCarTelemetry(telemetry, payload, sizeof(payload));
  assert(payload_length == 17);

  d_task::PacketHeader header{d_task::MessageType::CAR_TELEMETRY, static_cast<uint16_t>(payload_length),
                              0x43415231, 0x10203040, 0xFFFFFFFE, 0x01020304};
  uint8_t packet[d_task::kMaxPacketSize]; size_t packet_length = 0;
  assert(d_task::encodePacket(header, payload, kKey, sizeof(kKey), packet, sizeof(packet), packet_length));
  assert(packet_length == 49);

  // 固定向量可供 ROS/Python 端逐字节对照，防止结构体布局或字节序悄然变化。
  const uint8_t expected[] = {
      0x54, 0x44, 0x01, 0x02, 0x11, 0x00, 0x31, 0x52, 0x41, 0x43, 0x40, 0x30, 0x20, 0x10,
      0xfe, 0xff, 0xff, 0xff, 0x04, 0x03, 0x02, 0x01, 0x01, 0x01, 0x02, 0x07, 0x00, 0x03,
      0x00, 0x2e, 0xfb, 0xff, 0xff, 0x41, 0x01, 0x83, 0xff, 0x00, 0x00,
      0x50, 0xee, 0x78, 0xe8, 0x28, 0x7a, 0xb8, 0x44, 0x85, 0x4e};
  assert(sizeof(expected) == packet_length);
  assert(memcmp(packet, expected, packet_length) == 0);

  d_task::PacketHeader decoded{}; const uint8_t *decoded_payload = nullptr;
  assert(d_task::decodePacket(packet, packet_length, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::OK);
  d_task::CarTelemetry decoded_telemetry{};
  assert(d_task::decodeCarTelemetry(decoded_payload, decoded.payload_length, decoded_telemetry));
  assert(decoded_telemetry.displacement_mm == -1234);
  assert(decoded_telemetry.velocity_mm_s == 321);
  assert(decoded_telemetry.event_id == 7);

  uint8_t changed[d_task::kMaxPacketSize]; memcpy(changed, packet, packet_length);
  changed[d_task::kHeaderSize + 3] ^= 0x01;
  assert(d_task::decodePacket(changed, packet_length, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::BAD_CRC);
  memcpy(changed, packet, packet_length); changed[packet_length - 1] ^= 0x01;
  assert(d_task::decodePacket(changed, packet_length, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::BAD_AUTH);
  memcpy(changed, packet, packet_length); changed[0] ^= 0x01;
  assert(d_task::decodePacket(changed, packet_length, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::BAD_MAGIC);
  memcpy(changed, packet, packet_length); changed[2] = 2;
  assert(d_task::decodePacket(changed, packet_length, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::BAD_VERSION);
  assert(d_task::decodePacket(packet, packet_length - 1, kKey, sizeof(kKey), decoded, decoded_payload) == d_task::DecodeResult::BAD_LENGTH);
}

void testPayloadValidation() {
  uint8_t payload[18];
  assert(d_task::encodeTaskSelection({42, 0x12345678, 1}, payload, sizeof(payload)) == 9);
  d_task::TaskSelection selection{};
  assert(d_task::decodeTaskSelection(payload, 9, selection));
  assert(selection.selection_id == 42 && selection.car_boot_id == 0x12345678 && selection.task == 1);
  payload[8] = 3;
  assert(!d_task::decodeTaskSelection(payload, 9, selection));

  d_task::MissionStatus status{99, 0xAABBCCDD, 0x11223344, d_task::MissionPhase::ARMED_READY, 2, 0,
      static_cast<uint16_t>(d_task::MISSION_DRONE_LINK_OK | d_task::MISSION_VISION_VALID)};
  assert(d_task::encodeMissionStatus(status, payload, sizeof(payload)) == 18);
  d_task::MissionStatus decoded{};
  assert(d_task::decodeMissionStatus(payload, 18, decoded));
  assert(decoded.phase == d_task::MissionPhase::ARMED_READY && decoded.selected_task == 2);
  assert(decoded.car_boot_id == 0xAABBCCDD && decoded.hmi_boot_id == 0x11223344);
  assert(decoded.status_flags == status.status_flags);
  payload[17] = 0x80;
  assert(!d_task::decodeMissionStatus(payload, 18, decoded));
}

void testSequenceTracker() {
  d_task::SequenceTracker tracker;
  assert(!tracker.accept(1, 2, 1));
  tracker.beginSession(1, 2);
  assert(tracker.accept(1, 2, 0xFFFFFFFE));
  assert(!tracker.accept(1, 2, 0xFFFFFFFE));
  assert(tracker.accept(1, 2, 0xFFFFFFFF));
  assert(tracker.accept(1, 2, 0));
  assert(!tracker.accept(1, 2, 0xFFFFFFFF));
  assert(!tracker.accept(1, 3, 1));
}

d_task::CarTelemetry readyTelemetry() {
  return {d_task::CarState::READY, d_task::TurnClass::STRAIGHT, d_task::RouteEvent::NONE,
          0, static_cast<uint16_t>(d_task::QUALITY_LINE_VALID | d_task::QUALITY_ENCODER_VALID), 0, 0, 0, 0};
}

void testHmiStateMachine() {
  HmiStateMachine no_links;
  assert(no_links.chooseTask(1));
  assert(no_links.confirmChoice(99));
  assert(no_links.selectionNeedsSending());
  assert(no_links.selection().car_boot_id == 0 && no_links.selection().task == 1);

  HmiStateMachine hmi;
  hmi.setLocalBootId(0x11223344);
  hmi.onCarTelemetry(0xAABBCCDD, readyTelemetry());
  hmi.onMissionStatus({0, 0xAABBCCDD, 0x11223344, d_task::MissionPhase::PRESTART, 0, 0,
                       d_task::MISSION_ROS_READY});
  hmi.updateLinks(true, true, true);
  assert(hmi.state() == HmiState::PRESTART);
  assert(hmi.chooseTask(2));
  assert(hmi.confirmationVisible());
  assert(hmi.confirmChoice(1234));
  assert(hmi.state() == HmiState::SELECT_PENDING && hmi.selectionNeedsSending());
  assert(hmi.selection().car_boot_id == 0xAABBCCDD);

  hmi.onMissionStatus({1234, 0xAABBCCDD, 0x11223344, d_task::MissionPhase::SELECTION_ACKED, 2, 0,
                       d_task::MISSION_ROS_READY});
  assert(hmi.state() == HmiState::SELECTED && hmi.selectedTask() == 2);
  assert(hmi.chooseTask(1));
  assert(hmi.confirmationVisible());
  assert(hmi.cancelChoice());
  hmi.onMissionStatus({1234, 0xAABBCCDD, 0x11223344, d_task::MissionPhase::ARMED_READY, 2, 0,
                       static_cast<uint16_t>(d_task::MISSION_ROS_READY | d_task::MISSION_DRONE_LINK_OK |
                                             d_task::MISSION_DRONE_ARMED | d_task::MISSION_VISION_VALID)});
  assert(hmi.state() == HmiState::ARMED_READY);
  assert((hmi.missionStatusFlags() & d_task::MISSION_VISION_VALID) != 0);

  d_task::CarTelemetry running = readyTelemetry(); running.state = d_task::CarState::RUNNING;
  hmi.onCarTelemetry(0xAABBCCDD, running);
  assert(hmi.state() == HmiState::CAR_RUNNING);
  assert(hmi.chooseTask(1));
  assert(hmi.confirmChoice(5678));
  assert(hmi.selectionNeedsSending() && hmi.selection().task == 1);
  hmi.onCarTelemetry(0xAABBCCDD, running);
  assert(hmi.state() == HmiState::CAR_RUNNING && hmi.selectionNeedsSending());
  assert(hmi.onMissionStatus({5678, 0xAABBCCDD, 0x11223344,
                              d_task::MissionPhase::SELECTION_ACKED, 1, 0,
                              d_task::MISSION_ROS_READY}));
  assert(hmi.state() == HmiState::CAR_RUNNING && hmi.selectedTask() == 1);
  assert(!hmi.selectionNeedsSending());
  assert(hmi.chooseTask(2));
  assert(hmi.confirmChoice(6789));
  hmi.updateLinks(true, false, true);
  assert(hmi.state() == HmiState::BOOT_WAITING);
  assert((hmi.visibleFaults() & d_task::FAULT_STALE_DATA) != 0);
  assert(hmi.selectionNeedsSending());
  assert(hmi.chooseTask(1));
  assert(hmi.confirmationVisible());

  HmiStateMachine missing_selection;
  missing_selection.onCarTelemetry(7, running);
  assert(missing_selection.state() == HmiState::FAULT);
  assert((missing_selection.visibleFaults() & d_task::FAULT_NO_COMMITTED_SELECTION) != 0);

  HmiStateMachine reboot;
  reboot.setLocalBootId(9);
  reboot.onCarTelemetry(1, readyTelemetry());
  reboot.onMissionStatus({0, 1, 9, d_task::MissionPhase::PRESTART, 0, 0,
                          d_task::MISSION_ROS_READY});
  reboot.updateLinks(true, true, true);
  assert(reboot.state() == HmiState::PRESTART);
  reboot.onCarTelemetry(2, readyTelemetry());
  assert(reboot.state() == HmiState::BOOT_WAITING);
  reboot.updateLinks(true, true, true);
  assert(reboot.state() == HmiState::BOOT_WAITING);
  assert(reboot.onMissionStatus({0, 2, 9, d_task::MissionPhase::PRESTART, 0, 0,
                                 d_task::MISSION_ROS_READY}));
  reboot.updateLinks(true, true, true);
  assert(reboot.state() == HmiState::PRESTART);

  HmiStateMachine replay;
  replay.setLocalBootId(100);
  replay.onCarTelemetry(50, readyTelemetry());
  assert(!replay.onMissionStatus({0, 50, 99, d_task::MissionPhase::PRESTART, 0, 0,
                                  d_task::MISSION_ROS_READY}));
  replay.updateLinks(true, true, true);
  assert(replay.state() == HmiState::BOOT_WAITING);
}

}  // namespace

int main() {
  testCrcAndHmac();
  testGoldenPacketAndTampering();
  testPayloadValidation();
  testSequenceTracker();
  testHmiStateMachine();
  puts("全部协议与状态机测试通过");
  return 0;
}
