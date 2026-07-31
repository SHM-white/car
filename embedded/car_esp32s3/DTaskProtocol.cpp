#include "DTaskProtocol.h"

#include <string.h>

namespace d_task {
namespace {

void put16(uint8_t *buffer, uint16_t value) {
  buffer[0] = static_cast<uint8_t>(value);
  buffer[1] = static_cast<uint8_t>(value >> 8);
}

void put32(uint8_t *buffer, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) buffer[i] = static_cast<uint8_t>(value >> (8U * i));
}

uint16_t get16(const uint8_t *buffer) {
  return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
}

uint32_t get32(const uint8_t *buffer) {
  return static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8) |
         (static_cast<uint32_t>(buffer[2]) << 16) | (static_cast<uint32_t>(buffer[3]) << 24);
}

uint32_t rotateRight(uint32_t value, uint8_t bits) {
  return (value >> bits) | (value << (32U - bits));
}

constexpr uint32_t kShaConstants[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

class Sha256 {
 public:
  Sha256() { reset(); }

  void reset() {
    state_[0] = 0x6a09e667; state_[1] = 0xbb67ae85; state_[2] = 0x3c6ef372; state_[3] = 0xa54ff53a;
    state_[4] = 0x510e527f; state_[5] = 0x9b05688c; state_[6] = 0x1f83d9ab; state_[7] = 0x5be0cd19;
    memset(buffer_, 0, sizeof(buffer_));
    total_bytes_ = 0; buffer_length_ = 0;
  }

  void update(const uint8_t *data, size_t length) {
    if (data == nullptr || length == 0) return;
    total_bytes_ += length;
    while (length > 0) {
      const size_t available = 64 - buffer_length_;
      const size_t count = length < available ? length : available;
      memcpy(buffer_ + buffer_length_, data, count);
      buffer_length_ += count; data += count; length -= count;
      if (buffer_length_ == 64) { transform(buffer_); buffer_length_ = 0; }
    }
  }

  void finish(uint8_t output[32]) {
    const uint64_t bit_length = total_bytes_ * 8U;
    buffer_[buffer_length_++] = 0x80;
    if (buffer_length_ > 56) {
      while (buffer_length_ < 64) buffer_[buffer_length_++] = 0;
      transform(buffer_); buffer_length_ = 0;
    }
    while (buffer_length_ < 56) buffer_[buffer_length_++] = 0;
    for (uint8_t i = 0; i < 8; ++i) buffer_[63 - i] = static_cast<uint8_t>(bit_length >> (8U * i));
    transform(buffer_);
    for (uint8_t i = 0; i < 8; ++i) {
      output[4 * i] = static_cast<uint8_t>(state_[i] >> 24);
      output[4 * i + 1] = static_cast<uint8_t>(state_[i] >> 16);
      output[4 * i + 2] = static_cast<uint8_t>(state_[i] >> 8);
      output[4 * i + 3] = static_cast<uint8_t>(state_[i]);
    }
  }

 private:
  void transform(const uint8_t block[64]) {
    uint32_t schedule[64];
    for (uint8_t i = 0; i < 16; ++i) {
      schedule[i] = (static_cast<uint32_t>(block[4 * i]) << 24) |
                    (static_cast<uint32_t>(block[4 * i + 1]) << 16) |
                    (static_cast<uint32_t>(block[4 * i + 2]) << 8) | block[4 * i + 3];
    }
    for (uint8_t i = 16; i < 64; ++i) {
      const uint32_t s0 = rotateRight(schedule[i - 15], 7) ^ rotateRight(schedule[i - 15], 18) ^ (schedule[i - 15] >> 3);
      const uint32_t s1 = rotateRight(schedule[i - 2], 17) ^ rotateRight(schedule[i - 2], 19) ^ (schedule[i - 2] >> 10);
      schedule[i] = schedule[i - 16] + s0 + schedule[i - 7] + s1;
    }
    uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (uint8_t i = 0; i < 64; ++i) {
      const uint32_t s1 = rotateRight(e, 6) ^ rotateRight(e, 11) ^ rotateRight(e, 25);
      const uint32_t choice = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = h + s1 + choice + kShaConstants[i] + schedule[i];
      const uint32_t s0 = rotateRight(a, 2) ^ rotateRight(a, 13) ^ rotateRight(a, 22);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = s0 + majority;
      h = g; g = f; f = e; e = d + temp1; d = c; c = b; b = a; a = temp1 + temp2;
    }
    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
  }

  uint32_t state_[8];
  uint8_t buffer_[64];
  uint64_t total_bytes_;
  size_t buffer_length_;
};

}  // namespace

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

void hmacSha256(const uint8_t *key, size_t key_length, const uint8_t *data,
                size_t data_length, uint8_t output[32]) {
  uint8_t normalized_key[64] = {0};
  if (key_length > 64) {
    Sha256 hash; hash.update(key, key_length); hash.finish(normalized_key);
  } else if (key != nullptr && key_length > 0) {
    memcpy(normalized_key, key, key_length);
  }
  uint8_t inner_pad[64], outer_pad[64], inner_digest[32];
  for (uint8_t i = 0; i < 64; ++i) { inner_pad[i] = normalized_key[i] ^ 0x36; outer_pad[i] = normalized_key[i] ^ 0x5c; }
  Sha256 inner; inner.update(inner_pad, sizeof(inner_pad)); inner.update(data, data_length); inner.finish(inner_digest);
  Sha256 outer; outer.update(outer_pad, sizeof(outer_pad)); outer.update(inner_digest, sizeof(inner_digest)); outer.finish(output);
}

bool constantTimeEqual(const uint8_t *left, const uint8_t *right, size_t length) {
  uint8_t difference = 0;
  for (size_t i = 0; i < length; ++i) difference |= left[i] ^ right[i];
  return difference == 0;
}

bool isKnownMessageType(uint8_t type) {
  return type >= static_cast<uint8_t>(MessageType::HEARTBEAT) && type <= static_cast<uint8_t>(MessageType::DIAGNOSTIC);
}

bool encodePacket(const PacketHeader &header, const uint8_t *payload,
                  const uint8_t *auth_key, size_t auth_key_length,
                  uint8_t *output, size_t output_capacity, size_t &output_length) {
  output_length = 0;
  if (output == nullptr || auth_key == nullptr || auth_key_length == 0 ||
      header.payload_length > kMaxPayloadSize ||
      (header.payload_length > 0 && payload == nullptr) ||
      !isKnownMessageType(static_cast<uint8_t>(header.type))) return false;
  const size_t required = kHeaderSize + header.payload_length + kCrcSize + kAuthTagSize;
  if (output_capacity < required) return false;
  put16(output, kMagic); output[2] = kVersion; output[3] = static_cast<uint8_t>(header.type);
  put16(output + 4, header.payload_length); put32(output + 6, header.sender_id);
  put32(output + 10, header.boot_id); put32(output + 14, header.sequence); put32(output + 18, header.monotonic_ms);
  if (header.payload_length > 0) memcpy(output + kHeaderSize, payload, header.payload_length);
  const size_t crc_offset = kHeaderSize + header.payload_length;
  put16(output + crc_offset, crc16Ccitt(output, crc_offset));
  uint8_t digest[32]; hmacSha256(auth_key, auth_key_length, output, crc_offset + kCrcSize, digest);
  memcpy(output + crc_offset + kCrcSize, digest, kAuthTagSize);
  output_length = required;
  return true;
}

DecodeResult decodePacket(const uint8_t *packet, size_t packet_length,
                          const uint8_t *auth_key, size_t auth_key_length,
                          PacketHeader &header, const uint8_t *&payload) {
  payload = nullptr;
  if (packet == nullptr || packet_length < kHeaderSize + kCrcSize + kAuthTagSize) return DecodeResult::TOO_SHORT;
  if (packet_length > kMaxPacketSize) return DecodeResult::TOO_LONG;
  if (get16(packet) != kMagic) return DecodeResult::BAD_MAGIC;
  if (packet[2] != kVersion) return DecodeResult::BAD_VERSION;
  if (!isKnownMessageType(packet[3])) return DecodeResult::BAD_TYPE;
  const uint16_t payload_length = get16(packet + 4);
  if (payload_length > kMaxPayloadSize || packet_length != kHeaderSize + payload_length + kCrcSize + kAuthTagSize) return DecodeResult::BAD_LENGTH;
  const size_t crc_offset = kHeaderSize + payload_length;
  if (get16(packet + crc_offset) != crc16Ccitt(packet, crc_offset)) return DecodeResult::BAD_CRC;
  if (auth_key == nullptr || auth_key_length == 0) return DecodeResult::BAD_AUTH;
  uint8_t digest[32]; hmacSha256(auth_key, auth_key_length, packet, crc_offset + kCrcSize, digest);
  if (!constantTimeEqual(packet + crc_offset + kCrcSize, digest, kAuthTagSize)) return DecodeResult::BAD_AUTH;
  header.type = static_cast<MessageType>(packet[3]); header.payload_length = payload_length;
  header.sender_id = get32(packet + 6); header.boot_id = get32(packet + 10);
  header.sequence = get32(packet + 14); header.monotonic_ms = get32(packet + 18);
  payload = packet + kHeaderSize;
  return DecodeResult::OK;
}

size_t encodeCarTelemetry(const CarTelemetry &m, uint8_t *out, size_t capacity) {
  constexpr size_t kSize = 17; if (out == nullptr || capacity < kSize) return 0;
  out[0] = static_cast<uint8_t>(m.state); out[1] = static_cast<uint8_t>(m.turn); out[2] = static_cast<uint8_t>(m.event);
  put16(out + 3, m.event_id); put16(out + 5, m.quality_flags); put32(out + 7, static_cast<uint32_t>(m.displacement_mm));
  put16(out + 11, static_cast<uint16_t>(m.velocity_mm_s)); put16(out + 13, static_cast<uint16_t>(m.line_error_milli)); put16(out + 15, m.fault_flags);
  return kSize;
}

bool decodeCarTelemetry(const uint8_t *p, size_t length, CarTelemetry &m) {
  if (p == nullptr || length != 17 || p[0] > static_cast<uint8_t>(CarState::SAFE_STOP) ||
      p[1] > static_cast<uint8_t>(TurnClass::LARGE) || p[2] > static_cast<uint8_t>(RouteEvent::COMPLETE)) return false;
  m.state = static_cast<CarState>(p[0]); m.turn = static_cast<TurnClass>(p[1]); m.event = static_cast<RouteEvent>(p[2]);
  m.event_id = get16(p + 3); m.quality_flags = get16(p + 5); m.displacement_mm = static_cast<int32_t>(get32(p + 7));
  m.velocity_mm_s = static_cast<int16_t>(get16(p + 11)); m.line_error_milli = static_cast<int16_t>(get16(p + 13)); m.fault_flags = get16(p + 15);
  return true;
}

size_t encodeTaskSelection(const TaskSelection &m, uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < 9 || m.task < 1 || m.task > 3) return 0;
  put32(out, m.selection_id); put32(out + 4, m.car_boot_id); out[8] = m.task; return 9;
}

bool decodeTaskSelection(const uint8_t *p, size_t length, TaskSelection &m) {
  if (p == nullptr || length != 9 || p[8] < 1 || p[8] > 3) return false;
  m.selection_id = get32(p); m.car_boot_id = get32(p + 4); m.task = p[8]; return true;
}

size_t encodeMissionStatus(const MissionStatus &m, uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < 18) return 0;
  put32(out, m.selection_id); put32(out + 4, m.car_boot_id); put32(out + 8, m.hmi_boot_id);
  out[12] = static_cast<uint8_t>(m.phase); out[13] = m.selected_task;
  put16(out + 14, m.reason_flags); put16(out + 16, m.status_flags); return 18;
}

bool decodeMissionStatus(const uint8_t *p, size_t length, MissionStatus &m) {
  constexpr uint16_t kKnownStatusFlags = MISSION_DRONE_LINK_OK | MISSION_DRONE_ARMED |
                                         MISSION_VISION_VALID | MISSION_ROS_READY;
  if (p == nullptr || length != 18 || p[12] > static_cast<uint8_t>(MissionPhase::FAULT) ||
      p[13] > 3 || (get16(p + 16) & ~kKnownStatusFlags) != 0) return false;
  m.selection_id = get32(p); m.car_boot_id = get32(p + 4); m.hmi_boot_id = get32(p + 8);
  m.phase = static_cast<MissionPhase>(p[12]); m.selected_task = p[13];
  m.reason_flags = get16(p + 14); m.status_flags = get16(p + 16); return true;
}

SequenceTracker::SequenceTracker() { reset(); }
void SequenceTracker::beginSession(uint32_t sender_id, uint32_t boot_id) { sender_id_ = sender_id; boot_id_ = boot_id; has_sequence_ = false; session_active_ = true; }
bool SequenceTracker::accept(uint32_t sender_id, uint32_t boot_id, uint32_t sequence) {
  if (!session_active_ || sender_id != sender_id_ || boot_id != boot_id_) return false;
  if (!has_sequence_) { last_sequence_ = sequence; has_sequence_ = true; return true; }
  // 差值落在前半个 uint32 空间才算新包，既支持回绕，也拒绝旧包和重复包。
  const uint32_t forward_distance = sequence - last_sequence_;
  if (forward_distance == 0 || forward_distance >= 0x80000000UL) return false;
  last_sequence_ = sequence; return true;
}
void SequenceTracker::reset() { sender_id_ = boot_id_ = last_sequence_ = 0; session_active_ = has_sequence_ = false; }

}  // namespace d_task
