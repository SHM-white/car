#pragma once

#include <stddef.h>
#include <stdint.h>

class TwoWire {
 public:
  bool begin(int, int, uint32_t) { return true; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint16_t address) { address_ = address; write_count_ = 0; }
  size_t write(uint8_t value) {
    if (write_count_++ == 0) command_ = value;
    return 1;
  }
  uint8_t endTransmission(bool = true) { return address_ >= 0x4C && address_ <= 0x4F ? 0 : 2; }
  size_t requestFrom(uint16_t, size_t length, bool = true) {
    read_index_ = 0;
    read_length_ = length;
    return length;
  }
  int available() const { return read_index_ < read_length_ ? 1 : 0; }
  int read() {
    // I2C 通道 1 位于板子的最右侧；模拟黑线时返回 0。
    static constexpr uint8_t kAnalog[8] = {0, 255, 255, 255, 255, 255, 255, 255};
    uint8_t value = 0;
    if (command_ == 0xAA) value = 0x66;
    else if (command_ == 0xC1) value = 0x3E;
    else if (command_ == 0xB0 && read_index_ < sizeof(kAnalog)) value = kAnalog[read_index_];
    ++read_index_;
    return value;
  }

 private:
  uint16_t address_ = 0;
  size_t write_count_ = 0;
  size_t read_index_ = 0;
  size_t read_length_ = 0;
  uint8_t command_ = 0;
};

extern TwoWire Wire;
