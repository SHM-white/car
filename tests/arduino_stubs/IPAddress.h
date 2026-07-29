#pragma once

#include "Arduino.h"

class IPAddress {
 public:
  IPAddress() = default;
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : value_{a, b, c, d} {}
  bool operator==(const IPAddress &other) const {
    for (uint8_t i = 0; i < 4; ++i) if (value_[i] != other.value_[i]) return false;
    return true;
  }
  bool operator!=(const IPAddress &other) const { return !(*this == other); }
  String toString() const { return "192.168.20.3"; }

 private:
  uint8_t value_[4]{};
};

