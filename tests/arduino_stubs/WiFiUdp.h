#pragma once

#include "Arduino.h"
#include "IPAddress.h"

class WiFiUDP {
 public:
  uint8_t begin(uint16_t) { return 1; }
  void stop() {}
  int parsePacket() { return 0; }
  IPAddress remoteIP() const { return IPAddress(); }
  uint16_t remotePort() const { return 0; }
  int available() const { return 0; }
  int read() { return -1; }
  int read(uint8_t *, size_t) { return 0; }
  int beginPacket(const IPAddress &, uint16_t) { return 1; }
  size_t write(const uint8_t *, size_t length) { return length; }
  int endPacket() { return 1; }
};

