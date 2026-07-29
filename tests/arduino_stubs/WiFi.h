#pragma once

#include "Arduino.h"
#include "IPAddress.h"

constexpr int WIFI_STA = 1;
constexpr int WL_CONNECTED = 3;

class WiFiClass {
 public:
  void mode(int) {}
  void setAutoReconnect(bool) {}
  void begin(const char *, const char *) {}
  int status() const { return WL_CONNECTED; }
  IPAddress localIP() const { return IPAddress(192, 168, 20, 3); }
};

extern WiFiClass WiFi;

