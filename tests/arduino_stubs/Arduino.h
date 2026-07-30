#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define CHANGE 4
#define IRAM_ATTR
#define PI 3.14159265358979323846F

using String = std::string;
using BaseType_t = int;
using UBaseType_t = unsigned int;
using TickType_t = uint32_t;
using TaskHandle_t = void *;
using TaskFunction_t = void (*)(void *);

constexpr BaseType_t pdPASS = 1;
#define pdMS_TO_TICKS(milliseconds) (static_cast<TickType_t>(milliseconds))

template <typename T>
T constrain(T value, T minimum, T maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

class HardwareSerial {
 public:
  void begin(uint32_t) {}
  void println(const char *) {}
  int available() const { return 0; }
  int read() { return -1; }
  template <typename... Args>
  int printf(const char *, Args...) { return 0; }
};

extern HardwareSerial Serial;

inline void pinMode(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return LOW; }
inline void digitalWrite(uint8_t, uint8_t) {}
inline uint16_t analogRead(uint8_t) { return 2048; }
inline uint32_t millis() { static uint32_t value = 0; return ++value; }
inline uint32_t micros() { static uint32_t value = 0; value += 100; return value; }
inline void delay(uint32_t) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t, uint32_t) { return true; }
inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(TickType_t *, TickType_t) {}
inline BaseType_t xTaskCreate(TaskFunction_t, const char *, uint32_t, void *,
                              UBaseType_t, TaskHandle_t *) { return pdPASS; }
