#include "Hardware.h"

#if defined(ARDUINO_ARCH_ESP32)

Encoders *Encoders::instance_ = nullptr;

void IRAM_ATTR Encoders::leftInterrupt() {
  if (instance_ == nullptr) return;
  instance_->left_count_ +=
      digitalRead(car_config::LEFT_ENCODER_A_PIN) == digitalRead(car_config::LEFT_ENCODER_B_PIN) ? 1 : -1;
}

void IRAM_ATTR Encoders::rightInterrupt() {
  if (instance_ == nullptr) return;
  instance_->right_count_ +=
      digitalRead(car_config::RIGHT_ENCODER_A_PIN) == digitalRead(car_config::RIGHT_ENCODER_B_PIN) ? 1 : -1;
}

#endif

