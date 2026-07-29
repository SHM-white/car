#pragma once

#include <stdint.h>

enum esp_reset_reason_t { ESP_RST_UNKNOWN = 0, ESP_RST_BROWNOUT = 15 };
inline uint32_t esp_random() { return 0x12345678; }
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_UNKNOWN; }

