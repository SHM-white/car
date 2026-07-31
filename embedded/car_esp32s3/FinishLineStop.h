#pragma once

#include <math.h>
#include <stdint.h>

#include "Config.h"

static_assert(car_config::FINISH_LINE_ARM_DISTANCE_M > 0.0F,
              "finish-line arm distance must be positive");
static_assert(car_config::FINISH_LINE_RUNOUT_M > 0.0F,
              "finish-line runout must be positive");

class FinishLineStop {
 public:
  void reset() {
    candidate_active_ = false;
    marker_detected_ = false;
    stop_reached_ = false;
    candidate_since_ms_ = 0;
    marker_distance_m_ = 0.0F;
  }

  bool update(float displacement_m, bool all_channels_black, uint32_t now_ms) {
    if (stop_reached_) return true;

    const float distance_m = fabsf(displacement_m);
    if (!marker_detected_) {
      if (distance_m < car_config::FINISH_LINE_ARM_DISTANCE_M || !all_channels_black) {
        candidate_active_ = false;
        return false;
      }

      if (!candidate_active_) {
        candidate_active_ = true;
        candidate_since_ms_ = now_ms;
        marker_distance_m_ = distance_m;
      }
      if (now_ms - candidate_since_ms_ < car_config::FINISH_LINE_CONFIRM_MS) return false;

      marker_detected_ = true;
      candidate_active_ = false;
    }

    if (distance_m - marker_distance_m_ >= car_config::FINISH_LINE_RUNOUT_M) {
      stop_reached_ = true;
    }
    return stop_reached_;
  }

  bool markerDetected() const { return marker_detected_; }

 private:
  bool candidate_active_ = false;
  bool marker_detected_ = false;
  bool stop_reached_ = false;
  uint32_t candidate_since_ms_ = 0;
  float marker_distance_m_ = 0.0F;
};
