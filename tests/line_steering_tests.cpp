#include <math.h>
#include <stdio.h>

#include "LineSteering.h"

namespace {

void settle(LineSteering &steering, float error, LineSteering::Output &output) {
  // Raw logs are printed every 200 ms while control runs every 5 ms.
  for (int i = 0; i < 40; ++i) output = steering.update(error);
}

}  // namespace

int main() {
  LineSteering steering;
  LineSteering::Output output{};

  steering.reset(0.18F);
  output = steering.update(0.18F);
  const float first_step_limit = car_config::STEERING_APPLY_RATE_PER_S *
                                 car_config::CONTROL_PERIOD_US / 1000000.0F;
  if (output.correction <= 0.0F || output.correction > first_step_limit + 0.0001F) {
    fprintf(stderr, "First correction step is not gradual: correction=%f limit=%f\n",
            output.correction, first_step_limit);
    return 1;
  }
  settle(steering, 0.18F, output);
  if (output.correction <= 0.0F ||
      output.correction > car_config::MAX_STEERING_CORRECTION + 0.0001F ||
      output.base_speed_ratio < car_config::RECENTER_MIN_BASE_SPEED_RATIO - 0.0001F ||
      output.base_speed_ratio > 1.0001F) {
    fprintf(stderr, "Positive recentering left control bounds: correction=%f ratio=%f\n",
            output.correction, output.base_speed_ratio);
    return 2;
  }

  // The supplied straight-line samples cross from about +0.18 to -0.19.
  // The controller must reverse promptly instead of unwinding old integral.
  for (int i = 0; i < 20; ++i) output = steering.update(-0.19F);
  if (output.correction >= 0.0F) {
    fprintf(stderr, "Correction did not reverse promptly: %f\n", output.correction);
    return 3;
  }

  settle(steering, -0.19F, output);
  if (output.correction >= 0.0F ||
      fabsf(output.correction) > car_config::MAX_STEERING_CORRECTION + 0.0001F ||
      output.base_speed_ratio < car_config::RECENTER_MIN_BASE_SPEED_RATIO - 0.0001F ||
      output.base_speed_ratio > 1.0001F) {
    fprintf(stderr, "Negative recentering left control bounds: correction=%f ratio=%f\n",
            output.correction, output.base_speed_ratio);
    return 4;
  }

  settle(steering, 0.0F, output);
  if (fabsf(output.error) > 0.0001F || fabsf(output.correction) > 0.02F ||
      fabsf(output.base_speed_ratio - 1.0F) > 0.0001F) {
    fprintf(stderr, "Centered output did not settle: error=%f correction=%f ratio=%f\n",
            output.error, output.correction, output.base_speed_ratio);
    return 5;
  }

  // Weighted-centroid errors calculated from the supplied 32 straight-line
  // frames. Each value persists for the 200 ms interval between log records.
  const float recorded_errors[] = {
      0.099F,  0.120F,  0.181F,  0.094F, -0.069F, -0.144F, -0.093F,  0.035F,
     -0.188F, -0.090F, -0.049F, -0.089F, -0.037F, -0.076F,  0.011F,  0.105F,
      0.120F,  0.149F,  0.068F,  0.057F,  0.041F,  0.078F, -0.040F, -0.019F,
      0.019F, -0.002F, -0.055F, -0.074F, -0.086F, -0.150F, -0.046F, -0.104F,
  };
  steering.clear();
  for (float recorded_error : recorded_errors) {
    settle(steering, recorded_error, output);
    if (fabsf(output.correction) > car_config::MAX_STEERING_CORRECTION + 0.0001F ||
        2.0F * fabsf(output.correction) >
            car_config::MAX_MOTOR_COMMAND_DIFFERENCE + 0.0001F ||
        output.base_speed_ratio < car_config::RECENTER_MIN_BASE_SPEED_RATIO - 0.0001F ||
        output.base_speed_ratio > 1.0001F) {
      fprintf(stderr, "Recorded replay left control bounds: correction=%f ratio=%f\n",
              output.correction, output.base_speed_ratio);
      return 6;
    }
    if (fabsf(output.error) >= car_config::PID_RECOVERY_START_ERROR &&
        output.correction * output.error <= 0.0F) {
      fprintf(stderr, "Recorded replay steered the wrong way: error=%f correction=%f\n",
              recorded_error, output.correction);
      return 7;
    }
  }

  puts("Line steering recenter test passed");
  return 0;
}
