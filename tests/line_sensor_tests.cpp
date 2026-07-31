#include <math.h>
#include <stdio.h>

#include "Hardware.h"

int main() {
  LineSensors sensors;
  if (!sensors.begin()) {
    fprintf(stderr, "I2C line sensor initialization failed\n");
    return 1;
  }

  LineSensorFrame frame{};
  if (!sensors.readFrame(frame)) {
    fprintf(stderr, "I2C line sensor frame read failed\n");
    return 1;
  }
  const LineReading reading = frame.line;
  const float expected_error = car_config::LINE_SENSOR_CHANNELS_REVERSED ? 1.0F : -1.0F;
  if (frame.channels[0] != 0 || frame.channels[7] != 255 ||
      !reading.valid || fabsf(reading.error - expected_error) > 0.0001F ||
      fabsf(reading.strength - 0.125F) > 0.0001F) {
    fprintf(stderr, "Unexpected line reading: valid=%d error=%f strength=%f\n",
            reading.valid, reading.error, reading.strength);
    return 1;
  }

  const uint8_t all_black[car_config::LINE_SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t one_not_black[car_config::LINE_SENSOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 255};
  if (!LineSensors::allChannelsOnLine(all_black) ||
      LineSensors::allChannelsOnLine(one_not_black)) {
    fprintf(stderr, "Eight-channel black-line detection failed\n");
    return 1;
  }

  float left_command = 0.40F;
  float right_command = -0.20F;
  MotorDriver::limitCommandDifference(left_command, right_command);
  if (fabsf(left_command - right_command) >
      car_config::MAX_MOTOR_COMMAND_DIFFERENCE + 0.0001F) {
    fprintf(stderr, "Motor command difference limit failed: left=%f right=%f\n",
            left_command, right_command);
    return 1;
  }

  puts("Eight-channel I2C line sensor mapping test passed");
  return 0;
}
