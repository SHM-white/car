#include <math.h>
#include <stdio.h>

#include "Hardware.h"

int main() {
  LineSensors sensors;
  if (!sensors.begin()) {
    fprintf(stderr, "I2C line sensor initialization failed\n");
    return 1;
  }

  const LineReading reading = sensors.read();
  if (!reading.valid || fabsf(reading.error - 1.0F) > 0.0001F ||
      fabsf(reading.strength - 0.125F) > 0.0001F) {
    fprintf(stderr, "Unexpected line reading: valid=%d error=%f strength=%f\n",
            reading.valid, reading.error, reading.strength);
    return 1;
  }

  puts("Eight-channel I2C line sensor mapping test passed");
  return 0;
}
