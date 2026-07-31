#include <stdio.h>

#include "LineFollower.h"

int main() {
  FinishLineStop stop;
  const float arm = car_config::FINISH_LINE_ARM_DISTANCE_M;

  if (stop.update(arm - 0.01F, true, 100)) return 1;
  if (stop.update(arm, true, 200)) return 2;
  if (stop.update(arm + 0.001F, false, 220)) return 3;
  if (stop.update(arm + 0.010F, true, 300)) return 4;
  if (stop.update(arm + 0.012F, true, 329)) return 5;
  if (stop.update(arm + 0.013F, true, 330)) return 6;
  if (!stop.markerDetected()) return 7;
  if (stop.update(arm + 0.109F, false, 400)) return 8;
  if (!stop.update(arm + 0.111F, false, 420)) return 9;
  if (!stop.update(arm + 0.111F, false, 440)) return 10;

  puts("Finish-line delayed-stop test passed");
  return 0;
}
