#pragma once
#include <Arduino.h>
#include "calibration_values.h"
int applyDacCorrection(int desired);
class ActuatorIO {
 public:
  void begin();
  int write(int value);
  void sample();
  void reset();
  void centre() { write(applyDacCorrection(DAC_CENTRE)); }
  int dac = 0, raw = 0;
  float feedback = 0;
  bool clamped = false;
 private:
  int history[3] = {0, 0, 0};
  int count = 0;
};
