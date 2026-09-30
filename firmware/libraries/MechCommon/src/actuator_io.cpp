#include "actuator_io.h"
int applyDacCorrection(int desired) {
  return constrain((int)(((float)desired-DAC_CORR_OFFSET)*4095.0f/DAC_CORR_GAIN+0.5f), 0, 4095);
}
void ActuatorIO::begin() {
  analogWriteResolution(12);
  analogReadResolution(12);
  centre();
}
int ActuatorIO::write(int value) {
  dac = constrain(value, DAC_REG_MIN, DAC_REG_MAX);
  clamped = clamped || dac != value;
  analogWrite(A0, dac);
  return dac;
}
void ActuatorIO::reset() {
  count = 0;
  history[0] = history[1] = history[2] = 0;
  clamped = false;
}
void ActuatorIO::sample() {
  raw = analogRead(A1);
  history[0] = history[1]; history[1] = history[2]; history[2] = raw;
  if (count < 3) ++count;
  int filtered = raw;
  if (count == 3) {
    const int a = history[0], b = history[1], c = history[2];
    filtered = ((a <= b && b <= c) || (c <= b && b <= a)) ? b :
               (((b <= a && a <= c) || (c <= a && a <= b)) ? a : c);
  }
  feedback = applyDacCorrection(DAC_CENTRE) + (filtered-POT_RAW_AT_CENTRE)*POT_DAC_COUNTS_PER_ADC;
}
