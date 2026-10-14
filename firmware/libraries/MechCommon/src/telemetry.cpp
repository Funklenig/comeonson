#include "telemetry.h"
#include <stdio.h>
#include <stdarg.h>
static int32_t sampleAgeUs(uint32_t now, bool valid, uint32_t stamp) {
  return valid ? (int32_t)(now-stamp) : -1;
}
static bool appendField(char *buffer, size_t capacity, size_t &used, const char *format, ...) {
  if (used >= capacity) return false;
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(buffer + used, capacity - used, format, args);
  va_end(args);
  if (n < 0 || (size_t)n >= capacity - used) return false;
  used += n;
  return true;
}
int encodeTelemetry(const ScanSnapshot &sample, const TelemetryConfig &config, char *buffer, size_t capacity) {
  size_t used = 0;
  bool fits = appendField(buffer, capacity, used, "%s,%lu,%lu,%.3f,%d,%ld,", "DATA3",
                          (unsigned long)sample.sequence, (unsigned long)sample.row_ms, sample.flow, sample.flow_ok,
                          (long)sampleAgeUs(sample.row_us, sample.flow_ok, sample.flow_sample_us));
  int ok_count = 0;
  float temperature_sum = 0;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    fits = fits && appendField(buffer, capacity, used, "%.3f,", sample.pressure[ch].pressure_pa);
    ok_count += sample.pressure[ch].ok;
    if (sample.pressure[ch].ok) temperature_sum += sample.pressure[ch].temp_c;
  }
  fits = fits && appendField(buffer, capacity, used, "%.2f,", (ok_count ? temperature_sum / ok_count : 0.0f));
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) fits = fits && appendField(buffer, capacity, used, "%d,", sample.pressure[ch].ok);
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch)
    fits = fits &&
           appendField(buffer, capacity, used, "%ld,", (long)sampleAgeUs(sample.row_us, sample.pressure[ch].ok, sample.pressure[ch].sample_us));
  fits = fits && appendField(buffer, capacity, used, "%d,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d", sample.dac, sample.feedback,
                             100.0f * (sample.dac - config.centre_dac) / config.half_range,
                             100.0f * (sample.feedback - config.centre_dac) / config.half_range,
                             sample.target, sample.peak, sample.phase, sample.preset, sample.clamped);
    fits = fits && appendField(buffer, capacity, used, ",%s,%lu,%lu,%lu,%lu,%lu,%lu", sample.run_id[0] ? sample.run_id : "-", (unsigned long)sample.id,
                               (unsigned long)sample.row_us, (unsigned long)sample.cycle,
                               (unsigned long)sample.start_us, (unsigned long)sample.missed_slots,
                               (unsigned long)sample.dropped);
  fits = fits && appendField(buffer, capacity, used, ",%d,%s\n", sample.pot_raw_adc, sample.calibration_position);
  return fits ? (int)used : -1;
}
