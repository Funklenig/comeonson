#pragma once
#include "scan_snapshot.h"
// Formatting has no sensor or scanner side effects. Queue accounting remains
// with the transport, and the sample identity is captured before publication.
static bool appendField(size_t &used, const char *format, ...) {
  if (used >= TX_SIZE) return false;
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(format_buffer + used, TX_SIZE - used, format, args);
  va_end(args);
  if (n < 0 || (size_t)n >= TX_SIZE - used) return false;
  used += n;
  return true;
}
static void publishScan(const ScanSnapshot &sample) {
  size_t used = 0;
  bool fits = appendField(used, "%s,%lu,%lu,%.3f,%d,%ld,", sample.run_id[0] ? "DATA2" : "DATA",
                          (unsigned long)sample.sequence, (unsigned long)sample.row_ms, sample.flow, sample.flow_ok,
                          (long)sampleAgeUs(sample.row_us, sample.flow_ok, sample.flow_sample_us));
  int ok_count = 0;
  float temperature_sum = 0;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    fits = fits && appendField(used, "%.3f,", sample.pressure[ch].pressure_pa);
    ok_count += sample.pressure[ch].ok;
    if (sample.pressure[ch].ok) temperature_sum += sample.pressure[ch].temp_c;
  }
  fits = fits && appendField(used, "%.2f,", (ok_count ? temperature_sum / ok_count : 0.0f));
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) fits = fits && appendField(used, "%d,", sample.pressure[ch].ok);
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch)
    fits = fits &&
           appendField(used, "%ld,", (long)sampleAgeUs(sample.row_us, sample.pressure[ch].ok, sample.pressure[ch].sample_us));
  fits = fits && appendField(used, "%d,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d", sample.dac, sample.feedback,
                             100.0f * (sample.dac - corrected_centre_dac) / DAC_HALF_RANGE,
                             100.0f * (sample.feedback - corrected_centre_dac) / DAC_HALF_RANGE,
                             sample.target, sample.peak, sample.phase, sample.preset, sample.clamped);
  if (sample.run_id[0])
    fits = fits && appendField(used, ",%s,%lu,%lu,%lu,%lu,%lu,%lu", sample.run_id, (unsigned long)sample.id,
                               (unsigned long)sample.row_us, (unsigned long)sample.cycle,
                               (unsigned long)sample.start_us, (unsigned long)sample.missed_slots,
                               (unsigned long)sample.dropped);
  fits = fits && appendField(used, "\n");
  if (fits) {
    if (enqueueText(format_buffer, false, true)) final_enqueued_seq = sample.sequence;
  } else {
    ++tx_dropped_total;
  }
}
