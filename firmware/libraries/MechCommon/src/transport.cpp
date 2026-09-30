#include "transport.h"
#include "telemetry.h"
#include "actuator_io.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
bool Transport::enqueueText(const char *text, bool control, bool telemetry) {
  const size_t length = strlen(text);
  if (length >= TX_SIZE || tx_count >= (control ? TX_COUNT : TX_COUNT - 2)) {
    if (telemetry) ++tx_dropped_total;
    return false;
  }
  TxRecord &r = tx[(tx_head + tx_count) % TX_COUNT];
  memcpy(r.bytes, text, length);
  r.size = length;
  r.offset = 0;
  ++tx_count;
  return true;
}
bool Transport::enqueueFormat(bool control, const char *format, ...) {
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(format_buffer, TX_SIZE, format, args);
  va_end(args);
  if (n < 0 || n >= (int)TX_SIZE) return false;
  return enqueueText(format_buffer, control);
}
void Transport::serviceTx() {
  if (!tx_count) return;
  const int capacity = Serial.availableForWrite();
  if (capacity <= 0) return;
  TxRecord &r = tx[tx_head];
  size_t n = r.size - r.offset;
  if (n > 64) n = 64;
  if (n > (size_t)capacity) n = capacity;
  // Renesas 1.6.0: one writer; FIFO consumers can only free capacity.
  // No flush, retry, or Serial boolean conversion (which calls tud_task).
  const size_t sent = Serial.write((const uint8_t *)r.bytes + r.offset, n);
  r.offset += sent;
  if (r.offset == r.size) {
    tx_head = (tx_head + 1) % TX_COUNT;
    --tx_count;
  }
}

void Transport::resetCounters() { seq = tx_dropped_total = 0; final_enqueued_seq = -1; }
void Transport::publish(ScanSnapshot sample) {
  sample.sequence = seq++;
  sample.dropped = tx_dropped_total;
  const TelemetryConfig config{applyDacCorrection(DAC_CENTRE), DAC_HALF_RANGE};
  const int length = encodeTelemetry(sample, config, format_buffer, TX_SIZE);
  if (length < 0) ++tx_dropped_total;
  else if (enqueueText(format_buffer, false, true)) final_enqueued_seq = sample.sequence;
}
void Transport::armMotion() {
  host_watchdog_armed = active() || (host_seen && (uint32_t)(millis()-last_host_heartbeat_ms) < HOST_HEARTBEAT_TIMEOUT_MS);
}
bool Transport::expired() const {
  return host_watchdog_armed && (uint32_t)(millis()-last_host_heartbeat_ms) >= HOST_HEARTBEAT_TIMEOUT_MS;
}
bool Transport::completedId(const char *id) const {
  for (uint8_t i = 0; i < completed_count; ++i)
    if (!strcmp(id, completed_ids[i])) return true;
  return false;
}
void Transport::endSession(const char *reason) {
  // Hold first; unclosed acquisition is deliberately abandoned. ENDED is a
  // barrier behind all retained DATA3 frames, including a partially sent one.
  host_watchdog_armed = false;
  if (!run_id[0]) return;
  strcpy(ended_id, run_id);
  if (completed_count < COMPLETED_CAPACITY) strcpy(completed_ids[completed_count++], run_id);
  snprintf(ended_marker, sizeof ended_marker, "# ENDED,3,%s,%lld,%s,%lu\n", run_id,
           (long long)final_enqueued_seq, reason, (unsigned long)tx_dropped_total);
  enqueueText(ended_marker, true);
  run_id[0] = 0;
}
bool Transport::validId(const char *id) {
  if (strlen(id) != 16) return false;
  for (uint8_t i = 0; i < 16; ++i)
    if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) return false;
  return true;
}
char Transport::handleFrame(char *line) {
  char *first = strchr(line, ',');
  if (!first) return 0;
  *first++ = 0;
  char *mode = strchr(first, ',');
  if (mode) *mode++ = 0;
  if (!validId(first)) return 0;
  if (!strcmp(line, "@PING") && !mode) {
    if (run_id[0] && !strcmp(run_id, first)) last_host_heartbeat_ms = millis();
  } else if (!strcmp(line, "@STOP") && !mode) {
    if (run_id[0] && !strcmp(run_id, first))
      return 'E';
    else if (!strcmp(ended_id, first) && tx_count < TX_COUNT - 1)
      enqueueText(ended_marker, true);
  } else if (!strcmp(line, "@START") && mode) {
    const int selected = calibration ? (!strcmp(mode, "CAL") ? -3 : -1) : !strcmp(mode, "DEMO") ? -2
                                             : (strlen(mode) == 2 && mode[0] == 'P' &&
                                                        mode[1] >= '0' && mode[1] <= '3'
                                                    ? mode[1] - '0'
                                                    : -1);
    if (selected == -1 || completedId(first)) return 0;
    if (run_id[0]) {
      if (!strcmp(run_id, first) && !strcmp(run_mode, mode) && tx_count < TX_COUNT - 1)
        enqueueText(started_marker, true);
      return 0;
    }
    if (completed_count == COMPLETED_CAPACITY) {
      enqueueText("# ERROR,3,completed_id_capacity_reset_required\n");
      return 0;
    }
    // Reserve STARTED plus a future ENDED before any motion.
    if (tx_count > TX_COUNT - 2) return 0;
    snprintf(started_marker, sizeof started_marker, "# STARTED,3,%s,%s,%lu,%lu\n", first, mode,
             (unsigned long)millis(), (unsigned long)micros());
    if (!enqueueText(started_marker, true)) return 0;
    strcpy(run_id, first);
    strcpy(run_mode, mode);
    host_seen = true;
    last_host_heartbeat_ms = millis();
    preset = selected;
    resetCounters();
    armMotion();
    return 'B';
  }
  return 0;
}
#ifndef MECH_BUILD_ID
#define MECH_BUILD_ID "unknown"
#endif
#ifndef MECH_CORE_ID
#define MECH_CORE_ID "unknown"
#endif
// Values come from the same compiled constants as the waveform. The compile
// script provides a source digest; it is provenance, not a binary signature.
bool Transport::serviceMeta() {
  if (meta_step < 0 || tx_count >= TX_COUNT - 2) return false;
  bool ok = false;
  switch (meta_step) {
    case 0:
      ok = enqueueText("# PROTOCOL,3\n");
      break;
    case 1:
      ok = enqueueFormat(false, "# META_BEGIN,3,%s\n", MECH_BUILD_ID);
      break;
    case 2:
      ok = enqueueFormat(false, "# META,3,core,%s\n", MECH_CORE_ID);
      break;
    case 3:
      ok = enqueueFormat(false, "# META,3,tidal_ml,%.8g\n", V_TIDAL_ML);
      break;
    case 4:
      ok = enqueueFormat(false, "# META,3,bore_mm,%.8g\n", CYLINDER_DIAMETER_MM);
      break;
    case 5:
      ok = enqueueFormat(false, "# META,3,stroke_mm,%.8g\n", ACTUATOR_STROKE_MM);
      break;
    case 6:
      ok = enqueueFormat(false, "# META,3,dac_centre,%d\n", DAC_CENTRE);
      break;
    case 7:
      ok = enqueueFormat(false, "# META,3,dac_half_range,%d\n", DAC_HALF_RANGE);
      break;
    case 8:
      ok = enqueueFormat(false, "# META,3,dac_corr_offset,%.8g\n", DAC_CORR_OFFSET);
      break;
    case 9:
      ok = enqueueFormat(false, "# META,3,dac_corr_gain,%.8g\n", DAC_CORR_GAIN);
      break;
    case 10:
      ok = enqueueFormat(false, "# META,3,pot_raw_centre,%.8g\n", POT_RAW_AT_CENTRE);
      break;
    case 11:
      ok = enqueueFormat(false, "# META,3,pot_dac_per_adc,%.8g\n", POT_DAC_COUNTS_PER_ADC);
      break;
    case 12:
      ok = enqueueFormat(false, "# META,3,firmware_role,%s\n", calibration ? "calibration" : "production");
      break;
    case 13:
      ok = enqueueFormat(false, "# META,3,slot_us,%lu\n", (unsigned long)LOG_INTERVAL_US);
      break;
    case 14:
      ok = enqueueFormat(false, "# META,3,sfm_scale,%.8g\n", SFM3000_SCALE_FLOW_AIR);
      break;
    case 15:
      ok = enqueueText("# META,3,flow_reference,local_target_standard_293.15K_1013mbar_airN2\n");
      break;
    case 16:
      ok = enqueueFormat(false, "# META,3,cal_centre_dac,%d\n", applyDacCorrection(DAC_CENTRE));
      break;
    case 17:
      ok = enqueueFormat(false, "# META,3,cal_reference_dac,%d\n", applyDacCorrection(CALIBRATION_TARGET_JRK_SCALE));
      break;
    case 18:
      ok = CAL_MIN_DAC < 0 ? enqueueText("# META,3,cal_min_dac,unset\n") :
           enqueueFormat(false, "# META,3,cal_min_dac,%d\n", CAL_MIN_DAC);
      break;
    case 19:
      ok = CAL_MAX_DAC < 0 ? enqueueText("# META,3,cal_max_dac,unset\n") :
           enqueueFormat(false, "# META,3,cal_max_dac,%d\n", CAL_MAX_DAC);
      break;
    case 20:
      ok = enqueueFormat(false, "# META_END,3,%s\n", MECH_BUILD_ID);
      break;
    default:
      ok = enqueueText("# READY_QUERY\n");
      if (ok) meta_step = -2;
  }
  if (ok) ++meta_step;
  return ok;
}
char Transport::poll() {
  if (rx_in_frame && (uint32_t)(millis() - rx_start_ms) >= 250) rx_discard = true;
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (rx_in_frame) {
      if (c == '\n') {
        if (!rx_discard) {
          rx_frame[rx_length] = 0;
          const char action = handleFrame(rx_frame);
          rx_in_frame = rx_discard = false;
          rx_length = 0;
          if (action) return action;
        }
        rx_in_frame = rx_discard = false;
        rx_length = 0;
      } else if (!rx_discard) {
        if (rx_length >= sizeof(rx_frame) - 1 || c < 32 || c > 126)
          rx_discard = true;
        else
          rx_frame[rx_length++] = c;
      }
      continue;
    }
    if (c == '@') {
      rx_in_frame = true;
      rx_length = 0;
      rx_frame[rx_length++] = c;
      rx_start_ms = millis();
      continue;
    }
    if (c == '?') {
      if (!run_id[0]) meta_step = 0;
      continue;
    }
    if (c == '!') {
      if (!run_id[0]) {
        host_seen = true;
        last_host_heartbeat_ms = millis();
        host_watchdog_armed = true;
      }
      continue;
    }
    if (c == 't' || c == 'T') {
      monitor_enabled = !monitor_enabled;
      continue;
    }
    // B/E are internal actions and never accepted as operator commands.
    if (c != 'B' && c != 'E' && c != '\n' && c != '\r') return c;
  }
  return 0;
}
