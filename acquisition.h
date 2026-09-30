#pragma once
#include "scan_snapshot.h"
#include "telemetry.h"

enum class ScanStep : uint8_t { Flow, SelectPressure, ReadPressure, Advance };
struct ScanState {
  bool active = false;
  uint8_t channel = 0, done = 0, rotation = 0;
  ScanStep step = ScanStep::Flow;
  ScanSnapshot sample{};
};
static ScanState scanner;
static void closeScan();

// Each service pass performs at most one bus transaction.
static void serviceSensors() {
  if (!scanner.active) return;
  if (scanner.step == ScanStep::Flow) {
    sfm3000Service(micros());
    scanner.sample.flow = flow_slm;
    scanner.sample.flow_ok = flow_sensor_ok;
    scanner.sample.flow_sample_us = flow_sample_us;
    scanner.step = ScanStep::SelectPressure;
    return;
  }
  SdpSensor &s = psens[scanner.channel];
  if (scanner.step == ScanStep::SelectPressure) {
    if (s.recovery_state != SDP_RECOVERY_IDLE && !timeReachedUs(micros(), s.recovery_due_us)) {
      scanner.step = ScanStep::Advance;
    } else if (!muxSelect(scanner.channel)) {
      if (++s.consecutive_fails >= CONSECUTIVE_FAIL_REINIT_THRESHOLD) {
        s.consecutive_fails = 0;
        s.recovery_state = SDP_RECOVERY_RETRY_WAIT;
        s.recovery_due_us = micros() + SDP_RECOVERY_RETRY_US;
      }
      scanner.step = ScanStep::Advance;
    } else
      scanner.step = ScanStep::ReadPressure;
    return;
  }
  if (scanner.step == ScanStep::ReadPressure) {
    if (s.recovery_state == SDP_RECOVERY_RETRY_WAIT) {
      sdpStop();
      ++s.reinit_count;
      s.recovery_state = SDP_RECOVERY_WAIT_START;
      s.recovery_due_us = micros() + 1000;
    } else if (s.recovery_state == SDP_RECOVERY_WAIT_START) {
      const bool started = sdpStart();
      s.recovery_state = started ? SDP_RECOVERY_WAIT_READY : SDP_RECOVERY_RETRY_WAIT;
      s.recovery_due_us = micros() + (started ? 25000 : SDP_RECOVERY_RETRY_US);
    } else {
      s.recovery_state = SDP_RECOVERY_IDLE;
      s.ok = sdpRead(s.pressure_pa, s.temp_c);
      if (s.ok) {
        s.sample_us = micros();
        s.consecutive_fails = 0;
      } else if (++s.consecutive_fails >= CONSECUTIVE_FAIL_REINIT_THRESHOLD) {
        s.consecutive_fails = 0;
        s.recovery_state = SDP_RECOVERY_RETRY_WAIT;
        s.recovery_due_us = micros();
      }
    }
    scanner.sample.pressure[scanner.channel] = {s.pressure_pa, s.temp_c, s.ok, s.sample_us};
    scanner.step = ScanStep::Advance;
    return;
  }
  ++scanner.done;
  scanner.channel = (scanner.channel + 1) % NUM_PSENS;
  scanner.step = ScanStep::SelectPressure;
  if (scanner.done == NUM_PSENS) closeScan();
}
static void closeScan() {
  if (!scanner.active) return;
  scanner.active = false;
  ScanSnapshot &sample = scanner.sample;
  sample.row_us = micros();
  sample.row_ms = millis();
  if (sample.preset >= 0) {
    int closure_phase;
    float closure_peak;
    const float total = phi_us + (float)(uint32_t)(sample.row_us - phase_update_us);
    const float period = presets[sample.preset].period_us;
    commandedFlowSlm(fmodf(total, period), closure_phase, closure_peak);
    if (closure_phase != sample.phase || total >= period) {
      sample.flow_ok = flow_sensor_ok = false;
      for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
        sample.pressure[ch].ok = false;
        psens[ch].ok = false;
      }
    }
  }
  sample.sequence = seq++;
  sample.missed_slots = missed_slots_total;
  sample.dropped = tx_dropped_total;
  sample.clamped = actuator_clamped;
  publishScan(sample);
}

static void startScan(uint32_t now) {
  scanner.active = true;
  scanner.done = 0;
  scanner.step = ScanStep::Flow;
  scanner.channel = scanner.rotation;
  scanner.rotation = (scanner.rotation + 1) % NUM_PSENS;
  ScanSnapshot &sample = scanner.sample;
  sample.id = ++scan_id;
  sample.start_us = now;
  sample.flow = flow_slm;
  sample.flow_ok = flow_sensor_ok = false;
  sample.flow_sample_us = flow_sample_us;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    psens[ch].ok = false;
    sample.pressure[ch] = {psens[ch].pressure_pa, psens[ch].temp_c, false, psens[ch].sample_us};
  }
  sample.cycle = cycle_id;
  sample.preset = current_preset;
  sample.dac = last_dac_out;
  sample.feedback = last_feedback_dac;
  sample.target = commandedFlowSlm(phi_us, sample.phase, sample.peak);
  strncpy(sample.run_id, run_id, sizeof(sample.run_id) - 1);
  sample.run_id[sizeof(sample.run_id) - 1] = '\0';
}
