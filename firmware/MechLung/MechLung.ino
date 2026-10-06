#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <actuator_io.h>
#include <acquisition.h>
#include <transport.h>

static ActuatorIO actuator;
static SensorState sensors;
static Acquisition acquisition(sensors);
static Transport transport(false);
static int corrected_centre_dac;
static uint32_t slot_us = 0, phase_update_us = 0, scan_id = 0, cycle_id = 0, missed_slots_total = 0;
static uint32_t last_monitor_ms = 0;
struct Preset {
  float q_peak_Lmin;
  int   a_dac_corr;
  float t_exp_us;
  float t_insp_us;
  float period_us;
};

static Preset presets[4];

static float computeKarea() {
  const float r = CYLINDER_DIAMETER_MM / 2.0f;
  return (PI * r * r) / 1000.0f;   // mL of swept volume per mm of stroke
}

static int mmToDacCounts(float mm) {
  // VERIFY: This conversion assumes DAC_HALF_RANGE is on the uncorrected command scale, while cmd_pct/fdbk_pct use it on corrected DAC/A1 scales; a mismatch can shift tidal volume and peak flow by about 2.4%. Command a known DAC delta, measure piston travel with calipers, and determine which scale 1526 represents before changing the scale.
  return (int)(mm * ((float)DAC_HALF_RANGE / (ACTUATOR_STROKE_MM / 2.0f)) + 0.5f);
}

static void initPresets() {
  const float k_area      = computeKarea();
  const float gain_factor = 4095.0f / DAC_CORR_GAIN;

  const float a_mm  = V_TIDAL_ML / k_area;
  const int   a_dac = mmToDacCounts(a_mm);

  static const float q_Lmin[4] = { 10.0f, 15.0f, 20.0f, 30.0f };

  for (int i = 0; i < 4; i++) {
    const float Q_mls = q_Lmin[i] * 1000.0f / 60.0f;

    const float t_insp_s = (V_TIDAL_ML * PI) / (2.0f * Q_mls);
    const float t_exp_s  = t_insp_s * (IE_EXP_PARTS / IE_INSP_PARTS);
    const float period_s = t_exp_s + t_insp_s;

    presets[i].q_peak_Lmin = q_Lmin[i];
    presets[i].a_dac_corr  = (int)((float)a_dac * gain_factor + 0.5f);
    presets[i].t_exp_us    = t_exp_s  * 1e6f;
    presets[i].t_insp_us   = t_insp_s * 1e6f;
    presets[i].period_us   = period_s * 1e6f;
  }
}

// =============================================================================
//  BOOT-TIME GEOMETRY SUMMARY — informational only, not a safety check
// =============================================================================
//  This sketch has no visibility into what travel the Jrk G2 is currently
//  configured to permit — that configuration lives in the Jrk itself (Jrk
//  Configuration Utility, Input scaling) and firmware does not read it back.
//  Metadata states computed geometry, not proof of actual travel or volume.
// =============================================================================

// =============================================================================
//  ACTUATOR TIMING — 100Hz, matching the Jrk G2's default 10ms PID period.
//  Shares the 100 Hz acquisition-slot deadline.
// =============================================================================



static int      current_preset = -1;  // -1 idle, -2 demo sine, 0-3 presets
// VERIFY: phi_us remains float because changing it alone would still narrow through float preset periods and fmodf(); confirm its float-ULP resolution near preset 1's longest period is acceptable on the rig before making a coordinated double-precision change.
static float    phi_us         = 0.0f;




static int stepDac(float phi) {
  const Preset &p = presets[current_preset];
  float pos_norm;

  if (phi < p.t_exp_us) {
    pos_norm = 0.5f * (1.0f - cosf(PI * phi / p.t_exp_us));
  } else {
    pos_norm = 0.5f * (1.0f + cosf(PI * (phi - p.t_exp_us) / p.t_insp_us));
  }

  const float profile_dac = corrected_centre_dac + (float)p.a_dac_corr * pos_norm;
  return actuator.write((int)(profile_dac + 0.5f));
}

// The profile's volume derivative, expressed as geometric local L/min. Phase 0 is
// the longer expiration stroke; phase 1 is the faster inspiration stroke.
// The 2:1 timing ratio means expiration intentionally peaks at half the
// preset's named peak while inspiration reaches the named peak.
static float commandedFlowSlm(float phi, int &phase_out,
                              float &phase_peak_slm_out) {
  if (current_preset < 0) {
    phase_out = -1;
    phase_peak_slm_out = 0.0f;
    return 0.0f;
  }

  const Preset &p = presets[current_preset];
  if (phi < p.t_exp_us) {
    phase_out = 0;
    phase_peak_slm_out = p.q_peak_Lmin * (p.t_insp_us / p.t_exp_us);
    return phase_peak_slm_out * sinf(PI * phi / p.t_exp_us);
  }

  phase_out = 1;
  phase_peak_slm_out = p.q_peak_Lmin;
  return -phase_peak_slm_out *
         sinf(PI * (phi - p.t_exp_us) / p.t_insp_us);
}

// =============================================================================
//  DEMO SINE — full-range motion demo, not physiologically meaningful.
// Sweeps DAC register limits minus the margin. Actual travel depends on
// external Jrk input scaling; firmware does not infer a mechanical limit.

static const float DEMO_SINE_PERIOD_US = 3.0e6f;  // 3s/cycle
static const int   DEMO_SINE_MARGIN_DAC = 100;     // keep a small margin off the register rails

static int demo_sine_dac_min = 0;
static int demo_sine_dac_max = 4095;

static int stepDemoSine(float phi) {
  const float half_range = (float)(demo_sine_dac_max - demo_sine_dac_min) / 2.0f;
  const float centre     = (float)(demo_sine_dac_max + demo_sine_dac_min) / 2.0f;
  const float pos_norm   = sinf(2.0f * PI * phi / DEMO_SINE_PERIOD_US);

  const float dac_out_unclamped = centre + half_range * pos_norm;
  return actuator.write((int)(dac_out_unclamped + 0.5f));
}


static void stopSession(const char *reason) {
  actuator.centre();
  current_preset = -1;
  acquisition.cancel();
  transport.endSession(reason);
}
static void startMotion(int preset) {
  current_preset = preset;
  phi_us = 0;
  scan_id = cycle_id = missed_slots_total = 0;
  acquisition.cancel();
  actuator.reset();
  actuator.sample();
  transport.resetCounters();
  transport.armMotion();
  slot_us = phase_update_us = micros();
}
static void closure(uint32_t now, int &phase, uint32_t &cycle) {
  cycle = cycle_id;
  phase = -1;
  if (current_preset >= 0) {
    const float total = phi_us + (float)(uint32_t)(now-phase_update_us);
    const float period = presets[current_preset].period_us;
    float peak;
    commandedFlowSlm(fmodf(total, period), phase, peak);
    cycle += (uint32_t)(total/period);
  }
}
static void publish(ScanSnapshot &sample) {
  sample.missed_slots = missed_slots_total;
  transport.publish(sample);
}
static void dispatch(char key) {
  if (key == 'B') startMotion(transport.preset);
  else if (key == 'E') stopSession("host_stop");
  else if (key == 'H' || key == 'h' || key == '0') stopSession("manual_override");
  else if ((key >= '1' && key <= '4') || key == 's' || key == 'S') {
    if (transport.active()) stopSession("manual_override");
    startMotion(key == 's' || key == 'S' ? -2 : key-'1');
  } else if (key == 'X' || key == 'x' || key == 'm' || key == 'n') {
    transport.enqueueText("# ERROR,3,calibration_firmware_required\n");
  }
}
void setup() {
  actuator.begin(); // hold centre before USB and I2C initialization
  corrected_centre_dac = applyDacCorrection(DAC_CENTRE);
  initPresets();
  demo_sine_dac_min = DAC_REG_MIN + DEMO_SINE_MARGIN_DAC;
  demo_sine_dac_max = DAC_REG_MAX - DEMO_SINE_MARGIN_DAC;
  Serial.begin(460800);
  delay(2000);
  acquisition.begin();
  slot_us = phase_update_us = micros();
  transport.enqueueText("# BOOT,3\n");
}
void loop() {
  if (transport.expired()) stopSession("watchdog");
  uint32_t now = micros();
  ScanSnapshot completed{};
  int closure_phase;
  uint32_t closure_cycle;
  closure(now, closure_phase, closure_cycle);
  const uint32_t elapsed = now-slot_us;
  if (elapsed >= LOG_INTERVAL_US) {
    const uint32_t slots = elapsed/LOG_INTERVAL_US;
    slot_us += slots*LOG_INTERVAL_US;
    missed_slots_total += slots-1;
    if (acquisition.closeScan(completed, closure_phase, closure_cycle)) publish(completed);
    if (current_preset != -1) {
      const float period = current_preset == -2 ? DEMO_SINE_PERIOD_US : presets[current_preset].period_us;
      const float total = phi_us + (float)(uint32_t)(now-phase_update_us);
      cycle_id += (uint32_t)(total/period);
      phi_us = fmodf(total, period);
      if (current_preset == -2) stepDemoSine(phi_us);
      else stepDac(phi_us);
    }
    phase_update_us = now;
    actuator.sample();
    ScanSnapshot motion{};
    motion.id = ++scan_id; motion.start_us = now; motion.cycle = cycle_id;
    motion.preset = current_preset;
    motion.dac = actuator.dac; motion.feedback = actuator.feedback;
    motion.pot_raw_adc = actuator.raw; motion.clamped = actuator.clamped;
    motion.target = commandedFlowSlm(phi_us, motion.phase, motion.peak);
    strcpy(motion.run_id, transport.run_id);
    acquisition.startScan(motion); // acquisition continues in idle hold
  }
  dispatch(transport.poll());
  transport.serviceTx();
  transport.serviceMeta();
  closure(micros(), closure_phase, closure_cycle);
  if (acquisition.serviceSensors(completed, closure_phase, closure_cycle)) publish(completed);
  if (transport.monitor_enabled && (uint32_t)(millis()-last_monitor_ms) >= 300 && transport.monitorReady()) {
    last_monitor_ms = millis();
    int ok = 0; uint32_t reinits = 0;
    for (const auto &sensor : sensors.psens) { ok += sensor.ok; reinits += sensor.reinit_count; }
    transport.enqueueFormat(false, "# MONITOR,preset=%d,flow_ok=%d,pressure_ok=%d,flow_reinits=%lu,pressure_reinits=%lu,clamped=%d\n",
      current_preset, sensors.flow_sensor_ok, ok, (unsigned long)sensors.sfm_reinit_count,
      (unsigned long)reinits, actuator.clamped);
  }
}
