#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <initializer_list>
// Protocol 2: bounded transport and one I2C transaction per loop pass.
#include <Wire.h>

// VERIFY: Confirm pins 27/26 against the installed Renesas core before relying on these fallbacks; if that core exposes the pins as non-macro identifiers, these macros can silently shadow its real values.
#ifndef WIRE1_SDA_PIN
#define WIRE1_SDA_PIN 27
#endif
#ifndef WIRE1_SCL_PIN
#define WIRE1_SCL_PIN 26
#endif

// =============================================================================
//  ACTUATOR CALIBRATION
// =============================================================================
// Travel control belongs to the configured Jrk. This sketch only limits the
// hardware DAC register; it neither reads nor verifies Jrk travel settings.
// DAC_CENTRE is inherited calibration, not a measured safety boundary.
// VERIFY: DAC_CENTRE is inherited and unverified on this rig; re-derive it with the H/X hold pair before treating feedback_dac, fdbk_pct, or following error as quantitative.
static const int   DAC_CENTRE         = 1450;
static const float ACTUATOR_STROKE_MM = 300.0f;

// DAC_HALF_RANGE is the inherited counts-per-half-stroke scale. Telemetry
// percentages are 100 * corrected DAC offset / this value; they are not
// independently measured percentages of full mechanical stroke.
// VERIFY: DAC_HALF_RANGE is used for both mm-to-command conversion and corrected-DAC/A1 percentage normalization, which may be incompatible scales and can shift tidal volume and peak flow by about 2.4%. Command a known DAC delta, measure piston travel with calipers, and determine which scale 1526 represents before correcting either use.
static const int   DAC_HALF_RANGE     = 1526;

static const float CYLINDER_DIAMETER_MM = 115.0f;
static const float V_TIDAL_ML           = 700.0f;

static const float IE_EXP_PARTS  = 2.0f;
static const float IE_INSP_PARTS = 1.0f;

static const float DAC_CORR_OFFSET = 49.0f;
static const float DAC_CORR_GAIN   = 4002.0f;

static int corrected_centre_dac = 0;

static int applyDacCorrection(int jrk_desired) {
  float dac = ((float)jrk_desired - DAC_CORR_OFFSET) * 4095.0f / DAC_CORR_GAIN;
  return constrain((int)(dac + 0.5f), 0, 4095);
}

// =============================================================================
//  OUTPUT TIMING
// =============================================================================
// One slot coordinates the DAC tick, scan allocation and generated telemetry.
// Missed slots are skipped and counted; USB may drop whole queued telemetry.
static const uint32_t LOG_RATE_HZ   = 100;
static const uint32_t LOG_INTERVAL_US = 1000000UL / LOG_RATE_HZ;

// Each slot has a unique scan ID. Incomplete scans retain false validity
// for unacquired channels. Per-channel ages expose sequential read timing.
static const uint32_t SENSOR_CYCLE_US = LOG_INTERVAL_US;
static const uint32_t I2C_TRANSACTION_TIMEOUT_US = 5000UL;

// =============================================================================
//  I2C BUS RECOVERY — Wire; Wire1 has a corresponding helper below. The SFM3000 can be left
//  mid-transfer holding SDA low across a sketch re-upload; this clears
//  that before Wire.begin(). Must run before Wire.begin().
// =============================================================================

static void i2cBusRecovery() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);

  for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);

  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}

// =============================================================================
//  I2C BUS RECOVERY — Wire1 (Qwiic/mux bus). Same clock-out-a-stuck-SDA
//  technique as i2cBusRecovery() above, but for WIRE1_SDA_PIN/WIRE1_SCL_PIN
//  (pins 27/26 on the R4 WiFi). Added because the main bus already had a
//  recovery path and Wire1 didn't — if a sensor or the mux is left mid-
//  transaction across a reset, SDA1 can stay held low and Wire1.begin()
//  alone won't clear it (it only configures the peripheral, it doesn't
//  clock anything out). This runs before Wire1.begin(), same as the main
//  bus's recovery does, so it doesn't touch the FSP I2C driver at all. It is
//  therefore independent of the explicit 5ms transaction timeout that setup()
//  applies afterwards: the old Renesas timeout bug referenced in the
//  header was in the driver's runtime timeout handling, not in bit-banging
//  the pins before begin().
// =============================================================================

static void i2cBusRecovery1() {
  pinMode(WIRE1_SDA_PIN, INPUT_PULLUP);
  pinMode(WIRE1_SCL_PIN, OUTPUT);

  for (int i = 0; i < 9 && digitalRead(WIRE1_SDA_PIN) == LOW; i++) {
    digitalWrite(WIRE1_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(WIRE1_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  pinMode(WIRE1_SDA_PIN, OUTPUT);
  digitalWrite(WIRE1_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(WIRE1_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(WIRE1_SDA_PIN, HIGH);
  delayMicroseconds(5);

  pinMode(WIRE1_SDA_PIN, INPUT);
  pinMode(WIRE1_SCL_PIN, INPUT);
}

// =============================================================================
//  SFM3000 FLOW SENSOR — I2C (Wire, main bus)
// =============================================================================
//  Sensirion SFM3000, addr 0x40. flow[slm] = (raw - OFFSET) / SCALE.
//  Protocol: issue "start continuous measurement" (0x1000) once at init,
//  then read the 2-byte flow word and its CRC byte. Per the datasheet,
//  the very first measurement right after chip init is invalid and must be
//  discarded — done once here in sfm3000Init().
//
//  On a failed read, flow_slm holds its last-good value (flow_sensor_ok
//  reports which happened). The logger plots invalid readings as gaps. Past SFM_FAIL_REINIT_THRESHOLD consecutive
//  failures the sensor is re-initialised, mirroring what the SDP811
//  array already did — without it, one transient main-bus glitch left the
//  flow channel dead for the rest of the session.
// =============================================================================

static const uint8_t SFM3000_I2C_ADDR       = 0x40;
static const float   SFM3000_OFFSET_FLOW    = 32000.0f;
static const float   SFM3000_SCALE_FLOW_AIR = 140.0f;   // air/N2; 142.8 for O2

static const int SFM_FAIL_REINIT_THRESHOLD = 10;  // ~100ms at SENSOR_CYCLE_US

static bool  sfm_ok          = false; // did sfm3000Init() succeed at boot
static bool  flow_sensor_ok  = false; // did the most recent read succeed
static float flow_slm        = 0.0f;  // last valid reading; holds last-good on failure
static uint32_t flow_sample_us = 0;   // completion time of latest valid read

static uint16_t sfm_consecutive_fails = 0;
static uint32_t sfm_reinit_count      = 0;  // surfaced in the Monitor block

enum SfmRecoveryState : uint8_t {
  SFM_RECOVERY_RETRY_WAIT,
  SFM_RECOVERY_DISCARD_WAIT,
};

static const uint32_t SFM_RETRY_INTERVAL_US = 1000000UL;
static SfmRecoveryState sfm_recovery_state  = SFM_RECOVERY_RETRY_WAIT;
static uint32_t sfm_recovery_due_us         = 0;

static bool sfm3000Init();
static bool sfm3000ReadRaw(uint16_t &raw_out);
static bool sfm3000ReadFlow();

static bool timeReachedUs(uint32_t now_us, uint32_t due_us) {
  return (int32_t)(now_us - due_us) >= 0;
}

// Start continuous conversion without waiting for the first sample. Runtime
// recovery uses this split phase so the actuator loop keeps running during
// the sensor's mandatory 100ms conversion delay.
static void sfm3000BeginRecovery(uint32_t /*now_us*/) {
  sfm_ok = false;
  flow_sensor_ok = false;

  Wire.beginTransmission(SFM3000_I2C_ADDR);
  Wire.write(0x10);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    sfm_recovery_state = SFM_RECOVERY_DISCARD_WAIT;
    sfm_recovery_due_us = micros() + 100000UL;
  } else {
    sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
  }
}

static void sfm3000Service(uint32_t now_us) {
  if (sfm_ok) {
    sfm3000ReadFlow();
    return;
  }
  if (!timeReachedUs(now_us, sfm_recovery_due_us)) return;

  if (sfm_recovery_state == SFM_RECOVERY_DISCARD_WAIT) {
    uint16_t discard;
    if (sfm3000ReadRaw(discard)) {
      sfm_ok = true;
      sfm_consecutive_fails = 0;
      return;
    }
    sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
    return;
  }

  sfm_reinit_count++;
  sfm3000BeginRecovery(now_us);
}

static bool sfmCheckCrc(uint8_t hi, uint8_t lo, uint8_t expected) {
  // SFM3xxx: polynomial 0x31, initial value 0x00 (SDP uses 0xff).
  uint8_t crc = 0;
  for (uint8_t byte : {hi, lo}) {
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }
  return crc == expected;
}
static bool sfm3000ReadRaw(uint16_t &raw_out) {
  if (Wire.requestFrom((int)SFM3000_I2C_ADDR, 3) != 3) return false;
  if (Wire.available() < 3) return false;
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  if (!sfmCheckCrc(hi, lo, Wire.read())) return false;
  raw_out = ((uint16_t)hi << 8) | lo;
  return true;
}

static bool sfm3000ReadFlow() {
  uint16_t raw;
  flow_sensor_ok = sfm3000ReadRaw(raw);

  if (flow_sensor_ok) {
    flow_slm = ((float)raw - SFM3000_OFFSET_FLOW) / SFM3000_SCALE_FLOW_AIR;
    flow_sample_us = micros();
    sfm_consecutive_fails = 0;
    return true;
  }

  if (++sfm_consecutive_fails >= SFM_FAIL_REINIT_THRESHOLD) {
    sfm_consecutive_fails = 0;
    sfm_ok = false;
    sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sfm_recovery_due_us = micros();
  }
  return false;
}

static bool sfm3000Init() {
  Wire.beginTransmission(SFM3000_I2C_ADDR);
  Wire.write(0x10);
  Wire.write(0x00);
  const uint8_t i2c_status = Wire.endTransmission();
  if (i2c_status != 0) return false;

  delay(100);  // let the first (to-be-discarded) conversion complete

  uint16_t discard;
  return sfm3000ReadRaw(discard);
}

// =============================================================================
//  SDP811 × 7  (Wire1, Qwiic bus, via PCA9548 mux) — inherited from
//  seven channels with incremental runtime recovery and per-channel ages.
//
//  Mux channel N -> "Psens(N+1)". Channel 7 unused. Reliability: scale
//  factor read live (never hardcoded); muxSelect() checked before every
//  read attempt; all 7 channels attempted every cycle regardless of any
//  other channel's failure; consecutive-failure tracking triggers a real
//  re-init past a threshold.
//
//  Both buses use the same explicit timeout configured in setup().
// =============================================================================

static const uint8_t MUX_ADDR = 0x70;
static const uint8_t SDP_ADDR = 0x26;   // all 7 SDP811s share this address

static const int NUM_PSENS = 7;          // mux channels 0-6; channel 7 unused

static const uint8_t CMD_START_HI = 0x36;
static const uint8_t CMD_START_LO = 0x1E;  // differential-pressure temperature compensation, no averaging
static const uint8_t CMD_STOP_HI  = 0x3F;
static const uint8_t CMD_STOP_LO  = 0xF9;

static const int CONSECUTIVE_FAIL_REINIT_THRESHOLD = 5;

struct SdpSensor {
  float    pressure_pa;
  float    temp_c;
  bool     ok;
  uint16_t consecutive_fails;
  uint32_t reinit_count;   // cumulative, surfaced in the Monitor block
  uint8_t  recovery_state;
  uint32_t recovery_due_us;
  uint32_t sample_us;      // completion time of latest valid pressure read
};

enum SdpRecoveryState : uint8_t {
  SDP_RECOVERY_IDLE,
  SDP_RECOVERY_WAIT_START,
  SDP_RECOVERY_WAIT_READY,
  SDP_RECOVERY_RETRY_WAIT,
};

static const uint32_t SDP_RECOVERY_RETRY_US = 100000UL;

static SdpSensor psens[NUM_PSENS];

static bool muxSelect(uint8_t channel) {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(1 << channel);
  return Wire1.endTransmission() == 0;
}

static bool sdpCheckCrc(uint8_t msb, uint8_t lsb, uint8_t crc) {
  uint8_t val = 0xFF;
  val ^= msb;
  for (int b = 0; b < 8; b++) val = (val & 0x80) ? ((val << 1) ^ 0x31) : (val << 1);
  val ^= lsb;
  for (int b = 0; b < 8; b++) val = (val & 0x80) ? ((val << 1) ^ 0x31) : (val << 1);
  return val == crc;
}

static bool sdpStart() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(CMD_START_HI);
  Wire1.write(CMD_START_LO);
  return Wire1.endTransmission() == 0;
}

static void sdpStop() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(CMD_STOP_HI);
  Wire1.write(CMD_STOP_LO);
  Wire1.endTransmission();  // datasheet: stop may NACK harmlessly if already idle
}

static bool sdpRead(float &pressure_pa, float &temp_c) {
  uint8_t n = Wire1.requestFrom((uint8_t)SDP_ADDR, (uint8_t)9);
  if (n < 9) {
    // This path runs hundreds of times per second; do not depend on the
    // current core resetting its RX index on the next requestFrom().
    while (Wire1.available()) Wire1.read();
    return false;
  }

  uint8_t dp_msb = Wire1.read();
  uint8_t dp_lsb = Wire1.read();
  uint8_t dp_crc = Wire1.read();
  uint8_t t_msb  = Wire1.read();
  uint8_t t_lsb  = Wire1.read();
  uint8_t t_crc  = Wire1.read();
  uint8_t sf_msb = Wire1.read();
  uint8_t sf_lsb = Wire1.read();
  uint8_t sf_crc = Wire1.read();

  if (!sdpCheckCrc(dp_msb, dp_lsb, dp_crc)) return false;
  if (!sdpCheckCrc(t_msb,  t_lsb,  t_crc))  return false;
  if (!sdpCheckCrc(sf_msb, sf_lsb, sf_crc)) return false;

  int16_t raw_dp   = (int16_t)((uint16_t)dp_msb << 8 | dp_lsb);
  int16_t raw_temp = (int16_t)((uint16_t)t_msb  << 8 | t_lsb);
  int16_t raw_sf   = (int16_t)((uint16_t)sf_msb << 8 | sf_lsb);

  if (raw_sf == 0) return false;

  pressure_pa = (float)raw_dp   / (float)raw_sf;
  temp_c      = (float)raw_temp / 200.0f;
  return true;
}

static bool initPsensChannel(uint8_t channel) {
  if (!muxSelect(channel)) return false;
  delay(20);  // let the bus settle after the channel switch — boot-time only,
              // does not touch the paced read loop's speed
  sdpStop();
  delay(1);
  if (!sdpStart()) return false;
  return true;
}

// Mean temperature across whichever channels succeeded this cycle.
// =============================================================================
//  DAC REGISTER BOUND — NOT A TRAVEL-SAFETY LIMIT
// =============================================================================
// A0 is a 12-bit DAC. Clamping a computed command is a sticky session fault
// indicator. No feedback correction or firmware travel clamp is applied.

static const int DAC_REG_MIN = 0;
static const int DAC_REG_MAX = 4095;

// A motion session begins in either startPreset() or startDemoSine(); both
// clear the flag before any DAC write, so it describes only the current session.
static bool actuator_clamped = false;  // sticky within the current motion session: any rail hit since the session began
static int  last_dac_out     = 0;      // actual value last written to A0

// THE ONLY function in this sketch permitted to call analogWrite(A0, ...).
static int writeActuatorDac(int dac_value) {
  const int clamped = constrain(dac_value, DAC_REG_MIN, DAC_REG_MAX);
  actuator_clamped = actuator_clamped || (clamped != dac_value);
  last_dac_out = clamped;
  analogWrite(A0, clamped);
  return clamped;
}

// =============================================================================
//  ACTUATOR POSITION FEEDBACK (pot tap) — TELEMETRY ONLY
// =============================================================================
//  CALIBRATION REQUIRED — see MechLung_Final/FlowTestingPY calibration
//  procedure notes (H hold + X hold, pair pot_raw_adc against known DAC
//  targets). Values below are carried over from FlowTestingPY's logged
//  calibration; re-calibrate for this rig if the pot wiring changes.
//  The scaled feedback value is sampled at the same 100Hz tick as the DAC
//  command and is never fed back into that command. The Jrk remains the only
//  component responsible for closed-loop actuator control.
// =============================================================================

// A second Jrk feedback-scale target, distinct from DAC_CENTRE, used only to
// give the H/X hold pair two separated points to calibrate the pot-to-DAC
// relationship against. It carries no safety meaning — the Jrk's own Input
// scaling confines the actual motion regardless of what value is requested
// here — so any value clearly different from DAC_CENTRE works; this one is
// carried over unchanged from the previous calibration.
static const int CALIBRATION_TARGET_JRK_SCALE = 1900;

static const uint8_t POT_FEEDBACK_PIN = A1;   // confirm this pin is free/wired

// VERIFY: POT_RAW_AT_CENTRE is inherited and unverified on this rig; re-derive it with the H/X hold pair before treating feedback_dac, fdbk_pct, or following error as quantitative.
static const float POT_RAW_AT_CENTRE      = 1414.0f;
// VERIFY: POT_DAC_COUNTS_PER_ADC is inherited and unverified on this rig; re-derive it with the H/X hold pair before treating feedback_dac, fdbk_pct, or following error as quantitative.
static const float POT_DAC_COUNTS_PER_ADC = 5.0118f;

static float position_follow_error = 0.0f;  // commanded - measured, diagnostic only
static int   pot_raw_adc_last      = 0;     // last raw analogRead(A1) — unfiltered

// Median-of-3 outlier filter on the pot read.
static int pot_hist[3]     = {0, 0, 0};
static int pot_hist_count  = 0;

static int medianOf3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

static void resetPotFilter() {
  pot_hist[0] = pot_hist[1] = pot_hist[2] = 0;
  pot_hist_count = 0;
}

static float readPotAsDacEquivalent() {
  pot_raw_adc_last = analogRead(POT_FEEDBACK_PIN);

  pot_hist[0] = pot_hist[1];
  pot_hist[1] = pot_hist[2];
  pot_hist[2] = pot_raw_adc_last;
  if (pot_hist_count < 3) pot_hist_count++;

  const int filtered = (pot_hist_count < 3)
      ? pot_raw_adc_last
      : medianOf3(pot_hist[0], pot_hist[1], pot_hist[2]);

  return corrected_centre_dac + ((float)filtered - POT_RAW_AT_CENTRE) * POT_DAC_COUNTS_PER_ADC;
}

static void resetPositionFeedback() {
  position_follow_error = 0.0f;
  resetPotFilter();
}

// =============================================================================
//  PRESET STRUCT + CALCULATION — the stroke amplitude and timing per preset.
//  The position profile applied to these values is stepDac()'s raised-cosine
//  ramp.
// =============================================================================

struct Preset {
  float q_peak_Lmin;
  float k_area;
  float a_mm;
  int   a_dac_raw;
  int   a_dac_corr;
  float t_exp_us;
  float t_insp_us;
  float period_us;
  float rr_bpm;
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
    presets[i].k_area      = k_area;
    presets[i].a_mm        = a_mm;
    presets[i].a_dac_raw   = a_dac;
    presets[i].a_dac_corr  = (int)((float)a_dac * gain_factor + 0.5f);
    presets[i].t_exp_us    = t_exp_s  * 1e6f;
    presets[i].t_insp_us   = t_insp_s * 1e6f;
    presets[i].period_us   = period_s * 1e6f;
    presets[i].rr_bpm      = 60.0f / period_s;
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

static const uint32_t DAC_TICK_US = 10000UL;  // 100 Hz

static int      current_preset = -1;  // -1 idle, -2 demo sine, 0-3 presets
// VERIFY: phi_us remains float because changing it alone would still narrow through float preset periods and fmodf(); confirm its float-ULP resolution near preset 1's longest period is acceptable on the rig before making a coordinated double-precision change.
static float    phi_us         = 0.0f;
static uint32_t last_dac_us    = 0;

static float last_feedback_dac = 0.0f;  // scaled A1 reading from the last tick

static void holdCentre() {
  writeActuatorDac(corrected_centre_dac);
}

static int stepDac(float phi) {
  const Preset &p = presets[current_preset];
  float pos_norm;

  if (phi < p.t_exp_us) {
    pos_norm = 0.5f * (1.0f - cosf(PI * phi / p.t_exp_us));
  } else {
    pos_norm = 0.5f * (1.0f + cosf(PI * (phi - p.t_exp_us) / p.t_insp_us));
  }

  const float profile_dac = corrected_centre_dac + (float)p.a_dac_corr * pos_norm;
  return writeActuatorDac((int)(profile_dac + 0.5f));
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

static int32_t sampleAgeUs(uint32_t now_us, bool valid, uint32_t sample_us) {
  if (!valid) return -1;
  return (int32_t)(now_us - sample_us);
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
  return writeActuatorDac((int)(dac_out_unclamped + 0.5f));
}

// Fixed storage: eight 640-byte records (~5.2 KiB). Two slots are reserved
// for STARTED/ENDED. No record is evicted after any of its bytes are sent.
static const size_t TX_SIZE = 640;
static const uint8_t TX_COUNT = 8;
struct TxRecord {
  char bytes[TX_SIZE];
  uint16_t size, offset;
};
static TxRecord tx[TX_COUNT];
static uint8_t tx_head = 0, tx_count = 0;
static uint32_t tx_dropped_total = 0;
static char format_buffer[TX_SIZE];
static bool enqueueText(const char *text, bool control = false, bool telemetry = false) {
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
static bool enqueueFormat(bool control, const char *format, ...) {
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(format_buffer, TX_SIZE, format, args);
  va_end(args);
  if (n < 0 || n >= (int)TX_SIZE) return false;
  return enqueueText(format_buffer, control);
}
static void serviceTx() {
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
static uint32_t seq = 0, scan_id = 0, cycle_id = 0, missed_slots_total = 0;
static uint32_t slot_us = 0, phase_update_us = 0;
static bool monitor_enabled = true;
static const uint32_t HOST_HEARTBEAT_TIMEOUT_MS = 3000;
static bool host_watchdog_armed = false, host_seen = false;
static uint32_t last_host_heartbeat_ms = 0;
static char run_id[17] = "", ended_id[17] = "", run_mode[5] = "";
// Completed IDs are never evicted: after 64 owned runs reject new STARTs
// until a deliberate board reset. Correlation IDs are not authentication.
static const uint8_t COMPLETED_CAPACITY = 64;
static char completed_ids[COMPLETED_CAPACITY][17];
static uint8_t completed_count = 0;
static bool completedId(const char *id) {
  for (uint8_t i = 0; i < completed_count; ++i)
    if (!strcmp(id, completed_ids[i])) return true;
  return false;
}
static char ended_marker[128] = "", started_marker[128] = "";
static int64_t final_enqueued_seq = -1;
#include "acquisition.h"
static void startMotion(int preset) {
  current_preset = preset;
  phi_us = 0;
  seq = scan_id = cycle_id = missed_slots_total = 0;
  actuator_clamped = false;
  scanner.active = false;
  final_enqueued_seq = -1;
  tx_dropped_total = 0;
  slot_us = phase_update_us = last_dac_us = micros();
  resetPositionFeedback();
  last_feedback_dac = readPotAsDacEquivalent();
  host_watchdog_armed = run_id[0] || (host_seen && (uint32_t)(millis() - last_host_heartbeat_ms) <
                                                       HOST_HEARTBEAT_TIMEOUT_MS);
}
static void startPreset(int idx) {
  startMotion(constrain(idx, 0, 3));
}
static void startDemoSine() {
  startMotion(-2);
}
static void goIdleHold() {
  current_preset = -1;
  host_watchdog_armed = false;
  scanner.active = false;
  resetPositionFeedback();
  holdCentre();
}
static void endSession(const char *reason) {
  // Hold first; unclosed acquisition is deliberately abandoned. ENDED is a
  // barrier behind all retained DATA2 frames, including a partially sent one.
  goIdleHold();
  if (!run_id[0]) return;
  strcpy(ended_id, run_id);
  if (completed_count < COMPLETED_CAPACITY) strcpy(completed_ids[completed_count++], run_id);
  snprintf(ended_marker, sizeof ended_marker, "# ENDED,2,%s,%lld,%s,%lu\n", run_id,
           (long long)final_enqueued_seq, reason, (unsigned long)tx_dropped_total);
  enqueueText(ended_marker, true);
  run_id[0] = 0;
}
static bool validId(const char *id) {
  if (strlen(id) != 16) return false;
  for (uint8_t i = 0; i < 16; ++i)
    if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) return false;
  return true;
}
static void handleFrame(char *line) {
  char *first = strchr(line, ',');
  if (!first) return;
  *first++ = 0;
  char *mode = strchr(first, ',');
  if (mode) *mode++ = 0;
  if (!validId(first)) return;
  if (!strcmp(line, "@PING") && !mode) {
    if (run_id[0] && !strcmp(run_id, first)) last_host_heartbeat_ms = millis();
  } else if (!strcmp(line, "@STOP") && !mode) {
    if (run_id[0] && !strcmp(run_id, first))
      endSession("host_stop");
    else if (!strcmp(ended_id, first) && tx_count < TX_COUNT - 1)
      enqueueText(ended_marker, true);
  } else if (!strcmp(line, "@START") && mode) {
    const int preset = !strcmp(mode, "DEMO") ? -2
                                             : (strlen(mode) == 2 && mode[0] == 'P' &&
                                                        mode[1] >= '0' && mode[1] <= '3'
                                                    ? mode[1] - '0'
                                                    : -1);
    if (preset == -1 || completedId(first)) return;
    if (run_id[0]) {
      if (!strcmp(run_id, first) && !strcmp(run_mode, mode) && tx_count < TX_COUNT - 1)
        enqueueText(started_marker, true);
      return;
    }
    if (completed_count == COMPLETED_CAPACITY) {
      enqueueText("# ERROR,2,completed_id_capacity_reset_required\n");
      return;
    }
    // Reserve STARTED plus a future ENDED before any motion.
    if (tx_count > TX_COUNT - 2) return;
    snprintf(started_marker, sizeof started_marker, "# STARTED,2,%s,%s,%lu,%lu\n", first, mode,
             (unsigned long)millis(), (unsigned long)micros());
    if (!enqueueText(started_marker, true)) return;
    strcpy(run_id, first);
    strcpy(run_mode, mode);
    host_seen = true;
    last_host_heartbeat_ms = millis();
    startMotion(preset);
  }
}
#ifndef MECH_BUILD_ID
#define MECH_BUILD_ID "unknown"
#endif
#ifndef MECH_CORE_ID
#define MECH_CORE_ID "unknown"
#endif
// Values come from the same compiled constants as the waveform. The compile
// script provides a source digest; it is provenance, not a binary signature.
static int meta_step = -1;
static uint32_t last_monitor_ms = 0;
static bool serviceMeta() {
  if (meta_step < 0 || tx_count >= TX_COUNT - 2) return false;
  bool ok = false;
  switch (meta_step) {
    case 0:
      ok = enqueueText("# PROTOCOL,2\n");
      break;
    case 1:
      ok = enqueueFormat(false, "# META_BEGIN,2,%s\n", MECH_BUILD_ID);
      break;
    case 2:
      ok = enqueueFormat(false, "# META,2,core,%s\n", MECH_CORE_ID);
      break;
    case 3:
      ok = enqueueFormat(false, "# META,2,tidal_ml,%.8g\n", V_TIDAL_ML);
      break;
    case 4:
      ok = enqueueFormat(false, "# META,2,bore_mm,%.8g\n", CYLINDER_DIAMETER_MM);
      break;
    case 5:
      ok = enqueueFormat(false, "# META,2,stroke_mm,%.8g\n", ACTUATOR_STROKE_MM);
      break;
    case 6:
      ok = enqueueFormat(false, "# META,2,dac_centre,%d\n", DAC_CENTRE);
      break;
    case 7:
      ok = enqueueFormat(false, "# META,2,dac_half_range,%d\n", DAC_HALF_RANGE);
      break;
    case 8:
      ok = enqueueFormat(false, "# META,2,dac_corr_offset,%.8g\n", DAC_CORR_OFFSET);
      break;
    case 9:
      ok = enqueueFormat(false, "# META,2,dac_corr_gain,%.8g\n", DAC_CORR_GAIN);
      break;
    case 10:
      ok = enqueueFormat(false, "# META,2,pot_raw_centre,%.8g\n", POT_RAW_AT_CENTRE);
      break;
    case 11:
      ok = enqueueFormat(false, "# META,2,pot_dac_per_adc,%.8g\n", POT_DAC_COUNTS_PER_ADC);
      break;
    case 12:
      ok = enqueueFormat(false, "# META,2,calibration_target,%d\n", CALIBRATION_TARGET_JRK_SCALE);
      break;
    case 13:
      ok = enqueueFormat(false, "# META,2,slot_us,%lu\n", (unsigned long)LOG_INTERVAL_US);
      break;
    case 14:
      ok = enqueueFormat(false, "# META,2,sfm_scale,%.8g\n", SFM3000_SCALE_FLOW_AIR);
      break;
    case 15:
      ok = enqueueText("# META,2,flow_reference,local_target_standard_293.15K_1013mbar_airN2\n");
      break;
    case 16:
      ok = enqueueFormat(false, "# META_END,2,%s\n", MECH_BUILD_ID);
      break;
    default:
      ok = enqueueText("# READY_QUERY\n");
      if (ok) meta_step = -2;
  }
  if (ok) ++meta_step;
  return ok;
}
static char rx_frame[64];
static uint8_t rx_length = 0;
static bool rx_in_frame = false, rx_discard = false;
static uint32_t rx_start_ms = 0;
static void handleSerial() {
  if (rx_in_frame && (uint32_t)(millis() - rx_start_ms) >= 250) rx_discard = true;
  for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (rx_in_frame) {
      if (c == '\n') {
        if (!rx_discard) {
          rx_frame[rx_length] = 0;
          handleFrame(rx_frame);
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
    if (c == 'm' || c == 'M') {
      monitor_enabled = !monitor_enabled;
      continue;
    }
    if ((c >= '1' && c <= '4') || c == 's' || c == 'S' || c == 'H' || c == 'h' || c == '0' ||
        c == 'X' || c == 'x') {
      if (run_id[0]) endSession("manual_override");
      if (c >= '1' && c <= '4')
        startPreset(c - '1');
      else if (c == 's' || c == 'S')
        startDemoSine();
      else {
        goIdleHold();
        if (c == 'X' || c == 'x')
          writeActuatorDac(applyDacCorrection(CALIBRATION_TARGET_JRK_SCALE));
      }
    }
  }
}
void setup() {
  // Centre precedes USB diagnostics and all I2C initialization.
  initPresets();
  corrected_centre_dac = applyDacCorrection(DAC_CENTRE);
  demo_sine_dac_min = DAC_REG_MIN + DEMO_SINE_MARGIN_DAC;
  demo_sine_dac_max = DAC_REG_MAX - DEMO_SINE_MARGIN_DAC;
  analogWriteResolution(12);
  analogReadResolution(12);
  holdCentre();
  Serial.begin(115200);
  delay(2000);
  i2cBusRecovery();
  Wire.begin();
  Wire.setClock(200000);
  Wire.setWireTimeout(I2C_TRANSACTION_TIMEOUT_US, false);
  i2cBusRecovery1();
  Wire1.begin();
  Wire1.setClock(400000);
  Wire1.setWireTimeout(I2C_TRANSACTION_TIMEOUT_US, false);
  sfm_ok = sfm3000Init();
  sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    if (!initPsensChannel(ch)) psens[ch].recovery_state = SDP_RECOVERY_RETRY_WAIT;
    delay(25);
  }
  slot_us = phase_update_us = last_dac_us = micros();
  enqueueText("# BOOT,2\n");
  enqueueFormat(false, "# INIT,flow_ok=%d,geometry_ml=%.1f,dac_centre=%d\n", sfm_ok, V_TIDAL_ML,
                corrected_centre_dac);
}
void loop() {
  if (host_watchdog_armed &&
      (uint32_t)(millis() - last_host_heartbeat_ms) >= HOST_HEARTBEAT_TIMEOUT_MS)
    endSession("watchdog");
  const uint32_t now = micros();
  const uint32_t elapsed = now - slot_us;
  if (elapsed >= LOG_INTERVAL_US) {
    const uint32_t slots = elapsed / LOG_INTERVAL_US;
    slot_us += slots * LOG_INTERVAL_US;
    missed_slots_total += slots - 1;
    if (scanner.active) closeScan();
    if (current_preset != -1) {
      const float period =
          current_preset == -2 ? DEMO_SINE_PERIOD_US : presets[current_preset].period_us;
      const float total = phi_us + (float)(uint32_t)(now - phase_update_us);
      cycle_id += (uint32_t)(total / period);
      phi_us = fmodf(total, period);
      if (current_preset == -2)
        stepDemoSine(phi_us);
      else
        stepDac(phi_us);
    }
    phase_update_us = last_dac_us = now;
    last_feedback_dac = readPotAsDacEquivalent();
    position_follow_error = last_dac_out - last_feedback_dac;
    if (current_preset != -1) startScan(now);
  }
  handleSerial();
  serviceTx();
  serviceMeta();
  serviceSensors();
  if (monitor_enabled && (uint32_t)(millis() - last_monitor_ms) >= 300 && tx_count < TX_COUNT - 3) {
    last_monitor_ms = millis();
    int pressure_ok_count = 0;
    uint32_t pressure_reinits = 0;
    for (const auto &sensor : psens) {
      pressure_ok_count += sensor.ok;
      pressure_reinits += sensor.reinit_count;
    }
    enqueueFormat(false,
                  "# "
                  "MONITOR,preset=%d,flow_ok=%d,pressure_ok=%d,flow_reinits=%lu,pressure_reinits=%"
                  "lu,clamped=%d\n",
                  current_preset, flow_sensor_ok, pressure_ok_count,
                  (unsigned long)sfm_reinit_count, (unsigned long)pressure_reinits,
                  actuator_clamped);
  }
}
