#include "acquisition.h"
#include "calibration_values.h"
#include <Wire.h>
#include <initializer_list>
constexpr uint32_t I2C_TRANSACTION_TIMEOUT_US = 5000;
void Acquisition::i2cBusRecovery() {
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

void Acquisition::i2cBusRecovery1() {
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
//  On a failed read, sensors.flow_slm holds its last-good value (sensors.flow_sensor_ok
//  reports which happened). The logger plots invalid readings as gaps. Past SFM_FAIL_REINIT_THRESHOLD consecutive
//  failures the sensor is re-initialised, mirroring what the SDP811
//  array already did — without it, one transient main-bus glitch left the
//  flow channel dead for the rest of the session.
// =============================================================================

static const uint8_t SFM3000_I2C_ADDR       = 0x40;
static const float   SFM3000_OFFSET_FLOW    = 32000.0f;

static const int SFM_FAIL_REINIT_THRESHOLD = 10;  // ~100ms at SENSOR_CYCLE_US



static const uint32_t SFM_RETRY_INTERVAL_US = 1000000UL;


bool Acquisition::timeReachedUs(uint32_t now_us, uint32_t due_us) {
  return (int32_t)(now_us - due_us) >= 0;
}

// Start continuous conversion without waiting for the first sample. Runtime
// recovery uses this split phase so the actuator loop keeps running during
// the sensor's mandatory 100ms conversion delay.
void Acquisition::sfm3000BeginRecovery(uint32_t /*now_us*/) {
  sensors.sfm_ok = false;
  sensors.flow_sensor_ok = false;

  Wire.beginTransmission(SFM3000_I2C_ADDR);
  Wire.write(0x10);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    sensors.sfm_recovery_state = SFM_RECOVERY_DISCARD_WAIT;
    sensors.sfm_recovery_due_us = micros() + 100000UL;
  } else {
    sensors.sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sensors.sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
  }
}

void Acquisition::sfm3000Service(uint32_t now_us) {
  if (sensors.sfm_ok) {
    sfm3000ReadFlow();
    return;
  }
  if (!timeReachedUs(now_us, sensors.sfm_recovery_due_us)) return;

  if (sensors.sfm_recovery_state == SFM_RECOVERY_DISCARD_WAIT) {
    uint16_t discard;
    if (sfm3000ReadRaw(discard)) {
      sensors.sfm_ok = true;
      sensors.sfm_consecutive_fails = 0;
      return;
    }
    sensors.sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sensors.sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
    return;
  }

  sensors.sfm_reinit_count++;
  sfm3000BeginRecovery(now_us);
}

bool Acquisition::sfmCheckCrc(uint8_t hi, uint8_t lo, uint8_t expected) {
  // SFM3xxx: polynomial 0x31, initial value 0x00 (SDP uses 0xff).
  uint8_t crc = 0;
  for (uint8_t byte : {hi, lo}) {
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
  }
  return crc == expected;
}
bool Acquisition::sfm3000ReadRaw(uint16_t &raw_out) {
  if (Wire.requestFrom((int)SFM3000_I2C_ADDR, 3) != 3) return false;
  if (Wire.available() < 3) return false;
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  if (!sfmCheckCrc(hi, lo, Wire.read())) return false;
  raw_out = ((uint16_t)hi << 8) | lo;
  return true;
}

bool Acquisition::sfm3000ReadFlow() {
  uint16_t raw;
  sensors.flow_sensor_ok = sfm3000ReadRaw(raw);

  if (sensors.flow_sensor_ok) {
    sensors.flow_slm = ((float)raw - SFM3000_OFFSET_FLOW) / SFM3000_SCALE_FLOW_AIR;
    sensors.flow_sample_us = micros();
    sensors.sfm_consecutive_fails = 0;
    return true;
  }

  if (++sensors.sfm_consecutive_fails >= SFM_FAIL_REINIT_THRESHOLD) {
    sensors.sfm_consecutive_fails = 0;
    sensors.sfm_ok = false;
    sensors.sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
    sensors.sfm_recovery_due_us = micros();
  }
  return false;
}

bool Acquisition::sfm3000Init() {
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


static const uint8_t CMD_START_HI = 0x36;
static const uint8_t CMD_START_LO = 0x1E;  // differential-pressure temperature compensation, no averaging
static const uint8_t CMD_STOP_HI  = 0x3F;
static const uint8_t CMD_STOP_LO  = 0xF9;

static const int CONSECUTIVE_FAIL_REINIT_THRESHOLD = 5;

static const uint32_t SDP_RECOVERY_RETRY_US = 100000UL;


bool Acquisition::muxSelect(uint8_t channel) {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(1 << channel);
  return Wire1.endTransmission() == 0;
}

bool Acquisition::sdpCheckCrc(uint8_t msb, uint8_t lsb, uint8_t crc) {
  uint8_t val = 0xFF;
  val ^= msb;
  for (int b = 0; b < 8; b++) val = (val & 0x80) ? ((val << 1) ^ 0x31) : (val << 1);
  val ^= lsb;
  for (int b = 0; b < 8; b++) val = (val & 0x80) ? ((val << 1) ^ 0x31) : (val << 1);
  return val == crc;
}

bool Acquisition::sdpStart() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(CMD_START_HI);
  Wire1.write(CMD_START_LO);
  return Wire1.endTransmission() == 0;
}

void Acquisition::sdpStop() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(CMD_STOP_HI);
  Wire1.write(CMD_STOP_LO);
  Wire1.endTransmission();  // datasheet: stop may NACK harmlessly if already idle
}

bool Acquisition::sdpRead(float &pressure_pa, float &temp_c) {
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

bool Acquisition::initPsensChannel(uint8_t channel) {
  if (!muxSelect(channel)) return false;
  delay(20);  // let the bus settle after the channel switch — boot-time only,
              // does not touch the paced read loop's speed
  sdpStop();
  delay(1);
  if (!sdpStart()) return false;
  return true;
}


void Acquisition::begin() {
  i2cBusRecovery();
  Wire.begin(); Wire.setClock(200000); Wire.setWireTimeout(I2C_TRANSACTION_TIMEOUT_US, false);
  i2cBusRecovery1();
  Wire1.begin(); Wire1.setClock(400000); Wire1.setWireTimeout(I2C_TRANSACTION_TIMEOUT_US, false);
  sensors.sfm_ok = sfm3000Init();
  sensors.sfm_recovery_due_us = micros() + SFM_RETRY_INTERVAL_US;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    if (!initPsensChannel(ch)) sensors.psens[ch].recovery_state = SDP_RECOVERY_RETRY_WAIT;
    delay(25);
  }
}
bool Acquisition::serviceSensors(ScanSnapshot &completed, int closure_phase, uint32_t closure_cycle) {
  if (!scanner.active) return false;
  if (scanner.step == ScanStep::Flow) {
    sfm3000Service(micros());
    scanner.sample.flow = sensors.flow_slm;
    scanner.sample.flow_ok = sensors.flow_sensor_ok;
    scanner.sample.flow_sample_us = sensors.flow_sample_us;
    scanner.step = ScanStep::SelectPressure;
    return false;
  }
  SdpSensor &s = sensors.psens[scanner.channel];
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
    return false;
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
    return false;
  }
  ++scanner.done;
  scanner.channel = (scanner.channel + 1) % NUM_PSENS;
  scanner.step = ScanStep::SelectPressure;
  if (scanner.done == NUM_PSENS) return closeScan(completed, closure_phase, closure_cycle);
  return false;
}

bool Acquisition::closeScan(ScanSnapshot &completed, int closure_phase, uint32_t closure_cycle) {
  if (!scanner.active) return false;
  scanner.active = false;
  ScanSnapshot &sample = scanner.sample;
  sample.row_us = micros(); sample.row_ms = millis();
  if (sample.preset >= 0 && (closure_phase != sample.phase || closure_cycle != sample.cycle)) {
    sample.flow_ok = sensors.flow_sensor_ok = false;
    for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
      sample.pressure[ch].ok = sensors.psens[ch].ok = false;
    }
  }
  completed = sample;
  return true;
}
void Acquisition::startScan(const ScanSnapshot &motion) {
  scanner.active = true;
  scanner.done = 0; scanner.step = ScanStep::Flow;
  scanner.channel = scanner.rotation;
  scanner.rotation = (scanner.rotation+1) % NUM_PSENS;
  scanner.sample = motion;
  ScanSnapshot &sample = scanner.sample;
  sample.flow = sensors.flow_slm;
  sample.flow_ok = sensors.flow_sensor_ok = false;
  sample.flow_sample_us = sensors.flow_sample_us;
  for (uint8_t ch = 0; ch < NUM_PSENS; ++ch) {
    auto &s = sensors.psens[ch];
    s.ok = false;
    sample.pressure[ch] = {s.pressure_pa, s.temp_c, false, s.sample_us};
  }
}
