#pragma once
#include <Arduino.h>
#include "scan_snapshot.h"
enum SfmRecoveryState : uint8_t {
  SFM_RECOVERY_RETRY_WAIT,
  SFM_RECOVERY_DISCARD_WAIT,
};

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

struct SensorState {
  bool sfm_ok = false, flow_sensor_ok = false;
  float flow_slm = 0;
  uint32_t flow_sample_us = 0, sfm_reinit_count = 0, sfm_recovery_due_us = 0;
  uint16_t sfm_consecutive_fails = 0;
  SfmRecoveryState sfm_recovery_state = SFM_RECOVERY_RETRY_WAIT;
  SdpSensor psens[NUM_PSENS]{};
};
enum class ScanStep : uint8_t { Flow, SelectPressure, ReadPressure, Advance };
struct ScanState {
  bool active = false;
  uint8_t channel = 0, done = 0, rotation = 0;
  ScanStep step = ScanStep::Flow;
  ScanSnapshot sample{};
};
class Acquisition {
 public:
  explicit Acquisition(SensorState &owned_sensors) : sensors(owned_sensors) {}
  void begin();
  void startScan(const ScanSnapshot &motion);
  bool serviceSensors(ScanSnapshot &completed, int closure_phase, uint32_t closure_cycle);
  bool closeScan(ScanSnapshot &completed, int closure_phase, uint32_t closure_cycle);
  void cancel() { scanner.active = false; }
 private:
  SensorState &sensors;
  ScanState scanner;
  void i2cBusRecovery();
  void i2cBusRecovery1();
  bool timeReachedUs(uint32_t now_us, uint32_t due_us);
  void sfm3000BeginRecovery(uint32_t /*now_us*/);
  void sfm3000Service(uint32_t now_us);
  bool sfmCheckCrc(uint8_t hi, uint8_t lo, uint8_t expected);
  bool sfm3000ReadRaw(uint16_t &raw_out);
  bool sfm3000ReadFlow();
  bool sfm3000Init();
  bool muxSelect(uint8_t channel);
  bool sdpCheckCrc(uint8_t msb, uint8_t lsb, uint8_t crc);
  bool sdpStart();
  void sdpStop();
  bool sdpRead(float &pressure_pa, float &temp_c);
  bool initPsensChannel(uint8_t channel);
};
