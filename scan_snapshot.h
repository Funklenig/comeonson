#pragma once
// A completed scan is a value: formatting cannot change acquisition state.
struct PressureReading {
  float pressure_pa, temp_c;
  bool ok;
  uint32_t sample_us;
};
struct ScanSnapshot {
  uint32_t start_us, row_us, row_ms, id, cycle, sequence, missed_slots, dropped;
  int phase, preset, dac;
  float target, peak, feedback, flow;
  bool flow_ok, clamped;
  uint32_t flow_sample_us;
  PressureReading pressure[NUM_PSENS];
  char run_id[17];
};
