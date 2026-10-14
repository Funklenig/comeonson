#pragma once
#include <Arduino.h>
#include "scan_snapshot.h"
class Transport {
 public:
  explicit Transport(bool is_calibration) : calibration(is_calibration) {}
  // 'B' = acknowledged START, 'E' = matching STOP; other keys go to sketch.
  char poll();
  void serviceTx();
  bool serviceMeta();
  void publish(ScanSnapshot sample);
  void resetCounters();
  void endSession(const char *reason);
  void armMotion();
  bool expired() const;
  bool active() const { return run_id[0] != 0; }
  bool monitorReady() const { return tx_count < TX_COUNT-3; }
  bool enqueueText(const char *text, bool control=false, bool telemetry=false);
  bool enqueueFormat(bool control, const char *format, ...);
  bool monitor_enabled = true;
  int preset = -1;
  char run_id[17] = "";
 private:
  bool calibration;
  static constexpr size_t TX_SIZE = 640;
  static constexpr uint8_t TX_COUNT = 8;
  struct TxRecord { char bytes[TX_SIZE]; uint16_t size, offset; };
  TxRecord tx[TX_COUNT]{};
  uint8_t tx_head = 0, tx_count = 0;
  uint32_t tx_dropped_total = 0, seq = 0;
  int64_t final_enqueued_seq = -1;
  char format_buffer[TX_SIZE];
  static constexpr uint32_t HOST_HEARTBEAT_TIMEOUT_MS = 3000;
  bool host_watchdog_armed = false, host_seen = false;
  uint32_t last_host_heartbeat_ms = 0;
  char ended_id[17] = "", run_mode[5] = "";
  static constexpr uint8_t COMPLETED_CAPACITY = 64;
  char completed_ids[COMPLETED_CAPACITY][17]{};
  uint8_t completed_count = 0;
  char ended_marker[128] = "", started_marker[128] = "";
  int meta_step = -1;
  char rx_frame[64];
  uint8_t rx_length = 0;
  bool rx_in_frame = false, rx_discard = false;
  uint32_t rx_start_ms = 0;
  bool completedId(const char *id) const;
  static bool validId(const char *id);
  char handleFrame(char *line);
};
