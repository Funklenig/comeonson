#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <actuator_io.h>
#include <acquisition.h>
#include <transport.h>

static ActuatorIO actuator;
static SensorState sensors;
static Acquisition acquisition(sensors);
static Transport transport(true);
static const char *position = "centre";
static uint32_t slot_us = 0, scan_id = 0, missed_slots_total = 0, last_monitor_ms = 0;

static void publish(ScanSnapshot &sample) {
  sample.missed_slots = missed_slots_total;
  transport.publish(sample);
}
static void setPosition(int dac, const char *label) {
  // Finish the old-position scan before applying a new target; never relabel old measurements.
  ScanSnapshot completed{};
  if (acquisition.closeScan(completed, -1, 0)) publish(completed);
  actuator.write(dac);
  position = label;
}
static void stopSession(const char *reason) {
  actuator.centre();
  position = "centre";
  acquisition.cancel();
  transport.endSession(reason);
}
static void dispatch(char key) {
  if (key == 'B') {
    acquisition.cancel();
    scan_id = missed_slots_total = 0;
    actuator.reset();
    actuator.centre();
    position = "centre";
    slot_us = micros();
  } else if (key == 'E') stopSession("host_stop");
  else if (key == 'H' || key == 'h' || key == '0') {
    setPosition(applyDacCorrection(DAC_CENTRE), "centre");
  } else if (key == 'X' || key == 'x' || key == 'm' || key == 'n') {
    if (!transport.active()) {
      transport.enqueueText("# ERROR,3,start_CAL_session_first\n");
      return;
    }
    const int target = key == 'm' ? CAL_MAX_DAC : key == 'n' ? CAL_MIN_DAC :
                       applyDacCorrection(CALIBRATION_TARGET_JRK_SCALE);
    if (target < 0) transport.enqueueText("# ERROR,3,configure_rig_endpoint_first\n");
    else setPosition(target, key == 'm' ? "max" : key == 'n' ? "min" : "reference");
  } else if ((key >= '1' && key <= '4') || key == 's' || key == 'S') {
    transport.enqueueText("# ERROR,3,production_firmware_required\n");
  }
}
void setup() {
  actuator.begin();
  Serial.begin(460800);
  delay(2000);
  acquisition.begin();
  slot_us = micros();
  transport.enqueueText("# BOOT,3\n");
}
void loop() {
  if (transport.expired()) stopSession("watchdog");
  const uint32_t now = micros();
  const uint32_t elapsed = now-slot_us;
  ScanSnapshot completed{};
  if (elapsed >= LOG_INTERVAL_US) {
    const uint32_t slots = elapsed/LOG_INTERVAL_US;
    slot_us += slots*LOG_INTERVAL_US;
    missed_slots_total += slots-1;
    if (acquisition.closeScan(completed, -1, 0)) publish(completed);
    actuator.sample();
    ScanSnapshot motion{};
    motion.id = ++scan_id; motion.start_us = now;
    motion.preset = -3; motion.phase = -1; // no breathing target, cycle or peak policy
    motion.dac = actuator.dac; motion.feedback = actuator.feedback;
    motion.pot_raw_adc = actuator.raw; motion.calibration_position = position;
    motion.clamped = actuator.clamped;
    strcpy(motion.run_id, transport.run_id);
    acquisition.startScan(motion);
  }
  dispatch(transport.poll());
  transport.serviceTx();
  transport.serviceMeta();
  if (acquisition.serviceSensors(completed, -1, 0)) publish(completed);
  if (transport.monitor_enabled && (uint32_t)(millis()-last_monitor_ms) >= 300 && transport.monitorReady()) {
    last_monitor_ms = millis();
    transport.enqueueFormat(false, "# MONITOR,position=%s,dac=%d,pot_raw_adc=%d,feedback_dac=%.2f\n",
                            position, actuator.dac, actuator.raw, actuator.feedback);
  }
}
