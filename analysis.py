"""Deterministic analysis shared by live capture and file replay."""
import time
from collections import deque
from dataclasses import dataclass
from recording_schema import Sample
from peak_detection import PeakFlowDetector, PeakDetectionConfig, standard_flow, continuity_reason, PeakEvent

class BoardTimeReset(ValueError):
    """Board clock discontinuity."""

@dataclass
class AnalysisResult:
    sample: Sample
    events: list[PeakEvent]

class MillisUnwrapper:
    """Convert the board's uint32 millis counter into monotonic seconds.

    Unsigned subtraction naturally handles the roughly 49.7-day wrap. A
    backward jump smaller than half the uint32 range is treated as a board
    reset and starts a new segment without making plotted time go backward.
    """

    def __init__(self):
        self.last_ms = None
        self.elapsed_ms = 0
        self.reset_count = 0

    def update(self, ms):
        ms &= 0xFFFFFFFF
        reset = False
        if self.last_ms is not None:
            delta = (ms - self.last_ms) & 0xFFFFFFFF
            if delta <= 0x7FFFFFFF:
                self.elapsed_ms += delta
            else:
                self.reset_count += 1
                reset = True
        self.last_ms = ms
        return self.elapsed_ms / 1000.0, reset

class AnalysisSession:
    def __init__(self, avg_window=1, config=None, gas_temperature_c=None,
                 absolute_pressure_mbar=None):
        self.avg_window = max(1, avg_window)
        self.config = config or PeakDetectionConfig()
        standard_flow(1.0, gas_temperature_c, absolute_pressure_mbar)
        self.gas_temperature_c = gas_temperature_c
        self.absolute_pressure_mbar = absolute_pressure_mbar
        self.reset()

    def reset(self, start_ms=0, start_us=0):
        self._flow_hist = deque(maxlen=self.avg_window)
        self.last_seq = self.last_sample = None
        self._clock = MillisUnwrapper()
        self._clock.update(start_ms)
        self._last_row_us = start_us
        self._elapsed_row_us = 0
        self.segment_id = self.dropped_rows = 0
        self.boundaries = []
        self.detector = PeakFlowDetector(self.config)

    def break_continuity(self, reason, before_seq=None):
        self._flow_hist.clear()
        self.segment_id += 1
        self.boundaries.append(dict(after_seq=self.last_seq, before_seq=before_seq, reason=reason))
        return self.detector.break_continuity(reason)

    def finish(self):
        return self.detector.flush()

    def push(self, w, active=True, host_unix=None):
        elapsed_s, reset = self._clock.update(w['ms'])
        if reset:
            raise BoardTimeReset('millis reset')
        if w['protocol'] == 2:
            delta = (w['row_us'] - self._last_row_us) & 0xffffffff
            if delta > 0x7fffffff:
                raise BoardTimeReset('micros reset')
            self._elapsed_row_us += delta
            self._last_row_us = w['row_us']
            if active and abs(self._elapsed_row_us/1e6-elapsed_s) > .003:
                raise BoardTimeReset('millis/micros disagree')
            row_time_us = self._elapsed_row_us
            flow_time_us = row_time_us-w['flow_age_us'] if w['flow_age_us'] >= 0 else row_time_us
            t_s = flow_time_us/1e6
        else:
            row_time_us = int(elapsed_s*1e6)
            flow_time_us = row_time_us-w['flow_age_us'] if w['flow_age_us'] >= 0 else row_time_us
            t_s = flow_time_us/1e6
        previous = self.last_sample
        # Classify continuity before smoothing, using the same rule as the
        # standalone detector. The detector receives only normalized segments.
        reason = continuity_reason(previous, seq=w['seq'], scan_id=w['scan_id'],
                                   t_s=t_s, run_id=w['run_id'],
                                   missed_slots_total=w['missed_slots_total'],
                                   tx_dropped_total=w['tx_dropped_total'],
                                   flow_ok=w['flow_ok'], max_gap_us=self.config.max_sample_gap_us)
        if previous is not None:
            delta_seq = (w['seq'] - previous.seq) & 0xffffffff
            if 1 < delta_seq < 0x80000000:
                self.dropped_rows += delta_seq - 1
        events = self.break_continuity(reason, w['seq']) if reason else []
        if w['flow_ok']:
            self._flow_hist.append(w['flow_raw'])
        # Divide before summing so finite extreme inputs cannot poison a
        # running sum. The bounded window is also replayed with this order.
        average = sum(value/len(self._flow_hist) for value in self._flow_hist) if self._flow_hist else 0.0
        resolved = standard_flow(w['target_peak_slm'], self.gas_temperature_c, self.absolute_pressure_mbar)
        target = w['target_peak_slm'] if resolved is None else resolved
        sample = Sample(
            w['seq'], w['ms'], t_s, time.time() if host_unix is None else host_unix, w['flow_raw'], average,
            w['flow_ok'], w['flow_age_us'], w['pa'], w['temp'], w['pok'], w['pa_age_us'],
            w['cmd_dac'], w['feedback_dac'], w['cmd_pct'], w['fdbk_pct'],
            w['target_flow_slm'], target, w['phase'], w['preset'], w['clamped'],
            w['run_id'], w['scan_id'], w['row_us'], w['cycle_id'], w['scan_start_us'],
            w['missed_slots_total'], w['tx_dropped_total'], row_time_us, flow_time_us,
            [row_time_us-a if a >= 0 else None for a in w['pa_age_us']],
            self.segment_id, reason or '', resolved is not None,
            w['target_peak_slm'])
        self.last_seq = sample.seq
        self.last_sample = sample
        if active:
            events.extend(self.detector.push(sample, check_continuity=False))
        return AnalysisResult(sample, events)
