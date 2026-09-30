"""Bounded display buffers; filesystem work never holds the snapshot lock."""
import threading
from collections import deque
NUM_PSENS = 7

class PlotBuffer:
    def __init__(self, buffer_seconds=180.0, sample_hz=100.0):
        maxlen = int(buffer_seconds * sample_hz)
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.flow_raw = deque(maxlen=maxlen)
        self.flow_avg = deque(maxlen=maxlen)
        self.flow_ok = deque(maxlen=maxlen)
        self.flow_age_us = deque(maxlen=maxlen)
        self.psens_pa = [deque(maxlen=maxlen) for _ in range(NUM_PSENS)]
        self.psens_temp_avg = deque(maxlen=maxlen)
        self.psens_ok = [deque(maxlen=maxlen) for _ in range(NUM_PSENS)]
        self.psens_age_us = [deque(maxlen=maxlen) for _ in range(NUM_PSENS)]
        self.cmd_dac = deque(maxlen=maxlen)
        self.feedback_dac = deque(maxlen=maxlen)
        self.cmd_pct = deque(maxlen=maxlen)
        self.fdbk_pct = deque(maxlen=maxlen)
        self.target_flow_slm = deque(maxlen=maxlen)
        self.target_peak_slm = deque(maxlen=maxlen)
        self.phase = deque(maxlen=maxlen)
        self.preset = deque(maxlen=maxlen)
        self.clamped = deque(maxlen=maxlen)

    def clear(self):
        with self.lock:
            for name in ('t', 'flow_raw', 'flow_avg', 'flow_ok', 'flow_age_us',
                         'psens_temp_avg', 'cmd_dac', 'feedback_dac', 'cmd_pct',
                         'fdbk_pct', 'target_flow_slm', 'target_peak_slm', 'phase',
                         'preset', 'clamped'):
                getattr(self, name).clear()
            for name in ('psens_pa', 'psens_ok', 'psens_age_us'):
                for series in getattr(self, name):
                    series.clear()

    def append(self, sample):
        with self.lock:
            self.t.append(sample.t_s)
            for name in ('flow_raw', 'flow_avg', 'flow_ok', 'flow_age_us', 'cmd_dac',
                         'feedback_dac', 'cmd_pct', 'fdbk_pct', 'target_flow_slm',
                         'target_peak_slm', 'phase', 'preset', 'clamped'):
                value = getattr(sample, name)
                if name in ('flow_raw', 'flow_avg') and not sample.flow_ok:
                    value = float('nan')
                getattr(self, name).append(value)
            self.psens_temp_avg.append(sample.temp)
            for ch in range(7):
                self.psens_pa[ch].append(sample.pa[ch] if sample.pok[ch] else float('nan'))
                self.psens_ok[ch].append(sample.pok[ch])
                self.psens_age_us[ch].append(sample.pa_age_us[ch])

    def snapshot(self):
        with self.lock:
            return (list(self.t), list(self.flow_raw), list(self.flow_avg), list(self.flow_ok),
                    list(self.flow_age_us), [list(x) for x in self.psens_pa], list(self.psens_temp_avg),
                    [list(x) for x in self.psens_ok], [list(x) for x in self.psens_age_us],
                    list(self.cmd_dac), list(self.feedback_dac), list(self.cmd_pct), list(self.fdbk_pct),
                    list(self.target_flow_slm), list(self.target_peak_slm), list(self.phase),
                    list(self.preset), list(self.clamped))
