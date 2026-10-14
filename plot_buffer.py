"""Bounded display buffers; filesystem work never holds the snapshot lock."""
import threading
from collections import deque, namedtuple

NUM_PSENS = 7
# display name, sample field, per-channel field
PLOTTED_FIELDS = (
    ('t', 't_s', False), ('flow_raw', 'flow_raw', False), ('flow_avg', 'flow_avg', False),
    ('flow_ok', 'flow_ok', False), ('flow_age_us', 'flow_age_us', False),
    ('psens_pa', 'pa', True), ('psens_temp_avg', 'temp', False),
    ('psens_ok', 'pok', True), ('psens_age_us', 'pa_age_us', True),
    *((name, name, False) for name in ('cmd_dac', 'feedback_dac', 'cmd_pct', 'fdbk_pct',
       'target_flow_slm', 'target_peak_slm', 'phase', 'preset', 'clamped',
       'pot_raw_adc', 'calibration_position')),
)
PlotSnapshot = namedtuple('PlotSnapshot', [name for name, _, _ in PLOTTED_FIELDS])


class PlotBuffer:
    def __init__(self, buffer_seconds=180.0, sample_hz=100.0):
        self.lock = threading.Lock()
        maxlen = max(1, int(buffer_seconds*sample_hz))
        for name, _, channels in PLOTTED_FIELDS:
            setattr(self, name, [deque(maxlen=maxlen) for _ in range(NUM_PSENS)]
                    if channels else deque(maxlen=maxlen))

    def clear(self):
        with self.lock:
            for name, _, channels in PLOTTED_FIELDS:
                series = getattr(self, name)
                for values in series if channels else (series,):
                    values.clear()

    def append(self, sample):
        with self.lock:
            for name, field, channels in PLOTTED_FIELDS:
                value = getattr(sample, field)
                series = getattr(self, name)
                if channels:
                    for ch, values in enumerate(series):
                        values.append(float('nan') if field == 'pa' and not sample.pok[ch] else value[ch])
                else:
                    if field in ('flow_raw', 'flow_avg') and not sample.flow_ok:
                        value = float('nan')
                    series.append(value)

    def snapshot(self):
        with self.lock:
            return PlotSnapshot(**{
                name: [list(values) for values in getattr(self, name)] if channels
                else list(getattr(self, name)) for name, _, channels in PLOTTED_FIELDS})
