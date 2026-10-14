"""Peak analysis for saved recordings; acquisition uses capture.py alone."""

from capture import (
    CaptureSession,
    CaptureResult as AnalysisResult,
    BoardTimeReset,
    CaptureDiscontinuity,
    MillisUnwrapper,
)
from peak_detection import PeakFlowDetector, PeakDetectionConfig, standard_flow


class AnalysisSession(CaptureSession):
    def __init__(
        self,
        avg_window=1,
        config=None,
        gas_temperature_c=None,
        absolute_pressure_mbar=None,
        *,
        score_peaks=True
    ):
        self.config = config or PeakDetectionConfig()
        self.gas_temperature_c = gas_temperature_c
        self.absolute_pressure_mbar = absolute_pressure_mbar
        self.score_peaks = score_peaks
        standard_flow(1.0, gas_temperature_c, absolute_pressure_mbar)
        super().__init__(avg_window, self.config.max_sample_gap_us)

    def reset(self, start_ms=0, start_us=0):
        super().reset(start_ms, start_us)
        self.detector = PeakFlowDetector(self.config)

    def break_continuity(self, reason, before_seq=None):
        events = self.detector.break_continuity(reason)
        super().break_continuity(reason, before_seq)
        return events

    def finish(self):
        return self.detector.flush()

    def target_peak(self, local_peak):
        resolved = standard_flow(local_peak, self.gas_temperature_c, self.absolute_pressure_mbar)
        return (local_peak if resolved is None else resolved), resolved is not None

    def push(self, wire, active=True, host_unix=None):
        result = super().push(wire, active, host_unix)
        if active and self.score_peaks:
            result.events.extend(self.detector.push(result.sample, check_continuity=False))
        return result
