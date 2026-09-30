"""Peak-flow segmentation and pressure capture for the mechanical-lung logger."""

import math
from dataclasses import dataclass
from numbers import Integral
from typing import TypedDict, Optional
from recording_schema import Sample


DEFAULT_PEAK_TOLERANCE_PCT = 10.0
DEFAULT_MAX_SENSOR_AGE_MS = 20.0
DEFAULT_PEAK_THRESHOLD_FRAC = 0.35
DEFAULT_PEAK_THRESHOLD_FLOOR = 1.5
DEFAULT_WARMUP_BREATHS = 3


@dataclass(frozen=True)
class PeakDetectionConfig:
    """Validated peak-detection policy shared by analysis and display code."""

    threshold_frac: float = DEFAULT_PEAK_THRESHOLD_FRAC
    min_threshold: float = DEFAULT_PEAK_THRESHOLD_FLOOR
    exit_frac: float = 0.5
    min_samples: int = 9
    plateau_frac: float = 0.99
    smooth_window: int = 9
    target_tolerance_frac: float = DEFAULT_PEAK_TOLERANCE_PCT / 100.0
    max_sensor_age_us: float = DEFAULT_MAX_SENSOR_AGE_MS * 1000.0
    warmup_breaths: int = DEFAULT_WARMUP_BREATHS
    max_sample_gap_us: float = 15000.0

    def __post_init__(self):
        fractions = (
            "threshold_frac", "exit_frac", "plateau_frac",
            "target_tolerance_frac",
        )
        for field_name in fractions:
            value = self._finite_float(field_name, getattr(self, field_name))
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{field_name} must be greater than zero "
                                 "and no greater than one")
            object.__setattr__(self, field_name, value)

        for field_name in ("min_threshold", "max_sensor_age_us", "max_sample_gap_us"):
            value = self._finite_float(field_name, getattr(self, field_name))
            if value <= 0.0:
                raise ValueError(f"{field_name} must be greater than zero")
            object.__setattr__(self, field_name, value)

        for field_name, minimum in (("min_samples", 1),
                                    ("smooth_window", 1),
                                    ("warmup_breaths", 0)):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{field_name} must be an integer")
            value = int(value)
            if value < minimum:
                qualifier = "greater than zero" if minimum == 1 else "nonnegative"
                raise ValueError(f"{field_name} must be {qualifier}")
            object.__setattr__(self, field_name, value)

        if self.smooth_window % 2 == 0:
            raise ValueError("smooth_window must be odd (use 1 for no smoothing)")
        if self.min_samples < self.smooth_window:
            raise ValueError("min_samples must be at least smooth_window")

    @staticmethod
    def _finite_float(field_name, value):
        if isinstance(value, bool):
            raise ValueError(f"{field_name} must be a finite number")
        try:
            parsed = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name} must be a finite number") from exc
        if not math.isfinite(parsed):
            raise ValueError(f"{field_name} must be a finite number")
        return parsed

    @staticmethod
    def target_magnitude(target):
        if target is None:
            return 0.0
        target = abs(float(target))
        return target if math.isfinite(target) else 0.0

    def opening_threshold(self, target):
        return max(
            self.min_threshold,
            self.threshold_frac * self.target_magnitude(target),
        )


def standard_flow(local_flow, temperature_c, pressure_mbar):
    """Convert geometric local L/min to SFM air/N2 standard L/min."""
    if temperature_c is None and pressure_mbar is None:
        return None
    if temperature_c is None or pressure_mbar is None:
        raise ValueError("gas temperature and absolute pressure must be supplied together")
    values = (local_flow, temperature_c, pressure_mbar)
    if any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
        raise ValueError("flow reference inputs must be finite")
    if temperature_c <= -273.15 or pressure_mbar <= 0:
        raise ValueError("temperature must exceed absolute zero and pressure must be positive")
    return local_flow * (pressure_mbar / 1013.0) * (293.15 / (temperature_c + 273.15))


def centred_moving_average(vals, window):
    """Centered mean; unsupported edges/short segments are NaN, never raw.

    Only finite supported results may participate in peak/plateau selection.
    Window 1 explicitly requests unsmoothed analysis.
    """
    if isinstance(window, bool) or not isinstance(window, Integral) or window < 1 or window % 2 == 0:
        raise ValueError("window must be a positive odd integer")
    half = window // 2
    return [sum(value / window for value in vals[i-half:i+half+1])
            if half <= i < len(vals)-half else float('nan')
            for i in range(len(vals))]


def interpolate_peak(times, values):
    """Fit a concave parabola in actual relative seconds; return (dt, y, status)."""
    left, centre, right = times
    if not left < centre < right:
        return 0.0, values[1], 'nonmonotonic_time'
    x1, x2 = left-centre, right-centre
    y1, y0, y2 = values
    a = ((y2-y0)/x2 - (y1-y0)/x1) / (x2-x1)
    b = (y2-y0)/x2 - a*x2
    if not math.isfinite(a) or a >= 0:
        return 0.0, y0, 'nonconcave'
    dt = -b/(2*a)
    if not math.isfinite(dt) or not x1/2 <= dt <= x2/2:
        return 0.0, y0, 'vertex_outside_midpoints'
    return dt, y0 + b*dt + a*dt*dt, 'interpolated'


def continuity_reason(previous, *, seq, scan_id, t_s, run_id,
                      missed_slots_total, tx_dropped_total, flow_ok, max_gap_us):
    """One continuity policy for normalization and standalone detection."""
    if not flow_ok:
        return 'invalid_flow'
    if previous is None:
        return None
    if (seq - previous.seq) & 0xffffffff != 1:
        return 'sequence_discontinuity'
    if scan_id is not None and previous.scan_id is not None and (scan_id - previous.scan_id) & 0xffffffff != 1:
        return 'scan_discontinuity'
    if not 0 < (t_s - previous.t_s) * 1e6 <= max_gap_us:
        return 'flow_time_discontinuity'
    if run_id != previous.run_id:
        return 'run_discontinuity'
    if missed_slots_total != previous.missed_slots_total or tx_dropped_total != previous.tx_dropped_total:
        return 'firmware_loss'
    return None


class PeakEvent(TypedDict):
    peak_idx: int
    sample: Sample
    direction: int
    flow_peak_slm: float
    plateau_reference_flow_slm: float
    peak_time_s: float
    peak_offset_s: float
    interpolation_status: str
    nearest_sample_seq: int
    target_peak_slm: float
    target_error_slm: float
    target_error_pct: float
    target_met: Optional[bool]
    is_warmup: bool
    excursion_complete: bool
    pa_mean: list[float]
    pa_std: list[float]
    pa_ok_frac: list[float]
    pressure_all_ok: bool
    timing_ok: bool
    peak_sample_ok: bool
    plateau_capture_ok: bool
    qualified_peak: Optional[bool]
    reason_codes: list[str]
    plateau_n: int
    plateau_contiguous: bool
    max_pressure_age_us: int
    pressure_age_span_us: int
    flow_pressure_skew_us: int
    plateau_span_s: float
    excursion_n: int
    excursion_s: float


class PeakFlowDetector:
    """Baseline-armed, continuity-aware excursion analysis (policy version 2)."""
    def __init__(self, config):
        self.config = config
        self._buf = []
        self._sign = 0
        self._armed = False
        self._exit = None
        self._previous = None
        self._break_reason = None
        self.count = self.discarded = 0
        self.target_passes = self.target_misses = 0
        self.partial_excursions = self.warmup_peaks = self.reference_unresolved = 0
        self.pressure_incomplete = self.timing_failures = 0
        self.qualified_peaks = self.unqualified_peaks = 0
        self.discard_reasons = {}

    def break_continuity(self, reason) -> list[PeakEvent]:
        self._break_reason = reason
        events = self._close(False) if self._buf else []
        self._armed = False
        self._previous = None
        return events

    def push(self, sample: Sample, *, check_continuity=True) -> list[PeakEvent]:
        if not sample.flow_ok or not all(math.isfinite(v) for v in (sample.flow_raw, sample.flow_avg, sample.t_s)):
            return self.break_continuity('invalid_flow')
        events = []
        if check_continuity:
            reason = continuity_reason(self._previous, seq=sample.seq, scan_id=sample.scan_id,
                                       t_s=sample.t_s, run_id=sample.run_id,
                                       missed_slots_total=sample.missed_slots_total,
                                       tx_dropped_total=sample.tx_dropped_total,
                                       flow_ok=sample.flow_ok, max_gap_us=self.config.max_sample_gap_us)
            if reason:
                events.extend(self.break_continuity(reason))
        self._previous = sample
        threshold = self.config.opening_threshold(sample.target_peak_slm)
        exit_level = self._exit if self._exit is not None else threshold*self.config.exit_frac
        baseline = abs(sample.flow_raw) < exit_level and abs(sample.flow_avg) < exit_level
        if baseline:
            if self._buf:
                events.extend(self._close(True))
            self._armed = True
            self._exit = None
            self._break_reason = None
        elif self._buf:
            first = self._buf[0]
            if sample.phase != first.phase or sample.cycle_id != first.cycle_id:
                events.extend(self.break_continuity('phase_cycle_discontinuity'))
            elif self._sign * sample.flow_avg <= 0:
                events.extend(self.break_continuity('sign_reversal_without_baseline'))
            else:
                self._buf.append(sample)
        elif self._armed and abs(sample.flow_avg) >= threshold:
            self._sign = 1 if sample.flow_avg > 0 else -1
            self._exit = threshold*self.config.exit_frac
            self._buf = [sample]
        return events

    def flush(self) -> list[PeakEvent]:
        return self.break_continuity('session_end')

    def _close(self, complete):
        samples, sign = self._buf, self._sign
        self._buf, self._sign = [], 0
        if len(samples) < self.config.min_samples:
            self.discarded += 1
            self.discard_reasons['insufficient_support'] = self.discard_reasons.get('insufficient_support', 0)+1
            return []
        smooth = centred_moving_average([s.flow_raw for s in samples], self.config.smooth_window)
        supported = [i for i, v in enumerate(smooth) if math.isfinite(v)]
        peak_index = max(supported, key=lambda i: sign*smooth[i])
        reference = sign*smooth[peak_index]
        peak_sample = samples[peak_index]
        dt, magnitude, interpolation = 0.0, reference, 'unsupported_neighbours'
        if peak_index-1 in supported and peak_index+1 in supported:
            dt, magnitude, interpolation = interpolate_peak(
                [s.t_s for s in samples[peak_index-1:peak_index+2]],
                [sign*v for v in smooth[peak_index-1:peak_index+2]])
        indices = [i for i in supported if sign*smooth[i] >= self.config.plateau_frac*reference]
        plateau = [samples[i] for i in indices]
        contiguous = bool(indices) and all(b==a+1 for a,b in zip(indices,indices[1:]))
        means, stds, fractions = [], [], []
        for ch in range(7):
            vals = [s.pa[ch] for s in plateau if s.pok[ch] and math.isfinite(s.pa[ch])]
            mean = sum(vals)/len(vals) if vals else float('nan')
            means.append(mean)
            stds.append(math.sqrt(sum((v-mean)**2 for v in vals)/len(vals)) if vals else float('nan'))
            fractions.append(len(vals)/len(plateau) if plateau else 0)
        def pressure_ok(s):
            return all(ok and math.isfinite(v) for ok,v in zip(s.pok,s.pa))
        def timing_ok(s):
            return (0 <= s.flow_age_us <= self.config.max_sensor_age_us and
                    all(0 <= age <= self.config.max_sensor_age_us for age in s.pa_age_us))
        peak_ok = pressure_ok(peak_sample) and timing_ok(peak_sample)
        clamped = any(s.clamped for s in samples)
        capture_ok = contiguous and all(pressure_ok(s) and timing_ok(s) for s in plateau) and not clamped
        warmup = peak_sample.cycle_id is None or peak_sample.cycle_id < self.config.warmup_breaths
        comparable = peak_sample.target_reference_resolved and peak_sample.target_peak_slm > 0
        target = self.config.target_magnitude(peak_sample.target_peak_slm)
        error = magnitude-target if comparable else float('nan')
        target_met = abs(error) <= self.config.target_tolerance_frac*target if complete and comparable else None
        reasons = []
        if not complete: reasons.append(self._break_reason or 'partial')
        if not comparable: reasons.append('flow_reference_unresolved')
        if warmup: reasons.append('warmup')
        if not peak_ok: reasons.append('peak_sample_invalid')
        if not capture_ok: reasons.append('plateau_capture_invalid')
        if not contiguous: reasons.append('disjoint_plateau')
        if clamped: reasons.append('session_clamped')
        if target_met is False: reasons.append('target_miss')
        qualified = bool(target_met and peak_ok and capture_ok) if complete and comparable and not warmup else None
        if not complete: self.partial_excursions += 1
        elif warmup: self.warmup_peaks += 1
        elif not comparable: self.reference_unresolved += 1
        else:
            self.target_passes += target_met is True
            self.target_misses += target_met is False
            self.qualified_peaks += qualified is True
            self.unqualified_peaks += qualified is False
            self.pressure_incomplete += not all(pressure_ok(s) for s in plateau)
            self.timing_failures += not all(timing_ok(s) for s in plateau)
        ages = [a for ok,a in zip(peak_sample.pok,peak_sample.pa_age_us) if ok and a>=0]
        self.count += 1
        return [PeakEvent(
            peak_idx=self.count, sample=peak_sample, direction=sign,
            flow_peak_slm=sign*magnitude, plateau_reference_flow_slm=sign*reference,
            peak_time_s=peak_sample.t_s+dt, peak_offset_s=dt,
            interpolation_status=interpolation, nearest_sample_seq=peak_sample.seq,
            target_peak_slm=target, target_error_slm=error,
            target_error_pct=100*error/target if comparable else float('nan'),
            target_met=target_met, is_warmup=warmup, excursion_complete=complete,
            pa_mean=means,pa_std=stds,pa_ok_frac=fractions,
            pressure_all_ok=pressure_ok(peak_sample),timing_ok=timing_ok(peak_sample),
            peak_sample_ok=peak_ok,plateau_capture_ok=capture_ok,qualified_peak=qualified,
            reason_codes=reasons,plateau_n=len(plateau),plateau_contiguous=contiguous,
            max_pressure_age_us=max(ages) if ages else -1,
            pressure_age_span_us=max(ages)-min(ages) if ages else -1,
            flow_pressure_skew_us=max(abs(a-peak_sample.flow_age_us) for a in ages) if ages else -1,
            plateau_span_s=plateau[-1].t_s-plateau[0].t_s if plateau else 0,
            excursion_n=len(samples),excursion_s=samples[-1].t_s-samples[0].t_s)]
