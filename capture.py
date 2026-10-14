"""Sample timestamps, validity and loss accounting; no peak-analysis dependencies."""

import time
from collections import deque
from dataclasses import dataclass
from recording_schema import Sample


def continuity_reason(
    previous,
    *,
    seq,
    scan_id,
    t_s,
    run_id,
    missed_slots_total,
    tx_dropped_total,
    flow_ok,
    max_gap_us
):
    """One continuity policy for normalization and standalone detection."""
    if not flow_ok:
        return "invalid_flow"
    if previous is None:
        return None
    if (seq - previous.seq) & 0xFFFFFFFF != 1:
        return "sequence_discontinuity"
    if (
        scan_id is not None
        and previous.scan_id is not None
        and (scan_id - previous.scan_id) & 0xFFFFFFFF != 1
    ):
        return "scan_discontinuity"
    if not 0 < (t_s - previous.t_s) * 1e6 <= max_gap_us:
        return "flow_time_discontinuity"
    if run_id != previous.run_id:
        return "run_discontinuity"
    if (
        missed_slots_total != previous.missed_slots_total
        or tx_dropped_total != previous.tx_dropped_total
    ):
        return "firmware_loss"
    return None


class CaptureDiscontinuity(ValueError):
    """Owned counters regressed or repeated; capture integrity is lost."""


class BoardTimeReset(ValueError):
    """Board clock discontinuity."""


@dataclass
class CaptureResult:
    sample: Sample
    events: list


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


class CaptureSession:
    MAX_BOUNDARIES = 1024

    def __init__(self, avg_window=1, max_gap_us=15000):
        self.avg_window = max(1, avg_window)
        self.max_gap_us = max_gap_us
        self.reset()

    def reset(self, start_ms=0, start_us=0):
        self._flow_hist = deque(maxlen=self.avg_window)
        self.last_seq = self.last_sample = None
        self._clock = MillisUnwrapper()
        self._clock.update(start_ms)
        self._last_row_us = start_us
        self._elapsed_row_us = 0
        self.segment_id = self.dropped_rows = self.missed_acquisition_slots = 0
        self.firmware_tx_drops = 0
        self.owned = False
        self.boundaries = []
        self.boundaries_omitted = 0
        self._break_reason = None

    def set_origin(self, start_ms, start_us):
        """STARTED anchors this fresh session; statistics belong to allocation."""
        self._clock = MillisUnwrapper()
        self._clock.update(start_ms)
        self._last_row_us = start_us
        self._elapsed_row_us = 0
        self.owned = True

    def break_continuity(self, reason, before_seq=None):
        self._flow_hist.clear()
        events = []
        if self._break_reason == reason:
            return events
        self._break_reason = reason
        self.segment_id += 1
        if len(self.boundaries) < self.MAX_BOUNDARIES:
            self.boundaries.append(
                dict(after_seq=self.last_seq, before_seq=before_seq, reason=reason)
            )
        else:
            self.boundaries_omitted += 1
        return events

    def finish(self):
        return []

    def push(self, w, active=True, host_unix=None):
        clock = (
            self._clock.last_ms,
            self._clock.elapsed_ms,
            self._clock.reset_count,
            self._last_row_us,
            self._elapsed_row_us,
        )
        try:
            return self._push(w, active, host_unix)
        except (CaptureDiscontinuity, BoardTimeReset):
            (
                self._clock.last_ms,
                self._clock.elapsed_ms,
                self._clock.reset_count,
                self._last_row_us,
                self._elapsed_row_us,
            ) = clock
            raise

    def _push(self, w, active=True, host_unix=None):
        previous = self.last_sample
        if self.owned and previous is not None:
            for field in ("seq", "scan_id", "missed_slots_total", "tx_dropped_total"):
                value, prior = w[field], getattr(previous, field)
                if value is None or prior is None:
                    continue
                delta = (value - prior) & 0xFFFFFFFF
                if delta >= 0x80000000 or (field in ("seq", "scan_id") and delta == 0):
                    raise CaptureDiscontinuity(field + " repeated or regressed")
            if w["scan_id"] is not None and previous.scan_id is not None:
                if (w["scan_id"] - previous.scan_id) & 0xFFFFFFFF != (
                    w["seq"] - previous.seq
                ) & 0xFFFFFFFF:
                    raise CaptureDiscontinuity("scan and sequence increments disagree")
        if (
            self.owned
            and previous is not None
            and previous.phase in (0, 1)
            and w["phase"] in (0, 1)
        ):
            old = previous.cycle_id * 2 + previous.phase
            new = w["cycle_id"] * 2 + w["phase"]
            if (new - old) & 0x1FFFFFFFF > 0xFFFFFFFF:
                raise CaptureDiscontinuity("commanded phase regressed")
        elapsed_s, reset = self._clock.update(w["ms"])
        if reset:
            raise BoardTimeReset("millis reset")
        delta = (w["row_us"] - self._last_row_us) & 0xFFFFFFFF
        if delta > 0x7FFFFFFF:
            raise BoardTimeReset("micros reset")
        self._elapsed_row_us += delta
        self._last_row_us = w["row_us"]
        if active and abs(self._elapsed_row_us / 1e6 - elapsed_s) > 0.003:
            raise BoardTimeReset("millis/micros disagree")
        row_time_us = self._elapsed_row_us
        flow_time_us = row_time_us - w["flow_age_us"] if w["flow_age_us"] >= 0 else row_time_us
        t_s = flow_time_us / 1e6
        previous = self.last_sample
        # Classify continuity before smoothing, using the same rule as the
        # standalone detector. The detector receives only normalized segments.
        reason = continuity_reason(
            previous,
            seq=w["seq"],
            scan_id=w["scan_id"],
            t_s=t_s,
            run_id=w["run_id"],
            missed_slots_total=w["missed_slots_total"],
            tx_dropped_total=w["tx_dropped_total"],
            flow_ok=w["flow_ok"],
            max_gap_us=self.max_gap_us,
        )
        if previous is not None:
            delta_seq = (w["seq"] - previous.seq) & 0xFFFFFFFF
            if 1 < delta_seq < 0x80000000:
                self.dropped_rows += delta_seq - 1
        elif self.owned:
            self.dropped_rows += w["seq"]
            if w["seq"]:
                reason = "missing_prefix"
        for field, counter in (
            ("missed_slots_total", "missed_acquisition_slots"),
            ("tx_dropped_total", "firmware_tx_drops"),
        ):
            value = w[field]
            prior = getattr(previous, field) if previous else None
            if value is not None:
                delta = value if prior is None else (value - prior) & 0xFFFFFFFF
                if prior is None or delta < 0x80000000:
                    setattr(self, counter, getattr(self, counter) + delta)
        events = self.break_continuity(reason, w["seq"]) if reason else []
        if reason is None:
            self._break_reason = None
        if w["flow_ok"]:
            self._flow_hist.append(w["flow_raw"])
        # Divide before summing so finite extreme inputs cannot poison a
        # running sum. The bounded window is also replayed with this order.
        average = (
            sum(value / len(self._flow_hist) for value in self._flow_hist)
            if self._flow_hist
            else 0.0
        )
        target, resolved = self.target_peak(w["target_peak_slm"])
        sample = Sample(
            **{
                key: w[key]
                for key in (
                    "seq",
                    "ms",
                    "flow_raw",
                    "flow_ok",
                    "flow_age_us",
                    "pa",
                    "temp",
                    "pok",
                    "pa_age_us",
                    "cmd_dac",
                    "feedback_dac",
                    "cmd_pct",
                    "fdbk_pct",
                    "target_flow_slm",
                    "phase",
                    "preset",
                    "clamped",
                    "run_id",
                    "scan_id",
                    "row_us",
                    "cycle_id",
                    "scan_start_us",
                    "missed_slots_total",
                    "tx_dropped_total",
                    "pot_raw_adc",
                    "calibration_position",
                )
            },
            t_s=t_s,
            host_unix=time.time() if host_unix is None else host_unix,
            flow_avg=average,
            target_peak_slm=target,
            row_time_us=row_time_us,
            flow_time_us=flow_time_us,
            pa_time_us=[row_time_us - a if a >= 0 else None for a in w["pa_age_us"]],
            segment_id=self.segment_id,
            discontinuity_reason=reason or "",
            target_reference_resolved=resolved,
            target_peak_local_lmin=w["target_peak_slm"]
        )
        self.last_seq = sample.seq
        self.last_sample = sample
        if w["flow_ok"]:
            self._break_reason = None
        return CaptureResult(sample, events)

    def target_peak(self, local_peak):
        return local_peak, False
