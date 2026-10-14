#!/usr/bin/env python3
"""Analyze a saved run; retry with: python analysis_runner.py RUN_session.json."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import tempfile
import os
from analysis import AnalysisSession
from peak_detection import PeakDetectionConfig
from recording import save_json
from recording_schema import parse_wire, event_record, PEAK_COLUMNS, config_fingerprint

POPULATIONS = (
    "count",
    "discarded",
    "target_passes",
    "target_misses",
    "partial_excursions",
    "warmup_peaks",
    "reference_unresolved",
    "pressure_incomplete",
    "timing_failures",
    "qualified_peaks",
    "unqualified_peaks",
)


class PhaseSummary:
    def __init__(self):
        self.first = self.last = self.last_completed = None
        self.completed = 0
        self.first_complete = False

    def sample(self, sample):
        if sample.phase not in (0, 1):
            return
        ordinal = sample.cycle_id * 2 + sample.phase
        if self.first is None:
            self.first = self.last = ordinal
        else:
            delta = (ordinal - self.last) & 0x1FFFFFFFF
            if delta > 0xFFFFFFFF:
                raise ValueError("commanded phase regressed")
            self.last += delta

    def event(self, event):
        sample = event["sample"]
        if event["excursion_complete"] and sample.cycle_id is not None:
            raw = sample.cycle_id * 2 + sample.phase
            ordinal = self.last - ((self.last - raw) & 0x1FFFFFFFF)
            if ordinal != self.last_completed:
                self.completed += 1
                self.last_completed = ordinal
                self.first_complete |= ordinal == self.first

    def result(self):
        if self.first is None:
            return dict(missing_interior=0, first_phase_partial=False, last_phase_partial=False)
        last_complete = self.last == self.last_completed
        completed = (
            self.completed - self.first_complete - (last_complete if self.last != self.first else 0)
        )
        return dict(
            first_ordinal=self.first,
            last_ordinal=self.last,
            missing_interior=max(0, max(0, self.last - self.first - 1) - completed),
            first_phase_partial=not self.first_complete,
            last_phase_partial=not last_complete,
        )


def file_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def analyze_recording(summary_path, overrides=None):
    summary_path = Path(summary_path).resolve()
    # Capture and derived outputs never share a writable file.
    summary = json.loads(summary_path.read_text())
    if summary.get("sample_schema") != 4:
        raise ValueError("use a recording from the matching updated logger")
    source = summary_path.parent / summary["measurements"]
    result_path = summary_path.parent / summary["analysis_result"]
    peaks_path = summary_path.with_name(
        summary_path.name.removesuffix("_session.json") + "_peaks.csv"
    )
    result = dict(analysis_schema=1, status="failed", source=str(source), peaks=None, error=None)
    temporary = None
    try:
        options = dict(summary["analysis_options"])
        if overrides:
            options.update(overrides)
            options.pop("configuration_error", None)
        if options.get("configuration_error"):
            raise ValueError(options["configuration_error"])
        config = PeakDetectionConfig(**options.get("config", {}))
        session = AnalysisSession(
            options.get("avg_window", 1),
            config,
            options.get("gas_temperature_c"),
            options.get("absolute_pressure_mbar"),
        )
        origin = summary["start_ack"]
        if origin is None:
            raise ValueError("recording has no acknowledged time origin")
        session.set_origin(origin["millis"], origin["micros"])
        phases = PhaseSummary()
        last_segment = 0
        orientation_mismatches = 0
        expected = options.get("expected_expiration_sign")

        def events(writer, items):
            nonlocal orientation_mismatches
            for event in items:
                writer.writerow(event_record(event))
                phases.event(event)
                if expected and event["excursion_complete"]:
                    direction = 1 if expected == "positive" else -1
                    if event["sample"].phase == 1:
                        direction *= -1
                    orientation_mismatches += event["direction"] != direction

        with tempfile.NamedTemporaryFile(
            mode="w",
            newline="",
            dir=peaks_path.parent,
            prefix=peaks_path.name + ".",
            suffix=".tmp",
            delete=False,
        ) as out:
            temporary = Path(out.name)
            writer = csv.DictWriter(out, fieldnames=PEAK_COLUMNS)
            writer.writeheader()
            with source.open(newline="") as stream:
                for row in csv.DictReader(stream):
                    saved_segment = int(row["segment_id"])
                    if saved_segment != last_segment and not row["discontinuity_reason"]:
                        events(writer, session.break_continuity("malformed_data"))
                    normalized = session.push(
                        parse_wire(row["wire_line"]), host_unix=float(row["host_unix"])
                    )
                    phases.sample(normalized.sample)
                    events(writer, normalized.events)
                    last_segment = saved_segment
            if summary.get("final_segment_id", last_segment) > last_segment:
                events(writer, session.break_continuity("malformed_data"))
            events(writer, session.finish())
            out.flush()
        result.update(
            status="complete",
            peaks=str(peaks_path),
            source_sha256=file_digest(source),
            populations={name: getattr(session.detector, name) for name in POPULATIONS},
            commanded_phases=phases.result(),
            orientation_mismatches=orientation_mismatches,
            analysis_policy=2,
            detector_config=asdict(config),
            avg_window=session.avg_window,
            expected_expiration_sign=expected,
            flow_reference=dict(
                gas_temperature_c=session.gas_temperature_c,
                absolute_pressure_mbar=session.absolute_pressure_mbar,
            ),
        )
        result["config_sha256"] = config_fingerprint(result)
        os.replace(temporary, peaks_path)
        save_json(result_path, result)
    except Exception as exc:
        result.update(status="failed", error=f"{type(exc).__name__}: {exc}", peaks=None)
        save_json(result_path, result)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument(
        "--options", type=Path, help="JSON analysis settings to override saved options on retry"
    )
    args = parser.parse_args()
    try:
        overrides = json.loads(args.options.read_text()) if args.options else None
        result = analyze_recording(args.summary, overrides)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Analysis failed: {exc}; measurements are unchanged.\n")
    print(f"Analysis {result['status']}: {result['error'] or result['peaks']}")
    return 0 if result["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
