#!/usr/bin/env python3
"""Accelerated 30-minute preset captures with bounded memory and saved-file analysis."""
from pathlib import Path
import csv
import gc
import json
import math
import sys
import tempfile
import time
import tracemalloc

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "tests")]
from recording import Recording
from recording_schema import parse_wire
from plot_buffer import PlotBuffer
from analysis_runner import analyze_recording
from test_recording import row, metadata, RID


def soak(invalid=False, count=180000):
    started = time.monotonic()
    with tempfile.TemporaryDirectory() as directory:
        rec = Recording(directory=directory, gas_temperature_c=20, absolute_pressure_mbar=1013)
        rec.allocate(RID, "P0", 0, metadata())
        rec.start(0, 0)
        plots = PlotBuffer(buffer_seconds=2)
        tracemalloc.start()
        baseline = None
        try:
            for seq in range(count):
                fields = row(seq).split(",")
                phase = (seq // 200) % 2
                fields[3] = str(
                    (1 if phase == 0 else -1) * 10 * math.sin(math.pi * (seq % 200) / 200)
                )
                fields[33], fields[34], fields[40] = "10", str(phase), str(seq // 400)
                if invalid and seq % 2 == 0:
                    fields[4], fields[5] = "0", "-1"
                line = ",".join(fields)
                sample = rec.consume(parse_wire(line), line, host_unix=seq / 100)
                plots.append(sample)
                if seq == count // 2:
                    gc.collect()
                    baseline = tracemalloc.get_traced_memory()[0]
            result = rec.finish(
                "planned_duration", end_ack=dict(final_enqueued_seq=count - 1, tx_dropped_total=0)
            )
            gc.collect()
            current, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
            if rec.started:
                rec.finish("soak_failure", True)
        assert result.recording_complete, result
        assert rec.rows == count
        with open(rec.paths[0]) as stream:
            assert sum(1 for _ in csv.DictReader(stream)) == count
        assert len(plots.snapshot().t) == 200
        assert len(rec.tracker.boundaries) <= rec.tracker.MAX_BOUNDARIES
        assert current - baseline < 2_000_000, (baseline, current)
        analysis = analyze_recording(rec.paths[1])
        assert analysis["status"] == "complete", analysis
        if invalid:
            assert rec.tracker.boundaries_omitted > 0
            assert rec.manifest["invalid_flow_rows"] == count // 2
        else:
            assert analysis["populations"]["count"] == count // 200
            assert analysis["commanded_phases"]["missing_interior"] == 0
        print(
            json.dumps(
                dict(
                    invalid=invalid,
                    rows=count,
                    simulated_seconds=count / 100,
                    wall_seconds=round(time.monotonic() - started, 2),
                    retained_bytes=current,
                    peak_bytes=peak,
                    growth_second_half_bytes=current - baseline,
                    peaks=analysis["populations"]["count"],
                ),
                sort_keys=True,
            )
        )


if __name__ == "__main__":
    soak()
    soak(True)
