"""Preset capture/analysis contracts through serial, files and replay interfaces."""

import csv
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import builtins
from test_recording import FakeSerial, row
from serial_link import SerialLink


class ProductionSerial(FakeSerial):
    def __init__(self):
        super().__init__(role="production")
        self.preset = 0
        self.disconnect = False
        self.stop_ack = True

    def write(self, data):
        if data.startswith(b"@START,"):
            self.preset = int(data.decode().strip().split(",")[-1][1:])
        if data.startswith(b"@STOP,") and not self.stop_ack:
            self.writes.append(data)
            return len(data)
        return super().write(data)

    def read(self, size):
        if self.disconnect:
            raise OSError("synthetic disconnect")
        return super().read(size)

    def sample(self, *, seq=None, invalid=False):
        self.seq = self.seq + 1 if seq is None else seq
        fields = row(self.seq, preset=self.preset, position="none", run_id=self.run_id).split(",")
        if invalid:
            fields[4], fields[5] = "0", "-1"
            fields[16], fields[23] = "0", "-1"
        self.emit(",".join(fields))


class FaultFile:
    def __init__(self, stream, fail):
        self.stream, self.fail = stream, fail

    def __getattr__(self, name):
        return getattr(self.stream, name)

    def write(self, text):
        if self.fail.is_set():
            raise OSError("synthetic disk full")
        return self.stream.write(text)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


class RefinementTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.fake = ProductionSerial()
        self.link = SerialLink(
            None,
            460800,
            1,
            serial_instance=self.fake,
            run_seconds=None,
            recording_directory=self.tmp.name,
        )
        self.addCleanup(self.link.close)
        self.assertTrue(self.link.wait_until_ready(1))

    def wait(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(predicate())

    def start(self, preset=0):
        self.assertTrue(self.link.begin_timed_run(preset))
        self.wait(lambda: self.link.recording.started)

    def sample(self, **kw):
        before = self.link.recording.rows
        self.fake.sample(**kw)
        self.wait(lambda: self.link.recording.rows > before)

    def rows(self):
        with open(self.link.recording.paths[0]) as stream:
            return list(csv.DictReader(stream))

    def test_storage_failure_stops_and_keeps_saved_measurements(self):
        fail = threading.Event()
        original = builtins.open

        def open_storage(path, *a, **kw):
            stream = original(path, *a, **kw)
            if str(path).endswith(".csv") and not str(path).endswith("_peaks.csv"):
                return FaultFile(stream, fail)
            return stream

        with patch("builtins.open", side_effect=open_storage):
            self.start()
            self.sample()
            fail.set()
            self.fake.sample()
            self.wait(self.link.run.finalized.is_set)
        self.assertTrue(self.link.worker.join(1))
        self.assertEqual(len(self.rows()), 1)
        self.assertTrue(any(w.startswith(b"@STOP,") for w in self.fake.writes))
        self.assertTrue(self.link.last_result.stop_confirmed)
        self.assertEqual(self.link.last_result.failure_reason, "storage_fault")

    def test_capture_does_not_allocate_peak_output(self):
        self.start()
        self.sample()
        self.assertFalse(list(Path(self.tmp.name).glob("*_peaks.csv")))
        self.assertTrue(self.link.stop_run(timeout=0.3))
        self.assertEqual(len(self.rows()), 1)

    def test_planned_duration_survives_an_unconfirmed_stop(self):
        self.fake.stop_ack = False
        self.link.run_seconds = 0.03
        self.start()
        self.sample()
        self.assertTrue(self.link.run.finalized.wait(3))
        result = self.link.last_result
        self.assertTrue(result.planned_duration_reached)
        self.assertFalse(result.stop_confirmed)
        self.assertTrue(result.storage_complete)
        summary = json.loads(Path(self.link.recording.paths[1]).read_text())
        self.assertTrue(summary["planned_duration_reached"])

    def test_analysis_launch_failure_does_not_change_capture_completion(self):
        self.start()
        self.sample()
        original_start = threading.Thread.start

        def start_thread(thread):
            if thread.name == "analysis":
                raise RuntimeError("synthetic analysis launch failure")
            return original_start(thread)

        with patch.object(threading.Thread, "start", start_thread):
            self.assertTrue(self.link.stop_run(timeout=0.3))
        self.assertTrue(self.link.worker.join(1))
        self.assertTrue(self.link.last_result.storage_complete)
        self.assertTrue(self.link.last_result.stop_confirmed)
        self.assertEqual(len(self.rows()), 1)
        self.assertEqual(self.link.wait_for_analysis()["status"], "failed")
        summary = json.loads(Path(self.link.recording.paths[1]).read_text())
        result = json.loads((Path(self.tmp.name) / summary["analysis_result"]).read_text())
        self.assertEqual(result["status"], "failed")
        self.assertIn("synthetic analysis launch failure", result["error"])

    def test_saved_run_is_analyzed_automatically(self):
        self.start()
        self.sample()
        self.assertTrue(self.link.stop_run(timeout=0.3))
        result = self.link.wait_for_analysis(5)
        self.assertEqual(result["status"], "complete")
        self.assertTrue(Path(result["peaks"]).is_file())
        self.assertEqual(len(self.rows()), 1)

    def test_analysis_failure_can_be_retried_without_changing_capture(self):
        import subprocess, sys, hashlib

        self.link.close()
        self.fake = ProductionSerial()
        self.link = SerialLink(
            None,
            460800,
            1,
            serial_instance=self.fake,
            run_seconds=None,
            recording_directory=self.tmp.name,
            analysis_options={"config": {"smooth_window": 2}, "avg_window": 1},
        )
        self.addCleanup(self.link.close)
        self.assertTrue(self.link.wait_until_ready(1))
        self.start()
        self.sample()
        self.assertTrue(self.link.stop_run(timeout=0.3))
        before = [
            Path(path).read_bytes()
            for path in (self.link.recording.paths[0], self.link.recording.paths[1])
        ]
        failed = self.link.wait_for_analysis(5)
        self.assertEqual(failed["status"], "failed")
        self.assertTrue(self.link.last_result.recording_complete)
        options = Path(self.tmp.name) / "retry.json"
        options.write_text(json.dumps({"config": {"smooth_window": 1, "min_samples": 1}}))
        result = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().parents[1] / "analysis_runner.py"),
                self.link.recording.paths[1],
                "--options",
                str(options),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        after = [
            Path(path).read_bytes()
            for path in (self.link.recording.paths[0], self.link.recording.paths[1])
        ]
        self.assertEqual(before, after)

    def test_timed_presets_repeat_with_sensor_faults_and_gaps(self):
        self.link.run_seconds = 0.12
        for preset in range(4):
            self.start(preset)
            self.sample(seq=0, invalid=True)
            self.sample(seq=3, invalid=True)
            self.wait(self.link.run.finalized.is_set)
            result = self.link.last_result
            self.assertTrue(result.planned_duration_reached)
            self.assertTrue(result.recording_complete)
            self.assertFalse(result.quality_complete)
            self.assertEqual(self.link.state, "IDLE")
            summary = json.loads(Path(self.link.recording.paths[1]).read_text())
            self.assertEqual(summary["missing_rows"], 2)
            self.assertEqual(summary["invalid_flow_rows"], 2)
            self.assertEqual(summary["invalid_pressure_rows"], [0, 0, 2, 0, 0, 0, 0])
            self.assertEqual(len(self.rows()), 2)

    def test_removed_workflows_are_rejected_without_faulting(self):
        for preset in (-3, -2):
            self.assertFalse(self.link.begin_timed_run(preset))
            self.assertEqual(self.link.state, "IDLE")
        for key in "sSXxmn":
            self.assertFalse(self.link.send_command(key))

    def test_disconnect_reports_saved_data_and_unconfirmed_stop_separately(self):
        self.start()
        self.sample()
        self.fake.disconnect = True
        self.wait(self.link.run.finalized.is_set)
        self.assertTrue(self.link.worker.join(1))
        result = self.link.last_result
        self.assertFalse(result.recording_complete)
        self.assertTrue(result.storage_complete)
        self.assertFalse(result.stop_confirmed)
        self.assertEqual(len(self.rows()), 1)
        self.assertTrue(any(w.startswith(b"@STOP,") for w in self.fake.writes))

    def test_corrupt_measurement_counters_do_not_stop_capture(self):
        self.start()
        self.sample(seq=0)
        self.fake.emit(row(0, preset=0, position="none", run_id=self.link.run_id))
        self.fake.sample(seq=2)
        self.wait(lambda: self.link.recording.rows == 2 or self.link.run.finalized.is_set())
        self.assertEqual(self.link.state, "RUNNING")
        self.assertTrue(self.link.stop_run(timeout=0.3))
        summary = json.loads(Path(self.link.recording.paths[1]).read_text())
        self.assertEqual(summary["invalid_rows"], 1)
        self.assertEqual(summary["missing_rows"], 1)
        self.assertFalse(self.link.last_result.quality_complete)
        self.assertEqual(len(self.rows()), 2)

    def test_incompatible_firmware_never_reports_ready(self):
        for version, role in ((2, "production"), (3, "calibration")):
            fake = FakeSerial(version, role)
            link = SerialLink(
                None,
                460800,
                1,
                serial_instance=fake,
                run_seconds=None,
                recording_directory=self.tmp.name,
            )
            try:
                self.assertFalse(link.wait_until_ready(0.05))
                self.assertFalse(link.begin_timed_run(0))
            finally:
                link.close()

    def test_saved_csv_replay_matches_the_established_peak_fixture(self):
        from recording import Recording
        from recording_schema import parse_wire
        from analysis_runner import analyze_recording
        from test_recording import metadata

        root = Path(__file__).resolve().parents[1]
        lines = (root / "tests/fixtures/production_data3.csv").read_text().splitlines()
        first = parse_wire(lines[0])
        rec = Recording(directory=self.tmp.name, gas_temperature_c=20, absolute_pressure_mbar=1013)
        rec.allocate(first["run_id"], "P0", first["preset"], metadata())
        rec.start(0, 0)
        for line in lines:
            rec.consume(parse_wire(line), line, host_unix=0)
        last = parse_wire(lines[-1])
        self.assertTrue(
            rec.finish(
                "host_stop", end_ack=dict(final_enqueued_seq=last["seq"], tx_dropped_total=0)
            ).recording_complete
        )
        result = analyze_recording(rec.paths[1])
        self.assertEqual(result["status"], "complete")
        with open(result["peaks"]) as stream:
            actual = list(csv.DictReader(stream))
        expected = json.loads((root / "tests/fixtures/production_peaks.json").read_text())
        self.assertEqual(
            actual, [{key: str(value) for key, value in event.items()} for event in expected]
        )

    def test_gap_is_explicit_even_when_flow_is_invalid(self):
        self.start()
        self.sample(seq=0, invalid=True)
        self.sample(seq=3, invalid=True)
        self.assertTrue(self.link.stop_run(timeout=0.3))
        self.assertEqual([r["missing_before"] for r in self.rows()], ["0", "2"])
