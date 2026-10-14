"""Single-owner measurement saving. Analysis never runs during capture."""

import csv
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import time
from capture import CaptureSession, CaptureDiscontinuity, BoardTimeReset
from recording_schema import SAMPLE_COLUMNS, sample_record, SOFTWARE_FILES

SCRIPT_DIR = str(Path(__file__).resolve().parent)
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
PRESET_NAMES = {-1: "idle/hold", 0: "preset 1", 1: "preset 2", 2: "preset 3", 3: "preset 4"}


@dataclass(frozen=True)
class CompletionResult:
    stop_confirmed: bool
    recording_complete: bool
    failure_reason: object = None
    quality_complete: bool = True
    planned_duration_reached: bool = False
    storage_complete: bool = True


class RecordingError(Exception):
    def __init__(self, kind, cause):
        super().__init__(str(cause))
        self.kind, self.cause = kind, cause


def save_json(path, value):
    """Publish one JSON file atomically; clean up only this writer's temporary."""
    import tempfile

    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", dir=path.parent, prefix=path.name + ".", suffix=".tmp", delete=False
        ) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class Recording:
    def __init__(
        self,
        avg_window=1,
        config=None,
        gas_temperature_c=None,
        absolute_pressure_mbar=None,
        expected_expiration_sign=None,
        directory=None,
        *,
        analysis_options=None,
    ):
        self.tracker = CaptureSession(avg_window)
        self.directory = directory or RECORDINGS_DIR
        # Store requested analysis settings without loading or validating a detector.
        from dataclasses import asdict

        self.analysis_options = analysis_options or dict(
            config=asdict(config) if config is not None else {},
            avg_window=avg_window,
            gas_temperature_c=gas_temperature_c,
            absolute_pressure_mbar=absolute_pressure_mbar,
            expected_expiration_sign=expected_expiration_sign,
        )
        try:
            json.dumps(self.analysis_options, allow_nan=False)
        except (TypeError, ValueError) as exc:
            self.analysis_options = dict(configuration_error=str(exc))
        self._record_file = self._record_writer = None
        self._record_path = self._manifest_path = None
        self.manifest = None
        self.started = self.failed = False
        self.failure_reason = None
        self.storage_failed = False
        self.last_saved_seq = None
        self.rows = self.bad_lines = 0
        self.invalid_flow_rows = 0
        self.invalid_pressure_rows = [0] * 7

    @property
    def paths(self):
        # Measurement CSV and run summary; analysis owns all derived paths.
        return self._record_path, self._manifest_path

    def allocate(self, run_id, mode, preset, metadata):
        if self._record_file is not None:
            return self.paths
        if preset not in range(4):
            return None
        os.makedirs(self.directory, exist_ok=True)
        base = f'preset_{preset+1}_{time.strftime("%Y%m%d_%H%M%S")}'
        owned = []
        try:
            for index in range(10000):
                name = base + (f"_{index:02d}" if index else "")
                data = Path(self.directory) / (name + ".csv")
                summary = Path(self.directory) / (name + "_session.json")
                try:
                    stream = open(data, "x", newline="")
                except FileExistsError:
                    continue
                try:
                    marker = open(summary, "x")
                    marker.close()
                except FileExistsError:
                    stream.close()
                    data.unlink()
                    continue
                except Exception:
                    stream.close()
                    data.unlink()
                    raise
                owned = [data, summary]
                self._record_file = stream
                self._record_path, self._manifest_path = map(str, owned)
                break
            else:
                raise OSError("could not allocate unique recording names")
            self._record_writer = csv.DictWriter(self._record_file, fieldnames=SAMPLE_COLUMNS)
            self._record_writer.writeheader()
            self.manifest = dict(
                protocol_version=3,
                sample_schema=4,
                run_id=run_id,
                mode=mode,
                preset=preset,
                measurements=Path(self._record_path).name,
                firmware=metadata,
                software_hashes={
                    name: hashlib.sha256(Path(SCRIPT_DIR, name).read_bytes()).hexdigest()
                    for name in SOFTWARE_FILES
                },
                analysis_options=self.analysis_options,
                analysis_result=name + "_analysis.json",
                start_ack=None,
                end_ack=None,
                complete=False,
                storage_complete=False,
                quality_complete=False,
                planned_duration_reached=False,
                rows=0,
                termination_reason="open",
                failure_reason=None,
                tail_truncated=True,
                invalid_rows=0,
                missing_rows=0,
                missed_acquisition_slots=0,
                firmware_tx_drops=0,
                invalid_flow_rows=0,
                invalid_pressure_rows=[0] * 7,
                boundaries=[],
                boundaries_omitted=0,
            )
            self._save_manifest()
            return self.paths
        except Exception as exc:
            if self._record_file is not None:
                try:
                    self._record_file.close()
                except OSError:
                    pass
            self._record_file = self._record_writer = None
            print(f"-> recording allocation failed: {exc}; retained paths: {owned}")
            return None

    def _save_manifest(self):
        save_json(self._manifest_path, self.manifest)

    def start(self, start_ms, start_us):
        self.tracker.set_origin(start_ms, start_us)
        self.manifest["start_ack"] = dict(millis=start_ms, micros=start_us)
        self._save_manifest()
        self.started = True

    def consume(self, wire, line, host_unix=None):
        if not self.started or self.failed:
            return None
        if (
            wire["run_id"] != self.manifest["run_id"]
            or wire["preset"] != self.manifest["preset"]
            or wire["protocol"] != 3
        ):
            raise RecordingError("unexpected_session", "sample identity/protocol mismatch")
        tracker = self.tracker
        previous_seq = tracker.last_seq
        try:
            sample = tracker.push(wire, host_unix=host_unix).sample
        except (CaptureDiscontinuity, BoardTimeReset):
            self.malformed()
            return None
        missing_before = (
            wire["seq"] if previous_seq is None else ((wire["seq"] - previous_seq) & 0xFFFFFFFF) - 1
        )
        self._record_writer.writerow(
            dict(sample_record(sample, line), missing_before=missing_before)
        )
        self.rows += 1
        self.last_saved_seq = sample.seq
        self.invalid_flow_rows += not sample.flow_ok
        for channel, valid in enumerate(sample.pok):
            self.invalid_pressure_rows[channel] += not valid
        if self.rows % 100 == 0:
            self._record_file.flush()
            self._save_manifest()
        return sample

    def malformed(self):
        if self.started and not self.failed:
            self.bad_lines += 1
            self.tracker.break_continuity("malformed_data")

    def mark_failure(self, reason):
        self.failed = True
        self.storage_failed |= reason == "storage_fault"
        self.failure_reason = self.failure_reason or reason

    def finish(self, reason, tail_truncated=False, end_ack=None, planned_duration_reached=False):
        if end_ack is not None:
            self.manifest["end_ack"] = end_ack
        ack = self.manifest.get("end_ack")
        if tail_truncated:
            self.mark_failure(reason)
        if self._record_file is not None:
            try:
                self._record_file.flush()
                self._record_file.close()
            except Exception:
                self.mark_failure("storage_fault")
                try:
                    self._record_file.close()
                except Exception:
                    pass
            self._record_file = self._record_writer = None
        t = self.tracker
        tail_missing = ack is None or ack["final_enqueued_seq"] != (
            self.last_saved_seq if self.last_saved_seq is not None else -1
        )
        drops = t.firmware_tx_drops
        if ack is not None:
            prior = t.last_sample.tx_dropped_total if t.last_sample else 0
            delta = (ack["tx_dropped_total"] - (prior or 0)) & 0xFFFFFFFF
            if delta >= 0x80000000:
                self.mark_failure("capture_discontinuity")
            else:
                drops += delta
        if self.manifest["start_ack"] is None:
            self.mark_failure("start_unconfirmed")
        if ack is None:
            self.mark_failure("stop_unconfirmed")
        quality = not bool(
            tail_missing
            or t.dropped_rows
            or t.missed_acquisition_slots
            or self.bad_lines
            or drops
            or self.invalid_flow_rows
            or any(self.invalid_pressure_rows)
        )
        storage = not self.storage_failed
        complete = not self.failed
        self.manifest.update(
            complete=complete,
            storage_complete=storage,
            quality_complete=quality,
            planned_duration_reached=planned_duration_reached or reason == "planned_duration",
            stop_confirmed=ack is not None,
            tail_truncated=bool(tail_missing or tail_truncated),
            termination_reason=self.failure_reason or reason,
            failure_reason=self.failure_reason,
            rows=self.rows,
            missing_rows=t.dropped_rows,
            missed_acquisition_slots=t.missed_acquisition_slots,
            firmware_tx_drops=drops,
            invalid_rows=self.bad_lines,
            invalid_flow_rows=self.invalid_flow_rows,
            invalid_pressure_rows=self.invalid_pressure_rows,
            boundaries=t.boundaries,
            boundaries_omitted=t.boundaries_omitted,
            final_segment_id=t.segment_id,
        )
        try:
            self._save_manifest()
        except Exception:
            self.mark_failure("storage_fault")
            self.manifest.update(
                complete=False,
                storage_complete=False,
                failure_reason=self.failure_reason,
                termination_reason=self.failure_reason,
            )
        self.started = False
        return CompletionResult(
            ack is not None,
            self.manifest["complete"],
            self.failure_reason,
            quality,
            self.manifest["planned_duration_reached"],
            self.manifest["storage_complete"],
        )
