"""Single-owner recording storage and provenance, independent of serial/UI."""
import csv
import hashlib
import json
import os
import time
import traceback
from dataclasses import asdict
from pathlib import Path
from analysis import AnalysisSession, BoardTimeReset
from recording_schema import (SAMPLE_COLUMNS, PEAK_COLUMNS, SOFTWARE_FILES,
                              config_fingerprint, sample_record, event_record)

SCRIPT_DIR = str(Path(__file__).resolve().parent)
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, 'recordings')
PRESET_NAMES = {-2: 'demo-sine', -1: 'idle/hold', 0: 'preset 1',
                1: 'preset 2', 2: 'preset 3', 3: 'preset 4'}
POPULATIONS = ('count', 'discarded', 'target_passes', 'target_misses',
               'partial_excursions', 'warmup_peaks', 'reference_unresolved',
               'pressure_incomplete', 'timing_failures', 'qualified_peaks',
               'unqualified_peaks')

class RecordingError(Exception):
    def __init__(self, kind, cause):
        super().__init__(str(cause))
        self.kind = kind
        self.cause = cause

def open_unique_csv_pair(directory, base):
    """Create two recording files without ever truncating an old session."""
    os.makedirs(directory, exist_ok=True)
    for index in range(10000):
        suffix = "" if index == 0 else f"_{index:02d}"
        data_path = os.path.join(directory, base + suffix + ".csv")
        peak_path = os.path.join(directory, base + suffix + "_peaks.csv")
        data_file = None
        try:
            data_file = open(data_path, "x", newline="")
            try:
                peak_file = open(peak_path, "x", newline="")
            except FileExistsError:
                data_file.close()
                os.unlink(data_path)
                continue
            except Exception:
                data_file.close()
                os.unlink(data_path)
                raise
            return data_path, peak_path, data_file, peak_file
        except FileExistsError:
            continue
    raise OSError("could not allocate a unique recording filename")

def open_unique_session(directory, base):
    """Reserve all three related paths; clean up only paths this attempt owns."""
    os.makedirs(directory, exist_ok=True)
    for index in range(10000):
        name = base + (f'_{index:02d}' if index else '')
        paths = [os.path.join(directory, name + suffix) for suffix in ('.csv', '_peaks.csv', '_session.json')]
        files = []
        try:
            for path in paths:
                files.append(open(path, 'x', newline=''))
            return paths, files
        except Exception as exc:
            for path, stream in zip(paths, files):
                stream.close()
                os.unlink(path)
            if not isinstance(exc, FileExistsError):
                raise
    raise OSError('could not allocate unique recording names')

class Recording:
    """All methods run on the recording owner, except allocation before START."""
    def __init__(self, avg_window=1, config=None, gas_temperature_c=None,
                 absolute_pressure_mbar=None, expected_expiration_sign=None,
                 directory=None):
        self.analysis = AnalysisSession(avg_window, config, gas_temperature_c, absolute_pressure_mbar)
        self.expected_expiration_sign = expected_expiration_sign
        self.directory = directory
        self._record_file = self._peak_file = None
        self._record_writer = self._peak_writer = None
        self._record_path = self._peak_path = self._manifest_path = None
        self.manifest = None
        self.started = self.failed = False
        self.rows = self.bad_lines = self.orientation_mismatches = 0
        self._phase_first = self._phase_last = None
        self._completed_phases = set()

    @property
    def paths(self):
        return self._record_path, self._peak_path, self._manifest_path

    def allocate(self, run_id, mode, preset, metadata):
        if self._record_file is not None:
            return None
        base = f"{PRESET_NAMES.get(preset, 'unknown').replace('/', '-').replace(' ', '_')}_{time.strftime('%Y%m%d_%H%M%S')}"
        paths, files = (), ()
        try:
            paths, files = open_unique_session(self.directory or RECORDINGS_DIR, base)
            self._record_path, self._peak_path, self._manifest_path = paths
            self._record_file, self._peak_file, manifest_file = files
            manifest_file.close()
            self._record_writer = csv.DictWriter(self._record_file, fieldnames=SAMPLE_COLUMNS)
            self._peak_writer = csv.DictWriter(self._peak_file, fieldnames=PEAK_COLUMNS)
            self._record_writer.writeheader()
            self._peak_writer.writeheader()
            a = self.analysis
            self.manifest = dict(
                protocol_version=2, sample_schema=2, peak_schema=2, analysis_policy=2,
                run_id=run_id, mode=mode, preset=preset,
                detector_config=asdict(a.config), avg_window=a.avg_window,
                expected_expiration_sign=self.expected_expiration_sign,
                flow_reference=dict(gas_temperature_c=a.gas_temperature_c,
                                    absolute_pressure_mbar=a.absolute_pressure_mbar),
                software_hashes={name: hashlib.sha256(Path(SCRIPT_DIR, name).read_bytes()).hexdigest()
                                 for name in SOFTWARE_FILES},
                firmware=metadata, time_origin='STARTED read-completion-relative microseconds',
                start_ack=None, end_ack=None, boundaries=[], complete=False,
                tail_truncated=True, termination_reason='open', rows=0, missing_rows=0, invalid_rows=0)
            self.manifest['config_sha256'] = config_fingerprint(self.manifest)
            self._save_manifest()
        except Exception as exc:
            for stream in files:
                try:
                    stream.close()
                except OSError as close_error:
                    print(f'-> storage fault closing failed allocation: {close_error}')
            self._record_file = self._peak_file = None
            self._record_writer = self._peak_writer = None
            self._record_path = self._peak_path = self._manifest_path = None
            self.manifest = None
            print(f'-> recording fault allocating recording: {exc}')
            if paths:
                print(f'-> partial allocation files retained: {paths}')
            return None
        self.started = self.failed = False
        self.rows = self.bad_lines = self.orientation_mismatches = 0
        self._phase_first = self._phase_last = None
        self._completed_phases = set()
        return paths

    def start(self, start_ms, start_us):
        self.analysis.reset(start_ms, start_us)
        self.manifest['boundaries'] = self.analysis.boundaries
        self.manifest['start_ack'] = dict(millis=start_ms, micros=start_us)
        self._save_manifest()
        self.started = True

    def consume(self, wire, line, host_unix=None):
        if not self.started or self.failed:
            return None
        if wire['run_id'] != self.manifest['run_id'] or wire['preset'] != self.manifest['preset']:
            raise RecordingError('unexpected_session', 'sample identity mismatch')
        if wire['protocol'] != 2:
            raise RecordingError('legacy_row_in_v2_session', 'protocol mismatch')
        try:
            result = self.analysis.push(wire, host_unix=host_unix)
        except BoardTimeReset as exc:
            raise RecordingError('board_time_reset', exc) from exc
        except Exception as exc:
            traceback.print_exc()
            raise RecordingError('analysis_fault', exc) from exc
        sample = result.sample
        if sample.cycle_id is not None and sample.phase in (0, 1):
            ordinal = sample.cycle_id*2 + sample.phase
            if self._phase_first is None:
                self._phase_first = ordinal
            self._phase_last = ordinal
        try:
            self._record_writer.writerow(sample_record(sample, line))
            self.rows += 1
            if self.rows % 100 == 0:
                self._record_file.flush()
                self._save_manifest()
            self._write_events(result.events)
        except OSError:
            raise
        except RecordingError:
            raise
        except Exception as exc:
            traceback.print_exc()
            raise RecordingError('format_fault', exc) from exc
        return sample

    def malformed(self):
        if not self.started or self.failed:
            return
        self.bad_lines += 1
        try:
            events = self.analysis.break_continuity('malformed_data')
        except Exception as exc:
            traceback.print_exc()
            raise RecordingError('analysis_fault', exc) from exc
        self._write_events(events)

    def _write_events(self, events):
        for event in events:
            self._write_peak_row(event)

    def _write_peak_row(self, event):
        try:
            self._peak_writer.writerow(event_record(event))
        except OSError:
            raise
        except Exception as exc:
            traceback.print_exc()
            raise RecordingError('format_fault', exc) from exc
        self._peak_file.flush()
        if event['excursion_complete'] and event['sample'].cycle_id is not None:
            self._completed_phases.add(event['sample'].cycle_id*2+event['sample'].phase)
        if event['excursion_complete'] and self.expected_expiration_sign is not None:
            expected = 1 if self.expected_expiration_sign == 'positive' else -1
            if event['sample'].phase == 1:
                expected *= -1
            if event['direction'] != expected:
                self.orientation_mismatches += 1
                if self.orientation_mismatches == 1:
                    print('-> flow sign disagrees with configured phase orientation')
        result = 'unscored' if event['qualified_peak'] is None else str(event['qualified_peak'])
        print(f"-> peak {event['peak_idx']}: qualified={result}; reasons={','.join(event['reason_codes'])}")

    def _save_manifest(self):
        if self.manifest is None or self._manifest_path is None:
            return
        temporary = self._manifest_path + '.tmp'
        created = False
        try:
            with open(temporary, 'x') as stream:
                created = True
                json.dump(self.manifest, stream, indent=2, allow_nan=False)
                stream.write('\n')
            os.replace(temporary, self._manifest_path)
        finally:
            if created and os.path.exists(temporary):
                os.unlink(temporary)

    def _phase_summary(self):
        first, last = self._phase_first, self._phase_last
        if first is None or last is None:
            return dict(missing_interior=0, first_phase_partial=False, last_phase_partial=False)
        interior = max(0, last-first-1)
        completed = sum(first < phase < last for phase in self._completed_phases)
        return dict(first_ordinal=first, last_ordinal=last,
                    missing_interior=max(0, interior-completed),
                    first_phase_partial=first not in self._completed_phases,
                    last_phase_partial=last not in self._completed_phases)

    def finish(self, reason, tail_truncated=False, end_ack=None):
        if self._record_file is None:
            # A fault can arrive while final filesystem publication is blocked.
            # The owner processes that ordered correction before signalling done.
            if tail_truncated and self.manifest is not None:
                self.manifest.update(complete=False, tail_truncated=True, termination_reason=reason)
                try:
                    self._save_manifest()
                except OSError as exc:
                    print(f'-> storage fault correcting manifest: {exc}')
            return
        if end_ack is not None:
            self.manifest['end_ack'] = end_ack
            last_seq = self.analysis.last_seq
            tail_truncated |= (end_ack['final_enqueued_seq'] != (last_seq if last_seq is not None else -1)
                               or self.analysis.dropped_rows > 0 or self.bad_lines > 0
                               or end_ack['tx_dropped_total'] > 0)
        # Failed analysis/persistence is never invoked again during teardown.
        if not self.failed:
            try:
                events = self.analysis.finish()
            except Exception as exc:
                traceback.print_exc()
                self.failed = True
                reason = 'analysis_fault'
            else:
                try:
                    self._write_events(events)
                except (OSError, RecordingError) as exc:
                    self.failed = True
                    reason = exc.kind if isinstance(exc, RecordingError) else 'storage_fault'
                    print(f'-> {reason} finalizing recording: {exc}')
        counters = {name: getattr(self.analysis.detector, name) for name in POPULATIONS}
        for stream in (self._record_file, self._peak_file):
            try:
                stream.flush()
                stream.close()
            except OSError as exc:
                self.failed = True
                reason = 'storage_fault'
                print(f'-> storage fault closing recording: {exc}')
                try:
                    stream.close()
                except OSError:
                    pass
        self.manifest.update(complete=not (tail_truncated or self.failed),
                             tail_truncated=tail_truncated or self.failed, termination_reason=reason,
                             rows=self.rows, missing_rows=self.analysis.dropped_rows,
                             invalid_rows=self.bad_lines, orientation_mismatches=self.orientation_mismatches,
                             populations=counters, commanded_phases=self._phase_summary())
        try:
            self._save_manifest()
        except OSError as exc:
            self.failed = True
            self.manifest.update(complete=False, tail_truncated=True, termination_reason='storage_fault')
            print(f'-> storage fault publishing manifest: {exc}; original remains incomplete')
            raise RecordingError('storage_fault', exc) from exc
        finally:
            self._record_file = self._peak_file = None
            self._record_writer = self._peak_writer = None
            self.started = False
        print(f'-> recording populations (main totals exclude warmup/partial/unresolved): {counters}')
