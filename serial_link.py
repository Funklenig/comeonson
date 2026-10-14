"""Serial transport and owned-run control. Recording work has a separate owner."""
import os
import re
import secrets
import threading
import time
import traceback
from dataclasses import dataclass, field
import serial
from capture import CaptureSession, BoardTimeReset
from recording import Recording, CompletionResult
from recording_schema import valid_metadata, parse_wire, uint32
from recording_worker import RecordingWorker
from plot_buffer import PlotBuffer
from analysis_worker import AnalysisWorker, failed_job

DEFAULT_RUN_SECONDS = 180.0
COMMANDS = {'1': 'Preset 1', '2': 'Preset 2', '3': 'Preset 3', '4': 'Preset 4',
            'h': 'Hold / idle', 'H': 'Hold / idle', '0': 'Hold / idle',
            't': 'Toggle monitor', 'T': 'Toggle monitor', '?': 'Renegotiate readiness'}
PRESET_KEYS = frozenset('1234')
SAFETY_OVERRIDE_KEYS = frozenset('Hh0')

@dataclass
class RunContext:
    run_id: str
    mode: str
    recording: Recording
    worker: object = None
    timer: object = None
    start_monotonic: object = None
    start_deadline: float = 0
    stop_deadline: float = 0
    finalize_timeout: float = 2.0
    finalize_deadline: float = 0
    end_ack: object = None
    result: object = None
    failure_reason: object = None
    discard: bool = False
    fault_stop_attempted: bool = False
    analysis_job: object = None
    planned_duration_reached: bool = False
    started: threading.Event = field(default_factory=threading.Event)
    ended: threading.Event = field(default_factory=threading.Event)
    finalized: threading.Event = field(default_factory=threading.Event)


class IncompleteShutdownError(RuntimeError):
    pass


class SerialLink:
    def __init__(self, port, baud, avg_window, buffer_seconds=180.0,
                 sample_hz=100.0, peak_config=None, run_seconds=DEFAULT_RUN_SECONDS,
                 gas_temperature_c=None, absolute_pressure_mbar=None,
                 expected_expiration_sign=None, serial_instance=None, start_reader=True,
                 recording_directory=None, background_recording=True,
                 recording_capacity=256, recording_max_latency=1.5, analysis_options=None):
        self.analysis_options = analysis_options
        self._recording_args = (avg_window, peak_config, gas_temperature_c,
                                absolute_pressure_mbar, expected_expiration_sign, recording_directory)
        self._analysis_worker = None
        self.run = None
        self.last_result = None
        self.display_analysis = CaptureSession(avg_window)
        self.plots = PlotBuffer(buffer_seconds, sample_hz)
        self.background_recording = background_recording
        self.recording_capacity = recording_capacity
        self.recording_max_latency = recording_max_latency
        self.ser = serial_instance if serial_instance is not None else serial.Serial(port, baud, timeout=.2, write_timeout=1)
        self._serial_write_lock = threading.Lock()
        self._heartbeat_interval = .5
        self._last_heartbeat = 0
        self.connection_error = None
        self.connected = True
        self._ready_event = threading.Event()
        self._stop = threading.Event()
        self.bad_lines = 0
        self.run_seconds = run_seconds
        self._run_lock = threading.RLock()
        self._run_generation = 0
        self.state = 'IDLE'
        self.protocol_version = self.metadata = None
        self._meta_pending = self._meta_build = None
        self._meta_invalid = False
        self._heartbeat_enabled = False
        self._rx = bytearray()
        self._rx_discard = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        if start_reader:
            self._thread.start()

    @property
    def run_id(self):
        return self.run.run_id if self.run else None

    @property
    def mode(self):
        return self.run.mode if self.run else None

    @property
    def worker(self):
        return self.run.worker if self.run else None

    @property
    def recording(self):
        return self.run.recording if self.run else None

    def _transition(self, phase):
        """All control transitions occur while holding _run_lock."""
        allowed = {
            'IDLE': {'STARTING', 'FAULTED'},
            'STARTING': {'RUNNING', 'STOPPING', 'FINALIZING', 'FAULTED'},
            'RUNNING': {'STOPPING', 'FINALIZING', 'FAULTED'},
            'STOPPING': {'FINALIZING', 'FAULTED'},
            'FINALIZING': {'IDLE', 'FAULTED'},
            'FAULTED': {'IDLE', 'FAULTED'},
        }
        if phase != self.state and phase not in allowed[self.state]:
            raise RuntimeError(f'invalid run transition: {self.state} -> {phase}')
        self.state = phase

    def _write_bytes_locked(self, data, timeout=None):
        """Guard timeout configuration and writes; caller holds the write lock."""
        try:
            previous_timeout = getattr(self.ser, 'write_timeout', None)
            if timeout is not None:
                self.ser.write_timeout = timeout
            try:
                if self.ser.write(data) != len(data):
                    raise OSError('short serial write')
            finally:
                if timeout is not None:
                    self.ser.write_timeout = previous_timeout
            return True
        except (serial.SerialException, OSError, ValueError) as exc:
            self._heartbeat_enabled = False
            self.connection_error = str(exc)
            self.connected = False
            if not self._stop.is_set():
                print(f'-> transport fault: {exc}; stop unconfirmed')
            return False

    def send_command(self, ch):
        if self._stop.is_set() or ch not in COMMANDS:
            return False
        if ch == '?':
            with self._run_lock:
                if self.state in ('STARTING', 'RUNNING', 'STOPPING', 'FINALIZING'):
                    print('-> finish the active run before renegotiating readiness')
                    return False
                self._reset_handshake()
                with self._serial_write_lock:
                    return self._write_bytes_locked(b'\n?')
        confirmed = True
        if ch in SAFETY_OVERRIDE_KEYS:
            if self.run_in_progress():
                confirmed = self.stop_run()
        data = (('\n' if ch in SAFETY_OVERRIDE_KEYS else '') + ch).encode('ascii')
        with self._run_lock:
            owned_id = self.run_id if self.state in ('STARTING', 'RUNNING', 'STOPPING') else None
            with self._serial_write_lock:
                written = self._write_bytes_locked(data)
        if not written and owned_id is not None:
            self._fault('transport_fault', expected_run_id=owned_id)
        return written and confirmed

    def _send_heartbeat(self):
        with self._serial_write_lock:
            now = time.monotonic()
            if not self._heartbeat_enabled or now-self._last_heartbeat < self._heartbeat_interval:
                return True
            ok = self._write_bytes_locked(f'@PING,{self.run_id}\n'.encode('ascii'))
            if ok:
                self._last_heartbeat = now
            return ok

    def wait_until_ready(self, timeout):
        if not self.send_command('?'):
            return False
        return self._ready_event.wait(timeout)

    def _reader_loop(self):
        try:
            while not self._stop.is_set():
                with self._run_lock:
                    if self.worker is not None and self.state in ('STARTING', 'RUNNING', 'STOPPING', 'FINALIZING'):
                        if self.state == 'STOPPING' and self.run.failure_reason:
                            unhealthy = 'stop_unconfirmed' if time.monotonic() >= self.run.stop_deadline else None
                        elif self.state == 'FINALIZING':
                            unhealthy = ('incomplete_shutdown' if time.monotonic() >= self.run.finalize_deadline
                                         else None)
                        else:
                            unhealthy = self.worker.health_error()
                        if unhealthy:
                            self._fault(unhealthy, expected_run_id=self.run_id)
                if not self._send_heartbeat():
                    self._fault('transport_fault')
                    return
                try:
                    raw = self.ser.read(256)
                except (serial.SerialException, OSError) as exc:
                    if not self._stop.is_set():
                        self._fault('transport_fault', exc)
                    return
                if raw:
                    self.feed_bytes(raw)
        except Exception as exc:
            # Unexpected capture/format failures must retain a traceback and
            # disable host ownership; never silently kill only the reader.
            if not self._stop.is_set():
                traceback.print_exc()
                self._fault('capture_fault', exc)
        finally:
            self.connected = False

    def feed_bytes(self, raw):
        """Incremental bounded framing; timeout never turns fragments into rows."""
        for byte in raw:
            if byte == 10:
                if not self._rx_discard and self._rx:
                    try:
                        line = self._rx.decode('ascii').rstrip('\r')
                    except UnicodeDecodeError:
                        self._count_bad_line()
                    else:
                        self._handle_line(line)
                self._rx.clear()
                self._rx_discard = False
            elif not self._rx_discard:
                if len(self._rx) >= 1024:
                    self._rx.clear()
                    self._rx_discard = True
                    self._count_bad_line()
                else:
                    self._rx.append(byte)

    def _handle_line(self, line):
        if line.startswith('DATA'):
            self._handle_data_line(line)
        elif line.startswith('# '):
            self._handle_marker(line)

    def _handle_marker(self, line):
        try:
            with self._run_lock:
                self._process_marker(line)
        except OSError as exc:
            self._fault('storage_fault', exc)
        except Exception as exc:
            traceback.print_exc()
            self._fault('capture_fault', exc)

    def _reset_handshake(self):
        self._ready_event.clear()
        self.protocol_version = self.metadata = None
        self._meta_pending = self._meta_build = None
        self._meta_invalid = False

    def _process_marker(self, line):
        parts = line[2:].split(',')
        kind = parts[0]
        if kind == 'BOOT':
            if self.run_in_progress():
                self._fault('board_reset')
            self._reset_handshake()
        elif kind == 'PROTOCOL' and len(parts) == 2 and parts[1] == '3':
            if self.run_in_progress():
                self._fault('unexpected_handshake')
            self._reset_handshake()
            self.protocol_version = int(parts[1])
        elif kind == 'META_BEGIN' and len(parts) == 3 and parts[1] == str(self.protocol_version):
            self._meta_build = parts[2]
            self._meta_pending = {}
            self._meta_invalid = False
        elif kind == 'META' and len(parts) == 4 and parts[1] == str(self.protocol_version):
            if self._meta_pending is None or parts[2] in self._meta_pending:
                self._meta_invalid = True
            elif not all(re.fullmatch(r'[A-Za-z0-9_.+\-]+', x) for x in parts[2:]):
                self._meta_invalid = True
            else:
                self._meta_pending[parts[2]] = parts[3]
        elif kind == 'META_END' and len(parts) == 3:
            if (parts[1] == str(self.protocol_version) and parts[2] == self._meta_build and
                    not self._meta_invalid and self._meta_pending is not None and
                    valid_metadata(self.protocol_version, self._meta_pending)):
                self.metadata = dict(self._meta_pending, build_id=self._meta_build)
            else:
                self.metadata = None
            self._meta_pending = None
        elif parts == ['READY_QUERY']:
            if self.protocol_version==3 and self.metadata is not None and self.metadata['firmware_role']=='production':
                self.connected=True
                self.connection_error=None
                if self.state=='FAULTED' and (self.worker is None or self.worker.done.is_set()):
                    self._transition('IDLE')
                self._ready_event.set()
        elif kind == 'ERROR':
            print(f'-> firmware error: {line[2:]}')
        elif kind == 'STARTED' and len(parts) == 6 and parts[1] == str(self.protocol_version):
            if parts[2] != self.run_id:
                return
            try:
                start_ms, start_us = uint32(parts[4]), uint32(parts[5])
            except ValueError:
                self._fault('invalid_start_ack')
                return
            with self._run_lock:
                if self.state != 'STARTING' or parts[2] != self.run_id or parts[3] != self.mode:
                    return
                self.plots.clear()
                self._submit(self.recording.start, start_ms, start_us)
                if self.state == 'FAULTED':
                    return
                self._transition('RUNNING')
                self.run.start_monotonic = time.monotonic()
                if self.run_seconds is not None:
                    timer = threading.Timer(self.run_seconds, self._on_run_timeout, args=(self._run_generation,))
                    timer.daemon = True
                    self.run.timer = timer
                    timer.start()
                self.run.started.set()
        elif kind == 'ENDED' and len(parts) == 6 and parts[1] == str(self.protocol_version):
            if parts[2] != self.run_id:
                return
            try:
                final_seq = -1 if parts[3] == '-1' else uint32(parts[3])
                dropped = uint32(parts[5])
                if parts[4] not in ('host_stop', 'watchdog', 'manual_override', 'transport_fault'):
                    raise ValueError('unknown termination reason')
            except ValueError:
                self._fault('invalid_end_ack')
                return
            with self._run_lock:
                if self.state not in ('RUNNING', 'STOPPING', 'STARTING') or parts[2] != self.run_id:
                    return
                with self._serial_write_lock:
                    self._heartbeat_enabled = False
                self._cancel_run_timer_locked()
                self._transition('FINALIZING')
                end_ack = dict(final_enqueued_seq=final_seq, reason=parts[4], tx_dropped_total=dropped)
                self.run.end_ack = end_ack
                self.run.finalize_deadline = time.monotonic() + self.run.finalize_timeout
                self.run.ended.set()
                self.worker.finish(self.run.failure_reason or ('planned_duration' if self.run.planned_duration_reached and parts[4]=='host_stop' else parts[4]),
                                   bool(self.run.failure_reason), end_ack=end_ack,
                                   planned_duration_reached=self.run.planned_duration_reached)

    def _submit(self, operation, *args):
        if self.worker is not None and not self.worker.submit(operation, *args):
            if self.state != 'FINALIZING':
                self._fault(self.worker.health_error() or 'recording_closed')

    def _recording_finished(self, run, result):
        with self._run_lock:
            if self.run is not run:
                return
            run.result = self.last_result = result
            if self.state != 'FAULTED':
                self._transition('IDLE' if result.recording_complete else 'FAULTED')
            try:
                if not run.discard and run.recording.rows:
                    if self._analysis_worker is None:
                        self._analysis_worker = AnalysisWorker()
                    run.analysis_job = self._analysis_worker.submit(run.recording.paths[1])
            except Exception as exc:
                run.analysis_job = failed_job(run.recording.paths[1], 'failed',
                                              f'{type(exc).__name__}: {exc}')
            finally:
                run.finalized.set()
        if run.discard:
            self._discard_paths(run.recording.paths)
        print(f'-> session result: {result}')
        return True

    def _handle_data_line(self, line):
        try:
            wire = parse_wire(line)
            if wire["protocol"] != 3 or wire["preset"] not in (-1,0,1,2,3):
                raise ValueError("unsupported recording protocol/mode")
        except ValueError:
            self._count_bad_line()
            return
        with self._run_lock:
            if self.state == 'STARTING':
                return
            if self.state in ('RUNNING', 'STOPPING'):
                if self.run.end_ack is None:
                    self._submit(self.recording.consume, wire, line, time.time())
                return
            if self.state in ('FINALIZING', 'FAULTED'):
                return
        try:
            self.plots.append(self.display_analysis.push(wire, active=False).sample)
        except BoardTimeReset:
            self.display_analysis.reset(wire['ms'], wire['row_us'] or 0)

    def _count_bad_line(self):
        self.bad_lines += 1
        with self._run_lock:
            if self.state in ('RUNNING', 'STOPPING') and self.run.end_ack is None:
                self._submit(self.recording.malformed)

    def _cancel_run_timer_locked(self):
        self._run_generation += 1
        if self.run is not None and self.run.timer is not None:
            self.run.timer.cancel()
            self.run.timer = None

    def begin_timed_run(self, preset_index, command=None):
        with self._run_lock:
            if self._stop.is_set() or not self.connected:
                return False
            if self.state != 'IDLE' or self.protocol_version != 3 or self.metadata is None or self.metadata['firmware_role']!='production':
                print('-> runs require idle state and a matching production protocol-3 handshake')
                return False
            if preset_index not in range(4):
                return False
            if self.worker is not None and not self.worker.done.is_set():
                return False
            mode = f'P{preset_index}'
            recording = Recording(*self._recording_args, analysis_options=self.analysis_options)
            run = RunContext(secrets.token_hex(8), mode, recording)
            self.run = run
            self.last_result = None
            self._run_generation += 1
            self._transition('STARTING')
            run.start_deadline = time.monotonic() + 2.0
            paths = recording.allocate(run.run_id, mode, preset_index, self.metadata)
            if paths is None:
                run.result = self.last_result = CompletionResult(False, False, 'allocation_fault', storage_complete=False)
                self._transition('FAULTED')
                run.finalized.set()
                return False
            run.worker = RecordingWorker(
                recording, lambda sample: self._plot_sample(run, sample),
                lambda reason, exc: self._fault(reason, exc, expected_run_id=run.run_id),
                lambda result: self._recording_finished(run, result),
                capacity=self.recording_capacity, max_latency=self.recording_max_latency,
                background=self.background_recording, completion_lock=self._run_lock)
            with self._serial_write_lock:
                self._heartbeat_enabled = True
                sent = self._write_bytes_locked(
                    f'@START,{run.run_id},{run.mode}\n'.encode('ascii'),
                    timeout=max(.001, run.start_deadline-time.monotonic()))
        if not sent or not run.started.wait(max(0, run.start_deadline-time.monotonic())):
            self._fault('start_unconfirmed', expected_run_id=run.run_id)
            return False
        return self.state == 'RUNNING'

    def _plot_sample(self, run, sample):
        if self.run is run:
            self.plots.append(sample)

    def _on_run_timeout(self, generation):
        with self._run_lock:
            if generation != self._run_generation or self.state != 'RUNNING':
                return
            self.run.planned_duration_reached = True
        self.stop_run(generation=generation)

    def stop_run(self, generation=None, discard=False, timeout=2.0):
        return self._stop_run(generation, discard, timeout)

    def _stop_run(self, generation=None, discard=False, timeout=2.0, shutdown_deadline=None):
        with self._run_lock:
            if generation is not None and generation != self._run_generation:
                return False
            run = self.run
            if run is None or self.state not in ('STARTING', 'RUNNING', 'STOPPING', 'FINALIZING'):
                return bool(run and run.result and run.result.recording_complete)
            run.discard |= discard
            owner = self.state in ('STARTING', 'RUNNING')
            if owner:
                self._transition('STOPPING')
                self._cancel_run_timer_locked()
                run.stop_deadline = time.monotonic() + timeout
                run.finalize_timeout = timeout
                with self._serial_write_lock:
                    self._heartbeat_enabled = False
                    sent = self._write_bytes_locked(f'@STOP,{run.run_id}\n'.encode('ascii'), timeout=timeout)
                if not sent:
                    self._fault('transport_fault', expected_run_id=run.run_id)
            deadline = run.stop_deadline or (time.monotonic()+timeout)
        if shutdown_deadline is not None:
            deadline = min(deadline, shutdown_deadline)
        # ENDED confirms the actuator separately from the filesystem barrier.
        if not run.ended.wait(max(0, deadline-time.monotonic())):
            self._fault('stop_unconfirmed', expected_run_id=run.run_id)
        deadline = run.finalize_deadline or (time.monotonic()+timeout)
        if shutdown_deadline is not None:
            deadline = min(deadline, shutdown_deadline)
        if not run.finalized.wait(max(0, deadline-time.monotonic())):
            self._fault('incomplete_shutdown', expected_run_id=run.run_id)
        return bool(run.result and run.result.recording_complete)

    def _fault(self, reason, exc=None, expected_run_id=None):
        with self._run_lock:
            if expected_run_id is not None and (self.run_id != expected_run_id or self.state == 'IDLE'):
                return
            with self._serial_write_lock:
                self._heartbeat_enabled = False
            active_run = self.state != 'IDLE' and self.run is not None and not self.run.finalized.is_set()
            self._cancel_run_timer_locked()
            self._ready_event.clear()
            if active_run:
                run = self.run
                run.failure_reason = run.failure_reason or reason
                run.started.set()
                if not run.fault_stop_attempted and run.end_ack is None:
                    run.fault_stop_attempted = True
                    with self._serial_write_lock:
                        sent = self._write_bytes_locked(f'@STOP,{run.run_id}\n'.encode('ascii'), timeout=.2)
                    # A working reader can still acknowledge a storage failure.
                    if sent and reason not in ('transport_fault', 'board_reset', 'stop_unconfirmed'):
                        if self.state in ('STARTING', 'RUNNING'):
                            self._transition('STOPPING')
                        run.stop_deadline = time.monotonic() + .5
                        print(f'-> {reason}: {exc or "capture ended"}; awaiting STOP acknowledgement')
                        return False
                if self.worker is not None:
                    self.worker.finish(run.failure_reason, True, run.end_ack, run.planned_duration_reached)
            self._transition('FAULTED')
            if reason in ('transport_fault', 'board_reset'):
                self.connected = False
                self.connection_error = str(exc or reason)
        print(f'-> {reason}: {exc or "stop unconfirmed"}')


    def wait_for_analysis(self, timeout=5):
        job = self.run.analysis_job if self.run else None
        if job is None:
            return dict(status='not_started')
        if not job.done.wait(timeout):
            return dict(status='pending')
        return job.result

    def run_in_progress(self):
        with self._run_lock:
            return self.state in ('STARTING', 'RUNNING', 'STOPPING', 'FINALIZING')

    def run_remaining(self):
        with self._run_lock:
            if self.state != 'RUNNING' or self.run is None or self.run.start_monotonic is None or self.run_seconds is None:
                return None
            return max(0.0, self.run_seconds-(time.monotonic()-self.run.start_monotonic))

    def abort_run(self):
        return self.stop_run(discard=True)

    @staticmethod
    def _discard_paths(paths):
        for path in paths:
            if path is not None:
                try:
                    os.unlink(path)
                except FileNotFoundError:
                    pass
                except OSError as exc:
                    print(f'-> could not remove recording: {exc}')

    def close(self, timeout=5.0):
        deadline = time.monotonic()+timeout
        if self.run_in_progress():
            self._stop_run(timeout=min(2.0, timeout), shutdown_deadline=deadline)
        self._stop.set()
        with self._run_lock:
            self._cancel_run_timer_locked()
        with self._serial_write_lock:
            self._heartbeat_enabled = False
            self.ser.close()
        if self._thread.ident is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=max(0, deadline-time.monotonic()))
        if self.worker is not None:
            if not self.worker.done.is_set():
                self.worker.finish('incomplete_shutdown', True, self.run.end_ack, self.run.planned_duration_reached)
            if not self.worker.join(max(0, deadline-time.monotonic())):
                self._fault('incomplete_shutdown')
                raise IncompleteShutdownError('storage is still blocked; recording worker has not exited')
        if self._thread.is_alive():
            raise IncompleteShutdownError('serial reader has not exited')
        if self._analysis_worker is not None:
            self._analysis_worker.close(max(0, deadline-time.monotonic()))
        return self.last_result
