"""Serial transport and owned-run control. Recording work has a separate owner."""
import os
import re
import secrets
import threading
import time
import traceback
import serial
from analysis import AnalysisSession, BoardTimeReset
from peak_detection import PeakDetectionConfig
from recording import Recording
from recording_schema import META_KEYS, parse_wire, uint32
from recording_worker import RecordingWorker
from plot_buffer import PlotBuffer

DEFAULT_RUN_SECONDS = 180.0
COMMANDS = {
    '1': "Preset 1", '2': "Preset 2", '3': "Preset 3", '4': "Preset 4",
    's': "Demo sine", 'S': "Demo sine",
    'h': "Hold / idle", 'H': "Hold / idle", '0': "Hold / idle",
    'x': "Calibration hold", 'X': "Calibration hold",
    'm': "Toggle Monitor block", 'M': "Toggle Monitor block",
    '?': "Renegotiate firmware readiness",
}

PRESET_KEYS = frozenset('1234')
SAFETY_OVERRIDE_KEYS = frozenset('Hh0Xx')
MOTION_OVERRIDE_KEYS = frozenset('Ss')

class SerialLink:
    def __init__(self, port, baud, avg_window, buffer_seconds=180.0,
                 sample_hz=100.0, peak_config=None, run_seconds=DEFAULT_RUN_SECONDS,
                 gas_temperature_c=None, absolute_pressure_mbar=None,
                 expected_expiration_sign=None, serial_instance=None, start_reader=True,
                 recording_directory=None, background_recording=True,
                 recording_capacity=256, recording_max_latency=1.5):
        self.recording = Recording(avg_window, peak_config, gas_temperature_c,
                                   absolute_pressure_mbar, expected_expiration_sign,
                                   recording_directory)
        self.display_analysis = AnalysisSession(avg_window, peak_config, gas_temperature_c,
                                                absolute_pressure_mbar)
        self.plots = PlotBuffer(buffer_seconds, sample_hz)
        self.background_recording = background_recording
        self.recording_capacity = recording_capacity
        self.recording_max_latency = recording_max_latency
        self.worker = None
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
        self._run_timer = None
        self._run_start_monotonic = None
        self._run_generation = 0
        self.state = 'IDLE'
        self.run_id = self.mode = None
        self.protocol_version = self.metadata = None
        self._meta_pending = self._meta_build = None
        self._meta_invalid = False
        self._heartbeat_enabled = False
        self._started_event = threading.Event()
        self._ended_event = threading.Event()
        self._end_received = False
        self._discard_on_finish = False
        self._rx = bytearray()
        self._rx_discard = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        if start_reader:
            self._thread.start()

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
        if ch not in COMMANDS:
            return False
        if ch == '?':
            with self._run_lock:
                if self.state in ('STARTING', 'RUNNING', 'STOPPING'):
                    print('-> finish the active run before renegotiating readiness')
                    return False
                self._ready_event.clear()
                self.protocol_version = None
                self.metadata = None
                self._meta_pending = None
                self._meta_build = None
                self._meta_invalid = False
                with self._serial_write_lock:
                    return self._write_bytes_locked(b'\n?')
        if ch in PRESET_KEYS:
            return self.begin_timed_run(int(ch)-1, ch)
        if ch in MOTION_OVERRIDE_KEYS:
            if self.run_in_progress() and not self.stop_run():
                return False
            return self.begin_timed_run(-2, ch)
        confirmed = self.state != 'FAULTED'
        if ch in SAFETY_OVERRIDE_KEYS and self.run_in_progress():
            confirmed = self.stop_run()
        # An unconfirmed protocol stop must not suppress an explicit operator
        # override. Resynchronize any partial frame before the legacy command.
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
                if self.worker is not None and self.state in ('STARTING', 'RUNNING', 'STOPPING'):
                    unhealthy = self.worker.health_error()
                    if unhealthy:
                        self._fault(unhealthy)
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
            # Unexpected analysis/format failures must retain a traceback and
            # disable host ownership; never silently kill only the reader.
            if not self._stop.is_set():
                traceback.print_exc()
                self._fault('analysis_fault', exc)
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
            self._process_marker(line)
        except OSError as exc:
            self._fault('storage_fault', exc)
        except Exception as exc:
            traceback.print_exc()
            self._fault('analysis_fault', exc)

    def _process_marker(self, line):
        parts = line[2:].split(',')
        kind = parts[0]
        if kind == 'BOOT':
            if self.run_in_progress():
                self._fault('board_reset')
            self.protocol_version = None
            self.metadata = None
            self._ready_event.clear()
        elif parts == ['PROTOCOL', '2']:
            if self.run_in_progress():
                self._fault('unexpected_handshake')
            self.protocol_version = 2
        elif kind == 'META_BEGIN' and len(parts) == 3 and parts[1] == '2':
            self._meta_build = parts[2]
            self._meta_pending = {}
            self._meta_invalid = False
        elif kind == 'META' and len(parts) == 4 and parts[1] == '2':
            if self._meta_pending is None or parts[2] in self._meta_pending:
                self._meta_invalid = True
            elif not all(re.fullmatch(r'[A-Za-z0-9_.+\-]+', x) for x in parts[2:]):
                self._meta_invalid = True
            else:
                self._meta_pending[parts[2]] = parts[3]
        elif kind == 'META_END' and len(parts) == 3:
            if (parts[1] == '2' and parts[2] == self._meta_build and
                    not self._meta_invalid and self._meta_pending is not None and
                    set(self._meta_pending) == META_KEYS):
                self.metadata = dict(self._meta_pending, build_id=self._meta_build)
            else:
                self.metadata = None
            self._meta_pending = None
        elif parts == ['READY_QUERY']:
            # Legacy is display-only. Timed acquisition checks v2 + metadata.
            if (self.protocol_version is None and self.state != 'FAULTED') or self.metadata is not None:
                with self._run_lock:
                    if self.protocol_version == 2 and self.metadata is not None:
                        self.connected = True
                        self.connection_error = None
                        if self.state == 'FAULTED' and (self.worker is None or self.worker.done.is_set()):
                            self.state = 'IDLE'
                    self._ready_event.set()
        elif kind == 'ERROR':
            print(f'-> firmware error: {line[2:]}')
        elif kind == 'STARTED' and len(parts) == 6 and parts[1] == '2':
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
                self.state = 'RUNNING'
                self._run_start_monotonic = time.monotonic()
                timer = threading.Timer(self.run_seconds, self._on_run_timeout, args=(self._run_generation,))
                timer.daemon = True
                self._run_timer = timer
                timer.start()
                self._started_event.set()
        elif kind == 'ENDED' and len(parts) == 6 and parts[1] == '2':
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
                if self.state != 'STOPPING':
                    self._stop_deadline = time.monotonic() + 2.0
                self.state = 'STOPPING'
                self._end_received = True
                end_ack = dict(final_enqueued_seq=final_seq, reason=parts[4], tx_dropped_total=dropped)
                self.worker.finish(parts[4], end_ack=end_ack)

    def _submit(self, operation, *args):
        if self.worker is not None and not self.worker.submit(operation, *args):
            if not self._end_received:
                self._fault(self.worker.health_error() or 'recording_closed')

    def _recording_finished(self, owner, paths):
        with self._run_lock:
            discard = self.run_id == owner and self._discard_on_finish
        if discard:
            self._discard_paths(paths)
        with self._run_lock:
            if self.run_id == owner and self.state == 'STOPPING' and self._end_received:
                self.state = 'IDLE'
                self._ended_event.set()
                print('-> firmware session end confirmed; recording finalized')

    def _handle_data_line(self, line):
        try:
            wire = parse_wire(line)
        except ValueError:
            self._count_bad_line()
            return
        with self._run_lock:
            if self.state == 'STARTING':
                return
            if self.state in ('RUNNING', 'STOPPING'):
                if not self._end_received:
                    self._submit(self.recording.consume, wire, line, time.time())
                return
        try:
            self.plots.append(self.display_analysis.push(wire, active=False).sample)
        except BoardTimeReset:
            self.display_analysis.reset(wire['ms'], wire['row_us'] or 0)

    def _count_bad_line(self):
        self.bad_lines += 1
        with self._run_lock:
            if self.state in ('RUNNING', 'STOPPING') and not self._end_received:
                self._submit(self.recording.malformed)

    def start_recording(self, preset_index):
        paths = self.recording.allocate(self.run_id, self.mode, preset_index, self.metadata)
        if paths is not None:
            self._end_received = False
            self._discard_on_finish = False
            owner = self.run_id
            self.worker = RecordingWorker(
                self.recording, self.plots.append,
                lambda reason, exc: self._fault(reason, exc, expected_run_id=owner),
                lambda: self._recording_finished(owner, paths),
                capacity=self.recording_capacity, max_latency=self.recording_max_latency,
                background=self.background_recording)
        return paths

    def _cancel_run_timer_locked(self):
        self._run_generation += 1
        if self._run_timer is not None:
            self._run_timer.cancel()
        self._run_timer = None
        self._run_start_monotonic = None

    def begin_timed_run(self, preset_index, command):
        with self._run_lock:
            if self.state != 'IDLE' or self.protocol_version != 2 or self.metadata is None:
                print('-> timed runs require idle state and a complete protocol-2 handshake')
                return False
            self.run_id = secrets.token_hex(8)
            self.mode = 'DEMO' if preset_index == -2 else f'P{preset_index}'
            self._run_generation += 1
            self.state = 'STARTING'
            self._started_event = threading.Event()
            self._ended_event = threading.Event()
            started_event = self._started_event
            owned_id = self.run_id
            deadline = time.monotonic() + 2.0
            if self.start_recording(preset_index) is None:
                self.state = 'IDLE'
                return False
            with self._serial_write_lock:
                self._heartbeat_enabled = True
                sent = self._write_bytes_locked(
                    f'@START,{self.run_id},{self.mode}\n'.encode('ascii'),
                    timeout=max(.001, deadline-time.monotonic()))
        # ACK processing takes the run lock; wait without holding control locks.
        if not sent or not started_event.wait(max(0, deadline-time.monotonic())):
            self._fault('start_unconfirmed', expected_run_id=owned_id)
            return False
        return True

    def _on_run_timeout(self, generation):
        self.stop_run(generation=generation)

    def stop_run(self, generation=None, discard=False):
        with self._run_lock:
            if generation is not None and generation != self._run_generation:
                return False
            if self.state not in ('STARTING', 'RUNNING', 'STOPPING'):
                return False
            owner = self.state != 'STOPPING'
            owned_id = self.run_id
            ended_event = self._ended_event
            self._discard_on_finish |= discard
            if owner:
                self.state = 'STOPPING'
                self._cancel_run_timer_locked()
                self._stop_deadline = time.monotonic()+2.0
            deadline = self._stop_deadline
            with self._serial_write_lock:
                self._heartbeat_enabled = False
                if owner:
                    sent = self._write_bytes_locked(
                        f'@STOP,{owned_id}\n'.encode('ascii'),
                        timeout=max(.001, deadline-time.monotonic()))
                else:
                    sent = True
        confirmed = sent and ended_event.wait(min(1.0, max(0, deadline-time.monotonic())))
        if not confirmed and owner and sent:
            with self._run_lock:
                if self.run_id == owned_id and self.state == 'STOPPING' and time.monotonic() < deadline:
                    with self._serial_write_lock:
                        sent = self._write_bytes_locked(
                            f'@STOP,{owned_id}\n'.encode('ascii'),
                            timeout=max(.001, deadline-time.monotonic()))
            confirmed = sent and ended_event.wait(max(0, deadline-time.monotonic()))
        elif not confirmed and sent:
            confirmed = ended_event.wait(max(0, deadline-time.monotonic()))
        if not confirmed:
            self._fault('stop_unconfirmed', expected_run_id=owned_id)
        return confirmed

    def _fault(self, reason, exc=None, expected_run_id=None):
        with self._run_lock:
            if expected_run_id is not None and (self.run_id != expected_run_id or self.state == 'IDLE'):
                return
            with self._serial_write_lock:
                self._heartbeat_enabled = False
            self._cancel_run_timer_locked()
            self.state = 'FAULTED'
            self.protocol_version = None
            self.metadata = None
            self._meta_pending = None
            self._meta_build = None
            self._meta_invalid = True
            self._ready_event.clear()
            if self.worker is not None:
                self.worker.finish(reason, True)
        print(f'-> {reason}: {exc or "stop unconfirmed; heartbeat disabled"}')

    def run_in_progress(self):
        with self._run_lock:
            return self.state in ('STARTING', 'RUNNING', 'STOPPING')

    def run_remaining(self):
        with self._run_lock:
            if self._run_start_monotonic is None:
                return None
            return max(0.0, self.run_seconds-(time.monotonic()-self._run_start_monotonic))

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

    def close(self):
        if self.run_in_progress():
            self.stop_run()
        self._stop.set()
        with self._serial_write_lock:
            self._heartbeat_enabled = False
            self.ser.close()
        if self._thread.ident is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
