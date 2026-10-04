import csv
import json
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from pathlib import Path

from analysis import AnalysisSession
from recording import Recording
from recording_schema import parse_wire, META_KEYS, META_KEYS_V3
from recording_worker import RecordingWorker
from serial_link import SerialLink, IncompleteShutdownError

RID = '0123456789abcdef'


def row(seq=0, version=3, preset=-3, position='centre', run_id=RID, missed=0, ms=None):
    ms = (seq+1)*10 if ms is None else ms
    fields = [f'DATA{version}' if version > 1 else 'DATA', seq, ms, 12, 1, 0,
              *([1]*7), 20, *([1]*7), *([0]*7), 1434, 1434, 0, 0, 0, 0,
              -1 if preset < 0 else 0, preset, 0]
    if version >= 2:
        fields += [run_id, seq+1, (ms*1000) & 0xffffffff, 0, (ms*1000) & 0xffffffff, missed, 0]
    if version == 3:
        fields += [1414, position]
    return ','.join(map(str, fields))


def metadata(version=3, role='calibration'):
    if version == 2:
        return {key: '1' for key in META_KEYS}
    result = {key: '1' for key in META_KEYS_V3}
    result.update(firmware_role=role, cal_min_dac='1200', cal_max_dac='1800',
                  cal_centre_dac='1434', cal_reference_dac='1894')
    return result


class FakeSerial:
    write_timeout = 1
    def __init__(self, version=3, role='calibration'):
        self.rx = queue.Queue()
        self.writes = []
        self.version, self.role = version, role
        self.run_id = None
        self.seq = -1
        self.closed = False
        self.reject_start = False

    def emit(self, line):
        self.rx.put((line+'\n').encode())

    def read(self, size):
        try:
            return self.rx.get(timeout=.01)
        except queue.Empty:
            return b''

    def write(self, data):
        self.writes.append(data)
        if data == b'\n?':
            v = self.version
            self.emit(f'# PROTOCOL,{v}')
            self.emit(f'# META_BEGIN,{v},test')
            for key, value in metadata(v, self.role).items():
                self.emit(f'# META,{v},{key},{value}')
            self.emit(f'# META_END,{v},test')
            self.emit('# READY_QUERY')
        elif data.startswith(b'@START,') and not self.reject_start:
            _, self.run_id, mode = data.decode().strip().split(',')
            self.seq = -1
            self.emit(f'# STARTED,{self.version},{self.run_id},{mode},0,0')
        elif data.startswith(b'@STOP,'):
            self.emit(f'# ENDED,{self.version},{self.run_id},{self.seq},host_stop,0')
        elif data.decode() in ('H', 'X', 'm', 'n'):
            self.sample({'H': 'centre', 'X': 'reference', 'm': 'max', 'n': 'min'}[data.decode()])
        return len(data)

    def sample(self, position='centre', preset=-3):
        self.seq += 1
        self.emit(row(self.seq, self.version, preset, position, self.run_id))

    def close(self):
        self.closed = True


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def recording(self, preset=-3, version=3):
        rec = Recording(directory=self.tmp.name, score_peaks=preset != -3)
        rec.allocate(RID, 'CAL' if preset == -3 else 'P0', preset, metadata(version))
        rec.start(0, 0)
        self.addCleanup(lambda: rec.finish('cleanup', True) if rec.started else None)
        return rec

    def consume(self, rec, **kw):
        line = row(**kw)
        return rec.consume(parse_wire(line), line)

    def test_missing_prefix_and_slots_are_independent(self):
        rec = self.recording()
        self.consume(rec, seq=5, missed=2)
        self.consume(rec, seq=6, missed=3)
        result = rec.finish('host_stop', end_ack=dict(final_enqueued_seq=6, tx_dropped_total=0))
        self.assertTrue(result.stop_confirmed)
        self.assertFalse(result.recording_complete)
        self.assertEqual(rec.manifest['missing_rows'], 5)
        self.assertEqual(rec.manifest['missed_acquisition_slots'], 3)

    def test_consecutive_sequence_skipped_slot_is_incomplete(self):
        rec = self.recording()
        self.consume(rec, missed=1)
        result = rec.finish('host_stop', end_ack=dict(final_enqueued_seq=0, tx_dropped_total=0))
        self.assertFalse(result.recording_complete)
        self.assertEqual(rec.manifest['missing_rows'], 0)

    def test_wraps_do_not_add_loss(self):
        analysis = AnalysisSession()
        analysis.set_origin(0, 0)
        first = parse_wire(row(seq=0, ms=1))
        analysis.push(first)
        analysis.last_sample = analysis.last_sample._replace(seq=0xffffffff, scan_id=0xffffffff, missed_slots_total=0xffffffff)
        analysis.last_seq = 0xffffffff
        analysis.push(dict(parse_wire(row(seq=0, ms=2)), scan_id=0))
        self.assertEqual(analysis.dropped_rows, 0)
        self.assertEqual(analysis.missed_acquisition_slots, 1)

    def test_old_protocols_and_unowned_monitoring(self):
        for v in (1, 2):
            parsed = parse_wire(row(version=v, preset=-1, position='none'))
            self.assertIsNone(parsed['pot_raw_adc'])
        self.assertIsNone(parse_wire(row(run_id='-'))['run_id'])
        for invalid in (row().replace(',1414,centre', ',4096,centre'),
                        row(position='none'), row(version=2, preset=-3),
                        row(version=2, preset=0, run_id='-')):
            with self.assertRaises(ValueError):
                parse_wire(invalid)

    def test_complete_calibration_has_no_peaks(self):
        rec = self.recording()
        for seq, position in enumerate(('centre', 'reference', 'max', 'min', 'centre')):
            self.consume(rec, seq=seq, position=position)
        result = rec.finish('host_stop', end_ack=dict(final_enqueued_seq=4, tx_dropped_total=0))
        self.assertTrue(result.recording_complete)
        self.assertEqual(rec.analysis.detector.count, 0)
        with open(rec.paths[0]) as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual([r['calibration_position'] for r in rows], ['centre', 'reference', 'max', 'min', 'centre'])
        self.assertTrue(all(r['pot_raw_adc'] == '1414' for r in rows))

    def test_sensor_invalidity_is_distinct_from_missing_capture(self):
        rec = self.recording()
        line = row().split(',')
        line[4], line[5] = '0', '-1'
        line = ','.join(line)
        rec.consume(parse_wire(line), line)
        self.assertTrue(rec.finish('host_stop', end_ack=dict(final_enqueued_seq=0, tx_dropped_total=0)).recording_complete)

    def test_finalizer_failures_reach_result(self):
        for failure in ('analysis', 'flush', 'manifest'):
            with self.subTest(failure=failure):
                rec = self.recording()
                self.consume(rec)
                target, attr = {'analysis': (rec.analysis, 'finish'), 'flush': (rec._record_file, 'flush'),
                                'manifest': (rec, '_save_manifest')}[failure]
                with patch.object(target, attr, side_effect=OSError('injected')):
                    result = rec.finish('host_stop', end_ack=dict(final_enqueued_seq=0, tx_dropped_total=0))
                self.assertTrue(result.stop_confirmed)
                self.assertFalse(result.recording_complete)
                self.assertIsNotNone(result.failure_reason)
                with open(rec.paths[2]) as stream:
                    self.assertFalse(json.load(stream)['complete'])

    def test_worker_fault_supersedes_queued_completion(self):
        rec = self.recording()
        entered, release = threading.Event(), threading.Event()
        results = []
        worker = RecordingWorker(rec, lambda _: None, lambda *_: None, results.append)
        self.addCleanup(lambda: (release.set(), worker.join(2)))
        worker.submit(lambda: (entered.set(), release.wait())[1] and None)
        self.assertTrue(entered.wait(1))
        worker.finish('host_stop', end_ack=dict(final_enqueued_seq=-1, tx_dropped_total=0))
        worker.finish('transport_fault', True)
        release.set()
        self.assertTrue(worker.join(2))
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].failure_reason, 'transport_fault')


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def link(self, version=3, role='calibration'):
        fake = FakeSerial(version, role)
        link = SerialLink(None, 115200, 1, serial_instance=fake,
                          recording_directory=self.tmp.name, run_seconds=None)
        self.addCleanup(link.close)
        self.assertTrue(link.wait_until_ready(1))
        return link, fake

    def test_calibration_command_capture_and_shutdown(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        for key in 'HXmnH':
            self.assertTrue(link.send_command(key))
        self.assertTrue(link.stop_run())
        result = link.close()
        self.assertTrue(result.recording_complete)
        self.assertEqual(link.recording.rows, 5)
        self.assertEqual(link.plots.snapshot().calibration_position, ['centre', 'reference', 'max', 'min', 'centre'])
        self.assertEqual(link.recording.analysis.detector.count, 0)
        self.assertTrue(link.worker.join(0))

    def test_second_failed_start_has_fresh_statistics(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        fake.sample()
        self.assertTrue(link.stop_run())
        old_run = link.run
        fake.reject_start = True
        self.assertFalse(link.begin_timed_run(-3))
        link.close()
        self.assertEqual(link.recording.analysis.detector.count, 0)
        self.assertEqual(link.recording.analysis.dropped_rows, 0)
        self.assertEqual(link.recording.rows, 0)
        link._recording_finished(old_run, old_run.result)
        self.assertEqual(link.last_result.failure_reason, 'start_unconfirmed')

    def test_role_and_version_gate_endpoint_commands(self):
        for version, role in ((2, 'production'), (3, 'production')):
            link, fake = self.link(version, role)
            self.assertFalse(link.begin_timed_run(-3))
            for key in 'Xmn':
                self.assertFalse(link.send_command(key))
                self.assertNotIn(key.encode(), fake.writes)
            link.close()
        link, fake = self.link()
        self.assertFalse(link.send_command('m'))
        self.assertTrue(link.begin_timed_run(-3))
        link.metadata['cal_max_dac'] = 'unset'
        self.assertFalse(link.send_command('m'))

    def test_blocked_storage_close_is_explicit_and_recovers(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        link.worker.submit(lambda: (entered.set(), release.wait())[1] and None)
        self.assertTrue(entered.wait(1))
        with self.assertRaises(IncompleteShutdownError):
            link.close(timeout=.1)
        self.assertEqual(link.state, 'FAULTED')
        release.set()
        self.assertTrue(link.worker.join(2))
        self.assertTrue(link.last_result.stop_confirmed)
        self.assertFalse(link.last_result.recording_complete)

    def test_fault_while_manifest_publication_is_blocked(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = link.recording._save_manifest
        def save():
            if link.recording.manifest['complete']:
                entered.set()
                release.wait()
            original()
        with patch.object(link.recording, '_save_manifest', side_effect=save):
            thread = threading.Thread(target=link.stop_run)
            thread.start()
            self.assertTrue(entered.wait(1))
            link._fault('transport_fault', expected_run_id=link.run_id)
            release.set()
            thread.join(2)
            self.assertTrue(link.worker.join(2))
        self.assertFalse(link.last_result.recording_complete)
        self.assertTrue(link.last_result.stop_confirmed)
        self.assertEqual(link.last_result.failure_reason, 'transport_fault')
        with open(link.recording.paths[2]) as stream:
            self.assertFalse(json.load(stream)['complete'])

    def test_old_run_invalid_markers_are_ignored(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        link._handle_marker('# STARTED,3,ffffffffffffffff,CAL,bad,bad')
        link._handle_marker('# ENDED,3,ffffffffffffffff,bad,bad,bad')
        self.assertEqual(link.state, 'RUNNING')

    def test_concurrent_stop_finalizes_once(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(-3))
        fake.sample()
        results = []
        threads = [threading.Thread(target=lambda: results.append(link.stop_run())) for _ in range(4)]
        for thread in threads: thread.start()
        for thread in threads: thread.join(3)
        self.assertEqual(results, [True]*4)
        self.assertEqual(link.state, 'IDLE')
        self.assertEqual(link.recording.rows, 1)


if __name__ == '__main__':
    unittest.main()
