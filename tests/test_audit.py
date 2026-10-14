"""Independent integrity, analysis and lifecycle regressions from the code audit."""
import json
import math
import runpy
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from analysis import AnalysisSession
from peak_detection import PeakDetectionConfig, standard_flow
from recording import Recording, RecordingError
from recording_schema import parse_wire
from test_recording import RID, row, metadata, FakeSerial
from serial_link import SerialLink

ROOT = Path(__file__).resolve().parents[1]


def wire(seq=0, **changes):
    result = parse_wire(row(seq=seq, ms=(seq+1)*10))
    result.update(changes)
    return result


class IntegrityAuditTests(unittest.TestCase):
    def test_duplicate_and_backward_rows_cannot_complete(self):
        for field, value in (('seq', 0), ('seq', 0xffffffff), ('scan_id', 1),
                             ('missed_slots_total', 0), ('tx_dropped_total', 0)):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as directory:
                rec = Recording(directory=directory)
                rec.allocate(RID, 'P0', 0, metadata())
                rec.start(0, 0)
                initial = wire()
                if field.endswith('_total'):
                    initial[field] = 1
                rec.consume(initial, row())
                try:
                    self.assertIsNone(rec.consume(dict(wire(1), **{field: value}), row(1)))
                    self.assertEqual(rec.bad_lines,1)
                finally:
                    rec.finish('test', True)

    def test_end_counter_regression_cannot_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = Recording(directory=directory)
            rec.allocate(RID, 'P0', 0, metadata())
            rec.start(0, 0)
            rec.consume(wire(), row())
            result = rec.finish('host_stop', end_ack={'final_enqueued_seq': 0, 'tx_dropped_total': 0xffffffff})
            self.assertFalse(result.recording_complete)

    def test_end_without_start_cannot_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            rec = Recording(directory=directory)
            rec.allocate(RID, 'P0', 0, metadata())
            self.assertFalse(rec.finish('host_stop', end_ack={'final_enqueued_seq': -1, 'tx_dropped_total': 0}).recording_complete)

    def test_oversized_integer_row_is_counted_as_malformed(self):
        fake = FakeSerial()
        link = SerialLink(None, 115200, 1, serial_instance=fake, start_reader=False)
        self.addCleanup(link.close)
        line = row().split(',')
        line[1] = '9'*950
        link.feed_bytes((','.join(line)+'\n').encode())
        self.assertEqual(link.bad_lines, 1)


class AnalysisAuditTests(unittest.TestCase):
    def test_repeated_invalid_flow_has_bounded_boundaries(self):
        session = AnalysisSession(score_peaks=False)
        session.set_origin(0, 0)
        for seq in range(5000):
            session.push(wire(seq, flow_ok=False, flow_age_us=-1))
        self.assertLessEqual(len(session.boundaries), 2)

    def test_never_ending_excursion_is_bounded(self):
        session = AnalysisSession(config=PeakDetectionConfig(min_samples=1, smooth_window=1))
        sample = session.push(wire(flow_raw=0)).sample
        detector = session.detector
        detector.push(sample)
        for seq in range(70000):
            detector.push(sample._replace(seq=seq+1, t_s=(seq+1)/100, flow_raw=12, flow_avg=12), check_continuity=False)
        self.assertLessEqual(len(detector._buf), 60000)
        self.assertGreater(detector.partial_excursions, 0)

    def test_reference_overflow_rejected(self):
        with self.assertRaises(ValueError):
            standard_flow(1.0, -273.1499999999999, 1e308)

    def test_extreme_pressure_does_not_crash_peak_analysis(self):
        session = AnalysisSession(config=PeakDetectionConfig(min_samples=1, smooth_window=1),
                                  gas_temperature_c=20, absolute_pressure_mbar=1013)
        sample = session.push(wire(flow_raw=0, preset=0, phase=0)).sample
        detector = session.detector
        detector.push(sample)
        for seq, pressure in enumerate((1e200, -1e200, 1e200)):
            detector.push(sample._replace(seq=seq+1, t_s=(seq+1)/100, flow_raw=12, flow_avg=12,
                                          pa=[pressure]*7), check_continuity=False)
        events = detector.flush()
        self.assertEqual(len(events), 1)
        self.assertTrue(all(math.isfinite(v) for v in events[0]['pa_std']))


class GuiAuditTests(unittest.TestCase):
    def test_plot_setup_failure_closes_active_session(self):
        with tempfile.TemporaryDirectory() as directory:
            fake = FakeSerial()
            link = SerialLink(None, 115200, 1, serial_instance=fake, run_seconds=None,
                              recording_directory=directory)
            self.addCleanup(link.close)
            entry = runpy.run_path(str(ROOT/'Data_Logger.txt'))
            entry['main'].__globals__['SerialLink'] = lambda *a, **kw: link
            def fail_plot(*args,**kwargs):
                self.assertTrue(link.begin_timed_run(0))
                fake.sample()
                raise RuntimeError('display failed')
            with patch.object(sys, 'argv', ['logger', '--port', 'fake']), \
                 patch('matplotlib.pyplot.subplots', side_effect=fail_plot), \
                 patch('time.sleep', return_value=None):
                with self.assertRaisesRegex(RuntimeError, 'display failed'):
                    entry['main']()
            try:
                self.assertTrue(fake.closed)
                self.assertTrue(link.worker.join(0))
            finally:
                link.close()

class ControlAuditTests(unittest.TestCase):
    def link(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        fake = FakeSerial()
        link = SerialLink(None,115200,1,serial_instance=fake,run_seconds=None,
                          recording_directory=directory.name)
        self.addCleanup(link.close)
        self.assertTrue(link.wait_until_ready(1))
        return link, fake


    def test_closed_link_cannot_allocate_another_recording(self):
        link, fake = self.link()
        link.close()
        with patch('serial_link.Recording') as recording:
            self.assertFalse(link.begin_timed_run(0))
        recording.assert_not_called()

    def test_boot_discards_partial_handshake(self):
        link, fake = self.link()
        link._handle_marker('# META_BEGIN,3,stale')
        link._handle_marker('# BOOT,3')
        self.assertIsNone(link._meta_pending)
        self.assertIsNone(link._meta_build)

    def test_delayed_stop_ack_has_separate_storage_deadline(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        write, finish = fake.write, link.recording.finish
        timers = []
        def delayed_write(data):
            if data.startswith(b'@STOP,'):
                timer = threading.Timer(.25, write, args=(data,))
                timers.append(timer)
                timer.start()
                return len(data)
            return write(data)
        def delayed_finish(*args):
            time.sleep(.25)
            return finish(*args)
        try:
            with patch.object(fake, 'write', side_effect=delayed_write), \
                 patch.object(link.recording, 'finish', side_effect=delayed_finish):
                self.assertTrue(link.stop_run(timeout=.4))
        finally:
            for timer in timers:
                timer.join(1)
            link.close()

    def test_finalization_uses_its_deadline_not_capture_latency(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        finish = link.recording.finish
        def delayed_finish(*args):
            time.sleep(.1)
            return finish(*args)
        link.worker.max_latency = .02
        with patch.object(link.recording, 'finish', side_effect=delayed_finish):
            self.assertTrue(link.stop_run(timeout=.5))

    def test_stale_timer_cannot_stop_a_new_run(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        generation = link._run_generation
        self.assertTrue(link.stop_run())
        self.assertTrue(link.worker.join(1))
        self.assertTrue(link.begin_timed_run(0))
        link._on_run_timeout(generation)
        self.assertEqual(link.state, 'RUNNING')

    def test_reader_disconnect_finalizes_incomplete_and_exits(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        with patch.object(fake, 'read', side_effect=OSError('disconnected')):
            link._thread.join(1)
        self.assertFalse(link._thread.is_alive())
        self.assertTrue(link.worker.join(1))
        self.assertFalse(link.last_result.recording_complete)
        self.assertFalse(link.last_result.stop_confirmed)

    def test_duplicate_rows_are_reported_without_ending_capture(self):
        link,fake=self.link()
        self.assertTrue(link.begin_timed_run(0))
        fake.sample()
        fake.emit(row(0,run_id=link.run_id,ms=20))
        self.assertTrue(link.stop_run())
        self.assertEqual(link.recording.rows,1)
        self.assertTrue(link.last_result.recording_complete)
        self.assertFalse(link.last_result.quality_complete)
        self.assertEqual(link.recording.manifest['invalid_rows'],1)

    def test_fragmented_malformed_and_oversize_frames(self):
        fake = FakeSerial()
        link = SerialLink(None,115200,1,serial_instance=fake,start_reader=False)
        self.addCleanup(link.close)
        for part in (b'\xff\n', b'DATA3,'+b'0'*1100, b'\n', row(run_id='-').encode()[:30], row(run_id='-').encode()[30:]+b'\n'):
            link.feed_bytes(part)
        self.assertEqual(link.bad_lines, 2)
        self.assertEqual(len(link.plots.snapshot().t), 1)

    def test_storage_write_failure_is_persisted(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        with patch.object(link.recording._record_writer, 'writerow', side_effect=OSError('disk full')):
            fake.sample()
            self.assertTrue(link.run.finalized.wait(1))
        self.assertEqual(link.last_result.failure_reason, 'storage_fault')
        with open(link.recording.paths[1]) as stream:
            self.assertFalse(json.load(stream)['complete'])

    def test_recording_overflow_faults_without_losing_finalization(self):
        link, fake = self.link()
        self.assertTrue(link.begin_timed_run(0))
        entered, release = threading.Event(), threading.Event()
        self.assertTrue(link.worker.submit(lambda: (entered.set(), release.wait())[1] and None))
        self.assertTrue(entered.wait(1))
        link.worker.capacity = 1
        try:
            link._handle_data_line(row(0,run_id=link.run_id))
            link._handle_data_line(row(1,run_id=link.run_id))
            self.assertIn(link.state, ('STOPPING','FINALIZING','FAULTED'))
        finally:
            release.set()
        self.assertTrue(link.worker.join(1))
        self.assertFalse(link.last_result.recording_complete)
        self.assertEqual(link.last_result.failure_reason, 'recording_overflow')

    def test_successful_discard_removes_only_owned_files(self):
        link, fake = self.link()
        sentinel=Path(link._recording_args[-1])/'existing.csv'
        sentinel.write_text('keep')
        self.assertTrue(link.begin_timed_run(0))
        fake.sample()
        paths=link.recording.paths
        self.assertTrue(link.abort_run())
        self.assertTrue(link.worker.join(1))
        self.assertTrue(all(not Path(path).exists() for path in paths))
        self.assertEqual(sentinel.read_text(),'keep')

class AdditionalAnalysisTests(unittest.TestCase):
    def test_discontinuity_resets_old_target_exit_threshold(self):
        session = AnalysisSession(config=PeakDetectionConfig(min_samples=1, smooth_window=1))
        sample = session.push(wire(flow_raw=0)).sample
        detector = session.detector
        detector.push(sample._replace(target_peak_slm=100))
        detector.push(sample._replace(flow_raw=40, flow_avg=40, target_peak_slm=100), check_continuity=False)
        detector.break_continuity('gap')
        for seq in range(3):
            detector.push(sample._replace(seq=seq+2, t_s=(seq+2)/100, flow_raw=5, flow_avg=5,
                                          target_peak_slm=2), check_continuity=False)
        self.assertEqual(detector._buf, [])

    def test_repeated_gaps_never_join_separate_excursions(self):
        session = AnalysisSession(config=PeakDetectionConfig(min_samples=3,smooth_window=1,warmup_breaths=0),
                                  gas_temperature_c=20,absolute_pressure_mbar=1013)
        session.set_origin(0,0)
        events=[]
        for seq,flow in enumerate((0,0,4,8,10,8,4,0)):
            events += session.push(wire(seq*2,flow_raw=flow,target_peak_slm=10,preset=0,phase=0)).events
        self.assertEqual(events,[])

    def test_hand_computed_signed_peaks_and_pressure(self):
        for sign in (-1,1):
            session = AnalysisSession(config=PeakDetectionConfig(min_samples=3,smooth_window=1,warmup_breaths=0),
                                      gas_temperature_c=20,absolute_pressure_mbar=1013)
            session.set_origin(0,0)
            events = []
            for seq,flow in enumerate((0,4,8,10,8,4,0)):
                events += session.push(wire(seq,flow_raw=sign*flow,target_peak_slm=10,
                                           preset=0,phase=0,pa=[2]*7)).events
            self.assertEqual(len(events),1)
            peak = events[0]
            self.assertAlmostEqual(peak['flow_peak_slm'],sign*10)
            self.assertAlmostEqual(peak['peak_time_s'],.04)
            self.assertEqual(peak['pa_mean'],[2]*7)
            self.assertEqual(peak['pa_std'],[0]*7)
            self.assertTrue(peak['qualified_peak'])

    def test_sparse_scan_ids_cannot_hide_capture_loss(self):
        session = AnalysisSession()
        session.set_origin(0,0)
        session.push(wire())
        with self.assertRaises(ValueError):
            session.push(wire(1,scan_id=3))

    def test_nonfinite_metadata_is_not_ready(self):
        from recording_schema import valid_metadata
        for key in ('tidal_ml','dac_corr_gain','sfm_scale','slot_us'):
            with self.subTest(key=key):
                self.assertFalse(valid_metadata(3,dict(metadata(),**{key:'nan'})))
