"""Compile real sketches against synthetic GPIO/I2C/USB, then use the host logger."""
import csv
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import unittest

from recording_schema import parse_wire
from serial_link import SerialLink

ROOT = Path(__file__).resolve().parents[1]


class FirmwareSerial:
    write_timeout = 1
    def __init__(self, binary):
        self.process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.lock = threading.Lock()
        self.closed = False
        self.pending = bytearray()

    def command(self, command):
        self.process.stdin.write(command.encode()+b'\n')
        self.process.stdin.flush()
        result = bytearray()
        while True:
            c = self.process.stdout.read(1)
            if not c: raise OSError('firmware harness stopped')
            if c == b'\x1e':
                self.process.stdout.read(1)
                return bytes(result)
            result.extend(c)

    def write(self, data):
        with self.lock:
            self.pending.extend(self.command('W '+data.hex()))
        return len(data)

    def read(self, _size):
        time.sleep(.005)
        with self.lock:
            if self.closed: return b''
            self.pending.extend(self.command('T 10000'))
            result = bytes(self.pending)
            self.pending.clear()
            return result

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.process.stdin.close()
                self.process.wait(timeout=2)
                self.process.stdout.close()


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.directory = Path(cls.temp.name)
        library = cls.directory/'library'
        shutil.copytree(ROOT/'firmware/libraries/MechCommon/src', library)
        # Synthetic endpoints are NEVER written to the repository or uploaded.
        config = library/'calibration_values.h'
        config.write_text(config.read_text().replace('CAL_MIN_DAC = -1', 'CAL_MIN_DAC = 1200')
                          .replace('CAL_MAX_DAC = -1', 'CAL_MAX_DAC = 1800'))
        for binary, sketch, source_library in (('MechLung','MechLung',library),
                                               ('calibration','calibration',library),
                                               ('calibration_unset','calibration',ROOT/'firmware/libraries/MechCommon/src')):
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(ROOT/'tests/firmware_stubs'), '-I'+str(source_library),
                            '-DSKETCH_FILE="'+str(ROOT/'firmware'/sketch/(sketch+'.ino'))+'"',
                            str(ROOT/'tests/firmware_stubs/harness.cpp'),
                            *map(str, source_library.glob('*.cpp')), '-o', str(cls.directory/binary)], check=True)

    def test_calibration_capture_real_sketch_to_csv(self):
        serial = FirmwareSerial(self.directory/'calibration')
        link = SerialLink(None, 115200, 1, serial_instance=serial, run_seconds=None,
                          recording_directory=str(self.directory/'captures'))
        self.addCleanup(link.close)
        self.assertTrue(link.wait_until_ready(2))
        self.assertTrue(link.begin_timed_run(-3))
        for key in 'HXmnH':
            self.assertTrue(link.send_command(key))
            time.sleep(.04)
        self.assertTrue(link.stop_run())
        self.assertTrue(link.close().recording_complete)
        with open(link.recording.paths[0]) as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue(rows)
        self.assertEqual(set(row['calibration_position'] for row in rows), {'centre','reference','max','min'})
        self.assertEqual([int(row['seq']) for row in rows], list(range(len(rows))))
        times = [float(row['t_s']) for row in rows]
        self.assertTrue(all(b>a for a,b in zip(times,times[1:])))
        expected = {'centre': 1434, 'reference': 1894, 'max': 1800, 'min': 1200}
        for row in rows:
            self.assertEqual(int(row['cmd_dac']), expected[row['calibration_position']])
            self.assertTrue(1400 <= int(row['pot_raw_adc']) < 1500)
            self.assertEqual(row['flow_ok'], '0')
            self.assertEqual(row['flow_age_us'], '-1')
        self.assertEqual(link.recording.analysis.detector.count, 0)
        self.assertIn('max', link.plots.snapshot().calibration_position)

    def test_production_hold_before_and_after_stop(self):
        serial = FirmwareSerial(self.directory/'MechLung')
        self.addCleanup(serial.close)
        serial.write(b'?')
        output = b''.join(serial.read(1024) for _ in range(30)).decode()
        rows = [parse_wire(line) for line in output.splitlines() if line.startswith('DATA3')]
        self.assertGreater(len(rows), 5)
        self.assertTrue(all(row['run_id'] is None and row['preset'] == -1 for row in rows))
        serial.write(b'@START,0123456789abcdef,P0\n')
        output = b''.join(serial.read(1024) for _ in range(10)).decode()
        self.assertIn('# STARTED,3,0123456789abcdef,P0', output)
        serial.write(b'@STOP,0123456789abcdef\n')
        output = b''.join(serial.read(1024) for _ in range(15)).decode()
        after = output.split('# ENDED,3,0123456789abcdef,')[1].splitlines()[1:]
        rows = [parse_wire(line) for line in after if line.startswith('DATA3')]
        self.assertGreater(len(rows), 5)
        self.assertTrue(all(row['run_id'] is None and row['preset'] == -1 for row in rows))
        self.assertTrue(all(b['row_us'] > a['row_us'] for a,b in zip(rows,rows[1:])))

    def test_watchdog_and_old_monitor_alias(self):
        serial = FirmwareSerial(self.directory/'calibration')
        self.addCleanup(serial.close)
        serial.write(b'@START,0123456789abcdef,CAL\n')
        for _ in range(5): serial.read(1024)
        serial.write(b'X')
        for _ in range(5): serial.read(1024)
        with serial.lock:
            output = serial.command('T 3100000').decode()
            applied = serial.command('A').decode()
        self.assertIn(',watchdog,', output)
        self.assertIn('# APPLIED,1434', applied)
        serial.write(b'm')
        output = b''.join(serial.read(1024) for _ in range(5)).decode()
        self.assertIn('start_CAL_session_first', output)

    def harness(self, sketch='calibration'):
        serial = FirmwareSerial(self.directory/sketch)
        self.addCleanup(serial.close)
        return serial

    def exchange(self, serial, data=b'', us=100000):
        serial.command('W '+data.hex()) if data else None
        text = getattr(serial, '_exchange_tail', '') + serial.command(f'T {us}').decode()
        complete, separator, serial._exchange_tail = text.rpartition('\n')
        return complete + separator

    def healthy(self, serial):
        serial.command('I 0 0 1')
        for ch in range(7):
            serial.command(f'I 1 {ch} 1')
        self.exchange(serial, us=1500000)

    def rows(self, text):
        return [parse_wire(line) for line in text.splitlines() if line.startswith('DATA3')]

    def test_successful_sensor_acquisition_and_each_fault_recovers(self):
        serial = self.harness()
        self.healthy(serial)
        self.exchange(serial, b'@START,0123456789abcdef,CAL\n')
        rows = self.rows(self.exchange(serial))
        self.assertGreater(len(rows), 5)
        self.assertTrue(all(r['flow_ok'] and all(r['pok']) for r in rows))
        self.assertTrue(all(r['flow_raw']==10 and r['pa']==list(range(1,8)) and r['temp']==20 for r in rows))
        self.assertTrue(all(0 <= r['flow_age_us'] < 10000 and all(0 <= a < 10000 for a in r['pa_age_us']) for r in rows))
        for mode in (2,3,4,5,6):
            with self.subTest(mode=mode):
                serial.command(f'I 0 0 {mode if mode < 5 else 1}')
                serial.command(f'I 1 2 {mode}')
                self.exchange(serial, b'@PING,0123456789abcdef\n', us=200000)
                rows = self.rows(self.exchange(serial))
                self.assertTrue(rows)
                self.assertTrue(all(not r['pok'][2] and r['pa_age_us'][2]==-1 for r in rows))
                self.assertTrue(all(all(ok for ch,ok in enumerate(r['pok']) if ch!=2) for r in rows))
                if mode < 5:
                    self.assertTrue(all(not r['flow_ok'] and r['flow_age_us']==-1 for r in rows))
                serial.command('I 0 0 1'); serial.command('I 1 2 1')
                self.exchange(serial, b'@PING,0123456789abcdef\n', us=1500000)
                rows = self.rows(self.exchange(serial))
                self.assertTrue(all(r['flow_ok'] and all(r['pok']) for r in rows))
        counts = serial.command('V').decode().splitlines()[0].split(',')[1:]
        self.assertGreater(int(counts[0]), 1)
        self.assertGreater(int(counts[3]), 1)

    def test_backpressure_preserves_stop_barrier_and_reports_loss(self):
        serial = self.harness()
        self.healthy(serial)
        self.exchange(serial, b'@START,0123456789abcdef,CAL\n')
        serial.command('C 0')
        self.exchange(serial, us=200000)
        self.exchange(serial, b'@START,0123456789abcdef,CAL\n'*8)
        self.exchange(serial, b'@STOP,0123456789abcdef\n')
        self.assertIn('# APPLIED,1434', serial.command('A').decode())
        serial.command('C 64'); serial.command('L 3')
        output = self.exchange(serial, us=500000)
        before, after = output.split('# ENDED,3,0123456789abcdef,')
        fields = after.splitlines()[0].split(',')
        owned = [r for r in self.rows(before) if r['run_id']]
        self.assertEqual(int(fields[0]), owned[-1]['seq'])
        self.assertGreater(int(fields[2]), 0)
        self.assertTrue(all(r['run_id'] is None for r in self.rows('\n'.join(after.splitlines()[1:]))))

    def test_framing_duplicate_commands_stale_ids_and_capacity(self):
        serial = self.harness()
        output = self.exchange(serial, b'@START,0123456789abcdef,CA')
        self.assertNotIn('# STARTED', output)
        output = self.exchange(serial, b'L\n')
        self.assertIn('# STARTED', output)
        output = self.exchange(serial, b'@STOP,ffffffffffffffff\n')
        self.assertNotIn('# ENDED', output)
        output = self.exchange(serial, b'@START,0123456789abcdef,CAL\n')
        self.assertIn('# STARTED', output)
        output = self.exchange(serial, b'@STOP,0123456789abcdef\n')
        marker = next(line for line in output.splitlines() if line.startswith('# ENDED'))
        output = self.exchange(serial, b'@STOP,0123456789abcdef\n')
        self.assertIn(marker, output)
        self.assertNotIn('# STARTED', self.exchange(serial, b'@START,0123456789abcdef,CAL\n'))
        for index in range(63):
            rid = f'{index:016x}'
            self.assertIn('# STARTED', self.exchange(serial, f'@START,{rid},CAL\n'.encode(), 10000))
            self.assertIn('# ENDED', self.exchange(serial, f'@STOP,{rid}\n'.encode(), 10000))
        self.assertIn('completed_id_capacity_reset_required', self.exchange(serial, b'@START,ffffffffffffffff,CAL\n'))

    def test_invalid_and_expired_frames_cannot_move(self):
        serial = self.harness()
        for frame in (b'@START,'+b'a'*100+b',CAL\n', b'@START,gggggggggggggggg,CAL\n', b'BE\n'):
            self.assertNotIn('# STARTED', self.exchange(serial, frame))
        self.exchange(serial, b'@START,0123456789abcdef,', 300000)
        self.assertNotIn('# STARTED', self.exchange(serial, b'CAL\n'))
        self.assertIn('# STARTED', self.exchange(serial, b'@START,0123456789abcdef,CAL\n'))

    def test_clock_rollovers_and_sensor_latency(self):
        for clock in (2**32-50000, (2**32-50)*1000):
            with self.subTest(clock=clock):
                serial = self.harness()
                self.healthy(serial)
                # Cancel the old scan before moving simulated wall time.
                serial.command('W '+b'H'.hex())
                serial.command('T 100')
                serial.command(f'J {clock}')
                output = self.exchange(serial, b'@START,0123456789abcdef,CAL\n', 100000)
                self.assertIn('# STARTED', output)
                rows = [r for r in self.rows(output) if r['run_id']]
                self.assertGreater(len(rows), 5)
                from analysis import AnalysisSession
                session=AnalysisSession(score_peaks=False)
                ack=next(line.split(',') for line in output.splitlines() if line.startswith('# STARTED'))
                session.set_origin(int(ack[4]),int(ack[5]))
                normalized=[session.push(r).sample for r in rows]
                self.assertTrue(all(b.t_s>a.t_s for a,b in zip(normalized,normalized[1:])))
                self.assertTrue(all(r['flow_ok'] for r in rows))
                self.assertTrue(all((b['row_us']-a['row_us']) & 0xffffffff == 10000 for a,b in zip(rows,rows[1:])))
                serial.command('D 1 5000')
                rows = self.rows(self.exchange(serial, us=100000))
                self.assertTrue(any(not all(r['pok']) for r in rows))
                self.assertTrue(all(r['flow_age_us'] < 20000 for r in rows if r['flow_ok']))
                self.assertIn('host_stop', self.exchange(serial, b'@STOP,0123456789abcdef\n'))

    def test_production_capture_with_valid_sensors_to_manifest(self):
        serial = self.harness('MechLung')
        self.healthy(serial)
        link = SerialLink(None,115200,1,serial_instance=serial,run_seconds=None,
                          recording_directory=str(self.directory/'production'))
        self.addCleanup(link.close)
        self.assertTrue(link.wait_until_ready(2))
        self.assertTrue(link.begin_timed_run(0))
        time.sleep(.15)
        self.assertTrue(link.stop_run())
        self.assertTrue(link.close().recording_complete)
        with open(link.recording.paths[0]) as stream:
            rows=list(csv.DictReader(stream))
        self.assertGreater(len(rows), 10)
        self.assertTrue(all(row['flow_ok']=='1' for row in rows))
        with open(link.recording.paths[2]) as stream:
            self.assertTrue(json.load(stream)['complete'])

    def test_unset_firmware_endpoints_are_rejected(self):
        serial=self.harness('calibration_unset')
        self.assertIn('# STARTED',self.exchange(serial,b'@START,0123456789abcdef,CAL\n'))
        for key in (b'm',b'n'):
            self.assertIn('configure_rig_endpoint_first',self.exchange(serial,key))
            self.assertIn('# APPLIED,1434',serial.command('A').decode())

    def test_all_production_waveforms_and_demo(self):
        for preset,peak in enumerate((10,15,20,30)):
            with self.subTest(preset=preset):
                serial=self.harness('MechLung')
                self.healthy(serial)
                rows=self.rows(self.exchange(serial,f'@START,0123456789abcdef,P{preset}\n'.encode()))
                for _ in range(22):
                    rows+=self.rows(self.exchange(serial,b'@PING,0123456789abcdef\n',1000000))
                rows=[r for r in rows if r['run_id']]
                self.assertTrue(all(r['preset']==preset and not r['clamped'] for r in rows))
                self.assertGreaterEqual(max(r['cycle_id'] for r in rows),1)
                self.assertAlmostEqual(max(r['target_flow_slm'] for r in rows),peak/2,delta=.01)
                self.assertAlmostEqual(min(r['target_flow_slm'] for r in rows),-peak,delta=.01)
                for a,b in zip(rows,rows[1:]):
                    if (a['phase'],a['cycle_id'])==(b['phase'],b['cycle_id']):
                        self.assertGreaterEqual((b['cmd_dac']-a['cmd_dac'])*(1 if b['phase']==0 else -1),0)
                self.exchange(serial,b'@STOP,0123456789abcdef\n')
        serial=self.harness('MechLung')
        output=self.exchange(serial,b'@START,0123456789abcdef,DEMO\n')
        for _ in range(4):
            output+=self.exchange(serial,b'@PING,0123456789abcdef\n',1000000)
        rows=[r for r in self.rows(output) if r['run_id']]
        self.assertTrue(all(r['preset']==-2 and r['phase']==-1 and r['target_flow_slm']==0 for r in rows))
        self.assertEqual(min(r['cmd_dac'] for r in rows),100)
        self.assertEqual(max(r['cmd_dac'] for r in rows),3995)
