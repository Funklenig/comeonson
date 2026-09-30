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
        for sketch in ('MechLung', 'calibration'):
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(ROOT/'tests/firmware_stubs'), '-I'+str(library),
                            '-DSKETCH_FILE="'+str(ROOT/'firmware'/sketch/(sketch+'.ino'))+'"',
                            str(ROOT/'tests/firmware_stubs/harness.cpp'),
                            *map(str, library.glob('*.cpp')), '-o', str(cls.directory/sketch)], check=True)

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
