import json
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analysis import AnalysisSession
from recording_schema import parse_wire, event_record
from serial_link import SerialLink
from test_recording import FakeSerial

ROOT = Path(__file__).resolve().parents[1]


class CliReplayTests(unittest.TestCase):
    def test_production_peaks_match_original_main(self):
        session = AnalysisSession(gas_temperature_c=20, absolute_pressure_mbar=1013)
        session.set_origin(0, 0)
        events = []
        for line in (ROOT/'tests/fixtures/production_data3.csv').read_text().splitlines():
            events.extend(session.push(parse_wire(line), host_unix=0).events)
        events.extend(session.finish())
        expected = json.loads((ROOT/'tests/fixtures/production_peaks.json').read_text())
        self.assertEqual([event_record(event) for event in events], expected)
        self.assertGreater(len(events), 0)

    def test_readiness_timeout_default_override_and_cleanup(self):
        for options, timeout, baud in (([],20.0,460800),
                                       (['--ready-timeout','45'],45.0,460800),
                                       (['--baud','115200'],20.0,115200)):
            with self.subTest(options=options):
                link = Mock()
                link.wait_until_ready.return_value = False
                link.close.return_value = None
                entry = runpy.run_path(str(ROOT/'Data_Logger.txt'))
                factory = Mock(return_value=link)
                entry['main'].__globals__['SerialLink'] = factory
                with patch.object(sys,'argv',['logger','--port','fake']+options), \
                     patch('time.sleep',return_value=None):
                    with self.assertRaisesRegex(SystemExit,f'within {timeout:g} seconds'):
                        entry['main']()
                self.assertEqual(factory.call_args.args[:2], ('fake', baud))
                link.wait_until_ready.assert_called_once_with(timeout)
                link.close.assert_called_once_with()

    def test_readiness_timeout_rejects_invalid_values(self):
        for value in ('0','-1','nan','inf'):
            with self.subTest(value=value):
                result = subprocess.run([sys.executable,str(ROOT/'Data_Logger.txt'),
                                         '--port','unused','--ready-timeout',value],
                                        capture_output=True,text=True)
                self.assertEqual(result.returncode,2)
                self.assertIn('--ready-timeout',result.stderr)

    def test_analysis_config_does_not_block_capture_factory(self):
        entry=runpy.run_path(str(ROOT/'Data_Logger.txt'))
        factory=Mock(side_effect=RuntimeError('capture factory reached'))
        entry['main'].__globals__['SerialLink']=factory
        with patch.object(sys,'argv',['logger','--port','fake','--peak-smooth','2']):
            with self.assertRaisesRegex(RuntimeError,'capture factory reached'):
                entry['main']()
        factory.assert_called_once()

    def test_production_gui_callbacks_render_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            fake = FakeSerial()
            link = SerialLink(None, 115200, 1, serial_instance=fake, run_seconds=None,
                              recording_directory=directory)
            self.addCleanup(link.close)
            self.addCleanup(plt.close, 'all')
            callbacks = []
            real_sleep = time.sleep
            def show():
                link.begin_timed_run(0)
                fake.sample()
                for key in 'H':
                    link.send_command(key)
                self.assertTrue(link.stop_run())
                for callback in callbacks:
                    callback(0)
                for number in plt.get_fignums():
                    plt.figure(number).canvas.draw()
                actuator = plt.figure(2).axes[0]
                self.assertIn('raw A1: 1414', actuator.texts[0].get_text())
            entry = runpy.run_path(str(ROOT/'Data_Logger.txt'))
            with patch.object(sys, 'argv', ['Data_Logger.txt','--port','fake']), \
                 patch('serial_link.SerialLink', return_value=link), \
                 patch('matplotlib.animation.FuncAnimation', side_effect=lambda fig, fn, **kw: callbacks.append(fn)), \
                 patch('matplotlib.pyplot.show', side_effect=show), \
                 patch('time.sleep', side_effect=lambda seconds: None if seconds == 2.5 else real_sleep(seconds)):
                # run_path captured imports before patch; supply factory in main's globals.
                entry['main'].__globals__['SerialLink'] = lambda *args, **kw: link
                entry['main']()
