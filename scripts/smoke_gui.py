#!/usr/bin/env python3
"""Exercise real Tk windows and keyboard callbacks using a synthetic serial board.

Run from the repo root: xvfb-run -a .venv/bin/python scripts/smoke_gui.py
No hardware is opened; captures live in a temporary directory.
"""
from pathlib import Path
import runpy
import sys
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests')]
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backend_bases import KeyEvent
from serial_link import SerialLink
from test_recording import FakeSerial


def smoke():
    with tempfile.TemporaryDirectory() as directory:
        fake = FakeSerial()
        link = SerialLink(None,115200,1,serial_instance=fake,run_seconds=None,
                          recording_directory=directory)
        entry = runpy.run_path(str(ROOT/'Data_Logger.txt'))
        entry['main'].__globals__['SerialLink'] = lambda *a, **kw: link
        original_show = plt.show
        failures = []
        timers = []
        keys = iter(('1','H','2','p','3','H','4','H','q'))
        def show():
            assert len(plt.get_fignums()) == 3
            canvas = plt.figure(1).canvas
            def tick():
                try:
                    if link.state == 'RUNNING':
                        preset = link.recording.manifest['preset']
                        fake.sample(preset=preset, position='none')
                    key = next(keys, 'q')
                    if key == 'q':
                        timer.stop()
                    canvas.callbacks.process('key_press_event', KeyEvent('key_press_event', canvas, key=key))
                    if key != 'q':
                        for number in plt.get_fignums():
                            plt.figure(number).canvas.draw()
                except BaseException as exc:
                    failures.append(exc)
                    timer.stop()
                    plt.close('all')
            timer = canvas.new_timer(interval=300)
            timer.add_callback(tick)
            timers.append(timer)
            timer.start()
            original_show(block=True)
        argv = ['Data_Logger.txt','--port','synthetic']
        try:
            with patch.object(sys,'argv',argv), patch('matplotlib.pyplot.show',side_effect=show):
                entry['main']()
            if failures:
                raise failures[0]
            assert fake.closed and link.worker.join(0)
            assert link.last_result.recording_complete
            assert len(list(Path(directory).glob('*_session.json'))) == 3
        finally:
            plt.close('all')
            link.close()
    print('Four-preset Tk GUI smoke passed')


if __name__ == '__main__':
    smoke()
