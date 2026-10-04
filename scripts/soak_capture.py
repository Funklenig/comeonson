#!/usr/bin/env python3
"""Accelerated 30-minute capture per role, using synthetic rows and temporary CSVs."""
from pathlib import Path
import contextlib
import gc
import json
import math
import sys
import tempfile
import time
import tracemalloc

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests')]
from plot_buffer import PlotBuffer
from recording import Recording
from recording_schema import parse_wire
from test_recording import row, metadata, RID


class QuietSink:
    def write(self, text):
        return len(text)
    def flush(self):
        pass


def soak(calibration, count=180000):
    started = time.monotonic()
    with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(QuietSink()):
        rec = Recording(directory=directory, score_peaks=not calibration,
                        gas_temperature_c=20, absolute_pressure_mbar=1013)
        preset = -3 if calibration else 0
        rec.allocate(RID,'CAL' if calibration else 'P0',preset,
                     metadata(role='calibration' if calibration else 'production'))
        rec.start(0,0)
        plots = PlotBuffer(buffer_seconds=2)
        template = parse_wire(row(preset=preset,position='centre' if calibration else 'none'))
        tracemalloc.start()
        baseline = None
        try:
            for seq in range(count):
                sample = dict(template,seq=seq,ms=(seq+1)*10,row_us=((seq+1)*10000)&0xffffffff,
                              scan_id=seq+1,scan_start_us=((seq+1)*10000)&0xffffffff)
                if calibration:
                    # Repeated faults and alternating validity saturate diagnostic storage.
                    sample.update(flow_ok=bool(seq%2),flow_age_us=0 if seq%2 else -1)
                else:
                    phase=(seq//200)%2
                    sample.update(cycle_id=seq//400,phase=phase,target_peak_slm=10,
                                  flow_raw=(1 if phase==0 else -1)*10*math.sin(math.pi*(seq%200)/200))
                result = rec.consume(sample, 'synthetic-soak',host_unix=seq/100)
                plots.append(result)
                if seq == count//2:
                    gc.collect()
                    baseline = tracemalloc.get_traced_memory()[0]
            result = rec.finish('host_stop',end_ack=dict(final_enqueued_seq=count-1,tx_dropped_total=0))
            gc.collect()
            current, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
            if rec.started:
                rec.finish('soak_failure', True)
        assert result.recording_complete, result
        assert rec.rows == count
        assert len(plots.snapshot().t) == 200
        assert len(rec.analysis.boundaries) <= rec.analysis.MAX_BOUNDARIES
        assert current - baseline < 2_000_000, (baseline,current)
        if calibration:
            assert rec.analysis.boundaries_omitted > 0
            assert rec.analysis.detector.count == 0
        else:
            assert rec.analysis.detector.count == count//200
            assert rec.manifest['commanded_phases']['missing_interior'] == 0
        summary = dict(role='calibration' if calibration else 'production',rows=rec.rows,
                       simulated_seconds=count/100,wall_seconds=round(time.monotonic()-started,2),
                       retained_bytes=current,peak_bytes=peak,growth_second_half_bytes=current-baseline,
                       peaks=rec.analysis.detector.count,boundaries_omitted=rec.analysis.boundaries_omitted)
    print(json.dumps(summary,sort_keys=True))


if __name__ == '__main__':
    soak(False)
    soak(True)
