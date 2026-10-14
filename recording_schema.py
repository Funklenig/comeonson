"""Protocol 1/2/3 parsing and stable recording schemas; no transport or UI imports."""
import hashlib
import json
import math
import re
from collections import namedtuple

Sample = namedtuple("Sample", [
    "seq", "ms", "t_s", "host_unix",
    "flow_raw", "flow_avg", "flow_ok", "flow_age_us",
    "pa", "temp", "pok", "pa_age_us", "cmd_dac", "feedback_dac",
    "cmd_pct", "fdbk_pct", "target_flow_slm", "target_peak_slm",
    "phase", "preset", "clamped",
    "run_id", "scan_id", "row_us", "cycle_id", "scan_start_us",
    "missed_slots_total", "tx_dropped_total", "row_time_us", "flow_time_us",
    "pa_time_us", "segment_id", "discontinuity_reason", "target_reference_resolved",
    "target_peak_local_lmin", "pot_raw_adc", "calibration_position",
])

META_KEYS = set('core tidal_ml bore_mm stroke_mm dac_centre dac_half_range dac_corr_offset dac_corr_gain pot_raw_centre pot_dac_per_adc calibration_target slot_us sfm_scale flow_reference'.split())
META_KEYS_V3 = (META_KEYS - {'calibration_target'}) | set('firmware_role cal_centre_dac cal_reference_dac cal_max_dac cal_min_dac'.split())
POSITIONS = frozenset(('none', 'centre', 'reference', 'max', 'min'))

def valid_metadata(version, metadata):
    if set(metadata) != (META_KEYS_V3 if version == 3 else META_KEYS):
        return False
    numeric = META_KEYS - {'core', 'flow_reference'}
    if version == 3:
        numeric = numeric - {'calibration_target'}
    try:
        values = {key: float(metadata[key]) for key in numeric}
        if not all(math.isfinite(value) for value in values.values()):
            return False
        if any(values[key] <= 0 for key in ('tidal_ml', 'bore_mm', 'stroke_mm',
                                           'dac_half_range', 'dac_corr_gain', 'slot_us', 'sfm_scale')):
            return False
    except (TypeError, ValueError):
        return False
    if version == 3:
        if metadata['firmware_role'] not in ('production', 'calibration'):
            return False
        try:
            for key in ('cal_centre_dac', 'cal_reference_dac', 'cal_max_dac', 'cal_min_dac'):
                if key in ('cal_max_dac', 'cal_min_dac') and metadata[key] == 'unset':
                    continue
                if not 0 <= int(metadata[key]) <= 4095:
                    return False
        except ValueError:
            return False
    return True

SAMPLE_COLUMNS = ['wire_line', 'missing_before'] + list(Sample._fields)
PEAK_COLUMNS = ['peak_idx', 'seq', 'cycle_id', 'phase', 'direction', 'flow_peak_slm',
                'plateau_reference_flow_slm', 'peak_time_s', 'peak_offset_s',
                'interpolation_status', 'nearest_sample_seq', 'target_peak_slm',
                'target_error_slm', 'target_error_pct', 'target_met', 'is_warmup',
                'excursion_complete', 'pa_mean', 'pa_std', 'pa_ok_frac',
                'pressure_all_ok', 'timing_ok', 'peak_sample_ok', 'plateau_capture_ok',
                'qualified_peak', 'reason_codes', 'plateau_n', 'plateau_contiguous',
                'max_pressure_age_us', 'pressure_age_span_us', 'flow_pressure_skew_us',
                'plateau_span_s', 'excursion_n', 'excursion_s']


def config_fingerprint(manifest):
    fields = {key: manifest[key] for key in ('analysis_policy', 'detector_config',
              'avg_window', 'expected_expiration_sign', 'flow_reference')}
    encoded = json.dumps(fields, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode('ascii')).hexdigest()


def uint32(value):
    result = int(value)
    if not 0 <= result <= 0xffffffff:
        raise ValueError('uint32 out of range')
    return result


def parse_wire(line):
    p = line.split(',')
    protocol = 3 if p[0]=='DATA3' else None
    if protocol is None or len(p) != 46:
        raise ValueError('unknown DATA schema/width')
    def finite(i):
        value = float(p[i])
        if not math.isfinite(value):
            raise ValueError('nonfinite field')
        return value
    def flag(i):
        if p[i] not in ('0', '1'):
            raise ValueError('invalid flag')
        return p[i] == '1'
    def age(i, valid):
        value = int(p[i])
        if value < -1 or value > 0x7fffffff or (valid and value < 0):
            raise ValueError('invalid age')
        return value
    flow_ok = flag(4)
    pok = [flag(i) for i in range(14, 21)]
    phase, preset, dac = int(p[34]), int(p[35]), int(p[28])
    if phase not in (-1, 0, 1) or preset not in (-1,0,1,2,3) or not 0 <= dac <= 4095:
        raise ValueError('invalid phase/preset/DAC')
    if (preset >= 0 and phase == -1) or (preset < 0 and phase != -1):
        raise ValueError('phase/preset mismatch')
    rid = p[37] if protocol >= 2 else None
    if protocol == 3 and rid == '-':
        rid = None
    if protocol >= 2 and rid is not None and not re.fullmatch('[0-9a-f]{16}', rid):
        raise ValueError('invalid run ID')
    raw = int(p[44]) if protocol == 3 else None
    position = p[45] if protocol == 3 else None
    if protocol == 3:
        if not 0 <= raw <= 4095 or position != 'none':
            raise ValueError('invalid calibration fields')
    return dict(pot_raw_adc=raw, calibration_position=position, protocol=protocol, seq=uint32(p[1]), ms=uint32(p[2]),
                flow_raw=finite(3), flow_ok=flow_ok, flow_age_us=age(5, flow_ok),
                pa=[finite(i) for i in range(6, 13)], temp=finite(13), pok=pok,
                pa_age_us=[age(i, ok) for i, ok in zip(range(21, 28), pok)],
                cmd_dac=dac, feedback_dac=finite(29), cmd_pct=finite(30), fdbk_pct=finite(31),
                target_flow_slm=finite(32), target_peak_slm=finite(33), phase=phase, preset=preset,
                clamped=flag(36), run_id=rid,
                **{key: uint32(p[i]) if protocol >= 2 else None for key, i in
                   zip(('scan_id', 'row_us', 'cycle_id', 'scan_start_us', 'missed_slots_total', 'tx_dropped_total'), range(38, 44))})


def encode_cell(value):
    if isinstance(value, (list, tuple)):
        return json.dumps(value, separators=(',', ':'), allow_nan=True)
    if value is None:
        return ''
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return format(value, '.12g')
    return value


def sample_record(sample, line):
    return dict(wire_line=line, **{key: encode_cell(value) for key, value in sample._asdict().items()})


def event_record(event):
    result = {key: encode_cell(value) for key, value in event.items() if key != 'sample'}
    result.update(seq=event['sample'].seq, cycle_id=event['sample'].cycle_id, phase=event['sample'].phase)
    return result

SOFTWARE_FILES = ('Data_Logger.txt', 'peak_detection.py', 'analysis.py',
                  'recording_schema.py', 'recording.py', 'recording_worker.py',
                  'serial_link.py', 'plot_buffer.py', 'capture.py', 'analysis_runner.py', 'analysis_worker.py')
