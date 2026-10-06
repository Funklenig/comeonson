# Mechanical lung logger and firmware

One Python logger supports production breathing runs and interactive calibration
captures. Two independently compiled sketches share actuator I/O, sensor
acquisition, bounded serial transport and a protocol-3 encoder.

## Python setup and tests

Python 3.9 or newer is required. From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python Data_Logger.txt --port /dev/ttyACM0
```

The full test suite also requires `g++` (C++17) for the synthetic firmware harness.
Additional software acceptance checks (no board or serial port required):

```sh
xvfb-run -a .venv/bin/python scripts/smoke_gui.py
.venv/bin/python scripts/soak_capture.py
```

The GUI smoke uses the real Tk backend, all three plot windows, keyboard callbacks,
and a synthetic serial board for both roles. Linux needs Tk and Xvfb; on Amazon
Linux install missing packages with `sudo dnf install python3-tkinter xorg-x11-server-Xvfb xorg-x11-xauth`.
On a desktop with a display, omit `xvfb-run -a`. The soak writes 180,000 synthetic
rows per role (30 simulated minutes each), checks manifests, peaks, bounded plot
history and memory growth, then removes its temporary captures. It uses
`tracemalloc`, so runtime is longer than an ordinary replay.
Matplotlib needs a desktop display for interactive plot windows. `--list-ports`
lists devices. Only one program can own the board's serial port.

The logger waits 2.5 seconds for USB settling, then up to 20 seconds for the
firmware readiness handshake. Use `--ready-timeout SECONDS` to change the handshake
wait. A timeout can also indicate the wrong port or incompatible firmware; waiting
longer does not correct those causes. Keep the entry point named `Data_Logger.txt`:
recording provenance hashes that filename when allocating a capture.

Production defaults to 180 seconds per run. Use `--run-seconds SECONDS` to change
it. Invalid peak configuration is reported as a CLI argument error.

## Arduino builds

Compile target: **Arduino UNO R4 WiFi**, FQBN `arduino:renesas_uno:unor4wifi`,
Renesas UNO core **1.6.0**. This target matches the original source's Wire1/Qwiic
bus and the core's SDA1/SCL1 definitions (27/26). It is a verified compile target;
the physical rig model still needs confirmation before upload. The R4 Minima
is not an interchangeable target for this two-bus wiring.

Using Arduino CLI 1.2.2:

```sh
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno@1.6.0
python3 scripts/build_firmware.py MechLung
python3 scripts/build_firmware.py calibration
```

These are two independent compile commands. Each passes `--libraries
firmware/libraries` and supplies a digest of the firmware sources and build
procedure, plus core/board identity, through a temporary provenance header.
Outputs go into `build/MechLung` and `build/calibration`. The script refuses a
different installed core version. Pass `--cli /path/to/arduino-cli` if needed.
No build command uploads or moves hardware.

After confirming the actual board and configuration, upload **one** target:

```sh
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port /dev/ttyACM0 --input-dir build/MechLung
# Or upload the separately compiled calibration application:
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port /dev/ttyACM0 --input-dir build/calibration
```

### R4 WiFi readiness regression

The R4 WiFi routes `Serial` through a UART to its ESP32 USB bridge. In core
1.6.0 that UART inherits `Print.availableForWrite()`, which always returns zero.
Earlier transport code treated zero as a full transmit buffer and consequently
sent neither telemetry nor readiness replies. Rebuild and upload the fixed
firmware for **both** applications as needed; increasing `--ready-timeout` or
renaming the Python entry point cannot fix this firmware defect.

The transport now writes at most four UART bytes per loop (about 87 microseconds
of wire time at 460800 baud), allowing command, watchdog and acquisition
processing between slices. The separate SerialUSB path retains its
capacity checks. Native tests model both serial implementations.

**Baud-rate migration:** both sketches and the logger now default to **460800**.
The former 115200 setting carries at most 11,520 bytes/second (8N1), less than the
DATA3 stream at 100 rows/second. Rebuild/upload the new firmware and use the
matching logger; remove any old `--baud 115200` override. Set Arduino Serial
Monitor to 460800 too. For intentionally retained old firmware, the logger's
`--baud 115200` option remains available, but does not fix that firmware's
transmit bug or provide enough bandwidth for its 100 Hz DATA3 stream.

Native tests reproduce loss at 115200 and verify acknowledged, complete captures
at 460800 with simulated UART and sensor timing for both roles. Actual USB bridge
throughput and Windows operation still require a check on the board.

## Calibration configuration and procedure

`firmware/libraries/MechCommon/src/calibration_values.h` is the single source
for inherited centre, correction and feedback scaling, and configured endpoints.
**CAL_MIN_DAC and CAL_MAX_DAC are deliberately unset (`-1`)**. They mean applied
DAC commands for configured travel endpoints; they are not inferred from the
register limits, half-range or demo margin. Set explicit rig values within
0–4095, rebuild, and confirm the Jrk travel configuration before physical testing.
Unset endpoint commands are rejected by both logger and firmware. The synthetic
tests use temporary values 1200/1800; these are not rig recommendations.

The existing centre/reference scaling is provisional. Calibration never fits,
rewrites or persists configuration automatically. Suggested measurement workflow:

1. Upload calibration firmware. It boots into centre hold and continuously
   reports raw A1 ADC, scaled feedback, applied DAC and sensor validity/ages.
2. Start `.venv/bin/python Data_Logger.txt --port PORT --calibration`.
   This negotiates the calibration role and starts one acknowledged `CAL` session.
   There is **no default duration limit**; an explicit `--run-seconds` still works.
3. Record centre (`H`), second reference (`X`), and, once configured, maximum
   (`m`) and minimum (`n`), then centre (`H`). Allow settling and measure actual
   piston travel at each target. Raw A1 is shown in the actuator status and CSV.
4. Use `q` to close and keep the capture, or `p` to stop and discard it. STOP,
   normal close and the heartbeat watchdog return the firmware to centre.
5. Compare measured DAC/ADC pairs and travel; update explicit configuration
   after evaluating the measurements. Rebuild both applications as needed.

The raw sample is unfiltered; scaled feedback retains the inherited median-of-3
filter. Position changes preserve run ID, clock and sequence. An in-progress
old-target scan is closed before applying a new target. CAL has preset `-3`,
phase `-1`, zero breathing targets and no breathing peak population.

## Controls

Click a plot before pressing keys. Firmware role is negotiated before recording.

| Key | Production | Calibration |
| --- | --- | --- |
| `1`–`4` | Start preset and timed recording | Rejected |
| `s` / `S` | Demo sine recording | Rejected |
| `H` / `h` / `0` | Stop owned run and hold centre | Centre target; keep recording |
| `X` / `x` | Requires calibration firmware | Second reference target |
| `m` / `n` | Requires calibration firmware | Configured max / min target |
| `t` / `T` | Toggle monitor text | Toggle monitor text |
| `?` | Readiness/configuration handshake when idle | Same |
| `p` | Stop and discard run | Same |
| `q` | Close windows, stop and finalize | Same |

`m`/`M` are no longer monitor aliases. The logger refuses new monitor/endpoint
commands on older firmware. Legacy DATA and DATA2 parsers remain available;
protocol 2 owned production recordings are still supported. Endpoint commands
are only sent after a protocol-3 CAL acknowledgement.

## Protocol and ownership

Protocol 3 uses the DATA2 order followed by `pot_raw_adc` (12-bit integer) and
`calibration_position` (`none`, `centre`, `reference`, `max`, `min`): **46 fields
including DATA3**. Production labels are `none`. Unowned monitoring uses `-`
as run ID, normalized to `None`; owned rows must match the acknowledged ID and
preset. Older rows fill the added fields with `None`.

Metadata retains geometry, correction, sensor/reference and core information.
Version 3 replaces `calibration_target` with `firmware_role`, `cal_centre_dac`,
`cal_reference_dac`, `cal_min_dac`, and `cal_max_dac`. Unconfigured endpoints are
`unset`. STARTED/ENDED and manifest protocol versions are validated explicitly.

Each run gets a fresh Recording, AnalysisSession and bounded worker. SerialLink
owns transitions: IDLE → STARTING → RUNNING → STOPPING → FINALIZING → IDLE;
failed outcomes go to FAULTED. ENDED confirms the firmware stop; storage completion
separately determines recording success. CompletionResult exposes
`stop_confirmed`, `recording_complete`, and `failure_reason`. A new handshake
allows recovery after a fault once the worker exits. Old-run callbacks are ignored.

The worker drains ordered samples before finalization. A requested stop has
separate acknowledgement and storage-finalization deadlines (two seconds each by
default); `close(timeout=...)` caps the entire shutdown. The reader uses the
finalization deadline after ENDED rather than the shorter capture-latency limit. It is joined on close,
including faulted runs. A shutdown deadline with blocked storage raises
`IncompleteShutdownError`; the non-daemon worker remains alive until storage
recovers. Python cannot safely kill a thread blocked in filesystem I/O.

## Recording integrity and analysis

Recordings contain sample CSV, peak CSV and a session JSON manifest. The initial
manifest stays incomplete until final publication succeeds. CSV fields include
original wire lines, normalized times, raw A1 and target labels. The manifest
contains firmware identity/configuration, analysis policy and software hashes.

Loss diagnostics are **separate and may overlap**:

- `missing_rows`: leading and interior sequence gaps, using the owned zero origin.
- `missed_acquisition_slots`: scheduler counter, including its initial value.
- `firmware_tx_drops`: transmit counter and final ENDED count.
- `invalid_rows`: malformed input rows.

These diagnostics are not summed. Any known capture loss, failed storage,
unconfirmed end or missing tail makes `complete` false. Invalid sensor readings
remain present with validity flags and ages; they affect peak quality separately
from missing recording data. Sequence/counter wrap uses unsigned arithmetic.
Peak scoring for production retains the existing smoothing/segmentation policy.
A discontinuity clears the latched threshold before the detector can rearm.
Repeated invalid-flow/malformed-input notifications without a valid sample between
them share a diagnostic boundary; every discontinuity still resets detection.
The manifest retains at most 1,024 boundary details and reports additional ones
in `boundaries_omitted`. CSV rows retain their segment IDs and discontinuity reasons.
The detector caps an excursion at 60,000 samples (ten minutes at 100 Hz), closes
it as partial with `excursion_limit`, and waits for a new baseline. These bounds
are included in the additive `analysis_limits` manifest field. Phase summaries
use constant memory and account for cycle rollover.

Duplicate/backward row counters, inconsistent scan/sequence increments, regressing
loss counters, or regressing commanded phases fault the capture explicitly.
A manifest cannot be complete without STARTED and a consistent ENDED. Finite
pressure extremes cannot overflow the variance calculation; nonfinite reference
conversions and invalid numeric handshake metadata are rejected.

## Acceptance status and limitations

Automated checks cover repeated-run isolation, leading/slot loss, schema
compatibility, concurrent stop/fault handling, blocked storage and injected
analysis/flush/manifest failures. The native harness runs both actual sketches
against simulated hardware and exercises calibration capture through the logger,
continuous production hold, watchdog centre commands and command negotiation.
Arduino compilation checks target compatibility separately. The expanded harness
covers valid flow and all seven pressure channels, CRC errors, short reads, NACKs,
nonpositive pressure scales, recovery, clock rollover, delayed I2C, partial serial
writes and blocked output. It exercises all production presets, demo mode,
configured and unset calibration endpoints, duplicate/stale commands, and the
64-completed-session limit. The board must be reset after 64 completed owned
sessions; a new host handshake alone does not clear that firmware limit.

**Physical acceptance is pending:** confirm the board, configure measured
endpoints, measure travel and DAC/ADC scaling, and test watchdog return to centre
on the rig. Synthetic GPIO/I2C/USB tests cannot establish mechanical behavior,
real sensor timing, USB throughput or travel safety. No physical device was
attached during implementation.
