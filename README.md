# Mechanical lung logger

Use the matching Python logger and **Arduino UNO R4 WiFi** production firmware.
Select a flow preset (10, 15, 20 or 30 L/min), monitor the three live plots, and
save the timed experiment. Runs default to 180 seconds; `--run-seconds` changes
that duration. Presets can be repeated without restarting the logger or board.
Calibration and sine-demo execution paths and old recording formats are removed.
Physical geometry, sensor scaling and actuator feedback constants remain in
`firmware/libraries/MechCommon/src/calibration_values.h`.

## Python setup

Python 3.9 or newer:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python Data_Logger.txt --port /dev/ttyACM0
```

Use `--list-ports` to find the board. The logger and firmware default to **460800
baud**. The DATA3 stream at 100 Hz exceeds the capacity of 115200 baud; remove
old baud overrides. Only one program can own the serial port.

Opening the port is followed by 2.5 seconds of USB settling and a validated
production-firmware handshake, with a 20-second timeout. `--ready-timeout` adjusts
that wait. Calibration and legacy firmware do not satisfy readiness.

### Controls

Click a plot window first.

| Key | Action |
| --- | --- |
| `1`–`4` | Start the corresponding preset and recording |
| `H`, `h`, `0` | Stop and keep measurements; hold centre |
| `p` | Stop and discard this run's files |
| `t`, `T` | Toggle board monitor text |
| `?` | Retry readiness negotiation when idle or after a recovered storage fault |
| `q` | Stop, finalize measurements and close the plots |

Flow, actuator command/feedback and seven pressures are plotted independently.
Invalid sensor readings appear as gaps; valid channels continue to plot. The
status distinguishes recording state, saving, stop confirmation, data quality
and analysis. File-write or connection failures end the run and attempt an
explicit owned STOP. If its acknowledgement is unavailable, stop remains
unconfirmed. The firmware heartbeat watchdog remains a fallback.

## Saved measurements and analysis

Each run reserves only:

- `preset_N_TIMESTAMP.csv`: received measurements, original wire lines,
  timestamps, validity/age information and explicit `missing_before` sequence gaps.
- `preset_N_TIMESTAMP_session.json`: firmware/software identity and separate
  run, storage, stop and quality outcomes.

Measurements are saved by a bounded FIFO worker. Invalid sensors, malformed rows,
corrupt measurement counters and missing samples affect quality and diagnostics;
they do not terminate a connected recording with healthy storage. Corrupt rows
are rejected without replacing the last trustworthy timestamp. Received valid
rows remain available. No placeholder measurements are synthesized for gaps.

The summary's `complete` describes successful capture finalization;
`storage_complete` describes successful saving; `quality_complete` describes
measurement validity and loss. `planned_duration_reached`, `stop_confirmed`,
`termination_reason` and `failure_reason` describe the run separately. A run with
gaps can complete its planned duration and start the next preset normally.

`missing_rows`, `missed_acquisition_slots`, `firmware_tx_drops` and `invalid_rows`
are independent diagnostics that may overlap. **Do not sum them.** Invalid-flow
and per-pressure-channel counts identify sensor faults separately. The boundary
list is capped at 1,024 details; the summary reports omitted details, while CSV
rows retain their segment and gap information.

After capture finalizes, a separate process reads the saved CSV and produces:

- `preset_N_TIMESTAMP_peaks.csv`: peak results using the established rules.
- `preset_N_TIMESTAMP_analysis.json`: analysis outcome, policy, source digest,
  peak populations, commanded-phase summaries and orientation diagnostics.

Analysis never executes on the capture worker, allocates files before START, or
changes the measurement CSV/run summary. Invalid peak configuration and analysis
output failures are reported independently. Automatic analysis uses one process
and a bounded queue of four pending runs. If work is deferred or interrupted
while closing the logger, the measurements remain usable; retry the saved run.
A missing analysis result file means analysis has not been published successfully.

Retry without a board or live GUI:

```sh
.venv/bin/python analysis_runner.py recordings/preset_1_TIMESTAMP_session.json
```

To override analysis settings on retry, use `--options settings.json`. For example:

```json
{"config": {"smooth_window": 1, "min_samples": 3}, "avg_window": 1,
 "gas_temperature_c": 20, "absolute_pressure_mbar": 1013}
```

A `config` override replaces the saved detector configuration; omitted settings
use detector defaults. Otherwise the saved options are used. The existing peak
rules are preserved; this change does not establish their scientific suitability.
`--peak-*`, `--warmup-breaths` and flow-reference settings on the logger are passed
to analysis after capture. Detector limit validation happens then.

## Firmware build

Use **Arduino CLI 1.2.2**, Renesas UNO core **1.6.0**, FQBN
`arduino:renesas_uno:unor4wifi`:

```sh
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno@1.6.0
python3 scripts/build_firmware.py MechLung
```

The build script checks the core version, uses `firmware/libraries`, adds source
provenance and writes `build/MechLung`. It does not upload. To upload the matching
production build to the connected board:

```sh
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port /dev/ttyACM0 --input-dir build/MechLung
```

The R4 WiFi UART returns zero from `availableForWrite()` in this core. Transport
writes at most four bytes per service call so acquisition, command handling and
the watchdog continue. ENDED formatting uses decimal sequence text because this
target's `newlib-nano` does not support `%lld`. The firmware remembers 64 recent
completed IDs to reject retransmitted START commands, using a rolling cache;
there is no 64-run reset requirement. IDs older than this cache are not retained.

## Software checks

The unittest suite needs `g++` with C++17 for the synthetic firmware harness:

```sh
.venv/bin/python -m unittest discover -s tests -v
xvfb-run -a .venv/bin/python scripts/smoke_gui.py
.venv/bin/python scripts/soak_capture.py
```

Linux GUI testing needs Tk, Xvfb and xauth. On Amazon Linux:

```sh
sudo dnf install python3-tkinter xorg-x11-server-Xvfb xorg-x11-xauth
```

On a desktop display, omit `xvfb-run -a`. The smoke exercises all four preset keys,
stopping, discarding, plot redraws and shutdown with synthetic serial input.
The soak saves and independently counts 180,000 rows twice (30 simulated minutes
per case), checks bounded memory/display history and analyzes saved files. One
case includes persistent sensor faults. Temporary captures are deleted afterward.

Target-library regression (independent of the native host C library):

```sh
python3 -m venv /tmp/logger-formatter-tools
/tmp/logger-formatter-tools/bin/python -m pip install unicorn==2.1.4 pyelftools==0.32
/tmp/logger-formatter-tools/bin/python scripts/check_stop_formatter.py --gcc /path/to/arm-none-eabi-gcc
```

Use the ARM GCC installed with the pinned Arduino core. The probe executes actual
linked Cortex-M4 formatter code under Unicorn and checks `-1`, `0`, `42` and the
maximum uint32 sequence. It also checks an independent control. It does not
emulate board peripherals or establish physical stopping.

## Windows 10 rig procedure

With Python 3.9+ and Arduino CLI 1.2.2 installed, run in PowerShell from the repo:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno@1.6.0
.\.venv\Scripts\python.exe scripts\build_firmware.py MechLung
arduino-cli board list
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port COM5 --input-dir build/MechLung
.\.venv\Scripts\python.exe Data_Logger.txt --list-ports
.\.venv\Scripts\python.exe Data_Logger.txt --port COM5 --run-seconds 180
```

Replace `COM5` with the board's actual port; close Arduino Serial Monitor first.
To run the native unittest harness on Windows, install a C++17 `g++` toolchain
and put it on PATH. GUI smoke uses Python directly on a Windows desktop.

For physical acceptance:

1. Run every preset for 180 seconds, then repeat presets in the same logger.
   Verify the physical stop/centre behavior and timestamp/command scaling.
2. Independently count CSV rows and compare sequences, scan IDs and timestamps
   with the board's 10 ms acquisition schedule. Investigate every unexplained gap.
   Assess actual USB bridge throughput and all seven sensor validity/age fields.
3. Exercise invalid sensors and recover them: unaffected channels continue,
   measurements stay available, and quality flags identify the fault/gap.
4. Exercise connection loss and storage failure: verify the physical stop outcome
   and retained partial data against the reported stop/storage status.
5. Block a derived peak output or use invalid detector settings; verify the CSV
   survives and retry analysis successfully on the same saved run.

**Windows and physical acceptance are pending.** The sandbox has no attached rig
or Windows environment. Compilation and synthetic tests establish software
behavior; real sensor timing, motion, USB delivery and scaling still need the rig.
