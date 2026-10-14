#!/usr/bin/env python3
"""Run the actual firmware STOP snprintf call against its target newlib-nano.
Requires Python packages unicorn==2.1.4 and pyelftools==0.32 plus the pinned Arduino GCC.
Usage: python probe_stop_formatter.py --repo REPO --gcc /path/to/arm-none-eabi-gcc
Does not emulate the board peripherals or upload firmware. Exit 1 means formatter mismatch.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
parser.add_argument("--gcc", type=Path, required=True)
args = parser.parse_args()
transport = (args.repo / "firmware/libraries/MechCommon/src/transport.cpp").read_text()
call = re.search(
    r"(?:snprintf|encodeEnded)\(ended_marker, sizeof ended_marker,.*?;", transport, re.S
).group()
prelude = """#include <stdio.h>
#include <stdint.h>
#include <string.h>
volatile int64_t final_enqueued_seq;
unsigned long tx_dropped_total = 7;
const char *run_id = "0123456789abcdef";
const char *reason = "host_stop";
char ended_marker[160];
void probe(void) {
"""
original = prelude + call + "\n}\n"
if call.startswith("encodeEnded"):
    original = (
        '#include "'
        + str((args.repo / "firmware/libraries/MechCommon/src/stop_marker.h").resolve())
        + '"\n'
        + original
    )
# Positive control changes only how sequence text reaches printf.
control = (
    prelude
    + """char sequence[24];
if (final_enqueued_seq == -1) strcpy(sequence,"-1");
else snprintf(sequence,sizeof sequence,"%lu",(unsigned long)final_enqueued_seq);
snprintf(ended_marker,sizeof ended_marker,"# ENDED,3,%s,%s,%s,%lu\\n",run_id,sequence,reason,tx_dropped_total);
}"""
)

results = []
with tempfile.TemporaryDirectory(prefix="logger-formatter-") as directory:
    directory = Path(directory)
    for mode, source in [("actual_firmware_call", original), ("sequence_as_text_control", control)]:
        c = directory / (mode + ".c")
        elf = directory / (mode + ".elf")
        c.write_text(source)
        subprocess.run(
            [
                str(args.gcc),
                "-mcpu=cortex-m4",
                "-mthumb",
                "-mfpu=fpv4-sp-d16",
                "-mfloat-abi=hard",
                "-O0",
                "-fno-builtin",
                "-nostartfiles",
                "--specs=nano.specs",
                "--specs=nosys.specs",
                "-Wl,-Ttext=0x10000",
                "-Wl,-Tdata=0x20000000",
                "-Wl,-e,probe",
                str(c),
                "-o",
                str(elf),
            ],
            check=True,
        )
        with elf.open("rb") as stream:
            parsed = ELFFile(stream)
            segments = [
                (s["p_vaddr"], s.data()) for s in parsed.iter_segments() if s["p_type"] == "PT_LOAD"
            ]
            symbols = {
                s.name: s["st_value"] for s in parsed.get_section_by_name(".symtab").iter_symbols()
            }
        for seq in [-1, 0, 42, 0xFFFFFFFF]:
            uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
            uc.mem_map(0, 0x100000)
            uc.mem_map(0x20000000, 0x100000)
            for address, data in segments:
                uc.mem_write(address, data)
            uc.mem_write(symbols["final_enqueued_seq"], struct.pack("<q", seq))
            uc.reg_write(UC_ARM_REG_SP, 0x200F0000)
            sentinel = 0xF0000
            uc.reg_write(UC_ARM_REG_LR, sentinel | 1)
            done = []

            def returned(engine, address, size, user_data):
                if address == sentinel:
                    done.append(True)
                    engine.emu_stop()

            uc.hook_add(UC_HOOK_CODE, returned)
            error = None
            try:
                uc.emu_start(symbols["probe"] | 1, 0, count=100000)
            except UcError as exc:
                error = str(exc)
            actual = (
                bytes(uc.mem_read(symbols["ended_marker"], 160))
                .split(b"\0")[0]
                .decode("ascii", errors="backslashreplace")
            )
            expected = f"# ENDED,3,0123456789abcdef,{seq},host_stop,7\n"
            passed = bool(done) and error is None and actual == expected
            results.append(
                dict(
                    mode=mode,
                    sequence=seq,
                    expected=expected,
                    actual=actual,
                    returned=bool(done),
                    emulator_error=error,
                    passed=passed,
                )
            )
compiler = subprocess.check_output([str(args.gcc), "--version"], text=True).splitlines()[0]
lib = subprocess.check_output(
    [
        str(args.gcc),
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-print-file-name=libc_nano.a",
    ],
    text=True,
).strip()
report = {
    "compiler": compiler,
    "libc_nano_path": lib,
    "libc_nano_sha256": hashlib.sha256(Path(lib).read_bytes()).hexdigest(),
    "transport_sha256": hashlib.sha256(transport.encode()).hexdigest(),
    "extracted_call": call,
    "cases": results,
}
print(json.dumps(report, indent=2))
if not all(r["passed"] for r in results if r["mode"] == "sequence_as_text_control"):
    raise SystemExit("Probe positive control failed; do not trust original-call verdict.")
raise SystemExit(0 if all(r["passed"] for r in results) else 1)
