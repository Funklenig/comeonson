#!/usr/bin/env python3
"""Compile one sketch with pinned core and source provenance; never uploads."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CORE = 'arduino:renesas_uno'
VERSION = '1.6.0'
FQBN = CORE + ':unor4wifi'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sketch', choices=('MechLung',))
    parser.add_argument('--cli', default='arduino-cli')
    args = parser.parse_args()
    platforms = json.loads(subprocess.check_output([args.cli, 'core', 'list', '--format', 'json']))
    if not any(p['id'] == CORE and p['installed_version'] == VERSION for p in platforms['platforms']):
        parser.error(f'install {CORE}@{VERSION}; see README.md')
    digest = hashlib.sha256()
    for path in sorted((ROOT/'firmware').rglob('*')):
        if path.is_file():
            digest.update(str(path.relative_to(ROOT)).encode()+b'\0'+path.read_bytes()+b'\0')
    digest.update(Path(__file__).read_bytes())
    identity = digest.hexdigest()
    with tempfile.TemporaryDirectory(prefix='mech-provenance-') as directory:
        header = Path(directory)/'provenance.h'
        header.write_text(f'#define MECH_BUILD_ID "{identity}"\n#define MECH_CORE_ID "renesas_uno_{VERSION}_unor4wifi"\n')
        subprocess.run([args.cli, 'compile', '--fqbn', FQBN, '--libraries', str(ROOT/'firmware/libraries'),
                        '--build-path', str(ROOT/'build'/args.sketch),
                        '--build-property', f'compiler.cpp.extra_flags=-include "{header}"',
                        str(ROOT/'firmware'/args.sketch)], check=True)
    print(f'{args.sketch}: source={identity} core={CORE}@{VERSION} board={FQBN}')


if __name__ == '__main__':
    main()
