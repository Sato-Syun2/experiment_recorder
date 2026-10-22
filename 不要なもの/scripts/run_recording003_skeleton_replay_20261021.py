#!/usr/bin/env python3
"""Run on the Windows ZED SDK PC: local SVO2 -> shared SDK export -> 3D HTML."""
from pathlib import Path
from datetime import datetime
import argparse
import os
import subprocess
import sys
import webbrowser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('svo', type=Path, help="SDK PC's local recording_003.svo2")
    args = parser.parse_args()
    if str(args.svo).startswith('\\\\'):
        parser.error('Use the SDK PC local SVO2 path, not a UNC share.')
    svo = args.svo.expanduser().resolve()
    if not svo.is_file():
        parser.error(f'SVO2 not found: {svo}')
    scripts = Path(__file__).resolve().parent
    bundle = scripts.parent / '20261021_group002_recording_003_skeleton_replay'
    exporter = scripts / 'analyze_svo_body_tracking_recording003_20261021.py'
    builder = bundle / 'build_walk_skeleton_replay.py'
    for p in [exporter, builder, bundle/'bundle.json', bundle/'skeleton_replay_template.html']:
        if not p.is_file():
            parser.error(f'Missing prepared file: {p}')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    body_out = bundle / 'exports' / f'body_{stamp}'
    html_out = bundle / 'exports' / f'walk_skeleton_{stamp}.html'
    print(f'Python: {sys.executable}\nReading local SVO2: {svo}\nNew output: {body_out}', flush=True)
    subprocess.run([sys.executable, str(exporter), str(svo), '--output', str(body_out),
                    '--max-depth-m', '30', '--confidence', '35', '--preview-period-s', '.5'], check=True)
    subprocess.run([sys.executable, str(builder), '--bundle-dir', str(bundle),
                    '--body-dir', str(body_out), '--output', str(html_out)], check=True)
    print(f'Completed playback: {html_out}', flush=True)
    if os.name == 'nt':
        os.startfile(str(html_out))
    else:
        webbrowser.open(html_out.as_uri())


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f'Stopped: processing failed (exit {exc.returncode}). Read the error above. Existing output was preserved.', file=sys.stderr)
        sys.exit(exc.returncode)
