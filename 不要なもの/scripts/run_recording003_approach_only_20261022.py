#!/usr/bin/env python3
"""Windows SDK PC: fresh approach-only tracking at about 15m and 13m."""
from pathlib import Path
from datetime import datetime
import argparse
import json
import os
import subprocess
import sys
import webbrowser


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('svo',type=Path,help="SDK PC's local recording_003.svo2")
    args=parser.parse_args()
    if str(args.svo).startswith('\\\\'):
        parser.error('Use a local SVO2 path on the SDK PC.')
    svo=args.svo.expanduser().resolve()
    if not svo.is_file():parser.error(f'SVO2 not found: {svo}')
    scripts=Path(__file__).resolve().parent
    parent=scripts.parent/'20261021_group002_recording_003_skeleton_replay'
    package=parent/'approach_check_20261022'
    exporter=scripts/'analyze_svo_body_tracking_approach_20261022.py'
    plan=json.loads((package/'approach_plan.json').read_text(encoding='utf-8'))
    baseline=parent/'exports'/plan['baseline_body_folder']
    for p in [exporter,baseline/'camera.json',package/'build_walk_skeleton_replay.py',package/'compare_approach_tracking.py']:
        if not p.is_file():parser.error(f'Missing prepared file: {p}')
    run=package/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run.mkdir(parents=True,exist_ok=False)
    with (run/'run_start.json').open('x') as stream:
        json.dump(dict(source_svo=str(svo),python=sys.executable,case_ids=[c['id'] for c in plan['cases']],sdk_process_per_case='new'),stream,indent=2)
    errors={}
    for case in plan['cases']:
        print(f'\n{case["label"]}: fresh SDK process; no outbound tracking history.',flush=True)
        body=run/case['id']/'sdk'
        try:
            subprocess.run([sys.executable,str(exporter),str(svo),'--output',str(body),
                            '--window-manifest',str(package/case['window_file']),
                            '--max-depth-m','30','--confidence','35','--preview-period-s','.5'],check=True)
            subprocess.run([sys.executable,str(package/'build_walk_skeleton_replay.py'),'--bundle-dir',str(package),
                            '--body-dir',str(body),'--quality-json',str(package/'reference_quality.json'),
                            '--output',str(run/f'replay_{case["id"]}.html')],check=True)
        except subprocess.CalledProcessError as exc:
            errors[case['id']]=f'Processing exited with code {exc.returncode}. See console output.'
            print(errors[case['id']],flush=True)
    with (run/'case_errors.json').open('x') as stream:json.dump(errors,stream,indent=2)
    subprocess.run([sys.executable,str(package/'compare_approach_tracking.py'),'--bundle-dir',str(package),
                    '--run-dir',str(run),'--baseline-dir',str(baseline)],check=True)
    with (run/'run_complete.json').open('x') as stream:
        json.dump(dict(status='completed_with_errors' if errors else 'completed',case_errors=errors),stream,indent=2)
    report=run/'index.html'
    print(f'\nSaved comparison and playback: {report}',flush=True)
    if os.name=='nt':os.startfile(str(report))
    else:webbrowser.open(report.as_uri())
    if errors:raise SystemExit(1)


if __name__=='__main__':
    main()
