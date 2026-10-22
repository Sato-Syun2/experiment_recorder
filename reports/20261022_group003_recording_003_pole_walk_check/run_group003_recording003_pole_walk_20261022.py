"""Windows SDK PC: recording003 review, full walk, and fresh approach exports."""
from pathlib import Path
from datetime import datetime
import argparse,csv,hashlib,json,subprocess,sys


def save(path,data):
    with path.open('x',encoding='utf-8') as f:
        json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('svo',type=Path)
    args=parser.parse_args()
    svo=args.svo.expanduser().resolve()
    if str(svo).startswith('\\\\') or not svo.is_file():
        parser.error('Specify the existing local/SSD recording_003.svo2 file.')
    package=Path(__file__).resolve().parent.parent/'20261022_group003_recording_003_pole_walk_check'
    plan=json.loads((package/'analysis_plan.json').read_text(encoding='utf-8'))
    scripts=package/'sdk_tools'
    exporter=scripts/'export_svo_review.py';body_exporter=scripts/'analyze_svo_body_tracking.py'
    for p in [exporter,body_exporter]:
        if not p.is_file():raise FileNotFoundError(p)
    run=package/'sdk_runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run.mkdir(parents=True,exist_ok=False)
    stage='review'
    save(run/'run_start.json',dict(source_svo=str(svo),python=sys.executable,
        purpose='Pole/walker correspondence; full walk plus independently initialized approach',
        sources_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [exporter,body_exporter]},
        plan=plan))
    try:
        print('1/3: review RGB, metric depth and IMU for recording003',flush=True)
        subprocess.run([sys.executable,str(exporter),str(svo),'--output',str(run/'review'),
            '--every-seconds','0.2','--include-depth','--include-imu'],check=True)
        cam=json.loads((run/'review/camera.json').read_text(encoding='utf-8'))
        stage='full_sdk'
        print('2/3: full-walk BODY_34 export; full tracking history retained in this case',flush=True)
        subprocess.run([sys.executable,str(body_exporter),str(svo),'--output',str(run/'body_full'),
            '--max-depth-m','30','--confidence','35','--preview-period-s','0.2'],check=True)
        with (run/'body_full/frames.csv').open(encoding='utf-8-sig',newline='') as f:
            frames=list(csv.DictReader(f))
        with (run/'review/frames.csv').open(encoding='utf-8-sig',newline='') as f:
            sampled=list(csv.DictReader(f))
        stamp_by_index={int(r['frame_index']):int(r['image_timestamp_ns']) for r in frames}
        if len(stamp_by_index)!=cam['total_svo_frames']:
            raise RuntimeError('Full SDK frame count does not match review export.')
        if int(frames[0]['image_timestamp_ns'])!=int(cam['first_image_timestamp_ns']):
            raise RuntimeError('SDK and review do not share the same source origin.')
        if any(stamp_by_index.get(int(r['frame_index']))!=int(r['image_timestamp_ns']) for r in sampled):
            raise RuntimeError('Source timestamps differ between review and SDK export.')
        # The window is proposed from moving high-band LiDAR points, with raw
        # absolute clocks. It must later be checked against the new RGB images.
        chosen=[r for r in frames if int(plan['approach_start_header_ns'])<=int(r['image_timestamp_ns'])<=int(plan['approach_end_header_ns'])]
        if not chosen:raise RuntimeError('Proposed approach window does not overlap the SVO.')
        start=int(chosen[0]['frame_index']);end=int(chosen[-1]['frame_index'])+1
        if [int(r['frame_index']) for r in chosen]!=list(range(start,end)):
            raise RuntimeError('Approach source frame indices are not contiguous.')
        window=dict(kind='approach_only_fresh_sdk',recording='20261022_group003/recording_003',
            start_frame=start,end_frame_exclusive=end,source_total_frames=cam['total_svo_frames'],
            source_origin_ns=cam['first_image_timestamp_ns'],serial_number=cam['serial_number'],
            expected_image_timestamps_ns=[int(r['image_timestamp_ns']) for r in chosen],
            selection_note=plan['window_selection_note'])
        save(run/'window_approach.json',window)
        stage='fresh_approach_sdk'
        print('3/3: fresh SDK process; seek before positional/body tracking initialization',flush=True)
        subprocess.run([sys.executable,str(body_exporter),str(svo),'--output',str(run/'body_approach'),
            '--window-manifest',str(run/'window_approach.json'),'--max-depth-m','30',
            '--confidence','35','--preview-period-s','0.2'],check=True)
        save(run/'run_complete.json',dict(status='review_full_and_fresh_approach_exports_completed',
            full_frames=len(frames),approach_frames=len(chosen),
            note='Export completion only. Human/pole association, camera installation, joints and synchronization require subsequent review.'))
        print(f'\nSaved all outputs: {run}',flush=True)
        print('The review/full/fresh approach folders belong together. No accuracy verdict has been made.',flush=True)
    except Exception as exc:
        save(run/'run_failed.json',dict(stage=stage,error=str(exc),exception=type(exc).__name__))
        raise


if __name__=='__main__':main()
