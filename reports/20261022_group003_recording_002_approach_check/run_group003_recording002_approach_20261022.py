"""Windows SDK PC: verify SVO timestamps, then cold-start approach-only BODY_34."""
from pathlib import Path
from datetime import datetime
import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys


def save(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('svo', type=Path)
    args = parser.parse_args()
    svo = args.svo.expanduser().resolve()
    if str(svo).startswith('\\\\') or not svo.is_file():
        parser.error('Specify the existing SSD/local SVO2 file.')
    scripts = Path(__file__).resolve().parent
    package = scripts.parent/'20261022_group003_recording_002_approach_check'
    bundle = json.loads((package/'bundle.json').read_text(encoding='utf-8'))
    exporter = package/'sdk_tools'/'analyze_svo_body_tracking.py'
    builder = package/'sdk_tools'/'build_walk_skeleton_replay.py'
    expected_sparse = {int(r['frame_index']):int(r['image_timestamp_ns'])
        for r in csv.DictReader((package/'review_frame_timestamps.csv').open(encoding='utf-8-sig'))}
    # Frame 216 is the inspected post-turn image at 7.199s. Stop before frame
    # 630 (21.033s); no outbound or turning frame is sent to Body Tracking.
    start, end = 216, 630
    run = package/'sdk_runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run.mkdir(parents=True, exist_ok=False)
    save(run/'run_start.json', dict(source_svo=str(svo), python=sys.executable,
        start_frame=start,end_frame_exclusive=end,
        exporter_sha256=hashlib.sha256(exporter.read_bytes()).hexdigest(),
        geometry_note='Current floor/wall candidate. Longitudinal sensor translation and clock correction remain uncalibrated.'))
    try:
        import pyzed.sl as sl
        camera = sl.Camera()
        init = sl.InitParameters()
        init.set_from_svo_file(str(svo))
        init.svo_real_time_mode = False
        init.depth_mode = sl.DEPTH_MODE.NONE
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise RuntimeError(f'Could not open SVO: {status}')
        stamps = []
        try:
            info = camera.get_camera_information()
            if int(camera.get_svo_number_of_frames()) != int(bundle['camera']['total_svo_frames']):
                raise RuntimeError('Wrong recording: source frame count mismatch.')
            if int(info.serial_number) != int(bundle['camera']['serial_number']):
                raise RuntimeError('Wrong recording: camera serial mismatch.')
            # Decode for timestamps only; no depth or body/position tracker is
            # enabled in this camera instance. Never synthesize fps timestamps.
            for index in range(end):
                status = camera.grab(sl.RuntimeParameters())
                if status != sl.ERROR_CODE.SUCCESS:
                    raise RuntimeError(f'Timestamp scan stopped at {index}: {status}')
                stamp = int(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
                if index in expected_sparse and stamp != expected_sparse[index]:
                    raise RuntimeError(f'Export/SVO timestamp mismatch at source frame {index}.')
                if index >= start:
                    stamps.append(stamp)
        finally:
            camera.close()
        if len(stamps) != end-start or any(a>=b for a,b in zip(stamps,stamps[1:])):
            raise RuntimeError('Invalid window timestamp sequence.')
        window = dict(kind='approach_only_fresh_sdk',recording='20261022_group003/recording_002',
            start_frame=start,end_frame_exclusive=end,
            source_total_frames=bundle['camera']['total_svo_frames'],
            source_origin_ns=int(bundle['origin_ns']),serial_number=bundle['camera']['serial_number'],
            expected_image_timestamps_ns=stamps,
            selection_note='Image-reviewed approach after turn; nominal 15m declaration, not a surveyed position gate.')
        save(run/'window.json',window)
        subprocess.run([sys.executable,str(exporter),str(svo),'--output',str(run/'sdk'),
            '--window-manifest',str(run/'window.json'),'--max-depth-m','30',
            '--confidence','35','--preview-period-s','0.2'],check=True)
        subprocess.run([sys.executable,str(builder),'--bundle-dir',str(package),
            '--body-dir',str(run/'sdk'),'--output',str(run/'replay.html')],check=True)
        # Preserve every original SDK camera coordinate and also export the
        # independent current ZED -> corridor transformation explicitly.
        import numpy as np
        geom = json.loads((package/'geometry.json').read_text(encoding='utf-8'))['geometries']['zed']
        R = np.array(geom['rotation_raw_to_corridor']); t = np.array(geom['translation_raw_to_corridor'])
        fields = ['frame_index','image_timestamp_ns','elapsed_s','body_id','tracking_state','confidence',
                  'pelvis_corridor_x_m','pelvis_corridor_y_m','pelvis_corridor_z_m','keypoints_3d_corridor_json']
        with (run/'sdk'/'bodies.csv').open(encoding='utf-8-sig',newline='') as src, (run/'bodies_corridor.csv').open('x',encoding='utf-8-sig',newline='') as dst:
            writer = csv.DictWriter(dst,fieldnames=fields);writer.writeheader()
            for row in csv.DictReader(src):
                joints = np.array(json.loads(row['keypoints_3d_camera_json']),dtype=float)
                c = joints @ R.T + t
                clean = [[float(v) if np.isfinite(v) else None for v in p] for p in c]
                result = {key:row[key] for key in fields[:6]}
                result.update({f'pelvis_corridor_{a}_m':clean[0][i] for i,a in enumerate('xyz')})
                result['keypoints_3d_corridor_json'] = json.dumps(clean,allow_nan=False)
                writer.writerow(result)
        save(run/'run_complete.json',dict(status='sdk_export_and_replay_completed',
            expected_frames=end-start,
            note='Output complete is not an accuracy verdict. Inspect RGB alignment, SDK state, joints, floor height and ID continuity.'))
        print(f'\nSaved SDK export, corridor joints and replay: {run}',flush=True)
        if os.name=='nt':
            os.startfile(str(run/'replay.html'))
    except Exception as exc:
        save(run/'run_failed.json',dict(error=str(exc),exception=type(exc).__name__))
        raise


if __name__=='__main__':
    main()
