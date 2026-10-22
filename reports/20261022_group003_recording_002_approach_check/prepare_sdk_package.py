"""Prepare recording-specific SDK inputs; no SVO copies or SDK execution."""
from pathlib import Path
import csv
import json
import shutil

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def main():
    tools = OUT/'sdk_tools';tools.mkdir(exist_ok=False)
    for name in ['analyze_svo_body_tracking.py','build_walk_skeleton_replay.py']:
        source = (ROOT/'scripts'/name).read_text()
        source = source.replace('recording 003','recording 002 (20261022_group003)').replace('recording-003','recording-002')
        (tools/name).write_text(source)
    template = (ROOT/'scripts/walk_skeleton_replay_template.html').read_text()
    template = template.replace('録画003','録画002').replace('2026-10-21 / group002 / recording003','2026-10-22 / group003 / recording002')
    template = template.replace('<main><h1>', '<main><p style="background:#fff2cc;padding:12px">今回の床・壁からの校正候補。廊下方向の原点・センサー間の並進と時計差は未確定です。ZED深度の別区間では15 m先の床高に約10 cm相当の変動がありました。SDK OKや骨格表示だけで測位精度を判定しないでください。</p><h1>')
    (OUT/'skeleton_replay_template.html').write_text(template)
    geometry = json.loads((OUT/'geometry.json').read_text())
    cam = geometry['camera']
    observations = list(csv.DictReader((OUT/'tracks.csv').open()))
    ledger = list(csv.DictReader((OUT/'tracks.frames.csv').open()))
    lidar=[]
    for r in ledger:
        items=[]
        for p in observations:
            if p['frame']!=r['frame'] or int(p['track_id']) not in {9,53}:continue
            items.append(dict(id=int(p['track_id']),status=p['status'],detected=p['detected']=='1',
                x=float(p['corridor_x']),y=float(p['corridor_y']),z=float(p['corridor_z']),points=int(p['point_count'])))
        lidar.append(dict(t=float(r['time_sec']),tracks=items))
    review = Path('/data/experiment_data/20261022_group003_recording_002_calibration_review')
    rows = list(csv.DictReader((review/'frames.csv').open()))
    shutil.copyfile(review/'frames.csv',OUT/'review_frame_timestamps.csv')
    assets=OUT/'assets';assets.mkdir(exist_ok=False)
    images=[]
    for i,r in enumerate(rows):
        if not 7.15<=float(r['elapsed_s'])<=21.05:continue
        name=f'{i:04d}.jpg'
        shutil.copyfile(review/r['image_file'],assets/name)
        images.append(dict(t=float(r['elapsed_s']),file=name))
    bundle=dict(origin_ns=str(cam['first_image_timestamp_ns']),last_image_ns=str(cam['last_image_timestamp_ns']),
        camera=cam,duration=21.033,playback_start_s=7.199,initial_time_s=7.8,selected_ids=[9,53],
        corridor=geometry['geometries']['lidar']['corridor'],
        extrinsic=json.loads((OUT/'lidar_to_zed_candidate.json').read_text()),
        level_rotation=geometry['geometries']['lidar']['rotation_raw_to_level'],
        level_translation=geometry['geometries']['lidar']['translation_raw_to_level'],
        asset_directory='assets',lidar=lidar,images=images)
    with (OUT/'bundle.json').open('x') as stream:
        json.dump(bundle,stream,ensure_ascii=False,separators=(',',':'),allow_nan=False)
    print('Prepared SDK bundle and raw review images; SDK has NOT run.')


if __name__=='__main__':main()
