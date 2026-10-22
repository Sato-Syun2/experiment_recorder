"""Held-out floor/wall fits from 12--14s, independent of initial 0--2.5s fit."""
from pathlib import Path
import csv
import json
import sys
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from track_people import _iter_bag_clouds, _pointcloud2_to_numpy, _message_timestamp_ns, _rotation_from_up_vector
from calibrate_zed_lidar_planes import _fit_plane, _angle_between
from corridor_frame import estimate_corridor_frame
from prepare import REVIEW, BAG, save


def main():
    base = json.loads((OUT/'geometry.json').read_text())
    cam = base['camera']
    first = int(cam['first_image_timestamp_ns'])
    parts = {'lidar': [], 'zed': []}
    for msg, fb, _ in _iter_bag_clouds(BAG, '/livox/lidar', 'pointcloud2', 'mcap'):
        t = (_message_timestamp_ns(msg, fb)-first)/1e9
        if t > 14:
            break
        if t < 12:
            continue
        p = _pointcloud2_to_numpy(msg)
        keep = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1)<20) & (p[:, 0]>0)
        parts['lidar'].append(p[keep])
    for r in csv.DictReader((REVIEW/'frames.csv').open()):
        if not 12 <= float(r['elapsed_s']) <= 14:
            continue
        with np.load(REVIEW/r['depth_file']) as a:
            d = a['depth_m'][::6, ::6]
        v, u = np.mgrid[0:cam['height']:6, 0:cam['width']:6]
        p = np.stack(((u-cam['cx'])*d/cam['fx'], (v-cam['cy'])*d/cam['fy'], d), -1).reshape(-1, 3)
        keep = np.isfinite(p).all(axis=1) & (p[:,2]>.4) & (p[:,2]<18)
        parts['zed'].append(p[keep])
    results = {}
    for kind, chunks in parts.items():
        p = np.concatenate(chunks).astype(float)
        rng = np.random.default_rng(20261023)
        if len(p)>60000:
            p = p[rng.choice(len(p), 60000, replace=False)]
        b = base['geometries'][kind]
        floor = _fit_plane(p, np.array(b['floor']['normal']), b['floor']['d_m'], .4,
                    angle_tolerance_deg=6, point_tolerance_m=.025 if kind=='lidar' else .04,
                    iterations=1400, seed=20261023)
        R = np.array(b['rotation_raw_to_corridor'])
        t = np.array(b['translation_raw_to_corridor'])
        cp = p @ R.T + t
        corr = estimate_corridor_frame(cp, x_min=.5,x_max=16,y_min=-3,y_max=3,z_min=.3,z_max=2.3,
                voxel_size=.08,line_tolerance=.06,ransac_iterations=1800,seed=20261023)
        # Height of the held-out fitted plane along the frozen corridor z-axis.
        nf = R @ np.array(floor['normal'])
        df = floor['d_m'] - float(nf @ t)
        floor_z = [-float((nf[0]*x+df)/nf[2]) for x in [3., 15.]]
        results[kind] = dict(heldout_floor=floor, heldout_wall_fit=corr.to_dict(),
            floor_angle_difference_deg=_angle_between(np.array(b['floor']['normal']), np.array(floor['normal'])),
            floor_offset_difference_m=floor['d_m']-b['floor']['d_m'],
            heldout_floor_z_at_x3_and15_y0_m=floor_z,
            corridor_axis_difference_deg=float(np.degrees(np.arccos(np.clip(corr.longitudinal_xy[0],-1,1)))),
            wall_gap_difference_m=corr.wall_gap-b['corridor']['wall_gap'],
            heldout_centerline_y_at_x3_and15_m=[float(corr.origin_xy[1]+x*corr.longitudinal_xy[1]/corr.longitudinal_xy[0]) for x in [3.,15.]])
        print(kind, results[kind], flush=True)
    save('heldout_geometry.json', dict(interval_zed_s=[12.,14.],results=results,
        note='Separate frames, same installation; structural repeatability, not surveyed absolute accuracy. No trajectory-based tuning.'))
    l, z = [base['geometries'][k] for k in ['lidar','zed']]
    Rl, Rz = [np.array(v['rotation_raw_to_corridor']) for v in [l,z]]
    tl, tz = [np.array(v['translation_raw_to_corridor']) for v in [l,z]]
    # Floor and walls identify only lateral/vertical translation. Choose zero
    # longitudinal difference between sensor-projected origins for review only.
    R = Rz.T @ Rl
    t = Rz.T @ (tl-tz)
    save('lidar_to_zed_candidate.json',dict(version=1,source_frame='livox_frame',target_frame='zed_left_eye',
        coordinate_system='IMAGE',units='m',calibration_status='floor_wall_candidate_longitudinal_translation_unobserved',
        rotation_source_to_target=R.tolist(),translation_m=t.tolist(),
        unobservable_translation_direction_target=Rz.T[:,0].tolist(),
        unobservable_longitudinal_origin_difference_assumed_m=0.,
        notes='Current recording only. Sensor-projected longitudinal origins provisionally equated; absolute longitudinal translation uncalibrated. Not applied to active config.'))
    print('candidate transform saved; longitudinal translation remains unobserved', flush=True)


if __name__ == '__main__':
    main()
